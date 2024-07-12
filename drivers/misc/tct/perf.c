/*
 * Copyright (C) 2020 Tcl Corporation Limited
 * Author: wu-yan@tcl.com
 * Date: 2020/08/26
 * This module provide interface (/dev/tct_perf) to
 * pass performance hints from userspace.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/cred.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/sched/signal.h>
#include <linux/tct/uiturbo.h>
#include <trace/events/sched.h>
#include <trace/events/block.h>
#include <linux/sysctl.h>

#define ANDROID_SYSTEM_UID KUIDT_INIT(1000)

struct uiturbo_log_entry {
	uid_t uid;
	pid_t pid;
	pid_t tid;
	pid_t target;
	int turbo;
};

struct uiturbo_log {
	atomic_t cur;
	bool full;
	struct uiturbo_log_entry entry[32];
};

static struct uiturbo_log uiturbo_log;

static void uiturbo_log_add(pid_t tid, int turbo)
{
	struct uiturbo_log *log = &uiturbo_log;
	struct uiturbo_log_entry *e;
	unsigned int cur = atomic_inc_return(&log->cur);

	if (cur >= ARRAY_SIZE(log->entry))
		log->full = true;
	e = &log->entry[cur % ARRAY_SIZE(log->entry)];
	e->uid = from_kuid(current_user_ns(), current_uid());
	e->pid = current->tgid;
	e->tid = current->pid;
	e->target = tid;
	e->turbo = turbo;
}

static int uiturbo_log_show(struct seq_file *m, void *v)
{
	struct uiturbo_log *log = &uiturbo_log;
	struct uiturbo_log_entry *e;
	int end = atomic_read(&log->cur) + 1;
	int start = log->full ? end - ARRAY_SIZE(log->entry) : 0;
	seq_printf(m, "last %d set uiturbo log\n", end - start);
	seq_printf(m, "  uid   pid   tid target turbo\n");
	while (start < end) {
		e = &log->entry[start % ARRAY_SIZE(log->entry)];
		++start;
		seq_printf(m, "%5d %5d %5d %6d %5d\n", e->uid,
			   e->pid, e->tid, e->target, e->turbo);
	}
	return 0;
}

static int uiturbo_log_open(struct inode *inode, struct file *file)
{
	return single_open(file, uiturbo_log_show, PDE_DATA(inode));
}

static const struct file_operations uiturbo_log_fops = {
	.open = uiturbo_log_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static atomic_t nr_ui_threads = ATOMIC_INIT(0);

static bool check_permissions(struct task_struct *p)
{
	const struct cred *cred = current_cred();

	if (uid_eq(cred->euid, GLOBAL_ROOT_UID)
	    || uid_eq(cred->euid, ANDROID_SYSTEM_UID)
	    || capable(CAP_SYS_NICE))
		return true;
	return same_thread_group(current, p);
}

static long tct_perf_set_uiturbo(unsigned long arg)
{
	struct task_struct *p;
	struct uiturbo_data ud;
	void __user *uarg = (void __user *)arg;
	long ret = 0;

	if (!uarg)
		return -EINVAL;
	if (copy_from_user(&ud, uarg, sizeof(ud))) {
		pr_warn("failed to copy from user\n");
		return -EFAULT;
	}

	if (!ud.tid) {
		pr_warn("bad parameter\n");
		return -EINVAL;
	}
	p = find_get_task_by_vpid(ud.tid);
	if (unlikely(!p))
		return -EINVAL;
	if (!check_permissions(p)) {
		put_task_struct(p);
		return -EACCES;
	}
	if (ud.turbo != TURBO_UI &&	/* case 1: ui -> none ui */
	    READ_ONCE(p->static_uiturbo) == TURBO_UI) {
		WRITE_ONCE(p->static_uiturbo, ud.turbo);
		atomic_dec(&nr_ui_threads);
	} else if (ud.turbo == TURBO_UI &&	/* case 2: none ui -> ui */
		   READ_ONCE(p->static_uiturbo) != TURBO_UI) {
		if (atomic_inc_return(&nr_ui_threads) < uiturbo_max_threads) {
			WRITE_ONCE(p->static_uiturbo, ud.turbo);
		} else {
			atomic_dec(&nr_ui_threads);
			pr_warn("too many ui threads\n");
			ret = -EPERM;
		}
	} else {
		WRITE_ONCE(p->static_uiturbo, ud.turbo);
	}
	uiturbo_log_add(ud.tid, ud.turbo);
	put_task_struct(p);
	return ret;
}

static long tct_perf_get_uiturbo(unsigned long arg)
{
	struct task_struct *p;
	struct uiturbo_data ud;
	void __user *uarg = (void __user *)arg;
	if (!uarg)
		return -EINVAL;
	if (copy_from_user(&ud, uarg, sizeof(ud))) {
		pr_warn("failed to copy from user\n");
		return -EFAULT;
	}

	if (!ud.tid) {
		pr_warn("bad parameter\n");
		return -EINVAL;
	}
	p = find_get_task_by_vpid(ud.tid);
	if (unlikely(!p))
		return -EINVAL;
	if (!check_permissions(p)) {
		put_task_struct(p);
		return -EACCES;
	}
	ud.turbo = p->static_uiturbo;
	put_task_struct(p);
	if (copy_to_user(uarg, &ud, sizeof(ud))) {
		pr_warn("failed to copy to user\n");
		return -EFAULT;
	}

	return 0;
}

static long tct_perf_ioctl(struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	long ret = 0;
	if (_IOC_TYPE(cmd) != TCT_PERF_MAGIC) {
		pr_warn("invalid magic number, type = %d\n", _IOC_TYPE(cmd));
		return -EINVAL;
	}

	if (_IOC_NR(cmd) > TCT_PERF_MAX_NR) {
		pr_warn("invalid tct_perf cmd, cmd = %d\n", _IOC_NR(cmd));
	}

	switch (cmd) {
	case TCT_PERF_SET_UITURBO:
		ret = tct_perf_set_uiturbo(arg);
		break;
	case TCT_PERF_GET_UITURBO:
		ret = tct_perf_get_uiturbo(arg);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

#ifdef CONFIG_COMPAT
static long tct_perf_compat_ioctl(struct file *file, unsigned int cmd,
				  unsigned long arg)
{
	return tct_perf_ioctl(file, cmd, (unsigned long)(compat_ptr(arg)));
}
#endif

static int tct_perf_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int tct_perf_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations tct_perf_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = tct_perf_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = tct_perf_compat_ioctl,
#endif
	.open = tct_perf_open,
	.release = tct_perf_release,
};

static struct miscdevice tct_perf_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "tct_perf",
	.fops = &tct_perf_fops,
};

static void uiturbo_process_exit(void *ignore, struct task_struct *p)
{
	/* clear uiturbo flag when ui thread exit */
	if (READ_ONCE(p->static_uiturbo) == TURBO_UI) {
		WRITE_ONCE(p->static_uiturbo, TURBO_NONE);
		atomic_dec(&nr_ui_threads);
	}
}

static inline bool current_ioprio_high()
{
	return task_nice_ioclass(current) == IOPRIO_CLASS_RT ||
		test_task_uiturbo(current);
}

static inline bool bio_need_turbo(struct bio *bio)
{
	if (current_ioprio_high())
		return true;
	if (bio_op(bio) != REQ_OP_READ)
		return false;
	if (bio->bi_opf & (REQ_META | REQ_PRIO))
		return true;
	return false;
}

#define REQ_UI (1 << 31)
static void uiturbo_bio_queue(void *ignore, struct request_queue *q,
			      struct bio *bio)
{
	if (uiturbo_enable) {
		BUILD_BUG_ON(__REQ_NR_BITS > 30);
		if (bio_need_turbo(bio))
			bio->bi_opf |= REQ_UI;
	}
}

// sysctl for uiturbo
static int zero;
static int one = 1;
static int one_hundred = 100;
static int max_sched_delay_granularity = 16;
static int min_migration_delay_granularity = 4;
static int util_max = SCHED_CAPACITY_SCALE;

static struct ctl_table uiturbo_sysctl_table[] = {
	{
		.procname = "uiturbo_enable",
		.data = &uiturbo_enable,
		.maxlen = sizeof(unsigned int),
		.mode = 0644,
		.proc_handler = proc_dointvec_minmax,
		.extra1     = &zero,
		.extra2     = &one,
	},
	{
		.procname   = "uiturbo_sched_delay_granularity",
		.data       = &uiturbo_sched_delay_granularity,
		.maxlen     = sizeof(int),
		.mode       = 0644,
		.proc_handler = proc_dointvec_minmax,
		.extra1     = &zero,
		.extra2     = &max_sched_delay_granularity,
	},
	{
		.procname   = "uiturbo_sched_runtime",
		.data       = &uiturbo_sched_runtime,
		.maxlen     = sizeof(int),
		.mode       = 0644,
		.proc_handler = proc_dointvec_minmax,
		.extra1     = &one,
		.extra2     = &one_hundred,
	},
	{
		.procname   = "dynamic_uiturbo_sched_granularity",
		.data       = &dynamic_uiturbo_sched_granularity,
		.maxlen     = sizeof(int),
		.mode       = 0644,
		.proc_handler = proc_dointvec_minmax,
		.extra1     = &zero,
		.extra2     = &one_hundred,
	},
	{
		.procname   = "uiturbo_migration_delay",
		.data       = &uiturbo_migration_delay,
		.maxlen     = sizeof(int),
		.mode       = 0644,
		.proc_handler = proc_dointvec_minmax,
		.extra1     = &min_migration_delay_granularity,
		.extra2     = &one_hundred,
	},
	{
		.procname   = "uiturbo_max_depth",
		.data       = &uiturbo_max_depth,
		.maxlen     = sizeof(int),
		.mode       = 0644,
		.proc_handler = proc_dointvec_minmax,
		.extra1     = &zero,
		.extra2     = &one_hundred,
	},
	{
		.procname   = "uiturbo_max_threads",
		.data       = &uiturbo_max_threads,
		.maxlen     = sizeof(int),
		.mode       = 0644,
		.proc_handler = proc_dointvec_minmax,
		.extra1     = &zero,
		.extra2     = &one_hundred,
	},
	{
		.procname   = "uiturbo_util_clamp_min",
		.data       = &uiturbo_util_clamp_min,
		.maxlen     = sizeof(int),
		.mode       = 0644,
		.proc_handler = proc_dointvec_minmax,
		.extra1     = &zero,
		.extra2     = &util_max,
	},
	{ }
};

static int __init tct_perf_init(void)
{
	atomic_set(&uiturbo_log.cur, -1);
	proc_create_data("uiturbo_log", 0440, NULL, &uiturbo_log_fops, NULL);
	WARN_ON(register_trace_sched_process_exit(uiturbo_process_exit, NULL));
	WARN_ON(register_trace_block_bio_queue(uiturbo_bio_queue, NULL));
	register_sysctl("kernel", uiturbo_sysctl_table);

	return misc_register(&tct_perf_device);
}

static __exit void tct_perf_exit(void)
{
	WARN_ON(unregister_trace_sched_process_exit(uiturbo_process_exit, NULL));
	WARN_ON(unregister_trace_block_bio_queue(uiturbo_bio_queue, NULL));
	tracepoint_synchronize_unregister();
	misc_deregister(&tct_perf_device);
}

module_init(tct_perf_init);
module_exit(tct_perf_exit);
MODULE_AUTHOR("yanwu <wu-yan@tcl.com>");
MODULE_DESCRIPTION("tct performance driver");
MODULE_LICENSE("GPL v2");
