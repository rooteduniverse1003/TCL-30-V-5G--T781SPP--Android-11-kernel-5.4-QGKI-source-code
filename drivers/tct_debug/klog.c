#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/printk.h>
#include <linux/rwsem.h>
#include <linux/uaccess.h>
#include <net/genetlink.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/atomic.h>
#include <linux/workqueue.h>
#include <linux/klog.h>
#include <linux/proc_fs.h>

/* XXX: add your group here */
static struct genl_multicast_group klog_grps[] = {
	/* generic group */
	{.name		= "debug",	},
	{.name		= "info",	},
	{.name		= "warn",	},
	{.name		= "err",	},

	/* special group */
	{.name		= "ssr",	},
	{.name		= "fb",		},
	{.name		= "other",	},
	{.name		= "mmc",	},
};

static struct genl_family klog_fam __ro_after_init;

struct early_buffer {
	struct sk_buff *skb;
	int group_index;
	struct list_head node;
};

static DEFINE_SPINLOCK(early_list_lock);
static atomic_t klogd_started = ATOMIC_INIT(0);
static LIST_HEAD(klog_early_list);
static int early_buffer_count = 0;

static void klog_post_early_buffer(struct work_struct *work)
{
	struct early_buffer *new, *next;
	int ret;

	atomic_set(&klogd_started, 1);

	spin_lock(&early_list_lock);
	list_for_each_entry_safe(new, next, &klog_early_list, node) {

		ret = genlmsg_multicast(&klog_fam, new->skb, 0,
				new->group_index, GFP_ATOMIC);
		if (ret) {
			klog_debug("Error happened while sending early klog message %d.\n", ret);
		}

		list_del(&new->node);
		kfree(new);
	}
	spin_unlock(&early_list_lock);
}

static DECLARE_DELAYED_WORK(klog_work, klog_post_early_buffer);

static int klogd_start(struct sk_buff *skb, struct genl_info *info)
{
	schedule_delayed_work(&klog_work, 0);

	return 0;
}

static struct genl_ops klog_ops[] = {
	{
		.cmd = NL_KLOG_CMD_START,
		.doit = klogd_start,
	},
};

static struct genl_family klog_fam __ro_after_init = {
	.id = 0, //GENL_ID_GENERATE
	.name = "klog",
	.hdrsize = 0,
	.version = 1,
	.maxattr = NL_KLOG_ATTR_MAX,
	.netnsok = true,
	.mcgrps 	= klog_grps,
	.ops		= klog_ops,
	.n_ops		= ARRAY_SIZE(klog_ops),
	.n_mcgrps	= ARRAY_SIZE(klog_grps),

};

static int klog_early_print(struct sk_buff *skb, int group_index)
{
	struct early_buffer *buf;

	if (early_buffer_count++ > 100)
		return -ENOMEM;

	buf = kmalloc(sizeof(struct early_buffer), GFP_ATOMIC);
	if (!buf)
		return -ENOMEM;

	buf->skb = skb;
	buf->group_index = group_index;

	spin_lock(&early_list_lock);
	list_add_tail(&buf->node, &klog_early_list);
	spin_unlock(&early_list_lock);

	return 0;
}

static int klog_msg_event(unsigned int group,
		unsigned int attr, struct klog_msg *msg)
{
	struct sk_buff *skb;
	void *hdr;
	static unsigned int event_seqnum;

	skb = genlmsg_new(NLMSG_DEFAULT_SIZE, GFP_ATOMIC);
	if (!skb)
		return -ENOMEM;

	hdr = genlmsg_put(skb, 0, event_seqnum++, &klog_fam, 0, NL_KLOG_CMD_EVENT);
	if (!hdr) {
		nlmsg_free(skb);
		return -ENOMEM;
	}

	if (nla_put(skb, attr, sizeof(*msg), msg)) {
    	genlmsg_cancel(skb, hdr);
		nlmsg_free(skb);
    	return -EMSGSIZE;
	}

	genlmsg_end(skb, hdr);

	//nlmsg_free(skb);
		
	if (atomic_read(&klogd_started))
		return genlmsg_multicast(&klog_fam, skb, 0, group, GFP_ATOMIC);
	else
		return klog_early_print(skb, msg->id);
}

int klog_print(const char *name, const char *fmt, ...)
{
	va_list args;
	unsigned int group;
	int err;
	struct klog_msg *msg;
	int index = -1;
	int i;
	printk("%s %d\n", __func__, __LINE__);

	for (i = 0; i < ARRAY_SIZE(klog_grps); i++) {
		if (strncmp(name, klog_grps[i].name,
					strlen(klog_grps[i].name)) == 0)
		index = i;
	}

	if (index < 0) {
		printk(KERN_ERR "Please add your group [%s] to klog_grps.\n", name);
		return -EINVAL;
	}

	msg = kzalloc(sizeof(*msg), GFP_ATOMIC);
	if (!msg)
		return -ENOMEM;

	msg->id = index;
	msg->ts_nsec = local_clock();

	va_start(args, fmt);
	vsnprintf(msg->data, sizeof(msg->data), fmt, args);
	va_end(args);

	group = index;
	strncpy(msg->label, klog_grps[index].name, sizeof(msg->label));

	pr_info("[%d] %s", index, msg->data);

	err = klog_msg_event(group, NL_KLOG_ATTR_FIRST, msg);
	if (err)
		printk(KERN_ERR "Failed to klog_print(%s) %d\n", msg->label, err);

	kfree(msg);

	return err;
}
EXPORT_SYMBOL(klog_print);

static ssize_t klog_proc_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char *msg;
	char *name = "other";
	int i;

	if (count == 0)
		return 0;

	msg = kmalloc(count+1, GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	if (copy_from_user(msg, buf, count)) {
		printk(KERN_ERR "Failed to copy_from_user().\n");
		goto out;
	}
	msg[count] = '\0';

	for (i = 0; i < ARRAY_SIZE(klog_grps); i++) {
		if (strncmp(msg, klog_grps[i].name,
					strlen(klog_grps[i].name)) == 0)
			name = klog_grps[i].name;
	}

	klog_print(name, "%s", msg);

out:
	kfree(msg);
	return count;
}

static int klog_proc_show(struct seq_file *m, void *v)
{
	int i;

	seq_printf(m, "Family: %s\n", klog_fam.name);
	seq_printf(m, "Groups:");
	for (i = 0; i < ARRAY_SIZE(klog_grps); i++) {
		seq_printf(m, " %s", klog_grps[i].name);
	}
	seq_printf(m, "\n");

	return 0;
}

static int klog_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, klog_proc_show, NULL);
}

static const struct file_operations klog_proc_fops = {
	.open = klog_proc_open,
	.read = seq_read,
	.write = klog_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static struct proc_dir_entry *proc;

static int __init klog_genl_init(void)
{
	int err;

	klog_info("%s %d\n", __func__, __LINE__);
	
	err = genl_register_family(&klog_fam);
	if (err < 0){
		klog_debug("klog: genl_register_family fail!\n");
		return err;
	}
	proc = proc_create("klog", S_IRWXU, NULL, &klog_proc_fops);
	if (!proc){
		klog_debug("klog: creat proc/klog  fail!\n");
		goto out;
	}
	return 0;
out:
	genl_unregister_family(&klog_fam);
	return err;
}

static void __exit klog_genl_exit(void)
{
	proc_remove(proc);
	
	genl_unregister_family(&klog_fam);
}

module_init(klog_genl_init);
module_exit(klog_genl_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Export log to userspace through generic netlink.");
MODULE_AUTHOR("wentaohe@tcl.com");
