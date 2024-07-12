#include <linux/mm.h>
#include <linux/vmacache.h>
#include <linux/hugetlb.h>
#include <linux/huge_mm.h>
#include <linux/mount.h>
#include <linux/seq_file.h>
#include <linux/highmem.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/pagemap.h>
#include <linux/mempolicy.h>
#include <linux/rmap.h>
#include <linux/sched/mm.h>
#include <linux/shmem_fs.h>
#include <linux/uaccess.h>
#include <linux/mm_inline.h>
#include <linux/ctype.h>
#include <linux/fs.h>
#include <linux/swap.h>
#include <linux/pagewalk.h>
#include "internal.h"
#include <linux/module.h>
#include <linux/gfp.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <asm/tlbflush.h>
#include <linux/swapops.h>
#include <linux/sysctl.h>
#include <linux/input.h>
#include <linux/sysfs.h>

//#define RECLAIM_DEBUG

#ifdef RECLAIM_DEBUG
#define dprintk(x...) printk(x)
#else
#define dprintk(x...)
#endif

#define ABORT_BY_TOUCHING 1
#define ABORT_BY_MM_LOCK 2
#define ABORT_BY_APP_STARTING 3

enum reclaim_policy{
	RECLAIM_ANON_LEVEL0,
	RECLAIM_ANON_LEVEL1,
	RECLAIM_ANON_LEVEL2,
	RECLAIM_ANON_LEVEL3,
};

//extern int reclaim_pte_range_ext(pmd_t *pmd, unsigned long addr,unsigned long end, struct mm_walk *walk);

extern unsigned long invalidate_mapping_pages_ext(struct address_space *mapping,
		pgoff_t start, pgoff_t end);

extern vm_fault_t do_swap_page(struct vm_fault *vmf);

struct reclaim_param {
  	struct vm_area_struct *vma;
  	int nr_to_reclaim;
  	int nr_reclaimed;
	enum reclaim_policy rpolicy;
};

static int abort_reclaim = 0;

static atomic_t touching = ATOMIC_INIT(0);
static struct delayed_work input_event_wk;
static u64 last_input_time = 0;
#define TOUCH_INTERVAL_MSEC (3000)

static void input_event_check(struct work_struct *work)
{
	atomic_set(&touching,0);
}

static void reclaim_input_event(struct input_handle *handle,
		unsigned int type, unsigned int code, int value)
{
	u64 now;

	now = ktime_to_us(ktime_get());
	if (now - last_input_time < (2*USEC_PER_SEC))
		return;

	atomic_set(&touching,1);

	mod_delayed_work(system_wq, &input_event_wk,
					msecs_to_jiffies(TOUCH_INTERVAL_MSEC));

	last_input_time = ktime_to_us(ktime_get());
}

static int reclaim_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "mmreclaim";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void reclaim_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id reclaim_ids[] = {
	/* multi-touch touchscreen */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
			BIT_MASK(ABS_MT_POSITION_X) |
			BIT_MASK(ABS_MT_POSITION_Y) },
	},
	/* touchpad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_KEYBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
		.absbit = { [BIT_WORD(ABS_X)] =
			BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) },
	},
	/* Keypad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },
};

static struct input_handler reclaim_input_handler = {
	.event          = reclaim_input_event,
	.connect        = reclaim_input_connect,
	.disconnect     = reclaim_input_disconnect,
	.name           = "mmreclaim",
	.id_table       = reclaim_ids,
};

const char *parse_suffix(const char *path) {
	const char *dot = NULL;
	if (path == NULL) {
		return NULL;
	}
	dot = strrchr(path, '.');
	if (!dot || dot == path) {
		return NULL;
	}
	return dot + 1;
}


static bool is_pattern_vma(struct vm_area_struct *vma,char* type_buf) {
	const char *path;
	const char* suffix;
	bool reverse_type = false;
	
	if(!vma->vm_file)
		return false;

	if(strcmp(type_buf,"file")){
		return true;
	}

	if(strncmp(type_buf,"file:-",6)){
		reverse_type = true;
	}

	path = vma->vm_file->f_path.dentry->d_name.name;
	
	suffix = parse_suffix(path);
	if(suffix != NULL){
		if(strstr(type_buf, suffix)){
			if(reverse_type)
				return false;
			else{
				dprintk("reclaim vma path=%s,size:%dK\n",
					path,(vma->vm_end-vma->vm_start)/1024);
				return true;
			}
		}else if(reverse_type){
			dprintk("reclaim vma path=%s,size:%dK\n",
				path,(vma->vm_end-vma->vm_start)/1024);
			return true;
		}
	}

	return false;
}

int should_cancel_reclaim(struct mm_struct *mm){
	/*got touch event*/
	if(atomic_read(&touching))
		return ABORT_BY_TOUCHING;
	/* abort reclaim in case app starting*/
	if(abort_reclaim)
		return ABORT_BY_APP_STARTING;
/*
	if(mm && !list_empty(&mm->mmap_sem.wait_list)){
		printk("cancel reclaim by lock race\n");
		return ABORT_BY_MM_LOCK;
	}
*/
	return 0;
}

static unsigned long invalidate_mapping_vma(struct vm_area_struct * vma)
{
	unsigned long count = 0;

	if(vma->vm_file && vma->vm_file->f_mapping){

		count = invalidate_mapping_pages_ext(vma->vm_file->f_mapping,
					vma->vm_pgoff,(vma->vm_pgoff+vma_pages(vma)));

		dprintk("should reclaim count:%d,actually reclaim count:%d\n"
			,vma_pages(vma),count);
		
	}else{
		printk("invalidate_mapping_vma invalid vma\n");
	}
	return count;
}

/*Below function is same as "reclaim_pte_range" in task_mmu.c*/
int reclaim_pte_range_ext(pmd_t *pmd, unsigned long addr,
				unsigned long end, struct mm_walk *walk)
{
#ifdef CONFIG_INTELLIGENT_RECLAIM
	struct reclaim_param *rp = walk->private;
	struct vm_area_struct *vma = rp->vma;
	int reclaimed;
	int abort = 0;
#else
	struct vm_area_struct *vma = walk->private;
#endif
	pte_t *pte, ptent;
	spinlock_t *ptl;
	struct page *page;
	LIST_HEAD(page_list);
	int isolated;

	split_huge_pmd(vma, addr, pmd);
	if (pmd_trans_unstable(pmd))
		return 0;
cont:
	isolated = 0;
	pte = pte_offset_map_lock(vma->vm_mm, pmd, addr, &ptl);
	for (; addr != end; pte++, addr += PAGE_SIZE) {
#ifdef CONFIG_INTELLIGENT_RECLAIM
		if((abort = should_cancel_reclaim(vma->vm_mm)))
			break;
#endif
		ptent = *pte;
		if (!pte_present(ptent))
			continue;

		page = vm_normal_page(vma, addr, ptent);
		if (!page)
			continue;
#ifdef CONFIG_INTELLIGENT_RECLAIM
		if(PageUnevictable(page))
			continue;
		if(rp->rpolicy == RECLAIM_ANON_LEVEL1){
			//don't reclaim the page in active lru and have been taged with referenced flag
			if(PageActive(page) && PageReferenced(page))
				continue;
		}else if(rp->rpolicy == RECLAIM_ANON_LEVEL2){
			//don't reclaim the page in active lru
			if(PageActive(page))
				continue;
		}else if(rp->rpolicy == RECLAIM_ANON_LEVEL3){
			//don't reclaim the page in active lru or have been taged with referenced flag
			if(PageActive(page) || PageReferenced(page))
				continue;
		}
#endif
		if (isolate_lru_page(compound_head(page)))
			continue;

		/* MADV_FREE clears pte dirty bit and then marks the page
		 * lazyfree (clear SwapBacked). Inbetween if this lazyfreed page
		 * is touched by user then it becomes dirty.  PPR in
		 * shrink_page_list in try_to_unmap finds the page dirty, marks
		 * it back as PageSwapBacked and skips reclaim. This can cause
		 * isolated count mismatch.
		 */
		if (PageAnon(page) && !PageSwapBacked(page)) {
			putback_lru_page(page);
			continue;
		}

		list_add(&page->lru, &page_list);
		inc_node_page_state(page, NR_ISOLATED_ANON +
				page_is_file_cache(page));
		isolated++;
#ifdef CONFIG_INTELLIGENT_RECLAIM
		if ((isolated >= SWAP_CLUSTER_MAX) || !rp->nr_to_reclaim)
#else
		if (isolated >= SWAP_CLUSTER_MAX)
#endif
			break;
	}
	pte_unmap_unlock(pte - 1, ptl);
	reclaimed = reclaim_pages_from_list(&page_list, vma);
#ifdef CONFIG_INTELLIGENT_RECLAIM
	rp->nr_reclaimed += reclaimed;
  	rp->nr_to_reclaim -= reclaimed;
  	if (rp->nr_to_reclaim < 0)
  		rp->nr_to_reclaim = 0;
#endif
#ifdef CONFIG_INTELLIGENT_RECLAIM
	if (rp->nr_to_reclaim && (addr != end) && !abort)
#else
	if (addr != end)
#endif
		goto cont;

	cond_resched();
	return 0;
} 

/*
reclaim format:
reclaim file page include given file,ie "file:dex,odex,apk"
reclaim file page not including given file,ie "file:-so"
reclaim all anon page,ie "anon0" 
recliam anon page not including the page with hot and referenced flag,ie "anon1"
recliam anon page not including the page with hot flag,ie "anon2"
recliam anon page not including the page with hot or referenced flag,ie "anon3"
*/
static ssize_t intelligent_reclaim_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	struct task_struct *task;
	char buffer[200];
	struct mm_struct *mm;
	char *type_buf;
	struct vm_area_struct *vma;
	int abort=0;
	int reclaimed_count=0;
	struct timespec64 start;
	struct timespec64 end;
	struct timespec64 duration;
	struct reclaim_param rp;

	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	type_buf = strstrip(buffer);

	task = get_pid_task(proc_pid(file->f_path.dentry->d_inode),PIDTYPE_PID);
	if (!task)
		return -ESRCH;

	mm = get_task_mm(task);
	if (!mm){
		put_task_struct(task);
		return -ESRCH;
	}

	ktime_get_ts64(&start);

	down_read(&mm->mmap_sem);

	if (!strncmp(type_buf, "file",4)){
		for (vma = mm->mmap ; vma; vma = vma->vm_next) {
			if((abort = should_cancel_reclaim(mm))){
				break;
			}

			if (vma->vm_flags & (VM_LOCKED|VM_HUGETLB|VM_PFNMAP)) {
				continue;
			}
			/*don't relcaim anon page*/
			if (vma->anon_vma) {
				continue;
			}
			if (!vma->vm_file || !vma->vm_file->f_mapping) {
				continue;
			}
			/* only reclaim reg file */
			if(!S_ISREG(file_inode(vma->vm_file)->i_mode)){
				continue;
			}
		
			if (is_pattern_vma(vma,type_buf)) {
				/* unmapped the vma range */
				zap_page_range(vma,vma->vm_start,vma->vm_end-vma->vm_start);

				reclaimed_count += invalidate_mapping_vma(vma);

				dprintk("path:%s,vma->vm_end:%x,vma->vm_start:%x\n",
					vma->vm_file->f_path.dentry->d_name.name,vma->vm_end,vma->vm_start);
			}
		}
		
	}else if(!strncmp(type_buf, "anon",4)){
		const struct mm_walk_ops reclaim_walk_ops = {
			.pmd_entry = reclaim_pte_range_ext,
		};
		rp.nr_to_reclaim = INT_MAX;
		rp.nr_reclaimed = 0;
		if(type_buf[4] == '1'){
			rp.rpolicy = RECLAIM_ANON_LEVEL1;
		}else if(type_buf[4] == '2'){
			rp.rpolicy = RECLAIM_ANON_LEVEL2;
		}else if(type_buf[4] == '3'){
			rp.rpolicy = RECLAIM_ANON_LEVEL3;
		}else{
			rp.rpolicy = RECLAIM_ANON_LEVEL0;
		}
		
		for (vma = mm->mmap; vma; vma = vma->vm_next) {
			if((abort = should_cancel_reclaim(mm))){
				break;
			}

			if (is_vm_hugetlb_page(vma))
				continue;
			if (vma->vm_file)
				continue;
			rp.vma = vma;
			walk_page_range(mm, vma->vm_start, vma->vm_end,
					&reclaim_walk_ops, &rp);
		}
		reclaimed_count = rp.nr_reclaimed;
	}
	
	flush_tlb_mm(mm);
	up_read(&mm->mmap_sem);

	mmput(mm);

	ktime_get_ts64(&end);
	duration = timespec64_sub(end, start);
	printk("%s type:%s,reclaim task:%s count:%d elapsed %ld ms,abort:%d\n",
		__func__,type_buf,task->comm,reclaimed_count,(duration.tv_sec*1000+duration.tv_nsec/NSEC_PER_MSEC),abort);

	put_task_struct(task);
	if(abort)
		return -EAGAIN;
	return count;
}

const struct file_operations proc_intelligent_reclaim_operations = {
	.write		= intelligent_reclaim_write,
	.llseek		= noop_llseek,
};

static struct ctl_table reclaim_kern_table[] = {
  	{
  		.procname	= "abort_reclaim",
  		.data		= &abort_reclaim,
  		.maxlen		= sizeof(abort_reclaim),
  		.mode		= 0666,
  		.proc_handler	= proc_dointvec,
  	},
	{}
};

static struct ctl_table root_table[] = {
  	{
  		.procname	= "vm",
  		.mode		= 0555,
  		.child		= reclaim_kern_table,
  	},
  	{}
};

static int __init intelligent_reclaim_init(void)
{
	register_sysctl_table(root_table);
	INIT_DELAYED_WORK(&input_event_wk, input_event_check);
	input_register_handler(&reclaim_input_handler);
	return 0;
}

static void __exit intelligent_reclaim_exit(void)
{
	return;
}


module_init(intelligent_reclaim_init);
module_exit(intelligent_reclaim_exit);
MODULE_LICENSE("GPL");
