#include <linux/types.h>
#include <linux/list.h>
#include <linux/atomic.h>
#include <linux/shrinker.h>
#include <linux/netlink.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/mm.h>
#include <linux/kfifo.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/swap.h>
#include <net/sock.h>

#define MMPRESSURE_NAME "mmpressure"
#define FIFO_SIZE 4096

enum mm_msg_type{
	MM_MSG_TYPE_TMS = 1,
	MM_MSG_TYPE_DMCC = 2,
};
enum mm_pressure_level{
	MM_PRESSURE_OK = 0,
	MM_PRESSURE_CACHED_LEVEL = 1,
	MM_PRESSURE_HEAVY_LEVEL = 2,
	MM_PRESSURE_PERSISTENT_LEVEL = 3,
	MM_PRESSURE_MAX_LEVEL = 4,
};

struct umsg_header{
	int magic;
	int lenth; //message length;
	int type;  // MM_MSG_TYPE_TMS OR MM_MSG_TYPE_DMCC;
	int threshold_len; //how many param
};
struct tms_wmark_level_t{
	int tms_cached_level;
	int tms_heavy_level;
	int tms_persistent_level;
};

struct mm_pressure_msg{
	int length;
	int type;
	int param_lens;
	int wmark_level;
};

struct tms_wmark_level_t tms_wmark_level;

static struct kfifo mstate_fifo;
static spinlock_t fifo_lock;
static spinlock_t time_lock;
static struct sock *netlink_fd = NULL;
static struct task_struct *mmstate_thread = NULL;
static int mm_user_pid = -1;
static wait_queue_head_t wait_report;
static int tms_ok = 0;

static unsigned long time_stamp[MM_PRESSURE_MAX_LEVEL] = {0};

static int mfifo_in(struct mm_pressure_msg *msg)
{
	int len = 0;
	unsigned int available = 0;	
	spin_lock(&fifo_lock);

	available = kfifo_avail(&mstate_fifo);
	
	if(available > sizeof(struct mm_pressure_msg)){
		len = kfifo_in(&mstate_fifo, msg, sizeof(struct mm_pressure_msg));
	}
	
	spin_unlock(&fifo_lock);
	
	return len;
}
static int mfifo_out(struct mm_pressure_msg* msg)
{
	int len = 0;
	if(NULL == msg){
		printk(KERN_ALERT "[mmpressure]%s: msg is NULL, err\n",__func__);
		return 0;	
	}
	spin_lock(&fifo_lock);

	if(!kfifo_is_empty(&mstate_fifo)){
		len = kfifo_out(&mstate_fifo, msg, sizeof(struct mm_pressure_msg));
	}
	spin_unlock(&fifo_lock);
	
	return len;
}

static int water_lv(unsigned long mem_avaliable_kb)
{
	int lv = 0;
	
	if(mem_avaliable_kb <= tms_wmark_level.tms_persistent_level){
		lv = MM_PRESSURE_PERSISTENT_LEVEL;
	}else if((mem_avaliable_kb <= tms_wmark_level.tms_heavy_level) && (mem_avaliable_kb > tms_wmark_level.tms_persistent_level)){
		lv = MM_PRESSURE_HEAVY_LEVEL;
	}else if((mem_avaliable_kb <= tms_wmark_level.tms_cached_level) && (mem_avaliable_kb > tms_wmark_level.tms_heavy_level)){
		lv = MM_PRESSURE_CACHED_LEVEL;
	}else{
		lv = MM_PRESSURE_OK;
	}
	return lv;
}
static unsigned long lowmem_scan(struct shrinker *s, struct shrink_control *sc)
{
	int lv = 0;
	unsigned long mem_avaliable = 0;
	unsigned long mem_avaliable_kb = 0;
	struct mm_pressure_msg mmsg_s;

	if(tms_ok){
		mem_avaliable = si_mem_available();
		mem_avaliable_kb = mem_avaliable << (PAGE_SHIFT - 10);
		lv = water_lv(mem_avaliable_kb);
		if(lv){
			spin_lock(&time_lock);
			if(time_after(jiffies,(time_stamp[lv] + msecs_to_jiffies(300)))){
				time_stamp[lv] = jiffies;
				spin_unlock(&time_lock);

				mmsg_s.length = sizeof(struct mm_pressure_msg);
				mmsg_s.type = MM_MSG_TYPE_TMS;
				mmsg_s.param_lens = 0; //no use
				mmsg_s.wmark_level = lv;
                        	mfifo_in(&mmsg_s);
                        	wake_up_interruptible(&wait_report);
			}else{
				spin_unlock(&time_lock);
			}
		}else {
			//do nothing
		}
	}
	return SHRINK_STOP;
}

static unsigned long lowmem_count(struct shrinker *s, struct shrink_control *sc)
{
	return global_node_page_state(NR_ACTIVE_ANON) +
                global_node_page_state(NR_ACTIVE_FILE) +
                global_node_page_state(NR_INACTIVE_ANON) +
                global_node_page_state(NR_INACTIVE_FILE);
}


static void process_user_data(struct umsg_header *recv_buf, int lens)
{
	struct tms_wmark_level_t* umsg;
	if(NULL == recv_buf){
		printk(KERN_ALERT "[mmpressure]%s: recv_buf is NULL, err\n",__func__);
		return;
	}

	switch(recv_buf->type){
		case MM_MSG_TYPE_TMS:
			umsg = (struct tms_wmark_level_t*)(((char *)recv_buf) + sizeof(struct umsg_header));
			if(lens != (sizeof(struct umsg_header) + sizeof(struct tms_wmark_level_t))){
				printk(KERN_ALERT "[mmpressure] %s:lens error , lens should be %d, but is %d\n",
					__func__,(sizeof(struct umsg_header) + sizeof(struct tms_wmark_level_t)),lens);
				break;			
			}
			if(recv_buf->threshold_len != sizeof(struct tms_wmark_level_t)/sizeof(int)){
				printk(KERN_ALERT "[mmpressure] %s,param lens err\n",__func__);
				break;
			}

			tms_wmark_level.tms_cached_level = umsg->tms_cached_level;
			tms_wmark_level.tms_heavy_level = umsg->tms_heavy_level;
			tms_wmark_level.tms_persistent_level = umsg->tms_persistent_level;
			printk("[mmpressure] cached level:%d,heavy level:%d,persistent level:%d\n",
				tms_wmark_level.tms_cached_level,tms_wmark_level.tms_heavy_level,tms_wmark_level.tms_persistent_level);
			tms_ok = 1;
			break;
		default:
			printk(KERN_ALERT "[mmpressure] %s: get value err\n",__func__);
			break;
	}
}

static int magic_check(struct umsg_header *msg)
{
	int ret = 0;
	if(msg->magic == 0xaabbccdd)
		ret = 1;
	else
		printk(KERN_ALERT "[mmpressure] %s:magic_check failed, magic = 0x%x\n",__func__,msg->magic);
	return ret;
}
static void recv_from_user(struct sk_buff *skb)
{
	struct sk_buff *tmp_skb = NULL;
	struct nlmsghdr *nlh = NULL;
	int len = 0;

	if(!skb){
		printk(KERN_ALERT "[mmpressure] %s: skb is NULL\n",__func__);
		return;
	}
	printk(KERN_ALERT "[mmpressure] %s: enter\n",__func__);
	tmp_skb = skb;
	
	if(tmp_skb->len > NLMSG_SPACE(0)){
		nlh = nlmsg_hdr(tmp_skb);
		if(nlh->nlmsg_pid > 0){
			mm_user_pid = nlh->nlmsg_pid;
		}
		printk(KERN_ALERT "[mmpressure] %s: mm_user_pid is %d\n",__func__,mm_user_pid);	
		len = NLMSG_PAYLOAD(nlh, 0);

		if(!magic_check((struct umsg_header *)NLMSG_DATA(nlh))){
			printk(KERN_ALERT "[mmpressure] %s: magic check failed\n",__func__);
			return;
		}
		process_user_data((struct umsg_header*)(NLMSG_DATA(nlh)), len);
	}
	printk(KERN_ALERT "[mmpressure] %s:exit\n",__func__);
}
static int send_to_user(struct mm_pressure_msg *msg)
{
	struct sk_buff *skb = NULL;
	struct nlmsghdr *nlh = NULL;
	int len;
	int ret = -1;
	if(!msg){
		printk(KERN_ALERT "[mmpressure] mmsg is NULL\n");
		goto err;
	}
	len = sizeof(struct mm_pressure_msg);

	skb = alloc_skb(NLMSG_SPACE(len),GFP_ATOMIC);
	if(!skb){
		printk(KERN_ALERT "[mmpressure] alloc akb failed\n");
		goto err;
	}

	nlh = nlmsg_put(skb, 0, 0, 0, len, 0);
	
	NETLINK_CB(skb).portid = 0;
	NETLINK_CB(skb).dst_group = 0;
		
	memcpy(NLMSG_DATA(nlh), msg, len);


	if((ret = netlink_unicast(netlink_fd, skb, mm_user_pid, MSG_DONTWAIT)) < 0){
		printk(KERN_ALERT "[mmpressure] send user failed,ret = %d\n",ret);
		goto err;
	}
	ret = 0;

err:
	return ret;
}

static int mmpressrpt_report(void *data)
{
	struct mm_pressure_msg mmsg_s;
	int len = 0;
	while(!kthread_should_stop()){
		wait_event_interruptible(wait_report, !kfifo_is_empty(&mstate_fifo));
		while(!kfifo_is_empty(&mstate_fifo)){
			len = mfifo_out(&mmsg_s);
			if(len != 0){
				send_to_user(&mmsg_s);
			}
		}
	}
	printk("[mmpressure] Leave mmstate thread\n");
	return 0;
}

static struct shrinker lowmem_report_shrinker = {
	.scan_objects = lowmem_scan,
	.count_objects = lowmem_count,
	.seeks = DEFAULT_SEEKS*16,
};

static struct netlink_kernel_cfg cfg = {
		.groups = 0,
		.input = recv_from_user,
	};
static int __init mmpressrpt_init(void)
{
	int ret = 0;
	spin_lock_init(&fifo_lock);
	spin_lock_init(&time_lock);
	ret = kfifo_alloc(&mstate_fifo,FIFO_SIZE,GFP_ATOMIC);
	if(0 != ret){
		printk("[mmpressure]  %s: alloc fifo failed\n",__func__);
		goto err3;
	}
	register_shrinker(&lowmem_report_shrinker);
	netlink_fd = netlink_kernel_create(&init_net,NETLINK_MM_STATE, &cfg);
	if(!netlink_fd){
		printk("[mmpressure]  create netlink sock failed\n");
		goto err2;
	}
	init_waitqueue_head(&wait_report);
	/*running thread for report mm state*/
	mmstate_thread = kthread_run(mmpressrpt_report, NULL, MMPRESSURE_NAME);
	if(IS_ERR_OR_NULL(mmstate_thread)){
		printk("[mmpressure]  Create mmstate thread failed\n");
		goto err1;
	}
	printk("[mmpressure] mmpressrpt_init sucess\n");
	return 0;
err1:
	netlink_kernel_release(netlink_fd);
err2:
	unregister_shrinker(&lowmem_report_shrinker);
err3:	
	return -1;
}
static void __exit mmpressrpt_exit(void)
{
        if(mmstate_thread){
                kthread_stop(mmstate_thread);
        }

	if(netlink_fd){
		unregister_shrinker(&lowmem_report_shrinker);
		netlink_kernel_release(netlink_fd);
		kfifo_free(&mstate_fifo);
	}
}
module_init(mmpressrpt_init);
module_exit(mmpressrpt_exit);
MODULE_LICENSE("GPL V2");
MODULE_DESCRIPTION("use to send mm pressure");
MODULE_AUTHOR("TCL");
