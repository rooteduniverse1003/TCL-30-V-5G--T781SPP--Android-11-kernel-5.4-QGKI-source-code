#ifndef _TCT_UI_TURBO_H_
#define _TCT_UI_TURBO_H_

#include <linux/tct/perf.h>

struct rq;
struct task_struct;
struct sched_entity;

extern int uiturbo_enable;
extern int uiturbo_sched_delay_granularity;
extern int uiturbo_sched_runtime;
extern int dynamic_uiturbo_sched_granularity;
extern int uiturbo_migration_delay;
extern int uiturbo_max_depth;
extern int uiturbo_max_threads;
extern int uiturbo_util_clamp_min;

static inline bool task_is_ui(struct task_struct *p)
{
	return p->static_uiturbo == TURBO_UI;
}

static inline bool test_task_uiturbo(struct task_struct *p)
{
	if (!uiturbo_enable)
		return false;
	return p && (p->static_uiturbo == TURBO_UI ||
		     atomic64_read(&p->dynamic_uiturbo));
}

static inline bool test_task_uiturbo_depth(int depth)
{
	return depth < uiturbo_max_depth;
}

static inline bool test_set_dynamic_uiturbo(struct task_struct *p)
{
	return test_task_uiturbo(p)
	    && test_task_uiturbo_depth(p->uiturbo_depth);
}

extern void enqueue_uiturbo_thread(struct rq *rq, struct task_struct *p);
extern void dequeue_uiturbo_thread(struct rq *rq, struct task_struct *p);
extern void pick_uiturbo_thread(struct rq *rq, struct task_struct **p,
				struct sched_entity **se);
extern bool test_dynamic_uiturbo(struct task_struct *p, int type);
extern void dynamic_uiturbo_dequeue(struct task_struct *p, int type);
extern void dynamic_uiturbo_enqueue(struct task_struct *p, int type, int depth);
extern void trigger_uiturbo_balance(struct rq *rq);
extern void uiturbo_init_rq(struct rq *rq);
#endif
