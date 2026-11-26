/*
 * drivers/cpufreq/cpufreq_smartass2.c
 *
 * Copyright (C) 2010 Google, Inc.
 * SmartassV2 governor by Erasmux, adapted for Linux 3.4 MSM7x27a
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/sched.h>
#include <linux/tick.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/earlysuspend.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <asm/cputime.h>

static void (*pm_idle_old)(void);
static atomic_t active_count = ATOMIC_INIT(0);

struct smartass_info_s {
	struct cpufreq_policy *cur_policy;
	struct cpufreq_frequency_table *freq_table;
	struct timer_list timer;
	u64 time_in_idle;
	u64 idle_exit_time;
	u64 freq_change_time;
	u64 freq_change_time_in_idle;
	int cur_cpu_load;
	int old_freq;
	int ramp_dir;
	unsigned int enable;
	int ideal_speed;
};
static DEFINE_PER_CPU(struct smartass_info_s, smartass_info);

static struct workqueue_struct *up_wq;
static struct workqueue_struct *down_wq;
static struct work_struct freq_scale_work;

static cpumask_t work_cpumask;
static spinlock_t cpumask_lock;
static unsigned int suspended;

#define DEFAULT_AWAKE_IDEAL_FREQ 320000
static unsigned int awake_ideal_freq;

#define DEFAULT_SLEEP_IDEAL_FREQ 122880
static unsigned int sleep_ideal_freq;

#define DEFAULT_SLEEP_WAKEUP_FREQ 600000
static unsigned int sleep_wakeup_freq;

#define DEFAULT_MAX_CPU_LOAD 75
static unsigned long max_cpu_load;

#define DEFAULT_MIN_CPU_LOAD 45
static unsigned long min_cpu_load;

#define DEFAULT_RAMP_UP_STEP 160000
static unsigned int ramp_up_step;

#define DEFAULT_RAMP_DOWN_STEP 160000
static unsigned int ramp_down_step;

#define DEFAULT_UP_RATE_US 48000
static unsigned long up_rate_us;

#define DEFAULT_DOWN_RATE_US 99000
static unsigned long down_rate_us;

#define DEFAULT_SAMPLE_RATE_JIFFIES 2
static unsigned int sample_rate_jiffies;

static int cpufreq_governor_smartass(struct cpufreq_policy *policy,
				     unsigned int event);

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_SMARTASS2
static
#endif
struct cpufreq_governor cpufreq_gov_smartass2 = {
	.name = "smartassV2",
	.governor = cpufreq_governor_smartass,
	.max_transition_latency = 9000000,
	.owner = THIS_MODULE,
};

static inline void smartass_update_min_max(struct smartass_info_s *this_smartass,
					   struct cpufreq_policy *policy,
					   int suspend)
{
	if (suspend) {
		this_smartass->ideal_speed =
			policy->max > sleep_ideal_freq ?
			(sleep_ideal_freq > policy->min ? sleep_ideal_freq : policy->min) :
			policy->max;
	} else {
		this_smartass->ideal_speed =
			policy->min < awake_ideal_freq ?
			(awake_ideal_freq < policy->max ? awake_ideal_freq : policy->max) :
			policy->min;
	}
}

static inline void smartass_update_min_max_allcpus(void)
{
	unsigned int i;
	for_each_online_cpu(i) {
		struct smartass_info_s *this_smartass = &per_cpu(smartass_info, i);
		if (this_smartass->enable && this_smartass->cur_policy)
			smartass_update_min_max(this_smartass, this_smartass->cur_policy, suspended);
	}
}

static inline unsigned int validate_freq(struct cpufreq_policy *policy, int freq)
{
	if (freq > (int)policy->max)
		return policy->max;
	if (freq < (int)policy->min)
		return policy->min;
	return freq;
}

static inline void reset_timer(unsigned long cpu, struct smartass_info_s *this_smartass)
{
	this_smartass->time_in_idle = get_cpu_idle_time_us(cpu, &this_smartass->idle_exit_time);
	mod_timer(&this_smartass->timer, jiffies + sample_rate_jiffies);
}

static inline void work_cpumask_set(unsigned long cpu)
{
	unsigned long flags;
	spin_lock_irqsave(&cpumask_lock, flags);
	cpumask_set_cpu(cpu, &work_cpumask);
	spin_unlock_irqrestore(&cpumask_lock, flags);
}

static inline int work_cpumask_test_and_clear(unsigned long cpu)
{
	unsigned long flags;
	int res = 0;
	spin_lock_irqsave(&cpumask_lock, flags);
	res = cpumask_test_and_clear_cpu(cpu, &work_cpumask);
	spin_unlock_irqrestore(&cpumask_lock, flags);
	return res;
}

static int target_freq(struct cpufreq_policy *policy,
		       struct smartass_info_s *this_smartass,
		       int new_freq, int old_freq, int prefered_relation)
{
	int index, target;
	struct cpufreq_frequency_table *table = this_smartass->freq_table;

	if (new_freq == old_freq)
		return 0;
	new_freq = validate_freq(policy, new_freq);
	if (new_freq == old_freq)
		return 0;

	if (table &&
	    !cpufreq_frequency_table_target(policy, table, new_freq, prefered_relation, &index)) {
		target = table[index].frequency;
		if (target == old_freq) {
			if (new_freq > old_freq && ramp_up_step) {
				new_freq = validate_freq(policy, old_freq + ramp_up_step);
				cpufreq_frequency_table_target(policy, table, new_freq,
							       CPUFREQ_RELATION_H, &index);
				target = table[index].frequency;
			} else if (new_freq < old_freq && ramp_down_step) {
				new_freq = validate_freq(policy, old_freq - ramp_down_step);
				cpufreq_frequency_table_target(policy, table, new_freq,
							       CPUFREQ_RELATION_L, &index);
				target = table[index].frequency;
			}
		}
		if (target == old_freq)
			return 0;
	} else {
		target = new_freq;
	}

	__cpufreq_driver_target(policy, target, prefered_relation);
	return target;
}

static void cpufreq_smartass_timer(unsigned long cpu)
{
	u64 delta_idle;
	u64 delta_time;
	int cpu_load;
	int old_freq;
	u64 update_time;
	u64 now_idle;
	int queued_work = 0;
	struct smartass_info_s *this_smartass = &per_cpu(smartass_info, cpu);
	struct cpufreq_policy *policy = this_smartass->cur_policy;

	if (!this_smartass->enable || !policy)
		return;

	now_idle = get_cpu_idle_time_us(cpu, &update_time);
	old_freq = policy->cur;

	if (this_smartass->idle_exit_time == 0 || update_time == this_smartass->idle_exit_time)
		return;

	delta_idle = now_idle - this_smartass->time_in_idle;
	delta_time = update_time - this_smartass->idle_exit_time;

	if ((u32)delta_time < 1000)
		goto rearm;

	if (delta_idle > delta_time)
		cpu_load = 0;
	else
		cpu_load = 100 - ((unsigned int)delta_idle * 100) / (unsigned int)delta_time;

	this_smartass->cur_cpu_load = cpu_load;
	this_smartass->old_freq = old_freq;

	if ((u32)(update_time - this_smartass->freq_change_time) < up_rate_us &&
	    old_freq >= this_smartass->ideal_speed) {
		goto rearm;
	}

	if (cpu_load >= max_cpu_load) {
		if (old_freq < policy->max) {
			this_smartass->ramp_dir = 1;
			work_cpumask_set(cpu);
			queue_work(up_wq, &freq_scale_work);
			queued_work = 1;
		}
	} else if (cpu_load <= min_cpu_load &&
		   (u32)(update_time - this_smartass->freq_change_time) >= down_rate_us) {
		if (old_freq > policy->min) {
			this_smartass->ramp_dir = -1;
			work_cpumask_set(cpu);
			queue_work(down_wq, &freq_scale_work);
			queued_work = 1;
		}
	}

rearm:
	if (!timer_pending(&this_smartass->timer) && !queued_work)
		reset_timer(cpu, this_smartass);
}

static void cpufreq_idle(void)
{
	struct smartass_info_s *this_smartass = &per_cpu(smartass_info, smp_processor_id());

	if (!this_smartass->enable) {
		pm_idle_old();
		return;
	}

	pm_idle_old();

	if (!timer_pending(&this_smartass->timer))
		reset_timer(smp_processor_id(), this_smartass);
}

static void cpufreq_smartass_freq_change(struct work_struct *work)
{
	unsigned int cpu;
	int new_freq;
	int old_freq;
	int ramp_dir;
	struct smartass_info_s *this_smartass;
	struct cpufreq_policy *policy;
	unsigned int relation = CPUFREQ_RELATION_L;

	for_each_possible_cpu(cpu) {
		if (!work_cpumask_test_and_clear(cpu))
			continue;

		this_smartass = &per_cpu(smartass_info, cpu);
		if (!this_smartass->enable)
			continue;

		policy = this_smartass->cur_policy;
		if (!policy)
			continue;

		old_freq = policy->cur;
		ramp_dir = this_smartass->ramp_dir;
		this_smartass->ramp_dir = 0;

		if (ramp_dir > 0) {
			relation = CPUFREQ_RELATION_H;
			if (old_freq < this_smartass->ideal_speed)
				new_freq = this_smartass->ideal_speed;
			else if (ramp_up_step)
				new_freq = old_freq + ramp_up_step;
			else
				new_freq = policy->max;
			if (suspended && new_freq > sleep_ideal_freq)
				new_freq = sleep_ideal_freq;
		} else if (ramp_dir < 0) {
			relation = CPUFREQ_RELATION_L;
			if (old_freq > this_smartass->ideal_speed && !suspended) {
				int tmp = (ramp_down_step ? old_freq - ramp_down_step : this_smartass->ideal_speed);
				new_freq = (tmp < this_smartass->ideal_speed) ? this_smartass->ideal_speed : tmp;
			} else if (ramp_down_step) {
				new_freq = old_freq - ramp_down_step;
			} else {
				new_freq = policy->min;
			}
		} else {
			new_freq = old_freq;
		}

		if (target_freq(policy, this_smartass, new_freq, old_freq, relation)) {
			this_smartass->freq_change_time_in_idle =
				get_cpu_idle_time_us(cpu, &this_smartass->freq_change_time);
		}

		if (nr_running() < 1)
			del_timer(&this_smartass->timer);
		else
			reset_timer(cpu, this_smartass);
	}
}

#define SHOW_UINT(name) \
static ssize_t show_##name(struct kobject *kobj, struct attribute *attr, char *buf) \
{ return sprintf(buf, "%u\n", name); }

#define STORE_UINT(name) \
static ssize_t store_##name(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count) \
{ \
	unsigned long val; \
	if (kstrtoul(buf, 0, &val)) return -EINVAL; \
	name = val; \
	smartass_update_min_max_allcpus(); \
	return count; \
}

#define SHOW_ULONG(name) \
static ssize_t show_##name(struct kobject *kobj, struct attribute *attr, char *buf) \
{ return sprintf(buf, "%lu\n", name); }

#define STORE_ULONG(name) \
static ssize_t store_##name(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count) \
{ \
	unsigned long val; \
	if (kstrtoul(buf, 0, &val)) return -EINVAL; \
	name = val; \
	return count; \
}

SHOW_UINT(awake_ideal_freq);
STORE_UINT(awake_ideal_freq);
SHOW_UINT(sleep_ideal_freq);
STORE_UINT(sleep_ideal_freq);
SHOW_UINT(sleep_wakeup_freq);
STORE_UINT(sleep_wakeup_freq);
SHOW_ULONG(max_cpu_load);
STORE_ULONG(max_cpu_load);
SHOW_ULONG(min_cpu_load);
STORE_ULONG(min_cpu_load);
SHOW_UINT(ramp_up_step);
STORE_UINT(ramp_up_step);
SHOW_UINT(ramp_down_step);
STORE_UINT(ramp_down_step);
SHOW_ULONG(up_rate_us);
STORE_ULONG(up_rate_us);
SHOW_ULONG(down_rate_us);
STORE_ULONG(down_rate_us);
SHOW_UINT(sample_rate_jiffies);
STORE_UINT(sample_rate_jiffies);

#define SMARTASS_ATTR(_name) \
static struct global_attr _name##_attr = __ATTR(_name, 0644, show_##_name, store_##_name)

SMARTASS_ATTR(awake_ideal_freq);
SMARTASS_ATTR(sleep_ideal_freq);
SMARTASS_ATTR(sleep_wakeup_freq);
SMARTASS_ATTR(max_cpu_load);
SMARTASS_ATTR(min_cpu_load);
SMARTASS_ATTR(ramp_up_step);
SMARTASS_ATTR(ramp_down_step);
SMARTASS_ATTR(up_rate_us);
SMARTASS_ATTR(down_rate_us);
SMARTASS_ATTR(sample_rate_jiffies);

static struct attribute *smartass_attributes[] = {
	&awake_ideal_freq_attr.attr,
	&sleep_ideal_freq_attr.attr,
	&sleep_wakeup_freq_attr.attr,
	&max_cpu_load_attr.attr,
	&min_cpu_load_attr.attr,
	&ramp_up_step_attr.attr,
	&ramp_down_step_attr.attr,
	&up_rate_us_attr.attr,
	&down_rate_us_attr.attr,
	&sample_rate_jiffies_attr.attr,
	NULL,
};

static struct attribute_group smartass_attr_group = {
	.attrs = smartass_attributes,
	.name = "smartass",
};

static int cpufreq_governor_smartass(struct cpufreq_policy *new_policy,
				     unsigned int event)
{
	unsigned int cpu = new_policy->cpu;
	int rc;
	struct smartass_info_s *this_smartass = &per_cpu(smartass_info, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!new_policy->cur))
			return -EINVAL;

		this_smartass->cur_policy = new_policy;
		this_smartass->enable = 1;
		smartass_update_min_max(this_smartass, new_policy, suspended);
		this_smartass->freq_table = cpufreq_frequency_get_table(cpu);
		this_smartass->freq_change_time_in_idle =
			get_cpu_idle_time_us(cpu, &this_smartass->freq_change_time);

		if (atomic_inc_return(&active_count) == 1) {
			rc = sysfs_create_group(cpufreq_global_kobject, &smartass_attr_group);
			if (rc) {
				atomic_dec(&active_count);
				return rc;
			}
			pm_idle_old = pm_idle;
			pm_idle = cpufreq_idle;
		}

		reset_timer(cpu, this_smartass);
		break;

	case CPUFREQ_GOV_STOP:
		this_smartass->enable = 0;
		del_timer_sync(&this_smartass->timer);
		flush_workqueue(up_wq);
		flush_workqueue(down_wq);
		this_smartass->cur_policy = NULL;

		if (atomic_dec_return(&active_count) == 0) {
			pm_idle = pm_idle_old;
			sysfs_remove_group(cpufreq_global_kobject, &smartass_attr_group);
		}
		break;

	case CPUFREQ_GOV_LIMITS:
		smartass_update_min_max(this_smartass, new_policy, suspended);
		if (new_policy->max < new_policy->cur)
			__cpufreq_driver_target(new_policy, new_policy->max, CPUFREQ_RELATION_H);
		else if (new_policy->min > new_policy->cur)
			__cpufreq_driver_target(new_policy, new_policy->min, CPUFREQ_RELATION_L);
		break;
	}
	return 0;
}

static void smartass_suspend(int cpu, int suspend)
{
	struct smartass_info_s *this_smartass = &per_cpu(smartass_info, cpu);
	struct cpufreq_policy *policy = this_smartass->cur_policy;
	unsigned int new_freq;

	if (!this_smartass->enable || !policy)
		return;

	smartass_update_min_max(this_smartass, policy, suspend);
	if (!suspend) {
		new_freq = validate_freq(policy, sleep_wakeup_freq);
		__cpufreq_driver_target(policy, new_freq, CPUFREQ_RELATION_L);
	} else if (policy->cur > this_smartass->ideal_speed) {
		__cpufreq_driver_target(policy, this_smartass->ideal_speed, CPUFREQ_RELATION_H);
	}
}

static void smartass_early_suspend(struct early_suspend *h)
{
	unsigned int i;
	suspended = 1;
	for_each_online_cpu(i)
		smartass_suspend(i, 1);
}

static void smartass_late_resume(struct early_suspend *h)
{
	unsigned int i;
	suspended = 0;
	for_each_online_cpu(i)
		smartass_suspend(i, 0);
}

static struct early_suspend smartass_early_suspend_info = {
	.suspend = smartass_early_suspend,
	.resume = smartass_late_resume,
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1,
};

static int __init cpufreq_smartass_init(void)
{
	unsigned int i;
	struct smartass_info_s *this_smartass;

	awake_ideal_freq = DEFAULT_AWAKE_IDEAL_FREQ;
	sleep_ideal_freq = DEFAULT_SLEEP_IDEAL_FREQ;
	sleep_wakeup_freq = DEFAULT_SLEEP_WAKEUP_FREQ;
	max_cpu_load = DEFAULT_MAX_CPU_LOAD;
	min_cpu_load = DEFAULT_MIN_CPU_LOAD;
	ramp_up_step = DEFAULT_RAMP_UP_STEP;
	ramp_down_step = DEFAULT_RAMP_DOWN_STEP;
	up_rate_us = DEFAULT_UP_RATE_US;
	down_rate_us = DEFAULT_DOWN_RATE_US;
	sample_rate_jiffies = DEFAULT_SAMPLE_RATE_JIFFIES;

	spin_lock_init(&cpumask_lock);
	INIT_WORK(&freq_scale_work, cpufreq_smartass_freq_change);

	up_wq = alloc_workqueue("ksmartass_up", WQ_HIGHPRI, 1);
	down_wq = alloc_workqueue("ksmartass_down", 0, 1);
	if (!up_wq || !down_wq)
		return -ENOMEM;

	for_each_possible_cpu(i) {
		this_smartass = &per_cpu(smartass_info, i);
		init_timer_deferrable(&this_smartass->timer);
		this_smartass->timer.function = cpufreq_smartass_timer;
		this_smartass->timer.data = i;
	}

	register_early_suspend(&smartass_early_suspend_info);
	return cpufreq_register_governor(&cpufreq_gov_smartass2);
}

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_SMARTASS2
fs_initcall(cpufreq_smartass_init);
#else
module_init(cpufreq_smartass_init);
#endif

static void __exit cpufreq_smartass_exit(void)
{
	unregister_early_suspend(&smartass_early_suspend_info);
	cpufreq_unregister_governor(&cpufreq_gov_smartass2);
	destroy_workqueue(up_wq);
	destroy_workqueue(down_wq);
}

module_exit(cpufreq_smartass_exit);

MODULE_AUTHOR("Erasmux");
MODULE_DESCRIPTION("'cpufreq_smartass2' - A smartass cpufreq governor for Linux 3.4");
MODULE_LICENSE("GPL");
