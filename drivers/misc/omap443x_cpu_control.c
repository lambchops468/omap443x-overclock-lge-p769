/*
 * CPU frequency & voltage control for LG P769
 *
 * Copyright (C) 2014 Alexander Lam (lambchop468 (at) gmail.com)
 *
 * Based on
 *  - CPU frequency & voltage control for Motorola 3.0.8 kernel by Project Lense
 *	
 * License: GNU GPLv3
 * <http://www.gnu.org/licenses/gpl-3.0.html>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <plat/omap_device.h>
#include <plat/common.h>

#include "../../../arch/arm/mach-omap2/voltage.h"
#include "symsearch/symsearch.h"

#define MPU_MAX_UVOLT 1415000
#define MPU_MIN_UVOLT 830000

#define GPU_MAX_UVOLT 830000
/* Note: OMAP4460 uses 1250000 to reach 384 MHz. See
 * OMAP4460_VP_CORE_VLIMITTO_VDDMAX
 *
 * Motorola's sources indicate 1200000 for "OMAP4"
 * (those sources are probably for OMAP4430 only)
 */
#define GPU_MIN_UVOLT 1250000

/* arch/arm/mach-omap2/omap2plus-cpufreq.c */
static struct mutex *omap_cpufreq_lock_p = NULL;
static unsigned int *max_freq_p = NULL;

/* drivers/base/power/opp.c */
SYMSEARCH_DECLARE_FUNCTION_STATIC(int, opp_add_s, struct device *dev,
		unsigned long freq, unsigned long u_volt);
SYMSEARCH_DECLARE_FUNCTION_STATIC(int, opp_get_opp_count_s, struct device *dev);
SYMSEARCH_DECLARE_FUNCTION_STATIC(int, opp_enable_s, struct device *dev,
		unsigned long freq);
SYMSEARCH_DECLARE_FUNCTION_STATIC(struct opp *, opp_find_freq_floor_s,
		struct device *dev, unsigned long *freq);
SYMSEARCH_DECLARE_FUNCTION_STATIC(struct opp *, opp_find_freq_ceil_s,
		struct device *dev, unsigned long *freq);

/* arch/arm/mach-omap2/voltage.c */
SYMSEARCH_DECLARE_FUNCTION_STATIC(struct voltagedomain *, voltdm_lookup_s,
		char *name);

/* drivers/cpufreq/cpufreq.c */
static struct mutex *cpufreq_governor_mutex_p;
SYMSEARCH_DECLARE_FUNCTION_STATIC(int, __cpufreq_set_policy_s,
		struct cpufreq_policy *data, struct cpufreq_policy *policy);
SYMSEARCH_DECLARE_FUNCTION_STATIC(int, lock_policy_rwsem_write_s, int cpu);
SYMSEARCH_DECLARE_FUNCTION_STATIC(void, unlock_policy_rwsem_write_s, int cpu);
SYMSEARCH_DECLARE_FUNCTION_STATIC(struct cpufreq_governor*, __find_governor_s,
		const char *str_governor);

/* arch/arm/mach-omap2/dvfs.c */
static struct mutex *omap_dvfs_lock_p;


struct opp {
	char *hwmod_name;
	char *voltdm_name;
	char *clk_name;

	unsigned long rate;
	unsigned long u_volt;

	bool default_available;
};

struct opp_table {
	int index;
	unsigned long rate;
	unsigned long u_volt;
	struct opp *opp;
};

static struct cpufreq_frequency_table *freq_table;
static struct cpufreq_policy *policy;
static struct device *mpu_dev, *gpu_dev;
static struct omap_vdd_info *mpu_vdd, *gpu_vdd;
static struct clk *mpu_clk, *gpu_clk;
static struct opp_table *mpu_def_ft, *gpu_def_ft;
static int mpu_opp_count, gpu_opp_count;

static char prev_governor[CPUFREQ_NAME_LEN];
static char performance_governor[CPUFREQ_NAME_LEN] = "performance";

static struct kobject *cpucontrol_kobj;

/* A replacement for cpufreq_parse_governor().
 * For some reason the above function does not exist in the symbol tables.
 * (it was probably inlined...)
 */
static int cpufreq_get_governor(char *str_governor, unsigned int *policy,
				struct cpufreq_governor **governor) {
	struct cpufreq_governor *t;
	int err = -EINVAL;

	mutex_lock(cpufreq_governor_mutex_p);

	t = __find_governor_s(str_governor);

	if (t == NULL) {
		int ret;

		mutex_unlock(cpufreq_governor_mutex_p);
		ret = request_module("cpufreq_%s", str_governor);
		mutex_lock(cpufreq_governor_mutex_p);

		if (ret == 0)
			t = __find_governor_s(str_governor);
	}

	if (t != NULL) {
		*governor = t;
		err = 0;
	}

	mutex_unlock(cpufreq_governor_mutex_p);

	return err;
}

static int set_cpufreq_policy(char str_governor[CPUFREQ_NAME_LEN],
		unsigned int min_freq,
		unsigned int max_freq) {
	unsigned int ret = -EINVAL;
	struct cpufreq_policy new_policy;
	if (!policy)
		return ret;

	ret = cpufreq_get_policy(&new_policy, policy->cpu);
	if (ret)
		return ret;

	if (cpufreq_get_governor(str_governor, &new_policy.policy,
						&new_policy.governor))
		return ret;

	new_policy.min = min_freq;
	new_policy.max = max_freq;

	ret = __cpufreq_set_policy_s(policy, &new_policy);

	policy->user_policy.policy = policy->policy;
	policy->user_policy.governor = policy->governor;

	return ret;
}

static int set_cpufreq_governor(char str_governor[CPUFREQ_NAME_LEN]) {
	return set_cpufreq_policy(str_governor, policy->min, policy->max);
}

static int prepare_mpu_opp_modify(void) {
	int ret = 0;

	/* Lock the core cpufreq policy. The same policy object and lock exists
	 * for CPU0 and CPU1 because they are both related */
	if(lock_policy_rwsem_write_s(policy->cpu) < 0) {
		ret = -ENODEV;
		return ret;
	}


	/* change governor to performance. When this returns, we will have
	 * transistioned to the fastest frequency. Since the performance
	 * governor never calls the cpufreq driver's target() function ever
	 * again, we are safe from frequency switches caused by the governor.
	 * The other source of frequency switches is omap_thermal_throttle(),
	 * but that is protected by omap_cpufreq_lock. */
	pr_info("cpu-control : Current cpufreq gov : %s\n", policy->governor->name);
	strncpy(prev_governor, policy->governor->name, CPUFREQ_NAME_LEN);
	if (strnicmp(policy->governor->name, performance_governor, CPUFREQ_NAME_LEN)) {
		pr_info("cpu-control : Change cpufreq gov : %s\n", policy->governor->name);
		ret = set_cpufreq_governor(performance_governor);
		if (ret) {
			pr_err("cpu-control : Could not set cpufreq governor\n");
			goto out_err;
		}
	}

	/* Stop the omap2plus-cpufreq driver from trying to change frequency -
	 * For example, to stop omap_thermal_throttle(), which is not called
	 * from the cpufreq framework. */
	mutex_lock(omap_cpufreq_lock_p);
	/* Take this lock to prevent dvfs from attempting to use the opp
	 * being modified (this might occur during cross-domain changes)  */
	mutex_lock(omap_dvfs_lock_p);

	return 0;

out_err:
	unlock_policy_rwsem_write_s(policy->cpu);
	return ret;
}

static int finish_mpu_opp_modify(void) {
	int ret;

	unsigned int min_freq_new = freq_table[0].frequency;
	unsigned int max_freq_new = freq_table[mpu_opp_count-1].frequency;

	mutex_unlock(omap_dvfs_lock_p);

	/* Update omap2plus-cpufreq */
	*max_freq_p = max_freq_new;

	mutex_unlock(omap_cpufreq_lock_p);

	policy->cpuinfo.min_freq = policy->user_policy.min = min_freq_new;
	policy->cpuinfo.max_freq = policy->user_policy.max = max_freq_new;

	pr_info("cpu_control : Reset cpufreq gov : %s\n", policy->governor->name);
	/* This will cause the governor, even if the same governor,
	 * to notice the new frequency limits */
	ret = set_cpufreq_policy(prev_governor,
			freq_table[0].frequency,
			freq_table[mpu_opp_count-1].frequency);

	if (ret) {
		pr_err("cpu-control : Could not set cpufreq governor\n");
	}

	/* change governor back to whatever it was before we started */
	/* set appropriate min/max too */
	unlock_policy_rwsem_write_s(policy->cpu);

	return ret;
}

static void prepare_gpu_opp_modify(void) {
	mutex_lock(omap_dvfs_lock_p);
}

static void finish_gpu_opp_modify(void) {
	mutex_unlock(omap_dvfs_lock_p);
}

/*
 * Freq in Hz
 * Volt in uV
 */
static void set_one_mpu_opp(unsigned int index, unsigned int freq, unsigned int volt) {
	freq_table[index].frequency = freq/1000;
	mpu_vdd->volt_data[index].volt_nominal = volt;
	mpu_vdd->dep_vdd_info[0].dep_table[index].main_vdd_volt = volt;
	mpu_def_ft[index].opp->u_volt = volt;
	mpu_def_ft[index].opp->rate = freq;
}

static void set_one_gpu_opp(unsigned int index, unsigned int freq, unsigned int volt) {
	gpu_vdd->volt_data[index].volt_nominal = volt;
	gpu_def_ft[index].opp->u_volt = volt;
	gpu_def_ft[index].opp->rate = freq;
}

/*  sysfs */
static ssize_t cur_freq_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf) {
	return scnprintf(buf, PAGE_SIZE, "CPU : %lu Mhz\nGPU : %lu Mhz\n",
			clk_get_rate(mpu_clk)/1000000,
			clk_get_rate(gpu_clk)/1000000);
}

static ssize_t cpu_default_opp_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf) {
	int i, ret;

	ret = scnprintf(buf, PAGE_SIZE, "Id\tFreq(mHz)\tVolt(mV)\n");
	for(i = 0; i < mpu_opp_count; i++) {
		ret += scnprintf(buf+ret, PAGE_SIZE-ret, "%d\t%lu\t\t%lu\n",
			mpu_def_ft[i].index,
			mpu_def_ft[i].rate/1000000,
			mpu_def_ft[i].u_volt/1000);
	}

	return ret;
}

static ssize_t gpu_default_opp_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf) {
	int i, ret;

	ret = scnprintf(buf, PAGE_SIZE, "Id\tFreq(mHz)\tVolt(mV)\n");
	for(i = 0; i < gpu_opp_count; i++) {
		ret += scnprintf(buf+ret, PAGE_SIZE-ret, "%d\t%lu\t\t%lu\n",
			gpu_def_ft[i].index,
			gpu_def_ft[i].rate/1000000,
			gpu_def_ft[i].u_volt/1000);
	}

	return ret;
}

static ssize_t cpu_cur_opp_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf) {
	int i, ret;

	ret = scnprintf(buf, PAGE_SIZE, "Id\tFreq(mHz)\tVolt(mV)\n");
	for(i = 0; i < mpu_opp_count; i++) {
		ret += scnprintf(buf+ret, PAGE_SIZE-ret, "%d\t%lu\t%lu\n",
			mpu_def_ft[i].index,
			mpu_def_ft[i].opp->rate/1000000,
			mpu_def_ft[i].opp->u_volt/1000);
	}

	return ret;
}

static ssize_t gpu_cur_opp_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf) {
	int i, ret;

	ret = scnprintf(buf, PAGE_SIZE, "Id\tFreq(mHz)\tVolt(mV)\n");
	for(i = 0; i < gpu_opp_count; i++) {
		ret += scnprintf(buf+ret, PAGE_SIZE-ret, "%d\t%lu\t%lu\n",
			gpu_def_ft[i].index,
			gpu_def_ft[i].opp->rate/1000000,
			gpu_def_ft[i].opp->u_volt/1000);
	}

	return ret;
}

static ssize_t cpu_tweak_opp_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count) {
	unsigned int id;
	unsigned int freq; //in KHz
	unsigned int volt; //in mV
	int ret;

	if(sscanf(buf, "%u %u %u", &id, &freq, &volt) != 3) {
		return -EINVAL;
	}

	freq = freq * 1000000; /* convert from MHz to Hz */

	if (id > mpu_opp_count-1) {
		pr_err("cpu-control : wrong cpu opp id @ %u", id);
		return -EINVAL;
	}

	if (id > 0 && freq_table[id-1].frequency >= freq/1000) {
		pr_err("cpu-control : Frequency is not above previous OPP's frequency");
		return -EINVAL;
	}

	if (id < mpu_opp_count-1 && freq_table[id+1].frequency <= freq/1000) {
		pr_err("cpu-control : Frequency is not below next OPP's frequency");
		return -EINVAL;
	}

	freq = clk_round_rate(mpu_clk, freq);
	if (freq <= 0) {
		pr_err("cpu-control : Frequency could not be rounded");
		return -EINVAL;
	}

	if (volt > MPU_MAX_UVOLT/1000) {
		pr_info("cpu-control : Too high voltage, limiting to %u", MPU_MAX_UVOLT/1000);
		volt = MPU_MAX_UVOLT/1000;
	}

	if (volt < MPU_MIN_UVOLT/1000) {
		pr_info("cpu-control : Too low voltage, limiting to %u", MPU_MIN_UVOLT/1000);
		volt = MPU_MIN_UVOLT/1000;
	}

	pr_info("cpu-control : Change CPU operating point : %u %u Mhz %u mV\n",
			id, freq/1000000, volt);

	ret = prepare_mpu_opp_modify();
	if (ret)
		return ret;

	set_one_mpu_opp(id, freq, volt*1000);

	ret = finish_mpu_opp_modify();
	if (ret)
		return ret;

	return count;
}

static ssize_t gpu_tweak_opp_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count) {
	unsigned int id;
	unsigned int freq; //in KHz
	unsigned int volt; //in mV

	if(sscanf(buf, "%u %u %u", &id, &freq, &volt) != 3) {
		return -EINVAL;
	}

	freq = freq * 1000000; /* convert from MHz to Hz */

	if (id > gpu_opp_count-1) {
		pr_err("cpu-control : wrong cpu opp id @ %u", id);
		return -EINVAL;
	}

	if (id > 0 && freq_table[id-1].frequency >= freq/1000) {
		pr_err("cpu-control : Frequency is not above previous OPP's frequency");
		return -EINVAL;
	}

	if (id < gpu_opp_count-1 && freq_table[id+1].frequency <= freq/1000) {
		pr_err("cpu-control : Frequency is not below next OPP's frequency");
		return -EINVAL;
	}

	freq = clk_round_rate(gpu_clk, freq);
	if (freq <= 0) {
		pr_err("cpu-control : Frequency could not be rounded");
		return -EINVAL;
	}

	if (volt > GPU_MAX_UVOLT/1000) {
		pr_info("cpu-control : Too high voltage, limiting to %u", GPU_MAX_UVOLT/1000);
		volt = GPU_MAX_UVOLT/1000;
	}

	if (volt < GPU_MIN_UVOLT/1000) {
		pr_info("cpu-control : Too low voltage, limiting to %u", GPU_MIN_UVOLT/1000);
		volt = GPU_MIN_UVOLT/1000;
	}

	pr_info("cpu-control : Change GPU operating point : %u %u Mhz %u mV\n",
			id, freq/1000000, volt);

	prepare_gpu_opp_modify();
	set_one_gpu_opp(id, freq, volt*1000);
	finish_mpu_opp_modify();

	return count;
}

static struct kobj_attribute cur_freq_attr =
	__ATTR(cur_freq, S_IRUGO, cur_freq_show, NULL);
static struct kobj_attribute cpu_cur_opp_attr =
	__ATTR(cpu_cur_opp, S_IRUGO, cpu_cur_opp_show, NULL);
static struct kobj_attribute cpu_default_opp_attr =
	__ATTR(cpu_default_opp, S_IRUGO, cpu_default_opp_show, NULL);
static struct kobj_attribute cpu_tweak_opp_attr =
	__ATTR(cpu_tweak_opp, S_IWUSR, NULL, cpu_tweak_opp_store);
static struct kobj_attribute gpu_cur_opp_attr =
	__ATTR(gpu_cur_opp, S_IRUGO, gpu_cur_opp_show, NULL);
static struct kobj_attribute gpu_default_opp_attr =
	__ATTR(gpu_default_opp, S_IRUGO, gpu_default_opp_show, NULL);
static struct kobj_attribute gpu_tweak_opp_attr =
	__ATTR(gpu_tweak_opp, S_IWUSR, NULL, gpu_tweak_opp_store);

static struct attribute *attrs[] = {
	&cur_freq_attr.attr,
	&cpu_cur_opp_attr.attr,
	&cpu_default_opp_attr.attr,
	&cpu_tweak_opp_attr.attr,
	&gpu_cur_opp_attr.attr,
	&gpu_default_opp_attr.attr,
	&gpu_tweak_opp_attr.attr,
	NULL
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

/* sysfs end */

static int __init populate_def_mpu_freq_table(void) {
	unsigned long freq;
	int ret = 0;
	int i;

	/* half hearted attempt at being 'correct' - the opp interface asks us
	 * to rcu_read_lock() before using opp_find_*. It also requires that we
	 * hold the dev_opp_list_lock when modifying the list, and use appropriate
	 * rcu functions., but this is a moot point because this driver holds
	 * pointers to struct opp objects.
	 * This is technically illegal because the opp interface uses RCU, so
	 * if an object is modified, it is actually a new copy in a new
	 * location. So... when modifying struct opp objects, don't bother with
	 * locking at all.
	 *
	 * Note that in normal operation, nobody updates the MPU's opp list.
	 *
	 * see drivers/base/power/opp.c and Documentation/power/opp.txt for
	 * more details.
	 */
	rcu_read_lock();
	for(i = 0; i<mpu_opp_count; i++) {
		mpu_def_ft[i].index = i;

		freq = (freq_table[i].frequency-1000) * 1000;

		mpu_def_ft[i].opp = opp_find_freq_ceil_s(mpu_dev, &freq);
		if (IS_ERR(mpu_def_ft[i].opp)) {
			pr_err("cpu-control: %s: Unable to retrieve OPP\n", __func__);
			ret = -EINVAL;
			goto out;
		}
		
		mpu_def_ft[i].rate = mpu_def_ft[i].opp->rate;
		mpu_def_ft[i].u_volt = mpu_def_ft[i].opp->u_volt;

		pr_info("CPU Map %d : %lu Mhz : %lu mV\n", mpu_def_ft[i].index,
				mpu_def_ft[i].rate/1000000, mpu_def_ft[i].u_volt/1000);
	}
out:
	rcu_read_unlock();

	return ret;
}

static void __exit restore_def_mpu_freq_table(void) {
	int i;
	for(i = 0; i < mpu_opp_count; i++) {
		set_one_mpu_opp(i, mpu_def_ft[i].rate, mpu_def_ft[i].u_volt);
	}
}

static int __init populate_def_gpu_freq_table(void) {
	unsigned long freq = 0;
	int ret = 0;
	int i;

	rcu_read_lock();
	for(i = 0; i<gpu_opp_count; i++) {
		gpu_def_ft[i].index = i;

		gpu_def_ft[i].opp = opp_find_freq_ceil_s(gpu_dev, &freq);
		if (IS_ERR(gpu_def_ft[i].opp)) {
			pr_err("cpu-control: %s: Unable to retrieve OPP\n", __func__);
			ret = -EINVAL;
			goto out;
		}
		
		gpu_def_ft[i].rate = gpu_def_ft[i].opp->rate;
		gpu_def_ft[i].u_volt = gpu_def_ft[i].opp->u_volt;

		freq += 1;

		pr_info("GPU Map %d : %lu Mhz : %lu mV\n", gpu_def_ft[i].index,
				gpu_def_ft[i].rate/1000000, gpu_def_ft[i].u_volt/1000);
	}
out:
	rcu_read_unlock();

	return ret;
}

static void __exit restore_def_gpu_freq_table(void) {
	int i;
	for(i = 0; i < gpu_opp_count; i++) {
		set_one_gpu_opp(i, gpu_def_ft[i].rate, gpu_def_ft[i].u_volt);
	}
}


static int __init cpu_control_init(void) {
	struct opp *gpu_opp;
	struct voltagedomain *mpu_voltdm, *gpu_voltdm;
	int ret;
	unsigned long freq = ULONG_MAX;

	pr_info("cpu-control : Hello world!\n");

	if (!cpu_is_omap443x()) {
		pr_err("cpu-control: CPU is not OMAP443x\n");
		return 0;
	}

	/* arch/arm/mach-omap2/omap2plus-cpufreq.c */
	/* freq_table and max_freq are popular symbol names in the kernel!
	 * But we get lucky and the kernel only has one example of each
	 * symbol (see System.map after compilation) */
	SYMSEARCH_BIND_POINTER_TO(omap443x_cpu_control, struct mutex*, omap_cpufreq_lock, omap_cpufreq_lock_p);
	SYMSEARCH_BIND_POINTER_TO(omap_temp_sensor, unsigned int*, max_freq, max_freq_p);

	/* drivers/base/power/opp.c */
	SYMSEARCH_BIND_FUNCTION_TO(omap443x_cpu_control, opp_add, opp_add_s);
	SYMSEARCH_BIND_FUNCTION_TO(omap443x_cpu_control, opp_enable, opp_enable_s);
	SYMSEARCH_BIND_FUNCTION_TO(omap443x_cpu_control, opp_get_opp_count, opp_get_opp_count_s);
	SYMSEARCH_BIND_FUNCTION_TO(omap443x_cpu_control, opp_find_freq_floor, opp_find_freq_floor_s);
	SYMSEARCH_BIND_FUNCTION_TO(omap443x_cpu_control, opp_find_freq_ceil, opp_find_freq_ceil_s);

	/* arch/arm/mach-omap2/voltage.c */
	SYMSEARCH_BIND_FUNCTION_TO(omap443x_cpu_control, voltdm_lookup, voltdm_lookup_s);

	/* drivers/cpufreq/cpufreq.c */
	SYMSEARCH_BIND_POINTER_TO(omap443x_cpu_control, struct mutex*, cpufreq_governor_mutex, cpufreq_governor_mutex_p);
	SYMSEARCH_BIND_FUNCTION_TO(omap443x_cpu_control, lock_policy_rwsem_write, lock_policy_rwsem_write_s);
	SYMSEARCH_BIND_FUNCTION_TO(omap443x_cpu_control, unlock_policy_rwsem_write, unlock_policy_rwsem_write_s);
	SYMSEARCH_BIND_FUNCTION_TO(omap443x_cpu_control, __cpufreq_set_policy, __cpufreq_set_policy_s);
	SYMSEARCH_BIND_FUNCTION_TO(omap443x_cpu_control, __find_governor, __find_governor_s);

	/* arch/arm/mach-omap2/dvfs.c */
	SYMSEARCH_BIND_POINTER_TO(omap443x_cpu_control, struct mutex*, omap_dvfs_lock, omap_dvfs_lock_p);

	/* cpufreq_cpu_get() should only be done on CPU0 (the boot cpu). For other
	 * CPUs, the policy is destroyed/created on cpu hotplug (which happens during
	 * suspend). cpufreq_cpu_get() gets the omap2plus-cpufreq module  */
	policy = cpufreq_cpu_get(0);
	if (!policy) {
		pr_err("%s:No cpufreq driver\n", __func__);
		ret = -EINVAL;
		goto err_out;
	}

	freq_table = cpufreq_frequency_get_table(0);
	if (!freq_table) {
		pr_err("%s:No cpufreq driver\n", __func__);
		ret = -EINVAL;
		goto err_cpu_put;
	}

	mpu_clk = clk_get(NULL, "dpll_mpu_ck");
	if (IS_ERR(mpu_clk)) {
		pr_err("%s:Unable to get mpu_clk\n", __func__);
		ret = -ENODEV;
		goto err_cpu_put;
	}

	mpu_voltdm = voltdm_lookup_s("mpu");
	if (!mpu_voltdm || IS_ERR(mpu_voltdm)) {
		pr_err("cpu-control: %s: Unable to get mpu voltage domain\n", __func__);
		ret = -ENODEV;
		goto err_cpu_put;
	}
	mpu_vdd = mpu_voltdm->vdd;

	mpu_dev = omap2_get_mpuss_device();
	if (!mpu_dev || IS_ERR(mpu_dev)) {
		pr_err("cpu-control: %s: Unable to get mpu device\n", __func__);
		ret = -ENODEV;
		goto err_cpu_put;
	}

	mpu_opp_count = opp_get_opp_count_s(mpu_dev);
	if (mpu_opp_count <= 0) {
		pr_err("cpu-control: %s: Could not get opp count for mpu\n", __func__);
		ret = -EINVAL;
		goto err_cpu_put;
	}

	mpu_def_ft = kzalloc(sizeof(struct opp_table) * (mpu_opp_count), GFP_KERNEL);
	if (!mpu_def_ft) {
		pr_err("cpu-control: %s: Unable to allocate memory\n", __func__);
		ret = -ENOMEM;
		goto err_cpu_put;
	}

	ret = populate_def_mpu_freq_table();
	if (ret)
		goto err_free_mpu_def_ft;

	gpu_clk = clk_get(NULL, "gpu_fck");
	if (IS_ERR(gpu_clk)) {
		pr_err("%s:Unable to get gpu_clk\n", __func__);
		ret = -ENODEV;
		goto err_free_mpu_def_ft;
	}

	/* GPU is on core voltage domain */
	gpu_voltdm = voltdm_lookup_s("core");
	if (!gpu_voltdm || IS_ERR(gpu_voltdm)) {
		pr_err("cpu-control: %s: Unable to get gpu voltage domain\n", __func__);
		ret = -ENODEV;
		goto err_free_mpu_def_ft;
	}
	gpu_vdd = gpu_voltdm->vdd;

	gpu_dev = omap_hwmod_name_get_dev("gpu");
	if (!gpu_dev || IS_ERR(gpu_dev)) {
		pr_err("cpu-control: %s: Unable to get gpu device\n", __func__);
		ret = -ENODEV;
		goto err_free_mpu_def_ft;
	}

	gpu_opp_count = opp_get_opp_count_s(gpu_dev);
	if (gpu_opp_count <= 0) {
		pr_err("cpu-control: %s: Could not get opp count for gpu\n", __func__);
		ret = -EINVAL;
		goto err_free_mpu_def_ft;
	}

	gpu_def_ft = kzalloc(sizeof(struct opp_table) * (gpu_opp_count), GFP_KERNEL);
	if (!gpu_def_ft) {
		pr_err("cpu-control: %s: Unable to allocate memory\n", __func__);
		ret = -ENOMEM;
		goto err_free_mpu_def_ft;
	}

	ret = populate_def_gpu_freq_table();
	if (ret)
		goto err_free_gpu_def_ft;

	/* Create a new sysfs directory under /sys/devices/system/cpu */
	cpucontrol_kobj = kobject_create_and_add("cpucontrol", &cpu_sysdev_class.kset.kobj);
	if (!cpucontrol_kobj) {
		pr_err("cpu-control: %s: Unable to allocate new kobject\n", __func__);
		goto err_free_gpu_def_ft;
	}

	ret = sysfs_create_group(cpucontrol_kobj, &attr_group);
	if (ret) {
		pr_err("cpu-control: %s: Unable to create sysfs attributes\n", __func__);
		goto err_put_kobj;
	}

	gpu_opp = opp_find_freq_floor_s(gpu_dev, &freq);
	pr_info("cpu-control : GPU Default max value : %lu mV : %lu\n", gpu_opp->u_volt/1000, gpu_opp->rate/1000);

	return 0;

err_put_kobj:
	kobject_put(cpucontrol_kobj);
err_free_gpu_def_ft:
	kfree(gpu_def_ft);
err_free_mpu_def_ft:
	kfree(mpu_def_ft);
err_cpu_put:
	cpufreq_cpu_put(policy);
err_out:
	return ret;
}

static void __exit cpu_control_exit(void){
	int ret;

	pr_info("cpu_control : Reset opp table to default.\n");

	/* This will also remove the sysfs entries */
	kobject_put(cpucontrol_kobj);

	prepare_gpu_opp_modify();
	restore_def_gpu_freq_table();
	finish_gpu_opp_modify();


	ret = prepare_mpu_opp_modify();
	if (ret)
		goto out;

	restore_def_mpu_freq_table();

	finish_mpu_opp_modify();

out:
	kfree(gpu_def_ft);
	kfree(mpu_def_ft);

	cpufreq_cpu_put(policy);

	pr_info("cpu_control : Goodbye\n");
}

module_init(cpu_control_init);
module_exit(cpu_control_exit);

MODULE_AUTHOR("Alexander Lam (lambchop468 (at) gmail.com)");
MODULE_DESCRIPTION("cpu_control - Tweak cpu/gpu voltage & frequency for OMAP443x.");
MODULE_LICENSE("GPL");
