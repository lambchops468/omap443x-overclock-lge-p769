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
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/clk.h>
#include <linux/kallsyms.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <asm/uaccess.h>
#include <plat/omap-pm.h>
#include <plat/omap_device.h>
#include <plat/common.h>
#include "../../../arch/arm/mach-omap2/omap_opp_data.h"
#include "../../../arch/arm/mach-omap2/voltage.h"
#include "../symsearch/symsearch.h"

#define MPU_MAX_UVOLT 1415000
#define MPU_MIN_UVOLT 830000

/* arch/arm/mach-omap2/omap2plus-cpufreq.c */
static struct mutex *omap_cpufreq_lock_p = NULL;

/* drivers/base/power/opp.c */
SYMSEARCH_DECLARE_FUNCTION_STATIC(int, opp_add_s, struct device *dev, unsigned long freq, unsigned long u_volt);
SYMSEARCH_DECLARE_FUNCTION_STATIC(int, opp_get_opp_count_s, struct device *dev);
SYMSEARCH_DECLARE_FUNCTION_STATIC(int, opp_enable_s, struct device *dev, unsigned long freq);
SYMSEARCH_DECLARE_FUNCTION_STATIC(struct opp *, opp_find_freq_floor_s, struct device *dev, unsigned long *freq);
SYMSEARCH_DECLARE_FUNCTION_STATIC(struct opp *, opp_find_freq_ceil_s, struct device *dev, unsigned long *freq);
/* arch/arm/mach-omap2/voltage.c */
SYMSEARCH_DECLARE_FUNCTION_STATIC(struct voltagedomain *, voltdm_lookup_s, char *name);
/* drivers/cpufreq/cpufreq.c */
SYMSEARCH_DECLARE_FUNCTION_STATIC(struct cpufreq_governor *, __find_governor_s, const char *str_governor);
SYMSEARCH_DECLARE_FUNCTION_STATIC(int, __cpufreq_set_policy_s, struct cpufreq_policy *data, struct cpufreq_policy *policy);

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
static struct omap_vdd_info *mpu_vdd;
static struct clk *mpu_clk, *gpu_clk;
extern struct mutex omap_dvfs_lock;
static struct opp_table *def_ft;

static int mpu_opp_count;
static char prev_governor[CPUFREQ_NAME_LEN];
static char performance_governor[CPUFREQ_NAME_LEN] = "performance";

#define BUF_SIZE PAGE_SIZE
static char *buf;

static struct *kobject cpucontrol_kobj;

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

	if (cpufreq_parse_governor(str_governor, &new_policy.policy,
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

static int prepare_opp_modify() {
	/* Lock the core cpufreq policy. The same policy object and lock exists
	 * for CPU0 and CPU1 because they are both related */
	lock_policy_rwsem_write(policy->cpu);

	/* change governor to performance. When this returns, we will have
	 * transistioned to the fastest frequency. Since the performance
	 * governor never calls the cpufreq driver's target() function ever
	 * again, we are safe from frequency switches caused by the governor.
	 * The other source of frequency switches is omap_thermal_throttle(),
	 * but that is protected by omap_cpufreq_lock. */
	pr_info("cpu-control : Current cpufreq gov : %s\n", policy->governor->name);
	strncpy(prev_governor, policy->governor->name, CPUFREQ_NAME_LEN);
	if (strnicmp(policy->governor->name, performance_governor, CPUFREQ_NAME_LEN)) {
		set_cpufreq_governor(performance_governor);
		pr_info("cpu-control : Change cpufreq gov : %s\n", policy->governor->name);
	}

	/* Stop the omap2plus-cpufreq driver from trying to change frequency -
	 * For example, to stop omap_thermal_throttle(), which is not called
	 * from the cpufreq framework. */
	mutex_lock(omap_cpufreq_lock_p);
	/* Take this lock to prevent dvfs from attempting to use the opp
	 * being modified (this might occur during cross-domain changes)  */
	mutex_lock(&omap_dvfs_lock);
}

static int finish_opp_modify() {
	unsigned int min_freq_new = freq_table[0].frequency;
	unsigned int max_freq_new = freq_table[mpu_opp_count-1].frequency;

	mutex_unlock(&omap_dvfs_lock);

	/* Update omap2plus-cpufreq */
	max_freq = max_freq_new;

	mutex_unlock(omap_cpufreq_lock_p);

	policy->cpuinfo.min_freq = policy->user_policy.min = min_freq_new;
	policy->cpuinfo.max_freq = policy->user_policy.max = max_freq_new;

	/* This will cause the governor, even if the same governor,
	 * to notice the new frequency limits */
	set_cpufreq_policy(prev_governor,
			freq_table[0].frequency,
			freq_table[mpu_opp_count-1].frequency);
	pr_info("cpu_control : Reset cpufreq gov : %s\n", policy->governor->name);

	/* change governor back to whatever it was before we started */
	/* set appropriate min/max too */
	unlock_policy_rwsem_write(policy->cpu);
}

/*
 * Freq in Hz
 * Volt in uV
 */
static int set_one_opp(unsigned int index, unsigned int freq, unsigned int volt) {
	freq_table[index].frequency = freq/1000;
	mpu_vdd->volt_data[index].volt_nominal = volt;
	// this is wrong. wrong. wrong. all. wrong.
	mpu_vdd->dep_vdd_info[0].dep_table[index].main_vdd_volt = volt;
	def_ft[index].opp->u_volt = volt;
	def_ft[index].opp->rate = freq;
}

/*  sysfs */
static int cur_freq_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf) {
	return scnprintf(buf, PAGE_SIZE, "CPU : %lu Mhz\nGPU : %lu Mhz\n",
			clk_get_rate(mpu_clk)/1000000,
			clk_get_rate(gpu_clk)/1000000);
}

static int cpu_def_opp_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf) {
	int i, ret;

	ret = scnprintf(buffer, PAGE_SIZE, "Id\tFreq(mHz)\tVolt(mV)\n");
	for(i = 0; i < mpu_opp_count; i++) {
		ret += scnprintf(buffer+ret, PAGE_SIZE-ret, "%d\t%lu\t%lu\n",
			def_ft[i].index,
			def_ft[i].rate/1000000,
			def_ft[i].u_volt/1000);
	}

	return ret;
}

static int cpu_cur_opp_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf) {
	int i, ret;

	ret = scnprintf(buffer, PAGE_SIZE, "Id\tFreq(mHz)\tVolt(mV)\n");
	for(i = 0; i < mpu_opp_count; i++) {
		ret += scnprintf(buffer+ret, PAGE_SIZE-ret, "%d\t%lu\t%lu\n",
			def_ft[i].index,
			def_ft[i].opp->rate/1000000,
			def_ft[i].opp->u_volt/1000);
	}

	return ret;
}

static int cpu_tweak_opp_store(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf, size_t count) {
	unsigned int id;
	unsigned int freq; //in KHz
	unsigned int volt; //in mV

	if(sscanf(buf, "%u %u %u", &id, &freq, &volt) != 3) {
		return -EINVAL
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
		pr_info("cpu-control : Too high voltage, limiting to %lu", MPU_MAX_UVOLT/1000);
		volt = MPU_MAX_UVOLT/1000;
	}

	if (volt < MPU_MIN_UVOLT/1000) {
		pr_info("cpu-control : Too low voltage, limiting to %lu", MPU_MIN_UVOLT/1000);
		volt = MPU_MIN_UVOLT/1000;
	}

	pr_info("cpu-control : Change operating point : %lu %lu Mhz %lu mV\n",
			id, freq/1000000, volt);

	prepare_opp_modify();
	set_one_opp(id, freq, volt*1000);
	finish_opp_modify();

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

static struct attribute *attrs[] = {
	&cur_freq_attr.attr,
	&cpu_cur_opp_attr.attr,
	&cpu_default_opp.attr,
	&cpu_tweak_opp.attr,
	NULL
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

/* sysfs end */

static int __init populate_def_freq_table() {
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
		def_ft[i].index = i;

		freq = (freq_table[i].frequency-1000) * 1000;

		def_ft[i].opp = opp_find_freq_ceil_s(mpu_dev, &freq);
		if (IS_ERR(def_ft[i].opp)) {
			pr_err("cpu-control: %s: Unable to retrieve OPP\n", __func__);
			ret = -EINVAL;
			goto out;
		}
		
		def_ft[i].rate = def_ft[i].opp->rate;
		def_ft[i].u_volt = def_ft[i].opp->u_volt;

		pr_info("Map %d : %lu Mhz : %lu mV\n", def_ft[i].index,
				def_ft[i].rate/1000000, def_ft[i].u_volt/1000);
	}
out:
	rcu_read_unlock();

	return ret;
}

static int __exit restore_def_freq_table() {
	for(i = 0; i<opp_count; i++) {
		set_one_opp(i, def_ft[i].rate, def_ft[i].u_volt);
	}
}

static int __init cpu_control_init(void) {
	struct proc_dir_entry *proc_entry;
	struct opp *gpu_opp;
	struct voltagedomain *mpu_voltdm;
	int ret;
	unsigned long freq = ULONG_MAX;

	pr_info("cpu-control : Hello world!\n");

	if (!cpu_is_omap443x()) {
		pr_err("cpu-control: CPU is not OMAP443x\n");
		return 0;
        }

	/* arch/arm/mach-omap2/omap2plus-cpufreq.c */
	SYMSEARCH_BIND_POINTER_TO(omap_temp_sensor, struct mutex*, omap_cpufreq_lock, omap_cpufreq_lock_p);

	/* drivers/base/power/opp.c */
	SYMSEARCH_BIND_FUNCTION_TO(cpu_control, opp_add, opp_add_s);
	SYMSEARCH_BIND_FUNCTION_TO(cpu_control, opp_enable, opp_enable_s);
	SYMSEARCH_BIND_FUNCTION_TO(cpu_control, opp_get_opp_count, opp_get_opp_count_s);
	SYMSEARCH_BIND_FUNCTION_TO(cpu_control, opp_find_freq_floor, opp_find_freq_floor_s);
	SYMSEARCH_BIND_FUNCTION_TO(cpu_control, opp_find_freq_ceil, opp_find_freq_ceil_s);

	/* arch/arm/mach-omap2/voltage.c */
	SYMSEARCH_BIND_FUNCTION_TO(cpu_control, voltdm_lookup, voltdm_lookup_s);

	/* drivers/cpufreq/cpufreq.c */
	SYMSEARCH_BIND_FUNCTION_TO(cpu_control, __find_governor, __find_governor_s);
	SYMSEARCH_BIND_FUNCTION_TO(cpu_control, __cpufreq_set_policy, __cpufreq_set_policy_s);

	freq_table = cpufreq_frequency_get_table(0);
	if (!freq_table) {
		pr_err("%s:No cpufreq driver\n", __func__);
		ret = -EINVAL;
		goto err_out;
	}

	/* cpufreq_cpu_get() should only be done on CPU0 (the boot cpu). For other
	 * CPUs, the policy is destroyed/created on cpu hotplug (which happens during
	 * suspend). cpufreq_cpu_get() gets the omap2plus-cpufreq module  */
	policy = cpufreq_cpu_get(0);
	if (!policy) {
		pr_err("%s:No cpufreq driver\n", __func__);
		ret = -EINVAL;
		goto err_out;
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

	gpu_clk = clk_get(NULL, "gpu_fck");
	if (IS_ERR(gpu_clk)) {
		pr_err("%s:Unable to get gpu_clk\n", __func__);
		ret = -ENODEV;
		goto err_cpu_put;
	}

	gpu_dev = omap_hwmod_name_get_dev("gpu");
	if (!gpu_dev || IS_ERR(gpu_dev)) {
		pr_err("cpu-control: %s: Unable to get gpu device\n", __func__);
		ret = -ENODEV;
		goto err_cpu_put;
	}

	def_ft = kzalloc(sizeof(struct opp_table) * (mpu_opp_count), GFP_KERNEL);
	if (!def_ft) {
		pr_err("cpu-control: %s: Unable to allocate memory\n", __func__);
		ret = -ENOMEM;
		goto err_cpu_put;
	}

	ret = populate_def_freq_table()
        if (ret)
		goto err_free_def_ft;

		
	buf = (char *)vmalloc(BUF_SIZE);
	if (!buf) {
		pr_err("cpu-control: %s: Unable to allocate memory\n", __func__);
		ret = -ENOMEM;
		goto err_free_def_ft;
	}

	/* Create a new sysfs directory under /sys/devices/system/cpu */
	cpucontrol_kobj = kobject_create_and_add("cpucontrol", &cpu_sysdev_class.kset.kobj);
	if (!cpucontrol_kobj) {
		pr_err("cpu-control: %s: Unable to allocate new kobject\n", __func__);
		goto err_free_buf;
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
err_free_buf:
	vfree(buf);
err_free_def_ft:
	kfree(def_ft);
err_cpu_put:
	cpufreq_cpu_put(policy);
err_out:
	return ret;
}

static void __exit cpu_control_exit(void){
	pr_info("cpu_control : Reset opp table to default.\n");

	/* This will also remove the sysfs entries */
	kobject_put(cpucontrol_kobj);

	vfree(buf);

	prepare_opp_modify();
	restore_def_freq_table();
	finish_opp_modify();

	kfree(def_ft);

	cpufreq_cpu_put(policy);

	pr_info("cpu_control : Goodbye\n");
}

module_init(cpu_control_init);
module_exit(cpu_control_exit);

MODULE_AUTHOR("Alexander Lam (lambchop468 (at) gmail.com)");
MODULE_DESCRIPTION("cpu_control - Tweak cpu/gpu voltage & frequency for OMAP443x.");
MODULE_LICENSE("GPL");
