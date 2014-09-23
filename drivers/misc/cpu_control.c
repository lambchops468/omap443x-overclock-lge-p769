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
SYMSEARCH_DECLARE_FUNCTION_STATIC(void, voltdm_reset_s, struct voltagedomain *voltdm);
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
static struct voltagedomain *mpu_voltdm;
static struct omap_vdd_info *mpu_vdd;
static struct clk *mpu_clk, *gpu_clk;
extern struct mutex omap_dvfs_lock;
static struct opp_table *def_ft;

static int mpu_opp_count;
static char def_governor[CPUFREQ_NAME_LEN];
static char good_governor[CPUFREQ_NAME_LEN] = "hotplug";

#define BUF_SIZE PAGE_SIZE
static char *buf;

static int set_governor(struct cpufreq_policy *policy, char str_governor[CPUFREQ_NAME_LEN]) {
	unsigned int ret = -EINVAL;
	struct cpufreq_governor *t;
	struct cpufreq_policy new_policy;
	if (!policy)
		return ret;

	//NEED TO LOCK cpufreq_governor_mutex to call __find_governor_s()
	//NEED TO LOCK lock_policy_rwsem_write (yes) <-- this will also prevent calls to target()
	//maybe use parse_governor?

	cpufreq_get_policy(&new_policy, policy->cpu);
	t = __find_governor_s(str_governor);
	if (t != NULL) {
		new_policy.governor = t;
	} else {
		return ret;
	}

	ret = __cpufreq_set_policy_s(policy, &new_policy);

	policy->user_policy.policy = policy->policy;
	policy->user_policy.governor = policy->governor;
	return ret;
}

/*
 * Freq in Hz
 * Volt in uV
 */
static int set_one_opp(int index, int freq, int volt) {
	freq_table[index].frequency = freq/1000;
	mpu_vdd->volt_data[index].volt_nominal = volt;
	// this is wrong. wrong. wrong. all. wrong.
	mpu_vdd->dep_vdd_info[0].dep_table[index].main_vdd_volt = volt;
	def_ft[index].opp->u_volt = volt;
	def_ft[index].opp->rate = freq;
}

/* sysfs please. */
/*  proc fs */
static int proc_gpu_cpu_speed(char *buffer, char **buffer_location, off_t offset, int count, int *eof, void *data) {
	int ret = 0;
	if (offset > 0)
		ret = 0;
	else
		ret += scnprintf(buffer+ret, count-ret,"CPU : %lu Mhz\nGPU : %lu Mhz\n", clk_get_rate(mpu_clk)/1000000, clk_get_rate(gpu_clk)/1000000);
	return ret;
}

static int proc_mpu_opp_def(char *buffer, char **buffer_location, off_t offset, int count, int *eof, void *data) {
	int i, ret = 0;

	if (offset > 0)
		ret = 0;
	else
		ret += scnprintf(buffer+ret, count-ret, "Id\tFreq\tVolt(mV)\n");
		for(i = 0;i <mpu_opp_count; i++) {
			ret += scnprintf(buffer+ret, count-ret, "%d\t%lu\t%lu\n", def_ft[i].index, def_ft[i].rate/1000000, def_ft[i].u_volt/1000);
		}
	return ret;
}

static int proc_mpu_opp_cur(char *buffer, char **buffer_location, off_t offset, int count, int *eof, void *data) {
	int i, ret = 0;

	if (offset > 0)
		ret = 0;
	else
		ret += scnprintf(buffer+ret, count-ret, "Id\tFreq\tVolt(mV)\n");
		for(i = 0;i <mpu_opp_count; i++) {
			ret += scnprintf(buffer+ret, count-ret, "%d\t%lu\t%lu\n", def_ft[i].index, def_ft[i].opp->rate/1000000, def_ft[i].opp->u_volt/1000);
		}
	return ret;
}

static int proc_cpu_tweak(struct file *filp, const char __user *buffer, unsigned long len, void *data) {
	int id;
	int freq; //in KHz
	int volt; //in mV
	bool change;
	if(!len || len >= BUF_SIZE)
		return -ENOSPC;

	if(copy_from_user(buf, buffer, len))
		return -EFAULT;

	buf[len] = 0;
	if(sscanf(buf, "%d %d %d", &id, &freq, &volt) == 3) {
		if ((id > mpu_opp_count-1) || (id < 0)) {
			pr_err("cpu-control : wrong cpu opp id @ %d", id);
			return -EFAULT;
		}
		//some filter
		if (volt > OMAP4_VP_MPU_VLIMITTO_VDDMAX/1000) volt = OMAP4_VP_MPU_VLIMITTO_VDDMAX/1000;
		if (volt < OMAP4_VP_MPU_VLIMITTO_VDDMIN/1000) volt = OMAP4_VP_MPU_VLIMITTO_VDDMIN/1000;
		if (freq > 2000) freq = def_ft[mpu_opp_count-1].rate/1000000;
		if (freq < 0) freq = def_ft[0].rate/1000;

		pr_info("cpu-control : Change operating point : %d %d Mhz %d mV\n", id, freq, volt);

		policy->min = policy->cpuinfo.min_freq = policy->user_policy.min =
		policy->max = policy->cpuinfo.max_freq = policy->user_policy.max =
		def_ft[mpu_opp_count-1].rate/1000;

		pr_info("cpu-control : Current cpufreq gov : %s\n", policy->governor->name);
		if (policy->governor->name != good_governor) {
			strcpy(def_governor, policy->governor->name);
			set_governor(policy, good_governor);
			change = true;
			pr_info("cpu-control : Change cpufreq gov : %s\n", policy->governor->name);
		}

		mutex_lock(omap_cpufreq_lock_p);
		mutex_lock(&omap_dvfs_lock);

		set_one_opp(id, freq*1000000, volt*1000);

		//why? why? postpone to actual frequency change.
		voltdm_reset_s(mpu_voltdm);
		mutex_unlock(&omap_dvfs_lock);
		mutex_unlock(omap_cpufreq_lock_p);

		if (change) {
			set_governor(policy, def_governor);
			pr_info("cpu_control : Revert cpufreq gov : %s\n", policy->governor->name);
		}

		policy->min = policy->cpuinfo.min_freq = policy->user_policy.min = freq_table[0].frequency;
		policy->max = policy->cpuinfo.max_freq = policy->user_policy.max = freq_table[opp_count-1].frequency;
	}
	return len;
}
/* proc fs end */

static int __init populate_def_freq_table() {
	unsigned long freq;
	int ret = 0;
	int i;

	mutex_lock(omap_cpufreq_lock_p);
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
	mutex_unlock(omap_cpufreq_lock_p);

	return ret;
}

static int __exit restore_def_freq_table() {
	mutex_lock(omap_cpufreq_lock_p);
	mutex_lock(&omap_dvfs_lock);
	for(i = 0; i<opp_count; i++) {
		set_one_opp(i, def_ft[i].rate, def_ft[i].u_volt);
	}
		//why? why? postpone to actual frequency change.
	voltdm_reset_s(mpu_voltdm);
	mutex_unlock(&omap_dvfs_lock);
	mutex_unlock(omap_cpufreq_lock_p);
}

static int __init cpu_control_init(void) {
	struct proc_dir_entry *proc_entry;
	struct opp *gpu_opp;
	int ret;
	unsigned long freq = ULONG_MAX;

	pr_info("cpu-control : Hello world!\n");

	// todo: ensure this cpu is omap443x

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
	SYMSEARCH_BIND_FUNCTION_TO(cpu_control, voltdm_reset, voltdm_reset_s);

	/* drivers/cpufreq/cpufreq.c */
	SYMSEARCH_BIND_FUNCTION_TO(cpu_control, __find_governor, __find_governor_s);
	SYMSEARCH_BIND_FUNCTION_TO(cpu_control, __cpufreq_set_policy, __cpufreq_set_policy_s);

	freq_table = cpufreq_frequency_get_table(0);
	if (!freq_table) {
		pr_err("%s:No cpufreq driver\n", __func__);
		return -EINVAL;
	}

	/* cpufreq_cpu_get() should only be done on CPU0 (the boot cpu). For other
	 * CPUs, the policy is destroyed/created on cpu hotplug (which happens during
	 * suspend). cpufreq_cpu_get() gets the omap2plus-cpufreq module  */
	policy = cpufreq_cpu_get(0);
	if (!policy) {
		pr_err("%s:No cpufreq driver\n", __func__);
		return -EINVAL;
	}

	mpu_clk = clk_get(NULL, "dpll_mpu_ck");
	if (IS_ERR(mpu_clk)) {
		pr_err("%s:Unable to get mpu_clk\n", __func__);
		return -ENODEV;
	}

	mpu_voltdm = voltdm_lookup_s("mpu");
	if (!mpu_voltdm || IS_ERR(mpu_voltdm)) {
		pr_err("cpu-control: %s: Unable to get mpu voltage domain\n", __func__);
		return -ENODEV;
	}
	mpu_vdd = mpu_voltdm->vdd;

	mpu_dev = omap2_get_mpuss_device();
	if (!mpu_dev || IS_ERR(mpu_dev)) {
		pr_err("cpu-control: %s: Unable to get mpu device\n", __func__);
		return -ENODEV;
	}
	mpu_opp_count = opp_get_opp_count_s(mpu_dev);
	if (mpu_opp_count <= 0) {
		pr_err("cpu-control: %s: Could not get opp count for mpu\n", __func__);
		return -EINVAL;
	}

	gpu_clk = clk_get(NULL, "gpu_fck");
	if (IS_ERR(gpu_clk)) {
		pr_err("%s:Unable to get gpu_clk\n", __func__);
		return -ENODEV;
	}

	gpu_dev = omap_hwmod_name_get_dev("gpu");
	if (!gpu_dev || IS_ERR(gpu_dev)) {
		pr_err("cpu-control: %s: Unable to get gpu device\n", __func__);
		return -ENODEV;
	}

	def_ft = kzalloc(sizeof(struct opp_table) * (mpu_opp_count), GFP_KERNEL);
	if (!def_ft) {
		pr_err("cpu-control: %s: Unable to allocate memory\n", __func__);
		return -ENOMEM;
	}

	ret = populate_def_freq_table()
        if (!ret)
		goto populate_table_fail;

		
	//pr_info("GPU OPP counts : %d\n", opp_get_opp_count_s(gpu_dev));
	gpu_opp = opp_find_freq_floor_s(gpu_dev, &freq);
	//gpu_opp->rate = 384000000;
	pr_info("cpu-control : GPU Default max value : %lu mV : %lu\n", gpu_opp->u_volt/1000, gpu_opp->rate/1000);
/*	if (opp_get_opp_count_s(gpu_dev) == 2) {
		//opp_disable_s(gpu_dev, 384000000);
		opp_add_s(gpu_dev, 384000000, gpu_opp->u_volt);
		opp_enable_s(gpu_dev, 384000000);
	}
*/
	buf = (char *)vmalloc(BUF_SIZE);
	if (!buf) {
		pr_err("cpu-control: %s: Unable to allocate memory\n", __func__);
		ret = -ENOMEM;
		goto buf_alloc_fail;
	}

	proc_mkdir("cpu_control", NULL);
	proc_entry = create_proc_read_entry("cpu_control/frequency_current", 0444, NULL, proc_gpu_cpu_speed, NULL);
	proc_entry = create_proc_read_entry("cpu_control/opp_table_current", 0444, NULL, proc_mpu_opp_cur, NULL);
	proc_entry = create_proc_read_entry("cpu_control/opp_table_default", 0444, NULL, proc_mpu_opp_def, NULL);
	proc_entry = create_proc_read_entry("cpu_control/tweak_cpu", 0644, NULL, NULL, NULL);
	proc_entry->write_proc = proc_cpu_tweak;
/*
	//prepare
	pr_info("Current cpufreq gov : %s\n", policy->governor->name);
	if (policy->governor->name != good_governor) {
		strcpy(def_governor, policy->governor->name);
		set_governor(policy, good_governor);
		policy = cpufreq_cpu_get(0);
		change = true;
		pr_info("Current cpufreq gov : %s\n", policy->governor->name);
	}

	policy->min = policy->cpuinfo.min_freq = policy->user_policy.min =
	policy->max = policy->cpuinfo.max_freq = policy->user_policy.max =
	def_max_freq/1000;

	mutex_lock(&omap_dvfs_lock);
	//hack

	freq_table[0].frequency = LOW_FREQ/1000;
	vdd->volt_data[0].volt_nominal = LOW_VOLTAGE;
	vdd->dep_vdd_info[0].dep_table[0].main_vdd_volt = LOW_VOLTAGE;
	gpu_opp->u_volt = LOW_VOLTAGE;
	gpu_opp->rate = LOW_FREQ;

	freq_table[4].frequency = HIGH_FREQ/1000;
	vdd->volt_data[4].volt_nominal = HIGH_VOLTAGE;
	vdd->dep_vdd_info[0].dep_table[4].main_vdd_volt = HIGH_VOLTAGE;
	mpu_opp->u_volt = HIGH_VOLTAGE;
	mpu_opp->rate = HIGH_FREQ;

	//min_opp->rate= 384000000;
/*
	if (opp_count == 5) {
		opp_add_s(mpu_dev, LOW_FREQ, LOW_VOLTAGE);
		opp_add_s(mpu_dev, HIGH_FREQ, HIGH_VOLTAGE);
		opp_enable_s(mpu_dev, LOW_FREQ);
		opp_enable_s(mpu_dev, HIGH_FREQ);
		opp_count = opp_get_opp_count_s(mpu_dev);
		temp_ft = kzalloc(sizeof(struct cpufreq_frequency_table) * (opp_count + 1), GFP_KERNEL);
		for(i = 0; i<8; i++) {
			temp_ft[i].index = i;
			switch (i) {
				case 0:
					temp_ft[i].frequency = LOW_FREQ/1000;
					break;
				case 6:
					temp_ft[i].frequency = HIGH_FREQ/1000;
					break;
				case 7:					
					temp_ft[i].frequency = CPUFREQ_TABLE_END;
					break;
				default:
					temp_ft[i].frequency = freq_table[i-1].frequency;
			}
		}
		pr_info("Temp voltage & frequency map\n");
		for(i = 0; i<opp_count; i++) {
			pr_info("%d : %u mV %u Mhz\n",i,vdd->volt_data[i].volt_nominal/1000, temp_ft[i].frequency/1000);
		}
		cpufreq_frequency_table_put_attr(0);
		cpufreq_frequency_table_get_attr(temp_ft,0);
		temp_ft = NULL;
		freq_table = cpufreq_frequency_get_table(0);
	}
	if (opp_get_opp_count_s(gpu_dev) == 2) {
		//opp_disable_s(gpu_dev, 384000000);
		opp_add_s(gpu_dev, 384000000, 1127000);
		opp_enable_s(gpu_dev, 384000000);
	}
	voltdm_reset_s(voltdm);
	mutex_unlock(&omap_dvfs_lock);

	policy->min = policy->cpuinfo.min_freq = policy->user_policy.min = freq_table[0].frequency;
	policy->max = policy->cpuinfo.max_freq = policy->user_policy.max = freq_table[opp_count-1].frequency;

	pr_info("Optimize voltage & frequency map\n");
	for(i = 0; i<opp_count; i++) {
		pr_info("%d : %u mV %u Mhz\n",i,vdd->volt_data[i].volt_nominal/1000, freq_table[i].frequency/1000);
	}

	pr_info("hacked mpu OPP counts : %d\n", opp_get_opp_count_s(mpu_dev));
	pr_info("hacked GPU OPP counts : %d\n", opp_get_opp_count_s(gpu_dev));

	if (change == true) {
		set_governor(policy, def_governor);
		policy = cpufreq_cpu_get(0);
		pr_info("Current cpufreq gov : %s\n", policy->governor->name);
	}
*/
	return 0;

buf_alloc_fail:
populate_table_fail:
	kfree(def_ft);

	return ret;
}

static void __exit cpu_control_exit(void){
	int i;
	bool change;

	pr_info("cpu_control : Reset opp table to default.\n");
	remove_proc_entry("cpu_control/frequency_current", NULL);
	remove_proc_entry("cpu_control/opp_table_current", NULL);
	remove_proc_entry("cpu_control/opp_table_default", NULL);
	remove_proc_entry("cpu_control/tweak_cpu", NULL);
	remove_proc_entry("cpu_control", NULL);
	vfree(buf);

	// this seems unsafe. __set_policy should be able to handle most of this...
	// to do this, set the governor to performance. then we don't even need this following 3 lines...
	//
	// actually, we should extend set_governor to take min/max as args and change the to-be-inserted policy.
	// for the other half, we would also need to mdify cpuinfo.min_freq and cpuinfo.max_freq in the to-be-inserted policy. maybe.
	policy->min = policy->cpuinfo.min_freq = policy->user_policy.min =
	policy->max = policy->cpuinfo.max_freq = policy->user_policy.max =
	def_ft[opp_count-1].rate/1000;

	pr_info("cpu-control : Current cpufreq gov : %s\n", policy->governor->name);
	if (policy->governor->name != good_governor) {
		strcpy(def_governor, policy->governor->name);
		set_governor(policy, good_governor);
		change = true;
		pr_info("cpu-control : Change cpufreq gov : %s\n", policy->governor->name);
	}

	restore_def_freq_table();

	kfree(def_ft);

	if (change) {
		set_governor(policy, def_governor);
		pr_info("cpu_control : Revert cpufreq gov : %s\n", policy->governor->name);
	}

	// however, we do need this, as the frequencies have changed. or does set_governor() do this for us? hm.
	policy->min = policy->cpuinfo.min_freq = policy->user_policy.min = freq_table[0].frequency;
	policy->max = policy->cpuinfo.max_freq = policy->user_policy.max = freq_table[opp_count-1].frequency;
	pr_info("cpu_control : Goodbye\n");
}

module_init(cpu_control_init);
module_exit(cpu_control_exit);

MODULE_AUTHOR("Alexander Lam (lambchop468 (at) gmail.com)");
MODULE_DESCRIPTION("cpu_control - Tweak cpu/gpu voltage & frequency for OMAP443x.");
MODULE_LICENSE("GPL");
