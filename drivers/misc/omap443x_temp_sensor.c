/*
 * OMAP443X Temperature sensor driver file
 * Copyright (C) 2014 Alexander Lam (lambchop468 (at) gmail.com)
 *
 * This driver supports the OMAP4430 on-die bandgap temperature sensor.
 * ti-bandgap.c provides support for this sensor, but it does not exist in
 * older kernels, so this driver was written.
 * This driver supports reading of the temperature of the sensor.
 *
 * In theory the driver also supports the TSHUT signal generated by the sensor
 * when the temperature is above the critical shutdown temperature. Upon
 * receiving the TSHUT interrupt, this driver will throttle the cpu twice (use
 * the 3rd fastest cpu frequency) and poweroff the system. This TSHUT
 * functionality remains untested.
 *
 * The driver also features polling-based cpu speed throttling to keep the
 * temperature of the chip down.
 *
 * To read the temperature of the CPU:
 * $ cat /sys/devices/platform/omap/omap_temp_sensor.0/temperature
 *
 * To check the automatic throttling temperature limits:
 * $ cat /sys/devices/platform/omap/omap_temp_sensor.0/throttle_temp
 * This prints the cold throttle threshold in milliCelcius, a space, and the
 * hot throttle threshold in millicelsius. 
 *
 * To set the cold throttle threshold to 80 C and the hot throttle threshold
 * to 85 C:
 * $ echo 80000 85000 > /sys/devices/platform/omap/omap_temp_sensor.0/throttle_temp
 *
 * The throttle algorithm checks the temperature, and if it is above the hot
 * threshold, the driver will disable the highest enabled frequency. The next
 * temperature check in this case occurs in 1 second. If the temperature is
 * still above the hot threshold, then the next highest frequency is also
 * disabled. The next temperature check occurs in 1 second.
 *
 * When the temperature check is below the cold threshold, then  the driver
 * will enable the next faster frequency. The next temperature check will
 * occur in 1 second.
 *
 * If the current temperature during the check is 10 degrees below the cold
 * threshold, then the next check will occur in 5 seconds.
 *
 * Automatic throttling can be disabled by loading the module with
 * auto_throttle = 0:
 * $ insmod omap443x_temp_sensor.ko auto_throttle=0
 * This will cause the manual user-space throttling interface to be exposed:
 *
 * To throttle the CPU from userspace:
 * $ echo 1 >  /sys/devices/platform/omap/omap_temp_sensor.0/throttle
 * Each time "1" is written to tthe throttle file, the CPU frequency governor
 * will remove the next highest frequency from the frequency table.
 *
 * To unthrottle the CPU from userspace:
 * $ echo 0 >  /sys/devices/platform/omap/omap_temp_sensor.0/throttle
 * This will allow the next highest disabled frequency to be used again.
 *
 * This driver is extensively modified from omap_temp_sensor, written by:
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 * Author: J Keerthy <j-keerthy@ti.com>
 * Author: Moiz Sonasath <m-sonasath@ti.com>
 *
 * This driver incorporates ideas from ti-bandgap.c, written by:
 * Copyright (C) 2011-2012 Texas Instruments Incorporated - http://www.ti.com/
 * Author: J Keerthy <j-keerthy@ti.com>
 * Author: Moiz Sonasath <m-sonasath@ti.com>
 * Couple of fixes, DT and MFD adaptation:
 *   Eduardo Valentin <eduardo.valentin@ti.com>
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/types.h>

#include <plat/common.h>
#include <plat/omap_hwmod.h>
#include <plat/omap_device.h>
#include <plat/temperature_sensor.h>

/* TO DO: This needs to be fixed */
#include "../../../../arch/arm/mach-omap2/control.h"
/* #include <plat/control.h> */

#include <linux/kallsyms.h>
#include "symsearch/symsearch.h"

#define DRIVER_NAME "omap_temp_sensor"

/* arch/arm/mach-omap2/control.c */
SYMSEARCH_DECLARE_FUNCTION_STATIC(u32, omap_ctrl_readl_s, u16 offset);
SYMSEARCH_DECLARE_FUNCTION_STATIC(void, omap_ctrl_writel_s, u32 val, u16 offset);
/* arch/arm/plat-omap/omap_device.c */
SYMSEARCH_DECLARE_FUNCTION_STATIC(struct omap_device*, omap_device_build_s,
	const char *pdev_name, int pdev_id, struct omap_hwmod *oh, void *pdata,
	int pdata_len, struct omap_device_pm_latency *pm_lats, int pm_lats_cnt,
	int is_early_device);
SYMSEARCH_DECLARE_FUNCTION_STATIC(int, omap_device_enable_hwmods_s,
	struct omap_device *od);
SYMSEARCH_DECLARE_FUNCTION_STATIC(int, omap_device_idle_hwmods_s,
	struct omap_device *od);
/* drivers/cpufreq/cpufreq.c */
SYMSEARCH_DECLARE_FUNCTION_STATIC(int, lock_policy_rwsem_read_s, int cpu);
SYMSEARCH_DECLARE_FUNCTION_STATIC(void, unlock_policy_rwsem_read_s, int cpu);
SYMSEARCH_DECLARE_FUNCTION_STATIC(int, lock_policy_rwsem_write_s, int cpu);
SYMSEARCH_DECLARE_FUNCTION_STATIC(void, unlock_policy_rwsem_write_s, int cpu);
SYMSEARCH_DECLARE_FUNCTION_STATIC(int, __cpufreq_set_policy_s,
		struct cpufreq_policy *data, struct cpufreq_policy *policy);


static bool auto_throttle = true;
module_param(auto_throttle, bool, S_IRUGO);

static void throttle_delayed_work_fn(struct work_struct *work);

/**
 * The OMAP4430 Data Manual says the maximum average temperature of the chip
 * should not exceed 110 deg C and the absolute maximum is 125 deg C.
 *
 * Ensure that the temperatures selected are present in the adc_to_temp table.
 */
#define THROTTLE_COLD		83000	/* 83 deg C */
#define THROTTLE_HOT		85000	/* 85 deg C */
#define	THROTTLE_DELAY_HOT	1000	/* 1 second */
#define THROTTLE_WARM_OFFSET	10000	/* 10 deg C */
#define	THROTTLE_DELAY_COLD	5000	/* 5 seconds */


/**
 * OMAP4430_* definitions from
 * linux-3.16/drivers/thermal/ti-soc-thermal/omap4xxx-bandgap.h
 */

/**
 * Register and bit definitions for OMAP4430
 *
 * All the macros bellow define the required bits for
 * controlling temperature on OMAP4430. Bit defines are
 * grouped by register.
 */

/* OMAP4430.TEMP_SENSOR bits */
#define OMAP4430_BGAP_TEMPSOFF_MASK			BIT(12)
#define OMAP4430_BGAP_TSHUT_MASK			BIT(11)
#define OMAP4430_SINGLE_MODE_MASK			BIT(10)
#define OMAP4430_BGAP_TEMP_SENSOR_SOC_MASK		BIT(9)
#define OMAP4430_BGAP_TEMP_SENSOR_EOCZ_MASK		BIT(8)
#define OMAP4430_BGAP_TEMP_SENSOR_DTEMP_MASK		(0xff << 0)

/**
 * Temperature limits and thresholds for OMAP4430
 *
 * All the macros bellow are definitions for handling the
 * ADC conversions and representation of temperature limits
 * and thresholds for OMAP4430.
 */

/* ADC conversion table limits */
#define OMAP4430_ADC_START_VALUE			0
#define OMAP4430_ADC_END_VALUE				127
/* bandgap clock limits (no control on 4430) */
#define OMAP4430_MAX_FREQ				32768
#define OMAP4430_MIN_FREQ				32768
/* sensor limits */
#define OMAP4430_MIN_TEMP				-40000
#define OMAP4430_MAX_TEMP				125000
#define OMAP4430_HYST_VAL				5000

/*
 * omap_temp_sensor structure
 * @pdev - Platform device pointer
 * @dev - device pointer
 * @clock - Clock pointer
 * @lock - Lock for enable/disabling of sensor
 * @tshut_irq -  Thermal shutdown IRQ
 * @phy_base - Physical base of the temp I/O
 * @is_efuse_valid - Flag to determine if eFuse is valid or not
 * @clk_on - Manages the current clock state
 * @clk_rate - Holds current clock rate
 * @throttle_lock - Locking for throttling
 * @throttle_cold - The cold throttle threshold
 * @throttle_hot - The hot throttle threshold
 * @throttling - How much throttling is applied
 * @attempting_cool - Are we attempting to cool down?
 * @throttle_work - The work struct for throttling
 */
struct omap_temp_sensor {
	struct platform_device *pdev;
	struct device *dev;
	struct clk *clock;
	struct spinlock lock;
	int tshut_irq;
	unsigned long phy_base;
	int is_efuse_valid;
	u8 clk_on;
	unsigned long clk_rate;

	struct mutex throttle_lock;
	int throttle_cold;
	int throttle_hot;
	int throttling;
	u8 attempting_cool;

	struct delayed_work throttle_work;

	unsigned int throttle_freq;
	struct cpufreq_frequency_table *freq_table;
	struct cpufreq_policy *cpufreq_policy;
};

/*
 * Temperature values in milli degree celsius
 * ADC code values from 0 to 127
 */
static const int
omap4430_adc_to_temp[OMAP4430_ADC_END_VALUE - OMAP4430_ADC_START_VALUE + 1] = {
	-38000, -35000, -34000, -32000, -30000, -28000, -26000, -24000, -22000,
	-20000, -18000, -17000, -15000, -13000, -12000, -10000, -8000, -6000,
	-5000, -3000, -1000, 0, 2000, 3000, 5000, 6000, 8000, 10000, 12000,
	13000, 15000, 17000, 19000, 21000, 23000, 25000, 27000, 28000, 30000,
	32000, 33000, 35000, 37000, 38000, 40000, 42000, 43000, 45000, 47000,
	48000, 50000, 52000, 53000, 55000, 57000, 58000, 60000, 62000, 64000,
	66000, 68000, 70000, 71000, 73000, 75000, 77000, 78000, 80000, 82000,
	83000, 85000, 87000, 88000, 90000, 92000, 93000, 95000, 97000, 98000,
	100000, 102000, 103000, 105000, 107000, 109000, 111000, 113000, 115000,
	117000, 118000, 120000, 122000, 123000,
};

static unsigned long omap_temp_sensor_readl(struct omap_temp_sensor
					    *temp_sensor, u32 reg)
{
	return omap_ctrl_readl_s(temp_sensor->phy_base + reg);
}

static void omap_temp_sensor_writel(struct omap_temp_sensor *temp_sensor,
				    u32 val, u32 reg)
{
	omap_ctrl_writel_s(val, (temp_sensor->phy_base + reg));
}

static int adc_to_temp_conversion(int adc_val)
{
	if (adc_val < OMAP4430_ADC_START_VALUE || adc_val > OMAP4430_ADC_END_VALUE) {
		pr_err("%s:Temp read is invalid %i\n", __func__, adc_val);
		return -EINVAL;
	}

	return omap4430_adc_to_temp[adc_val - OMAP4430_ADC_START_VALUE];
}

static int temp_to_adc_conversion(long temp)
{
	int i;

	if (temp < omap4430_adc_to_temp[0])
		return -EINVAL;

	for (i = 1; i <= OMAP4430_ADC_END_VALUE - OMAP4430_ADC_START_VALUE; i++)
		if (temp < omap4430_adc_to_temp[i])
			return OMAP4430_ADC_START_VALUE + i - 1;
	return -EINVAL;
}

static inline int omap_read_temp(struct omap_temp_sensor *temp_sensor)
{
	int temp;

	temp = omap_temp_sensor_readl(temp_sensor, TEMP_SENSOR_CTRL_OFFSET);
	temp &= (OMAP4430_BGAP_TEMP_SENSOR_DTEMP_MASK);

	return temp;
}

static int omap_read_current_temp(struct omap_temp_sensor *temp_sensor)
{
	int adc;

	adc = omap_read_temp(temp_sensor);

	if (!temp_sensor->is_efuse_valid)
		pr_err_once("%s: Invalid EFUSE, Non-trimmed BGAP,"
			    "Temp not accurate\n", __func__ );

	if (adc < OMAP4430_ADC_START_VALUE || adc > OMAP4430_ADC_END_VALUE) {
		pr_err("%s:Invalid adc code reported by the sensor %d",
			__func__, adc);
		return -EINVAL;
	}

	return adc_to_temp_conversion(adc);
}

static void omap_enable_continuous_mode(struct omap_temp_sensor *temp_sensor)
{
	u32 val, counter;

	/* Disable continuous mode */
	val = omap_temp_sensor_readl(temp_sensor, TEMP_SENSOR_CTRL_OFFSET);
	val &= ~(OMAP4430_SINGLE_MODE_MASK);
	omap_temp_sensor_writel(temp_sensor, val, TEMP_SENSOR_CTRL_OFFSET);

	/* Wait for EOCZ to go low */
	counter = 100;
	val = omap_temp_sensor_readl(temp_sensor, TEMP_SENSOR_CTRL_OFFSET);
	while (val & OMAP4430_BGAP_TEMP_SENSOR_EOCZ_MASK && --counter) {
		val = omap_temp_sensor_readl(temp_sensor, TEMP_SENSOR_CTRL_OFFSET);
		udelay(100);
	}

	/* Enable continuous mode */
	val = omap_temp_sensor_readl(temp_sensor, TEMP_SENSOR_CTRL_OFFSET);
	val |= OMAP4430_SINGLE_MODE_MASK;
	omap_temp_sensor_writel(temp_sensor, val, TEMP_SENSOR_CTRL_OFFSET);

	/* Start ADC conversion */
	/* Start of Conversion = 1 */
	val = omap_temp_sensor_readl(temp_sensor, TEMP_SENSOR_CTRL_OFFSET);
	val |= OMAP4430_BGAP_TEMP_SENSOR_SOC_MASK;
	omap_temp_sensor_writel(temp_sensor, val, TEMP_SENSOR_CTRL_OFFSET);

	/* Wait for EOCZ to go high */
	counter = 100;
	val = omap_temp_sensor_readl(temp_sensor, TEMP_SENSOR_CTRL_OFFSET);
	while (!(val & OMAP4430_BGAP_TEMP_SENSOR_EOCZ_MASK) && --counter) {
		val = omap_temp_sensor_readl(temp_sensor, TEMP_SENSOR_CTRL_OFFSET);
		udelay(100);
	}

	/* Start of Conversion = 0 */
	val = omap_temp_sensor_readl(temp_sensor, TEMP_SENSOR_CTRL_OFFSET);
	val &= ~(OMAP4430_BGAP_TEMP_SENSOR_SOC_MASK);
	omap_temp_sensor_writel(temp_sensor, val, TEMP_SENSOR_CTRL_OFFSET);

	/* DTEMP will be updated in <= 36 cycles */
}

/**
 * Schedule the next temperature check - if it is hot, check the temperature
 * sooner. This allows us to decide if we need to unthrottle()/throttle(),
 * benefiting both the health of the chip and performance.
 */
static bool schedule_throttle_work(struct omap_temp_sensor *temp_sensor,
		int curr_temp)
{
	int delay_ms;
	if (curr_temp >= temp_sensor->throttle_cold - THROTTLE_WARM_OFFSET) {
		delay_ms = THROTTLE_DELAY_HOT;
	} else {
		delay_ms = THROTTLE_DELAY_COLD;
	}
	return queue_delayed_work(system_freezable_wq,
			&temp_sensor->throttle_work,
			msecs_to_jiffies(delay_ms));
}

/* Find the next lowest frequency lower than freq. Capable of handling the case
 * where freq is no longer in the frequency table.
 */
static unsigned int cpufreq_next_lower_freq(
		struct cpufreq_frequency_table *freq_table,
		unsigned int freq)
{
	int i;
	
	unsigned int candidate = 0;
	unsigned int lowest = ~0;

	for (i = 0; freq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
		if (freq_table[i].frequency > candidate &&
				freq_table[i].frequency < freq)
			candidate = freq_table[i].frequency;
		if (freq_table[i].frequency < lowest)
			lowest = freq_table[i].frequency;
	}

	if (candidate == 0)
		return lowest;
	else
		return candidate;
}

/* Find the next lowest frequency higher than freq. Capable of handling the case
 * where freq is no longer in the frequency table.
 */
static unsigned int cpufreq_next_higher_freq(
		struct cpufreq_frequency_table *freq_table,
		unsigned int freq)
{
	int i;
	
	unsigned int candidate = ~0;
	unsigned int highest = 0;
	for (i = 0; freq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
		if (freq_table[i].frequency < candidate &&
				freq_table[i].frequency > freq)
			candidate = freq_table[i].frequency;
		if (freq_table[i].frequency > highest)
			highest = freq_table[i].frequency;
	}

	if (candidate == ~0)
		return highest;
	else
		return candidate;
}

static unsigned int cpufreq_get_max_freq(struct cpufreq_policy *policy)
{
	unsigned int freq;

	if (lock_policy_rwsem_read_s(policy->cpu) < 0)
		return -1;

	freq = policy->cpuinfo.max_freq;

	unlock_policy_rwsem_read_s(policy->cpu);

	return freq;
}

static int omap_rethrottle_cpu(struct omap_temp_sensor *temp_sensor,
		bool throttle)
{
	struct cpufreq_policy new_policy;
	struct cpufreq_policy *policy = temp_sensor->cpufreq_policy;
	struct cpufreq_frequency_table *freq_table = temp_sensor->freq_table;
	unsigned int new_freq;
	int ret = 0;

	if (lock_policy_rwsem_write_s(policy->cpu) < 0)
		return -ENODEV;

	/* Derive new throttle frequency from new_policy.max,
	 * which takes into account the user's max and the
	 * cpuinfo.max */
	if (throttle)
		new_freq = cpufreq_next_lower_freq(freq_table,
				policy->max);
	else
		new_freq = cpufreq_next_higher_freq(freq_table,
				policy->max);
	
	/* If we can't do anything, bail and tell the caller */
	if (new_freq == policy->cpuinfo.max_freq) {
		ret = 1;
		goto out;
	}

	/* Cannot set new_policy.cpuinfo.max_freq because policy->cpuinfo will
	 * be copied into new_policy.cpuinfo by __cpufreq_set_policy()
	 */
	policy->cpuinfo.max_freq = policy->max =
		policy->user_policy.max = new_freq;
	temp_sensor->throttle_freq = new_freq;

	/* new_policy only serves to be a placeholder argument for
	 * __cpufreq_set_policy() */
	ret = cpufreq_get_policy(&new_policy, policy->cpu);
	if (ret)
		goto out;

	ret = __cpufreq_set_policy_s(policy, &new_policy);

	policy->user_policy.policy = policy->policy;
	policy->user_policy.governor = policy->governor;

out:
	unlock_policy_rwsem_write_s(policy->cpu);

	return ret;
}

/*
 * sysfs hook functions
 */
static ssize_t omap_temp_show_current(struct device *dev,
				struct device_attribute *devattr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_temp_sensor *temp_sensor = platform_get_drvdata(pdev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", omap_read_current_temp(temp_sensor));
}

static ssize_t omap_throttle_store(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_temp_sensor *temp_sensor = platform_get_drvdata(pdev);
	int ret;

	if (count && buf[0] == '1') {
		ret = omap_rethrottle_cpu(temp_sensor, true);
	} else {
		ret = omap_rethrottle_cpu(temp_sensor, false);
	}

	if (ret)
		return ret;

	return count;
}

static ssize_t omap_throttle_temp_store(struct device *dev,
				struct device_attribute *devattr,
				const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_temp_sensor *temp_sensor = platform_get_drvdata(pdev);

	int ret = count, new_cold, new_hot, new_cold_adc, new_hot_adc;

	if (sscanf(buf, "%d %d", &new_cold, &new_hot) < 2) {
		pr_err("%s:Two temperatures could not be read\n", __func__);
		ret = -EINVAL;
		goto out;
	} else if (!(new_cold < new_hot)) {
		pr_err("%s:Cold temperature is not less than hot temperature\n", __func__);
		ret = -EINVAL;
		goto out;
	} else if ((new_cold_adc = temp_to_adc_conversion(new_cold)) < 0) {
		pr_err("%s:Cold temperature is out of range\n", __func__);
		ret = -EINVAL;
		goto out;
	} else if ((new_hot_adc = temp_to_adc_conversion(new_hot)) < 0) {
		pr_err("%s:Hot temperature is out of range\n", __func__);
		ret = -EINVAL;
		goto out;
	} else if (!(new_cold_adc < new_hot_adc)) {
		pr_err("%s:The temperature sensor cannot represent the"
			"difference between the hot and cold temperatures."
			"Try a larger difference\n" , __func__);
		ret = -EINVAL;
		goto out;
	}

	/* Store temperatures in temperatures which can be represented so
	 * reading the throttle_temp file provides the actual thresholds */
	new_cold = adc_to_temp_conversion(new_cold_adc);
	new_hot = adc_to_temp_conversion(new_hot_adc);

	cancel_delayed_work_sync(&temp_sensor->throttle_work);

	mutex_lock(&temp_sensor->throttle_lock);
	schedule_throttle_work(temp_sensor, omap_read_current_temp(temp_sensor));

	temp_sensor->throttle_hot = new_hot;
	temp_sensor->throttle_cold = new_cold;
	mutex_unlock(&temp_sensor->throttle_lock);

out:
	return ret;
}

static ssize_t omap_throttle_temp_show(struct device *dev,
				struct device_attribute *devattr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_temp_sensor *temp_sensor = platform_get_drvdata(pdev);
	ssize_t bytes;

	mutex_lock(&temp_sensor->throttle_lock);
	bytes = scnprintf(buf, PAGE_SIZE, "%d %d\n", temp_sensor->throttle_cold,
			temp_sensor->throttle_hot);
	mutex_unlock(&temp_sensor->throttle_lock);

	return bytes;
}

static DEVICE_ATTR(temperature, S_IRUGO, omap_temp_show_current, NULL);
static DEVICE_ATTR(throttle, S_IWUSR, NULL, omap_throttle_store);
static DEVICE_ATTR(throttle_temp, S_IWUSR | S_IRUGO, omap_throttle_temp_show, omap_throttle_temp_store);
#define THROTTLE_ATTR_INDEX 1
static struct attribute *omap_temp_sensor_attributes[] = {
	&dev_attr_temperature.attr,
	NULL,
	NULL
};

static const struct attribute_group omap_temp_sensor_group = {
	.attrs = omap_temp_sensor_attributes,
};

static int omap_temp_sensor_enable(struct omap_temp_sensor *temp_sensor)
{
	u32 temp;
	u32 ret = 0;
	unsigned long clk_rate;

	unsigned long flags;

	spin_lock_irqsave(&temp_sensor->lock, flags);

	if (temp_sensor->clk_on) {
		pr_debug("%s: clock already on\n", __func__);
		goto out;
	}

	/* Enables the clock, see _od_runtime_suspend() in
	 * arch/arm/plat-omap/omap_device.c
	 */
	ret = pm_runtime_get_sync(&temp_sensor->pdev->dev);
	if (ret) {
		pr_err("%s:get sync failed\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	clk_set_rate(temp_sensor->clock, OMAP4430_MAX_FREQ);
	clk_rate = clk_get_rate(temp_sensor->clock);
	temp_sensor->clk_rate = clk_rate;

	/* write BGAP_TEMPSOFF should be reset to 0 */
	temp = omap_temp_sensor_readl(temp_sensor,
					TEMP_SENSOR_CTRL_OFFSET);
	temp &= ~(OMAP4430_BGAP_TEMPSOFF_MASK);
	omap_temp_sensor_writel(temp_sensor, temp,
				TEMP_SENSOR_CTRL_OFFSET);

	udelay(5);	/* wait for 5 us */

	omap_enable_continuous_mode(temp_sensor);

	temp_sensor->clk_on = 1;

out:
	spin_unlock_irqrestore(&temp_sensor->lock, flags);
	return ret;
}


static int omap_temp_sensor_disable(struct omap_temp_sensor *temp_sensor)
{
	u32 temp;
	u32 ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&temp_sensor->lock, flags);

	if (!temp_sensor->clk_on) {
		pr_debug("%s: clock already off\n", __func__);
		goto out;
	}
	temp = omap_temp_sensor_readl(temp_sensor,
				TEMP_SENSOR_CTRL_OFFSET);
	temp |= OMAP4430_BGAP_TEMPSOFF_MASK;

	/* write BGAP_TEMPSOFF should be set to 1 before gating clock */
	omap_temp_sensor_writel(temp_sensor, temp,
				TEMP_SENSOR_CTRL_OFFSET);

	/* Gate the clock */
	ret = pm_runtime_put_sync_suspend(&temp_sensor->pdev->dev);
	if (ret) {
		pr_err("%s:put sync failed\n", __func__);
		ret = -EINVAL;
		goto out;
	}
	temp_sensor->clk_on = 0;

out:
	spin_unlock_irqrestore(&temp_sensor->lock, flags);
	return ret;
}

/*
 * Check if the die sensor is cooling down. If it's higher than
 * t_hot since the last throttle then throttle it again.
 * OMAP junction temperature could stay for a long time in an
 * unacceptable temperature range. The idea here is to check after
 * t_hot->throttle the system really came below t_hot else re-throttle
 * and keep doing till it's under t_hot temp range.
 */
static void throttle_delayed_work_fn(struct work_struct *work)
{
	int curr, ret;
	struct omap_temp_sensor *temp_sensor =
				container_of(work, struct omap_temp_sensor,
					     throttle_work.work);
	curr = omap_read_current_temp(temp_sensor);

	mutex_lock(&temp_sensor->throttle_lock);
	if ((curr >= temp_sensor->throttle_hot && temp_sensor->attempting_cool == 0) ||
	    (curr > temp_sensor->throttle_hot && temp_sensor->attempting_cool == 1) ||
	    curr < 0) {

		temp_sensor->throttling++;
		temp_sensor->attempting_cool = 1;
		ret = omap_rethrottle_cpu(temp_sensor, true);
		if (ret < 0) {
			pr_err("%s: OMAP temp read %d exceeds hot threshold, "
				"but could not throttle: %d",
				__func__, curr, ret);
			goto out;
		}
		pr_info("%s: OMAP temp read %d exceeds hot threshold, throttling "
			"at %u MHz\n",
			__func__, curr, temp_sensor->throttle_freq/1000);

	} else if (temp_sensor->throttling > 0 &&
		   ((curr <= temp_sensor->throttle_cold && temp_sensor->attempting_cool == 1) ||
		    (curr <  temp_sensor->throttle_cold && temp_sensor->attempting_cool == 0))) {

		temp_sensor->throttling--;
		temp_sensor->attempting_cool = 0;
		ret = omap_rethrottle_cpu(temp_sensor, false);
		if (ret < 0) {
			pr_err("%s: OMAP temp read %d below cold threshold, "
				"but could not unthrottle: %d",
				__func__, curr, ret);
			goto out;
		}

		pr_info("%s: OMAP temp read %d below cold threshold, unthrottling "
			"at %u MHz\n",
			__func__, curr, temp_sensor->throttle_freq/1000);
	}

out:
	schedule_throttle_work(temp_sensor, curr);
	mutex_unlock(&temp_sensor->throttle_lock);
}

static irqreturn_t omap_tshut_irq_handler(int irq, void *data)
{
	struct omap_temp_sensor *temp_sensor = (struct omap_temp_sensor *)data;

	/* Need to handle thermal mgmt in bootloader
	 * to avoid restart again at kernel level
	 */
	if (temp_sensor->is_efuse_valid) {
		pr_emerg("%s: Thermal shutdown reached, poweroff\n",
			__func__);

		/* Knock it down two steps so we're generating
		 * less heat while shutting down */
		omap_rethrottle_cpu(temp_sensor, true);
		omap_rethrottle_cpu(temp_sensor, true);

		orderly_poweroff(true);
	} else {
		pr_err("%s:Invalid EFUSE, Non-trimmed BGAP\n", __func__);
	}

	return IRQ_HANDLED;
}

static int __devinit omap_temp_sensor_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct omap_temp_sensor_pdata *pdata = pdev->dev.platform_data;
	struct omap_temp_sensor *temp_sensor;
	struct resource *mem;
	int ret = 0, curr;

	if (!pdata) {
		dev_err(dev, "%s: platform data missing\n", __func__);
		return -EINVAL;
	}

	temp_sensor = kzalloc(sizeof(struct omap_temp_sensor), GFP_KERNEL);
	if (!temp_sensor)
		return -ENOMEM;

	spin_lock_init(&temp_sensor->lock);

	if (auto_throttle) {
		/* Get some cpufreq stuff for throttling purposes */
		/* cpufreq_cpu_get() should only be done on CPU0 (the boot cpu).
		 * For other CPUs, the policy is destroyed/created on cpu
		 * hotplug (which happens during suspend). cpufreq_cpu_get()
		 * gets the omap2plus-cpufreq module  */
		temp_sensor->cpufreq_policy = cpufreq_cpu_get(0);
		if (!temp_sensor->cpufreq_policy) {
			dev_err(dev, "%s:No cpufreq driver\n", __func__);
			return -EINVAL;
			goto cpufreq_get_err;
		}

		temp_sensor->freq_table = cpufreq_frequency_get_table(0);
		if (!temp_sensor->freq_table) {
			dev_err(dev, "%s:No cpufreq driver\n", __func__);
			ret = -EINVAL;
			goto freq_table_err;
		}
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(dev, "%s:no mem resource\n", __func__);
		ret = -EINVAL;
		goto plat_res_err;
	}

	ret = gpio_request_one(OMAP_TSHUT_GPIO, GPIOF_DIR_IN,
		"thermal_shutdown");
	if (ret) {
		dev_err(dev, "%s: Could not get tshut_gpio\n",
			__func__);
		temp_sensor->tshut_irq = 0;
	} else {
		temp_sensor->tshut_irq = gpio_to_irq(OMAP_TSHUT_GPIO);
		if (temp_sensor->tshut_irq < 0) {
			dev_err(dev, "%s:Cannot get thermal shutdown irq\n",
				__func__);
			ret = -EINVAL;
			goto get_tshut_irq_err;
		}
	}

	temp_sensor->phy_base = pdata->offset;
	temp_sensor->pdev = pdev;
	temp_sensor->dev = dev;

	pm_runtime_enable(dev);
	pm_runtime_irq_safe(dev);

	/**
	 * check if the efuse has a non-zero value if not
	 * it is an untrimmed sample and the temperatures
	 * may not be accurate */
	if (omap_readl(OMAP4_CTRL_MODULE_CORE +
			OMAP4_CTRL_MODULE_CORE_STD_FUSE_OPP_BGAP))
		temp_sensor->is_efuse_valid = 1;

	temp_sensor->clock = clk_get(&temp_sensor->pdev->dev, "fck");
	if (IS_ERR(temp_sensor->clock)) {
		ret = PTR_ERR(temp_sensor->clock);
		pr_err("%s:Unable to get fclk: %d\n", __func__, ret);
		ret = -EINVAL;
		goto clk_get_err;
	}

	platform_set_drvdata(pdev, temp_sensor);

	ret = omap_temp_sensor_enable(temp_sensor);
	if (ret) {
		dev_err(dev, "%s:Cannot enable temp sensor\n", __func__);
		goto sensor_enable_err;
	}

	/* Wait till the first conversion is done - a conversion takes 54
	 * cycles, which is 1.7 ms with a 32768 Hz clock */
	mdelay(2);

	/* Read the temperature once due to hw issue*/
	curr = omap_read_current_temp(temp_sensor);

	if (temp_sensor->tshut_irq > 0) {
		ret = request_threaded_irq(temp_sensor->tshut_irq, NULL,
				omap_tshut_irq_handler,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"tshut", (void *)temp_sensor);
		if (ret) {
			dev_err(dev, "Request threaded irq failed for TSHUT.\n");
			goto tshut_irq_req_err;
		}
	}

	mutex_init(&temp_sensor->throttle_lock);

	INIT_DELAYED_WORK(&temp_sensor->throttle_work,
			throttle_delayed_work_fn);

	if (auto_throttle) {
		temp_sensor->throttle_cold = THROTTLE_COLD;
		temp_sensor->throttle_hot = THROTTLE_HOT;
		temp_sensor->throttle_freq = 
			cpufreq_get_max_freq(temp_sensor->cpufreq_policy);
		if (temp_sensor->throttle_freq < 0)
			goto cpufreq_freq_err;
		schedule_throttle_work(temp_sensor, curr);
		/* Enable the sysfs throttle threshold control file */
		omap_temp_sensor_attributes[THROTTLE_ATTR_INDEX] =
			&dev_attr_throttle_temp.attr;
	} else {
		/* Enable the sysfs userspace throttle control file */
		omap_temp_sensor_attributes[THROTTLE_ATTR_INDEX] =
			&dev_attr_throttle.attr;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &omap_temp_sensor_group);
	if (ret) {
		dev_err(&pdev->dev, "could not create sysfs files\n");
		goto sysfs_create_err;
	}

	dev_info(dev, "%s probed", pdata->name);

	return 0;

sysfs_create_err:
	cancel_delayed_work_sync(&temp_sensor->throttle_work);
cpufreq_freq_err:
	if (temp_sensor->tshut_irq > 0)
		free_irq(temp_sensor->tshut_irq, temp_sensor);
tshut_irq_req_err:
	omap_temp_sensor_disable(temp_sensor);
sensor_enable_err:
	platform_set_drvdata(pdev, NULL);
	clk_put(temp_sensor->clock);
clk_get_err:
	pm_runtime_disable(dev);
get_tshut_irq_err:
	if (temp_sensor->tshut_irq > 0)
		gpio_free(OMAP_TSHUT_GPIO);
plat_res_err:
freq_table_err:
	if (temp_sensor->cpufreq_policy)
		cpufreq_cpu_put(temp_sensor->cpufreq_policy);
cpufreq_get_err:
	kfree(temp_sensor);
	return ret;
}

static int __devexit omap_temp_sensor_remove(struct platform_device *pdev)
{
	struct omap_temp_sensor *temp_sensor = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &omap_temp_sensor_group);
	cancel_delayed_work_sync(&temp_sensor->throttle_work);
	omap_temp_sensor_disable(temp_sensor);
	clk_put(temp_sensor->clock);
	platform_set_drvdata(pdev, NULL);
 	if (temp_sensor->tshut_irq > 0) {
 		free_irq(temp_sensor->tshut_irq, temp_sensor);
	        gpio_free(OMAP_TSHUT_GPIO);
	}
	kfree(temp_sensor);

	return 0;
}

#ifdef CONFIG_SUSPEND
static int omap_temp_sensor_suspend(struct device *dev)
{
	struct omap_temp_sensor *temp_sensor =
			platform_get_drvdata(to_platform_device(dev));

	omap_temp_sensor_disable(temp_sensor);

	return 0;
}

static int omap_temp_sensor_resume(struct device *dev)
{
	struct omap_temp_sensor *temp_sensor =
			platform_get_drvdata(to_platform_device(dev));

	omap_temp_sensor_enable(temp_sensor);

	return 0;
}

#else
omap_temp_sensor_suspend NULL
omap_temp_sensor_resume NULL

#endif /* CONFIG_SUSPEND */

static const struct dev_pm_ops omap_temp_sensor_dev_pm_ops = {
	.suspend = omap_temp_sensor_suspend,
	.resume = omap_temp_sensor_resume,
	.runtime_suspend = NULL,
	.runtime_resume = NULL,
};

static struct platform_driver omap_temp_sensor_driver = {
	.probe = omap_temp_sensor_probe,
	.remove = omap_temp_sensor_remove,
	.driver = {
		.name = DRIVER_NAME,
		.pm = &omap_temp_sensor_dev_pm_ops,
	},
};

/* from arch/arm/mach-omap2/temp_sensor_device.c */
static struct omap_device_pm_latency omap_temp_sensor_latency[] = {
	{
	 .deactivate_func = NULL, /* set in omap_temp_sensor_init() */
	 .activate_func = NULL,
	 .flags = OMAP_DEVICE_LATENCY_AUTO_ADJUST,
	}
};

/* from arch/arm/mach-omap2/temp_sensor_device.c */
static int temp_sensor_dev_init(struct omap_hwmod *oh, void *user)
{
	struct omap_temp_sensor_pdata *temp_sensor_pdata;
	struct omap_device *od;
	static int i;
	int ret = 0;

	temp_sensor_pdata =
	    kzalloc(sizeof(struct omap_temp_sensor_pdata), GFP_KERNEL);
	if (!temp_sensor_pdata) {
		pr_err
		    ("%s: Unable to allocate memory for %s.Error!\n",
			__func__, oh->name);
		return -ENOMEM;
	}

	temp_sensor_pdata->offset = OMAP4_CTRL_MODULE_CORE_TEMP_SENSOR;

	temp_sensor_pdata->name = DRIVER_NAME;

	od = omap_device_build_s(temp_sensor_pdata->name, i, oh,
				 temp_sensor_pdata,
				 sizeof(*temp_sensor_pdata),
				 omap_temp_sensor_latency,
				 ARRAY_SIZE(omap_temp_sensor_latency), 0);
	if (IS_ERR(od)) {
		pr_warning("%s: Could not build omap_device for %s: %s.\n\n",
			   __func__, temp_sensor_pdata->name, oh->name);
		ret = -EINVAL;
		goto done;
	}

	i++;
done:
	kfree(temp_sensor_pdata);
	return ret;
}


/* From arch/arm/mach-omap2/omap_hwmod_44xx_data.c - modifications to
 * - omap44xx_thermal_sensor_addrs,
 * - omap44xx_l4_cfg__thermal_sensor, and
 * - omap44xx_thermal_sensor_slaves
 */
static struct omap_hwmod_addr_space omap443x_bandgap_addrs[] = {
	{
		.pa_start       = 0x4a00232c,
		.pa_end         = 0x4a00232f,
	},
};

/* l4_cfg -> ctrl_module_core */
static struct omap_hwmod_ocp_if omap44xx_l4_cfg__bandgap = {
	.master         = NULL, /* Set in omap_temp_sensor_init */
	.slave          = NULL, /* Set in omap_temp_sensor_init */
	.clk            = "l4_div_ck",
	.addr           = omap443x_bandgap_addrs,
	.addr_cnt       = ARRAY_SIZE(omap443x_bandgap_addrs),
	.user           = OCP_USER_MPU | OCP_USER_SDMA,
};

/* ctrl_module_core slave ports */
static struct omap_hwmod_ocp_if *omap443x_bandgap_slaves[] = {
	&omap44xx_l4_cfg__bandgap,
};

int __init omap_temp_sensor_init(void)
{
	struct omap_hwmod *omap44xx_l4_cfg_hwmod_p = NULL;
        struct omap_hwmod *omap443x_bandgap_hwmod_p = NULL;
        int ret;

	if (!cpu_is_omap443x()) {
		pr_err("CPU is not OMAP443x\n");
		return 0;
        }

	/* arch/arm/mach-omap2/control.c */
	SYMSEARCH_BIND_FUNCTION_TO(omap_temp_sensor, omap_ctrl_readl, omap_ctrl_readl_s);
	SYMSEARCH_BIND_FUNCTION_TO(omap_temp_sensor, omap_ctrl_writel, omap_ctrl_writel_s);
	/* arch/arm/plat-omap/omap_device.c */
	SYMSEARCH_BIND_FUNCTION_TO(omap_temp_sensor, omap_device_build, omap_device_build_s);
	SYMSEARCH_BIND_FUNCTION_TO(omap_temp_sensor, omap_device_enable_hwmods, omap_device_enable_hwmods_s);
	SYMSEARCH_BIND_FUNCTION_TO(omap_temp_sensor, omap_device_idle_hwmods, omap_device_idle_hwmods_s);
	/* drivers/cpufreq/cpufreq.c */
	SYMSEARCH_BIND_FUNCTION_TO(omap_temp_sensor, lock_policy_rwsem_read, lock_policy_rwsem_read_s);
	SYMSEARCH_BIND_FUNCTION_TO(omap_temp_sensor, unlock_policy_rwsem_read, unlock_policy_rwsem_read_s);
	SYMSEARCH_BIND_FUNCTION_TO(omap_temp_sensor, lock_policy_rwsem_write, lock_policy_rwsem_write_s);
	SYMSEARCH_BIND_FUNCTION_TO(omap_temp_sensor, unlock_policy_rwsem_write, unlock_policy_rwsem_write_s);
	SYMSEARCH_BIND_FUNCTION_TO(omap_temp_sensor, __cpufreq_set_policy, __cpufreq_set_policy_s);
        /* arch/arm/mach-omap2/omap_hwmod_44xx_data.c */
	SYMSEARCH_BIND_POINTER_TO(omap_temp_sensor, struct omap_hwmod*, omap44xx_l4_cfg_hwmod, omap44xx_l4_cfg_hwmod_p);
	SYMSEARCH_BIND_POINTER_TO(omap_temp_sensor, struct omap_hwmod*, omap443x_bandgap_hwmod, omap443x_bandgap_hwmod_p);

	omap44xx_l4_cfg__bandgap.master = omap44xx_l4_cfg_hwmod_p;
	omap44xx_l4_cfg__bandgap.slave = omap443x_bandgap_hwmod_p;
	omap443x_bandgap_hwmod_p->slaves = omap443x_bandgap_slaves;
	omap443x_bandgap_hwmod_p->slaves_cnt = ARRAY_SIZE(omap443x_bandgap_slaves);

        omap_temp_sensor_latency[0].deactivate_func = omap_device_idle_hwmods_s;
        omap_temp_sensor_latency[0].activate_func = omap_device_enable_hwmods_s;

	if ((ret = temp_sensor_dev_init(omap443x_bandgap_hwmod_p, NULL))) {
		pr_err("temp_sensor_dev_init() failed: %d - Forging on! "
			"The device might have already been registered.\n", ret);
	}

	if ((ret = platform_driver_register(&omap_temp_sensor_driver))) {
		pr_err("platform_driver_register() failed: %d\n", ret);
                return ret;
        } else {
                return 0;
        }
}

static void __exit omap_temp_sensor_exit(void)
{
	platform_driver_unregister(&omap_temp_sensor_driver);
}

module_init(omap_temp_sensor_init);
module_exit(omap_temp_sensor_exit);

MODULE_DESCRIPTION("OMAP443X Temperature Sensor Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Alexander Lam (lambchop468 (a) gmail.com)");
