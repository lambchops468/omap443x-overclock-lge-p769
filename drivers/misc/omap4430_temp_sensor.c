/*
 * OMAP4 Temperature sensor driver file
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 * Author: J Keerthy <j-keerthy@ti.com>
 * Author: Moiz Sonasath <m-sonasath@ti.com>
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
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/types.h>

#include <plat/common.h>
#include <plat/omap-pm.h>
#include <plat/omap_device.h>
#include <plat/temperature_sensor.h>
#include <plat/omap-pm.h>

/* TO DO: This needs to be fixed */
#include "../../../../arch/arm/mach-omap2/control.h"
/* #include <plat/control.h> */

#include <mach/ctrl_module_core_44xx.h>

#define DRIVER_NAME "omap443x_temp_sensor"

extern void omap_thermal_throttle(void);
extern void omap_thermal_unthrottle(void);

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
 * @sensor_mutex - Mutex for sysfs, irq and PM
 * @irq - MPU Irq number for thermal alertemp_sensor
 * @tshut_irq -  Thermal shutdown IRQ
 * @phy_base - Physical base of the temp I/O
 * @is_efuse_valid - Flag to determine if eFuse is valid or not
 * @clk_on - Manages the current clock state
 * @clk_rate - Holds current clock rate
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
	u32 current_temp;
};

#ifdef CONFIG_PM
struct omap_temp_sensor_regs {
	u32 temp_sensor_ctrl;
};

static struct omap_temp_sensor_regs temp_sensor_context;
static struct omap_temp_sensor *temp_sensor_pm;
#endif

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
	return omap_ctrl_readl(temp_sensor->phy_base + reg);
}

static void omap_temp_sensor_writel(struct omap_temp_sensor *temp_sensor,
				    u32 val, u32 reg)
{
	omap_ctrl_writel(val, (temp_sensor->phy_base + reg));
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

static int omap_force_single_read(struct omap_temp_sensor *temp_sensor)
{
	u32 temp = 0, counter = 1000;

	/* Select single conversion mode */
	temp = omap_temp_sensor_readl(temp_sensor, TEMP_SENSOR_CTRL_OFFSET);
	temp &= ~(OMAP4430_SINGLE_MODE_MASK);
	omap_temp_sensor_writel(temp_sensor, temp, TEMP_SENSOR_CTRL_OFFSET);

	/* Start of Conversion = 1 */
	temp = omap_temp_sensor_readl(temp_sensor, TEMP_SENSOR_CTRL_OFFSET);
	temp = temp | OMAP4430_BGAP_TEMP_SENSOR_SOC_MASK;
	omap_temp_sensor_writel(temp_sensor, temp, TEMP_SENSOR_CTRL_OFFSET);
	/* Wait until DTEMP is updated */
	temp = omap_read_temp(temp_sensor);

	while ((temp == 0) && --counter)
	        temp = omap_read_temp(temp_sensor);

	/* Start of Conversion = 0 */
	temp = omap_temp_sensor_readl(temp_sensor, TEMP_SENSOR_CTRL_OFFSET);
	temp &= ~(OMAP4430_BGAP_TEMP_SENSOR_SOC_MASK);
	omap_temp_sensor_writel(temp_sensor, temp, TEMP_SENSOR_CTRL_OFFSET);

	return 0;
}

static void omap_enable_continuous_mode(struct omap_temp_sensor *temp_sensor)
{
	u32 val;

	val = omap_temp_sensor_readl(temp_sensor, TEMP_SENSOR_CTRL_OFFSET);

	val = val | OMAP4430_SINGLE_MODE_MASK;

	omap_temp_sensor_writel(temp_sensor, val, TEMP_SENSOR_CTRL_OFFSET);
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

	return sprintf(buf, "%d\n", omap_read_current_temp(temp_sensor));
}

static ssize_t omap_throttle_store(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	if (count && buf[0] == '1')
		omap_thermal_throttle();
	else
		omap_thermal_unthrottle();

	return count;
}

static DEVICE_ATTR(temperature, S_IRUGO, omap_temp_show_current, NULL);
static DEVICE_ATTR(throttle, S_IWUSR, NULL, omap_throttle_store);
static struct attribute *omap_temp_sensor_attributes[] = {
	&dev_attr_temperature.attr,
	&dev_attr_throttle.attr,
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

        /* Perform a single read just before enabling continuous */
	omap_force_single_read(temp_sensor);
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
		omap_thermal_throttle();
                /* Assuming omap_thermal_throttle() not designed to
                 * be called in rapid succession */
                mdelay(5);
		omap_thermal_throttle();

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
	int ret = 0;

	if (!pdata) {
		dev_err(dev, "%s: platform data missing\n", __func__);
		return -EINVAL;
	}

	temp_sensor = kzalloc(sizeof(struct omap_temp_sensor), GFP_KERNEL);
	if (!temp_sensor)
		return -ENOMEM;

	spin_lock_init(&temp_sensor->lock);

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

	/*
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
	omap_read_current_temp(temp_sensor);

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

	ret = sysfs_create_group(&pdev->dev.kobj, &omap_temp_sensor_group);
	if (ret) {
		dev_err(&pdev->dev, "could not create sysfs files\n");
		goto sysfs_create_err;
	}

	dev_info(dev, "%s probed", pdata->name);

	temp_sensor_pm = temp_sensor;

	return 0;

sysfs_create_err:
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
	kfree(temp_sensor);
	return ret;
}

static int __devexit omap_temp_sensor_remove(struct platform_device *pdev)
{
	struct omap_temp_sensor *temp_sensor = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &omap_temp_sensor_group);
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

#ifdef CONFIG_PM
static void omap_temp_sensor_save_ctxt(struct omap_temp_sensor *temp_sensor)
{
	temp_sensor_context.temp_sensor_ctrl =
	    omap_temp_sensor_readl(temp_sensor, TEMP_SENSOR_CTRL_OFFSET);
}

static void omap_temp_sensor_restore_ctxt(struct omap_temp_sensor *temp_sensor)
{
	omap_temp_sensor_writel(temp_sensor,
				temp_sensor_context.temp_sensor_ctrl,
				TEMP_SENSOR_CTRL_OFFSET);
}

static int omap_temp_sensor_suspend(struct platform_device *pdev,
				    pm_message_t state)
{
	struct omap_temp_sensor *temp_sensor = platform_get_drvdata(pdev);

	omap_temp_sensor_disable(temp_sensor);

	return 0;
}

static int omap_temp_sensor_resume(struct platform_device *pdev)
{
	struct omap_temp_sensor *temp_sensor = platform_get_drvdata(pdev);

	omap_temp_sensor_enable(temp_sensor);

	return 0;
}

void omap_temp_sensor_idle(int idle_state)
{
	if (!temp_sensor_pm)
		return;

	if (idle_state)
		omap_temp_sensor_disable(temp_sensor_pm);
	else
		omap_temp_sensor_enable(temp_sensor_pm);
}

#else
omap_temp_sensor_suspend NULL
omap_temp_sensor_resume NULL

#endif /* CONFIG_PM */

static int omap_temp_sensor_runtime_suspend(struct device *dev)
{
	struct omap_temp_sensor *temp_sensor =
			platform_get_drvdata(to_platform_device(dev));

	omap_temp_sensor_save_ctxt(temp_sensor);
	return 0;
}

static int omap_temp_sensor_runtime_resume(struct device *dev)
{
	struct omap_temp_sensor *temp_sensor =
			platform_get_drvdata(to_platform_device(dev));
	if (omap_pm_was_context_lost(dev)) {
		omap_temp_sensor_restore_ctxt(temp_sensor);
	}
	return 0;
}

static const struct dev_pm_ops omap_temp_sensor_dev_pm_ops = {
	.runtime_suspend = omap_temp_sensor_runtime_suspend,
	.runtime_resume = omap_temp_sensor_runtime_resume,
};

static struct platform_driver omap_temp_sensor_driver = {
	.probe = omap_temp_sensor_probe,
	.remove = omap_temp_sensor_remove,
	.suspend = omap_temp_sensor_suspend,
	.resume = omap_temp_sensor_resume,
	.driver = {
		.name = DRIVER_NAME,
		.pm = &omap_temp_sensor_dev_pm_ops,
	},
};

int __init omap_temp_sensor_init(void)
{
        int ret;

	if (!cpu_is_omap443x()) {
		pr_err("CPU is not OMAP443x\n");
		return 0;
        }

	if ((ret = platform_driver_register(&omap_temp_sensor_driver))) {
		pr_err("platform_driver_register() failed\n");
        }

        return ret;
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
MODULE_AUTHOR("Texas Instruments Inc");
