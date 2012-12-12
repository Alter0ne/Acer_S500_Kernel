/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 * msm_dsps - control DSPS clocks, gpios and vregs.
 *
 */

#include <linux/types.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/msm_dsps.h>

#include <mach/irqs.h>
#include <mach/peripheral-loader.h>
#include <mach/msm_iomap.h>
#include <mach/msm_smsm.h>
#include <mach/msm_dsps.h>
#include <mach/subsystem_restart.h>
#include <mach/subsystem_notif.h>

#include "timer.h"

#define DRV_NAME	"msm_dsps"
#define DRV_VERSION	"3.03"


#define PPSS_TIMER0_32KHZ_REG	0x1004
#define PPSS_TIMER0_20MHZ_REG	0x0804

#ifdef DSPS_DEVINFO
#define MAX_VENDOR_NAME_SIZE 40

#define devinfo_attr(_name, _prefix)			\
static struct kobj_attribute _prefix##_attr = {		\
	.attr = {									\
			.name = __stringify(_name),		\
			.mode = 0644,					\
	},										\
	.show = _prefix##_show,					\
}

#define attr(_prefix)							\
static struct attribute * _prefix##_group[] = {		\
	&_prefix##_vendor_attr.attr,				\
	&_prefix##_chip_attr.attr,					\
	NULL,									\
}

#define attr_group(_prefix)						\
static struct attribute_group _prefix##_attr_group = {	\
	.attrs = _prefix##_group,						\
}

static u8 accel_vendor_info[MAX_VENDOR_NAME_SIZE];
static u8 gyro_vendor_info[MAX_VENDOR_NAME_SIZE];
static u8 mag_vendor_info[MAX_VENDOR_NAME_SIZE];
static u8 prox_light_vendor_info[MAX_VENDOR_NAME_SIZE];

static u8 accel_chip_info = 0;
static u8 gyro_chip_info = 0;
static u8 mag_chip_info = 0;
static u8 prox_light_chip_info = 0;

static struct kobject *accel_dev_info_kobj;
static struct kobject *gyro_dev_info_kobj;
static struct kobject *mag_dev_info_kobj;
static struct kobject *prox_light_dev_info_kobj;

static ssize_t accel_vendor_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	char *s = buf;
	if (accel_vendor_info[0] != 0)
		s += sprintf(s, "%s\n", accel_vendor_info);
	else
		s += sprintf(s, "%s\n", "unknown");
	return (s - buf);
}

static ssize_t gyro_vendor_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	char *s = buf;
	if (gyro_vendor_info[0] != 0)
		s += sprintf(s, "%s\n", gyro_vendor_info);
	else
		s += sprintf(s, "%s\n", "unknown");
	return (s - buf);
}

static ssize_t mag_vendor_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	char *s = buf;
	if (mag_vendor_info[0] != 0)
		s += sprintf(s, "%s\n", mag_vendor_info);
	else
		s += sprintf(s, "%s\n", "unknown");
	return (s - buf);
}

static ssize_t prox_light_vendor_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	char *s = buf;
	if (prox_light_vendor_info[0] != 0)
		s += sprintf(s, "%s\n", prox_light_vendor_info);
	else
		s += sprintf(s, "%s\n", "unknown");
	return (s - buf);
}

static ssize_t accel_chip_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	char *s = buf;
	if (accel_chip_info != 0)
		s += sprintf(s, "0x%02x\n", accel_chip_info);
	else
		s += sprintf(s, "%s\n", "unknown");
	return (s - buf);
}

static ssize_t gyro_chip_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	char *s = buf;
	if (gyro_chip_info != 0)
		s += sprintf(s, "0x%02x\n", gyro_chip_info);
	else
		s += sprintf(s, "%s\n", "unknown");
	return (s - buf);
}

static ssize_t mag_chip_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	char *s = buf;
	if (mag_chip_info != 0)
		s += sprintf(s, "0x%02x\n", mag_chip_info);
	else
		s += sprintf(s, "%s\n", "unknown");
	return (s - buf);
}

static ssize_t prox_light_chip_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	char *s = buf;
	if (prox_light_chip_info != 0)
		s += sprintf(s, "0x%02x\n", prox_light_chip_info);
	else
		s += sprintf(s, "%s\n", "unknown");
	return (s - buf);
}

devinfo_attr(vendor, accel_vendor);
devinfo_attr(vendor, gyro_vendor);
devinfo_attr(vendor, mag_vendor);
devinfo_attr(vendor, prox_light_vendor);

devinfo_attr(chip, accel_chip);
devinfo_attr(chip, gyro_chip);
devinfo_attr(chip, mag_chip);
devinfo_attr(chip, prox_light_chip);

attr(accel);
attr(gyro);
attr(mag);
attr(prox_light);

attr_group(accel);
attr_group(gyro);
attr_group(mag);
attr_group(prox_light);
#endif

/**
 *  Driver Context
 *
 *  @dev_class - device class.
 *  @dev_num - device major & minor number.
 *  @dev - the device.
 *  @cdev - character device for user interface.
 *  @pdata - platform data.
 *  @pil - handle to DSPS Firmware loader.
 *  @is_on - DSPS is on.
 *  @ref_count - open/close reference count.
 *  @ppss_base - ppss registers virtual base address.
 */
struct dsps_drv {

	struct class *dev_class;
	dev_t dev_num;
	struct device *dev;
	struct cdev *cdev;

	struct msm_dsps_platform_data *pdata;

	void *pil;

	int is_on;
	int ref_count;

	void __iomem *ppss_base;
};

/**
 * Driver context.
 */
static struct dsps_drv *drv;

/**
 * self-initiated shutdown flag
 */
static int dsps_crash_shutdown_g;


static void dsps_fatal_handler(struct work_struct *work);

/**
 *  Load DSPS Firmware.
 */
static int dsps_load(const char *name)
{
	pr_debug("%s.\n", __func__);

	drv->pil = pil_get(name);

	if (IS_ERR(drv->pil)) {
		pr_err("%s: fail to load DSPS firmware %s.\n", __func__, name);
		return -ENODEV;
	}
	msleep(20);
	return 0;
}

/**
 *  Unload DSPS Firmware.
 */
static void dsps_unload(void)
{
	pr_debug("%s.\n", __func__);

	pil_put(drv->pil);
}

/**
 *  Suspend DSPS CPU.
 *
 * Only call if dsps_pwr_ctl_en is false.
 * If dsps_pwr_ctl_en is true, then DSPS will control its own power state.
 */
static void dsps_suspend(void)
{
	pr_debug("%s.\n", __func__);

	writel_relaxed(1, drv->ppss_base + drv->pdata->ppss_pause_reg);
	mb(); /* Make sure write commited before ioctl returns. */
}

/**
 *  Resume DSPS CPU.
 *
 * Only call if dsps_pwr_ctl_en is false.
 * If dsps_pwr_ctl_en is true, then DSPS will control its own power state.
 */
static void dsps_resume(void)
{
	pr_debug("%s.\n", __func__);

	writel_relaxed(0, drv->ppss_base + drv->pdata->ppss_pause_reg);
	mb(); /* Make sure write commited before ioctl returns. */
}

/**
 * Read DSPS slow timer.
 */
static u32 dsps_read_slow_timer(void)
{
	u32 val;

	/* Read the timer value from the MSM sclk. The MSM slow clock & DSPS
	 * timers are in sync, so these are the same value */
	val = msm_timer_get_sclk_ticks();
	pr_debug("%s.count=%d.\n", __func__, val);

	return val;
}

/**
 * Read DSPS fast timer.
 */
static u32 dsps_read_fast_timer(void)
{
	u32 val;

	val = readl_relaxed(drv->ppss_base + PPSS_TIMER0_20MHZ_REG);
	rmb(); /* order reads from the user output buffer */

	pr_debug("%s.count=%d.\n", __func__, val);

	return val;
}

/**
 *  Power on request.
 *
 *  Set clocks to ON.
 *  Set sensors chip-select GPIO to non-reset (on) value.
 *
 */
static int dsps_power_on_handler(void)
{
	int ret = 0;
	int i, ci, gi, ri;

	pr_debug("%s.\n", __func__);

	if (drv->is_on) {
		pr_debug("%s: already ON.\n",  __func__);
		return 0;
	}

	for (ci = 0; ci < drv->pdata->clks_num; ci++) {
		const char *name = drv->pdata->clks[ci].name;
		u32 rate = drv->pdata->clks[ci].rate;
		struct clk *clock = drv->pdata->clks[ci].clock;

		if (clock == NULL)
			continue;

		if (rate > 0) {
			ret = clk_set_rate(clock, rate);
			pr_debug("%s: clk %s set rate %d.",
				__func__, name, rate);
			if (ret) {
				pr_err("%s: clk %s set rate %d. err=%d.",
					__func__, name, rate, ret);
				goto clk_err;
			}

		}

		ret = clk_enable(clock);
		if (ret) {
			pr_err("%s: enable clk %s err %d.",
			       __func__, name, ret);
			goto clk_err;
		}
	}

	for (gi = 0; gi < drv->pdata->gpios_num; gi++) {
		const char *name = drv->pdata->gpios[gi].name;
		int num = drv->pdata->gpios[gi].num;
		int val = drv->pdata->gpios[gi].on_val;
		int is_owner = drv->pdata->gpios[gi].is_owner;

		if (!is_owner)
			continue;

		ret = gpio_direction_output(num, val);
		if (ret) {
			pr_err("%s: set GPIO %s num %d to %d err %d.",
			       __func__, name, num, val, ret);
			goto gpio_err;
		}
	}

	for (ri = 0; ri < drv->pdata->regs_num; ri++) {
		const char *name = drv->pdata->regs[ri].name;
		struct regulator *reg = drv->pdata->regs[ri].reg;
		int volt = drv->pdata->regs[ri].volt;

		if (reg == NULL)
			continue;

		pr_debug("%s: set regulator %s.", __func__, name);

		ret = regulator_set_voltage(reg, volt, volt);

		if (ret) {
			pr_err("%s: set regulator %s voltage %d err = %d.\n",
				__func__, name, volt, ret);
			goto reg_err;
		}

		ret = regulator_enable(reg);
		if (ret) {
			pr_err("%s: enable regulator %s err = %d.\n",
				__func__, name, ret);
			goto reg_err;
		}
	}

	drv->is_on = true;

	return 0;

	/*
	 * If failling to set ANY clock/gpio/regulator to ON then we set
	 * them back to OFF to avoid consuming power for unused
	 * clocks/gpios/regulators.
	 */
reg_err:
	for (i = 0; i < ri; i++) {
		struct regulator *reg = drv->pdata->regs[ri].reg;

		if (reg == NULL)
			continue;

		regulator_disable(reg);
	}

gpio_err:
	for (i = 0; i < gi; i++) {
		int num = drv->pdata->gpios[i].num;
		int val = drv->pdata->gpios[i].off_val;
		int is_owner = drv->pdata->gpios[i].is_owner;

		if (!is_owner)
			continue;

		ret = gpio_direction_output(num, val);
	}

clk_err:
	for (i = 0; i < ci; i++) {
		struct clk *clock = drv->pdata->clks[i].clock;

		if (clock == NULL)
			continue;

		clk_disable(clock);
	}

	return -ENODEV;
}

/**
 *  Power off request.
 *
 *  Set clocks to OFF.
 *  Set sensors chip-select GPIO to reset (off) value.
 *
 */
static int dsps_power_off_handler(void)
{
	int ret;
	int i;

	pr_debug("%s.\n", __func__);

	if (!drv->is_on) {
		pr_debug("%s: already OFF.\n", __func__);
		return 0;
	}

	for (i = 0; i < drv->pdata->clks_num; i++)
		if (drv->pdata->clks[i].clock) {
			const char *name = drv->pdata->clks[i].name;

			pr_debug("%s: set clk %s off.", __func__, name);
			clk_disable(drv->pdata->clks[i].clock);
		}

	for (i = 0; i < drv->pdata->regs_num; i++)
		if (drv->pdata->regs[i].reg) {
			const char *name = drv->pdata->regs[i].name;

			pr_debug("%s: set regulator %s off.", __func__, name);
			regulator_disable(drv->pdata->regs[i].reg);
		}

	/* Clocks on/off has reference count but GPIOs don't. */
	drv->is_on = false;

	for (i = 0; i < drv->pdata->gpios_num; i++) {
		const char *name = drv->pdata->gpios[i].name;
		int num = drv->pdata->gpios[i].num;
		int val = drv->pdata->gpios[i].off_val;

		pr_debug("%s: set gpio %s off.", __func__, name);

		ret = gpio_direction_output(num, val);
		if (ret) {
			pr_err("%s: set GPIO %s err %d.", __func__, name, ret);
			return ret;
		}
	}

	return 0;
}

static DECLARE_WORK(dsps_fatal_work, dsps_fatal_handler);

/**
 *  Watchdog interrupt handler
 *
 */
static irqreturn_t dsps_wdog_bite_irq(int irq, void *dev_id)
{
	pr_debug("%s\n", __func__);
	(void)schedule_work(&dsps_fatal_work);
	disable_irq_nosync(irq);
	return IRQ_HANDLED;
}

/**
 * IO Control - handle commands from client.
 *
 */
static long dsps_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	u32 val = 0;
	#ifdef DSPS_DEVINFO
	u8 vendor_info[MAX_VENDOR_NAME_SIZE];
	#endif

	pr_debug("%s.\n", __func__);

	switch (cmd) {
	case DSPS_IOCTL_ON:
		if (!drv->pdata->dsps_pwr_ctl_en) {
			ret = dsps_power_on_handler();
			dsps_resume();
		}
		break;
	case DSPS_IOCTL_OFF:
		if (!drv->pdata->dsps_pwr_ctl_en) {
			dsps_suspend();
			ret = dsps_power_off_handler();
		}
		break;
	case DSPS_IOCTL_READ_SLOW_TIMER:
		val = dsps_read_slow_timer();
		ret = put_user(val, (u32 __user *) arg);
		break;
	case DSPS_IOCTL_READ_FAST_TIMER:
		val = dsps_read_fast_timer();
		ret = put_user(val, (u32 __user *) arg);
		break;
	case DSPS_IOCTL_RESET:
		dsps_fatal_handler(NULL);
		ret = 0;
		break;
	#ifdef DSPS_DEVINFO
	case DSPS_IOCTL_WRITE_ACCEL_VENDOR_INFO:
		memset(vendor_info, 0, MAX_VENDOR_NAME_SIZE);
		ret = copy_from_user(vendor_info, (void __user*) arg, MAX_VENDOR_NAME_SIZE);
		sprintf((char *)accel_vendor_info, "%s", (char *)vendor_info);
		break;
	case DSPS_IOCTL_WRITE_MAG_VENDOR_INFO:
		memset(vendor_info, 0, MAX_VENDOR_NAME_SIZE);
		ret = copy_from_user(vendor_info, (void __user*) arg, MAX_VENDOR_NAME_SIZE);
		sprintf((char *)mag_vendor_info, "%s", (char *)vendor_info);
		break;
	case DSPS_IOCTL_WRITE_GYRO_VENDOR_INFO:
		memset(vendor_info, 0, MAX_VENDOR_NAME_SIZE);
		ret = copy_from_user(vendor_info, (void __user*) arg, MAX_VENDOR_NAME_SIZE);
		sprintf((char *)gyro_vendor_info, "%s", (char *)vendor_info);
		break;
	case DSPS_IOCTL_WRITE_PROX_LIGHT_VENDOR_INFO:
		memset(vendor_info, 0, MAX_VENDOR_NAME_SIZE);
		ret = copy_from_user(vendor_info, (void __user*) arg, MAX_VENDOR_NAME_SIZE);
		sprintf((char *)prox_light_vendor_info, "%s", (char *)vendor_info);
		break;
	case DSPS_IOCTL_WRITE_ACCEL_CHIP_INFO:
		ret = get_user(accel_chip_info, (u8 __user *) arg);
		break;
	case DSPS_IOCTL_WRITE_MAG_CHIP_INFO:
		ret = get_user(mag_chip_info, (u8 __user *) arg);
		break;
	case DSPS_IOCTL_WRITE_GYRO_CHIP_INFO:
		ret = get_user(gyro_chip_info, (u8 __user *) arg);
		break;
	case DSPS_IOCTL_WRITE_PROX_LIGHT_CHIP_INFO:
		ret = get_user(prox_light_chip_info, (u8 __user *) arg);
		break;
	#endif
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

/**
 * allocate resources.
 * @pdev - pointer to platform device.
 */
static int dsps_alloc_resources(struct platform_device *pdev)
{
	int ret = -ENODEV;
	struct resource *ppss_res;
	struct resource *ppss_wdog;
	int i;

	pr_debug("%s.\n", __func__);

	if ((drv->pdata->signature != DSPS_SIGNATURE)) {
		pr_err("%s: invalid signature for pdata.", __func__);
		return -EINVAL;
	}

	ppss_res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						"ppss_reg");
	if (!ppss_res) {
		pr_err("%s: failed to get ppss_reg resource.\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < drv->pdata->clks_num; i++) {
		const char *name = drv->pdata->clks[i].name;
		struct clk *clock;

		drv->pdata->clks[i].clock = NULL;

		pr_debug("%s: get clk %s.", __func__, name);

		clock = clk_get(drv->dev, name);
		if (IS_ERR(clock)) {
			pr_err("%s: can't get clk %s.", __func__, name);
			goto clk_err;
		}
		drv->pdata->clks[i].clock = clock;
	}

	for (i = 0; i < drv->pdata->gpios_num; i++) {
		const char *name = drv->pdata->gpios[i].name;
		int num = drv->pdata->gpios[i].num;

		drv->pdata->gpios[i].is_owner = false;

		pr_debug("%s: get gpio %s.", __func__, name);

		ret = gpio_request(num, name);
		if (ret) {
			pr_err("%s: request GPIO %s err %d.",
			       __func__, name, ret);
			goto gpio_err;
		}

		drv->pdata->gpios[i].is_owner = true;

	}

	for (i = 0; i < drv->pdata->regs_num; i++) {
		const char *name = drv->pdata->regs[i].name;

		drv->pdata->regs[i].reg = NULL;

		pr_debug("%s: get regulator %s.", __func__, name);

		drv->pdata->regs[i].reg = regulator_get(drv->dev, name);
		if (IS_ERR(drv->pdata->regs[i].reg)) {
			pr_err("%s: get regulator %s failed.",
			       __func__, name);
			goto reg_err;
		}
	}

	drv->ppss_base = ioremap(ppss_res->start,
				 resource_size(ppss_res));

	ppss_wdog = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
						"ppss_wdog");
	if (ppss_wdog) {
		ret = request_irq(ppss_wdog->start, dsps_wdog_bite_irq,
				  IRQF_TRIGGER_RISING, "dsps_wdog", NULL);
		if (ret) {
			pr_err("%s: request_irq fail %d\n", __func__, ret);
			goto request_irq_err;
		}
	} else {
		pr_debug("%s: ppss_wdog not supported.\n", __func__);
	}

	if (drv->pdata->init)
		drv->pdata->init(drv->pdata);

	return 0;

request_irq_err:
	iounmap(drv->ppss_base);

reg_err:
	for (i = 0; i < drv->pdata->regs_num; i++) {
		if (drv->pdata->regs[i].reg) {
			regulator_put(drv->pdata->regs[i].reg);
			drv->pdata->regs[i].reg = NULL;
		}
	}

gpio_err:
	for (i = 0; i < drv->pdata->gpios_num; i++)
		if (drv->pdata->gpios[i].is_owner) {
			gpio_free(drv->pdata->gpios[i].num);
			drv->pdata->gpios[i].is_owner = false;
		}
clk_err:
	for (i = 0; i < drv->pdata->clks_num; i++)
		if (drv->pdata->clks[i].clock) {
			clk_put(drv->pdata->clks[i].clock);
			drv->pdata->clks[i].clock = NULL;
		}

	return ret;
}

/**
 * Open File.
 *
 */
static int dsps_open(struct inode *ip, struct file *fp)
{
	int ret = 0;

	pr_debug("%s.\n", __func__);

	if (drv->ref_count == 0) {

		/* clocks must be ON before loading.*/
		ret = dsps_power_on_handler();
		if (ret)
			return ret;

		ret = dsps_load(drv->pdata->pil_name);

		if (ret) {
			dsps_power_off_handler();
			return ret;
		}

		if (!drv->pdata->dsps_pwr_ctl_en)
			dsps_resume();
	}
	drv->ref_count++;

	return ret;
}

/**
 * free resources.
 *
 */
static void dsps_free_resources(void)
{
	int i;

	pr_debug("%s.\n", __func__);

	for (i = 0; i < drv->pdata->clks_num; i++)
		if (drv->pdata->clks[i].clock) {
			clk_put(drv->pdata->clks[i].clock);
			drv->pdata->clks[i].clock = NULL;
		}

	for (i = 0; i < drv->pdata->gpios_num; i++)
		if (drv->pdata->gpios[i].is_owner) {
			gpio_free(drv->pdata->gpios[i].num);
			drv->pdata->gpios[i].is_owner = false;
		}

	for (i = 0; i < drv->pdata->regs_num; i++) {
		if (drv->pdata->regs[i].reg) {
			regulator_put(drv->pdata->regs[i].reg);
			drv->pdata->regs[i].reg = NULL;
		}
	}

	iounmap(drv->ppss_base);
}

/**
 * Close File.
 *
 * The client shall close and re-open the file for re-loading the DSPS
 * firmware.
 * The file system will close the file if the user space app has crashed.
 *
 * If the DSPS is running, then we must reset DSPS CPU & HW before
 * setting the clocks off.
 * The DSPS reset should be done as part of the pil_put().
 * The DSPS reset should be used for error recovery if the DSPS firmware
 * has crashed and re-loading the firmware is required.
 */
static int dsps_release(struct inode *inode, struct file *file)
{
	pr_debug("%s.\n", __func__);

	drv->ref_count--;

	if (drv->ref_count == 0) {
		if (!drv->pdata->dsps_pwr_ctl_en) {
			dsps_suspend();

			dsps_unload();

			dsps_power_off_handler();
		}
	}

	return 0;
}

const struct file_operations dsps_fops = {
	.owner = THIS_MODULE,
	.open = dsps_open,
	.release = dsps_release,
	.unlocked_ioctl = dsps_ioctl,
};

/**
 *  Fatal error handler
 *  Resets DSPS.
 */
static void dsps_fatal_handler(struct work_struct *work)
{
	uint32_t dsps_state;

	dsps_state = smsm_get_state(SMSM_DSPS_STATE);

	pr_debug("%s: DSPS state 0x%x\n", __func__, dsps_state);

	if (dsps_state & SMSM_RESET) {
		pr_err("%s: DSPS fatal error detected. Resetting\n",
		       __func__);
		panic("DSPS fatal error detected.");
	} else {
		pr_debug("%s: User-initiated DSPS reset. Resetting\n",
			 __func__);
		panic("User-initiated DSPS reset.");
	}
}


/**
 *  SMSM state change callback
 *
 */
static void dsps_smsm_state_cb(void *data, uint32_t old_state,
			       uint32_t new_state)
{
	pr_debug("%s\n", __func__);
	if (dsps_crash_shutdown_g == 1) {
		pr_debug("%s: SMSM_RESET state change ignored\n",
			 __func__);
		dsps_crash_shutdown_g = 0;
		return;
	}

	if (new_state & SMSM_RESET) {
		pr_err
		    ("%s: SMSM_RESET state detected. restarting the DSPS\n",
		     __func__);
		panic("SMSM_RESET state detected.");
	}
}

/**
 *  Shutdown function
 * called by the restart notifier
 *
 */
static int dsps_shutdown(const struct subsys_data *subsys)
{
	pr_debug("%s\n", __func__);
	dsps_unload();
	return 0;
}

/**
 *  Powerup function
 * called by the restart notifier
 *
 */
static int dsps_powerup(const struct subsys_data *subsys)
{
	pr_debug("%s\n", __func__);
	if (dsps_load(drv->pdata->pil_name) != 0) {
		pr_err("%s: fail to restart DSPS after reboot\n",
		       __func__);
		return 1;
	}
	return 0;
}

/**
 *  Crash shutdown function
 * called by the restart notifier
 *
 */
static void dsps_crash_shutdown(const struct subsys_data *subsys)
{
	pr_debug("%s\n", __func__);
	dsps_crash_shutdown_g = 1;
	smsm_change_state(SMSM_DSPS_STATE, SMSM_RESET, SMSM_RESET);
}

/**
 *  Ramdump function
 * called by the restart notifier
 *
 */
static int dsps_ramdump(int enable, const struct subsys_data *subsys)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static struct subsys_data dsps_ssrops = {
	.name = "dsps",
	.shutdown = dsps_shutdown,
	.powerup = dsps_powerup,
	.ramdump = dsps_ramdump,
	.crash_shutdown = dsps_crash_shutdown
};

/**
 * platform driver
 *
 */
static int __devinit dsps_probe(struct platform_device *pdev)
{
	int ret;

	pr_debug("%s.\n", __func__);

	if (pdev->dev.platform_data == NULL) {
		pr_err("%s: platform data is NULL.\n", __func__);
		return -ENODEV;
	}

	drv = kzalloc(sizeof(*drv), GFP_KERNEL);
	if (drv == NULL) {
		pr_err("%s: kzalloc fail.\n", __func__);
		goto alloc_err;
	}
	drv->pdata = pdev->dev.platform_data;

	drv->dev_class = class_create(THIS_MODULE, DRV_NAME);
	if (drv->dev_class == NULL) {
		pr_err("%s: class_create fail.\n", __func__);
		goto res_err;
	}

	ret = alloc_chrdev_region(&drv->dev_num, 0, 1, DRV_NAME);
	if (ret) {
		pr_err("%s: alloc_chrdev_region fail.\n", __func__);
		goto alloc_chrdev_region_err;
	}

	drv->dev = device_create(drv->dev_class, NULL,
				     drv->dev_num,
				     drv, DRV_NAME);
	if (IS_ERR(drv->dev)) {
		pr_err("%s: device_create fail.\n", __func__);
		goto device_create_err;
	}

	drv->cdev = cdev_alloc();
	if (drv->cdev == NULL) {
		pr_err("%s: cdev_alloc fail.\n", __func__);
		goto cdev_alloc_err;
	}
	cdev_init(drv->cdev, &dsps_fops);
	drv->cdev->owner = THIS_MODULE;

	ret = cdev_add(drv->cdev, drv->dev_num, 1);
	if (ret) {
		pr_err("%s: cdev_add fail.\n", __func__);
		goto cdev_add_err;
	}

	ret = dsps_alloc_resources(pdev);
	if (ret) {
		pr_err("%s: failed to allocate dsps resources.\n", __func__);
		goto cdev_add_err;
	}

	ret =
	    smsm_state_cb_register(SMSM_DSPS_STATE, SMSM_RESET,
				   dsps_smsm_state_cb, 0);
	if (ret) {
		pr_err("%s: smsm_state_cb_register fail %d\n", __func__,
		       ret);
		goto smsm_register_err;
	}

	ret = ssr_register_subsystem(&dsps_ssrops);
	if (ret) {
		pr_err("%s: ssr_register_subsystem fail %d\n", __func__,
		       ret);
		goto ssr_register_err;
	}

	#ifdef DSPS_DEVINFO
	accel_dev_info_kobj = kobject_create_and_add("dev_info_g-sensor", NULL);
	if (accel_dev_info_kobj == NULL) {
		pr_err("dev_info_g-sensor: Create kobject failed\n");
	} else {
		ret = sysfs_create_group(accel_dev_info_kobj, &accel_attr_group);
		if (ret)
			pr_err("dev_info_g-sensor: Create accel_attr_group failed\n");
	}

	gyro_dev_info_kobj = kobject_create_and_add("dev_info_gyro-sensor", NULL);
	if (gyro_dev_info_kobj == NULL) {
		pr_info("dev_info_gyro-sensor: Create kobject failed\n");
	} else {
		ret = sysfs_create_group(gyro_dev_info_kobj, &gyro_attr_group);
		if (ret)
			pr_info("dev_info_gyro-sensor: Create gyro_attr_group failed\n");
	}

	mag_dev_info_kobj = kobject_create_and_add("dev_info_e-compass", NULL);
	if (mag_dev_info_kobj == NULL) {
		pr_info("dev_info_e-compass: Create kobject failed\n");
	} else {
		ret = sysfs_create_group(mag_dev_info_kobj, &mag_attr_group);
		if (ret)
			pr_info("dev_info_e-compass: Create mag_attr_group failed\n");
	}

       prox_light_dev_info_kobj = kobject_create_and_add("dev_info_prox_light-sensor", NULL);
	if (prox_light_dev_info_kobj == NULL) {
		pr_info("dev_info_prox_light-sensor: Create kobject failed\n");
	} else {
		ret = sysfs_create_group(prox_light_dev_info_kobj, &prox_light_attr_group);
		if (ret)
			pr_info("dev_info_prox_light-sensor: Create mag_attr_group failed\n");
	}

	memset(accel_vendor_info, 0, MAX_VENDOR_NAME_SIZE);
	memset(gyro_vendor_info, 0, MAX_VENDOR_NAME_SIZE);
	memset(mag_vendor_info, 0, MAX_VENDOR_NAME_SIZE);
	memset(prox_light_vendor_info, 0, MAX_VENDOR_NAME_SIZE);
	#endif

	return 0;

ssr_register_err:
	smsm_state_cb_deregister(SMSM_DSPS_STATE, SMSM_RESET,
				 dsps_smsm_state_cb,
				 0);
smsm_register_err:
	cdev_del(drv->cdev);
cdev_add_err:
	kfree(drv->cdev);
cdev_alloc_err:
	device_destroy(drv->dev_class, drv->dev_num);
device_create_err:
	unregister_chrdev_region(drv->dev_num, 1);
alloc_chrdev_region_err:
	class_destroy(drv->dev_class);
res_err:
	kfree(drv);
	drv = NULL;
alloc_err:
	return -ENODEV;
}

static int __devexit dsps_remove(struct platform_device *pdev)
{
	pr_debug("%s.\n", __func__);

	dsps_power_off_handler();
	dsps_free_resources();

	cdev_del(drv->cdev);
	kfree(drv->cdev);
	drv->cdev = NULL;
	device_destroy(drv->dev_class, drv->dev_num);
	unregister_chrdev_region(drv->dev_num, 1);
	class_destroy(drv->dev_class);
	kfree(drv);
	drv = NULL;

	return 0;
}

static struct platform_driver dsps_driver = {
	.probe          = dsps_probe,
	.remove         = __exit_p(dsps_remove),
	.driver         = {
		.name   = "msm_dsps",
	},
};

/**
 * Module Init.
 */
static int __init dsps_init(void)
{
	int ret;

	pr_info("%s driver version %s.\n", DRV_NAME, DRV_VERSION);

	ret = platform_driver_register(&dsps_driver);

	if (ret)
		pr_err("dsps_init.err=%d.\n", ret);

	return ret;
}

/**
 * Module Exit.
 */
static void __exit dsps_exit(void)
{
	pr_debug("%s.\n", __func__);

	platform_driver_unregister(&dsps_driver);
}

module_init(dsps_init);
module_exit(dsps_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Dedicated Sensors Processor Subsystem (DSPS) driver");
MODULE_AUTHOR("Amir Samuelov <amirs@codeaurora.org>");

