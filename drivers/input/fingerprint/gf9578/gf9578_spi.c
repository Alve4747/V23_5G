/*
 * TEE driver for goodix fingerprint sensor
 * Copyright (C) 2016 Goodix
 *
 * This program is free software; you can redistribute it and / or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt)		"[FP_KERN] " KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/ktime.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/timer.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/pm_qos.h>
#include <linux/cpufreq.h>
#include <linux/pm_wakeup.h>


#if IS_ENABLED(CONFIG_BBK_FINGERPRINT)
#include "../fp_id.h"
#endif

#include "gf9578_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

#define VER_MAJOR   1
#define VER_MINOR   2
#define PATCH_LEVEL 1

#define WAKELOCK_HOLD_TIME 500 /* in ms */

/*device name after register in character */
#define GF_DEV_NAME			"goodix_fp"
#define	GF_INPUT_NAME		"goodixfp"	/*"goodix_fp" */

#define	CHRD_DRIVER_NAME	"goodix_fp_spi"
#define	CLASS_NAME			"goodix_fp"

#define N_SPI_MINORS		32	/* ... up to 256 */
static int SPIDEV_MAJOR;

static DECLARE_BITMAP(minors, N_SPI_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static struct gf_dev gf;
static uint8_t support_soft_fpid;

static int gf_remove(struct platform_device *pdev);

/*vivo qishuangcheng add begin */
struct vivo_wakeup {
	/*kernel mode: synchronize with hal */
	uint32_t ff_mode_state;
	uint32_t current_mode;
	/*reset */
	uint8_t reset_irq_flag;
	spinlock_t lock_reset_irq;
	/*wakelock when recognizing in ff mode */
	struct wakeup_source *fp_wakelock;
};
static struct vivo_wakeup vivo_wakeup_t;
/*vivo qishuangcheng add end */

struct gf_key_map gf9578_key_maps[] = {
	{ EV_KEY, GF_KEY_INPUT_HOME },
	{ EV_KEY, GF_KEY_INPUT_MENU },
	{ EV_KEY, GF_KEY_INPUT_BACK },
	{ EV_KEY, GF_KEY_INPUT_POWER },
#if defined(SUPPORT_NAV_EVENT)
	{ EV_KEY, GF_NAV_INPUT_UP },
	{ EV_KEY, GF_NAV_INPUT_DOWN },
	{ EV_KEY, GF_NAV_INPUT_RIGHT },
	{ EV_KEY, GF_NAV_INPUT_LEFT },
	{ EV_KEY, GF_KEY_INPUT_CAMERA },
	{ EV_KEY, GF_NAV_INPUT_CLICK },
	{ EV_KEY, GF_NAV_INPUT_DOUBLE_CLICK },
	{ EV_KEY, GF_NAV_INPUT_LONG_PRESS },
	{ EV_KEY, GF_NAV_INPUT_HEAVY },
#endif
	{ EV_KEY, GF_KEY_INPUT_WAKE },
};

void gf9578_enable_irq(struct gf_dev *gf_dev)
{
	if (gf_dev->irq_enabled) {
		pr_warn("IRQ has been enabled");
	} else {
		enable_irq(gf_dev->irq);
		gf_dev->irq_enabled = 1;
	}
}

void gf9578_disable_irq(struct gf_dev *gf_dev)
{
	if (gf_dev->irq_enabled) {
		gf_dev->irq_enabled = 0;
		disable_irq(gf_dev->irq);
	} else {
		pr_warn("IRQ has been disabled");
	}
}

#if CONFIG_PM_QOS_OPP

#if defined(CONFIG_EXYNOS_PM_QOS) || defined(CONFIG_EXYNOS_PM_QOS_MODULE)
static struct exynos_pm_qos_request pm_qos_req_cpu[2];
static struct exynos_pm_qos_request pm_qos_req_mif;
static struct exynos_pm_qos_request pm_qos_req_int;
#endif

int gf9578_pm_qos_opp_init(void)
{
	pr_info("%s: add qos class.", __func__);

#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS) || IS_ENABLED(CONFIG_EXYNOS_PM_QOS_MODULE)
	exynos_pm_qos_add_request(&pm_qos_req_cpu[0], PM_QOS_CLUSTER0_FREQ_MIN, 0);
	exynos_pm_qos_add_request(&pm_qos_req_cpu[1], PM_QOS_CLUSTER1_FREQ_MIN, 0);
	exynos_pm_qos_add_request(&pm_qos_req_mif, PM_QOS_BUS_THROUGHPUT, 0);
	exynos_pm_qos_add_request(&pm_qos_req_int, PM_QOS_DEVICE_THROUGHPUT, 0);
#endif
	return 0;
}

int gf9578_pm_qos_opp_deinit(void)
{
	pr_info("%s: remove qos class.", __func__);
#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS) || IS_ENABLED(CONFIG_EXYNOS_PM_QOS_MODULE)
	exynos_pm_qos_remove_request(&pm_qos_req_cpu[0]);
	exynos_pm_qos_remove_request(&pm_qos_req_cpu[1]);
	exynos_pm_qos_remove_request(&pm_qos_req_mif);
	exynos_pm_qos_remove_request(&pm_qos_req_int);
#endif

	return 0;
}

int gf9578_pm_qos_opp_hold(void)
{
	pr_info("%s:Request MAX QOS.", __func__);

#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS) || IS_ENABLED(CONFIG_EXYNOS_PM_QOS_MODULE)
	if (exynos_pm_qos_request_active(&pm_qos_req_int)) {
		exynos_pm_qos_update_request(&pm_qos_req_int, INT_MAX);
	}

	if (exynos_pm_qos_request_active(&pm_qos_req_mif)) {
		exynos_pm_qos_update_request(&pm_qos_req_mif, INT_MAX);
	}

	if (exynos_pm_qos_request_active(&pm_qos_req_cpu[0])) {
		exynos_pm_qos_update_request(&pm_qos_req_cpu[0], INT_MAX);
	}

	if (exynos_pm_qos_request_active(&pm_qos_req_cpu[1])) {
		exynos_pm_qos_update_request(&pm_qos_req_cpu[1], INT_MAX);
	}
#endif
	return 0;
}

int gf9578_pm_qos_opp_release(void)
{
	pr_info("%s:Request normal QOS.", __func__);

#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS) || IS_ENABLED(CONFIG_EXYNOS_PM_QOS_MODULE)
	if (exynos_pm_qos_request_active(&pm_qos_req_int)) {
		exynos_pm_qos_update_request(&pm_qos_req_mif, 0);
	}
	if (exynos_pm_qos_request_active(&pm_qos_req_int)) {
		exynos_pm_qos_update_request(&pm_qos_req_int, 0);
	}
	if (exynos_pm_qos_request_active(&pm_qos_req_cpu[0])) {
		exynos_pm_qos_update_request(&pm_qos_req_cpu[0], 0);
	}
	if (exynos_pm_qos_request_active(&pm_qos_req_cpu[1])) {
		exynos_pm_qos_update_request(&pm_qos_req_cpu[1], 0);
	}
#endif
	return 0;
}

int gf9578_pm_qos_opp_hold_int_mif(void)
{
#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS) || IS_ENABLED(CONFIG_EXYNOS_PM_QOS_MODULE)
	if (exynos_pm_qos_request_active(&pm_qos_req_int)) {
		exynos_pm_qos_update_request(&pm_qos_req_int, INT_MAX);
	}

	if (exynos_pm_qos_request_active(&pm_qos_req_mif)) {
		exynos_pm_qos_update_request(&pm_qos_req_mif, INT_MAX);
	}
#endif
	return 0;
}

int gf9578_pm_qos_opp_release_int_mif(void)
{
#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS) || IS_ENABLED(CONFIG_EXYNOS_PM_QOS_MODULE)
	if (exynos_pm_qos_request_active(&pm_qos_req_int))
		exynos_pm_qos_update_request(&pm_qos_req_int, 0);

	if (exynos_pm_qos_request_active(&pm_qos_req_mif))
		exynos_pm_qos_update_request(&pm_qos_req_mif, 0);
#endif
	return 0;
}
#endif

#ifdef AP_CONTROL_CLK
static void spi_clock_set(struct gf_dev *gf_dev, int speed)
{
	int rc;
#if 0
	unsigned long rate_core;
	unsigned long rate_iface;

	rate_core = clk_get_rate(gf_dev->core_clk);
	rate_iface = clk_get_rate(gf_dev->iface_clk);
	pr_info("%s, spi clock speed (%lld, %lld)", __func__, rate_core, rate_iface);
#endif
	rc = clk_set_rate(gf_dev->core_clk, speed);
	rc = clk_set_rate(gf_dev->iface_clk, speed);
}

static int gfspi_ioctl_clk_init(struct gf_dev *data)
{
	pr_debug("%s, enter", __func__);

	data->clk_enabled = 0;
	data->core_clk = clk_get(&data->spi->dev, "gate_spi_clk"/*"spi"*/);
	if (IS_ERR_OR_NULL(data->core_clk)) {
		pr_err("%s: fail to get core_clk", __func__);
		return -EPERM;
	}
	data->iface_clk = clk_get(&data->spi->dev, "ipclk_spi"/*"spi_busclk0"*/);
	if (IS_ERR_OR_NULL(data->iface_clk)) {
		pr_err("%s: fail to get iface_clk", __func__);
		clk_put(data->core_clk);
		data->core_clk = NULL;
		return -ENOENT;
	}
	return 0;
}

static int gfspi_ioctl_clk_enable(struct gf_dev *data, uint32_t flag)
{
	int err;

	pr_debug("%s, enter", __func__);

	if (data->clk_enabled) {
		pr_warn("%s warning: spi clock is already enabled, return!", __func__);
		return 0;
	}

	err = clk_prepare_enable(data->core_clk);
	if (err) {
		pr_err("%s, fail to enable core_clk", __func__);
		return -EPERM;
	}

	err = clk_prepare_enable(data->iface_clk);
	if (err) {
		pr_err("%s, fail to enable iface_clk", __func__);
		clk_disable_unprepare(data->core_clk);
		return -ENOENT;
	}

	gf9578_pm_qos_opp_hold_int_mif();

	if (GF_LOW_SPEED == flag) {
		spi_clock_set(data, SPI_LOW_SPEED);
	} else if (GF_HIGH_SPEED == flag) {
		spi_clock_set(data, SPI_HIGH_SPEED);
	} else if (GF_SUPER_SPEED == flag) {
		spi_clock_set(data, SPI_SUPER_SPEED);
	} else {
		pr_err("%s, fail to enable clock, flag(%d) is not in range[0-2].", __func__, flag);
	}

	data->clk_enabled = 1;

	return 0;
}

static int gfspi_ioctl_clk_disable(struct gf_dev *data)
{
	pr_debug("%s, enter", __func__);

	if (!data->clk_enabled) {
		pr_warn("%s warning: spi clock is disabled, return!", __func__);
		return 0;
	}

	clk_disable_unprepare(data->core_clk);
	clk_disable_unprepare(data->iface_clk);
	data->clk_enabled = 0;

	gf9578_pm_qos_opp_release_int_mif();

	return 0;
}

static int gfspi_ioctl_clk_uninit(struct gf_dev *data)
{
	pr_debug("%s, enter", __func__);

	if (data->clk_enabled)
		gfspi_ioctl_clk_disable(data);

	if (!IS_ERR_OR_NULL(data->core_clk)) {
		clk_put(data->core_clk);
		data->core_clk = NULL;
	}

	if (!IS_ERR_OR_NULL(data->iface_clk)) {
		clk_put(data->iface_clk);
		data->iface_clk = NULL;
	}

	return 0;
}
#else
extern void vfp_spi_clk_enable(uint8_t bonoff);
static int gfspi_ioctl_clk_enable(struct gf_dev *data, uint32_t flag)
{
	vfp_spi_clk_enable(1);
}
static int gfspi_ioctl_clk_disable(struct gf_dev *data)
{
	vfp_spi_clk_enable(0);
}
#endif

#if defined(SUPPORT_NAV_EVENT)
static void nav_event_input(struct gf_dev *gf_dev, gf_nav_event_t nav_event)
{
	uint32_t nav_input = 0;

	switch (nav_event) {
	case GF_NAV_FINGER_DOWN:
		pr_debug("%s, nav finger down", __func__);
		break;

	case GF_NAV_FINGER_UP:
		pr_debug("%s, nav finger up", __func__);
		break;

	case GF_NAV_DOWN:
		nav_input = GF_NAV_INPUT_DOWN;
		pr_debug("%s, nav down", __func__);
		break;

	case GF_NAV_UP:
		nav_input = GF_NAV_INPUT_UP;
		pr_debug("%s, nav up", __func__);
		break;

	case GF_NAV_LEFT:
		nav_input = GF_NAV_INPUT_LEFT;
		pr_debug("%s, nav left", __func__);
		break;

	case GF_NAV_RIGHT:
		nav_input = GF_NAV_INPUT_RIGHT;
		pr_debug("%s, nav right", __func__);
		break;

	case GF_NAV_CLICK:
		nav_input = GF_NAV_INPUT_CLICK;
		pr_debug("%s, nav click", __func__);
		break;

	case GF_NAV_HEAVY:
		nav_input = GF_NAV_INPUT_HEAVY;
		pr_debug("%s, nav heavy", __func__);
		break;

	case GF_NAV_LONG_PRESS:
		nav_input = GF_NAV_INPUT_LONG_PRESS;
		pr_debug("%s, nav long press", __func__);
		break;

	case GF_NAV_DOUBLE_CLICK:
		nav_input = GF_NAV_INPUT_DOUBLE_CLICK;
		pr_debug("%s, nav double click", __func__);
		break;

	default:
		pr_warn("%s, unknown nav event: %d", __func__, nav_event);
		break;
	}

	if ((nav_event != GF_NAV_FINGER_DOWN) && (nav_event != GF_NAV_FINGER_UP)) {
		input_report_key(gf_dev->input, nav_input, 1);
		input_sync(gf_dev->input);
		input_report_key(gf_dev->input, nav_input, 0);
		input_sync(gf_dev->input);
	}
}
#endif


static void gf_kernel_key_input(struct gf_dev *gf_dev, struct gf_key *gf_key)
{
	uint32_t key_input = 0;

	if (gf_key->key == GF_KEY_HOME) {
		key_input = GF_KEY_INPUT_HOME;
	} else if (gf_key->key == GF_KEY_POWER) {
		key_input = GF_KEY_INPUT_POWER;
	} else if (gf_key->key == GF_KEY_CAMERA) {
		key_input = GF_KEY_INPUT_CAMERA;
	} else {
		/* add special key define */
		key_input = gf_key->key;
	}
	pr_info("%s, received key event[%d], key = %d, value = %d",
			__func__, key_input, gf_key->key, gf_key->value);

	if ((gf_key->key == GF_KEY_POWER || gf_key->key == GF_KEY_CAMERA)
			 && (gf_key->value == 1)) {
		input_report_key(gf_dev->input, key_input, 1);
		input_sync(gf_dev->input);
		input_report_key(gf_dev->input, key_input, 0);
		input_sync(gf_dev->input);
	}

	if (gf_key->key == GF_KEY_HOME) {
		input_report_key(gf_dev->input, key_input, gf_key->value);
		input_sync(gf_dev->input);
	}
}

static irqreturn_t gf_irq(int irq, void *handle)
{
	struct gf_dev *gf_dev = &gf;

#if defined(GF_NETLINK_ENABLE)
	char temp[2];
	temp[0] = GF_NET_EVENT_IRQ;
	temp[1] = '\0';

	gf9578_sendnlmsg(temp);
#elif defined(GF_FASYNC)
	if (gf_dev->async)
		kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
#endif

	if (vivo_wakeup_t.reset_irq_flag) {
		pr_warn("%s, reset irq shouldn't report wake event", __func__);
		spin_lock(&vivo_wakeup_t.lock_reset_irq);
		vivo_wakeup_t.reset_irq_flag = 0;
		spin_unlock(&vivo_wakeup_t.lock_reset_irq);
	} else if (GF_FF_MODE == vivo_wakeup_t.current_mode && vivo_wakeup_t.ff_mode_state) {
		pr_warn("%s, current mode is %d,ff_mode_state is %d,report wake event", __func__, vivo_wakeup_t.current_mode, vivo_wakeup_t.ff_mode_state);
		vivo_wakeup_t.ff_mode_state = 0;
		input_report_key(gf_dev->input, GF_KEY_INPUT_WAKE, 1);
		input_sync(gf_dev->input);
		input_report_key(gf_dev->input, GF_KEY_INPUT_WAKE, 0);
		input_sync(gf_dev->input);
		__pm_wakeup_event(vivo_wakeup_t.fp_wakelock, msecs_to_jiffies(WAKELOCK_HOLD_TIME));
		pr_warn("%s, wake_lock 1s", __func__);
	}

	return IRQ_HANDLED;
}

static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gf_dev *gf_dev = &gf;
	struct gf_key gf_key;
#if defined(SUPPORT_NAV_EVENT)
	gf_nav_event_t nav_event = GF_NAV_NONE;
#endif
	int retval = 0;
	u32 spi_speed = 0;
	uint32_t delay_ms = 0;
	u8 netlink_route = NETLINK_TEST;
	struct gf_ioc_chip_info info;

	if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
		return -ENODEV;

	if (_IOC_DIR(cmd) & _IOC_READ)
		retval = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		retval = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (retval)
		return -EFAULT;

	if (gf_dev->device_available == 0) {
		if ((cmd == GF_IOC_ENABLE_POWER) || (cmd == GF_IOC_DISABLE_POWER)
			 || (cmd == GF_IOC_GET_POWER_STATE)) {
			pr_info("%s, power cmd", __func__);
		}	else if (cmd == GF_IOC_REQUEST_SEOURCE || cmd == GF_IOC_RELEASE_SEOURCE) {
			pr_info("resource request or release cmd.");
		} else {
			pr_info("%s, Sensor is power off currently", __func__);
			return -ENODEV;
		}
	}

	switch (cmd) {
	case GF_IOC_INIT:
		pr_debug("%s, GF_IOC_INIT", __func__);
		if (copy_to_user((u8 __user *)arg, (u8 *)&netlink_route, sizeof(u8))) {
			retval = -EFAULT;
			break;
		}
		break;
	case GF_IOC_EXIT:
		pr_debug("%s, GF_IOC_EXIT", __func__);
		break;
	case GF_IOC_DISABLE_IRQ:
		pr_debug("%s, GF_IOC_DISABEL_IRQ", __func__);
		gf9578_disable_irq(gf_dev);
		break;
	case GF_IOC_ENABLE_IRQ:
		pr_debug("%s, GF_IOC_ENABLE_IRQ", __func__);
		gf9578_enable_irq(gf_dev);
		break;
	case GF_IOC_RESET:
		pr_info("%s, GF_IOC_RESET", __func__);
		gf9578_hw_reset(gf_dev, 3);
		/*vivo qishuangcheng add begin */
		spin_lock(&vivo_wakeup_t.lock_reset_irq);
		vivo_wakeup_t.reset_irq_flag = 1;
		spin_unlock(&vivo_wakeup_t.lock_reset_irq);
		/*vivo qishuangcheng add end */
		break;
	case GF_IOC_INPUT_KEY_EVENT:
		if (copy_from_user(&gf_key, (struct gf_key *)arg, sizeof(struct gf_key))) {
			pr_err("%s, Failed to copy input key event from user to kernel", __func__);
			retval = -EFAULT;
			break;
		}

		gf_kernel_key_input(gf_dev, &gf_key);
		break;
#if defined(SUPPORT_NAV_EVENT)
	case GF_IOC_NAV_EVENT:
		pr_debug("%s, GF_IOC_NAV_EVENT", __func__);
		if (copy_from_user(&nav_event, (gf_nav_event_t *)arg, sizeof(gf_nav_event_t))) {
			pr_err("%s, Failed to copy nav event from user to kernel", __func__);
			retval = -EFAULT;
			break;
		}

		nav_event_input(gf_dev, nav_event);
		break;
#endif

	case GF_IOC_ENABLE_SPI_CLK:
		pr_debug("%s, GF_IOC_ENABLE_SPI_CLK", __func__);
#ifdef AP_CONTROL_CLK
		if (copy_from_user(&spi_speed, (void __user *)arg, sizeof(spi_speed))) {
			pr_info("%s, GF_IOC_ENABLE_SPI_CLK copy_from_user error.", __func__);
			retval = -EFAULT;
			break;
		}
		gfspi_ioctl_clk_enable(gf_dev, spi_speed);
#else
		gfspi_ioctl_clk_enable(gf_dev, spi_speed);
#endif
		break;
	case GF_IOC_DISABLE_SPI_CLK:
		pr_debug("%s, GF_IOC_DISABLE_SPI_CLK", __func__);
#ifdef AP_CONTROL_CLK
		gfspi_ioctl_clk_disable(gf_dev);
#else
		gfspi_ioctl_clk_disable(gf_dev);
#endif
		break;
	case GF_IOC_ENABLE_POWER:
		pr_debug("%s, GF_IOC_ENABLE_POWER", __func__);
		if (gf_dev->device_available == 1) {
			pr_info("%s, Sensor has already powered-on", __func__);
		} else {
			gf9578_power_on(gf_dev);
		}
		gf_dev->device_available = 1;
		break;
	case GF_IOC_DISABLE_POWER:
		pr_debug("%s, GF_IOC_DISABLE_POWER", __func__);
		if (gf_dev->device_available == 0)
			pr_info("%s, Sensor has already powered-off", __func__);
		else
			gf9578_power_off(gf_dev);
		gf_dev->device_available = 0;
		break;
	case GF_IOC_ENTER_SLEEP_MODE:
		pr_debug("%s, GF_IOC_ENTER_SLEEP_MODE", __func__);
		break;
	case GF_IOC_GET_FW_INFO:
		pr_debug("%s, GF_IOC_GET_FW_INFO", __func__);
		break;
	case GF_IOC_REMOVE:
		pr_debug("%s, GF_IOC_REMOVE", __func__);
		break;
	case GF_IOC_CHIP_INFO:
		pr_debug("%s, GF_IOC_CHIP_INFO", __func__);
		if (copy_from_user(&info, (struct gf_ioc_chip_info *)arg, sizeof(struct gf_ioc_chip_info))) {
			retval = -EFAULT;
			break;
		}
		pr_info("%s, vendor_id : 0x%x, mode : 0x%x, operation: 0x%x", __func__, info.vendor_id, info.mode, info.operation);
		break;
	/*vivo qishuangcheng add begin */
	case GF_IOC_SEND_MODE:
		retval = __get_user(vivo_wakeup_t.current_mode, (u32 __user *) arg);
		if (retval == 0) {
			pr_warn("%s, current mode is %d", __func__, vivo_wakeup_t.current_mode);
			if (GF_FF_MODE == vivo_wakeup_t.current_mode) {
				vivo_wakeup_t.ff_mode_state = 1;
			} else {
				vivo_wakeup_t.ff_mode_state = 0;
			}
		} else {
			pr_warn("%s, Failed to get current mode from user, retval = %d", __func__, retval);
		}
		break;
	case GF_IOC_WAKE_LOCK:
		retval = __get_user(delay_ms, (u32 __user *) arg);
		if (retval == 0) {
			/*if (GF_FF_MODE == vivo_wakeup_t.current_mode) { */
				__pm_wakeup_event(vivo_wakeup_t.fp_wakelock, msecs_to_jiffies(delay_ms));
				pr_warn("%s, wake lock delay %d ms", __func__, delay_ms);
			/*} */
		}
		break;
	/*vivo qishuangcheng add end */
	case GF_IOC_GET_POWER_STATE:
			pr_warn("%s, GF_IOC_GET_POWER_STATE", __func__);
			retval = gf9578_hw_get_power_state(gf_dev);
		break;
	case GF_IOC_PM_QOS_REQUEST:
#ifdef CONFIG_PM_QOS_OPP
		gf9578_pm_qos_opp_hold();
#endif
		break;
	case GF_IOC_PM_QOS_RELEASE:
#ifdef CONFIG_PM_QOS_OPP
		gf9578_pm_qos_opp_release();
#endif
		break;
	case GF_IOC_REQUEST_SEOURCE:
		pr_info("%s: GF_IOC_REQUEST_SEOURCE ======", __func__);
		if (1 == support_soft_fpid) {
			retval = gf9578_request_resource(gf_dev, gf_irq);
		}
		break;
	case GF_IOC_RELEASE_SEOURCE:
		pr_info("%s: GF_IOC_RELEASE_SEOURCE ======", __func__);
		if (1 == support_soft_fpid) {
			gf9578_release_resource(gf_dev);
		}
		break;
	default:
		pr_warn("%s, unsupport cmd:0x%x", __func__, cmd);
		break;
	}

	return retval;
}

#ifdef CONFIG_COMPAT
static long gf_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return gf_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif /*CONFIG_COMPAT */



static int gf_open(struct inode *inode, struct file *filp)
{
	struct gf_dev *gf_dev;
	int status = -ENXIO;

	mutex_lock(&device_list_lock);

	list_for_each_entry(gf_dev, &device_list, device_entry) {
		if (gf_dev->devt == inode->i_rdev) {
			pr_info("%s, Found", __func__);
			status = 0;
			break;
		}
	}

	if (status == 0) {
		if (status == 0) {
			gf_dev->users++;
			filp->private_data = gf_dev;
			nonseekable_open(inode, filp);
			pr_info("%s, Succeed to open device, irq = %d", __func__, gf_dev->irq);
			if (0 == support_soft_fpid) {
				if (gf_dev->device_available == 0) {
					gf9578_power_on(gf_dev);
					mdelay(20);
				}
				if (gf_dev->users == 1)
					gf9578_enable_irq(gf_dev);
				gf9578_hw_reset(gf_dev, 3);
				gf_dev->device_available = 1;
			} else {
				pr_info("Succeed to open device.");
			}
		}
	} else {
		pr_info("%s, No device for minor %d", __func__, iminor(inode));
	}
	mutex_unlock(&device_list_lock);
	return status;
}

static ssize_t gf_read(struct file *filp, char __user *ubuf, size_t count, loff_t *offp)
{
	char *buf;
	ssize_t ret, out_offset, out_count;

	out_count = 1000;
	buf = kmalloc(out_count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	out_offset = 0;
	out_offset += snprintf(buf + out_offset, out_count - out_offset,
				"goodix fingerprint driver debug function:\n");
	out_offset += snprintf(buf + out_offset, out_count - out_offset,
				"0. power off.\n");
	out_offset += snprintf(buf + out_offset, out_count - out_offset,
				"1. power on.\n");
	out_offset += snprintf(buf + out_offset, out_count - out_offset,
				"2. hardware reset.\n");

	if (out_offset > out_count)
		out_offset = out_count;

	ret = simple_read_from_buffer(ubuf, count, offp, buf, out_offset);
	kfree(buf);
	return ret;
}

static ssize_t gf_write(struct file *file, const char __user *ubuf, size_t count, loff_t *off)
{
	unsigned int val;
	int ret;
	struct gf_dev *gf_dev = &gf;

	ret = kstrtouint_from_user(ubuf, count, 10, &val);
	if (ret) {
		pr_err("%s Failed to convert to unsigned int", __func__);
		return ret;
	}

	switch (val) {
	case 0:
		pr_info("%s option 0: power off", __func__);
		gf9578_power_off(gf_dev);
		break;
	case 1:
		pr_info("%s option 1: power on", __func__);
		gf9578_power_on(gf_dev);
		break;
	case 2:
		pr_info("%s option 2: hw reset", __func__);
		gf9578_hw_reset(gf_dev, 3);
		break;
	default:
		pr_warn("%s option(%d) not support.", __func__, val);
		break;
	}
	return count;
}

#ifdef GF_FASYNC
static int gf_fasync(int fd, struct file *filp, int mode)
{
	struct gf_dev *gf_dev = filp->private_data;
	int ret;

	ret = fasync_helper(fd, filp, mode, &gf_dev->async);
	pr_info("%s, ret = %d", __func__, ret);
	return ret;
}
#endif

static int gf_release(struct inode *inode, struct file *filp)
{
	struct gf_dev *gf_dev;
	int status = 0;

	pr_debug("%s, enter", __func__);
	mutex_lock(&device_list_lock);
	gf_dev = filp->private_data;
	filp->private_data = NULL;

	/*last close?? */
	gf_dev->users--;
	if (!gf_dev->users) {
		if (0 == support_soft_fpid) {
			pr_info("disble_irq. irq = %d", gf_dev->irq);
			gf9578_disable_irq(gf_dev);
			/*power off the sensor */
			gf_dev->device_available = 0;
		}
		gf9578_power_off(gf_dev);
	}
	mutex_unlock(&device_list_lock);
	pr_debug("%s, exit", __func__);

	return status;
}

static const struct file_operations gf_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = gf_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gf_compat_ioctl,
#endif /*CONFIG_COMPAT */
	.open = gf_open,
	.read = gf_read,
	.write = gf_write,
	.release = gf_release,
#ifdef GF_FASYNC
	.fasync = gf_fasync,
#endif
};

static struct class *gf_class;
#if defined(USE_SPI_BUS)
static int gf_probe(struct spi_device *spi)
#elif defined(USE_PLATFORM_BUS)
static int gf_probe(struct platform_device *pdev)
#endif
{
	struct gf_dev *gf_dev = &gf;
	int status = -EINVAL;
	unsigned long minor;
	int i;

	/* Initialize the driver data */
	INIT_LIST_HEAD(&gf_dev->device_entry);
#if defined(USE_SPI_BUS)
	gf_dev->spi = spi;
#elif defined(USE_PLATFORM_BUS)
	gf_dev->spi = pdev;
#endif

	/*vivo fingerprint team add begin */
	spin_lock_init(&vivo_wakeup_t.lock_reset_irq);
	vivo_wakeup_t.fp_wakelock = wakeup_source_register(&gf_dev->spi->dev, "fp_wakelock");
	/*vivo fingerprint team add end */

	gf_dev->irq_gpio = -EINVAL;
	gf_dev->reset_gpio = -EINVAL;
	gf_dev->vdd_en_gpio = -EINVAL;
	gf_dev->vddio_en_gpio = -EINVAL;
	gf_dev->device_available = 0;
	gf_dev->fb_black = 0;
	gf_dev->vdd_use_gpio = false;
	gf_dev->vdd_use_pmic = false;
	gf_dev->vddio_use_gpio = false;
	gf_dev->vddio_use_pmic = false;
	gf_dev->resource_requested = false;

	if (gf9578_parse_dts(gf_dev))
		goto error_hw;

#ifdef AP_CONTROL_CLK
	pr_info("Get the clk resource.");
	/* Enable spi clock */
	if (gfspi_ioctl_clk_init(gf_dev))
		goto gfspi_probe_clk_init_failed;

	if (gfspi_ioctl_clk_enable(gf_dev))
		goto gfspi_probe_clk_enable_failed;

	spi_clock_set(gf_dev, 1000000);
#endif

	if (0 == support_soft_fpid) {
		status = gf9578_request_resource(gf_dev, gf_irq);
		if (status) {
			pr_err("failed to request IRQ:%d", gf_dev->irq);
			goto error_hw;
		}
	}

	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		gf_dev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(gf_class, &gf_dev->spi->dev, gf_dev->devt,
				gf_dev, GF_DEV_NAME);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		dev_dbg(&gf_dev->spi->dev, "no minor number available!");
		status = -ENODEV;
		mutex_unlock(&device_list_lock);
		goto error_hw;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&gf_dev->device_entry, &device_list);
	} else {
		gf_dev->devt = 0;
	}
	mutex_unlock(&device_list_lock);

	if (status == 0) {
		/*input device subsystem */
		gf_dev->input = input_allocate_device();
		if (gf_dev->input == NULL) {
			pr_err("%s, failed to allocate input device", __func__);
			status = -ENOMEM;
			goto error_dev;
		}
		for (i = 0; i < ARRAY_SIZE(gf9578_key_maps); i++)
			input_set_capability(gf_dev->input, gf9578_key_maps[i].type, gf9578_key_maps[i].code);

		gf_dev->input->name = GF_INPUT_NAME;
		status = input_register_device(gf_dev->input);
		if (status) {
			pr_err("%s, failed to register input device", __func__);
			goto error_input;
		}
	}
#ifdef AP_CONTROL_CLK
	pr_info("%s, Get the clk resource", __func__);
	/* Enable spi clock */
	if (gfspi_ioctl_clk_init(gf_dev))
		goto gfspi_probe_clk_init_failed;

	if (gfspi_ioctl_clk_enable(gf_dev, GF_LOW_SPEED))
		goto gfspi_probe_clk_enable_failed;
	gfspi_ioctl_clk_disable(gf_dev);
#endif

	pr_info("%s, version V%d.%d.%02d", __func__, VER_MAJOR, VER_MINOR, PATCH_LEVEL);

#ifdef CONFIG_PM_QOS_OPP
	gf9578_pm_qos_opp_init();
#endif

	return status;


#ifdef AP_CONTROL_CLK
gfspi_probe_clk_enable_failed:
	gfspi_ioctl_clk_uninit(gf_dev);
gfspi_probe_clk_init_failed:
#endif

error_input:
	if (gf_dev->input != NULL)
		input_free_device(gf_dev->input);
error_dev:
	if (gf_dev->devt != 0) {
		pr_err("%s, Error status = %d", __func__, status);
		mutex_lock(&device_list_lock);
		list_del(&gf_dev->device_entry);
		device_destroy(gf_class, gf_dev->devt);
		clear_bit(MINOR(gf_dev->devt), minors);
		mutex_unlock(&device_list_lock);
	}
error_hw:
	gf9578_release_resource(gf_dev);
	gf_dev->device_available = 0;

	return status;
}

#if defined(USE_SPI_BUS)
static int gf_remove(struct spi_device *spi)
#elif defined(USE_PLATFORM_BUS)
static int gf_remove(struct platform_device *pdev)
#endif
{
	struct gf_dev *gf_dev = &gf;

#ifdef CONFIG_PM_QOS_OPP
	gf9578_pm_qos_opp_deinit();
#endif
	wakeup_source_unregister(vivo_wakeup_t.fp_wakelock);

	if (gf_dev->input != NULL)
		input_unregister_device(gf_dev->input);
	input_free_device(gf_dev->input);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&gf_dev->device_entry);
	device_destroy(gf_class, gf_dev->devt);
	clear_bit(MINOR(gf_dev->devt), minors);
	if (gf_dev->resource_requested) {
		gf9578_release_resource(gf_dev);
		gf_dev->resource_requested = false;
	} else {
		pr_info("resource is not requested.");
	}

	/*fb_unregister_client(&gf_dev->notifier); */
	mutex_unlock(&device_list_lock);

	return 0;
}

#if defined(USE_PLATFORM_BUS)
int gf_resume(struct platform_device *pdev)
{
#ifdef AP_CONTROL_CLK
	struct gf_dev *gf_dev = &gf;
	unsigned long rate_core;
	unsigned long rate_iface;

	rate_core = clk_get_rate(gf_dev->core_clk);
	rate_iface = clk_get_rate(gf_dev->iface_clk);

	pr_debug("%s, spi clock freq (%lld, %lld).", __func__, rate_core, rate_iface);
#endif
	return 0;
}

int gf_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct gf_dev *gf_dev = &gf;
	pr_debug("%s.", __func__);
	return 0;
}
#endif

#if defined(USE_SPI_BUS)
static void gf_shutdown(struct spi_device *spi)
#elif defined(USE_PLATFORM_BUS)
static void gf_shutdown(struct platform_device *pdev)
#endif
{
	struct gf_dev *gf_dev = &gf;
	pr_info("%s", __func__);
	gf9578_power_off(gf_dev);
}

static const struct of_device_id gx_match_table[] = {
	{ .compatible = "goodix,udfingerprint" },
	{ .compatible = "mediatek,goodix-udfp" },
	{},
};

#if defined(USE_SPI_BUS)
static struct spi_driver gf_driver = {
#elif defined(USE_PLATFORM_BUS)
static struct platform_driver gf_driver = {
#endif
	.driver = {
		.name = GF_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = gx_match_table,
	},
	.probe = gf_probe,
	.remove = gf_remove,
	.shutdown = gf_shutdown,
#if defined(USE_PLATFORM_BUS)
	.resume = gf_resume,
	.suspend = gf_suspend,
#endif
};

extern unsigned int is_atboot;
//extern unsigned int power_off_charging_mode;
#if 0
//vivo wuzengshun add for boot pure system to upgrade begin
extern unsigned int os_boot_puresys;
//vivo wuzengshun add for boot pure system to upgrade end
#endif
extern int get_fp_id(void);

static int __init gf_init(void)
{
	int status;
	pr_info("%s enter.", __func__);

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev / mdev to add / remove / dev nodes.  Last, register
	 * the driver which manages those device numbers.
	*/

	if (is_atboot == 1) {
		/*do not load gf driver */
		pr_info("%s, in AT mode, not load gf driver!", __func__);
		return 0;
	}

/* 	if (power_off_charging_mode == 1) {
		pr_info("%s, power off charging mode, not load gf driver!", __func__);
		return 0;
	}*/
#if 0
	//vivo wuzengshun add for boot pure system to upgrade begin
	if (os_boot_puresys == 1) {
		/*do not load gf driver */
		pr_info("%s, boot puresys, not load gf driver!", __func__);
		return 0;
	}
	//vivo wuzengshun add for boot pure system to upgrade end
#endif

	struct device_node *node = of_find_compatible_node(NULL, NULL, "fp-id");
	if (node && of_property_read_bool(node, "vivo,support_soft_fingerprint_id")) {
		support_soft_fpid = 1;
		pr_info("%s, support software detect fpid.", __func__);
	} else {
		support_soft_fpid = 0;
	}

#if IS_ENABLED(CONFIG_BBK_FINGERPRINT)
	if (0 == support_soft_fpid) {
		if ((get_fp_id() != GOODIX_GF9578) && (get_fp_id() != GOODIX_GF9518)) {
			pr_err("%s: wrong fp id, not gf9578 driver, exit", __func__);
			return 0;
		}
	}
#endif

	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME, &gf_fops);
	if (status < 0) {
		pr_warn("%s, Failed to register char device!", __func__);
		return status;
	}
	SPIDEV_MAJOR = status;
	gf_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(gf_class)) {
		unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
		pr_warn("%s, Failed to create class", __func__);
		return PTR_ERR(gf_class);
	}
#if defined(USE_PLATFORM_BUS)
	status = platform_driver_register(&gf_driver);
#elif defined(USE_SPI_BUS)
	status = spi_register_driver(&gf_driver);
#endif
	if (status < 0) {
		class_destroy(gf_class);
		unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
		pr_warn("%s, Failed to register SPI driver", __func__);
	}

#ifdef GF_NETLINK_ENABLE
	gf9578_netlink_init();
#endif
	pr_info("%s, status = 0x%x", __func__, status);
	return 0;
}

late_initcall(gf_init);

static void __exit gf_exit(void)
{
#ifdef GF_NETLINK_ENABLE
	gf9578_netlink_exit();
#endif
#if defined(USE_PLATFORM_BUS)
	platform_driver_unregister(&gf_driver);
#elif defined(USE_SPI_BUS)
	spi_unregister_driver(&gf_driver);
#endif
	class_destroy(gf_class);
	unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
}
module_exit(gf_exit);

MODULE_AUTHOR("Jiangtao Yi, <yijiangtao@goodix.com>");
MODULE_DESCRIPTION("goodix fingerprint sensor device driver");
MODULE_LICENSE("GPL");
