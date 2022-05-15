/*
 * SAMSUNG NFC Controller
 *
 * Copyright (C) 2013 Samsung Electronics Co.Ltd
 * Author: Woonki Lee <woonki84.lee@samsung.com>
 *         Heejae Kim <heejae12.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program
 *
 */
#ifdef CONFIG_SEC_NFC_IF_I2C_GPIO
#define CONFIG_SEC_NFC_IF_I2C
#endif

#ifndef CONFIG_SEC_NFC_RETRY
#define CONFIG_SEC_NFC_RETRY
#endif

#include <linux/wait.h>
#include <linux/delay.h>

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/clk-provider.h>
#ifdef CONFIG_SEC_NFC_GPIO_CLK
#include <linux/interrupt.h>
#endif
#ifdef CONFIG_SEC_NFC_WAKE_LOCK
// #include <linux/wakelock.h>
#include <linux/pm_wakeup.h>
#endif
#include <linux/of_gpio.h>
#include <linux/clk.h>
#ifdef CONFIG_SEC_NFC_RETRY
#include <linux/workqueue.h>
#endif
#include <mt-plat/mtk_boot_common.h>

#ifndef CONFIG_SEC_NFC_IF_I2C
struct sec_nfc_i2c_info {};
#define sec_nfc_read            NULL
#define sec_nfc_write           NULL
#define sec_nfc_poll            NULL
#define sec_nfc_i2c_irq_clear(x)

#define SEC_NFC_GET_INFO(dev) platform_get_drvdata(to_platform_device(dev))

#else /* CONFIG_SEC_NFC_IF_I2C */
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/i2c.h>
#include "sec_nfc.h"

#define SEC_NFC_GET_INFO(dev) i2c_get_clientdata(to_i2c_client(dev))
enum sec_nfc_irq {
    SEC_NFC_SKIP = -1,
    SEC_NFC_NONE,
    SEC_NFC_INT,
    SEC_NFC_READ_TIMES,
};

struct sec_nfc_i2c_info {
    struct i2c_client *i2c_dev;
    struct mutex read_mutex;
    enum sec_nfc_irq read_irq;
    wait_queue_head_t read_wait;
    size_t buflen;
    u8 *buf;
};
#endif

#ifdef CONFIG_SEC_NFC_RETRY
struct sec_nfc_retry_info {
	struct delayed_work retry_work;
	size_t buflen;
	u8 *buf;
	bool is_active;
	bool is_read;
};
#endif

struct sec_nfc_info {
    struct miscdevice miscdev;
    struct mutex mutex;
    enum sec_nfc_mode mode;
    struct device *dev;
    struct sec_nfc_platform_data *pdata;
    struct sec_nfc_i2c_info i2c_info;
#ifdef CONFIG_SEC_NFC_WAKE_LOCK
    struct wakeup_source *nfc_wake_lock;
#endif
#ifdef CONFIG_SEC_NFC_GPIO_CLK
    bool clk_ctl;
    bool clk_state;
#endif
#ifdef CONFIG_SEC_NFC_DEDICATED_CLK
    void __iomem *clkctrl;
#endif
#ifdef CONFIG_SEC_NFC_RETRY
	struct sec_nfc_retry_info retry_info;
#endif
};

/*vivo caibinchen add a node for load nfc service start*/
#define DTS_NFC_SUPPORT_STR	"vivo,nfc_support"
#define DTS_BOARDVERSION_SHIFT_STR	"vivo,boardversion_shift"
#define DTS_BOARDVERSION_MASK_STR	"vivo,boardversion_mask"
#define DTS_BOARDVERSION_NUM_STR	"vivo,boardversion_num"
#define DTS_BOARDVERSION_STR		"vivo,boardversions"
unsigned int   nfc_result;
struct kobject nfc_kobject;
unsigned int nfc_support;
unsigned int boardversion_mask;
unsigned int boardversion_shift;
unsigned int boardversion_num;
const char *boardversions[20];
extern char *get_board_version(void);
#define DEVFS_MODE_RO (S_IRUSR|S_IRGRP|S_IROTH)
extern unsigned int power_off_charging_mode;
struct attribute nfc_enable_attr = {
	.name = "nfc_enable",
	.mode = DEVFS_MODE_RO,
};
static struct attribute *our_own_sys_attrs[] = {
	&nfc_enable_attr,
	NULL,
};
/*add a node for load nfc service end*/
extern unsigned int get_boot_mode(void);

/*add vivo smt test*/
#ifdef CONFIG_VIVO_NFC_FACTORY_SMT_TEST
struct pinctrl_state *nfc_pinctrl_clkreq_output;
struct pinctrl_state *nfc_pinctrl_default;
struct pinctrl *nfc_pinctrl;
#endif
bool device_wakeup_flag = false;
int need_send_no_resp_cmd = false;


#ifdef CONFIG_ESE_COLDRESET
struct mutex coldreset_mutex;
struct mutex sleep_wake_mutex;
bool sleep_wakeup_state[2];
u8 disable_combo_reset_cmd[4] = { 0x2F, 0x30, 0x01, 0x00};
enum sec_nfc_mode cur_mode;
#endif

#ifdef CONFIG_SEC_NFC_RETRY
static void retry_wq_handler(struct work_struct *work)
{
	struct sec_nfc_info *info = container_of(work, struct sec_nfc_info, retry_info.retry_work.work);
	int ret = 0;
	dev_err(info->dev, "No response send last cmd again cmd len: %d \n", info->retry_info.buflen);
	ret = i2c_master_send(info->i2c_info.i2c_dev, info->retry_info.buf, info->retry_info.buflen);
	if (ret < 0 || ret != info->retry_info.buflen) {
		dev_err(info->dev, "Retry send failed ret: %d\n", ret);
	}
}
#endif

#ifdef CONFIG_SEC_NFC_IF_I2C
static irqreturn_t sec_nfc_irq_thread_fn(int irq, void *dev_id)
{
    struct sec_nfc_info *info = dev_id;
    struct sec_nfc_platform_data *pdata = info->pdata;

    dev_err(info->dev, "[NFC] Read Interrupt is occurred!\n");

    pr_err("%s:  pdata->irq=%d\n", __func__,  gpio_get_value(pdata->irq));
    if (gpio_get_value(pdata->irq) == 0) {
		dev_err(info->dev, "[NFC] Warning,irq-gpio state is low!\n");
		return IRQ_HANDLED;
    }
     pr_err("%s:  pdata->irq=%d\n", __func__,  gpio_get_value(pdata->irq));
    mutex_lock(&info->i2c_info.read_mutex);
    /* Skip interrupt during power switching
     * It is released after first write */
    if (info->i2c_info.read_irq == SEC_NFC_SKIP) {
		dev_err(info->dev, "%s: Now power swiching. Skip this IRQ\n", __func__);
		mutex_unlock(&info->i2c_info.read_mutex);
		return IRQ_HANDLED;
    }

    info->i2c_info.read_irq += SEC_NFC_READ_TIMES;
    mutex_unlock(&info->i2c_info.read_mutex);

    wake_up_interruptible(&info->i2c_info.read_wait);
#ifdef CONFIG_SEC_NFC_WAKE_LOCK
    pm_wakeup_ws_event(info->nfc_wake_lock, 2000, false);
#endif

    return IRQ_HANDLED;
}

static ssize_t sec_nfc_read(struct file *file, char __user *buf,
					size_t count, loff_t *ppos)
{
    struct sec_nfc_info *info = container_of(file->private_data,
						struct sec_nfc_info, miscdev);
    enum sec_nfc_irq irq;
    int ret = 0;

    dev_err(info->dev, "%s: info: %p, count: %zu\n", __func__,
		info, count);

	mutex_lock(&info->mutex);

	if (info->mode == SEC_NFC_MODE_OFF) {
		dev_err(info->dev, "sec_nfc is not enabled\n");
		ret = -ENODEV;
		goto out;
    }

	mutex_lock(&info->i2c_info.read_mutex);
	if (count == 0) {
		if (info->i2c_info.read_irq >= SEC_NFC_INT)
			info->i2c_info.read_irq--;
		wake_up_interruptible(&info->i2c_info.read_wait);
		mutex_unlock(&info->i2c_info.read_mutex);
		goto out;
	}

    irq = info->i2c_info.read_irq;
    mutex_unlock(&info->i2c_info.read_mutex);
    if (irq == SEC_NFC_NONE) {
		if (file->f_flags & O_NONBLOCK) {
			dev_err(info->dev, " %s it is nonblock\n", __func__);
			ret = -EAGAIN;
			goto out;
		}
    }

    /* i2c recv */
    if (count > info->i2c_info.buflen)
		count = info->i2c_info.buflen;

    if (count > SEC_NFC_MSG_MAX_SIZE) {
		dev_err(info->dev, "user required wrong size :%d ,%s\n", (u32)count, __func__);
		ret = -EINVAL;
		goto out;
    }

    mutex_lock(&info->i2c_info.read_mutex);
    memset(info->i2c_info.buf, 0, count);
    ret = i2c_master_recv(info->i2c_info.i2c_dev, info->i2c_info.buf, (u32)count);
    dev_err(info->dev, "recv size : %d %s\n", ret, __func__);

    if (ret == -EREMOTEIO) {
		ret = -ERESTART;
		goto read_error;
    } else if (ret != count) {
		dev_err(info->dev, "read failed: return: %d count: %d\n",
			ret, (u32)count);
		goto read_error;
    }

    if (info->i2c_info.read_irq >= SEC_NFC_INT)
		info->i2c_info.read_irq--;

    if (info->i2c_info.read_irq == SEC_NFC_READ_TIMES)
		wake_up_interruptible(&info->i2c_info.read_wait);

#ifdef CONFIG_SEC_NFC_RETRY
	if (info->retry_info.is_active && !info->retry_info.is_read) {
		info->retry_info.is_read = true;
		cancel_delayed_work(&info->retry_info.retry_work);
	}
#endif
    mutex_unlock(&info->i2c_info.read_mutex);

    if (copy_to_user(buf, info->i2c_info.buf, ret)) {
		dev_err(info->dev, "copy failed to user\n");
		ret = -EFAULT;
    }

    goto out;

read_error:
    info->i2c_info.read_irq = SEC_NFC_NONE;
    mutex_unlock(&info->i2c_info.read_mutex);
out:
    mutex_unlock(&info->mutex);

    return ret;
}

static ssize_t sec_nfc_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
    struct sec_nfc_info *info = container_of(file->private_data,
						struct sec_nfc_info, miscdev);
    int ret = 0;
	u8 fw_no_response_cmd[3] = {0xff, 0xff, 0x00};

    dev_err(info->dev, "%s: info: %p, count %d\n", __func__,
		info, (u32)count);

	mutex_lock(&info->mutex);

    if (info->mode == SEC_NFC_MODE_OFF) {
		dev_err(info->dev, "sec_nfc is not enabled\n");
		ret = -ENODEV;
		goto out;
    }

    if (count > info->i2c_info.buflen)
		count = info->i2c_info.buflen;

    if (count > SEC_NFC_MSG_MAX_SIZE) {
		dev_err(info->dev, "user required wrong size :%d\n", (u32)count);
		ret = -EINVAL;
		goto out;
    }

    if (copy_from_user(info->i2c_info.buf, buf, count)) {
		dev_err(info->dev, "copy failed from user\n");
		ret = -EFAULT;
		goto out;
    }
#ifdef CONFIG_SEC_NFC_RETRY
	if (info->retry_info.is_active && (info->i2c_info.buf[0] & 0x20)) {
		info->retry_info.is_read = false;
		info->retry_info.buflen = count;
		memcpy(info->retry_info.buf, info->i2c_info.buf, count);
		schedule_delayed_work(&info->retry_info.retry_work, msecs_to_jiffies(1000));
	}
#endif
	/* Skip interrupt during power switching
	 * It is released after first write
	 */
    mutex_lock(&info->i2c_info.read_mutex);
	if (need_send_no_resp_cmd && device_wakeup_flag) {
		dev_err(info->dev, "%s: write test begin\n", __func__);
		ret = i2c_master_send(info->i2c_info.i2c_dev, fw_no_response_cmd, 3);
		dev_err(info->dev, "%s: write test ret = %d\n", __func__, ret);
		usleep_range(5000, 5100);
	}
	device_wakeup_flag = false;
	need_send_no_resp_cmd = true;

    ret = i2c_master_send(info->i2c_info.i2c_dev, info->i2c_info.buf, count);
    if (info->i2c_info.read_irq == SEC_NFC_SKIP)
		info->i2c_info.read_irq = SEC_NFC_NONE;
    mutex_unlock(&info->i2c_info.read_mutex);

    if (ret == -EREMOTEIO) {
		dev_err(info->dev, "send failed: return: %d count: %d\n",
		ret, (u32)count);
		ret = -ERESTART;
		goto out;
    }

    if (ret != count) {
		dev_err(info->dev, "send failed: return: %d count: %d\n",
		ret, (u32)count);
		ret = -EREMOTEIO;
    }

out:
    mutex_unlock(&info->mutex);

    return ret;
}

static unsigned int sec_nfc_poll(struct file *file, poll_table *wait)
{
    struct sec_nfc_info *info = container_of(file->private_data,
					struct sec_nfc_info, miscdev);
    enum sec_nfc_irq irq;

    int ret = 0;

    dev_err(info->dev, "%s: info: %p\n", __func__, info);

    mutex_lock(&info->mutex);

    if (info->mode == SEC_NFC_MODE_OFF) {
		dev_err(info->dev, "sec_nfc is not enabled\n");
		ret = -ENODEV;
		goto out;
    }

    poll_wait(file, &info->i2c_info.read_wait, wait);

    mutex_lock(&info->i2c_info.read_mutex);
    irq = info->i2c_info.read_irq;
    if (irq == SEC_NFC_READ_TIMES)
		ret = (POLLIN | POLLRDNORM);
    mutex_unlock(&info->i2c_info.read_mutex);

out:
    mutex_unlock(&info->mutex);

    return ret;
}

void sec_nfc_i2c_irq_clear(struct sec_nfc_info *info)
{
    /* clear interrupt. Interrupt will be occurred at power off */
    mutex_lock(&info->i2c_info.read_mutex);
    info->i2c_info.read_irq = SEC_NFC_NONE;
    mutex_unlock(&info->i2c_info.read_mutex);
}

int sec_nfc_i2c_probe(struct i2c_client *client)
{
    struct device *dev = &client->dev;
    struct sec_nfc_info *info = dev_get_drvdata(dev);
    struct sec_nfc_platform_data *pdata = info->pdata;
    int ret;

    pr_err("%s: start: %p\n", __func__, info);

    info->i2c_info.buflen = SEC_NFC_MAX_BUFFER_SIZE;
    info->i2c_info.buf = kzalloc(SEC_NFC_MAX_BUFFER_SIZE, GFP_KERNEL);
    if (!info->i2c_info.buf) {
		dev_err(dev,
			"failed to allocate memory for sec_nfc_info->buf\n");
		return -ENOMEM;
    }
    info->i2c_info.i2c_dev = client;
    info->i2c_info.read_irq = SEC_NFC_NONE;
    mutex_init(&info->i2c_info.read_mutex);
    init_waitqueue_head(&info->i2c_info.read_wait);
    i2c_set_clientdata(client, info);

    info->dev = dev;

    ret = gpio_request(pdata->irq, "nfc_int");
    if (ret) {
		pr_err("GPIO request is failed to register IRQ\n");
		goto err_irq_req;
    }

    client->irq = gpio_to_irq(pdata->irq);
    pr_err("%s: push interrupt no = %d , pdata->irq=%d\n", __func__, client->irq, gpio_get_value(pdata->irq));

     ret = request_threaded_irq(client->irq, NULL, sec_nfc_irq_thread_fn,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, SEC_NFC_DRIVER_NAME,
			info);
    if (ret < 0) {
		pr_err("failed to register IRQ handler\n");
		kfree(info->i2c_info.buf);
		return ret;
    }

    pr_info("%s success\n", __func__);
    return 0;

err_irq_req:
    return ret;
}

void sec_nfc_i2c_remove(struct device *dev)
{
    struct sec_nfc_info *info = dev_get_drvdata(dev);
    struct i2c_client *client = info->i2c_info.i2c_dev;
    struct sec_nfc_platform_data *pdata = info->pdata;
    free_irq(client->irq, info);
    gpio_free(pdata->irq);
}
#endif /* CONFIG_SEC_NFC_IF_I2C */

#ifdef CONFIG_SEC_NFC_GPIO_CLK
static irqreturn_t sec_nfc_clk_irq_thread(int irq, void *dev_id)
{
    struct sec_nfc_info *info = dev_id;
    struct sec_nfc_platform_data *pdata = info->pdata;
    bool value;

    dev_err(info->dev, " %s [NFC]Clock Interrupt is occurred!\n", __func__);
    value = gpio_get_value(pdata->clk_req) > 0 ? true : false;

    if (value == info->clk_state)
		return IRQ_HANDLED;

    if (value)
		clk_prepare_enable(pdata->clk);
    else
		clk_disable_unprepare(pdata->clk);

    info->clk_state = value;

    return IRQ_HANDLED;
}

void sec_nfc_clk_ctl_enable(struct sec_nfc_info *info)
{
    struct sec_nfc_platform_data *pdata = info->pdata;
    unsigned int irq;
    int ret;

    irq = gpio_to_irq(pdata->clk_req);

    if (info->clk_ctl)
		return;

    info->clk_state = false;
    ret = request_threaded_irq(irq, NULL, sec_nfc_clk_irq_thread,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			SEC_NFC_DRIVER_NAME, info);
    if (ret < 0) {
		dev_err(info->dev, "failed to register CLK REQ IRQ handler\n");
    }
    info->clk_ctl = true;
}
void sec_nfc_clk_ctl_disable(struct sec_nfc_info *info)
{
    struct sec_nfc_platform_data *pdata = info->pdata;
    unsigned int irq;

    if (pdata->clk_req)
		irq = gpio_to_irq(pdata->clk_req);
    else
		return;

    if (!info->clk_ctl)
		return;

    free_irq(irq, info);
    if (info->clk_state)
		clk_disable_unprepare(pdata->clk);
    info->clk_state = false;
    info->clk_ctl = false;
}
#else
#define sec_nfc_clk_ctl_enable(x)
#define sec_nfc_clk_ctl_disable(x)
#endif /* CONFIG_SEC_NFC_GPIO_CLK */

static void sec_nfc_set_mode(struct sec_nfc_info *info,
				enum sec_nfc_mode mode)
{
    struct sec_nfc_platform_data *pdata = info->pdata;
#ifdef CONFIG_ESE_COLDRESET
	int alreadFirmHigh = 0;
	int ret ;
	enum sec_nfc_mode oldmode = info->mode;
#endif
#ifdef CONFIG_SEC_NFC_DEDICATED_CLK
	unsigned int val = readl_relaxed(info->clkctrl);
#endif

	/* intfo lock is aleady gotten before calling this function */
	if (info->mode == mode) {
		pr_err("Power mode is already %d", mode);
		return;
	}
	info->mode = mode;

#ifdef CONFIG_SEC_NFC_IF_I2C
	/* Skip interrupt during power switching
	 * It is released after first write */
	mutex_lock(&info->i2c_info.read_mutex);
#ifdef CONFIG_ESE_COLDRESET
	cur_mode = mode;
	memset(sleep_wakeup_state, false, sizeof(sleep_wakeup_state));
	if (oldmode == SEC_NFC_MODE_OFF) {
		if (gpio_get_value(pdata->firm) == 1) {
			alreadFirmHigh = 1;
			pr_err("Firm is already high; do not anything");
		} else {
			/*Firm pin is low*/
			gpio_set_value(pdata->firm, SEC_NFC_FW_ON);
			msleep(SEC_NFC_VEN_WAIT_TIME);
		}

		if (gpio_get_value(pdata->ven) == SEC_NFC_PW_ON) {
			ret = i2c_master_send(info->i2c_info.i2c_dev, disable_combo_reset_cmd,
					sizeof(disable_combo_reset_cmd)/sizeof(u8));
			pr_err("disable combo_reset_command ret: %d", ret);
		} else
			pr_err("skip disable combo_reset_command");

		if (alreadFirmHigh == 1) {
			pr_err("Firm is already high; do not anything2");
		} else {
			/*Firm pin is low*/
			mdelay(3);
			/*wait for FW*/
			gpio_set_value(pdata->firm, SEC_NFC_FW_OFF);
		}
	}
#endif
    info->i2c_info.read_irq = SEC_NFC_SKIP;
    mutex_unlock(&info->i2c_info.read_mutex);
#endif

#ifdef CONFIG_ESE_COLDRESET
	mdelay(1);
	pr_err("FIRMWARE_GUARD_TIME(+1ms) in PW_OFF(total:4ms)\n");
#endif

	gpio_set_value(pdata->ven, SEC_NFC_PW_OFF);
	if (pdata->firm)
		gpio_set_value(pdata->firm, SEC_NFC_FW_OFF);

	if (mode == SEC_NFC_MODE_BOOTLOADER)
		if (pdata->firm)
			gpio_set_value(pdata->firm, SEC_NFC_FW_ON);
#ifdef CONFIG_SEC_NFC_RETRY
	if (mode == SEC_NFC_MODE_FIRMWARE) {
		info->retry_info.is_active = true;
	} else {
		info->retry_info.is_active = false;
	}
#endif
    if (mode != SEC_NFC_MODE_OFF) {
		msleep(SEC_NFC_VEN_WAIT_TIME);
		gpio_set_value(pdata->ven, SEC_NFC_PW_ON);
		sec_nfc_clk_ctl_enable(info);
#ifdef CONFIG_SEC_NFC_DEDICATED_CLK
		val |= 0xC0000000;
		writel_relaxed (val, info->clkctrl);
#endif
#ifdef CONFIG_SEC_NFC_IF_I2C
		enable_irq_wake(info->i2c_info.i2c_dev->irq);
#endif
		msleep(SEC_NFC_VEN_WAIT_TIME/2);
    } else {
#ifdef CONFIG_ESE_COLDRESET
		int PW_OFF_DURATION = 50;
		mdelay(PW_OFF_DURATION);
		gpio_set_value(pdata->ven, SEC_NFC_PW_ON);
		pr_err("DeepStby: enter DeepStby(PW_ON)\n");
#endif
		sec_nfc_clk_ctl_disable(info);
#ifdef CONFIG_SEC_NFC_DEDICATED_CLK
		val &= ~0xC0000000;
		writel_relaxed (val, info->clkctrl);
#endif
#ifdef CONFIG_SEC_NFC_IF_I2C
		disable_irq_wake(info->i2c_info.i2c_dev->irq);
#endif
    }

#ifdef CONFIG_SEC_NFC_WAKE_LOCK
	if (info->nfc_wake_lock->active)
		__pm_relax(info->nfc_wake_lock);
#endif

	pr_info("%s NFC mode is : %d\n", __func__, mode);
}

#ifdef CONFIG_ESE_COLDRESET
struct cold_reset_gpio {
	int firm_gpio;
	int coldreset_gpio;
};

struct cold_reset_gpio cold_reset_gpio_data;

void init_coldreset_mutex(void)
{
	mutex_init(&coldreset_mutex);
}

void init_sleep_wake_mutex(void)
{
	mutex_init(&sleep_wake_mutex);
}

void check_and_sleep_nfc(unsigned int gpio, int value)
{
	if (sleep_wakeup_state[IDX_SLEEP_WAKEUP_NFC] == true ||
			sleep_wakeup_state[IDX_SLEEP_WAKEUP_ESE] == true) {
		pr_info("%s keep wake up state\n", __func__);
		return;
	}
	gpio_set_value(gpio, value);
}

int trig_cold_reset_id(int id)
{
	int wakeup_delay = 20;
	int duration = 18;
	//struct timeval t0, t1, t2; //This function is deprecated
	struct timespec t0, t1, t2;
	int isFirmHigh = 0;
	int ret = 0;

	if (id == ESE_ID)
		mutex_lock(&coldreset_mutex);

	//do_gettimeofday(&t0); //This function is deprecated
    getnstimeofday(&t0);
	if (gpio_get_value(cold_reset_gpio_data.firm_gpio) == 1) {
		isFirmHigh = 1;
	} else {
		gpio_set_value(cold_reset_gpio_data.firm_gpio, SEC_NFC_FW_ON);
		mdelay(wakeup_delay);
	}

	//do_gettimeofday(&t1);
    getnstimeofday(&t1);
	gpio_set_value(cold_reset_gpio_data.coldreset_gpio, SEC_NFC_COLDRESET_ON);
	mdelay(duration);
	gpio_set_value(cold_reset_gpio_data.coldreset_gpio, SEC_NFC_COLDRESET_OFF);
	//do_gettimeofday(&t2);
    getnstimeofday(&t2);
	ret = gpio_get_value(cold_reset_gpio_data.coldreset_gpio);

	if (isFirmHigh == 1)
		pr_err("COLDRESET: FW_PIN already high, do not FW_OFF\n");
	else
		gpio_set_value(cold_reset_gpio_data.firm_gpio, SEC_NFC_FW_OFF);

	pr_err("COLDRESET: FW_ON time (%ld-%ld)\n", t0.tv_nsec*1000, t1.tv_nsec*1000);
	//USEIT: instead t0.tv_usec, use t0.tv_nsec / 1000);
	pr_err("COLDRESET: GPIO3 ON time (%ld-%ld)\n", t1.tv_nsec*1000, t2.tv_nsec*1000);

	if (id == ESE_ID)
		mutex_unlock(&coldreset_mutex);

	pr_err("COLDRESET: exit");
	return ret;
}

int trig_cold_reset(void)
{
    pr_info("%s trig_cold_reset\n", __func__);
    return trig_cold_reset_id(ESE_ID);
}
EXPORT_SYMBOL(trig_cold_reset);
int trig_nfc_wakeup(void)
{
	pr_info("%s trig_nfc_wakeup\n", __func__);

	if (cur_mode == SEC_NFC_MODE_OFF || cur_mode == SEC_NFC_MODE_BOOTLOADER) {
		pr_err("nfc mode not support to wake up");
		return -EPERM;;
	}
	mutex_lock(&sleep_wake_mutex);
	gpio_set_value(cold_reset_gpio_data.firm_gpio, SEC_NFC_WAKE_UP);
	sleep_wakeup_state[IDX_SLEEP_WAKEUP_ESE] = true;
	mutex_unlock(&sleep_wake_mutex);
	return 0;
}
EXPORT_SYMBOL(trig_nfc_wakeup);

int trig_nfc_sleep(void)
{
	pr_info("%s trig_nfc_sleep\n", __func__);
	if (cur_mode == SEC_NFC_MODE_OFF || cur_mode == SEC_NFC_MODE_BOOTLOADER) {
		pr_err("nfc mode not support to sleep");
		return -EPERM;;
	}
	mutex_lock(&sleep_wake_mutex);
	sleep_wakeup_state[IDX_SLEEP_WAKEUP_ESE] = false;
	check_and_sleep_nfc(cold_reset_gpio_data.firm_gpio, SEC_NFC_WAKE_SLEEP);
	mutex_unlock(&sleep_wake_mutex);
	return 0;
}
EXPORT_SYMBOL(trig_nfc_sleep);
#endif

static long sec_nfc_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
    struct sec_nfc_info *info = container_of(file->private_data,
						struct sec_nfc_info, miscdev);
    struct sec_nfc_platform_data *pdata = info->pdata;
    unsigned int new = (unsigned int)arg;
    int ret = 0;

    dev_dbg(info->dev, "%s: info: %p, cmd: 0x%x\n",
			__func__, info, cmd);

    mutex_lock(&info->mutex);
#ifdef CONFIG_ESE_COLDRESET
	mutex_lock(&coldreset_mutex);
#endif

	switch (cmd) {
	case SEC_NFC_SET_MODE:
		need_send_no_resp_cmd = false;
		if (info->mode == new)
			break;

		if (new >= SEC_NFC_MODE_COUNT) {
			dev_err(info->dev, "wrong mode (%d)\n", new);
			ret = -EFAULT;
			break;
		}
		sec_nfc_set_mode(info, new);

		break;

#if defined(CONFIG_SEC_NFC_PRODUCT_N3)
	case SEC_NFC_SLEEP:
	case SEC_NFC_WAKEUP:
		break;

#elif defined(CONFIG_SEC_NFC_PRODUCT_N5) || defined(CONFIG_SEC_NFC_PRODUCT_N7)
	case SEC_NFC_SLEEP:
		if (info->mode != SEC_NFC_MODE_BOOTLOADER) {
#ifdef CONFIG_SEC_NFC_WAKE_LOCK
			if (info->nfc_wake_lock->active)
				__pm_relax(info->nfc_wake_lock);
#endif
#ifdef CONFIG_ESE_COLDRESET
			mutex_lock(&sleep_wake_mutex);
			sleep_wakeup_state[IDX_SLEEP_WAKEUP_NFC] = false;
			check_and_sleep_nfc(pdata->wake, SEC_NFC_WAKE_SLEEP);
			mutex_unlock(&sleep_wake_mutex);
#else
			gpio_set_value(pdata->wake, SEC_NFC_WAKE_SLEEP);
			pr_err("%s: device sleep!!!  GPIO0 = %d\n", __func__, gpio_get_value(pdata->wake));
#endif
		}
		break;

	case SEC_NFC_WAKEUP:
		if (info->mode != SEC_NFC_MODE_BOOTLOADER) {
			gpio_set_value(pdata->wake, SEC_NFC_WAKE_UP);
			device_wakeup_flag = true;
			pr_err("%s: device  wakeup!!  GPIO0 = %d ,device_wakeup_flag set true\n", __func__, gpio_get_value(pdata->wake));
#ifdef CONFIG_ESE_COLDRESET
			sleep_wakeup_state[IDX_SLEEP_WAKEUP_NFC] = true;
#endif
#ifdef CONFIG_SEC_NFC_WAKE_LOCK
			if (!info->nfc_wake_lock->active)
				__pm_stay_awake(info->nfc_wake_lock);
#endif
		}
		break;
#endif
#ifdef CONFIG_ESE_COLDRESET
	case SEC_NFC_COLD_RESET:
		ret = trig_cold_reset_id(DEVICEHOST_ID);
		break;

	case SEC_ESE_COLD_RESET:
		ret = trig_cold_reset();
		break;
	case SEC_NFC_SHUTDOWN:
		//do nothing
		break;
#endif
#ifdef CONFIG_VIVO_NFC_FACTORY_SMT_TEST
	case SEC_NFC_FACTORY_SMT_TEST_BEGIN: //before check nfc ic in factory smt, should set clkreq by config output and pull down
	   pr_info("%s: SEC_NFC_FACTORY_SMT_TEST_BEGIN \n", __func__);
	   ret = pinctrl_select_state(nfc_pinctrl, nfc_pinctrl_clkreq_output);
		if (ret != 0) {
			pr_err("nfc_pinctrl nfc_pinctrl_clkreq_output status =%d\n", ret);
		}
		gpio_direction_output(pdata->clkreq, 0);
		gpio_set_value(pdata->clkreq, 0);
		break;
	case SEC_NFC_FACTORY_SMT_TEST_END: //after check nfc ic in factory smt, should set clkreq config by nfc clk req
	   pr_info("%s: SEC_NFC_FACTORY_SMT_TEST_END \n", __func__);
	   ret = pinctrl_select_state(nfc_pinctrl, nfc_pinctrl_default);
		if (ret != 0) {
			pr_err("nfc_pinctrl nfc_pinctrl_default status =%d\n", ret);
		}
		break;
#endif
    default:
		dev_err(info->dev, "Unknown ioctl 0x%x\n", cmd);
		ret = -ENOIOCTLCMD;
		break;
	}

#ifdef CONFIG_ESE_COLDRESET
	mutex_unlock(&coldreset_mutex);
#endif
    mutex_unlock(&info->mutex);

    return ret;
}

static int sec_nfc_open(struct inode *inode, struct file *file)
{
    struct sec_nfc_info *info = container_of(file->private_data,
						struct sec_nfc_info, miscdev);
    int ret = 0;

    pr_info("%s  info : %p\n", __func__, info);
    mutex_lock(&info->mutex);
    if (info->mode != SEC_NFC_MODE_OFF) {
		dev_err(info->dev, "sec_nfc is busy\n");
		ret = -EBUSY;
		goto out;
    }

    sec_nfc_set_mode(info, SEC_NFC_MODE_OFF);

out:
    mutex_unlock(&info->mutex);
    return ret;
}

static int sec_nfc_close(struct inode *inode, struct file *file)
{
    struct sec_nfc_info *info = container_of(file->private_data,
						struct sec_nfc_info, miscdev);

    pr_info("%s: info : %p\n", __func__, info);

    mutex_lock(&info->mutex);
    sec_nfc_set_mode(info, SEC_NFC_MODE_OFF);
    mutex_unlock(&info->mutex);

    return 0;
}

static const struct file_operations sec_nfc_fops = {
    .owner      = THIS_MODULE,
    .read       = sec_nfc_read,
    .write      = sec_nfc_write,
    .poll       = sec_nfc_poll,
    .open       = sec_nfc_open,
    .release    = sec_nfc_close,
    .unlocked_ioctl = sec_nfc_ioctl,
};

#ifdef CONFIG_PM
static int sec_nfc_suspend(struct device *dev)
{
    struct sec_nfc_info *info = SEC_NFC_GET_INFO(dev);
    int ret = 0;

    mutex_lock(&info->mutex);

    if (info->mode == SEC_NFC_MODE_BOOTLOADER)
		ret = -EPERM;

    mutex_unlock(&info->mutex);

	return ret;
}

static int sec_nfc_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(sec_nfc_pm_ops, sec_nfc_suspend, sec_nfc_resume);
#endif

#ifdef CONFIG_OF
/*device tree parsing*/
static int sec_nfc_parse_dt(struct device *dev,
	struct sec_nfc_platform_data *pdata)
{
	struct device_node *np = dev->of_node;


	pdata->ven = of_get_named_gpio(np, "sec-nfc,ven-gpio", 0);
	pdata->firm = of_get_named_gpio(np, "sec-nfc,firm-gpio", 0);
	pdata->wake = pdata->firm;
#ifdef CONFIG_SEC_NFC_IF_I2C
	pdata->irq = of_get_named_gpio(np, "sec-nfc,irq-gpio", 0);
#endif
#ifdef CONFIG_VIVO_NFC_FACTORY_SMT_TEST
    pdata->clkreq = of_get_named_gpio(np, "sec-nfc,clkreq-gpio", 0);
#endif

#ifdef CONFIG_ESE_COLDRESET
	pdata->coldreset = of_get_named_gpio(np, "sec-nfc,coldreset-gpio", 0);
	cold_reset_gpio_data.firm_gpio = pdata->firm;
	cold_reset_gpio_data.coldreset_gpio = pdata->coldreset;
#endif

#ifdef CONFIG_SEC_NFC_LDO_EN
	pdata->pvdd_en = of_get_named_gpio(np, "sec-nfc,ldo_en", 0);
#endif
#ifdef CONFIG_SEC_NFC_PMIC_LDO
	if (of_property_read_string(np, "sec-nfc,pmic-ldo", &pdata->pvdd_regulator_name))
		pr_err("%s: Failed to get %s regulator.\n", __func__, pdata->pvdd_regulator_name);
#endif

#ifdef CONFIG_SEC_NFC_DEDICATED_CLK
	if (of_property_read_u32(np, "clkctrl-reg", (u32 *)&pdata->clkctrl_addr))
		return -EINVAL;
#endif
#ifdef CONFIG_SEC_NFC_GPIO_CLK
	pdata->clk_req = of_get_named_gpio(np, "sec-nfc,clk_req-gpio", 0);
	pdata->clk = clk_get(dev, "OSC_NFC");
#endif
#ifdef CONFIG_VIVO_NFC_FACTORY_SMT_TEST
	nfc_pinctrl = devm_pinctrl_get(dev);
	nfc_pinctrl_clkreq_output = pinctrl_lookup_state(nfc_pinctrl, "clkreq-smt");
	if (IS_ERR(nfc_pinctrl_clkreq_output)) {
		pr_err("failed to get nfc_pinctrl_clkreq_output	pin state\n");
	}
	nfc_pinctrl_default = pinctrl_lookup_state(nfc_pinctrl, "default");
	if (IS_ERR(nfc_pinctrl_default)) {
		pr_err("failed to get nfc_pinctrl_default  pin state\n");
	}
#endif

    pr_err("%s: irq : %d, ven : %d, firm : %d, pdata->coldreset %d\n",
			__func__, pdata->irq, pdata->ven, pdata->firm, pdata->coldreset);
//    pr_err("%s: get %s regulator.\n", __func__, pdata->pvdd_regulator_name);
    //pr_err(" %s, pdata->clk_req %d\n", __func__, pdata->clk_req);

	return 0;
}

#else
static int sec_nfc_parse_dt(struct device *dev,
	struct sec_nfc_platform_data *pdata)
{
	return -ENODEV;
}
#endif

/*vivo caibinchen add a node for load nfc service start*/
static ssize_t nfc_enable_object_show(struct kobject *k, struct attribute *attr, char *buf)
{
	if (nfc_result == 1) {
		pr_info("nfc_enable_show result 1, sn4v retrun 3");
		return snprintf(buf, 2, "%d", 3);
	} else {
		pr_info("nfc_enable_show result 0");
		return snprintf(buf, 2, "%d", 0);
	}
}

static ssize_t nfc_enable_object_store(struct kobject *k, struct attribute *attr,
				const char *buf, size_t count)
{
	pr_info("nfc_enable_store result 0");
	return nfc_result;
}

static void nfc_enable_object_release(struct kobject *kobj)
{
	/* nothing to do temply */
	return;
}

static const struct sysfs_ops nfc_enable_object_sysfs_ops = {
	.show = nfc_enable_object_show,
	.store = nfc_enable_object_store,
};

static struct kobj_type nfc_enable_object_type = {
	.sysfs_ops	= &nfc_enable_object_sysfs_ops,
	.release	= nfc_enable_object_release,
	.default_attrs = our_own_sys_attrs,
};

int is_support_nfc(struct device *dev)
{
	int i = 0;
	int ret = 0;
	char *board_version = NULL;
	struct device_node *np = dev->of_node;
	ret = of_property_read_u32(np, DTS_NFC_SUPPORT_STR, &nfc_support);
	if (ret < 0) {
		pr_err("nfc Failure reading vivo,nfc_support, ret = %d\n", ret);
		nfc_support = 0;
		return -EINVAL;
	}
	printk("nfc nfc_support:%d\n", nfc_support);
	ret = of_property_read_u32(np, DTS_BOARDVERSION_SHIFT_STR, &boardversion_shift);
	if (ret < 0) {
		pr_err("nfc Failure reading vivo,boardversion_shift, ret = %d\n", ret);
		return -EINVAL;
	}
	printk("nfc boardversion_shift:%d\n", boardversion_shift);
	ret = of_property_read_u32(np, DTS_BOARDVERSION_MASK_STR, &boardversion_mask);
	if (ret < 0) {
		pr_err("nfc Failure reading vivo,boardversion_mask, ret = %d\n", ret);
		return -EINVAL;
	}
	printk("nfc boardversion_mask:%d\n", boardversion_mask);
	ret = of_property_read_u32(np, DTS_BOARDVERSION_NUM_STR, &boardversion_num);
	if (ret < 0) {
		pr_err("nfc Failure reading vivo,boardversion_num, ret = %d\n", ret);
		return -EINVAL;
	}
	printk("nfc boardversion_num:%d\n", boardversion_num);
	if (boardversion_num > 0) {
		ret = of_property_read_string_array(np, DTS_BOARDVERSION_STR, boardversions, boardversion_num);
		if (ret < 0) {
			pr_err("nfc Failure reading vivo,boardversions, ret = %d\n", ret);
			return -EINVAL;
		}
		for (i = 0; i < boardversion_num; i++)
			printk("nfc :%s\n", boardversions[i]);
	}
	if (nfc_support == 1) {
		if (boardversion_num == 0) {
				pr_err("%s: The machine has NFC HW\n", __func__);
				nfc_result = 1;
		} else {
			//board_version = "0";
			board_version = get_board_version();
			pr_err("%s:nfc board_version is %s:***********\n", __func__, board_version);
			for (i = 0; i < boardversion_num; i++) {
				// printk("nfc:%s,%s\n",boardversions[i],&(board_version[boardversion_shift]));
				if (strncmp(boardversions[i], &(board_version[boardversion_shift]), boardversion_mask) == 0) {
					pr_err("%s: The machine has NFC HW\n", __func__);
					nfc_result = 1;
					break;
				}
			}
			if (i == boardversion_num) {
				nfc_result = 0;
			}
		}
	} else {
		nfc_result = 0;
	}
	return nfc_result;
}
/*add a node for load nfc service end*/

static int __sec_nfc_probe(struct device *dev)
{
	struct sec_nfc_info *info;
	struct sec_nfc_platform_data *pdata = NULL;
	int ret = 0;

	pr_err("%s : start\n", __func__);

	ret = kobject_init_and_add(&nfc_kobject, &nfc_enable_object_type, NULL, "nfc");
	if (ret) {
		pr_err("%s: Create nfc error!", __func__);
		return -ENOMEM;
	}

	if (dev->of_node) {
		pdata = devm_kzalloc(dev,
			sizeof(struct sec_nfc_platform_data), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;

		ret = sec_nfc_parse_dt(dev, pdata);
		if (ret)
			return ret;
	} else
		pdata = dev->platform_data;

	if (!pdata) {
		dev_err(dev, "No platform data\n");
		ret = -ENOMEM;
		goto err_pdata;
	}
// Temporary block, get_bbk_board_version not working now.
	ret = is_support_nfc(dev);
	if (ret < 0) {
		pr_err("%s: The machine hasn't NFC HW, ret = %d\n", __func__, ret);
		goto err_pdata;
	}

	info = kzalloc(sizeof(struct sec_nfc_info), GFP_KERNEL);
	if (!info) {
		ret = -ENOMEM;
		goto err_info_alloc;
	}
	info->dev = dev;
	info->pdata = pdata;
	info->mode = SEC_NFC_MODE_OFF;

	mutex_init(&info->mutex);
	dev_set_drvdata(dev, info);

	info->miscdev.minor = MISC_DYNAMIC_MINOR;
	info->miscdev.name = SEC_NFC_DRIVER_NAME;
	info->miscdev.fops = &sec_nfc_fops;
	info->miscdev.parent = dev;
	ret = misc_register(&info->miscdev);
	if (ret < 0) {
		dev_err(dev, "failed to register Device\n");
		goto err_dev_reg;
	}

#ifdef CONFIG_SEC_NFC_DEDICATED_CLK
	if (pdata->clkctrl_addr != 0) {
		info->clkctrl = ioremap_nocache(pdata->clkctrl_addr, 0x80);
		if (!info->clkctrl) {
			dev_err(dev, "cannot remap nfc clock register\n");
			ret = -ENXIO;
			goto err_iomap;
		}
	}
#endif
#ifdef CONFIG_SEC_NFC_LDO_EN
	{
		ret = gpio_request(pdata->pvdd_en, "ldo_en");
		if (ret < 0) {
			pr_err("failed to request about pvdd_en pin\n");
			goto err_dev_reg;
		}
		if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT)
			gpio_direction_output(pdata->pvdd_en, NFC_I2C_LDO_ON);
		pr_err("pvdd en: %d\n", gpio_get_value(pdata->pvdd_en));
    }
#endif
#ifdef CONFIG_SEC_NFC_PMIC_LDO
	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT) {
		pdata->pvdd_regulator = regulator_get(NULL, pdata->pvdd_regulator_name);
		if (IS_ERR(pdata->pvdd_regulator))
			pr_err("%s: Failed to get %s pmic regulator.\n", __func__, pdata->pvdd_regulator);
		ret = regulator_enable(pdata->pvdd_regulator);
		if (ret < 0)
			pr_err("%s: Failed to enable pmic regulator: %d\n", __func__, ret);
    }
    pr_err("%s: nfc success to get %s pmic regulator.\n", __func__, pdata->pvdd_regulator);
#endif
    ret = gpio_request(pdata->ven, "nfc_ven");
    if (ret) {
        dev_err(dev, "failed to get gpio ven\n");
        goto err_gpio_ven;
    }
    if ((get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT) || (get_boot_mode() == RECOVERY_BOOT))
		gpio_direction_output(pdata->ven, SEC_NFC_PW_ON);
	else
		gpio_direction_output(pdata->ven, SEC_NFC_PW_OFF);
    pr_err("nfc_ven en: %d\n", gpio_get_value(pdata->ven));
    if (pdata->firm) {
		ret = gpio_request(pdata->firm, "nfc_firm");
		if (ret) {
			dev_err(dev, "failed to get gpio firm\n");
			goto err_gpio_firm;
		}
		gpio_direction_output(pdata->firm, SEC_NFC_FW_OFF);
    }
    pr_err("nfc_firm  en: %d\n", gpio_get_value(pdata->firm));
#ifdef CONFIG_VIVO_NFC_FACTORY_SMT_TEST
    ret = gpio_request(pdata->clkreq, "nfc_clkreq");
    if (ret) {
        dev_err(dev, "failed to get gpio ven\n");
        goto err_gpio_clkreq;
    }
#endif

#ifdef CONFIG_ESE_COLDRESET
		init_coldreset_mutex();
		init_sleep_wake_mutex();
		memset(sleep_wakeup_state, false, sizeof(sleep_wakeup_state));
		ret = gpio_request(pdata->coldreset, "nfc_coldreset");
		if (ret) {
				dev_err(dev, "failed to get gpio coldreset(NFC-GPIO3)\n");
				goto err_gpio_coldreset;
		}
		gpio_direction_output(pdata->coldreset, SEC_NFC_COLDRESET_OFF);
		dev_err(dev, " nfc get gpio coldreset(NFC-GPIO3");
		pr_err("coldreset en: %d\n", gpio_get_value(pdata->coldreset));
#endif
#ifdef CONFIG_SEC_NFC_WAKE_LOCK
    // wake_lock_init(&info->nfc_wake_lock, WAKE_LOCK_SUSPEND, "nfc_wake_lock");
	info->nfc_wake_lock = wakeup_source_create("nfc_wake_lock");
	if (!info->nfc_wake_lock)
		pr_err("failed to create nfc_wake_lock\n");
	else
		wakeup_source_add(info->nfc_wake_lock);
#endif
#ifdef CONFIG_SEC_NFC_RETRY
	INIT_DELAYED_WORK(&info->retry_info.retry_work, retry_wq_handler);
	info->retry_info.buf = kzalloc(SEC_NFC_MAX_BUFFER_SIZE, GFP_KERNEL);
#endif
    dev_err(dev, "%s: success info: %p, pdata %p\n", __func__, info, pdata);

    return 0;

#ifdef CONFIG_ESE_COLDRESET
err_gpio_coldreset:
	gpio_free(pdata->coldreset);
#endif
#ifdef CONFIG_VIVO_NFC_FACTORY_SMT_TEST
err_gpio_clkreq:
    gpio_free(pdata->clkreq);
#endif
err_gpio_firm:
    gpio_free(pdata->ven);
err_gpio_ven:
#ifdef CONFIG_SEC_NFC_DEDICATED_CLK
    iounmap(info->clkctrl);
err_iomap:
#endif
err_dev_reg:
    mutex_destroy(&info->mutex);
    kfree(info);
err_info_alloc:
    devm_kfree(dev, pdata);
err_pdata:
    return ret;
}

static int __sec_nfc_remove(struct device *dev)
{
    struct sec_nfc_info *info = dev_get_drvdata(dev);
    struct i2c_client *client = info->i2c_info.i2c_dev;
    struct sec_nfc_platform_data *pdata = info->pdata;

    dev_dbg(info->dev, "%s\n", __func__);

#ifdef CONFIG_SEC_NFC_GPIO_CLK
    if (pdata->clk)
		clk_unprepare(pdata->clk);
#endif

    misc_deregister(&info->miscdev);
    sec_nfc_set_mode(info, SEC_NFC_MODE_OFF);
    free_irq(client->irq, info);
    gpio_free(pdata->irq);
    gpio_set_value(pdata->firm, 0);
    gpio_free(pdata->ven);
    if (pdata->firm)
		gpio_free(pdata->firm);

//add a node for load nfc service start
	kobject_del(&nfc_kobject);
//add a node for load nfc service end

#ifdef CONFIG_SEC_NFC_WAKE_LOCK
    // wake_lock_destroy(&info->nfc_wake_lock);
	wakeup_source_remove(info->nfc_wake_lock);
	wakeup_source_destroy(info->nfc_wake_lock);
#endif
#ifdef CONFIG_SEC_NFC_RETRY
	cancel_delayed_work(&info->retry_info.retry_work);
#endif
    kfree(info);

    return 0;
}

#ifdef CONFIG_SEC_NFC_IF_I2C
typedef struct i2c_driver sec_nfc_driver_type;
#define SEC_NFC_INIT(driver)    i2c_add_driver(driver)
#define SEC_NFC_EXIT(driver)    i2c_del_driver(driver)

static int sec_nfc_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
    int ret = 0;

    ret = __sec_nfc_probe(&client->dev);
    if (ret)
		return ret;
	printk("zms %s\n", __func__);
    if (sec_nfc_i2c_probe(client))
		__sec_nfc_remove(&client->dev);
	printk("zms aa %s\n", __func__);

    return ret;
}

static int sec_nfc_remove(struct i2c_client *client)
{
    sec_nfc_i2c_remove(&client->dev);
    return __sec_nfc_remove(&client->dev);
}

static struct i2c_device_id sec_nfc_id_table[] = {
    { SEC_NFC_DRIVER_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, sec_nfc_id_table);
#else   /* CONFIG_SEC_NFC_IF_I2C */
MODULE_DEVICE_TABLE(platform, sec_nfc_id_table);
typedef struct platform_driver sec_nfc_driver_type;
#define SEC_NFC_INIT(driver)    platform_driver_register(driver);
#define SEC_NFC_EXIT(driver)    platform_driver_unregister(driver);

static int sec_nfc_probe(struct platform_device *pdev)
{
    return __sec_nfc_probe(&pdev->dev);
}

static int sec_nfc_remove(struct platform_device *pdev)
{
    return __sec_nfc_remove(&pdev->dev);
}

static struct platform_device_id sec_nfc_id_table[] = {
    { SEC_NFC_DRIVER_NAME, 0 },
    { }
};

#endif /* CONFIG_SEC_NFC_IF_I2C */

#ifdef CONFIG_OF
static const struct of_device_id nfc_match_table[] = {
    { .compatible = SEC_NFC_DRIVER_NAME,},
    {},
};
#else
#define nfc_match_table NULL
#endif

static struct i2c_driver sec_nfc_driver = {
    .probe = sec_nfc_probe,
    .id_table = sec_nfc_id_table,
    .remove = sec_nfc_remove,
    .driver = {
		.name = SEC_NFC_DRIVER_NAME,
#ifdef CONFIG_PM
		.pm = &sec_nfc_pm_ops,
#endif
		.of_match_table = nfc_match_table,
    },
};

static int __init sec_nfc_init(void)
{
    return SEC_NFC_INIT(&sec_nfc_driver);
}

static void __exit sec_nfc_exit(void)
{
    SEC_NFC_EXIT(&sec_nfc_driver);
}

module_init(sec_nfc_init);
module_exit(sec_nfc_exit);

MODULE_DESCRIPTION("Samsung sec_nfc driver");
MODULE_LICENSE("GPL");
