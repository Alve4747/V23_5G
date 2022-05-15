/* Version: v1.3.5
 *
 * Copyright (c) 2018 AWINIC Technology CO., LTD
 *
 *  Author: Nick Li < liweilei@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#define DEBUG
#define pr_fmt(fmt) "lra_haptic: " fmt
// #define dev_fmt(fmt) "awinic_haptics: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/syscalls.h>
#include <linux/power_supply.h>
#include <linux/vmalloc.h>
#include <linux/pm_qos.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/control.h>
#include <sound/soc.h>

#include "aw8624_reg.h"
#include "aw8624.h"

#include <linux/mman.h>

#define AW8624_VERSION "v2021.6.27"

#define AW8624_I2C_NAME "lra_haptic_aw8624"

#define AW_I2C_RETRIES 2
#define AW_I2C_RETRY_DELAY 2

#define RTP_SLAB_SIZE 512
#define RTP_BIN_MAX_SIZE 2000000 //用于加载RTP的固件

#define BASE_SCENE_COUNT_MAX 300

#define DEFAULT_CALI_F0 0
#define DEFAULT_OSC_CALI_DATA 0

#define PLAYBACK_INFINITELY_RAM_ID 6 //用于长振的RAM_ID

#define AWINIC_RAM_UPDATE_DELAY

static volatile int i2c_suspend;

static struct pm_qos_request pm_qos_req_vb;

static struct kmem_cache *rtp_cachep;
static struct haptic_rtp_container *aw8624_rtp;

static struct aw8624 *g_aw8624;

static struct workqueue_struct *rtp_wq;

static bool at_test;
static bool ram_load = false;

static bool rtp_check_flag;
static int g_order;


static volatile int resist_flag = 1;

static struct haptic_wavefrom_info waveform_list_default[1] = {

	{1, 1, 8000, 12000, false, "default"},
};

static char *aw8624_ram_name = "aw8624_haptic.bin";

static bool g_logDts = true;

// Prototype
static int aw8624_ram_update(struct aw8624 *aw8624);

/***********************************************************************************************
*
* aw8624 i2c write/read
*
***********************************************************************************************/

/* returning negative errno else zero on success */
static int aw8624_i2c_write(struct aw8624 *aw8624, unsigned char reg_addr, unsigned char reg_data)
{
	int ret = 0;
	unsigned char cnt = 0;

	if (i2c_suspend) {
		dev_err(aw8624->dev, "%s device in suspend, skip IIC Control\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&aw8624->bus_lock);
	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_write_byte_data(aw8624->i2c, reg_addr, reg_data);
		if (ret) {
			dev_err(aw8624->dev, "%s: error: addr=%#x data=%#x cnt=%d error=%d\n", __func__,
				reg_addr, reg_data, cnt, ret);
		} else {
			break;
		}
		cnt++;
		usleep_range(AW_I2C_RETRY_DELAY * 1000, AW_I2C_RETRY_DELAY * 1000 + 500);

	}
	mutex_unlock(&aw8624->bus_lock);

	return ret;
}

/* returning negative errno on failure, else a data byte received from the device*/
static int aw8624_i2c_read(struct aw8624 *aw8624, unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = 0;
	unsigned char cnt = 0;

	if (i2c_suspend) {
		dev_err(aw8624->dev, "%s device in suspend, skip IIC Control\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&aw8624->bus_lock);
	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(aw8624->i2c, reg_addr);
		if (ret < 0) {
			dev_err(aw8624->dev, "%s: error: addr=%#x cnt=%d error=%d\n", __func__, reg_addr, cnt, ret);
		} else {
			*reg_data = ret;
			break;
		}
		cnt++;
		usleep_range(AW_I2C_RETRY_DELAY * 1000, AW_I2C_RETRY_DELAY * 1000 + 500);
	}
	mutex_unlock(&aw8624->bus_lock);

	return ret;
}

/* returning negative errno else zero on success */
static int aw8624_i2c_write_bits(struct aw8624 *aw8624, unsigned char reg_addr,
								unsigned int mask, unsigned char reg_data)
{
	unsigned char reg_val = 0;
	int ret = 0;

	if (i2c_suspend) {
		dev_err(aw8624->dev, "%s device in suspend, skip IIC Control\n", __func__);
		return -EINVAL;
	}

	ret = aw8624_i2c_read(aw8624, reg_addr, &reg_val);
	if (ret < 0) {
		dev_err(aw8624->dev, "%s i2c read failed, ret=%d\n", __func__, ret);
		return ret;
	}

	reg_val &= mask;
	reg_val |= reg_data;
	ret = aw8624_i2c_write(aw8624, reg_addr, reg_val);
	if (ret < 0) {
		dev_err(aw8624->dev, "%s: i2c write failed, ret=%d\n", __func__, ret);
		return ret;
	}

	return 0;
}

/* returning negative errno on failure, or else the number of bytes written */
static int aw8624_i2c_writes(struct aw8624 *aw8624,
		unsigned char reg_addr, unsigned char *buf, unsigned int len)
{
	int ret = 0;
	unsigned char *data = NULL;

	if (i2c_suspend) {
		dev_err(aw8624->dev, "%s device in suspend, skip IIC Control\n", __func__);
		return -EINVAL;
	}

	if ((len > RTP_SLAB_SIZE) || (rtp_cachep == NULL)) {

		data = kzalloc(len+1, GFP_KERNEL);
		if (data == NULL) {
			dev_err(aw8624->dev, "%s: can not allocate memory\n", __func__);
			return -ENOMEM;
		}
	} else {
		data = kmem_cache_alloc(rtp_cachep, GFP_KERNEL);
		if (!data) {
			dev_err(aw8624->dev, "%s can not alloc cache memory\n", __func__);
			return -ENOMEM;
		}
	}

	data[0] = reg_addr;
	memcpy(&data[1], buf, len);

	mutex_lock(&aw8624->bus_lock);

	ret = i2c_master_send(aw8624->i2c, data, len+1);
	if (ret < 0) {
		dev_err(aw8624->dev, "%s: i2c master send error\n", __func__);
	}

	mutex_unlock(&aw8624->bus_lock);

	if ((len > RTP_SLAB_SIZE) || (rtp_cachep == NULL)) {

		if (data != NULL)
			kfree(data);
	} else {
		if (data != NULL)
			kmem_cache_free(rtp_cachep, data);
	}

	return ret;
}

static void aw8624_interrupt_setup(struct aw8624 *aw8624)
{
	unsigned char reg_val = 0;

	aw8624_i2c_read(aw8624, AW8624_REG_SYSINT, &reg_val);

	dev_info(aw8624->dev, "%s: reg SYSINT=0x%x\n", __func__, reg_val);

	aw8624_i2c_write_bits(aw8624, AW8624_REG_DBGCTRL, AW8624_BIT_DBGCTRL_INT_MODE_MASK, AW8624_BIT_DBGCTRL_INT_MODE_EDGE);

	aw8624_i2c_write_bits(aw8624, AW8624_REG_SYSINTM, AW8624_BIT_SYSINTM_UVLO_MASK, AW8624_BIT_SYSINTM_UVLO_EN);
	aw8624_i2c_write_bits(aw8624, AW8624_REG_SYSINTM, AW8624_BIT_SYSINTM_OCD_MASK, AW8624_BIT_SYSINTM_OCD_EN);
	aw8624_i2c_write_bits(aw8624, AW8624_REG_SYSINTM, AW8624_BIT_SYSINTM_OT_MASK, AW8624_BIT_SYSINTM_OT_EN);

	aw8624_i2c_write_bits(aw8624, AW8624_REG_SYSINTM, AW8624_BIT_SYSINTM_DONE_MASK, AW8624_BIT_SYSINTM_DONE_EN);
}

static int aw8624_hw_reset(struct aw8624 *aw8624)
{

	if (gpio_is_valid(aw8624->reset_gpio)) {
		gpio_set_value_cansleep(aw8624->reset_gpio, 0);
		msleep(1);
		gpio_set_value_cansleep(aw8624->reset_gpio, 1);
		msleep(5);
	} else {
		dev_err(aw8624->dev, "%s: failed\n", __func__);
	}

	return 0;
}


static int aw8624_haptic_softreset(struct aw8624 *aw8624)
{

	aw8624_i2c_write(aw8624, AW8624_REG_ID, 0xAA);
	usleep_range(2000, 2500);
	return 0;
}


static int aw8624_read_chipid(struct aw8624 *aw8624)
{
	int ret = 0;
	unsigned char reg = 0;
	int retry = 3;

	while (retry--) {

		ret = aw8624_i2c_read(aw8624, AW8624_REG_ID, &reg);
		if (ret < 0) {
			dev_err(aw8624->dev, "%s: failed to read register AW8624_REG_ID: %d\n", __func__, ret);
			aw8624_hw_reset(aw8624);
			continue;
		}

		switch (reg) {
		case AW8624_CHIPID:
			dev_info(aw8624->dev, "%s aw8624 detected\n", __func__);
			aw8624->chipid = AW8624_CHIPID;
			aw8624_haptic_softreset(aw8624);
			return 0;
		default:
			dev_info(aw8624->dev, "%s unsupported device revision (0x%x)\n", __func__, reg);
			break;
		}
	}

	return -ENODEV;
}

/* Determine whether the motor has entered a steady state of rest */
static int aw8624_haptic_stop_delay(struct aw8624 *aw8624) //
{
	unsigned char reg_val = 0;
	unsigned int cnt = 40;

	while (cnt--) {
		aw8624_i2c_read(aw8624, AW8624_REG_GLB_STATE, &reg_val);
		if ((reg_val&0x0f) == 0x00) {
			dev_info(aw8624->dev, "%s finish\n", __func__);
			return 0;
		}

		dev_info(aw8624->dev, "%s wait for standby, glb_state=0x%02x\n",
								__func__, reg_val);

		mdelay(2);
	}
	dev_err(aw8624->dev, "%s do not enter standby automatically\n", __func__);

	return 0;
}

static int aw8624_haptic_set_gain(struct aw8624 *aw8624, unsigned char gain) //
{
	aw8624_i2c_write(aw8624, AW8624_REG_DATDBG, gain);
	dev_info(aw8624->dev, "%s, gain = %d\n", __func__, gain);

	return 0;
}

static int aw8624_haptic_play_init(struct aw8624 *aw8624)
{

	if (aw8624->play_mode == AW8624_HAPTIC_CONT_MODE) {
		aw8624_i2c_write(aw8624, AW8624_REG_SW_BRAKE, (unsigned char)0x2c);
	} else {
		aw8624_i2c_write(aw8624, AW8624_REG_SW_BRAKE, (unsigned char)0x08);
	}

	return 0;
}

static void aw8624_interrupt_clear(struct aw8624 *aw8624) //
{
	unsigned char reg_val = 0;
	aw8624_i2c_read(aw8624, AW8624_REG_SYSINT, &reg_val);
	dev_info(aw8624->dev, "%s, reg SYSINT=0x%x\n", __func__, reg_val);
}

static int aw8624_haptic_active(struct aw8624 *aw8624) //
{

	aw8624_haptic_play_init(aw8624);

	aw8624_i2c_write_bits(aw8624, AW8624_REG_SYSCTRL,
			AW8624_BIT_SYSCTRL_WORK_MODE_MASK, AW8624_BIT_SYSCTRL_ACTIVE);
	aw8624_interrupt_clear(aw8624);
	aw8624_i2c_write_bits(aw8624, AW8624_REG_SYSINTM,
			AW8624_BIT_SYSINTM_UVLO_MASK, AW8624_BIT_SYSINTM_UVLO_EN);

	return 0;
}

static int aw8624_haptic_play_go(struct aw8624 *aw8624, bool flag)
{
	s64 delta_ms = 0;

	if (flag == true) {
		aw8624_i2c_write_bits(aw8624, AW8624_REG_GO,
				AW8624_BIT_GO_MASK, AW8624_BIT_GO_ENABLE);
		aw8624->begin = ktime_get();
	} else {
		aw8624->cancel = ktime_get();
		delta_ms = ktime_to_ms(ktime_sub(aw8624->cancel, aw8624->begin));

		if (delta_ms < 5) {
			dev_info(aw8624->dev, "%s --->delta_ms=%d , interval too short, cancel delay 5ms\n", __func__, delta_ms);
			mdelay(5);
		}
		aw8624_i2c_write_bits(aw8624, AW8624_REG_GO,
				AW8624_BIT_GO_MASK, AW8624_BIT_GO_DISABLE);
	}

	return 0;
}

static int aw8624_haptic_play_mode(struct aw8624 *aw8624, unsigned char play_mode) //
{

	switch (play_mode) {
	case AW8624_HAPTIC_STANDBY_MODE:
		aw8624->play_mode = AW8624_HAPTIC_STANDBY_MODE;
		aw8624_i2c_write_bits(aw8624, AW8624_REG_SYSINTM,
				AW8624_BIT_SYSINTM_UVLO_MASK,
				AW8624_BIT_SYSINTM_UVLO_OFF);
		aw8624_i2c_write_bits(aw8624,
				AW8624_REG_SYSCTRL,
				AW8624_BIT_SYSCTRL_WORK_MODE_MASK,
				AW8624_BIT_SYSCTRL_STANDBY);
		break;
	case AW8624_HAPTIC_RAM_MODE:
		aw8624->play_mode = AW8624_HAPTIC_RAM_MODE;
		aw8624_i2c_write_bits(aw8624,
				AW8624_REG_SYSCTRL,
				AW8624_BIT_SYSCTRL_PLAY_MODE_MASK,
				AW8624_BIT_SYSCTRL_PLAY_MODE_RAM);
		aw8624_haptic_active(aw8624);
		break;
	case AW8624_HAPTIC_RAM_LOOP_MODE:
		aw8624->play_mode = AW8624_HAPTIC_RAM_LOOP_MODE;
		aw8624_i2c_write_bits(aw8624,
				AW8624_REG_SYSCTRL,
				AW8624_BIT_SYSCTRL_PLAY_MODE_MASK,
				AW8624_BIT_SYSCTRL_PLAY_MODE_RAM);
		aw8624_haptic_active(aw8624);
		break;
	case AW8624_HAPTIC_RTP_MODE:
		aw8624->play_mode = AW8624_HAPTIC_RTP_MODE;
		aw8624_i2c_write_bits(aw8624,
				AW8624_REG_SYSCTRL,
				AW8624_BIT_SYSCTRL_PLAY_MODE_MASK,
				AW8624_BIT_SYSCTRL_PLAY_MODE_RTP);
		aw8624_haptic_active(aw8624);
		break;
	case AW8624_HAPTIC_TRIG_MODE:
		aw8624->play_mode = AW8624_HAPTIC_TRIG_MODE;
		aw8624_i2c_write_bits(aw8624,
				AW8624_REG_SYSCTRL,
				AW8624_BIT_SYSCTRL_PLAY_MODE_MASK,
				AW8624_BIT_SYSCTRL_PLAY_MODE_RAM);
		aw8624_haptic_active(aw8624);
		break;
	case AW8624_HAPTIC_CONT_MODE:
		aw8624->play_mode = AW8624_HAPTIC_CONT_MODE;
		aw8624_i2c_write_bits(aw8624,
				AW8624_REG_SYSCTRL,
				AW8624_BIT_SYSCTRL_PLAY_MODE_MASK,
				AW8624_BIT_SYSCTRL_PLAY_MODE_CONT);
		aw8624_haptic_active(aw8624);
		break;
	default:
		dev_err(aw8624->dev, "%s: play mode %d err",
				__func__, play_mode);
		break;
	}
	return 0;
}

static int aw8624_haptic_stop(struct aw8624 *aw8624) //
{

	aw8624_haptic_play_go(aw8624, false);
	aw8624_haptic_stop_delay(aw8624);
	aw8624_haptic_play_mode(aw8624, AW8624_HAPTIC_STANDBY_MODE);

	return 0;
}

static int aw8624_haptic_start(struct aw8624 *aw8624)
{

	aw8624_haptic_play_go(aw8624, true);

	return 0;
}

static int aw8624_haptic_set_wav_seq(struct aw8624 *aw8624, //
		unsigned char wav, unsigned char seq)
{

	aw8624_i2c_write(aw8624, AW8624_REG_WAVSEQ1+wav, seq);
	return 0;
}

static void aw8624_double_click_switch(struct aw8624 *aw8624, bool sw)
{

	if (sw) {
		aw8624_haptic_set_wav_seq(aw8624, 0x00, 0x01);
		aw8624_haptic_set_wav_seq(aw8624, 0x01, ((AW8624_DUOBLE_CLICK_DELTA / SEQ_WAIT_UNIT) & 0x7f) | 0x80);
		aw8624_haptic_set_wav_seq(aw8624, 0x02, 0x01);
	} else {
		aw8624_haptic_set_wav_seq(aw8624, 0x00, 0x00);
		aw8624_haptic_set_wav_seq(aw8624, 0x01, 0x00);
		aw8624_haptic_set_wav_seq(aw8624, 0x02, 0x00);
	}
}


static int aw8624_haptic_set_wav_loop(struct aw8624 *aw8624, //
		unsigned char wav, unsigned char loop)
{
	unsigned char tmp = 0;

	if (wav%2) {
		tmp = loop<<0;
		aw8624_i2c_write_bits(aw8624, AW8624_REG_WAVLOOP1+(wav/2),
				AW8624_BIT_WAVLOOP_SEQNP1_MASK, tmp);
	} else {
		tmp = loop<<4;
		aw8624_i2c_write_bits(aw8624, AW8624_REG_WAVLOOP1+(wav/2),
				AW8624_BIT_WAVLOOP_SEQN_MASK, tmp);
	}

	return 0;
}

static int aw8624_haptic_play_wav_seq(struct aw8624 *aw8624, unsigned char flag) //
{

	if (flag) {
		aw8624_haptic_play_mode(aw8624, AW8624_HAPTIC_RAM_MODE);
		aw8624_haptic_start(aw8624);
	}
	return 0;
}

static int aw8624_haptic_play_repeat_seq(struct aw8624 *aw8624, unsigned char flag) //
{

	if (flag) {
		aw8624_haptic_play_mode(aw8624, AW8624_HAPTIC_RAM_LOOP_MODE);
		aw8624_haptic_start(aw8624);
	}

	return 0;
}

static int  aw8624_lra_information_ctr(struct aw8624 *aw8624)
{

	switch (aw8624->lra_information) {
	case AW8624_LRA_0832:
		dev_info(aw8624->dev, "%s enter AW8624_LRA_0832\n", __func__);
		aw8624->lra_info.AW8624_HAPTIC_F0_PRE = 2350;
		aw8624->lra_info.AW8624_HAPTIC_F0_CALI_PERCEN = 7;
		aw8624->lra_info.AW8624_HAPTIC_CONT_DRV_LVL = 106;
		aw8624->lra_info.AW8624_HAPTIC_CONT_DRV_LVL_OV = 106;
		aw8624->lra_info.AW8624_HAPTIC_CONT_TD = 0xF06c;
		aw8624->lra_info.AW8624_HAPTIC_CONT_ZC_THR = 0x08F8;
		aw8624->lra_info.AW8624_HAPTIC_CONT_NUM_BRK = 3;
		break;
	case AW8624_LRA_0815:
		dev_info(aw8624->dev, "%s enter AW8624_LRA_0815\n", __func__);
		aw8624->lra_info.AW8624_HAPTIC_F0_PRE = 1700;   // 170Hz
		aw8624->lra_info.AW8624_HAPTIC_F0_CALI_PERCEN = 7;       // -7%~7%
		aw8624->lra_info.AW8624_HAPTIC_CONT_DRV_LVL = 53;   // 71*6.1/256=1.69v
		aw8624->lra_info.AW8624_HAPTIC_CONT_DRV_LVL_OV = 125;    // 125*6.1/256=2.98v
		aw8624->lra_info.AW8624_HAPTIC_CONT_TD = 0x009a;
		aw8624->lra_info.AW8624_HAPTIC_CONT_ZC_THR = 0x0ff1;
		aw8624->lra_info.AW8624_HAPTIC_CONT_NUM_BRK = 3;
		aw8624->lra_info.AW8624_HAPTIC_RATED_VOLTAGE = 1270; //mv-Vp
		break;
	default:
		dev_info(aw8624->dev, "%s enter AW8624_LRA_DEFAULT\n", __func__);
		aw8624->lra_info.AW8624_HAPTIC_F0_PRE = 1700;   // 170Hz
		aw8624->lra_info.AW8624_HAPTIC_F0_CALI_PERCEN = 7;       // -7%~7%
		aw8624->lra_info.AW8624_HAPTIC_CONT_DRV_LVL = 71;   // 71*6.1/256=1.69v
		aw8624->lra_info.AW8624_HAPTIC_CONT_DRV_LVL_OV = 125;    // 125*6.1/256=2.98v
		aw8624->lra_info.AW8624_HAPTIC_CONT_TD = 0x009a;
		aw8624->lra_info.AW8624_HAPTIC_CONT_ZC_THR = 0x0ff1;
		aw8624->lra_info.AW8624_HAPTIC_CONT_NUM_BRK = 3;
		aw8624->lra_info.AW8624_HAPTIC_RATED_VOLTAGE = 1700; //mv-Vp
		break;
	}
	dev_dbg(aw8624->dev, "%s aw8624->lra_information = %d \n", __func__, aw8624->lra_information);
	dev_dbg(aw8624->dev, "%s AW8624_HAPTIC_F0_PRE = %d \n", __func__, aw8624->lra_info.AW8624_HAPTIC_F0_PRE);
	dev_dbg(aw8624->dev, "%s AW8624_HAPTIC_F0_CALI_PERCEN = %d\n", __func__, aw8624->lra_info.AW8624_HAPTIC_F0_CALI_PERCEN);
	dev_dbg(aw8624->dev, "%s AW8624_HAPTIC_CONT_DRV_LVL = %d\n", __func__, aw8624->lra_info.AW8624_HAPTIC_CONT_DRV_LVL);
	dev_dbg(aw8624->dev, "%s AW8624_HAPTIC_CONT_DRV_LVL_OV = %d\n", __func__, aw8624->lra_info.AW8624_HAPTIC_CONT_DRV_LVL_OV);
	dev_dbg(aw8624->dev, "%s AW8624_HAPTIC_CONT_TD = %d\n", __func__, aw8624->lra_info.AW8624_HAPTIC_CONT_TD);
	dev_dbg(aw8624->dev, "%s AW8624_HAPTIC_CONT_ZC_THR = %d\n", __func__, aw8624->lra_info.AW8624_HAPTIC_CONT_ZC_THR);
	dev_dbg(aw8624->dev, "%s AW8624_HAPTIC_CONT_NUM_BRK = %d;\n", __func__, aw8624->lra_info.AW8624_HAPTIC_CONT_NUM_BRK);
	dev_dbg(aw8624->dev, "%s AW8624_HAPTIC_RATED_VOLTAGE = %d\n", __func__, aw8624->lra_info.AW8624_HAPTIC_RATED_VOLTAGE);
	return 0;
}

// static int aw8624_set_clock(struct aw8624 *aw8624, int clock_type)
// {
	// unsigned char code;

	// if (clock_type == AW8624_HAPTIC_CLOCK_CALI_F0) {
		// code = (unsigned char)atomic_read(&aw8624->f0_freq_cali);
		// dev_info(aw8624->dev, "%s cali f0 below 0, vlaue=%d, code=%#x\n", __func__, atomic_read(&aw8624->f0_freq_cali), code);
		// aw8624_i2c_write(aw8624, AW8624_REG_TRIM_LRA, code);

	// } else if (clock_type == AW8624_HAPTIC_CLOCK_CALI_OSC_STANDARD) {
		// code = (unsigned char)atomic_read(&aw8624->standard_osc_freq_cali);
		// dev_info(aw8624->dev, "%s cali f0 below 0, value=%d, code=%#x\n", __func__, atomic_read(&aw8624->standard_osc_freq_cali), code);
		// aw8624_i2c_write(aw8624, AW8624_REG_TRIM_LRA, code);
	// } else {
		// dev_info(aw8624->dev, "%s no f0 offset set\n", __func__);
	// }
	// return 0;
// }

static int aw8624_haptic_set_repeat_wav_seq(struct aw8624 *aw8624, unsigned char seq) //
{
	dev_dbg(aw8624->dev, "%s seq = %u\n.", __func__, seq);

	aw8624_haptic_set_wav_seq(aw8624, 0x00, seq);
	aw8624_haptic_set_wav_loop(aw8624, 0x00, AW8624_BIT_WAVLOOP_INIFINITELY);

	return 0;
}

static int aw8624_haptic_set_f0_preset(struct aw8624 *aw8624)
{
	unsigned int f0_reg = 0;

	f0_reg = 1000000000 / (aw8624->f0_pre * AW8624_HAPTIC_F0_COEFF);
	aw8624_i2c_write(aw8624, AW8624_REG_F_PRE_H, (unsigned char)((f0_reg>>8)&0xff));
	aw8624_i2c_write(aw8624, AW8624_REG_F_PRE_L, (unsigned char)((f0_reg>>0)&0xff));

	return 0;
}

static int aw8624_haptic_cont(struct aw8624 *aw8624)
{


	unsigned char brake0_level = 0;
	unsigned char en_brake1 = 0;
	unsigned char brake1_level = 0;
	unsigned char en_brake2 = 0;
	unsigned char brake2_level = 0;
	unsigned char brake2_p_num = 0;
	unsigned char brake1_p_num = 0;
	unsigned char brake0_p_num = 0;

	dev_info(aw8624->dev, "%s enter\n.", __func__);
	/* work mode */
	aw8624_haptic_active(aw8624);
	aw8624_haptic_play_mode(aw8624, AW8624_HAPTIC_CONT_MODE);

	/* preset f0 */
	aw8624->f0_pre = aw8624->f0;
	aw8624_haptic_set_f0_preset(aw8624);

	/* lpf */
	aw8624_i2c_write_bits(aw8624,
				AW8624_REG_DATCTRL,
				AW8624_BIT_DATCTRL_FC_MASK,
				AW8624_BIT_DATCTRL_FC_1000HZ);
	aw8624_i2c_write_bits(aw8624,
				AW8624_REG_DATCTRL,
				AW8624_BIT_DATCTRL_LPF_ENABLE_MASK,
				AW8624_BIT_DATCTRL_LPF_ENABLE);

	/* brake */
	en_brake1 = 0;
	en_brake2 = 0;
	brake0_level = 90;
	brake1_level = 42;
	brake2_level = 20;
	brake0_p_num = 5;
	brake1_p_num = 2;
	brake2_p_num = 2;

	aw8624_i2c_write(aw8624,
			AW8624_REG_BRAKE0_CTRL,
			(brake0_level << 0));
	aw8624_i2c_write(aw8624,
			AW8624_REG_BRAKE1_CTRL,
			(en_brake1 << 7)|(brake1_level << 0));
	aw8624_i2c_write(aw8624,
			AW8624_REG_BRAKE2_CTRL,
			(en_brake2 << 7)|(brake2_level << 0));
	aw8624_i2c_write(aw8624,
			AW8624_REG_BRAKE_NUM,
			((brake2_p_num << 6)|(brake1_p_num << 3) |
			(brake0_p_num << 0)));

	/* cont config */
	aw8624_i2c_write_bits(aw8624,
				AW8624_REG_CONT_CTRL,
				AW8624_BIT_CONT_CTRL_ZC_DETEC_MASK,
				AW8624_BIT_CONT_CTRL_ZC_DETEC_ENABLE);
	aw8624_i2c_write_bits(aw8624,
				AW8624_REG_CONT_CTRL,
				AW8624_BIT_CONT_CTRL_WAIT_PERIOD_MASK,
				AW8624_BIT_CONT_CTRL_WAIT_1PERIOD);

	aw8624_i2c_write_bits(aw8624,
				AW8624_REG_CONT_CTRL,
				AW8624_BIT_CONT_CTRL_MODE_MASK,
				AW8624_BIT_CONT_CTRL_BY_GO_SIGNAL);

	aw8624_i2c_write_bits(aw8624,
				AW8624_REG_CONT_CTRL,
				AW8624_BIT_CONT_CTRL_EN_CLOSE_MASK,
				AW8624_BIT_CONT_CTRL_CLOSE_PLAYBACK);
	aw8624_i2c_write_bits(aw8624,
				AW8624_REG_CONT_CTRL,
				AW8624_BIT_CONT_CTRL_F0_DETECT_MASK,
				AW8624_BIT_CONT_CTRL_F0_DETECT_DISABLE);
	aw8624_i2c_write_bits(aw8624,
				AW8624_REG_CONT_CTRL,
				AW8624_BIT_CONT_CTRL_O2C_MASK,
				AW8624_BIT_CONT_CTRL_O2C_DISABLE);

	/* TD time */
	aw8624_i2c_write(aw8624,
			AW8624_REG_TD_H,
			(unsigned char)(aw8624->cont_td>>8));
	aw8624_i2c_write(aw8624,
			AW8624_REG_TD_L,
			(unsigned char)(aw8624->cont_td>>0));


	aw8624_i2c_write_bits(aw8624,
			AW8624_REG_BEMF_NUM,
			AW8624_BIT_BEMF_NUM_BRK_MASK,
			aw8624->cont_num_brk);
	aw8624_i2c_write(aw8624,
			AW8624_REG_TIME_NZC,
			0x1f);

	/* f0 driver level */
	aw8624_i2c_write(aw8624,
			AW8624_REG_DRV_LVL,
			aw8624->cont_drv_lvl);
	aw8624_i2c_write(aw8624,
			AW8624_REG_DRV_LVL_OV,
			aw8624->cont_drv_lvl_ov);

	/* cont play go */
	aw8624_haptic_play_go(aw8624, true);

	return 0;
}

static void aw8624_haptic_upload_lra(struct aw8624 *aw8624, unsigned int flag) //
{

	switch (flag) {
	case AW8624_HAPTIC_F0_CALI_LRA:
		dev_info(aw8624->dev, "%s f0_cali_lra=%d\n", __func__, aw8624->f0_cali_data);
		aw8624_i2c_write(aw8624, AW8624_REG_TRIM_LRA, (unsigned char)aw8624->f0_cali_data);
		break;
	case AW8624_HAPTIC_RTP_CALI_LRA:
		dev_info(aw8624->dev, "%s rtp_cali_lra=%d\n", __func__, aw8624->osc_cali_data);
		aw8624_i2c_write(aw8624, AW8624_REG_TRIM_LRA, (unsigned char)aw8624->osc_cali_data);
		break;
	case AW8624_HAPTIC_ZERO:
		dev_info(aw8624->dev, "%s write zero to trim_lra!\n", __func__);
		aw8624_i2c_write(aw8624, AW8624_REG_TRIM_LRA, 0);
		break;
	default:
		break;
	}
}


static void aw8624_haptic_set_rtp_aei(struct aw8624 *aw8624, bool flag) //
{

	dev_info(aw8624->dev, "%s: set empty irq, flag (%d)\n", __func__, flag);
	if (flag) {
		aw8624_i2c_write_bits(aw8624, AW8624_REG_SYSINTM,
				AW8624_BIT_SYSINTM_FF_AE_MASK, AW8624_BIT_SYSINTM_FF_AE_EN);
	} else {
		aw8624_i2c_write_bits(aw8624, AW8624_REG_SYSINTM,
				AW8624_BIT_SYSINTM_FF_AE_MASK, AW8624_BIT_SYSINTM_FF_AE_OFF);
	}
}

/**
 *
 * 通过SYSST状态寄存器的fifo满bit位来判断fifo是否满，代替通过SYSINT的fifo满bit位
 * 防止读SYSINT寄存器判断fifo是否满的时候，同时将fifo空的bit位清除掉了
 *
 */
static int aw8624_haptic_rtp_get_fifo_afs(struct aw8624 *aw8624) //
{
	unsigned char ret = 0;
	unsigned char reg_val = 0;
	int rc = 0;

	rc = aw8624_i2c_read(aw8624, AW8624_REG_SYSST, &reg_val);
	if (rc < 0) {
		dev_err(aw8624->dev, "%s failed, ret=%d, reg_val=%#x", __func__, ret, reg_val);
		return -EBUSY;
	}

	reg_val &= AW8624_BIT_SYSST_FF_AFS;
	ret = reg_val >> 3;

	return ret; //0x01表示fifo满，0x00表示fifo空，负数表示i2c读取错误
}

static int aw8624_haptic_set_pwm(struct aw8624 *aw8624, unsigned char mode) //
{

	switch (mode) {
	case AW8624_PWM_48K:
		aw8624_i2c_write_bits(aw8624,
				AW8624_REG_PWMDBG,
				AW8624_BIT_PWMDBG_PWM_MODE_MASK,
				AW8624_BIT_PWMDBG_PWM_48K);
		break;
	case AW8624_PWM_24K:
		aw8624_i2c_write_bits(aw8624,
				AW8624_REG_PWMDBG,
				AW8624_BIT_PWMDBG_PWM_MODE_MASK,
				AW8624_BIT_PWMDBG_PWM_24K);
		break;
	case AW8624_PWM_12K:
		aw8624_i2c_write_bits(aw8624,
				AW8624_REG_PWMDBG,
				AW8624_BIT_PWMDBG_PWM_MODE_MASK,
				AW8624_BIT_PWMDBG_PWM_12K);
		break;
	default:
		break;
	}
	return 0;
}

static int aw8624_haptic_swicth_motorprotect_config(struct aw8624 *aw8624, unsigned char addr, unsigned char val) //
{

	if (addr == 1) {
		aw8624_i2c_write_bits(aw8624,
				AW8624_REG_DETCTRL,
				AW8624_BIT_DETCTRL_PROTECT_MASK,
				AW8624_BIT_DETCTRL_PROTECT_SHUTDOWN);
		aw8624_i2c_write_bits(aw8624,
				AW8624_REG_PWMPRC,
				AW8624_BIT_PWMPRC_PRC_EN_MASK,
				AW8624_BIT_PWMPRC_PRC_ENABLE);
		aw8624_i2c_write_bits(aw8624,
				AW8624_REG_PRLVL,
				AW8624_BIT_PRLVL_PR_EN_MASK,
				AW8624_BIT_PRLVL_PR_ENABLE);
	} else if (addr == 0) {
		aw8624_i2c_write_bits(aw8624,
				AW8624_REG_DETCTRL,
				AW8624_BIT_DETCTRL_PROTECT_MASK,
				AW8624_BIT_DETCTRL_PROTECT_NO_ACTION);
		aw8624_i2c_write_bits(aw8624,
				AW8624_REG_PWMPRC,
				AW8624_BIT_PWMPRC_PRC_EN_MASK,
				AW8624_BIT_PWMPRC_PRC_DISABLE);
		aw8624_i2c_write_bits(aw8624,
				AW8624_REG_PRLVL,
				AW8624_BIT_PRLVL_PR_EN_MASK,
				AW8624_BIT_PRLVL_PR_DISABLE);
	} else if (addr == 0x2d) {
		aw8624_i2c_write_bits(aw8624, AW8624_REG_PWMPRC,
			AW8624_BIT_PWMPRC_PRCTIME_MASK, val);
	} else if (addr == 0x3e) {
		aw8624_i2c_write_bits(aw8624, AW8624_REG_PRLVL,
			AW8624_BIT_PRLVL_PRLVL_MASK, val);
	} else if (addr == 0x3f) {
		aw8624_i2c_write_bits(aw8624, AW8624_REG_PRTIME,
			AW8624_BIT_PRTIME_PRTIME_MASK, val);
	} else {
		 /*nothing to do;*/
	}

	 return 0;
}

static int aw8624_haptic_vbat_mode(struct aw8624 *aw8624, unsigned char flag) //
{

	if (flag == AW8624_HAPTIC_VBAT_HW_COMP_MODE) {
		aw8624_i2c_write_bits(aw8624,
				AW8624_REG_ADCTEST,
				AW8624_BIT_DETCTRL_VBAT_MODE_MASK,
				AW8624_BIT_DETCTRL_VBAT_HW_COMP);
	} else {
		aw8624_i2c_write_bits(aw8624,
				AW8624_REG_ADCTEST,
				AW8624_BIT_DETCTRL_VBAT_MODE_MASK,
				AW8624_BIT_DETCTRL_VBAT_SW_COMP);
	}
	return 0;
}

/*********************************************
* usleep_range void __sched usleep_range(unsigned long min, unsigned long max)
* 功能：用于非原子环境的睡眠，目前内核建议用这个函数替换之前udelay
*********************************************/
static int aw8624_vbat_monitor_detector(struct aw8624 *aw8624)
{
	unsigned char reg_val = 0;
	unsigned char reg_val_sysctrl = 0;
	unsigned char reg_val_detctrl = 0;
	unsigned int vbat = 0;


	aw8624_haptic_stop(aw8624);
	aw8624_i2c_read(aw8624, AW8624_REG_SYSCTRL, &reg_val_sysctrl);
	aw8624_i2c_read(aw8624, AW8624_REG_DETCTRL, &reg_val_detctrl);
	/*step 1:EN_RAMINIT*/
	aw8624_i2c_write_bits(aw8624,
				AW8624_REG_SYSCTRL,
				AW8624_BIT_SYSCTRL_RAMINIT_MASK,
				AW8624_BIT_SYSCTRL_RAMINIT_EN);

	/*step 2 :launch offset cali */
	aw8624_i2c_write_bits(aw8624,
				AW8624_REG_DETCTRL,
				AW8624_BIT_DETCTRL_DIAG_GO_MASK,
				AW8624_BIT_DETCTRL_DIAG_GO_ENABLE);
	/*step 3 :delay */
	usleep_range(2000, 2500);

	/*step 4 :launch power supply testing */
	aw8624_i2c_write_bits(aw8624,
				AW8624_REG_DETCTRL,
				AW8624_BIT_DETCTRL_VBAT_GO_MASK,
				AW8624_BIT_DETCTRL_VBAT_GO_ENABLE);
	usleep_range(2000, 2500);

	aw8624_i2c_read(aw8624, AW8624_REG_VBATDET, &reg_val);
	vbat = 6100 * reg_val / 256;

	/*step 5: return val*/
	aw8624_i2c_write(aw8624, AW8624_REG_SYSCTRL, reg_val_sysctrl);

	return vbat;
}

static int aw8624_haptic_ram_vbat_comp(struct aw8624 *aw8624, bool flag) //
{
	int temp_gain = 0;
	int vbat = 0;


	if (flag) {
		if (aw8624->ram_vbat_comp == AW8624_HAPTIC_RAM_VBAT_COMP_ENABLE) {
			vbat = aw8624_vbat_monitor_detector(aw8624);
			temp_gain = aw8624->gain * AW8624_VBAT_REFER / vbat;
			if (temp_gain > (128 * AW8624_VBAT_REFER / AW8624_VBAT_MIN)) {
				temp_gain = 128*AW8624_VBAT_REFER/AW8624_VBAT_MIN;
				dev_info(aw8624->dev, "%s gain limit=%d\n", __func__, temp_gain);
			}
			dev_info(aw8624->dev, "%s temp_gain=%d\n.", __func__, temp_gain);
			aw8624_haptic_set_gain(aw8624, temp_gain);
		} else {
			aw8624_haptic_set_gain(aw8624, aw8624->gain);
		}
	} else {
		aw8624_haptic_set_gain(aw8624, aw8624->gain);
	}

	return 0;
}


/*****************************************************
 *
 * haptic f0 cali and beme get
 *
 *****************************************************/

static void aw8624_lra_resist_get(struct aw8624 *aw8624, unsigned char *reg_val)//根据aw8624修改了这个阻抗的获得函数
{
	unsigned char reg_val_sysctrl = 0;
	unsigned char reg_val_anactrl = 0;
	unsigned char reg_val_d2scfg = 0;

	aw8624_i2c_read(aw8624, AW8624_REG_SYSCTRL, &reg_val_sysctrl);
	aw8624_i2c_read(aw8624, AW8624_REG_ANACTRL, &reg_val_anactrl);
	aw8624_i2c_read(aw8624, AW8624_REG_D2SCFG, &reg_val_d2scfg);
	aw8624_haptic_stop(aw8624);
	aw8624_i2c_write_bits(aw8624,
				AW8624_REG_SYSCTRL,
				AW8624_BIT_SYSCTRL_RAMINIT_MASK,
				AW8624_BIT_SYSCTRL_RAMINIT_EN);


	aw8624_i2c_write_bits(aw8624,
				AW8624_REG_ANACTRL,
				AW8624_BIT_ANACTRL_EN_IO_PD1_MASK,
				AW8624_BIT_ANACTRL_EN_IO_PD1_HIGH);

	aw8624_i2c_write_bits(aw8624,
				AW8624_REG_D2SCFG,
				AW8624_BIT_D2SCFG_CLK_ADC_MASK,
				AW8624_BIT_D2SCFG_CLK_ASC_1P5MHZ);

	aw8624_i2c_write_bits(aw8624,
				AW8624_REG_DETCTRL,
				AW8624_BIT_DETCTRL_RL_OS_MASK,
				AW8624_BIT_DETCTRL_RL_DETECT);
	aw8624_i2c_write_bits(aw8624,
				AW8624_REG_DETCTRL,
				AW8624_BIT_DETCTRL_DIAG_GO_MASK,
				AW8624_BIT_DETCTRL_DIAG_GO_ENABLE);
	usleep_range(3000, 3500);
	aw8624_i2c_read(aw8624, AW8624_REG_RLDET, reg_val);


	aw8624->lra = 298 * (*reg_val);
	dev_info(aw8624->dev, "resist get : %d\n", aw8624->lra);
	aw8624_i2c_write(aw8624, AW8624_REG_D2SCFG, reg_val_d2scfg);
	aw8624_i2c_write(aw8624, AW8624_REG_ANACTRL, reg_val_anactrl);
	aw8624_i2c_write(aw8624, AW8624_REG_SYSCTRL, reg_val_sysctrl);

}

static void aw8624_stop_work_routine(struct work_struct *work) //
{
	struct aw8624 *aw8624 = container_of(work, struct aw8624, stop_work.work);

	aw8624_i2c_write_bits(aw8624,
				AW8624_REG_CONT_CTRL,
				AW8624_BIT_CONT_CTRL_EN_CLOSE_MASK,
				AW8624_BIT_CONT_CTRL_CLOSE_PLAYBACK);
	aw8624_i2c_write_bits(aw8624,
				AW8624_REG_CONT_CTRL,
				AW8624_BIT_CONT_CTRL_F0_DETECT_MASK,
				AW8624_BIT_CONT_CTRL_F0_DETECT_DISABLE);
}


static int aw8624_haptic_read_f0(struct aw8624 *aw8624)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_reg = 0;
	unsigned long f0_tmp = 0;

	ret = aw8624_i2c_read(aw8624, AW8624_REG_F_LRA_F0_H, &reg_val);
	f0_reg = (reg_val<<8);
	ret = aw8624_i2c_read(aw8624, AW8624_REG_F_LRA_F0_L, &reg_val);
	f0_reg |= (reg_val<<0);
	if (!f0_reg) {
		aw8624->f0 = 0;
		dev_info(aw8624->dev, "%s : get f0 failed with the value becoming 0!\n", __func__);
		return 0;
	}

	f0_tmp = 1000000000 / (f0_reg * 260);
	aw8624->f0 = (unsigned int)f0_tmp;
	dev_info(aw8624->dev, "%s f0=%d\n", __func__, aw8624->f0);

	return 0;
}

static int aw8624_haptic_read_beme(struct aw8624 *aw8624) //
{
	int ret = 0;
	unsigned char reg_val = 0;

	ret = aw8624_i2c_read(aw8624, AW8624_REG_WAIT_VOL_MP, &reg_val);
	aw8624->max_pos_beme = (reg_val<<0);
	ret = aw8624_i2c_read(aw8624, AW8624_REG_WAIT_VOL_MN, &reg_val);
	aw8624->max_neg_beme = (reg_val<<0);

	dev_info(aw8624->dev, "%s max_pos_beme=%d\n", __func__, aw8624->max_pos_beme);
	dev_info(aw8624->dev, "%s max_neg_beme=%d\n", __func__, aw8624->max_neg_beme);

	return 0;
}

static int aw8624_haptic_get_f0(struct aw8624 *aw8624)
{
	int ret = 0;
	int timer_val = 0;
	unsigned char reg_val = 0;
	unsigned char f0_pre_num = 0;
	unsigned char f0_wait_num = 0;
	int f0_repeat_num = 0;
	unsigned char f0_trace_num = 0;
	unsigned int t_f0_ms = 0;
	unsigned int t_f0_trace_ms = 0;
	unsigned char i = 0;
	unsigned int f0_cali_cnt = 50;


	aw8624->f0 = aw8624->f0_pre;
	/* f0 calibrate work mode */
	aw8624_haptic_stop(aw8624);
	aw8624_i2c_write(aw8624, AW8624_REG_TRIM_LRA, 0x00);
	aw8624_haptic_play_mode(aw8624, AW8624_HAPTIC_CONT_MODE);


	aw8624_i2c_write_bits(aw8624,
			AW8624_REG_CONT_CTRL,
			AW8624_BIT_CONT_CTRL_EN_CLOSE_MASK,
			AW8624_BIT_CONT_CTRL_OPEN_PLAYBACK);
	aw8624_i2c_write_bits(aw8624,
			AW8624_REG_CONT_CTRL,
			AW8624_BIT_CONT_CTRL_F0_DETECT_MASK,
			AW8624_BIT_CONT_CTRL_F0_DETECT_ENABLE);

	/* LPF */
	aw8624_i2c_write_bits(aw8624,
			AW8624_REG_DATCTRL,
			AW8624_BIT_DATCTRL_FC_MASK,
			AW8624_BIT_DATCTRL_FC_1000HZ);
	aw8624_i2c_write_bits(aw8624,
			AW8624_REG_DATCTRL,
			AW8624_BIT_DATCTRL_LPF_ENABLE_MASK,
			AW8624_BIT_DATCTRL_LPF_ENABLE);

	/* LRA OSC Source */
	if (aw8624->f0_cali_flag == AW8624_HAPTIC_CALI_F0) {
		aw8624_i2c_write_bits(aw8624,
			AW8624_REG_ANACTRL,
			AW8624_BIT_ANACTRL_LRA_SRC_MASK,
			AW8624_BIT_ANACTRL_LRA_SRC_REG);
	} else {
		aw8624_i2c_write_bits(aw8624,
			AW8624_REG_ANACTRL,
			AW8624_BIT_ANACTRL_LRA_SRC_MASK,
			AW8624_BIT_ANACTRL_LRA_SRC_EFUSE);
	}

	/* preset f0 */
	aw8624_haptic_set_f0_preset(aw8624);
	/* f0 driver level */
	aw8624_i2c_write(aw8624, AW8624_REG_DRV_LVL, aw8624->cont_drv_lvl);
	/* f0 trace parameter */
	if (!aw8624->f0_pre) {
		dev_err(aw8624->dev, "%s:fail to get t_f0_ms\n", __func__);
		return 0;
	}

	t_f0_ms = (1000*10 / 2) / aw8624->f0_pre;
	/* f0 trace parameter */
	f0_pre_num = 0x05;
	f0_wait_num = 0x03;
	f0_repeat_num = 0x01;
	f0_trace_num = 0x0f;


	dev_info(aw8624->dev, "%s:first f0_repeat_num  = %d\n", __func__, f0_repeat_num);
	if (f0_repeat_num <= 0) {
		f0_repeat_num = 1;
	} else {
		if (f0_repeat_num % 100 >= 50)
			f0_repeat_num = f0_repeat_num / 100 + 1;
		else
			f0_repeat_num = f0_repeat_num / 100;
	}
	dev_info(aw8624->dev, "%s:finally f0_repeat_num  = %d\n", __func__, f0_repeat_num);
	aw8624_i2c_write(aw8624,
			AW8624_REG_NUM_F0_1,
			(f0_pre_num<<4)|(f0_wait_num<<0));
	aw8624_i2c_write(aw8624,
			AW8624_REG_NUM_F0_2,
			(char)(f0_repeat_num<<0));
	aw8624_i2c_write(aw8624,
			AW8624_REG_NUM_F0_3,
			(f0_trace_num<<0));

	/* clear aw8624 interrupt */
	ret = aw8624_i2c_read(aw8624, AW8624_REG_SYSINT, &reg_val);

	/* play go and start f0 calibration */
	aw8624_haptic_play_go(aw8624, true);

	/* f0 trace time */
	t_f0_trace_ms = t_f0_ms *
	(f0_pre_num + f0_wait_num + 2 + (f0_trace_num+f0_wait_num + 2)) + 50;
	mdelay(t_f0_trace_ms + 10);

	for (i = 0; i < f0_cali_cnt; i++) {
		ret = aw8624_i2c_read(aw8624, AW8624_REG_GLB_STATE, &reg_val);
		/* f0 calibrate done */
		if ((reg_val & 0x0f) == 0x00) {
			aw8624_haptic_read_f0(aw8624);
			aw8624_haptic_read_beme(aw8624);
			break;
		}
		usleep_range(10000, 10500);
		dev_info(aw8624->dev, "%s f0 cali sleep 10ms,glb_state=0x%x\n",
							__func__, reg_val);
	}

	if (i == f0_cali_cnt)
		ret = -1;
	else
		ret = 0;

	timer_val = 1;//原生代码的设备树是这个值
	INIT_DELAYED_WORK(&aw8624->stop_work, aw8624_stop_work_routine);
	schedule_delayed_work(&aw8624->stop_work, msecs_to_jiffies(timer_val));

	return ret;
}

static int aw8624_haptic_f0_calibration(struct aw8624 *aw8624)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_limit = 0;
	char f0_cali_lra = 0;
	int f0_cali_step = 0;
	/*int f0_dft_step = 0;*/

	if (aw8624_haptic_get_f0(aw8624)) {
		dev_err(aw8624->dev, "%s get f0 error, user defafult f0\n", __func__);
	} else {
		 /* max and min limit */
		f0_limit = aw8624->f0;
		if (aw8624->f0*100 < 2350 * (100-7)) {
			f0_limit = 2350;
		}
		if (aw8624->f0*100 > 2350 *
		(100+7)) {
			f0_limit = 2350;
		}
		/* calculate cali step */
		f0_cali_step =
		100000*((int)f0_limit-(int)aw8624->f0_pre)/((int)f0_limit*25);

		if (f0_cali_step >= 0) {  /*f0_cali_step >= 0*/
			if (f0_cali_step % 10 >= 5) {
				f0_cali_step = f0_cali_step/10 + 1 +
					(aw8624->chipid_flag == 1 ? 32 : 16);
			} else {
				f0_cali_step = f0_cali_step/10 +
					(aw8624->chipid_flag == 1 ? 32 : 16);
			}
		} else { /*f0_cali_step < 0*/
			if (f0_cali_step % 10 <= -5) {
				f0_cali_step =
					(aw8624->chipid_flag == 1 ? 32 : 16) +
					(f0_cali_step/10 - 1);
			} else {
				f0_cali_step =
					(aw8624->chipid_flag == 1 ? 32 : 16) +
					f0_cali_step/10;
			}
		}

		if (aw8624->chipid_flag == 1) {
			if (f0_cali_step > 31)
				f0_cali_lra = (char)f0_cali_step - 32;
			else
				f0_cali_lra = (char)f0_cali_step + 32;
		} else {
			if (f0_cali_step < 16 ||
			(f0_cali_step > 31 && f0_cali_step < 48)) {
				f0_cali_lra = (char)f0_cali_step + 16;
			} else {
				f0_cali_lra = (char)f0_cali_step - 16;
			}
		}

		aw8624->f0_cali_data = (int)f0_cali_lra;
		/* update cali step */
		aw8624_haptic_upload_lra(aw8624,
					AW8624_HAPTIC_F0_CALI_LRA);
		aw8624_i2c_read(aw8624,
				AW8624_REG_TRIM_LRA,
				&reg_val);
		dev_info(aw8624->dev, "%s final trim_lra=0x%02x\n", __func__, reg_val);
	}

	/* if (aw8624_haptic_get_f0(aw8624)) { */
	/* dev_err(aw8624->dev,*/
	/*	"%s get f0 error, user defafult f0\n", __func__); */
	/* } */

	aw8624_haptic_play_mode(aw8624, AW8624_HAPTIC_STANDBY_MODE);
	aw8624_haptic_stop(aw8624);

	return ret;
}


static void aw8624_vibrator_work_routine(struct work_struct *work)
{
	struct aw8624 *aw8624 = container_of(work, struct aw8624, vibrator_work);

	dev_info(aw8624->dev, "%s enter\n", __func__);

	mutex_lock(&aw8624->lock);
	/* Enter standby mode */
	aw8624_haptic_stop(aw8624);
	aw8624_haptic_upload_lra(aw8624, AW8624_HAPTIC_F0_CALI_LRA);
	if (aw8624->state) {
		if (aw8624->activate_mode == AW8624_HAPTIC_ACTIVATE_RAM_MODE) {
			aw8624_haptic_ram_vbat_comp(aw8624, false);
			aw8624_haptic_play_mode(aw8624, AW8624_HAPTIC_RAM_MODE);
			aw8624_haptic_play_go(aw8624, true);
		} else if (aw8624->activate_mode == AW8624_HAPTIC_ACTIVATE_RAM_LOOP_MODE) {
			aw8624_haptic_ram_vbat_comp(aw8624, true);
			aw8624_haptic_play_repeat_seq(aw8624, true);
			hrtimer_start(&aw8624->timer, ktime_set(aw8624->duration / 1000,
						(aw8624->duration % 1000) * 1000000), HRTIMER_MODE_REL);
		} else if (aw8624->activate_mode == AW8624_HAPTIC_ACTIVATE_CONT_MODE) {
			aw8624_haptic_cont(aw8624);
			/* run ms timer */
			hrtimer_start(&aw8624->timer, ktime_set(aw8624->duration / 1000,
						(aw8624->duration % 1000) * 1000000), HRTIMER_MODE_REL);
		} else {
			dev_err(aw8624->dev, "%s: activate_mode error\n", __func__);
		}

	}
	mutex_unlock(&aw8624->lock);

	if (lra_wake_lock_active(aw8624->wklock)) {
		lra_wake_unlock(aw8624->wklock);
		dev_info(aw8624->dev, "%s, release wake lock\n", __func__);
	}

}


static enum hrtimer_restart aw8624_vibrator_timer_func(struct hrtimer *timer)
{
	struct aw8624 *aw8624 = container_of(timer, struct aw8624, timer);

	dev_info(aw8624->dev, "%s +++\n", __func__);
	aw8624->state = 0;
	schedule_work(&aw8624->vibrator_work);

	return HRTIMER_NORESTART;
}

static int aw8624_haptic_rtp_init(struct aw8624 *aw8624)
{
	unsigned int buf_len = 0;
	int ret = 0, retval = 0;
	unsigned char glb_st = 0;


	dev_info(aw8624->dev, "%s  enter\n", __func__);

	//pm_qos_add_request(&pm_qos_req_vb, PM_QOS_CPU_DMA_LATENCY, 400);

	mutex_lock(&aw8624->rtp_lock);

	aw8624->rtp_cnt = 0;
	retval = aw8624_haptic_rtp_get_fifo_afs(aw8624);
	if ((!retval) && (aw8624->play_mode == AW8624_HAPTIC_RTP_MODE)) {
		dev_info(aw8624->dev, "%s rtp cnt = %d\n", __func__, aw8624->rtp_cnt);

		if (aw8624_rtp->len > aw8624->rtp_cnt) {
			if ((aw8624_rtp->len - aw8624->rtp_cnt) < (aw8624->ram.base_addr)) {
				buf_len = aw8624_rtp->len-aw8624->rtp_cnt;
			} else {
				buf_len = (aw8624->ram.base_addr);//2k 数据写入
			}

			ret = aw8624_i2c_writes(aw8624, AW8624_REG_RTP_DATA,
						&aw8624_rtp->data[aw8624->rtp_cnt], buf_len);
			if (ret < 0) {
				dev_err(aw8624->dev, "%s: i2c write error: total length: %d, play length: %d\n",
					__func__, aw8624_rtp->len, aw8624->rtp_cnt);
				aw8624->rtp_cnt = 0;
				mutex_unlock(&aw8624->rtp_lock);
				//pm_qos_remove_request(&pm_qos_req_vb);
				return -EBUSY;
			}
			aw8624->rtp_cnt += buf_len;
		}

		if (aw8624->rtp_cnt >= aw8624_rtp->len) {
			dev_info(aw8624->dev, "%s fist frame complete: total length: %d, play length: %d\n", __func__,
				aw8624_rtp->len, aw8624->rtp_cnt);
			aw8624->rtp_cnt = 0;
			mutex_unlock(&aw8624->rtp_lock);
			//pm_qos_remove_request(&pm_qos_req_vb);
			return 0;
		}
	}

	if (retval < 0) {
		dev_info(aw8624->dev, "%s: i2c read error: total length: %d, play length: %d\n",
								__func__, aw8624_rtp->len, aw8624->rtp_cnt);
		aw8624->rtp_cnt = 0;
		mutex_unlock(&aw8624->rtp_lock);
		//pm_qos_remove_request(&pm_qos_req_vb);
		return 0;
	}

	while ((!(retval = aw8624_haptic_rtp_get_fifo_afs(aw8624))) &&
			(aw8624->play_mode == AW8624_HAPTIC_RTP_MODE)) {

		dev_info(aw8624->dev, "%s rtp cnt = %d\n", __func__, aw8624->rtp_cnt);

		if (aw8624_rtp->len > aw8624->rtp_cnt) {
			if ((aw8624_rtp->len-aw8624->rtp_cnt) < (aw8624->ram.base_addr>>2)) {
				buf_len = aw8624_rtp->len-aw8624->rtp_cnt;
			} else {
				buf_len = (aw8624->ram.base_addr>>2);
			}

			ret = aw8624_i2c_writes(aw8624, AW8624_REG_RTP_DATA,
					&aw8624_rtp->data[aw8624->rtp_cnt], buf_len);
			if (ret < 0) {
				dev_err(aw8624->dev, "%s: while i2c write error: total length: %d, play length: %d\n",
							__func__, aw8624_rtp->len, aw8624->rtp_cnt);
				aw8624->rtp_cnt = 0;
				mutex_unlock(&aw8624->rtp_lock);
				//pm_qos_remove_request(&pm_qos_req_vb);
				return 0;
			}

			aw8624->rtp_cnt += buf_len;

		}

		aw8624_i2c_read(aw8624, AW8624_REG_GLB_STATE, &glb_st);

		if (aw8624->rtp_cnt >= aw8624_rtp->len || ((glb_st & 0x0f) == 0x00)) {
			dev_err(aw8624->dev, "%s complete total length: %d, play length: %d\n",
				__func__, aw8624_rtp->len, aw8624->rtp_cnt);
			aw8624->rtp_cnt = 0;
			mutex_unlock(&aw8624->rtp_lock);
			//pm_qos_remove_request(&pm_qos_req_vb);
			return 0;
		}
	}

	if (retval < 0) {
		dev_err(aw8624->dev, "%s: i2c read error--->2: total length: %d, play length: %d\n",
					__func__, aw8624_rtp->len, aw8624->rtp_cnt);
		aw8624->rtp_cnt = 0;
		mutex_unlock(&aw8624->rtp_lock);
		//pm_qos_remove_request(&pm_qos_req_vb);
		return 0;
	}


	if ((aw8624->play_mode == AW8624_HAPTIC_RTP_MODE) && (retval >= 0)) {
		dev_err(aw8624->dev, "%s open rtp irq\n", __func__);
		aw8624_haptic_set_rtp_aei(aw8624, true);
	}

	dev_info(aw8624->dev, "%s exit\n", __func__);

	mutex_unlock(&aw8624->rtp_lock);
	//pm_qos_remove_request(&pm_qos_req_vb);
	return 0;
}


static void aw8624_rtp_work_routine(struct work_struct *work)
{
	const struct firmware *rtp_file;
	int ret = -1;
	struct aw8624 *aw8624 = container_of(work, struct aw8624, rtp_work);
	struct aw8624_play_info *play = &aw8624->play;
	struct haptic_wavefrom_info *effect_list = aw8624->effect_list;
	int gain;

	dev_info(aw8624->dev, "%s +++ \n", __func__);

	if (play->type != RTP_TYPE && play->type != RTP_MMAP_TYPE) {
		dev_err(aw8624->dev, "new not rtp effect coming\n");
		return;
	}

	if (play->type == RTP_TYPE) {
		/* fw loaded */
		ret = request_firmware(&rtp_file, effect_list[aw8624->rtp_file_num].rtp_file_name, aw8624->dev);
		if (ret < 0) {
			dev_err(aw8624->dev, "%s: failed to read [%s]\n", __func__, effect_list[aw8624->rtp_file_num]);
			return ;
		}
		mutex_lock(&aw8624->rtp_lock);
		aw8624->rtp_init = 0;
		aw8624_haptic_set_rtp_aei(aw8624, false);

		if (aw8624_rtp == NULL) {
			aw8624_rtp = devm_kzalloc(aw8624->dev, RTP_BIN_MAX_SIZE + sizeof(int), GFP_KERNEL);
			if (aw8624_rtp == NULL) {
				dev_err(aw8624->dev, "%s devm kzalloc failed\n", __func__);
				release_firmware(rtp_file);
				mutex_unlock(&aw8624->rtp_lock);
				return;
			}
		}
		memset(aw8624_rtp, 0, RTP_BIN_MAX_SIZE + sizeof(int));

		dev_info(aw8624->dev, "%s: rtp file [%s] size = %d\n", __func__,
				effect_list[aw8624->rtp_file_num].rtp_file_name, rtp_file->size);

		if (rtp_file->size < RTP_BIN_MAX_SIZE)
			aw8624_rtp->len = rtp_file->size;
		else
			aw8624_rtp->len = RTP_BIN_MAX_SIZE;
		memcpy(aw8624_rtp->data, rtp_file->data, aw8624_rtp->len);
		release_firmware(rtp_file);

		aw8624->rtp_init = 1;
		aw8624->rtp_cnt = 0;
		aw8624_i2c_write_bits(aw8624,
				AW8624_REG_DBGCTRL,
				AW8624_BIT_DBGCTRL_INTN_TRG_SEL_MASK,
				AW8624_BIT_DBGCTRL_INTN_SEL_ENABLE); //选择trig/intn引脚的功能,使能中断
		mutex_unlock(&aw8624->rtp_lock);

	} else {
		if (!aw8624->rtp_mmap_page_alloc_flag) {
			dev_err(aw8624->dev, "%s mmap rtp container invalid\n", __func__);
			return;
		}
		aw8624_haptic_set_rtp_aei(aw8624, false);
		aw8624->rtp_init = 1;
		aw8624->rtp_cnt = 0;
	}

	/* gain */
	//aw8624_haptic_ram_vbat_comp(aw8624, false);
	gain = play->vmax * 128 / HAPTIC_BATTERY_VOLTAGE;
	aw8624_haptic_set_gain(aw8624, gain);

	/* rtp mode config */
	aw8624_haptic_play_mode(aw8624, AW8624_HAPTIC_RTP_MODE);

	/* haptic start */
	mutex_lock(&aw8624->rtp_check_lock);
	if (rtp_check_flag) {
		aw8624_haptic_upload_lra(aw8624, AW8624_HAPTIC_RTP_CALI_LRA);
		aw8624_haptic_start(aw8624);
		dev_info(aw8624->dev, "%s ------------------>\n", __func__);
	} else {
		dev_info(aw8624->dev, "%s rtp work has cancel\n", __func__);
		mutex_unlock(&aw8624->rtp_check_lock);
		return;
	}
	mutex_unlock(&aw8624->rtp_check_lock);

	aw8624_haptic_rtp_init(aw8624);
}


static irqreturn_t aw8624_irq(int irq, void *data)
{
	struct aw8624 *aw8624 = data;
	unsigned char reg_val = 0;
	unsigned char glb_st = 0;
	unsigned int buf_len = 0;

	aw8624_i2c_read(aw8624, AW8624_REG_SYSINT, &reg_val);

	dev_info(aw8624->dev, "%s: reg SYSINT=0x%x\n", __func__, reg_val);

	if (reg_val & AW8624_BIT_SYSINT_UVLI)
		dev_info(aw8624->dev, "%s chip uvlo int error\n", __func__);
	if (reg_val & AW8624_BIT_SYSINT_OCDI)
		dev_info(aw8624->dev, "%s chip over current int error\n", __func__);
	if (reg_val & AW8624_BIT_SYSINT_OTI)
		dev_info(aw8624->dev, "%s chip over temperature int error\n", __func__);
	if (reg_val & AW8624_BIT_SYSINT_DONEI)
		dev_info(aw8624->dev, "%s chip playback done\n", __func__);

	if (reg_val & AW8624_BIT_SYSINT_FF_AFI)
		dev_info(aw8624->dev, "%s: aw8624 rtp mode fifo full empty\n", __func__);

	if (reg_val & AW8624_BIT_SYSINT_UVLI) {
		aw8624_i2c_read(aw8624, AW8624_REG_GLB_STATE, &glb_st);
		if (glb_st == 0) {
			aw8624_i2c_write_bits(aw8624,
						AW8624_REG_SYSINTM,
						AW8624_BIT_SYSINTM_UVLO_MASK,
						AW8624_BIT_SYSINTM_UVLO_OFF);
		}
	}

	if (reg_val & AW8624_BIT_SYSINT_FF_AEI) {
// 		pm_qos_add_request(&pm_qos_req_vb, PM_QOS_CPU_DMA_LATENCY, 400);
		mutex_lock(&aw8624->rtp_lock);
		if (aw8624->rtp_init) {
			while ((!aw8624_haptic_rtp_get_fifo_afs(aw8624)) && (aw8624->play_mode == AW8624_HAPTIC_RTP_MODE)) {
				//dev_info(aw8624->dev, "%s: aw8624 rtp mode fifo update, cnt=%d\n", __func__, aw8624->rtp_cnt);
				if ((aw8624_rtp->len-aw8624->rtp_cnt) < (aw8624->ram.base_addr>>2)) {
					buf_len = aw8624_rtp->len-aw8624->rtp_cnt;
				} else {
					buf_len = (aw8624->ram.base_addr>>2);
				}
				aw8624_i2c_writes(aw8624, AW8624_REG_RTP_DATA, &aw8624_rtp->data[aw8624->rtp_cnt], buf_len);
				aw8624->rtp_cnt += buf_len;

				aw8624_i2c_read(aw8624, AW8624_REG_GLB_STATE, &glb_st);
				if ((aw8624->rtp_cnt == aw8624_rtp->len) || ((glb_st & 0x0f) == 0x00)) {
					dev_info(aw8624->dev, "%s:rtp load completely, glb_st:0x%x, play_count=%d, total_len=%d\n",
						__func__, glb_st, aw8624->rtp_cnt, aw8624_rtp->len);

					aw8624_haptic_set_rtp_aei(aw8624, false);
					aw8624->rtp_cnt = 0;
					aw8624->rtp_init = 0;
					break;
				}
			}
		}
		mutex_unlock(&aw8624->rtp_lock);
//		pm_qos_remove_request(&pm_qos_req_vb);
	}

	if (aw8624->play_mode != AW8624_HAPTIC_RTP_MODE)
		aw8624_haptic_set_rtp_aei(aw8624, false);

	dev_info(aw8624->dev, "%s exit\n", __func__);
	return IRQ_HANDLED;
}


/*****************************************************************************
 *
 * haptic core driver
 *
 ****************************************************************************/
static int aw8624_load_effect(struct aw8624 *aw8624, struct haptic_effect *p_effect)
{
	struct aw8624_play_info *play = &aw8624->play;
	s16 level, custom_data[CUSTOM_DATA_LEN] = {0, 0, 0};
	int real_vmax, i, j = 0, scene_effect_id = 0, scene_vmax = 0;
	struct haptic_wavefrom_info *effect_list = aw8624->effect_list;
	int gain;
	int secne_idx = 0;

	memset(play, 0, sizeof(struct aw8624_play_info));

	dev_info(aw8624->dev, "%s: p_effect->type %d\n", __func__, p_effect->type);
	switch (p_effect->type) {

	case HAPTIC_CONSTANT:

		if (copy_from_user(custom_data, p_effect->custom_data, sizeof(s16) * CUSTOM_DATA_LEN)) {
			dev_err(aw8624->dev, "%s constant, copy from user failed\n", __func__);
			return -EFAULT;
		}


		play->playLength = p_effect->length * USEC_PER_MSEC;
		level = p_effect->magnitude;
		play->vmax = level * aw8624->default_vmax / 0x7fff;
		play->type = TIME_TYPE;
		gain = play->vmax * 128 / HAPTIC_BATTERY_VOLTAGE;
		aw8624->gain = gain;

		dev_info(aw8624->dev, "%s: constant, length_us = %d, vmax_mv = %d, level = %d \n", __func__, play->playLength, play->vmax, level);

		if (!lra_wake_lock_active(aw8624->wklock)) {
			lra_wake_lock(aw8624->wklock);
			dev_info(aw8624->dev, "%s, constant add wake lock\n", __func__);
		}

		aw8624_haptic_stop(aw8624);
		aw8624_haptic_set_repeat_wav_seq(aw8624, PLAYBACK_INFINITELY_RAM_ID);
		aw8624_haptic_set_gain(aw8624, gain);

		break;

	case HAPTIC_RTP_STREAM: //mmap
		if (copy_from_user(custom_data, p_effect->custom_data, sizeof(s16) * CUSTOM_DATA_LEN)) {
			dev_err(aw8624->dev, "%s mmap copy from user failed\n", __func__);
			return -EFAULT;
		}

		if (!aw8624->rtp_mmap_page_alloc_flag) {
			dev_err(aw8624->dev, "%s mmap rtp container invalid\n", __func__);
			return -ENOMEM;
		}

		real_vmax = 4000;

		play->type = RTP_MMAP_TYPE;
		level = p_effect->magnitude;
		play->vmax = level * real_vmax / 0x7fff;
		play->times_ms = custom_data[0];
		play->times_ms = p_effect->data_count / 24; // 24 means sample rate
		gain = level * 128 / 0x7fff;

		if (aw8624_rtp->len != p_effect->data_count) {
			dev_err(aw8624->dev, "data count not eq, rtp len: %d, data count: %d\n", aw8624_rtp->len, p_effect->data_count);
			return -EFAULT;
		}

		aw8624_haptic_stop(aw8624);
		aw8624_haptic_set_rtp_aei(aw8624, false);
//		aw8624_haptic_set_wav_loop(aw8624, 0x00, 0x00);
		aw8624_haptic_set_gain(aw8624, gain);
		aw8624_interrupt_clear(aw8624);

		break;

	case HAPTIC_CUSTOM:

		if (copy_from_user(custom_data, p_effect->custom_data, sizeof(s16) * CUSTOM_DATA_LEN)) {
			dev_err(aw8624->dev, "%s custom, copy from user failed\n", __func__);
			return -EFAULT;
		}

		dev_info(aw8624->dev, "scene id %d\n", custom_data[CUSTOM_DATA_EFFECT_IDX]);
		secne_idx = custom_data[CUSTOM_DATA_EFFECT_IDX];

		if (custom_data[CUSTOM_DATA_EFFECT_IDX] < BASE_SCENE_COUNT_MAX) { //小于300的场景编号，从base scene列表里面找

			for (j = 0; j < aw8624->base_scene_count; j++) {
				if (aw8624->base_scene_list[j].scene_id == custom_data[CUSTOM_DATA_EFFECT_IDX])
					break;
			}

			if (j == aw8624->base_scene_count) {
				dev_err(aw8624->dev, "scene:%d not support\n", custom_data[CUSTOM_DATA_EFFECT_IDX]);
				return -EINVAL;
			}

			scene_vmax = aw8624->base_scene_list[j].real_vmax; //real_vmax为场景设计的实际电压
			scene_effect_id = aw8624->base_scene_list[j].effect_id;
		} else { //大于300的场景编号，从扩展ext scene列表里面找

			for (j = 0; j < aw8624->ext_scene_count; j++) {
				if (aw8624->ext_scene_list[j].scene_id == custom_data[CUSTOM_DATA_EFFECT_IDX])
					break;
			}

			if (j == aw8624->ext_scene_count) {
				dev_err(aw8624->dev, "scene:%d not support\n", custom_data[CUSTOM_DATA_EFFECT_IDX]);
				return -EINVAL;
			}

			scene_vmax = aw8624->ext_scene_list[j].real_vmax;
			scene_effect_id = aw8624->ext_scene_list[j].effect_id;
		}


		for (i = 0; i < aw8624->effects_count; i++) //从效果中寻找场景需要的效果
			if (aw8624->effect_list[i].idx == scene_effect_id)
				break;

		if (i == aw8624->effects_count) {
			dev_err(aw8624->dev, "scene: %d effect: %d not supported!\n",
			custom_data[CUSTOM_DATA_EFFECT_IDX], scene_effect_id);
			return -EINVAL;
		}

		//更新real vmax值，dts配置和hidl配置，取较小值
		if (scene_vmax > 0 && scene_vmax < effect_list[i].vmax)
			real_vmax = scene_vmax;
		else
			real_vmax = effect_list[i].vmax;
		// no boost
		if (real_vmax > HAPTIC_BATTERY_VOLTAGE)
			real_vmax = HAPTIC_BATTERY_VOLTAGE;


		dev_info(aw8624->dev, "real_vamx = %d, scene_vamx = %d, effect_vmax = %d\n", real_vmax, scene_vmax, effect_list[i].vmax);

		if (!effect_list[i].rtp_enable) {

			play->type = RAM_TYPE;
			level = p_effect->magnitude;
			play->vmax = level * real_vmax / 0x7fff;
			play->times_ms = effect_list[i].times_ms;
			play->ram_id = effect_list[i].ram_id;
			gain = play->vmax * 128 / HAPTIC_BATTERY_VOLTAGE;

			dev_err(aw8624->dev, "ram, effect_id = %d, ram_id = %d, vmax_mv = %d, length = %d, level = %d\n",
								scene_effect_id, play->ram_id, play->vmax, play->times_ms, level);
			aw8624_haptic_stop(aw8624);
			aw8624_haptic_set_wav_loop(aw8624, 0x00, 0x00);
			aw8624_haptic_set_wav_seq(aw8624, 0x00, play->ram_id);
			aw8624_haptic_set_gain(aw8624, gain);
			if (aw8624->play.ram_id == 0) {
				aw8624_double_click_switch(aw8624, true);
			}
		} else {

			play->type = RTP_TYPE;
			level = p_effect->magnitude;
			play->vmax = level * real_vmax / 0x7fff;
			play->times_ms = effect_list[i].times_ms;
			strlcpy(play->rtp_file, effect_list[i].rtp_file_name, 128);
			aw8624->rtp_file_num = i;

			dev_info(aw8624->dev, "%s: rtp, effect_id = %d, rtp_name: %s, vamx_mv = %d, length = %d, level = %d\n",
										__func__, scene_effect_id, play->rtp_file, play->vmax, play->times_ms, level);

			if ((secne_idx < 500) && (secne_idx > 400)) {
				if (!lra_wake_lock_active(aw8624->wklock)) {
					lra_wake_lock(aw8624->wklock);
					dev_info(aw8624->dev, "%s, notification add wake lock\n", __func__);
				}
			}

			aw8624_haptic_stop(aw8624);
			aw8624_haptic_set_rtp_aei(aw8624, false);
//			aw8624_haptic_set_wav_loop(aw8624, 0x00, 0x00);
			aw8624_interrupt_clear(aw8624);
		}


		custom_data[CUSTOM_DATA_TIMEOUT_SEC_IDX] = play->times_ms / MSEC_PER_SEC;
		custom_data[CUSTOM_DATA_TIMEOUT_MSEC_IDX] = play->times_ms % MSEC_PER_SEC;

		if (copy_to_user(p_effect->custom_data, custom_data, sizeof(s16) * CUSTOM_DATA_LEN)) {
			dev_err(aw8624->dev, "%s copy to user failed\n", __func__);
			return -EFAULT;
		}

		break;

	default:
		dev_err(aw8624->dev, "%s Unsupported effect type: %d\n", __func__, p_effect->type);
		return -ENODEV;
		break;
	}

	return 0;
}

static int aw8624_playback_vib(struct aw8624 *aw8624, int val)
{
	struct aw8624_play_info *play = &aw8624->play;
	int len;

	dev_info(aw8624->dev, "%s: val=%d\n", __func__, val);

	if (resist_flag == 0) {
		dev_info(aw8624->dev, "%s: resist detect is failed,return\n", __func__);
		return 0;
	}

	if (val) {

		switch (play->type) {
		case RAM_TYPE:
			dev_info(aw8624->dev, "%s: ---> start ram mode\n", __func__);
			aw8624_haptic_play_wav_seq(aw8624, true);
			break;

		case RTP_TYPE:
			pm_qos_update_request(&pm_qos_req_vb, 50);
			dev_info(aw8624->dev, "%s: ---> start rtp mode\n", __func__);
			queue_work(rtp_wq, &aw8624->rtp_work);
			rtp_check_flag = true;
			break;

		case RTP_MMAP_TYPE:
			dev_info(aw8624->dev, "%s: ---> start RTP_MMAP_TYPE mode\n", __func__);
			queue_work(rtp_wq, &aw8624->rtp_work);
			rtp_check_flag = true;
			break;

		case TIME_TYPE:
			aw8624->activate_mode = AW8624_HAPTIC_ACTIVATE_RAM_MODE;

			dev_err(aw8624->dev, "play vmax = %d, aw8624 gain = %d\n", play->vmax, aw8624->gain);

			len = play->playLength + 10000;

			dev_info(aw8624->dev, "%s: ---> start time mode, length = %d, vmax = %d, gain = %d\n",
													__func__, len, play->vmax, aw8624->gain);

			if (aw8624->activate_mode == AW8624_HAPTIC_ACTIVATE_RAM_MODE) {
//				aw8624_haptic_ram_vbat_comp(aw8624, false);
				aw8624_haptic_play_repeat_seq(aw8624, true);
			} else if (aw8624->activate_mode == AW8624_HAPTIC_ACTIVATE_CONT_MODE) {
				aw8624_haptic_cont(aw8624);
			} else {
				dev_info(aw8624->dev, "%s: not suppoert activate mode\n", __func__);
			}
			/* run us timer */
			hrtimer_start(&aw8624->timer,
				ktime_set(len / USEC_PER_SEC, (len % USEC_PER_SEC) * NSEC_PER_USEC),
				HRTIMER_MODE_REL);

			break;

		default:
			dev_info(aw8624->dev, "%s: not suppoert play type\n", __func__);
			break;

		}

	} else {

		pm_qos_update_request(&pm_qos_req_vb, PM_QOS_DEFAULT_VALUE);
		if (hrtimer_active(&aw8624->timer)) {
			hrtimer_cancel(&aw8624->timer);
			dev_info(aw8624->dev, "%s playback cancel timer\n", __func__);
		}

		if (cancel_work_sync(&aw8624->rtp_work)) {
			dev_info(aw8624->dev, "%s palyback pending work cancle success\n", __func__);
		}

		mutex_lock(&aw8624->rtp_check_lock);

		aw8624_haptic_stop(aw8624);
		rtp_check_flag = false;

		mutex_unlock(&aw8624->rtp_check_lock);

		if (aw8624->play.ram_id == 0) {
			aw8624_double_click_switch(aw8624, false);
		}

		aw8624_haptic_upload_lra(aw8624, AW8624_HAPTIC_F0_CALI_LRA);
	}
	return 0;
}


static int aw8624_upload_sync(struct haptic_handle *hp, struct haptic_effect *effect)
{
	int ret = 0;
	struct aw8624 *aw8624 = (struct aw8624 *)hp->chip;

	dev_info(aw8624->dev, "%s  +++\n", __func__);

	ret = aw8624_load_effect(aw8624, effect);

	return ret;

}

static int aw8624_erase_sync(struct haptic_handle *hp)
{
	return 0;
}

static int aw8624_playback_sync(struct haptic_handle *hp, int value)
{
	struct aw8624 *aw8624 = (struct aw8624 *)hp->chip;
	int ret = 0;

	dev_info(aw8624->dev, "%s +++\n", __func__);

	ret = aw8624_playback_vib(aw8624, !!value);

	if (!value) {
		if (lra_wake_lock_active(aw8624->wklock)) {
			lra_wake_unlock(aw8624->wklock);
			dev_info(aw8624->dev, "%s, release wake lock\n", __func__);
		}
	}


	return ret;
}

static void aw8624_set_gain_sync(struct haptic_handle *hp, u16 gain)
{

	struct aw8624 *aw8624 = (struct aw8624 *)hp->chip;
	struct aw8624_play_info *play = &aw8624->play;

	dev_info(aw8624->dev, "%s gain=%d\n", __func__, gain);

	if (play->type == TIME_TYPE) {
		if (gain > 0x7fff)
			gain = 0x7fff;

		aw8624->gain = ((u32)(gain * 0x80)) / 0x7fff;
		aw8624_haptic_ram_vbat_comp(aw8624, false);
	}

}

static void aw8624_get_motor_type_sync(struct haptic_handle *hp, int *motorType)
{
	struct aw8624 *aw8624 = (struct aw8624 *)hp->chip;

	*motorType = (int)aw8624->lra_information;

}

static void aw8624_get_f0_sync(struct haptic_handle *hp, int *f0)
{
	struct aw8624 *aw8624 = (struct aw8624 *)hp->chip;

	*f0 = (int)((aw8624->cali_f0 + 5) / 10);

}

static void aw8624_get_driver_ic_sync(struct haptic_handle *hp, enum ic_type *driver_ic)
{
	struct aw8624 *aw8624 = (struct aw8624 *)hp->chip;

	*driver_ic = AW8624;

	dev_info(aw8624->dev, "%s, ic type (%d)\n", __func__, *driver_ic);

}

static int aw8624_judge_effect_support_sync(struct haptic_handle *hp, int16_t effectNo)
{
	struct aw8624 *aw8624 = (struct aw8624 *)hp->chip;
	int i, j, scene_effect_id = 0;

	if (effectNo < BASE_SCENE_COUNT_MAX) { //小于300的场景编号，从base scene列表里面找

			for (j = 0; j < aw8624->base_scene_count; j++) {
				if (aw8624->base_scene_list[j].scene_id == effectNo)
					break;
			}

			if (j == aw8624->base_scene_count) {
				dev_err(aw8624->dev, "judge: scene:%d not support\n", effectNo);
				return -EINVAL;
			}

			scene_effect_id = aw8624->base_scene_list[j].effect_id;
		} else { //大于300的场景编号，从扩展ext scene列表里面找

			for (j = 0; j < aw8624->ext_scene_count; j++) {
				if (aw8624->ext_scene_list[j].scene_id == effectNo)
					break;
			}

			if (j == aw8624->ext_scene_count) {
				dev_err(aw8624->dev, "judge: scene:%d not support\n", effectNo);
				return -EINVAL;
			}

			scene_effect_id = aw8624->ext_scene_list[j].effect_id;
		}


		for (i = 0; i < aw8624->effects_count; i++) //从效果中寻找场景需要的效果
			if (aw8624->effect_list[i].idx == scene_effect_id)
				break;

		if (i == aw8624->effects_count) {
			dev_err(aw8624->dev, "judge: scene: %d effect: %d not supported!\n",
			effectNo, scene_effect_id);
			return -EINVAL;
		}

		return 0;
}

static void aw8624_set_cali_params_sync(struct haptic_handle *hp, struct haptic_cali_param *cali_params)
{

	struct aw8624 *aw8624 = (struct aw8624 *)hp->chip;

	if (cali_params->calibration_data_len == AWINIC_LEN) {
		aw8624->f0_cali_data = cali_params->u.awinic.f0_offset;
		aw8624->cali_f0 = cali_params->u.awinic.f0;
		dev_info(aw8624->dev, "%s f0_offset = %d, cali_f0 = %d\n", __func__,
			cali_params->u.awinic.f0_offset, cali_params->u.awinic.f0);
	} else {
		dev_info(aw8624->dev, "%s ic type not support\n", __func__);
	}

}

/*********************************************************************************************
 *
 * sysfs ops
 *
 *********************************************************************************************/
static ssize_t aw8624_duration_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8624 *aw8624 = container_of(cdev, struct aw8624, cdev);
	struct aw8624_play_info *play = &aw8624->play;
	unsigned int val = 0;
	int rc = 0;
	dev_info(aw8624->dev, "%s  +++\n", __func__);

	if(ram_load==false)
		return count;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	/* setting 0 on duration is NOP for now */
	if (val <= 0)
		return count;

	aw8624->duration = val;
	play->playLength = val * USEC_PER_MSEC;
	play->ram_id = PLAYBACK_INFINITELY_RAM_ID;
	play->type = TIME_TYPE;

	return count;
}

static ssize_t aw8624_activate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8624 *aw8624 = container_of(cdev, struct aw8624, cdev);
	struct aw8624_play_info *play = &aw8624->play;
	unsigned int val = 0;
	int rc = 0;
	dev_info(aw8624->dev, "%s +++\n", __func__);

	if(ram_load==false)
		return count;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val != 0 && val != 1)
		return count;

	dev_info(aw8624->dev, "%s: constant, length_us = %d, vmax_mv = %d\n", __func__, play->playLength, play->vmax);
	aw8624_haptic_stop(aw8624);
	aw8624_haptic_set_repeat_wav_seq(aw8624, PLAYBACK_INFINITELY_RAM_ID);
	play->vmax = aw8624->default_vmax;
	aw8624->gain = 0x80;
	aw8624_playback_vib(aw8624, val);
	return count;
}


static ssize_t aw8624_iic_int_rst_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8624 *aw8624 = container_of(cdev, struct aw8624, cdev);
	unsigned char reg = 0;

	aw8624_i2c_read(aw8624, AW8624_REG_ID, &reg);

	if (reg != AW8624_CHIPID) {
		dev_err(aw8624->dev, "%s +++:chip id incorrect! reg = %d\n", __func__, reg);
		return snprintf(buf, PAGE_SIZE, "+IIC:\"0\"\n+INT:\"1\"\n+RST:\"1\"\n");
	}
	dev_info(aw8624->dev, "%s +++\n", __func__);
	return snprintf(buf, PAGE_SIZE, "+IIC:\"1\"\n+INT:\"1\"\n+RST:\"1\"\n");
}

static ssize_t aw8624_at_trigger_state_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	ssize_t len = 0;

	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8624 *aw8624 = container_of(cdev, struct aw8624, cdev);

	dev_info(aw8624->dev, "%s +++\n", __func__);
	len += snprintf(buf+len, PAGE_SIZE-len, "aw8624 no trig function");
	return len;
}

static ssize_t aw8624_at_trigger_state_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	return count;
}


/*use current f0,1040 lra no need to cali*/
static ssize_t aw8624_f0_offset_10_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8624 *aw8624 = container_of(cdev, struct aw8624, cdev);


	unsigned int databuf[1] = {0};
	unsigned char reg_val = 0;
	unsigned int f0_limit = 0;
	char f0_cali_lra = 0;
	int f0_cali_step = 0;


	if (aw8624->lra_information == AW8624_LRA_1040) {
		dev_info(aw8624->dev, "%s +++, motor type 1040, no need cali\n", __func__);
	} else {

		if (1 == sscanf(buf, "%d", &databuf[0])) {
			dev_info(aw8624->dev, "%s +++\n", __func__);
			mutex_lock(&aw8624->lock);

			at_test = true;

			aw8624->f0_cali_flag = AW8624_HAPTIC_CALI_F0;
			aw8624_i2c_write(aw8624, AW8624_REG_TRIM_LRA, 0x00);

			switch (databuf[0]) {
			case 0:
				f0_limit = aw8624->f0;
				break;
			case 10:
				f0_limit = aw8624->f0 + 100;
				break;
			case -10:
				f0_limit = aw8624->f0 - 100;
				break;
			default:
				f0_limit = aw8624->f0;
				break;
			}

		/* calculate cali step */
			f0_cali_step = 100000*((int)f0_limit-(int)aw8624->f0_pre) / ((int)f0_limit*25);

			if (f0_cali_step >= 0) {   /*f0_cali_step >= 0*/
				if (f0_cali_step % 10 >= 5)
					f0_cali_step = f0_cali_step/10 + 1 + 32;
				else
					f0_cali_step = f0_cali_step/10  + 32;
			} else {  /*f0_cali_step < 0*/
				if (f0_cali_step % 10 <= -5) {
					f0_cali_step = 32 + (f0_cali_step/10 - 1);
				} else {
					f0_cali_step = 32 + f0_cali_step/10;
				}
			}

			if (f0_cali_step > 31) {
				f0_cali_lra = (char)f0_cali_step - 32;
			} else {
				f0_cali_lra = (char)f0_cali_step + 32;
			}

			/* update cali step */
			aw8624_i2c_write(aw8624, AW8624_REG_TRIM_LRA, (unsigned char)f0_cali_lra);
			aw8624->f0_cali_data = f0_cali_lra;
			aw8624_i2c_read(aw8624, AW8624_REG_TRIM_LRA, &reg_val);

			/* restore default work mode */
			aw8624_haptic_play_mode(aw8624, AW8624_HAPTIC_STANDBY_MODE);
			aw8624->play_mode = AW8624_HAPTIC_RAM_MODE;
			aw8624_i2c_write_bits(aw8624, AW8624_REG_SYSCTRL,
					AW8624_BIT_SYSCTRL_PLAY_MODE_MASK, AW8624_BIT_SYSCTRL_PLAY_MODE_RAM);
			aw8624_haptic_stop(aw8624);

			mdelay(100);
			dev_info(aw8624->dev, "%s, f0_pre=%d, f0_cali_step=%d, f0_cali_lra=%d, reg_val=%#x, set freq to %dHZ\n",
				__func__, aw8624->f0_pre, f0_cali_step, f0_cali_lra, reg_val, f0_limit);

			at_test = false;

			mutex_unlock(&aw8624->lock);

		} else {
			dev_info(aw8624->dev, "%s +++, argument more than one\n", __func__);
		}
	}
	return count;
}

static ssize_t aw8624_is_need_cali_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8624 *aw8624 = container_of(cdev, struct aw8624, cdev);

	ssize_t len = 0;
	int need_cali = 1;
	char lra[] = "X-LRA 0619";

	dev_info(aw8624->dev, "%s +++\n", __func__);

	switch (aw8624->lra_information) {

	case AW8624_LRA_0815:
		strcpy(lra, "X-LRA 0815");
		need_cali = 1;
		break;
	case AW8624_LRA_1040:
		strcpy(lra, "Z-LRA 1040");
		need_cali = 0;
		break;
	case AW8624_LRA_0832:
		strcpy(lra, "Z-LRA 0832");
		need_cali = 1;
		break;
	default:
		strcpy(lra, "X-LRA 0619");
		need_cali = 1;
		break;
	}

	len += snprintf(buf+len, PAGE_SIZE-len, "+Type:\"%s\"\n", lra);
	len += snprintf(buf+len, PAGE_SIZE-len, "+Require:\"%d\"\n", need_cali);

	return len;
}


//先校准再读f0值，再读lra_resistance值
static ssize_t aw8624_cali_f0_resis_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8624 *aw8624 = container_of(cdev, struct aw8624, cdev);

	unsigned char reg_val = 0;
	ssize_t len = 0;
	mutex_lock(&aw8624->lock);

	at_test = true;
	dev_info(aw8624->dev, "%s+++\n", __func__);

	//get resistance
	aw8624_lra_resist_get(aw8624, &reg_val);
	aw8624->lra = 298 * reg_val;

	if ((aw8624->lra >= aw8624->resistance_min) && (aw8624->lra <= aw8624->resistance_max)) {
		dev_info(aw8624->dev, "%s lra resistent test ok, lra=%d\n", __func__, aw8624->lra);
		resist_flag = 1;
	} else {
		dev_info(aw8624->dev, "%s lra resistent over range, lra=%d\n", __func__, aw8624->lra);
		resist_flag = 0;
	}

	len += snprintf(buf+len, PAGE_SIZE-len, "resistance#%s#%d.%d#%d.%d-%d.%d#ou\n",
		(aw8624->lra >= aw8624->resistance_min && aw8624->lra <= aw8624->resistance_max) ? "ok" : "fail",
		aw8624->lra/1000, aw8624->lra%1000,
		aw8624->resistance_min/1000, aw8624->resistance_min%1000/100,
		aw8624->resistance_max/1000, aw8624->resistance_max%1000/100);


	if (aw8624->lra_information == AW8624_LRA_1040) {

		len += snprintf(buf+len, PAGE_SIZE-len, "freqency#ok#170.0#%d.%d-%d.%d#hz\n",
			aw8624->freq_min/10, aw8624->freq_min%10, aw8624->freq_max/10, aw8624->freq_max%10);

		len += snprintf(buf+len, PAGE_SIZE-len, "f0_offset#ok#0\n",
			(aw8624->f0 >= aw8624->freq_min && aw8624->f0 <= aw8624->freq_max) ? "ok" : "fail",
			aw8624->f0_cali_data);
		dev_info(aw8624->dev, "motor type:1040 f0_offset=%d\n", aw8624->f0_cali_data);
	} else {

		mdelay(20);

		aw8624_haptic_f0_calibration(aw8624);

		mdelay(200);

		aw8624->f0_cali_flag = AW8624_HAPTIC_CALI_F0;
		aw8624_haptic_upload_lra(aw8624, AW8624_HAPTIC_F0_CALI_LRA);
		aw8624_haptic_get_f0(aw8624);

		len += snprintf(buf+len, PAGE_SIZE-len, "freqency#%s#%d.%d#%d.%d-%d.%d#hz\n",
			(aw8624->f0 >= aw8624->freq_min && aw8624->f0 <= aw8624->freq_max) ? "ok" : "fail",
			aw8624->f0/10, aw8624->f0%10,
			aw8624->freq_min/10, aw8624->freq_min%10, aw8624->freq_max/10, aw8624->freq_max%10);

		len += snprintf(buf+len, PAGE_SIZE-len, "f0_offset#ok#%d,%d\n", aw8624->f0_cali_data, aw8624->f0);
		aw8624->cali_f0 = aw8624->f0;
	}

	at_test = false;
	mutex_unlock(&aw8624->lock);

	return len;
}



static ssize_t aw8624_resis_f0_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8624 *aw8624 = container_of(cdev, struct aw8624, cdev);

	unsigned char reg_val = 0;
	ssize_t len = 0;
	mutex_lock(&aw8624->lock);

	at_test = true;
	dev_info(aw8624->dev, "%s +++\n", __func__);

	aw8624_lra_resist_get(aw8624, &reg_val);
	aw8624->lra = 298 * reg_val;

	if ((aw8624->lra >= aw8624->resistance_min) && (aw8624->lra <= aw8624->resistance_max)) {
		dev_info(aw8624->dev, "%s lra resistent test ok, lra=%d\n", __func__, aw8624->lra);
		resist_flag = 1;
	} else {
		dev_info(aw8624->dev, "%s lra resistent over range, lra=%d\n", __func__, aw8624->lra);
		resist_flag = 0;
	}

	len += snprintf(buf+len, PAGE_SIZE-len, "resistance#%s#%d.%d#%d.%d-%d.%d#ou\n",
		(aw8624->lra >= aw8624->resistance_min && aw8624->lra <= aw8624->resistance_max) ? "ok" : "fail",
		aw8624->lra/1000, aw8624->lra%1000,
		aw8624->resistance_min/1000, aw8624->resistance_min%1000/100,
		aw8624->resistance_max/1000, aw8624->resistance_max%1000/100);

	mdelay(20);

	aw8624->f0_cali_flag = AW8624_HAPTIC_CALI_F0;
	aw8624_haptic_upload_lra(aw8624, AW8624_HAPTIC_F0_CALI_LRA);
	aw8624_haptic_get_f0(aw8624);

	len += snprintf(buf+len, PAGE_SIZE-len, "freqency#%s#%d.%d#%d.%d-%d.%d#hz\n",
		(aw8624->f0 >= aw8624->freq_min && aw8624->f0 <= aw8624->freq_max) ? "ok" : "fail",
		aw8624->f0/10, aw8624->f0%10,
		aw8624->freq_min/10, aw8624->freq_min%10, aw8624->freq_max/10, aw8624->freq_max%10);

	at_test = false;

	mutex_unlock(&aw8624->lock);

	return len;
}


/* 售后工具线性马达校准项
 * AT+BK_VBR_CAL=1
 *【指令】：cat /sys/class/leds/vibrator/cali_f0
 * Z轴线性马达不需要校准，校准反而可能会影响震动效果
 * 故offset值返回0，且不进行校准，只读f0
 */
static ssize_t aw8624_cali_f0_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8624 *aw8624 = container_of(cdev, struct aw8624, cdev);

	ssize_t len = 0;
	dev_info(aw8624->dev, "%s +++\n", __func__);

	mutex_lock(&aw8624->lock);

	at_test = true;

	if (aw8624->lra_information == AW8624_LRA_1040) {

		dev_info(aw8624->dev, "%s, motor type 1040, no need calibration\n", __func__);

		len += snprintf(buf+len, PAGE_SIZE-len, "ok f0 170.0 (range:%d.%d-%d.%d)hz f0_offset=0\n",
			aw8624->freq_min/10, aw8624->freq_min%10, aw8624->freq_max/10, aw8624->freq_max%10);

	} else {

		aw8624_haptic_f0_calibration(aw8624);

		mdelay(200);

		aw8624->f0_cali_flag = AW8624_HAPTIC_CALI_F0;
		aw8624_haptic_upload_lra(aw8624, AW8624_HAPTIC_F0_CALI_LRA);
		aw8624_haptic_get_f0(aw8624);

		len += snprintf(buf+len, PAGE_SIZE-len, "%s f0 %d.%d (range:%d.%d-%d.%d)hz f0_offset=%d,%d\n",
			(aw8624->f0 >= aw8624->freq_min && aw8624->f0 <= aw8624->freq_max) ? "ok" : "fail",
			aw8624->f0/10, aw8624->f0%10,
			aw8624->freq_min/10, aw8624->freq_min%10, aw8624->freq_max/10, aw8624->freq_max%10,
			aw8624->f0_cali_data, aw8624->f0);

		aw8624->cali_f0 = aw8624->f0;
	}

	at_test = false;

	mutex_unlock(&aw8624->lock);

	return len;
}


static ssize_t aw8624_i2c_reg_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8624 *aw8624 = container_of(cdev, struct aw8624, cdev);

	ssize_t len = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;
	dev_info(aw8624->dev, "%s +++ enter\n.", __func__);
	for (i = 0; i < AW8624_REG_MAX; i++) {
		if (!(aw8624_reg_access[i]&REG_RD_ACCESS))
			continue;
		aw8624_i2c_read(aw8624, i, &reg_val);
		len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%02x=0x%02x \n", i, reg_val);
	}
	return len;
}


static ssize_t aw8624_i2c_reg_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8624 *aw8624 = container_of(cdev, struct aw8624, cdev);


	unsigned int databuf[2] = {0, 0};
	dev_info(aw8624->dev, "%s +++ enter\n.", __func__);

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		aw8624_i2c_write(aw8624, (unsigned char)databuf[0], (unsigned char)databuf[1]);
	}

	return count;
}


static ssize_t aw8624_i2c_ram_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8624 *aw8624 = container_of(cdev, struct aw8624, cdev);

	ssize_t len = 0;
	unsigned int i = 0;
	unsigned char reg_val = 0;
	dev_info(aw8624->dev, "%s +++ enter\n.", __func__);

	aw8624_haptic_stop(aw8624);
	/* RAMINIT Enable */
	aw8624_i2c_write_bits(aw8624, AW8624_REG_SYSCTRL,
			AW8624_BIT_SYSCTRL_RAMINIT_MASK, AW8624_BIT_SYSCTRL_RAMINIT_EN);

	aw8624_i2c_write(aw8624, AW8624_REG_RAMADDRH, (unsigned char)(aw8624->ram.base_addr>>8));
	aw8624_i2c_write(aw8624, AW8624_REG_RAMADDRL, (unsigned char)(aw8624->ram.base_addr&0x00ff));
	len += snprintf(buf+len, PAGE_SIZE-len, "aw8624_haptic_ram:\n");
	for (i = 0; i < aw8624->ram.len; i++) {
		aw8624_i2c_read(aw8624, AW8624_REG_RAMDATA, &reg_val);
		len += snprintf(buf+len, PAGE_SIZE-len, "0x%02x,", reg_val);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	/* RAMINIT Disable */
	aw8624_i2c_write_bits(aw8624, AW8624_REG_SYSCTRL,
			AW8624_BIT_SYSCTRL_RAMINIT_MASK, AW8624_BIT_SYSCTRL_RAMINIT_OFF);

	return len;
}

static ssize_t aw8624_i2c_ram_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8624 *aw8624 = container_of(cdev, struct aw8624, cdev);


	unsigned int databuf[1] = {0};
	dev_info(aw8624->dev, "%s +++ enter\n.", __func__);

	if (1 == sscanf(buf, "%x", &databuf[0])) {
		if (1 == databuf[0]) {
			aw8624_ram_update(aw8624);
		}
	}
	return count;
}

static DEVICE_ATTR(duration, S_IWUSR | S_IRUGO, NULL, aw8624_duration_store);
static DEVICE_ATTR(activate, S_IWUSR | S_IRUGO, NULL, aw8624_activate_store);
static DEVICE_ATTR(iic_int_rst, S_IWUSR | S_IRUGO, aw8624_iic_int_rst_show, NULL);
static DEVICE_ATTR(at_trigger_state, S_IWUSR | S_IRUGO, aw8624_at_trigger_state_show, aw8624_at_trigger_state_store);
static DEVICE_ATTR(f0_offset_10, S_IWUSR | S_IRUGO, NULL, aw8624_f0_offset_10_store);
static DEVICE_ATTR(is_need_cali, S_IWUSR | S_IRUGO, aw8624_is_need_cali_show, NULL);
static DEVICE_ATTR(cali_f0_resis, S_IWUSR | S_IRUGO, aw8624_cali_f0_resis_show, NULL);
static DEVICE_ATTR(resis_f0, S_IWUSR | S_IRUGO, aw8624_resis_f0_show, NULL);
static DEVICE_ATTR(cali_f0, S_IWUSR | S_IRUGO, aw8624_cali_f0_show, NULL);
static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, aw8624_i2c_reg_show, aw8624_i2c_reg_store);
static DEVICE_ATTR(ram, S_IWUSR | S_IRUGO, aw8624_i2c_ram_show, aw8624_i2c_ram_store);


static struct attribute *aw8624_attributes[] = {
	&dev_attr_duration.attr,
	&dev_attr_activate.attr,
	&dev_attr_iic_int_rst.attr,
	&dev_attr_at_trigger_state.attr,
	&dev_attr_f0_offset_10.attr,
	&dev_attr_is_need_cali.attr,
	&dev_attr_cali_f0_resis.attr,
	&dev_attr_resis_f0.attr,
	&dev_attr_cali_f0.attr,
	&dev_attr_reg.attr,
	&dev_attr_ram.attr,
	NULL
};

static struct attribute_group aw8624_attribute_group = {
	.attrs = aw8624_attributes
};

static enum led_brightness aw8624_haptic_brightness_get(struct led_classdev *cdev)
{
	struct aw8624 *aw8624 = container_of(cdev, struct aw8624, cdev);
	dev_info(aw8624->dev, "%s\n", __func__);

	return 0;
}

static void aw8624_haptic_brightness_set(struct led_classdev *cdev,
				enum led_brightness level)
{
	struct aw8624 *aw8624 = container_of(cdev, struct aw8624, cdev);
	dev_info(aw8624->dev, "%s\n", __func__);

}

/********************************************************
 *
 * scene and effect dts parse
 *
 ********************************************************/
static haptic_parse_per_effect_dt(struct aw8624 *chip, struct device_node *child,
	struct haptic_wavefrom_info *effect_node)
{
	int rc;

	rc = of_property_read_u32(child, "awinic,effect-id", &effect_node->idx);
	if (rc < 0) {
		dev_err(chip->dev, "Read awinic effect-id failed, rc=%d\n", rc);
		return rc;
	}
	rc = of_property_read_u32(child, "awinic,wf-vmax-mv", &effect_node->vmax);
	if (rc < 0) {
		dev_err(chip->dev, "Read awinic wf-vmax-mv failed, rc=%d\n", rc);
		return rc;
	}

	rc = of_property_read_u32(child, "awinic,wf-length", &effect_node->times_ms);
	if (rc < 0) {
		dev_err(chip->dev, "Read awinic wf-length failed, rc=%d\n", rc);
		return rc;
	}

	effect_node->rtp_enable = of_property_read_bool(child, "awinic,rtp-enable");

	if (effect_node->rtp_enable) {

		rc = of_property_read_string(child, "awinic,rtp-file", &effect_node->rtp_file_name);
		if (rc < 0) {
			dev_err(chip->dev, "Read awinic rtp-file failed, rc=%d\n", rc);
			return rc;
		}

		// rc = of_property_read_u32(child, "awinic,ram-id", &effect_node->ram_id);
		// if (rc < 0) {
			// dev_err(chip->dev, "Read awinic ram-id failed, rc=%d\n", rc);
			// return rc;
		// }

		effect_node->ram_id = 1;

	} else {

		rc = of_property_read_u32(child, "awinic,ram-id", &effect_node->ram_id);
		if (rc < 0) {
			dev_err(chip->dev, "Read awinic ram-id failed, rc=%d\n", rc);
			return rc;
		}
		effect_node->rtp_file_name = "default";
	}

	return 0;
}


static int haptic_effects_parse_dt(struct aw8624 *chip, struct device_node *effect_node)
{
	struct device_node *child = NULL;
	int i = 0, ret;


	for_each_available_child_of_node(effect_node, child) {
		if (!of_find_property(child, "awinic,effect-id", NULL))
			continue;

		ret = haptic_parse_per_effect_dt(chip, child, &chip->effect_list[i]);
		if (ret < 0) {
			dev_err(chip->dev, "parse effect %d failed, rc=%d\n", i, ret);
			of_node_put(child);
			return ret;
			}
		i++;
	}

	chip->effects_count = i;

	return 0;
}

static int haptic_scenes_parse_dt(struct aw8624 *chip, struct device_node *scene_node,
						int base_scene_element_count, int ext_scene_element_count)
{
	int ret = 0, i;

	#define SCENE_NU    1024
	u16 *temp_effect = NULL;

	temp_effect = devm_kzalloc(chip->dev, base_scene_element_count * sizeof(u16), GFP_KERNEL);// 1024/3=341
	if (!temp_effect) {
		dev_err(chip->dev, "Kzalloc for temp_effect failed\n");
		return -ENOMEM;
	}

	ret = of_property_read_u16_array(scene_node, "base_scene", temp_effect, base_scene_element_count);
	if (ret) {
		dev_err(chip->dev, "Get base effect failed, ret=%d\n", ret);
		return -ENODEV;
	} else {
		for (i = 0; i < base_scene_element_count / 3; i++) {
			chip->base_scene_list[i].scene_id = temp_effect[3 * i];
			chip->base_scene_list[i].effect_id = temp_effect[3 * i + 1];
			chip->base_scene_list[i].real_vmax = temp_effect[3 * i + 2];
		}
		chip->base_scene_count = base_scene_element_count / 3;
	}

	devm_kfree(chip->dev, temp_effect);
	temp_effect = NULL;


	temp_effect = devm_kzalloc(chip->dev, ext_scene_element_count * sizeof(u16), GFP_KERNEL);// 1024/3=341
	if (!temp_effect) {
		dev_err(chip->dev, "Kzalloc for temp_effect failed\n");
		return -ENOMEM;
	}

	ret = of_property_read_u16_array(scene_node, "ext_scene", temp_effect, ext_scene_element_count);
	if (ret) {
		dev_err(chip->dev, "Get ext effect failed, ret=%d\n", ret);
		return -ENODEV;
	} else {
		for (i = 0; i < ext_scene_element_count / 3; i++) {
			chip->ext_scene_list[i].scene_id = temp_effect[3 * i];
			chip->ext_scene_list[i].effect_id = temp_effect[3 * i + 1];
			chip->ext_scene_list[i].real_vmax = temp_effect[3 * i + 2];
		}
		chip->ext_scene_count = ext_scene_element_count / 3;
	}

	devm_kfree(chip->dev, temp_effect);


	return 0;
}

static int aw8624_vibrator_init(struct aw8624 *aw8624)
{
	int ret = 0;
	struct haptic_misc *hm = NULL;
	struct haptic_handle *handle = NULL;
	int order = 0;

	struct device_node *effect_node = NULL;
	struct device_node *child = NULL;
	int effects_count = 0;

	struct device_node *scene_node = NULL;
	int base_scene_element_count = 0;
	int ext_scene_element_count = 0;

	struct device_node *node = aw8624->dev->of_node;


	dev_info(aw8624->dev, "%s enter\n", __func__);

	hrtimer_init(&aw8624->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw8624->timer.function = aw8624_vibrator_timer_func;
	INIT_WORK(&aw8624->vibrator_work, aw8624_vibrator_work_routine);

	/* rtp模式要求数据写入及时，不能断流，故建立单独的工作队列，专门处理rtp数据传输，而不使用系统默认的工作队列 */
	rtp_wq = create_singlethread_workqueue("rtp_wq");
	if (!rtp_wq) {
		dev_err(aw8624->dev, "Create rtp work queue failed\n");
		return -ENOENT;
	}

	INIT_WORK(&aw8624->rtp_work, aw8624_rtp_work_routine);

	mutex_init(&aw8624->lock);
	mutex_init(&aw8624->rtp_check_lock);
	mutex_init(&aw8624->rtp_lock);

	aw8624->wklock = lra_wake_lock_register(aw8624->dev, "vivo-aw8624-wakelock");

	aw8624->f0_cali_data = DEFAULT_CALI_F0;
	aw8624->osc_cali_data = DEFAULT_OSC_CALI_DATA;
	aw8624->activate_mode = AW8624_HAPTIC_ACTIVATE_RAM_MODE;//0 是ram模式 1是CONT
	aw8624->cali_f0 = 2350; //235hz

	/* 由于每次写512字节，并且频繁申请和释放，所以为rtp模式开辟单独的高速缓存，提高内存分配速度 */
	rtp_cachep = kmem_cache_create("aw8624-hap", RTP_SLAB_SIZE + 1, 0, SLAB_HWCACHE_ALIGN, NULL);
	if (rtp_cachep == NULL) {
		dev_err(aw8624->dev, "%s alloc high cache failed\n", __func__);
	} else {
		dev_info(aw8624->dev, "%s alloc high cache success\n", __func__);
	}

	// scene and effect dts parse begin
	effect_node = of_parse_phandle(node, "effect_list_aw862xx", 0);
	if (!effect_node) {
		dev_err(aw8624->dev, "Get effect_list phandle failed\n");
		ret = -ENODEV;
		goto dts_err;
	}

	for_each_available_child_of_node(effect_node, child) {
		if (of_find_property(child, "awinic,effect-id", NULL))
			effects_count++;
	}

	aw8624->effect_list = devm_kcalloc(aw8624->dev, effects_count,
				sizeof(struct haptic_wavefrom_info), GFP_KERNEL);
	if (!aw8624->effect_list) {
		dev_err(aw8624->dev, "%s mem alloc failed\n", __func__);
		ret = -ENOMEM;
		goto dts_err;
	}

	ret = haptic_effects_parse_dt(aw8624, effect_node);
	if (ret) {
		dev_err(aw8624->dev, "%s parse effect failed\n", __func__);
		ret = -EFAULT;
		goto dts_mem_err;
	}

	scene_node = of_parse_phandle(node, "scene_array_aw862xx", 0);
	if (!scene_node) {
		dev_err(aw8624->dev, "Get effect array phandle failed\n");
		ret = -ENODEV;
		goto dts_mem_err;
	}

	base_scene_element_count = of_property_count_u16_elems(scene_node, "base_scene");
	if (base_scene_element_count < 0) {
		dev_err(aw8624->dev, "Get base effect elements count failed\n");
		ret = -EINVAL;
		goto dts_mem_err;
	}

	aw8624->base_scene_list = devm_kcalloc(aw8624->dev, base_scene_element_count / 3, sizeof(struct scene_effect_info), GFP_KERNEL);
	if (!aw8624->base_scene_list) {
		dev_err(aw8624->dev, "Kcalloc for base effect failed\n");
		ret = -ENOMEM;
		goto dts_mem_err;
	}


	ext_scene_element_count = of_property_count_u16_elems(scene_node, "ext_scene");
	if (ext_scene_element_count < 0) {
		dev_err(aw8624->dev, "Get ext effect elements count failed\n");
		ret = -EINVAL;
		goto dts_mem_err_2;

	}

	aw8624->ext_scene_list = devm_kcalloc(aw8624->dev, ext_scene_element_count / 3, sizeof(struct scene_effect_info), GFP_KERNEL);
	if (!aw8624->ext_scene_list) {
		dev_err(aw8624->dev, "Kcalloc for ext effect failed\n");
		ret = -ENOMEM;
		goto dts_mem_err_2;

	}

	ret = haptic_scenes_parse_dt(aw8624, scene_node, base_scene_element_count, ext_scene_element_count);
	if (ret) {
		dev_err(aw8624->dev, "%s parse scenes failed\n", __func__);
		ret = -EFAULT;
		goto dts_mem_err_3;
	}

	if (g_logDts) {
		int j;
		dev_err(aw8624->dev, "Effects Dump begin ++++++++++++++++++++++++++++++++++++++++++++++\n");
		for (j = 0; j < aw8624->effects_count; j++) {
			dev_err(aw8624->dev, "effect: %d, idx: %u, ram_id: %u, vmax: %u, time_ms: %u, rtp_enable: %d, rtp_file_name: %s\n",
				j, aw8624->effect_list[j].idx, aw8624->effect_list[j].ram_id, aw8624->effect_list[j].vmax, aw8624->effect_list[j].times_ms,
				aw8624->effect_list[j].rtp_enable, aw8624->effect_list[j].rtp_file_name);
		}
		dev_err(aw8624->dev, "Base Scenes Dump begin ++++++++++++++++++++++++++++++++++++++++++++++\n");
		for (j = 0; j < aw8624->base_scene_count; j++) {
			dev_err(aw8624->dev, "base scene: %d, scene_id: %u, effect_id: %u, real_vmax: %u\n",
				j, aw8624->base_scene_list[j].scene_id, aw8624->base_scene_list[j].effect_id, aw8624->base_scene_list[j].real_vmax);
		}
		dev_err(aw8624->dev, "Ext Scenes Dump begin ++++++++++++++++++++++++++++++++++++++++++++++\n");
		for (j = 0; j < aw8624->ext_scene_count; j++) {
			dev_err(aw8624->dev, "ext scene: %d, scene_id: %u, effect_id: %u, real_vmax: %u\n",
				j, aw8624->ext_scene_list[j].scene_id, aw8624->ext_scene_list[j].effect_id, aw8624->ext_scene_list[j].real_vmax);
		}

	}


	/* register haptic miscdev */
	hm = devm_kzalloc(aw8624->dev, sizeof(struct haptic_misc), GFP_KERNEL);
	if (!hm) {
		dev_err(aw8624->dev, "%s: kzalloc for haptic_misc failed\n", __func__);
		ret = -ENOMEM;
		goto dts_mem_err_3;
	}

	ret = haptic_handle_create(hm, "vivo_haptic");
	if (ret < 0) {
		dev_err(aw8624->dev, "%s: handle create failed\n", __func__);
//		return ret;
		goto dts_mem_err_4;

	}
	handle = hm->handle;
	handle->chip = aw8624;
	handle->dev = &aw8624->i2c->dev;
	handle->upload = aw8624_upload_sync;
	handle->playback = aw8624_playback_sync;
	handle->erase = aw8624_erase_sync;
	handle->set_gain = aw8624_set_gain_sync;
	handle->get_motor_type = aw8624_get_motor_type_sync;
	handle->get_f0 = aw8624_get_f0_sync;
	handle->set_cali_params = aw8624_set_cali_params_sync;
	handle->get_driver_ic = aw8624_get_driver_ic_sync;
	handle->judge_effect_support = aw8624_judge_effect_support_sync;
	haptic_device_set_capcity(handle, HAPTIC_MASK_BIT_SUPPORT_EFFECT);
	haptic_device_set_capcity(handle, HAPTIC_MASK_BIT_SUPPORT_GAIN);
	haptic_device_set_capcity(handle, HAPTIC_MASK_BIT_TRIGGER_INTENSITY);

	ret = haptic_miscdev_register(hm);
	if (ret) {
		dev_err(aw8624->dev, "%s register misc register failed, ret=%d\n", __func__, ret);
		ret = -EFAULT;
		goto dts_mem_err_4;
	}
	aw8624->hm = hm;

	order = get_order(RTP_BIN_MAX_SIZE + sizeof(int));
	aw8624_rtp = (struct haptic_rtp_container *)__get_free_pages(GFP_KERNEL, order);
	if (aw8624_rtp == NULL) {
		dev_err(aw8624->dev, "Error __get_free_pages failed\n");
		aw8624->rtp_mmap_page_alloc_flag = false;
	} else {
		aw8624->rtp_mmap_page_alloc_flag = true;
		SetPageReserved(virt_to_page(aw8624_rtp));
		aw8624->hm->handle->haptic_container = aw8624_rtp;
		g_order = order;

	}


	aw8624->cdev.name = "vibrator";
	aw8624->cdev.brightness_get = aw8624_haptic_brightness_get;
	aw8624->cdev.brightness_set = aw8624_haptic_brightness_set;

	ret = devm_led_classdev_register(&aw8624->i2c->dev, &aw8624->cdev);
	if (ret < 0) {
		dev_err(aw8624->dev, "%s: fail to create led dev\n", __func__);
		goto sysfs_err;
	}

	ret = sysfs_create_group(&aw8624->cdev.dev->kobj, &aw8624_attribute_group);
	if (ret) {
		dev_err(aw8624->dev, "%s  error creating sysfs attr files\n", __func__);
		goto sysfs_err;
	}

	return 0;

sysfs_err:
	if (aw8624_rtp)
		free_pages((unsigned long)aw8624_rtp, order);
	haptic_miscdev_unregister(hm);
dts_mem_err_4:
	devm_kfree(aw8624->dev, hm);
dts_mem_err_3:
	devm_kfree(aw8624->dev, aw8624->ext_scene_list);
dts_mem_err_2:
	devm_kfree(aw8624->dev, aw8624->base_scene_list);
dts_mem_err:
	devm_kfree(aw8624->dev, aw8624->effect_list);
dts_err:
	if (rtp_cachep)
		kmem_cache_destroy(rtp_cachep);
	lra_wake_lock_unregister(aw8624->wklock);
	destroy_workqueue(rtp_wq);
	mutex_destroy(&aw8624->lock);
	mutex_destroy(&aw8624->rtp_lock);
	mutex_destroy(&aw8624->rtp_check_lock);
	return ret;

}

static int aw8624_haptic_init(struct aw8624 *aw8624)
{
	int ret = 0;
	unsigned char reg_flag = 0;
	unsigned char bemf_config = 0;

	dev_info(aw8624->dev, "%s  enter\n", __func__);

	ret = aw8624_i2c_read(aw8624, AW8624_REG_EF_RDATAH, &reg_flag);
	if ((ret >= 0) && ((reg_flag & 0x1) == 1)) {
		aw8624->chipid_flag = 1;
	} else {
		dev_err(aw8624->dev, "%s: to read register AW8624_REG_EF_RDATAH: %d\n", __func__, ret);
	}


	//	ret = aw8624_i2c_read(aw8624, AW8624_REG_WAVSEQ1, &reg_val);
	//	aw8624->index = reg_val & 0x7F;
	//	ret = aw8624_i2c_read(aw8624, AW8624_REG_DATDBG, &reg_val);
	//	aw8624->gain = reg_val & 0xFF;
	//	for (i = 0; i < AW8624_SEQUENCER_SIZE; i++) {
	//		ret = aw8624_i2c_read(aw8624, AW8624_REG_WAVSEQ1+i, &reg_val);
	//		aw8624->seq[i] = reg_val;
	//	}

	aw8624_haptic_play_mode(aw8624, AW8624_HAPTIC_STANDBY_MODE);
	aw8624_haptic_set_pwm(aw8624, AW8624_PWM_24K);

	aw8624_haptic_swicth_motorprotect_config(aw8624, 0x0, 0x0);
	aw8624_haptic_vbat_mode(aw8624, AW8624_HAPTIC_VBAT_HW_COMP_MODE);


	/* f0 calibration */
	aw8624->f0_pre = aw8624->lra_info.AW8624_HAPTIC_F0_PRE;
	aw8624->cont_drv_lvl = aw8624->lra_info.AW8624_HAPTIC_CONT_DRV_LVL;
	aw8624->cont_drv_lvl_ov = aw8624->lra_info.AW8624_HAPTIC_CONT_DRV_LVL_OV;
	aw8624->cont_td = aw8624->lra_info.AW8624_HAPTIC_CONT_TD;
	aw8624->cont_zc_thr = aw8624->lra_info.AW8624_HAPTIC_CONT_ZC_THR;
	aw8624->cont_num_brk = aw8624->lra_info.AW8624_HAPTIC_CONT_NUM_BRK;

	aw8624->ram_vbat_comp = AW8624_HAPTIC_RAM_VBAT_COMP_ENABLE;

	aw8624_i2c_write_bits(aw8624, AW8624_REG_R_SPARE,
		AW8624_BIT_R_SPARE_MASK, AW8624_BIT_R_SPARE_ENABLE);

	/*LRA trim source select register*/
	aw8624_i2c_write_bits(aw8624, AW8624_REG_ANACTRL,
				AW8624_BIT_ANACTRL_LRA_SRC_MASK,
				AW8624_BIT_ANACTRL_LRA_SRC_REG);
	aw8624_haptic_upload_lra(aw8624, AW8624_HAPTIC_ZERO);

	/*brake*/
	aw8624_i2c_write(aw8624, AW8624_REG_SW_BRAKE, (unsigned char)0x2c);
	aw8624_i2c_write(aw8624, AW8624_REG_THRS_BRA_END, 0x00);
	aw8624_i2c_write_bits(aw8624, AW8624_REG_WAVECTRL,
			AW8624_BIT_WAVECTRL_NUM_OV_DRIVER_MASK,
			AW8624_BIT_WAVECTRL_NUM_OV_DRIVER);

	aw8624->f0_value = 20000 / 2350 + 1;

	/* zero cross */
	aw8624_i2c_write(aw8624, AW8624_REG_ZC_THRSH_H, (unsigned char)(aw8624->cont_zc_thr>>8));
	aw8624_i2c_write(aw8624, AW8624_REG_ZC_THRSH_L, (unsigned char)(aw8624->cont_zc_thr>>0));
	aw8624_i2c_write(aw8624, AW8624_REG_TSET, 0x11);

	/* bemf */
	bemf_config = 0x10;
	aw8624_i2c_write(aw8624, AW8624_REG_BEMF_VTHH_H, bemf_config);
	bemf_config = 0x08;
	aw8624_i2c_write(aw8624, AW8624_REG_BEMF_VTHH_L, bemf_config);
	bemf_config = 0x23;
	aw8624_i2c_write(aw8624, AW8624_REG_BEMF_VTHL_H, bemf_config);
	bemf_config = 0xf8;
	aw8624_i2c_write(aw8624, AW8624_REG_BEMF_VTHL_L, bemf_config);

	aw8624_i2c_write_bits(aw8624,
			AW8624_REG_DBGCTRL,
			AW8624_BIT_DBGCTRL_INTN_TRG_SEL_MASK,
			AW8624_BIT_DBGCTRL_INTN_SEL_ENABLE); //选择trig/intn引脚的功能,使能中断

	return 0;
}


/*************************************************************************************************
 *
 * ram init
 *
 **********************************************************************************************/
static void aw8624_container_update(struct aw8624 *aw8624, struct haptic_rtp_container *aw8624_cont) //
{
	int i = 0;
	unsigned int shift = 0;

	mutex_lock(&aw8624->lock);

	aw8624->ram.baseaddr_shift = 2;
	aw8624->ram.ram_shift = 4;

	/* RAMINIT Enable */
	aw8624_i2c_write_bits(aw8624, AW8624_REG_SYSCTRL,
		AW8624_BIT_SYSCTRL_RAMINIT_MASK, AW8624_BIT_SYSCTRL_RAMINIT_EN);

	/* base addr */
	shift = aw8624->ram.baseaddr_shift;
	aw8624->ram.base_addr = (unsigned int)((aw8624_cont->data[0+shift]<<8) |
		(aw8624_cont->data[1+shift]));
	dev_info(aw8624->dev, "%s: base_addr=0x%4x\n", __func__, aw8624->ram.base_addr);

	aw8624_i2c_write(aw8624,
			AW8624_REG_BASE_ADDRH,
			aw8624_cont->data[0+shift]);
	aw8624_i2c_write(aw8624,
			AW8624_REG_BASE_ADDRL,
			aw8624_cont->data[1+shift]);

	aw8624_i2c_write(aw8624,
			AW8624_REG_FIFO_AEH,
			(unsigned char)((aw8624->ram.base_addr>>2)>>8));
	aw8624_i2c_write(aw8624,
			AW8624_REG_FIFO_AEL,
			(unsigned char)((aw8624->ram.base_addr>>2)&0x00FF));
	aw8624_i2c_write(aw8624,
			AW8624_REG_FIFO_AFH,
			(unsigned char)((aw8624->ram.base_addr
			- (aw8624->ram.base_addr>>2))>>8));
	aw8624_i2c_write(aw8624,
			AW8624_REG_FIFO_AFL,
			(unsigned char)((aw8624->ram.base_addr
			-(aw8624->ram.base_addr>>2))&0x00FF));

	/* ram */
	shift = aw8624->ram.baseaddr_shift;
	aw8624_i2c_write(aw8624,
			AW8624_REG_RAMADDRH, aw8624_cont->data[0+shift]);
	aw8624_i2c_write(aw8624,
			AW8624_REG_RAMADDRL, aw8624_cont->data[1+shift]);
	shift = aw8624->ram.ram_shift;
	for (i = shift; i < aw8624_cont->len; i++) {
		aw8624_i2c_write(aw8624,
				AW8624_REG_RAMDATA, aw8624_cont->data[i]);
	}

	/* RAMINIT Disable */
	aw8624_i2c_write_bits(aw8624,
				AW8624_REG_SYSCTRL,
				AW8624_BIT_SYSCTRL_RAMINIT_MASK,
				AW8624_BIT_SYSCTRL_RAMINIT_OFF);

	mutex_unlock(&aw8624->lock);

}

static void aw8624_ram_loaded(const struct firmware *cont, void *context) //
{
	struct aw8624 *aw8624 = context;
	struct haptic_rtp_container *aw8624_fw;
	int i = 0;
	unsigned short check_sum = 0;

	if (!cont) {
		dev_err(aw8624->dev, "%s: failed to read [%s]\n", __func__, aw8624_ram_name);
		release_firmware(cont);
		return;
	}

	dev_info(aw8624->dev, "%s enter: loaded [%s] - size: %zu\n", __func__, aw8624_ram_name,
					cont ? cont->size : 0);

	/* check sum */
	for (i = 2; i < cont->size; i++) {
		check_sum += cont->data[i];
	}
	if (check_sum != (unsigned short)((cont->data[0]<<8)|(cont->data[1]))) {
		dev_err(aw8624->dev, "%s: check sum err: check_sum=0x%04x\n", __func__, check_sum);
		return;
	} else {
		dev_info(aw8624->dev, "%s: check sum pass : 0x%04x\n", __func__, check_sum);
		aw8624->ram.check_sum = check_sum;
	}

	/* aw8624 ram update */
	aw8624_fw = kzalloc(cont->size+sizeof(int), GFP_KERNEL);
	if (!aw8624_fw) {
		release_firmware(cont);
		dev_err(aw8624->dev, "%s: Error allocating memory\n", __func__);
		return;
	}
	aw8624_fw->len = cont->size;
	memcpy(aw8624_fw->data, cont->data, cont->size);
	release_firmware(cont);

	aw8624_container_update(aw8624, aw8624_fw);

	aw8624->ram.len = aw8624_fw->len;

	kfree(aw8624_fw);

	aw8624->ram_init = 1;
	ram_load = true;
	dev_info(aw8624->dev, "%s: ram fw update complete\n", __func__);
}

static int aw8624_ram_update(struct aw8624 *aw8624)
{
	//char file_name[128] = {0};

	aw8624->ram_init = 0;
	aw8624->rtp_init = 0;

	//if (aw8624->add_suffix) {
		//snprintf(file_name, 128, "%s%s", "_", aw8624_ram_name);
	//} else {
		//strlcpy(file_name, aw8624_ram_name, 128);
	//}

	dev_info(aw8624->dev, "%s, file_name: %s\n", __func__, aw8624_ram_name);

	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				aw8624_ram_name, aw8624->dev, GFP_KERNEL,
				aw8624, aw8624_ram_loaded);
}


#ifdef AWINIC_RAM_UPDATE_DELAY
static void aw8624_ram_work_routine(struct work_struct *work)
{
	struct aw8624 *aw8624 = container_of(work, struct aw8624, ram_work.work);

	dev_info(aw8624->dev, "%s enter\n", __func__);

	aw8624_ram_update(aw8624);

}
#endif

static int aw8624_ram_init(struct aw8624 *aw8624)
{

#ifdef AWINIC_RAM_UPDATE_DELAY
	int ram_timer_val = 5000;

	INIT_DELAYED_WORK(&aw8624->ram_work, aw8624_ram_work_routine);
	schedule_delayed_work(&aw8624->ram_work, msecs_to_jiffies(ram_timer_val));
#else
	aw8624_ram_update(aw8624);
#endif
	return 0;
}

static int aw8624_parse_common_dt(struct device *dev, struct aw8624 *aw8624,
		struct device_node *np)
{
	int ret;

	aw8624->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw8624->reset_gpio < 0) {
		dev_err(dev, "%s: no reset gpio provided, will not HW reset device\n", __func__);
		return -ENODEV;
	}

	aw8624->irq_gpio =	of_get_named_gpio(np, "irq-gpio", 0);
	if (aw8624->irq_gpio < 0) {
		dev_err(dev, "%s: no irq gpio provided.\n", __func__);
		return -ENODEV;
	}

	if (of_property_read_u32(np, "awinic,vmax", &aw8624->default_vmax)) {
		dev_err(dev, "%s: no default vmax\n", __func__);
		return -EFAULT;
	}

	if (of_property_read_u32(np, "resistance_min", &aw8624->resistance_min)) {
		dev_err(dev, "%s: no resistance_min\n", __func__);
		return -EFAULT;
	}


	if (of_property_read_u32(np, "resistance_max", &aw8624->resistance_max)) {
		dev_err(dev, "%s: no resistance_max\n", __func__);
		return -EFAULT;
	}

	if (of_property_read_u32(np, "freq_min", &aw8624->freq_min)) {
		dev_err(dev, "%s: no freq_min\n", __func__);
		return -EFAULT;
	}

	if (of_property_read_u32(np, "freq_max", &aw8624->freq_max)) {
		dev_err(dev, "%s: no freq_max\n", __func__);
		return -EFAULT;
	}

	if (of_property_read_bool(np, "disable-trigger")) {
		dev_info(dev, "not support trigger\n");
		aw8624->no_trigger = true;
	}

	  /* prase lra_info */
	if (of_property_read_u32(np, "lra_info", &aw8624->lra_information)) {
		aw8624->lra_information = 619;
		dev_err(dev, "lra_info:%d\n", aw8624->lra_information);
	}

	ret = aw8624_lra_information_ctr(aw8624);
	if (ret < 0) {
		dev_err(dev, "%s: lra_information_ctr get failed\n", __func__);
	}

	return 0;
}


static int aw8624_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct aw8624 *aw8624;
	struct device_node *np = i2c->dev.of_node;
	int irq_flags = 0;
	int ret = -1;

	dev_err(&i2c->dev, "%s +++\n", __func__);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, " check_functionality failed\n");
		return -EIO;
	}

	aw8624 = devm_kzalloc(&i2c->dev, sizeof(struct aw8624), GFP_KERNEL);
	if (aw8624 == NULL)
		return -ENOMEM;

	aw8624->dev = &i2c->dev;
	aw8624->i2c = i2c;
	i2c_set_clientdata(i2c, aw8624);
	g_aw8624 = aw8624;

	mutex_init(&aw8624->bus_lock);

	if (np) {
		ret = aw8624_parse_common_dt(&i2c->dev, aw8624, np);
		if (ret) {
			dev_err(&i2c->dev, "%s:  failed to parse device tree node\n", __func__);
			return -ENODEV;
		}
	} else {
		aw8624->reset_gpio = -1;
		aw8624->irq_gpio = -1;
		dev_err(aw8624->dev, "No dts device node\n");
		return -ENODEV;
	}

	if (gpio_is_valid(aw8624->reset_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw8624->reset_gpio, GPIOF_OUT_INIT_LOW, "aw8624_rst");
		if (ret) {
			dev_err(&i2c->dev, "%s: rst request failed\n", __func__);
			return -ENODEV;
		}

		msleep(2);
		gpio_set_value_cansleep(aw8624->reset_gpio, 1);
		msleep(5);

	}

	/* aw8624 chip id */
	ret = aw8624_read_chipid(aw8624);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: aw8624_read_chipid failed ret=%d\n", __func__, ret);
		return -ENODEV;
	}

	if (gpio_is_valid(aw8624->irq_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw8624->irq_gpio, GPIOF_DIR_IN, "aw8624_int");
		if (ret) {
			dev_err(&i2c->dev, "%s: int request failed\n", __func__);
			return -ENODEV;
		}

		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(&i2c->dev, gpio_to_irq(aw8624->irq_gpio),
					NULL, aw8624_irq, irq_flags, "aw8624", aw8624);
		if (ret != 0) {
			dev_err(&i2c->dev, "%s: failed to request IRQ %d: %d\n",
					__func__, gpio_to_irq(aw8624->irq_gpio), ret);
			return -ENODEV;
		}

	}

	aw8624_interrupt_setup(aw8624);

	/* ic hw init */
	aw8624_haptic_init(aw8624);

	/* Resource request */

	ret = aw8624_vibrator_init(aw8624);
	if (ret) {
		dev_err(&i2c->dev, "%s: vibrator init failed, ret=%d\n", __func__, ret);
		return -EFAULT;
	}

	aw8624_ram_init(aw8624);

	pm_qos_add_request(&pm_qos_req_vb, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);

	dev_info(&i2c->dev, "%s probe completed successfully!\n", __func__);

	return 0;
}




static int aw8624_i2c_remove(struct i2c_client *i2c)
{
	struct aw8624 *aw8624 = i2c_get_clientdata(i2c);

	dev_info(aw8624->dev, "%s enter\n", __func__);
	pm_qos_remove_request(&pm_qos_req_vb);
	sysfs_remove_group(&i2c->dev.kobj, &aw8624_attribute_group);

	if (aw8624_rtp && g_order)
		free_pages((unsigned long)aw8624_rtp, g_order);
	haptic_miscdev_unregister(aw8624->hm);
	lra_wake_lock_unregister(aw8624->wklock);
	if (rtp_cachep)
		kmem_cache_destroy(rtp_cachep);
	destroy_workqueue(rtp_wq);
	mutex_destroy(&aw8624->lock);
	mutex_destroy(&aw8624->rtp_lock);
	mutex_destroy(&aw8624->rtp_check_lock);
	mutex_destroy(&aw8624->bus_lock);

	return 0;
}

static void aw8624_pm_shutdown(struct i2c_client *i2c)
{

	dev_info(&i2c->dev, "%s enter\n", __func__);

}

static int aw8624_pm_suspend (struct device *dev)
{
	struct aw8624 *aw8624 = dev_get_drvdata(dev);

	if (hrtimer_active(&aw8624->timer)) {
		hrtimer_cancel(&aw8624->timer);
	}

	i2c_suspend = 1;
	dev_info(aw8624->dev, "%s enter, pm_suspend_flag = %d\n", __func__, i2c_suspend);


	return 0;

}


static int aw8624_pm_resume (struct device *dev)
{
	struct aw8624 *aw8624 = dev_get_drvdata(dev);
	i2c_suspend = 0;
	dev_info(aw8624->dev, "%s enter, pm_suspend_flag = %d\n", __func__, i2c_suspend);
	return 0;


}

static struct dev_pm_ops aw8624_pm_ops = {
	.suspend = aw8624_pm_suspend,
	.resume = aw8624_pm_resume,
};


static const struct i2c_device_id aw8624_i2c_id[] = {
	{ AW8624_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw8624_i2c_id);

static struct of_device_id aw8624_dt_match[] = {
	{ .compatible = "awinic,aw8624_haptic" },
	{ },
};

static struct i2c_driver aw8624_i2c_driver = {
	.driver = {
		.name = AW8624_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw8624_dt_match),
		.pm = &aw8624_pm_ops,
	},
	.shutdown = aw8624_pm_shutdown,
	.probe = aw8624_i2c_probe,
	.remove = aw8624_i2c_remove,
	.id_table = aw8624_i2c_id,
};

static int __init aw8624_i2c_init(void)
{
	int ret = 0;

	pr_info("aw8624 driver version %s\n", AW8624_VERSION);

	ret = i2c_add_driver(&aw8624_i2c_driver);
	if (ret) {
		pr_err("aw8624 fail to add device into i2c\n");
		return ret;
	}

	return 0;
}
late_initcall(aw8624_i2c_init);

static void __exit aw8624_i2c_exit(void)
{
	i2c_del_driver(&aw8624_i2c_driver);
}
module_exit(aw8624_i2c_exit);


MODULE_DESCRIPTION("AW8624 Haptic Driver");
MODULE_LICENSE("GPL v2");

