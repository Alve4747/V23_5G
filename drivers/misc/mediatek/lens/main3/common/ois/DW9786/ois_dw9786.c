/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include "ois_dw9786.h"

#define I2COP_CHECK(cond) \
	if (cond) {                   \
		LOG_OIS_ERR("ois i2c fail(%d)", cond); \
		goto p_err; \
	}

//current specific fw
const struct ois_fw_info EXP_FW = {
	.version = 0x0207,
	.type    = FW_VIVO,
	.date    = 0x0130
};

#define GAIN_TABLE_SIZE_MAX 100
#define VIVO_OTP_DATA_SIZE 0x1A40
extern unsigned char otp_data_ov08a10[VIVO_OTP_DATA_SIZE];
extern int log_ois_level_tele;

static int dw9786_log_control(struct ois *ois, int level)
{
	int ret = 0;

	if (level > OIS_LOG_START && level < OIS_LOG_END)
		log_ois_level_tele = level;

	LOG_OIS_INF("log %d", log_ois_level_tele);
	return ret;
}

static int dw9786_reset(struct ois *ois)
{
	int ret        = 0;
	struct i2c_client *client = ois->client;

	OIS_BUG(!client);
	ois->client->addr = DW9786_SLAVE_ADDR >> 1;

	/*1 shutdown mode*/
	ret = ois_i2c_write_tele(client, DW9786_CHIP_EN, DW9786_SHUTDOWN_MODE);
	I2COP_CHECK(ret);
	mdelay(2);
	/*2 standby mode*/
	ret = ois_i2c_write_tele(client, DW9786_CHIP_EN, DW9786_STANDBY_MODE);
	I2COP_CHECK(ret);
	mdelay(5);
	/*3 soft protection*/
	ret = ois_i2c_write_tele(client, DW9786_SOFT_PROTECT, DW9786_SOFTPT_OFF);
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(client, DW9786_IMON_MUX_SEL, DW9786_IMONSEL_ON);
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(client, DW9786_SOFT_PROTECT, DW9786_SOFTPT_ON);
	I2COP_CHECK(ret);
	mdelay(1);
	/*4 idle mode(MCU on)*/
	ret = ois_i2c_write_tele(client, DW9786_MCU_ACTIVE, ON);
	I2COP_CHECK(ret);
	mdelay(50);

	LOG_OIS_INF("ois reset success(%d)", ret);
p_err:
	return ret;
}

static int dw9786_set_mode(struct ois *ois, int mode)
{
	int ret       = 0;
	u16 exp_mode  = 0x0000;
	u16 old_mode  = 0x0000;
	u16 new_mode  = 0x0000;
	u16 servo     = 0x0000;
	u16 op_status = OPERATE_DONE;
	struct i2c_client *client = ois->client;

	LOG_OIS_INF("dw9786_set_mode ois mode(%d)", mode);
	OIS_BUG(!client);

	ret = ois_i2c_read_tele(client, DW9786_OIS_MODE_SELECT, &old_mode);

	switch (mode) {
	case OIS_CENTER_MODE: {
		exp_mode = DW9786_CENTERING_MODE;
		break;
	}
	case OIS_STILL_MODE: {
		exp_mode = DW9786_STILL_MODE;
		break;
	}
	case OIS_VIDEO_MODE: {
		exp_mode = DW9786_VIDEO_MODE;
		break;
	}
	case OIS_ZOOM_MODE: {
		exp_mode = DW9786_ZOOM_MODE;
		break;
	}
	case OIS_FIX_MODE: {
		/*ret = ois_i2c_write_tele(client, DW9786_REG_OIS_CTRL, SERVO_ON);
		ret = ois_i2c_write_tele(ois->client, DW9786_REG_OIS_CL_TARGETX, 0x0000);
		I2COP_CHECK(ret);
		ret = ois_i2c_write_tele(ois->client, DW9786_REG_OIS_CL_TARGETY, 0x0000);
		I2COP_CHECK(ret);*/
		break;
	}
	case OIS_CIRCLEWAVE_MODE: {
		exp_mode = DW9786_CIRCLEWAVE_MODE;
		break;
	}
	default: {
		LOG_OIS_INF("unsupport ois mode(%d)", mode);
		goto p_err;
	}
	}

	if (old_mode != exp_mode) {
		ret = ois_i2c_write_tele(client, DW9786_OIS_MODE_SELECT, exp_mode);  //select mode
		I2COP_CHECK(ret);

		if(exp_mode == DW9786_CENTERING_MODE)
		{
			ret = ois_i2c_write_tele(client, DW9786_MODE_SWITCH_STEP, 0x66);  //10%
			I2COP_CHECK(ret);
			ret = ois_i2c_write_tele(client, DW9786_MODE_SWITCH_DELAY, 0xD); //0.005sec
			I2COP_CHECK(ret);
		}

		ret = ois_i2c_write_tele(client, DW9786_OIS_MODE_SWITCH, REG_DEAFULT_ON); //write Q-command
		I2COP_CHECK(ret);
	}

	ois->flash_info->mode = new_mode;
	ois->flash_info->serveOn = servo;

	LOG_OIS_INF("mode(old:0x%04x exp:0x%04x new:0x%04x servo:0x%04x), op status(0x%04x) result(%d)",
		old_mode, exp_mode, new_mode, servo, op_status, ret);

p_err:
	return ret;
}

static int dw9786_start_ois(struct ois *ois)
{
	int ret        = 0;
	struct i2c_client *client = ois->client;

	OIS_BUG(!client);

	/*1. servo on*/
	ret = ois_i2c_write_tele(client, DW9786_OIS_CTROL_MODE, OPERATION_MODE);// operation mode
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(client, DW9786_OIS_ACTIVE, XY_SERVO_ON); //x&y axis servo on
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(client, DW9786_AF_SERVO_ACTIVE, Z_SERVO_ON);//z axis servo on
	I2COP_CHECK(ret);
	mdelay(1);

	/*2. ois on*/
	ret = ois_i2c_write_tele(client, DW9786_OIS_CTROL_MODE, OPERATION_MODE);// ois mode
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(client, DW9786_OIS_ACTIVE, XY_OIS_ON);  //x&y axis ois on
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(client, DW9786_AF_SERVO_ACTIVE, Z_SERVO_ON);//z axis servo on
	I2COP_CHECK(ret);

	/*3. trip mode on*/
	ret = ois_i2c_write_tele(client, DW9786_REG_OIS_TRIPODE_CTRL, REG_DEAFULT_ON);
	I2COP_CHECK(ret);
	mdelay(2);

	/*4. enter still mode*/
	ret = ois_i2c_write_tele(client, DW9786_OIS_MODE_SELECT, DW9786_STILL_MODE);  //select still mode
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(client, DW9786_OIS_MODE_SWITCH, REG_DEAFULT_ON); //write Q-command
	I2COP_CHECK(ret);

	LOG_OIS_INF("ois restart success(%d)", ret);

p_err:
	return ret;
}

static int dw9786_get_init_info(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	s16 s16_rxdata = 0x0000;
	u16 u16_rxdata = 0x0000;
	struct i2c_client *client = ois->client;
	struct ois_flash_info *flash_info = ois->flash_info;

	OIS_BUG(!client);
	OIS_BUG(!flash_info);
	OIS_BUG(!user_buf);

	ois->client->addr = DW9786_SLAVE_ADDR >> 1;

	ret = ois_i2c_read_tele(client, DW9786_RELEASE_VER_H, &u16_rxdata);
	flash_info->fwInfo.version = u16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_RELEASE_VER_L, &u16_rxdata);
	flash_info->fwInfo.type = u16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_RELEASE_DATE, &u16_rxdata);
	flash_info->fwInfo.date = u16_rxdata;
	/*
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_PANTILT_CTRL, &u16_rxdata);
	flash_info->hallInfo.pantiltOn = u16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_PANTILT_DERGEEX, &u16_rxdata);
	flash_info->hallInfo.totalLimitX = u16_rxdata;
	flash_info->hallInfo.totalLimitY = u16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_IMU_SELECT, &u16_rxdata);
	flash_info->imuInfo.imuType = u16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_GYRO_READ_CTRL, &u16_rxdata);
	flash_info->imuInfo.imuReadEn = u16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_SPI_MODE, &u16_rxdata);
	flash_info->imuInfo.spiMode = u16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_MODE, &u16_rxdata);
	flash_info->mode = u16_rxdata;*/
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_TRIPODE_CTRL, &u16_rxdata);
	flash_info->tripodFlag = u16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_GYRO_OFFSETX, &s16_rxdata);
	flash_info->imuInfo.gyroOffsetX = s16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_GYRO_OFFSETY, &s16_rxdata);
	flash_info->imuInfo.gyroOffsetY = s16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_GYRO_GAINX, &s16_rxdata);
	flash_info->imuInfo.gyroGainX = s16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_GYRO_GAINY, &s16_rxdata);
	flash_info->imuInfo.gyroGainY = s16_rxdata;
	/*ret = ois_i2c_read_tele(client, DW9786_REG_OIS_ACC_GAINX, &s16_rxdata);
	flash_info->imuInfo.accGainX = s16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_ACC_GAINY, &s16_rxdata);
	flash_info->imuInfo.accGainY = s16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_SMOOTH_EN, &u16_rxdata);
	flash_info->smoothInfo.on = u16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_SMOOTH_STEP, &u16_rxdata);
	flash_info->smoothInfo.step = u16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_SMOOTH_DELAY, &u16_rxdata);
	flash_info->smoothInfo.delay = u16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_AF_DRIFT_COMP, &s16_rxdata);
	flash_info->driftCompOn = s16_rxdata;*/
	ret = ois_i2c_read_tele(client, DW9786_REG_INFO_SAMPLE_CTRL, &u16_rxdata);
	LOG_OIS_INF("len info sample ctrl 0x%04x", u16_rxdata);

	if (OFF != flash_info->imuInfo.accGainX)
		flash_info->accOn = ON;

	ret = copy_to_user(user_buf, flash_info, sizeof(struct ois_flash_info));
	if (ret) {
		LOG_OIS_ERR("fail to copy flash info, ret(%d)\n", ret);
		goto p_err;
	}

	LOG_OIS_INF("flash info:fw(ver:0x%04x date:0x%04x type:0x%04x) gyro(type:0x%04x spi:0x%04x readEn:0x%04x) pantilt(%d,limitx:%d,limity:%d) offset(%d,%d),gain(%d,%d,%d,%d) smooth(%d %d %d)",
		flash_info->fwInfo.version, flash_info->fwInfo.date, flash_info->fwInfo.type,
		flash_info->imuInfo.imuType, flash_info->imuInfo.spiMode, flash_info->imuInfo.imuReadEn,
		flash_info->hallInfo.pantiltOn, flash_info->hallInfo.totalLimitX, flash_info->hallInfo.totalLimitY,
		flash_info->imuInfo.gyroOffsetX, flash_info->imuInfo.gyroOffsetY,
		flash_info->imuInfo.gyroGainX, flash_info->imuInfo.gyroGainY, flash_info->imuInfo.accGainX, flash_info->imuInfo.accGainY,
		flash_info->smoothInfo.on, flash_info->smoothInfo.step, flash_info->smoothInfo.delay);
	LOG_OIS_INF("control info: chip(0x%04x) dsp(0x%04x) writeAthr(0x%04x) reset(0x%04x) tripod(0x%04x) mode(0x%04x) acc(0x%04x) drift(0x%04x)",
		flash_info->chipEn, flash_info->dspEn, flash_info->writeAuthority, flash_info->logicReset,
		flash_info->tripodFlag, flash_info->imuInfo.spiMode, flash_info->accOn, flash_info->driftCompOn);
p_err:
	return ret;
}

/****************************************************************************************
 * func name: dw9786_check_data_init
 * para     : ois *ois
 * brief    : check ois init data for DW9786
*****************************************************************************************
 *   Ver          Date        Auth        detail
 *   v1.0      2021/05/17
****************************************************************************************/
static int dw9786_check_data_init(struct ois *ois)
{
	int               ret     = 0;
	struct i2c_client *client = ois->client;
	u16               data    = 0;
	u16               csh     = 0;
	u16               csl     = 0;
	u32    mcs_checksum_flash = 0;

	OIS_BUG(!client);
	OIS_BUG(!(ois->flash_info));

	ois->client->addr = DW9786_SLAVE_ADDR >> 1;

	/*0. check data before ois reset*/
	ret = ois_i2c_write_tele(client, DW9786_CHIP_EN, OFF);//shutdown mode
	I2COP_CHECK(ret);
	mdelay(2);
	ret = ois_i2c_read_tele(client, DW9786_PRODUCT_ID, &data); //read product id
	if(DW9786_PRODUCT_NUM != data)
	{
		LOG_OIS_INF("DW9786 init check chip ID failed!");
	}
	ret = ois_i2c_write_tele(client, DW9786_CHIP_EN, ON);//standby mode
	I2COP_CHECK(ret);
	mdelay(5);

	ret = ois_i2c_read_tele(client, 0xED5C, &data);
	if(0 != data)
	{
		LOG_OIS_INF("DW9786 init check addr:0xED5C failed!");
	}

	ret = ois_i2c_read_tele(client, 0xED60, &csh);//read checksum
	ret = ois_i2c_read_tele(client, 0xED64, &csl);
	mcs_checksum_flash = ((u32)(csh << 16)) | csl;
	if(mcs_checksum_flash != DW9786_REF_MCS_CHECKSUM)
	{
		LOG_OIS_INF("DW9786 init check checksum failed!");
	}

	/* Check the firmware version */
	ret = ois_i2c_read_tele(client, DW9786_RELEASE_VER_L, &csh);
	ret = ois_i2c_read_tele(client, DW9786_RELEASE_DATE, &csl);
	if ((csh != DW9786_FW_VERSION) || (csl != DW9786_FW_DATE))
	{
		LOG_OIS_INF("DW9786 init check version failed!");
	}

p_err:
	return ret;
}

/****************************************************************************************
 * func name: dw9786_init
 * para     : ois *ois
 * brief    : ois init for DW9786
*****************************************************************************************
 *   Ver          Date        Auth        detail
 *   v1.0      2021/05/17
****************************************************************************************/
static int dw9786_init(struct ois *ois)
{
	int               ret     = 0;
	struct i2c_client *client = ois->client;

	OIS_BUG(!client);
	OIS_BUG(!(ois->flash_info));

	ois->client->addr = DW9786_SLAVE_ADDR >> 1;
	LOG_OIS_INF("dw9786_init!");

	/*1. servo on*/
	ret = ois_i2c_write_tele(client, DW9786_OIS_CTROL_MODE, OPERATION_MODE);// operation mode
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(client, DW9786_OIS_ACTIVE, XY_SERVO_ON); //x&y axis servo on
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(client, DW9786_AF_SERVO_ACTIVE, Z_SERVO_ON);//z axis servo on
	I2COP_CHECK(ret);
	mdelay(1);

	/*2. ois on*/
	ret = ois_i2c_write_tele(client, DW9786_OIS_CTROL_MODE, OPERATION_MODE);// ois mode
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(client, DW9786_OIS_ACTIVE, XY_OIS_ON);  //x&y axis ois on
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(client, DW9786_AF_SERVO_ACTIVE, Z_SERVO_ON);//z axis servo on
	I2COP_CHECK(ret);
	mdelay(1);

	/*3. O+E sampling*/
	ret = ois_i2c_write_tele(client, DW9786_REG_INFO_SAMPLE_CTRL, 0x1104);//write sampling time to 4ms
	I2COP_CHECK(ret);

	/*4. gyro gain choose*/
	ret = ois_i2c_write_tele(client, DW9786_REG_OIS_GAIN_CHOOSE, 0x0000);//choose main gain
	I2COP_CHECK(ret);

	/*5. trip mode on*/
	ret = ois_i2c_write_tele(client, DW9786_REG_OIS_TRIPODE_CTRL, REG_DEAFULT_ON);
	I2COP_CHECK(ret);
	LOG_OIS_INF("dw9786_init end!");
p_err:
	return ret;
}

static int dw9786_dependency_init(struct ois *ois)
{
	int               ret     = 0;
	struct i2c_client *client = ois->client;

	LOG_OIS_INF("dw9786_dependency_init!");

	OIS_BUG(!client);
	OIS_BUG(!(ois->flash_info));

	ois->client->addr = DW9786_SLAVE_ADDR >> 1;
	ois->flash_info->chipEn = 0x01;

	dw9786_check_data_init(ois);

	/*1. ois reset*/
	ret = dw9786_reset(ois);
	I2COP_CHECK(ret);

	/*2. gyro init*/
	LOG_OIS_INF("gyroInitOn start");
	mdelay(10);
	ret = ois_i2c_write_tele(client, DW9786_GYRO_INIT_CMD, 0x0001);/* SPI master*/
	I2COP_CHECK(ret);
	mdelay(90);
	ret = ois_i2c_write_tele(client, DW9786_GYRO_READ_CMD, 0x0001);/* tart To Read Gyro*/
	I2COP_CHECK(ret);
	LOG_OIS_INF("gyroInitOn end");
	mdelay(1);

	LOG_OIS_INF("dw9786_dependency_init end!");
p_err:
	return ret;
}

static int dw9786_deinit(struct ois *ois)
{
	int ret = 0;
	struct i2c_client *client = ois->client;

	LOG_OIS_INF("E");

	OIS_BUG(!client);
	/*1.disable af drift compensation*
	ret = ois_i2c_write_tele(client, DW9786_REG_OIS_AF_DRIFT_COMP, OFF);
	I2COP_CHECK(ret);
	*2.servo off*
	ret = ois_i2c_write_tele(client, DW9786_REG_OIS_CTRL, SERVO_OFF);
	I2COP_CHECK(ret);*/

	ret = ois_i2c_write_tele(client, DW9786_OIS_CTROL_MODE, OPERATION_MODE);// operation mode
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(client, DW9786_OIS_ACTIVE, OFF); //x&y axis servo off
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(client, DW9786_AF_SERVO_ACTIVE, OFF);//z axis servo off
	I2COP_CHECK(ret);

	ois->flash_info->chipEn = 0x00;
	LOG_OIS_INF("X(%d)", ret);
p_err:
	return ret;
}

static int dw9786_stream_on(struct ois *ois)
{
	int ret = 0;
	return ret;
}

static int dw9786_stream_off(struct ois *ois)
{
	int ret = 0;
	return ret;
}

static void dw9786_vsync_process_fn(struct kthread_work *work)
{
	int result = 0;
	struct ois *ois = NULL;
	struct i2c_client *client = NULL;
	u16 sample_ready = 0x0000;
	struct ois_lens_info *lens_info = NULL;
	u8 insert_idx = 0;
	u8 data[DW9786_MAX_LENS_INFO_SIZE] = {0, };
	u16 data_idx = 0, group_idx = 0;
	u16 hallCnt = 0;
	s16 timeDif = 0;
	u16 valid_size = 0;
	struct timespec start_time, end_time;

	OIS_BUG_VOID(!work);


	LOG_OIS_VERB("ois vsync processor E");


	ois = container_of(work, struct ois, vsync_work);
	OIS_BUG_VOID(!ois);
	client = ois->client;
	OIS_BUG_VOID(!client);
	OIS_BUG_VOID(!work);
	OIS_BUG_VOID(!(ois->lens_info_buf));

	insert_idx = ois->lens_info_buf->insertidx;
	lens_info = &ois->lens_info_buf->buf[insert_idx];

	mdelay(1);

	ktime_get_ts(&start_time);

	mutex_lock(&ois->op_lock);
	memset(lens_info, 0x00, sizeof(struct ois_lens_info));
	//check if data ready and read first packet(62 bytes)
	result = ois_i2c_read_tele(client, DW9786_DATA_READY, &sample_ready);
	result = ois_i2c_read_tele(client, DW9786_REG_LENS_INFO_START, (u16 *)(&data[0]));
	hallCnt = data[0] & 0xFF;
	valid_size = hallCnt * 6 + 2;
	if (0 == (sample_ready & 0x01) || !data[0] || valid_size > DW9786_MAX_LENS_INFO_SIZE) {
		LOG_OIS_INF("skip: sample_ready=%d valid size=%d", sample_ready, valid_size);
		goto p_err;
	}
	lens_info->validnum = (hallCnt > LENS_INFO_GROUPS_MAX) ? LENS_INFO_GROUPS_MAX : hallCnt;
	lens_info->fcount = ois->vsync_info.ois_vsync_cnt;

	//if there are more data
	while (data_idx < valid_size - 1) {
		result = ois_i2c_read_block_tele(client, DW9786_REG_LENS_INFO_START, &data[data_idx], DW9786_LENS_PACKET_SIZE);
		result = ois_i2c_write_tele(ois->client, DW9786_DATA_READY, 0x0100);//inform ic one packet read done
		I2COP_CHECK(result);
		mdelay(1);
		data_idx += DW9786_LENS_PACKET_SIZE;
	}

	//LOG_OIS_INF("read done(idx=%d)", data_idx);

	for (group_idx = 0, data_idx = 2; group_idx < lens_info->validnum; group_idx++) {
		lens_info->ic_timecount[group_idx] = (!group_idx) ? (100 * (OIS_BIG_ENDIAN_TRANS2(data, data_idx))) :
			(OIS_BIG_ENDIAN_TRANS2(data, data_idx));//onvert to us
		data_idx += 2;
		lens_info->hallx[group_idx] = (s16)(OIS_BIG_ENDIAN_TRANS2(data, data_idx));
		data_idx += 2;
		lens_info->hally[group_idx] = (s16)(OIS_BIG_ENDIAN_TRANS2(data, data_idx));
		data_idx += 2;
		LOG_OIS_INF("fcount %lu data[%d] timestamp %lu hallx %d hally %d",
			lens_info->fcount, group_idx,  lens_info->ic_timecount[group_idx],
			lens_info->hallx[group_idx], lens_info->hally[group_idx]);
	}

	timeDif = (s16)(OIS_BIG_ENDIAN_TRANS2(data, 2));
	lens_info->timestampboot = ois->vsync_info.sof_timestamp_boot  + timeDif * 100000;
	LOG_OIS_VERB("info ready=0x%04x, isp vsync %llu fcount=%llu ts=%llu validnum=%d insert=%d timeDif=%d",
		sample_ready, ois->vsync_info.vsync_cnt, lens_info->fcount, lens_info->timestampboot, lens_info->validnum, insert_idx, timeDif);
	ois->vsync_info.ois_vsync_cnt++;

	if (++ois->lens_info_buf->insertidx == LENS_INFO_FRAMES_MAX)
			ois->lens_info_buf->insertidx = 0;

	ktime_get_ts(&end_time);

	LOG_OIS_VERB("ois vsync processor X %llu ms",
		(((u64)end_time.tv_sec * 1000 + end_time.tv_nsec / 1000000) - ((u64)start_time.tv_sec * 1000 + start_time.tv_nsec / 1000000)));
p_err:
	mutex_unlock(&ois->op_lock);
	return;
}

static int dw9786_vsync_signal(struct ois *ois, void *buf)
{
	int ret = 0;
	struct timespec curtime;
	struct ois_vsync_info *tempInfo = NULL;
	u64 module_idx = 0, vsync_cnt = 0;

	OIS_BUG(!buf);
	OIS_BUG(!(ois->lens_info_buf));

	tempInfo = (struct ois_vsync_info *)buf;
	module_idx = tempInfo->module_idx;
	vsync_cnt = tempInfo->vsync_cnt;

	if (OIS_VSYNC_TYPE_INT_CAM_A_ST != module_idx && OIS_VSYNC_TYPE_INT_CAM_B_ST != module_idx) {
		LOG_OIS_INF("module %lu skip vsync %lu process", module_idx, vsync_cnt);
		return ret;
	}
	curtime = ktime_to_timespec(tempInfo->sof_timestamp_boot);
	ois->vsync_info.sof_timestamp_boot = (u64)curtime.tv_sec * 1000000000 + curtime.tv_nsec;
	ois->vsync_info.vsync_cnt = vsync_cnt;
	ois->vsync_info.module_idx = module_idx;

	if (ois->vsync_task != NULL) {
		kthread_queue_work(&ois->vsync_worker, &ois->vsync_work);
	}

	LOG_OIS_INF("module %lu isp vsync %llu timestamp %llu", module_idx, vsync_cnt, ois->vsync_info.sof_timestamp_boot);

	return ret;
}

static int dw9786_init_vsync_thread(struct ois *ois)
{
	int ret = 0;
	struct sched_param param = { .sched_priority =  MAX_RT_PRIO - 50 };

	if (NULL == ois->vsync_task) {
		spin_lock_init(&ois->ois_vsync_lock);
		kthread_init_work(&ois->vsync_work, dw9786_vsync_process_fn);
		kthread_init_worker(&ois->vsync_worker);
		ois->vsync_task = kthread_run(kthread_worker_fn, &ois->vsync_worker, "vsync_processor");
		if (IS_ERR_OR_NULL(ois->vsync_task)) {
			ret = PTR_ERR(ois->vsync_task);
			LOG_OIS_ERR("failed to create vsync processor, err(%ld)", PTR_ERR(ois->vsync_task));
			ois->vsync_task = NULL;
			goto p_err;
		}
		//set vsync thread priority
		ret = sched_setscheduler_nocheck(ois->vsync_task, SCHED_FIFO, &param);
		if (ret) {
			LOG_OIS_ERR("ois priority set fail(%d)", ret);
			goto p_err;
		}
		LOG_OIS_INF("start ois vsync processor done(%p)", ois->vsync_task);
	} else {
		LOG_OIS_INF("ois vsync processor already start");
	}
p_err:
	return ret;
}

static int dw9786_deinit_vsync_thread(struct ois *ois)
{
	int ret = 0;

	if (!IS_ERR_OR_NULL(ois->vsync_task)) {
		LOG_OIS_INF("stop ois vsync start(%p)", ois->vsync_task);
		ret = kthread_stop(ois->vsync_task);
		if (ret) {
			LOG_OIS_ERR("vsync processor stop fail");
		}
		ois->vsync_task = NULL;
		LOG_OIS_INF("stop ois vsync done");
	} else {
		LOG_OIS_INF("ois vsync already done(%p)", ois->vsync_task);
	}
	return ret;
}

static int dw9786_get_lens_info(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	u8  idx = 0;
	struct timespec start_time, end_time;
	//struct ois_lens_info_buf info_buf = {0};

	OIS_BUG(!user_buf);

	//get hal frame id
	//ret = copy_from_user(&info_buf, user_buf, sizeof(struct ois_lens_info_buf));
	//LOG_OIS_INF("read hal frame %llu", info_buf.buf[0].frame_id);

	ktime_get_ts(&start_time);

	mutex_lock(&ois->op_lock);
	ret = copy_to_user(user_buf, ois->lens_info_buf, sizeof(struct ois_lens_info_buf));
	if (ret) {
		LOG_OIS_ERR("fail to copy lens info, ret(%d)\n", ret);
	}

	ktime_get_ts(&end_time);

	for (idx = 0; idx < LENS_INFO_FRAMES_MAX; idx++)
		LOG_OIS_VERB("fcount %lu data[0] timestamp %lu hallx %d hally %d",
			ois->lens_info_buf->buf[idx].fcount, ois->lens_info_buf->buf[idx].ic_timecount[0],
			ois->lens_info_buf->buf[idx].hallx[0], ois->lens_info_buf->buf[idx].hally[0]);

	LOG_OIS_INF("lens info copy %d done %d(%llums)", ois->lens_info_buf->buf[idx].fcount, ret,
		(((u64)end_time.tv_sec * 1000 + end_time.tv_nsec / 1000000) - ((u64)start_time.tv_sec * 1000 + start_time.tv_nsec / 1000000)));

	mutex_unlock(&ois->op_lock);
	return ret;
}

static int dw9786_get_fw_version(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	struct ois_flash_info *flash_info = ois->flash_info;

	OIS_BUG(!flash_info);
	OIS_BUG(!user_buf);

	ret = copy_to_user(user_buf, &flash_info->fwInfo.version, sizeof(int));
	if (ret) {
		LOG_OIS_ERR("copy_to_user fail(%d)", ret);
	}

	LOG_OIS_INF("fw ver 0x%08x", flash_info->fwInfo.version);

	return ret;
}

static int dw9786_get_gyro_offset(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	struct ois_flash_info *flash_info = ois->flash_info;
	struct i2c_client *client = ois->client;
	s16 rxdata = 0x0000;

	OIS_BUG(!flash_info);
	OIS_BUG(!user_buf);
	OIS_BUG(!client);

	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_GYRO_OFFSETX, &rxdata);
	flash_info->imuInfo.gyroOffsetX = rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_GYRO_OFFSETY, &rxdata);
	flash_info->imuInfo.gyroOffsetY = rxdata;

	ret = copy_to_user(user_buf, &flash_info->imuInfo.gyroOffsetX, (2 * sizeof(int)));
	if (ret) {
		LOG_OIS_ERR("copy_to_user fail(%d)", ret);
	}

	LOG_OIS_INF("offset(%d, %d)", flash_info->imuInfo.gyroOffsetX, flash_info->imuInfo.gyroOffsetY);
	return ret;
}

static int dw9786_get_gyro_gain(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	struct ois_flash_info *flash_info = ois->flash_info;
	struct i2c_client *client = ois->client;
	s16 rxdata = 0x0000;

	OIS_BUG(!flash_info);
	OIS_BUG(!user_buf);
	OIS_BUG(!client);

	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_GYRO_GAINX, &rxdata);
	flash_info->imuInfo.gyroGainX = rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_GYRO_GAINY, &rxdata);
	flash_info->imuInfo.gyroGainY = rxdata;

	ret = copy_to_user(user_buf, &flash_info->imuInfo.gyroGainX, (2 * sizeof(int)));
	if (ret) {
		LOG_OIS_ERR("copy_to_user fail(%d)", ret);
	}

	LOG_OIS_INF("gyro gain(0x%08x, 0x%08x)", flash_info->imuInfo.gyroGainX, flash_info->imuInfo.gyroGainY);
	return ret;
}

static int dw9786_get_mode(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	/*struct ois_flash_info *flash_info = ois->flash_info;
	struct i2c_client *client = ois->client;
	u16 rxdata = 0x0000;

	OIS_BUG(!flash_info);
	OIS_BUG(!user_buf);
	OIS_BUG(!client);

	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_MODE, &rxdata);
	flash_info->mode = rxdata;

	ret = copy_to_user(user_buf, &flash_info->mode, sizeof(int));
	if (ret) {
		LOG_OIS_ERR("copy_to_user fail(%d)", ret);
	}

	LOG_OIS_INF("ois mode(0x%x)", flash_info->mode);
*/
	return ret;
}

static int dw9786_set_smooth(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	/*struct i2c_client *client = ois->client;
	struct ois_smooth_info smooth = { 0, };
	u16 on = 0x0000;

	OIS_BUG(!client);

	ret = copy_from_user(&smooth, user_buf, sizeof(struct ois_smooth_info));
	if (ret) {
		LOG_OIS_ERR("copy_from_user fail(%d)", ret);
		goto p_err;
	}

	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_SMOOTH_EN, &on);
	if (smooth.on != on) {
		ret = ois_i2c_write_tele(client, DW9786_REG_OIS_SMOOTH_EN, REG_DEAFULT_ON);
		I2COP_CHECK(ret);
		ret = ois_i2c_write_tele(client, DW9786_REG_OIS_SMOOTH_STEP, smooth.step);
		I2COP_CHECK(ret);
		ret = ois_i2c_write_tele(client, DW9786_REG_OIS_SMOOTH_DELAY, smooth.delay);
		I2COP_CHECK(ret);
	}
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_SMOOTH_EN, &on);

	LOG_OIS_INF("smooth(%d %d %d) read(%d)", smooth.on, smooth.step, smooth.delay, on);
p_err:*/
	return ret;
}

static int dw9786_flash_save(struct ois *ois)
{
	int ret = 0;
	u16 ois_status = 0x0000;
	struct i2c_client *client = ois->client;

	OIS_BUG(!client);

	//save info
	 /*store*/
	client->addr = DW9786_MEM_ADDR >> 1;
	ret = ois_i2c_write_tele(client, DW9786_USER_PROTECT_CTRL, 0x00EA);
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(client, DW9786_MEM_AREA, 0x0000);
	I2COP_CHECK(ret);
	mdelay(10);

	client->addr = DW9786_SLAVE_ADDR >> 1;
	ret = ois_i2c_write_tele(client, DW9786_OIS_CTROL_MODE, 0x000A);
	I2COP_CHECK(ret);
	ret = ois_i2c_read_tele(client, DW9786_OIS_STATUS, &ois_status);
	LOG_OIS_INF("Read B020 status: %d", ois_status);
	ret = ois_i2c_write_tele(client, DW9786_USER_PT_A, 0xA23F);
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(client, DW9786_OIS_ACTIVE, 0x0001);
	I2COP_CHECK(ret);
	mdelay(100);
	ret = ois_i2c_read_tele(client, DW9786_OIS_STATUS, &ois_status);
	LOG_OIS_INF("Read B020 status: %d", ois_status);

	//reset ois
	ret = dw9786_reset(ois);
	I2COP_CHECK(ret);

	//restart ois
	ret = dw9786_start_ois(ois);

	LOG_OIS_INF("ois flash save success(%d)", ret);
p_err:
	return ret;
}

static int dw9786_set_gyro_gain(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	struct i2c_client *client = ois->client;
	int gain_param[3] =  { 0, };
	u16 tx_data = 0x0000;
	s16 rx_data = 0x0000;
	u16 addr    = 0x0000;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = copy_from_user(gain_param, user_buf, sizeof(gain_param));
	if (ret) {
		LOG_OIS_ERR("copy gain fail(%d)", ret);
		goto p_err;
	}

	if (gain_param[0] > 0) {
		addr = DW9786_REG_OIS_GYRO_GAINY;
		tx_data = (u16)(gain_param[2]);
	} else {
		addr = DW9786_REG_OIS_GYRO_GAINX;
		tx_data = (u16)(gain_param[1]);
	}

	ret = ois_i2c_write_tele(client, addr, tx_data);
	I2COP_CHECK(ret);
	ret = ois_i2c_read_tele(client, addr, &rx_data);

	LOG_OIS_INF("gain param(%d %d %d), set(addr=0x%04x value=%d) read %d(0x%04x)",
		gain_param[0], gain_param[1], gain_param[2], addr, tx_data, rx_data, rx_data);

p_err:
	return ret;
}

static int dw9786_set_offset_calibration(struct ois *ois)
{
	int ret = 0;
	u8 rdCnt = 0;
	u16 offsetx = 0x0000;
	u16 offsety = 0x0000;
	u16 ois_status = 0x0000;
	u16 afCode = 0;
	struct i2c_client *client = ois->client;
	struct ois_flash_info *flash_info = ois->flash_info;

	ois->client->addr = DW9786_SLAVE_ADDR >> 1;

	LOG_OIS_INF("dw9786_set_offset_calibration start update0528!!!");

	OIS_BUG(!client);
	OIS_BUG(!flash_info);

	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_GYRO_OFFSETX, &offsetx);
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_GYRO_OFFSETY, &offsety);
	ret = ois_i2c_read_tele(client, DW9786_AF_CODE, &afCode);
	LOG_OIS_INF("before cal offset(0x%04x 0x%04x) afCode(%d) ret(%d)", offsetx, offsety, afCode, ret);

	mutex_lock(&ois->op_lock);
	ret = ois_i2c_write_tele(client, DW9786_OIS_CTROL_MODE, 0x0006);
	I2COP_CHECK(ret);
	ret = ois_i2c_read_tele(client, DW9786_OIS_STATUS, &ois_status);
	LOG_OIS_INF("Read B020 status: %d", ois_status);
	ret = ois_i2c_write_tele(client, DW9786_OIS_ACTIVE, 0x0001);
	I2COP_CHECK(ret);
	mdelay(5);

	do {
		ret = ois_i2c_read_tele(client, DW9786_OIS_STATUS, &ois_status);
		rdCnt++;
		mdelay(50);
	} while ((ois_status != 0x6001) && (rdCnt < 20));

	ret = ois_i2c_read_tele(client, DW9786_OIS_STATUS, &ois_status);
	LOG_OIS_INF("Read B020 status: %d", ois_status);
	ret = ois_i2c_read_tele(client, DW9786_GYRO_OFFSET_CAL_STATUS, &ois_status);
	LOG_OIS_INF("Read gyro offset cal status: %d", ois_status);
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_GYRO_OFFSETX, &offsetx);
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_GYRO_OFFSETY, &offsety);
	LOG_OIS_INF("after cal offset(0x%04x 0x%04x) ret(%d)", offsetx, offsety, ret);

	/*store*/
	ois->client->addr = DW9786_MEM_ADDR >> 1;
	ret = ois_i2c_write_tele(client, DW9786_USER_PROTECT_CTRL, 0x00EA);
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(client, DW9786_MEM_AREA, 0x0000);
	I2COP_CHECK(ret);
	mdelay(10);

	ois->client->addr = DW9786_SLAVE_ADDR >> 1;
	ret = ois_i2c_write_tele(client, DW9786_OIS_CTROL_MODE, 0x000A);
	I2COP_CHECK(ret);
	ret = ois_i2c_read_tele(client, DW9786_OIS_STATUS, &ois_status);
	LOG_OIS_INF("Read B020 status: %d", ois_status);
	ret = ois_i2c_read_tele(client, DW9786_GYRO_OFFSET_CAL_STATUS, &ois_status);
	LOG_OIS_INF("Read gyro offset cal status: %d", ois_status);
	ret = ois_i2c_write_tele(client, DW9786_USER_PT_A, 0xA23F);
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(client, DW9786_OIS_ACTIVE, 0x0001);
	I2COP_CHECK(ret);
	mdelay(100);
	ret = ois_i2c_read_tele(client, DW9786_OIS_STATUS, &ois_status);
	LOG_OIS_INF("Read B020 status: %d", ois_status);
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_GYRO_OFFSETX, &offsetx);
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_GYRO_OFFSETY, &offsety);
	LOG_OIS_INF("check again cal offset(0x%04x 0x%04x) ret(%d)", offsetx, offsety, ret);

	/*set ois on after calibration*/
	dw9786_reset(ois);
	/*servo on*/
	/*ret = ois_i2c_write_tele(client, DW9786_OIS_CTROL_MODE, OPERATION_MODE);// operation mode
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(client, DW9786_OIS_ACTIVE, XY_SERVO_ON); //x&y axis servo on
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(client, DW9786_AF_SERVO_ACTIVE, Z_SERVO_ON);//z axis servo on
	I2COP_CHECK(ret);
	mdelay(1);*/

	/*ois on*/
	ret = ois_i2c_write_tele(client, DW9786_OIS_CTROL_MODE, OPERATION_MODE);// ois mode
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(client, DW9786_OIS_ACTIVE, XY_OIS_ON);  //x&y axis ois on
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(client, DW9786_AF_SERVO_ACTIVE, Z_SERVO_ON);//z axis servo on
	I2COP_CHECK(ret);
	mdelay(1);

	/*gyro init*/
	ret = ois_i2c_write_tele(client, DW9786_GYRO_INIT_CMD, ON);  //gyro init
	I2COP_CHECK(ret);
	mdelay(90);
	ret = ois_i2c_write_tele(client, DW9786_GYRO_READ_CMD, ON);  //gyro read start
	I2COP_CHECK(ret);
	mdelay(2);

	ret = ois_i2c_write_tele(client, DW9786_AF_CODE, afCode);
	I2COP_CHECK(ret);
	ret = ois_i2c_read_tele(client, DW9786_AF_CODE, &afCode);
	LOG_OIS_INF("after cal offset afCode(%d) ret(%d)", afCode, ret);

p_err:
	mutex_unlock(&ois->op_lock);
	return ret;
}

static int dw9786_set_acc(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	return ret;
}

static int dw9786_af_crosstalk_compensation(struct ois *ois, struct ois_af_drift_param *drift)
{
	int ret = 0;
	/*u16 tx_data = 0x0000;
	struct i2c_client *client = ois->client;
	s16 drift_on = 0x0000, af_position = 0x0000;

	OIS_BUG(!client);
	OIS_BUG(!drift);
	OIS_BUG(!(ois->flash_info));
	OIS_BUG(!(ois->flash_info->chipEn));

	LOG_OIS_VERB("drift info(%d %d %d)", drift->driftCompOn, drift->currFocusDac, drift->preFocusDac);

	if (-1 == drift->driftCompOn) {//hard code -1 for normal af code update case
		ret = ois_i2c_read_tele(client, DW9786_REG_OIS_AF_DRIFT_COMP, &drift_on);
		if (OFF == drift_on) {
			LOG_OIS_INF("drift set skip", drift_on);
			goto p_err;
		}
	} else {
		tx_data = (drift->driftCompOn == 1) ? (REG_DEAFULT_ON) : (OFF);
		ret = ois_i2c_write_tele(ois->client, DW9786_REG_OIS_AF_DRIFT_COMP, tx_data);
		I2COP_CHECK(ret);
		LOG_OIS_INF("drift status change(0x%04x)", drift_on);
		goto p_err;
	}

	tx_data = (u16)drift->currFocusDac;

	ret = ois_i2c_write_tele(ois->client, DW9786_REG_AF_POSITION, tx_data);
	I2COP_CHECK(ret);

	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_AF_DRIFT_COMP, &drift_on);
	ret = ois_i2c_read_tele(client, DW9786_REG_AF_POSITION, &af_position);

	LOG_OIS_VERB("drift set(%d) read(0x%02x, %d) ret(%d)", tx_data, drift_on, af_position, ret);

p_err:*/
	return ret;
}

static int dw9786_set_tripod(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	struct i2c_client *client = ois->client;
	int on = 0x0000;
	u16 rx_data = 0x0000;
	u16 tx_data = 0x0000;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = copy_from_user(&on, user_buf, sizeof(int));
	if (ret) {
		LOG_OIS_ERR("copy tripod fail(%d)", ret);
		goto p_err;
	}

	tx_data = (on) ? (REG_DEAFULT_ON) : (OFF);

	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_TRIPODE_CTRL, &rx_data);
	if (tx_data == rx_data) {
		LOG_OIS_INF("config already done(0x%04x)", rx_data);
		goto p_err;
	}

	ret = ois_i2c_write_tele(ois->client, DW9786_REG_OIS_TRIPODE_CTRL, tx_data);
	I2COP_CHECK(ret);
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_TRIPODE_CTRL, &rx_data);

	LOG_OIS_INF("tripod set(%d 0x%04x) read(0x%04x)", on, tx_data, rx_data);

p_err:
	return ret;
}

static int dw9786_set_sinewave(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	/*struct i2c_client *client = ois->client;
	struct ois_sinemode_parameter sine = { 0, };
	u16 mode = 0x0000;
	u16 amp = 0x0000;
	u16 frequency = 0x0000;
	u16 cmd_status = 0x0000;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = copy_from_user(&sine, user_buf, sizeof(struct ois_sinemode_parameter));
	if (ret) {
		LOG_OIS_ERR("copy sine params fail(%d)", ret);
		goto p_err;
	}

	LOG_OIS_INF("sine set(%d %d %d %d)", sine.axis, sine.frequency, sine.amplitudeX, sine.amplitudeY);

	if (1 == sine.axis)
		mode = 0x8005;
	else if (2 == sine.axis)
		mode = 0x8004;
	else {
		LOG_OIS_INF("invalid axis(%d)", sine.axis);
		goto p_err;
	}

	if (sine.amplitudeX > -1 && sine.amplitudeX <= 100)
		amp = (u16)(0xFFFF * sine.amplitudeX / 200);
	else {
		LOG_OIS_INF("invalid amplitude(%d)", sine.amplitudeX);
		goto p_err;
	}

	frequency = (u16)sine.frequency;

	ret = ois_i2c_write_tele(ois->client, DW9786_REG_OIS_MODE, mode);
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(ois->client, DW9786_REG_OIS_SINEWAVE_AMP, amp);
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(ois->client, DW9786_REG_OIS_SINEWAVE_FREQ, frequency);
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(ois->client, DW9786_REG_OIS_CMD_STATUS, REG_DEAFULT_ON);
	I2COP_CHECK(ret);

	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_CMD_STATUS, &cmd_status);
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_SINEWAVE_AMP, &amp);
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_SINEWAVE_FREQ, &frequency);
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_MODE, &mode);

	LOG_OIS_INF("sine read(0x%04x 0x%04x) mode(0x%04x) cmd(0x%04x)", frequency, amp, mode, cmd_status);

p_err:*/
	return ret;
}

static int dw9786_set_circlewave(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	/*struct i2c_client *client = ois->client;
	struct ois_circlemode_parameter circle = { 0, };
	u16 mode = 0x0000;
	u16 amp = 0x0000;
	u16 frequency = 0x0000;
	u16 cmd_status = 0x0000;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = copy_from_user(&circle, user_buf, sizeof(struct ois_circlemode_parameter));
	if (ret) {
		LOG_OIS_ERR("copy circle params fail(%d)", ret);
		goto p_err;
	}

	LOG_OIS_INF("circle set(%d %d %d)", circle.frequency, circle.amplitudeX, circle.amplitudeY);

	if (circle.amplitudeX > -1 && circle.amplitudeX <= 100)
		amp = (u16)(0xFFFF * circle.amplitudeX / 200);
	else {
		LOG_OIS_INF("invalid amplitude(%d)", circle.amplitudeX);
		goto p_err;
	}

	frequency = (u16)circle.frequency;

	ret = ois_i2c_write_tele(ois->client, DW9786_REG_OIS_MODE, DW9786_CIRCLEWAVE_MODE);
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(ois->client, DW9786_REG_OIS_SINEWAVE_AMP, amp);
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(ois->client, DW9786_REG_OIS_SINEWAVE_FREQ, frequency);
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(ois->client, DW9786_REG_OIS_CMD_STATUS, REG_DEAFULT_ON);
	I2COP_CHECK(ret);

	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_CMD_STATUS, &cmd_status);
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_SINEWAVE_AMP, &amp);
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_SINEWAVE_FREQ, &frequency);
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_MODE, &mode);

	LOG_OIS_INF("circle read(0x%04x 0x%04x) mode(0x%04x) cmd(0x%04x)", frequency, amp, mode, cmd_status);

p_err:*/
	return ret;
}

static int dw9786_set_target(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	/*u16 servo = 0x0000;
	struct i2c_client     *client    = ois->client;
	struct ois_fixmode_parameter fixmode = { 0, };
	int min_code = 0, max_code = 0;
	s16 hallx = 0x0000, hally = 0x0000;
	s16 targetx = 0x0000, targety = 0x0000;
	u16 ampX = 0x0000, ampY = 0x0000;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_CTRL, &servo);
	if (SERVO_ON != servo) {
		ret = ois_i2c_write_tele(ois->client, DW9786_REG_OIS_CTRL, SERVO_ON);
		I2COP_CHECK(ret);
		LOG_OIS_INF("switch to servo on");
	}

	ret = copy_from_user(&fixmode, user_buf, sizeof(struct ois_fixmode_parameter));
	if (ret) {
		LOG_OIS_ERR("copy target fail(%d)", ret);
		goto p_err;
	}

	LOG_OIS_INF("target set(%d %d)", fixmode.targetX, fixmode.targetY);

	ampX = ois->flash_info->hallInfo.totalLimitX;
	ampY = ois->flash_info->hallInfo.totalLimitY;
	targetx = (s16)(ampX * fixmode.targetX / 100);
	targety = (s16)(ampY * fixmode.targetY / 100);

	if (abs(targetx) <= ampX && abs(targety) <= ampY) {
		ret = ois_i2c_write_tele(ois->client, DW9786_REG_OIS_CL_TARGETX, targetx);
		I2COP_CHECK(ret);
		ret = ois_i2c_write_tele(ois->client, DW9786_REG_OIS_CL_TARGETY, targety);
		I2COP_CHECK(ret);
	} else {
		LOG_OIS_INF("invalid target(%d, %d)", targetx, targety);
		goto p_err;
	}

	max_code = abs(ois->flash_info->hallInfo.totalLimitX);
	min_code = 0 - max_code;

	mdelay(3);

	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_CL_TARGETX, &targetx);
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_CL_TARGETY, &targety);
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_LENS_POSX, &hallx);
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_LENS_POSY, &hally);

	LOG_OIS_INF("hall read target(0x%04x 0x%04x) hall(0x%04x 0x%04x) amp(0x%04x 0x%04x)",
		targetx, targety, hallx, hally, ampX, ampY);

p_err:*/
	return ret;
}

static int dw9786_set_servo(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	int servo_param = 0;
	struct i2c_client *client    = ois->client;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = copy_from_user(&servo_param, user_buf, sizeof(int));
	if (ret) {
		LOG_OIS_ERR("copy servo info fail(%d)", ret);
		goto p_err;
	}

	LOG_OIS_INF("chip set 0x%04x", servo_param);

	ret = ois_i2c_write_tele(ois->client, DW9786_OIS_CTROL_MODE, OPERATION_MODE);//operate mode
	I2COP_CHECK(ret);
	if (!servo_param) {
		ret = ois_i2c_write_tele(ois->client, DW9786_OIS_ACTIVE, 0x0);//servo off
		I2COP_CHECK(ret);
		ret = ois_i2c_write_tele(ois->client, DW9786_AF_SERVO_ACTIVE, 0x0);//af servo off
		I2COP_CHECK(ret);
	} else {
		ret = ois_i2c_write_tele(ois->client, DW9786_OIS_ACTIVE, 0x2);//servo on
		I2COP_CHECK(ret);
		ret = ois_i2c_write_tele(ois->client, DW9786_AF_SERVO_ACTIVE, 0x1);//af servo on
		I2COP_CHECK(ret);
	}
	mdelay(1);

p_err:
	return ret;
}

static int dw9786_set_pantilt(struct ois *ois, __user void *user_buf)
{
	int ret             = 0;
	/*u16 pantilt_on      = 0x0000;
	u16 pantilt_degreeX = 0x0000;
	u16 pantilt_degreeY = 0x0000;
	struct i2c_client     *client    = ois->client;
	struct ois_pantilt_param pantilt = { 0, };

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = copy_from_user(&pantilt, user_buf, sizeof(struct ois_pantilt_param));
	if (ret) {
		LOG_OIS_ERR("copy pantilt fail(%d)", ret);
		goto p_err;
	}

	if (ON == pantilt.on) {
		ret = ois_i2c_write_tele(ois->client, DW9786_REG_OIS_PANTILT_CTRL, ON);
		I2COP_CHECK(ret);
		ret = ois_i2c_write_tele(ois->client, DW9786_REG_OIS_PANTILT_DERGEEX, pantilt.limitX);
		I2COP_CHECK(ret);
	} else if (OFF == pantilt.on) {
		ret = ois_i2c_write_tele(ois->client, DW9786_REG_OIS_PANTILT_CTRL, OFF);
		I2COP_CHECK(ret);
	}

	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_PANTILT_CTRL, &pantilt_on);
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_PANTILT_DERGEEX, &pantilt_degreeX);
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_PANTILT_DERGEEY, &pantilt_degreeY);

	LOG_OIS_INF("pantilt info(%d %d %d) read(%d %d %d) ret(%d)",
		pantilt.on, pantilt.limitX, pantilt.limitY,
		pantilt_on, pantilt_degreeX, pantilt_degreeY, ret);
p_err:*/
	return ret;
}


static int dw9786_flash_acess(struct i2c_client     *client, int type)
{
	int ret = 0;

	ret = ois_i2c_write_tele(client, DW9786_CHIP_EN, ON);
	I2COP_CHECK(ret);
	ret = ois_i2c_write_tele(client, DW9786_MCU_ACTIVE, OFF);
	I2COP_CHECK(ret);
	if (type == DW9786_MCS_LUT) {
		ret = ois_i2c_write_tele(client, DW9786_MCS_LUT_ADDR, 0xDB01);
		I2COP_CHECK(ret);
	} else if (type == DW9786_MCS_CODE) {
		ret = ois_i2c_write_tele(client, DW9786_CODE_PT_RELEASE, 0xC0DE);
		I2COP_CHECK(ret);
	} else if (type == DW9786_MCS_USER) {
		ret = ois_i2c_write_tele(client, DW9786_USER_PT_A, 0xA23F);
		I2COP_CHECK(ret);
	}
	ret = ois_i2c_write_tele(client, DW9786_FLASH_MEM_SEL, 0x0000);
	I2COP_CHECK(ret);
	mdelay(1);
p_err:
	return ret;
}

static int dw9786_regdata_check(struct i2c_client *client, u16 reg, u16 ref)
{
	/*
	reg : read target register
	ref : compare reference data
	*/
	u16 r_data;
	int ret = 0;
	int i = 0;
	for (i = 0; i < 5; i++) {
		ret = ois_i2c_read_tele(client, reg, &r_data); //Read status
		if (r_data == ref) {
			LOG_OIS_INF("[dw9786_regdata_check]Reg data check ok!");
			ret = 1;
			break;
		}
		mdelay(6);
	}
	return ret;
}

static int dw9786_checksum_func(struct i2c_client *client, int type)
{
	/*
	Error code definition
	0 : No Error
	-1 : CHECKSUM_ERROR
	*/
	u16 csh, csl;
	u32 mcs_checksum_flash;
	int ret = 0;
	//logi("[dw9786_checksum_func] start....");
	/* Set the checksum area */
	if (type == DW9786_MCS_LUT) {
		/* not used */
	} else if (type == DW9786_MCS_CODE) {
		dw9786_flash_acess(client, DW9786_MCS_CODE);
		ret = ois_i2c_write_tele(client, 0xED48, DW9786_MCS_START_ADDRESS);
		I2COP_CHECK(ret);
		ret = ois_i2c_write_tele(client, 0xED4C, DW9786_MCS_CHECKSUM_SIZE);
		I2COP_CHECK(ret);
		ret = ois_i2c_write_tele(client, 0xED50, 0x0001);
		I2COP_CHECK(ret);
	}
	dw9786_regdata_check(client, 0xED04, 0x00);
	ret = ois_i2c_read_tele(client, 0xED54, &csh);
	ret = ois_i2c_read_tele(client, 0xED58, &csl);
	mcs_checksum_flash = ((unsigned int)(csh << 16)) | csl;
	LOG_OIS_INF("[dw9786_checksum_func]flash memory checksum , [0x%08X]", mcs_checksum_flash);
p_err:
	return mcs_checksum_flash;
}

static int dw9786_chip_enable(struct i2c_client *client, unsigned short en)
{
	int ret = 0;
	if (en == 0) {
		LOG_OIS_INF("[dw9786_chip_enable]dw9786_sleep");
	} else if (en == 1) {
		LOG_OIS_INF("[dw9786_chip_enable]dw9786_standby");
	}
	ret = ois_i2c_write_tele(client, DW9786_CHIP_EN, en);
	I2COP_CHECK(ret);
p_err:
	return ret;
}

/*write fw to flash*/
static int dw9786_fw_download(struct ois *ois)
{
	int                   ret              = 0;
	int                   i                = 0;
	u16                   idx              = 0;
	u16                   update_flag      = 0;
	const struct firmware *ois_fw          = NULL;
	const u8              *fw_write_data   = NULL;
	u16                   fw_size          = 0;
	u16                   addr             = 0;
	size_t                unit_size        = sizeof(u16)/sizeof(u8);
	u8                    *fw_read_data    = NULL;
	unsigned int          checksum_criteria, checksum_flash = 0;
	struct i2c_client     *client          = ois->client;

	OIS_BUG(!client);
	OIS_BUG(!(ois->dev));

	LOG_OIS_INF("[dw9786_fw_download] start....");
	/*MCS start -----------------------------------------------------------------------------*/
	dw9786_flash_acess(client, DW9786_MCS_CODE);
	for (i = 0; i < DW9786_FMC_PAGE; i++) {
		if (!i)
			addr = DW9786_MCS_START_ADDRESS;
		ret = ois_i2c_write_tele(client, 0xED08, addr);  // Set erase address
		I2COP_CHECK(ret);
		ret = ois_i2c_write_tele(client, 0xED0C, 0x0002);  // Sector Erase(2KB)
		I2COP_CHECK(ret);
		addr += 0x800;
		mdelay(5);
	}

	//1.load fw to kernel space
	ret = request_firmware(&ois_fw, DW9786_OIS_FW_NAME, ois->dev);
	if (ret) {
		LOG_OIS_ERR("load fw failed(%d)", ret);
		goto p_err;
	}
	fw_write_data = (u8 *)(kzalloc(ois_fw->size, GFP_KERNEL));
	if (!fw_write_data) {
		LOG_OIS_ERR("ois fw buffer alloc fail");
		goto p_err;
	}
	fw_size = ois_fw->size;
	memcpy((void *)fw_write_data, (void *)(ois_fw->data), (ois_fw->size));
	fw_read_data = (u8 *)(kzalloc(ois_fw->size, GFP_KERNEL));
	if (!fw_read_data) {
		LOG_OIS_ERR("ois fw buffer alloc fail");
		goto p_err;
	}
	release_firmware(ois_fw);
	LOG_OIS_INF("ois_fw buffer %p, size %d unit_size %d", fw_write_data, fw_size, unit_size);

	//write new fw to flash
	for (idx = 0; idx < fw_size; idx += DW9786_BLOCK_SIZE) {
		if (idx == 0) {
			ret = ois_i2c_write_tele(client, 0xED28, DW9786_MCS_START_ADDRESS);
			I2COP_CHECK(ret);
		}

		ret = ois_i2c_write_block_tele(client, 0xED2C, &fw_write_data[idx], DW9786_BLOCK_SIZE);
		mdelay(10);
		if (ret) {
			update_flag = 1;
			LOG_OIS_ERR("flash[%d] write: addr 0x%04x failed(%d)", idx, (MTP_START_ADDR + idx / unit_size), ret);
			break;
		} else {
			LOG_OIS_INF("flash[%d] write: addr 0x%04x success(%d)", idx, (MTP_START_ADDR + idx / unit_size), ret);
		}
	}
	if (update_flag)
		goto p_err;

	/* Checksum calculation for mcs data */
	checksum_criteria = DW9786_REF_MCS_CHECKSUM;
	/* Set the checksum area */
	checksum_flash = dw9786_checksum_func(client, DW9786_MCS_CODE);
	if (checksum_criteria != checksum_flash) {
		ret = CHECKSUM_ERROR;
		LOG_OIS_INF("[dw9786_fw_download]flash checksum failed!, checksum_criteria: 0x%08X, checksum_flash: 0x%08X", checksum_criteria, checksum_flash);
		dw9786_chip_enable(client, OFF);
		LOG_OIS_INF("[dw9786_fw_download]firmware download failed.");
		goto p_err;
	}
p_err:
	if (fw_write_data) {
		LOG_OIS_INF("ois fw buffer(%p) release", fw_write_data);
		kfree(fw_write_data);
		fw_write_data = NULL;
	}
	if (fw_read_data) {
		LOG_OIS_INF("ois fw buffer(%p) release", fw_read_data);
		kfree(fw_read_data);
		fw_read_data = NULL;
	}
	return ret;
}

static int dw9786_fw_update(struct ois *ois, void __user *user_buf)
{
	int                   ret              = 0;
	struct i2c_client     *client          = ois->client;
	u16                   exp_fw_version   = EXP_FW.version;
	u16                   pjt = 0, ver = 0, dat = 0;
	int                   froce_version    = 0;

	OIS_BUG(!client);

	LOG_OIS_INF("E");

	ois->client->addr = DW9786_SLAVE_ADDR >> 1;

	dw9786_check_data_init(ois);

	//1. check fw version & type
	if (NULL != user_buf) {
		ret = copy_from_user(&froce_version, user_buf, sizeof(int));
		if (ret) {
			LOG_OIS_ERR("copy_from_user fail(%d)", ret);
			goto p_err;
		}
		exp_fw_version = froce_version;
		LOG_OIS_INF("force ois update(0x%04x)", froce_version);
	}

	/* Check the firmware version */
	//dw9786_chip_enable(MODE_ON);
	ret = ois_i2c_read_tele(client, DW9786_RELEASE_VER_H, &pjt);
	ret = ois_i2c_read_tele(client, DW9786_RELEASE_VER_L, &ver);
	ret = ois_i2c_read_tele(client, DW9786_RELEASE_DATE, &dat);
	LOG_OIS_INF("[dw9786_download_fw] module prj_info:[0x%.4x] ver:[0x%.4x] date:[0x%.4x]", pjt, ver, dat);
	LOG_OIS_INF("[dw9786_download_fw] fw prj_info:[0x%.4x] ver:[0x%.4x] date:[0x%.4x]", DW9786_PJT_VERSION, DW9786_FW_VERSION, DW9786_FW_DATE);

	if ((ver != DW9786_FW_VERSION) || (dat != DW9786_FW_DATE)) {
		LOG_OIS_INF("[dw9786_download_fw] the new firmware version is checked and the update starts.");
		if (dw9786_fw_download(ois) != FUNC_PASS) {
			return FUNC_FAIL;
		}
	} else {
		LOG_OIS_INF("[dw9786_download_fw]This is the latest version.");
		if (dw9786_checksum_func(client, DW9786_MCS_CODE) != DW9786_REF_MCS_CHECKSUM) {
			LOG_OIS_INF("[dw9786_download_fw] start download firmware due to a checksum error.");
			if (dw9786_fw_download(ois) != FUNC_PASS) {
				return FUNC_FAIL;
			}
		}
	}
	dw9786_reset(ois);

p_err:
	return ret;
}

static int dw9786_status_check(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	s16 s16_rxdata = 0x0000;
	struct i2c_client     *client      = ois->client;
	struct ois_flash_info *flash_info = ois->flash_info;

	OIS_BUG(!client);
	OIS_BUG(!flash_info);
	OIS_BUG(!(ois->flash_info->chipEn));

	ret = ois_i2c_read_tele(client, DW9786_RELEASE_VER_H, &s16_rxdata);
	LOG_OIS_INF("ois release ver_H %x", s16_rxdata);
	ret = ois_i2c_read_tele(client, DW9786_RELEASE_VER_L, &s16_rxdata);
	LOG_OIS_INF("ois release ver_L %x", s16_rxdata);
	ret = ois_i2c_read_tele(client, DW9786_RELEASE_DATE, &s16_rxdata);
	LOG_OIS_INF("ois release date %x", s16_rxdata);

	ret = ois_i2c_read_tele(client, DW9786_CHIP_ID, &s16_rxdata);
	LOG_OIS_INF("ois CHIP ID %04x", s16_rxdata);
	ret = ois_i2c_read_tele(client, DW9786_PRODUCT_ID, &s16_rxdata);
	LOG_OIS_INF("ois PRODUCT ID %04x", s16_rxdata);

	ret = ois_i2c_read_tele(client, DW9786_OIS_ON_OFF, &s16_rxdata);
	LOG_OIS_INF("ois on/off %d", s16_rxdata);
	ret = ois_i2c_read_tele(client, DW9786_OIS_STATUS, &s16_rxdata);
	LOG_OIS_INF("ois STATUS %x", s16_rxdata);
	ret = ois_i2c_read_tele(client, DW9786_OIS_ACTIVE, &s16_rxdata);
	LOG_OIS_INF("ois ACTIVE %d", s16_rxdata);
	ret = ois_i2c_read_tele(client, DW9786_OIS_CTROL_MODE, &s16_rxdata);
	LOG_OIS_INF("ois CONTROL MODE %d", s16_rxdata);

	ret = ois_i2c_read_tele(client, DW9786_LENS_POSX, &s16_rxdata);
	LOG_OIS_INF("ois posX %d", s16_rxdata);
	ret = ois_i2c_read_tele(client, DW9786_LENS_POSY, &s16_rxdata);
	LOG_OIS_INF("ois posY %d", s16_rxdata);

	/*ret = ois_i2c_read_tele(client, DW9786_REG_OIS_GYRO_RAWX, &s16_rxdata);
	flash_info->imuInfo.gyroRawX = s16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_GYRO_RAWY, &s16_rxdata);
	flash_info->imuInfo.gyroRawY = s16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_ACC_RAWX, &s16_rxdata);
	flash_info->imuInfo.accRawX = s16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_ACC_RAWY, &s16_rxdata);
	flash_info->imuInfo.accRawY = s16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_GYRO_READ_CTRL, &u16_rxdata);
	flash_info->imuInfo.imuReadEn = u16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_SPI_MODE, &u16_rxdata);
	flash_info->imuInfo.spiMode = u16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_LENS_POSX, &s16_rxdata);
	flash_info->hallInfo.lensPosX = s16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_LENS_POSY, &s16_rxdata);
	flash_info->hallInfo.lensPosY = s16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_MODE, &u16_rxdata);
	flash_info->mode = u16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_CTRL, &u16_rxdata);
	flash_info->serveOn = u16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_DSP_CTRL, &u16_rxdata);
	flash_info->dspEn = u16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_LOGIC_RESET, &u16_rxdata);
	flash_info->logicReset = u16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_ACC_GAINX, &s16_rxdata);
	flash_info->imuInfo.accGainX = s16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_ACC_GAINY, &s16_rxdata);
	flash_info->imuInfo.accGainY = s16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_TRIPODE_STATUS, &u16_rxdata);
	flash_info->tripodFlag = u16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_AF_DRIFT_COMP, &s16_rxdata);
	drift_on = s16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_AF_POSITION, &s16_rxdata);
	af_position = s16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_PANTILT_DERGEEX, &u16_rxdata);
	flash_info->hallInfo.totalLimitX = u16_rxdata;
	ret = ois_i2c_read_tele(client, DW9786_REG_OIS_PANTILT_DERGEEY, &u16_rxdata);
	flash_info->hallInfo.totalLimitY = u16_rxdata;
	if (SERVO_ON == flash_info->serveOn) {
		ret = ois_i2c_read_tele(client, DW9786_REG_OIS_CL_TARGETX, &s16_rxdata);
		flash_info->targetInfo.totalTargetX = s16_rxdata;
		ret = ois_i2c_read_tele(client, DW9786_REG_OIS_CL_TARGETY, &s16_rxdata);
		flash_info->targetInfo.totalTargetY = s16_rxdata;
	} else if (OIS_ON == flash_info->serveOn) {
		ret = ois_i2c_read_tele(client, DW9786_REG_OIS_TARGETX, &s16_rxdata);
		flash_info->targetInfo.totalTargetX = s16_rxdata;
		ret = ois_i2c_read_tele(client, DW9786_REG_OIS_TARGETY, &s16_rxdata);
		flash_info->targetInfo.totalTargetY = s16_rxdata;
	}

	LOG_OIS_INF("gyro: %d %d acc: %d %d gain 0x%04x 0x%04x target: %d %d lensPos: %d %d",
		flash_info->imuInfo.gyroRawX, flash_info->imuInfo.gyroRawY, flash_info->imuInfo.accRawX, flash_info->imuInfo.accRawY,
		flash_info->imuInfo.gyroGainX, flash_info->imuInfo.gyroGainY,
		flash_info->targetInfo.totalTargetX, flash_info->targetInfo.totalTargetY,
		flash_info->hallInfo.lensPosX, flash_info->hallInfo.lensPosY);

	LOG_OIS_VERB("servo:%d dsp:%d reset:%d mode:%d tripod:%d spi mode:%d read:0x%04x drift:%d %d limit:%d %d",
		flash_info->serveOn, flash_info->dspEn, flash_info->logicReset, flash_info->mode,
		flash_info->tripodFlag, flash_info->imuInfo.imuReadEn, flash_info->imuInfo.spiMode, drift_on, af_position,
		flash_info->hallInfo.totalLimitX, flash_info->hallInfo.totalLimitY);
*/
	return ret;
}

static int dw9786_format_otp_data(struct ois *ois, void __user *user_buf)
{
	int                 ret       = 0;
	u8                  *otp_buf  = NULL;
	u8                  *sn_data  = NULL;
	u8                  *fuse_id  = NULL;
	u8                  data_size = 0;
	u8                  idx       = 0;
	struct ois_otp_info *ois_otp  = NULL;
	s16                 s16_data  = 0x0000;

	OIS_BUG(!user_buf);

	ois_otp = (struct ois_otp_info *)(kzalloc(sizeof(struct ois_otp_info), GFP_KERNEL));
	if (!ois_otp) {
		LOG_OIS_ERR("ois otp data kzalloc failed(%d)\n", ret);
		goto p_err;
	}

	ois_otp->inited = 0x00;

	otp_buf = otp_data_ov08a10;

	ois_otp->fwVersion = OIS_BIG_ENDIAN_TRANS4(otp_buf, 0x0017);
	ois_otp->gyroGainX = OIS_BIG_ENDIAN_TRANS4(otp_buf, 0x0C55);
	ois_otp->gyroGainY = OIS_BIG_ENDIAN_TRANS4(otp_buf, 0x0C59);
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x0C51);
	ois_otp->gyroOffsetX = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x0C53);
	ois_otp->gyroOffsetY = s16_data;
	ois_otp->hallMechCenterX = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x0C5D);
	ois_otp->hallMechCenterY = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x0C5F);
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x0C73);
	ois_otp->hallXMin = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x0C75);
	ois_otp->hallYMin = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x0C6F);
	ois_otp->hallXMax = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x0C71);
	ois_otp->hallYMax = s16_data;
	ois_otp->tiltSRX = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x0C77);
	ois_otp->tiltSRY = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x0C79);
	//ois_otp->accSRX = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2EAA);
	//ois_otp->accSRY = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2EAC);
	ois_otp->icType = DW9786;
	data_size = 0x0026 - 0x0021 + 1;
	fuse_id = kzalloc(sizeof(u8) * data_size * 2 + 2, GFP_KERNEL);
	if (!fuse_id) {
		LOG_OIS_ERR("fuse id kzalloc failed(%d)\n", ret);
		goto p_err;
	}
	for (idx = 0; idx < data_size; idx++)
		sprintf(&fuse_id[idx * 2], "%02x", otp_buf[0x0021 + idx]);

	data_size = 0x0051 - 0x0046 + 1;
	sn_data = kzalloc(sizeof(u8) * data_size * 2 + 2, GFP_KERNEL);
	if (!sn_data) {
		LOG_OIS_ERR("sn data kzalloc failed(%d)\n", ret);
		goto p_err;
	}
	for (idx = 0; idx < data_size; idx++)
		sprintf(&sn_data[idx * 2], "%02x", otp_buf[0x0046 + idx]);

	ret = copy_to_user(user_buf, ois_otp, sizeof(struct ois_otp_info));
	if (ret) {
		LOG_OIS_ERR("fail to copy otp info, ret(%d)\n", ret);
		goto p_err;
	}

	ois_otp->inited = 0x01;

	LOG_OIS_INF("ois otp: sn(0x%s) fuseId(0x%s) fwVer(0x%08x) gyroGain(%d, %d) gyroOffset(%d,%d) hall(%d,%d,%d,%d,%d,%d) SR(%d, %d)",
		sn_data, fuse_id, ois_otp->fwVersion, ois_otp->gyroGainX, ois_otp->gyroGainY,
		ois_otp->gyroOffsetX, ois_otp->gyroOffsetY, ois_otp->hallXMin, ois_otp->hallXMax, ois_otp->hallMechCenterX,
		ois_otp->hallYMin, ois_otp->hallYMax, ois_otp->hallMechCenterY,
		ois_otp->tiltSRX, ois_otp->tiltSRY);
p_err:
	if (ois_otp)
		kfree(ois_otp);
	if (fuse_id)
		kfree(fuse_id);
	if (sn_data)
		kfree(sn_data);
	return ret;
}

static int dw9786_set_stroke_limit(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	return ret;
}

static struct ois_core_ops dw9786_ois_ops = {
	.ois_init = dw9786_init,
	.ois_deinit = dw9786_deinit,
	.ois_stream_on = dw9786_stream_on,
	.ois_stream_off = dw9786_stream_off,
	.ois_get_mode = dw9786_get_mode,
	.ois_set_mode = dw9786_set_mode,
	.ois_fw_update = dw9786_fw_update,
	.ois_get_fw_version = dw9786_get_fw_version,
	.ois_get_gyro_offset = dw9786_get_gyro_offset,
	.ois_set_offset_calibration = dw9786_set_offset_calibration,
	.ois_get_gyro_gain = dw9786_get_gyro_gain,
	.ois_set_gyro_gain = dw9786_set_gyro_gain,
	.ois_flash_save = dw9786_flash_save,
	.ois_set_acc = dw9786_set_acc,
	.ois_set_target = dw9786_set_target,
	.ois_get_init_info = dw9786_get_init_info,
	.ois_status_check = dw9786_status_check,
	.ois_init_vsync_thread = dw9786_init_vsync_thread,
	.ois_deinit_vsync_thread = dw9786_deinit_vsync_thread,
	.ois_vsync_signal = dw9786_vsync_signal,
	.ois_get_lens_info = dw9786_get_lens_info,
	.ois_format_otp_data = dw9786_format_otp_data,
	.ois_set_sinewave = dw9786_set_sinewave,
	.ois_set_stroke_limit = dw9786_set_stroke_limit,
	.ois_set_pantilt = dw9786_set_pantilt,
	.ois_reset = dw9786_reset,
	.ois_set_smooth = dw9786_set_smooth,
	.ois_set_tripod = dw9786_set_tripod,
	.ois_set_circlewave = dw9786_set_circlewave,
	.ois_af_crosstalk_compensation = dw9786_af_crosstalk_compensation,
	.ois_set_servo = dw9786_set_servo,
	.ois_log_control = dw9786_log_control,
	.ois_dependency_init = dw9786_dependency_init,
};

/*ic entry expose to ois_core*/
void dw9786_get_ops(struct ois *ois)
{
	ois->ops = &dw9786_ois_ops;
}