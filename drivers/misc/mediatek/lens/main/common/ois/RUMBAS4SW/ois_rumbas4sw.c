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

#include "ois_rumbas4sw.h"

#define OIS_DRVNAME "OIS_RUMBAS4SW"
#define RUMBAS4SW_SLAVE_ADDR 0x48
#define OTP2ARR(addr) (addr-0x2A06 + 0x0E12)

int spi_monitor_debug = -1;
module_param(spi_monitor_debug, int, 0644);
int vsync_debug = -1;
module_param(vsync_debug, int, 0644);

extern struct i2c_client *g_pstAF_I2Cclient;
extern u32 get_tg_info(void);

#define I2COP_CHECK(cond)                                                      \
	if (cond) {                                                                  \
		LOG_OIS_ERR("ois i2c fail(%d)", cond);                                     \
		goto p_err;                                                                \
	}
#define ois_i2c_write_one_byte(client, addr, val, ret)                         \
	do {                                                                         \
		txData[0] = val;                                                           \
		ret = ois_i2c_write_block(client, addr, txData, 1);                   \
		I2COP_CHECK(ret);                                                          \
	} while (0)

#define ois_i2c_read_one_byte(client, addr, val, ret)                          \
	do {                                                                         \
		ret = ois_i2c_read_block(client, addr, rxData, 1);                    \
		val = rxData[0];                                                           \
		I2COP_CHECK(ret);                                                          \
	} while (0)
#define OIS_LITTLE_ENDIAN_TRANS2_1(u16data) (((u16data & 0xFF00)>> 8) | ((u16data & 0xFF)<<8) )

#if defined(CONFIG_MTK_CAM_PD2085) || defined(CONFIG_MTK_CAM_PD2120A)
#define VIVO_OTP_DATA_SIZE 0x07F8
extern unsigned char vivo_otp_data_imx598pd2085[VIVO_OTP_DATA_SIZE];
#elif defined(CONFIG_MTK_CAM_PD2135)
#define VIVO_OTP_DATA_SIZE 0x2EB0
extern unsigned char vivo_otp_data_imx766pd2135[VIVO_OTP_DATA_SIZE];
extern unsigned char vivo_otp_data_imx766pd2133[VIVO_OTP_DATA_SIZE];
#endif


static int rumbas4sw_log_control(struct ois *ois, int level)
{
	int ret = 0;

	if (level > OIS_LOG_START && level < OIS_LOG_END)
		log_ois_level = level;

	LOG_OIS_INF("log %d", log_ois_level);
	return ret;
}

static int rumbas4sw_set_idle(struct ois *ois)
{
	int is_idle = 0;
	int ret = 0;
	u8 rxData[4] = {0}, txData[4] = {0};
	int ic_state = -1;
	int read_retry = 20;

	struct i2c_client *client = ois->client;

	OIS_BUG(!client);

	//set and wait ic to idle state
	ois_i2c_write_one_byte(client, RUMBA_REG_OIS_CTRL, OFF, ret);
	mdelay(2);
	ois_i2c_read_one_byte(client, RUMBA_REG_OIS_STS, ic_state, ret);
	while (RUMBA_IDLE != ic_state) {
		mdelay(10);
		ois_i2c_read_one_byte(client, RUMBA_REG_OIS_STS, ic_state, ret);
		read_retry--;
		if (!read_retry) {
			LOG_OIS_INF("retry timeout, state(%d)", ic_state);
			break;
		}
	}
	if (RUMBA_IDLE == ic_state)
		is_idle = 1;

	LOG_OIS_INF("state %d", ic_state);
p_err:
	return is_idle;
}

static int rumbas4sw_set_mode(struct ois *ois, int mode, int is_internal)
{
	int ret = 0;
	int user_mode = 0;
	u8 rxData[4] = {0}, txData[4] = {0};
	u8 exp_mode = 0x00, old_mode = 0x00, new_mode = 0x00;
	u8 needRestart = 0, servoOn = 0x00;
	struct i2c_client *client = ois->client;

	OIS_BUG(!client);

	ois_i2c_read_one_byte(client, RUMBA_REG_OIS_MODE, old_mode, ret);

	if (!is_internal) {
		switch (mode) {
		case OIS_CENTER_MODE: {
			exp_mode = RUMBA_CENTERING_MODE;
			break;
		}
		case OIS_STILL_MODE: {
			exp_mode = RUMBA_STILL_MODE;
			break;
		}
		case OIS_VIDEO_MODE: {
			exp_mode = RUMBA_VIDEO_MODE;
			break;
		}
		case OIS_ZOOM_MODE: {
			exp_mode = RUMBA_STILL_MODE;
			break;
		}
		case OIS_FIX_MODE: {
			exp_mode = RUMBA_FIX_MODE;
			needRestart = 1;
			break;
		}
		case OIS_SINEWAVE_MODE: {
			exp_mode = RUMBA_SINE_WAVE_MODE;
			break;
		}
		case OIS_CIRCLEWAVE_MODE: {
			exp_mode = RUMBA_CIR_WAVE_MODE;
			break;
		}
		case OIS_ACTIVE_CEN_ON: {
			ois_i2c_write_one_byte(client, RUMBA_REG_OIS_ACTIVE_ON, ON, ret);
			exp_mode = RUMBA_STILL_MODE;
			break;
		}
		case OIS_ACTIVE_CEN_OFF: {
			ois_i2c_write_one_byte(client, RUMBA_REG_OIS_ACTIVE_ON, OFF, ret);
			exp_mode = RUMBA_STILL_MODE;
			break;
		}

		default: {
			LOG_OIS_INF("unsupport ois mode(%d)", user_mode);
			goto p_err;
		}
		}
	} else
		exp_mode = mode;

	if (0x0000 != exp_mode && old_mode != exp_mode) {
		if (needRestart)
			ret = rumbas4sw_set_idle(ois);
		ois_i2c_write_one_byte(client, RUMBA_REG_OIS_MODE, exp_mode, ret);
		/*if (exp_mode == RUMBA_VIDEO_MODE) {
			rxData[3] = 0x3B;
			rxData[2] = 0x03;
			rxData[1] = 0x12;
			rxData[0] = 0x00;// 1 second
			ret = ois_i2c_write_block(ois->client, RUMBA_REG_MODEX_TO_CENTER_TIME, txData, 4);
			ret = ois_i2c_write_block(ois->client, RUMBA_REG_CENTER_TO_MODEX_TIME, txData, 4);
			LOG_OIS_INF("set smooth");
		}*/
		if (needRestart)
			ois_i2c_write_one_byte(ois->client, RUMBA_REG_OIS_CTRL, ON, ret);
	}

	ois_i2c_read_one_byte(client, RUMBA_REG_OIS_MODE, new_mode, ret);
	ois_i2c_read_one_byte(client, RUMBA_REG_OIS_CTRL, servoOn, ret);

	LOG_OIS_INF("internal(%d) old mode(0x%02x), user mode(0x%02x) exp mode(0x%02x) new mode(0x%02x) servo(0x%02x) result(%d)",
		is_internal, old_mode, user_mode, exp_mode, new_mode, servoOn, ret);

p_err:
	return ret;
}

/*
static int rumbas4sw_sw_reset (struct ois *ois)
{
	int ret = 0;
	u8 rxData[4] = {0};
	u8 txData[4] = {0};
	int read_try = 20;
	struct i2c_client *client = ois->client;

	OIS_BUG(!client);

	ois_i2c_write_one_byte(ois->client, RUMBA_REG_OIS_DFLSCTRL, 0x01, ret);
	do {
		if (0 == read_try) {
			LOG_OIS_ERR("ois data area write timeout!");
			break;
		}
		mdelay(5);
		ois_i2c_read_one_byte(client, RUMBA_REG_OIS_DATAWRITE, rxData[0], ret);
		read_try--;
	} while (rxData[0] != RUMBA_DFLS_UPDATE);

	ois_i2c_write_one_byte(ois->client, RUMBA_REG_OIS_DFLSCMD, 0x06, ret);
	mdelay(20);

p_err:
	return ret;
}
*/
/*temp func to init 5x ois ic*/
static int rumbas4sw_dependency_init(struct ois *ois)
{
	int ret = 0;
	u16 spi = 0x0000, gyro = 0x0000;
	u8 txData[2] = {0};
	struct i2c_client *client = ois->client;

	OIS_BUG(!client);

	client->addr = RUMBAS4SW_SLAVE_ADDR >> 1;

#if defined(CONFIG_MTK_CAM_PD2135)
	if (NULL != g_pstAF_I2Cclient)
		g_pstAF_I2Cclient->addr = 0x32 >> 1;
	//temp code start
	if (NULL != g_pstAF_I2Cclient && '0' == ois->ccm_board_version[0] && (ois_i2c_write(g_pstAF_I2Cclient, 0xE000, 0x0000) == 0) && (ois->sat_mode == SAT_ENABLE || ois->sat_mode == EG_MODE)) {//PD2135
		mdelay(2);
		ret = ois_i2c_write(g_pstAF_I2Cclient, 0xE000, 0x0001);
		mdelay(5);
		ret = ois_i2c_write(g_pstAF_I2Cclient, 0xE2FC, 0xAC1E);
		ret = ois_i2c_write(g_pstAF_I2Cclient, 0xE164, 0x0008);
		ret = ois_i2c_write(g_pstAF_I2Cclient, 0xE2FC, 0x0000);
		mdelay(1);
		ret = ois_i2c_write(g_pstAF_I2Cclient, 0xE004, 0x0001);
		mdelay(50);
		ret = ois_i2c_write(g_pstAF_I2Cclient, 0xB026, 0x0000);
		mdelay(5);
		ret = ois_i2c_write(g_pstAF_I2Cclient, 0xB02C, 0x0001);
		mdelay(80);
		ret = ois_i2c_write(g_pstAF_I2Cclient, 0xB034, 0x0001);
		ret = ois_i2c_read(g_pstAF_I2Cclient, 0xB02C, &spi);
		ret = ois_i2c_read(g_pstAF_I2Cclient, 0xB034, &gyro);
		ois_i2c_write_one_byte(client, RUMBA_REG_OIS_MONITOR_MODE, ENABLE, ret);
	} else {
		LOG_OIS_INF("rumba master");
		ois_i2c_write_one_byte(client, RUMBA_REG_OIS_MONITOR_MODE, OFF, ret);//rumba as master
	}
	if (1 == spi_monitor_debug) {
		LOG_OIS_INF("rumba slave");
		ois_i2c_write_one_byte(client, RUMBA_REG_OIS_MONITOR_MODE, ENABLE, ret);
	} else if (0 == spi_monitor_debug) {
		LOG_OIS_INF("rumba master");
		ois_i2c_write_one_byte(client, RUMBA_REG_OIS_MONITOR_MODE, OFF, ret);
	}
	LOG_OIS_INF("master spi 0x%04x gyro 0x%04x slave monitor %d", spi, gyro, spi_monitor_debug);
#else
	LOG_OIS_INF("rumba master");
	ois_i2c_write_one_byte(client, RUMBA_REG_OIS_MONITOR_MODE, OFF, ret);//rumba as master
#endif

p_err:
	return ret;
}

static int rumbas4sw_init(struct ois *ois)
{
	int ret = 0;
	u8 txData[2] = {0};
	u8 rxData[2] = {0};
	u16 circle_limit = 0x0000, acc_limit = 0x0000;
	struct i2c_client *client = ois->client;

	OIS_BUG(!client);

	LOG_OIS_INF("E");
	client->addr = RUMBAS4SW_SLAVE_ADDR >> 1;

	//set acc limit,0x044C=1100=1.1°s
	txData[1] = 0x04;
	txData[0] = 0x4C;
	ret = ois_i2c_write_block(client, RUMBA_REG_OIS_ACC_LIMIT, txData, 2);
	if (ret) {
		LOG_OIS_ERR("write acc limit  fail(%d)", ret);
	}

	//set circle limit
	txData[1] = 0x0B;
	txData[0] = 0xB8;
	ret = ois_i2c_write_block(client, RUMBA_REG_GYRO_TARGET_LIMITX, txData, 2);
	if (ret) {
		LOG_OIS_ERR("write circle limit  fail(%d)", ret);
	}
	txData[1] = 0x0B;
	txData[0] = 0xB8;
	ret = ois_i2c_write_block(client, RUMBA_REG_GYRO_TARGET_LIMITY, txData, 2);
	if (ret) {
		LOG_OIS_ERR("write circle limit  fail(%d)", ret);
	}
	txData[1] = 0x0B;
	txData[0] = 0xB8;
	ret = ois_i2c_write_block(client, RUMBA_REG_OIS_CIRCLELIMIT, txData, 2);
	if (ret) {
		LOG_OIS_ERR("write circle limit  fail(%d)", ret);
	}
	ois_i2c_write_one_byte(client, RUMBA_REG_OIS_ACTIVE_ON, OFF, ret);

#if defined(CONFIG_MTK_CAM_PD2135)
	//check if pre-dependency init failed
	if (!ois->dependency_ready && '0' == ois->ccm_board_version[0])
		ret = rumbas4sw_dependency_init(ois);
#endif

	//ois on
	ret = rumbas4sw_set_mode(ois, RUMBA_STILL_MODE, 1);
	ois_i2c_write_one_byte(ois->client, RUMBA_REG_OIS_CTRL, ON, ret);

	//enable ic internal info update
	ois_i2c_write_one_byte(client, RUMBA_REG_OIS_FWINFO_UPDATE, ON, ret);

	ret = ois_i2c_read_block(ois->client, RUMBA_REG_OIS_ACC_LIMIT, rxData, 2);
	acc_limit = rxData[0] & rxData[1] << 8;
	ret = ois_i2c_read_block(ois->client, RUMBA_REG_OIS_CIRCLELIMIT, rxData, 2);
	circle_limit = rxData[0] & rxData[1] << 8;

	ois->flash_info->chipEn = 0x0001;
	LOG_OIS_INF("X:cmd(0x%08x, 0x%08x, 0x%08x, 0x%08x, 0x%08x, 0x%08x, 0x%08x) angle(acc=0x%04x, circle=0x%04x)",
		AFIOC_X_OIS_SETMODE, AFIOC_X_OIS_SETACC, AFIOC_X_OIS_STATUSCHECK, AFIOC_X_OIS_OFFSETCAL,
		AFIOC_X_OIS_GETINITINFO, AFIOC_X_OIS_GETOTPINFO, AFIOC_X_OIS_SETFIXMODE,
		acc_limit, circle_limit);
p_err:
	return ret;
}

static int rumbas4sw_get_init_info(struct ois *ois, void __user *user_buf) 
{
	int ret = 0;
	struct i2c_client *client = ois->client;
	struct ois_flash_info *flash_info = ois->flash_info;
	u8 rxData[4] = {0};
	u16 u16_rxdata = 0x0000;

	OIS_BUG(!client);
	OIS_BUG(!flash_info);
	OIS_BUG(!user_buf);

	ret = ois_i2c_read_block(client, RUMBA_REG_OIS_FWVER, rxData, 4);
	flash_info->fwInfo.version = OIS_LITTLE_ENDIAN_TRANS4(rxData, 0);

	ret = ois_i2c_read(client, RUMBA_REG_OIS_HALLXMAX, &u16_rxdata);
	flash_info->caliInfo.hallXMax = OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);
	ret = ois_i2c_read(client, RUMBA_REG_OIS_HALLYMAX, &u16_rxdata);
	flash_info->caliInfo.hallYMax = OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);
	ret = ois_i2c_read(client, RUMBA_REG_OIS_HALLXMIM, &u16_rxdata);
	flash_info->caliInfo.hallXMin = OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);
	ret = ois_i2c_read(client, RUMBA_REG_OIS_HALLYMIN, &u16_rxdata);
	flash_info->caliInfo.hallYMin = OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);
	ret = ois_i2c_read(client, RUMBA_REG_XMECHCENTER, &u16_rxdata);
	flash_info->caliInfo.hallMechCenterX = OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);
	ret = ois_i2c_read(client, RUMBA_REG_YMECHCENTER, &u16_rxdata);
	flash_info->caliInfo.hallMechCenterY = OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);

	ret = ois_i2c_read(client, RUMBA_REG_OIS_GOFFSETX, &u16_rxdata);
	flash_info->caliInfo.gyroOffsetX = (s16)OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);
	ret = ois_i2c_read(client, RUMBA_REG_OIS_GOFFSETY, &u16_rxdata);
	flash_info->caliInfo.gyroOffsetY = (s16)OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);

	ret = ois_i2c_read(client, RUMBA_REG_GYRO_TARGET_LIMITX, &u16_rxdata);
	flash_info->hallInfo.totalLimitX = OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);

	ret = ois_i2c_read(client, RUMBA_REG_GYRO_TARGET_LIMITY, &u16_rxdata);
	flash_info->hallInfo.totalLimitY = OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);

	ret = ois_i2c_read(client, RUMBA_REG_OIS_CIRCLELIMIT, &u16_rxdata);
	flash_info->hallInfo.totalLimitCircle = OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);

	ois_i2c_read_one_byte(client, RUMBA_REG_GYRO_ORIENT, rxData[0], ret);
	flash_info->caliInfo.gyroOrient = rxData[0];
	ois_i2c_read_one_byte(client, RUMBA_REG_ACC_MIRROR, rxData[0], ret);
	flash_info->caliInfo.accMirror = rxData[0];
	ois_i2c_read_one_byte(client, RUMBA_REG_OIS_MODE, rxData[0], ret);
	flash_info->mode = rxData[0];
	ois_i2c_read_one_byte(client, RUMBA_REG_OIS_CTRL, rxData[0], ret);
	flash_info->serveOn = rxData[0];

	ret = ois_i2c_read_block(client, RUMBA_REG_OIS_GGX, rxData, 4);
	flash_info->caliInfo.gyroGainX = OIS_LITTLE_ENDIAN_TRANS4(rxData, 0);
	ret = ois_i2c_read_block(client, RUMBA_REG_OIS_GGY, rxData, 4);
	flash_info->caliInfo.gyroGainY = OIS_LITTLE_ENDIAN_TRANS4(rxData, 0);

	ret = ois_i2c_read_block(client, RUMBA_REG_OIS_ACCGAINX, rxData, 4);
	flash_info->imuInfo.accGainX = OIS_LITTLE_ENDIAN_TRANS4(rxData, 0);
	ret = ois_i2c_read_block(client, RUMBA_REG_OIS_ACCGAINY, rxData, 4);
	flash_info->imuInfo.accGainY = OIS_LITTLE_ENDIAN_TRANS4(rxData, 0);
	ret = ois_i2c_read_block(client, RUMBA_REG_OIS_ACCFGAINX, rxData, 4);
	flash_info->imuInfo.accFineGainX = OIS_LITTLE_ENDIAN_TRANS4(rxData, 0);
	ret = ois_i2c_read_block(client, RUMBA_REG_OIS_ACCFGAINY, rxData, 4);
	flash_info->imuInfo.accFineGainY = OIS_LITTLE_ENDIAN_TRANS4(rxData, 0);

	ret = ois_i2c_read_block(client, RUMBA_REG_MODEX_TO_CENTER_TIME, rxData, 4);
	flash_info->smoothInfo.delay = OIS_LITTLE_ENDIAN_TRANS4(rxData, 0);
	ret = ois_i2c_read_block(client, RUMBA_REG_CENTER_TO_MODEX_TIME, rxData, 4);
	flash_info->smoothInfo.step = OIS_LITTLE_ENDIAN_TRANS4(rxData, 0);

	ret = copy_to_user(user_buf, flash_info, sizeof(struct ois_flash_info));
	if (ret) {
		LOG_OIS_ERR("fail to copy flash info, ret(%d)\n", ret);
		goto p_err;
	}

	LOG_OIS_INF("flash info:fw(%d), status(%d, %d)hallCenter(%d, %d), hallX(%d, %d), hallY(%d, %d), gyroGain(0x%x, 0x%x), gyroOffset(%d, %d), limit(%d, %d, %d), accGain(0x%x,0x%x), accFGain(0x%x, 0x%x)",
		flash_info->fwInfo.version, flash_info->serveOn, flash_info->mode, flash_info->caliInfo.hallMechCenterX, flash_info->caliInfo.hallMechCenterY, flash_info->caliInfo.hallXMin, 
		flash_info->caliInfo.hallXMax, flash_info->caliInfo.hallYMin, flash_info->caliInfo.hallYMax, flash_info->caliInfo.gyroGainX, 
		flash_info->caliInfo.gyroGainY, flash_info->caliInfo.gyroOffsetX, flash_info->caliInfo.gyroOffsetY, flash_info->hallInfo.totalLimitX,
		flash_info->hallInfo.totalLimitY, flash_info->hallInfo.totalLimitCircle, flash_info->imuInfo.accGainX, flash_info->imuInfo.accGainY,
		flash_info->imuInfo.accFineGainX, flash_info->imuInfo.accFineGainY);
p_err:

	return ret;
}

static int rumbas4sw_deinit(struct ois *ois)
{
	int ret = 0;
	struct i2c_client *client = ois->client;
	
	u8 txData[4] = {0};
	LOG_OIS_INF("E");

	OIS_BUG(!client);

	txData[0] = 0;
	ois_i2c_write_one_byte(client, RUMBA_REG_OIS_CTRL, OFF, ret);
	ois->flash_info->chipEn = 0x00;

	LOG_OIS_INF("X(%d)", ret);
p_err:
	return ret;
}

static int rumbas4sw_stream_on(struct ois *ois)
{
	int ret = 0;
	return ret;
}

static int rumbas4sw_stream_off(struct ois *ois)
{
	int ret = 0;
	return ret;
}

static void rumbas4sw_vsync_work_fn(struct kthread_work *work)
{
	int ret = 0;
	struct ois* ois = NULL;
	struct i2c_client* client = NULL;
	u8     insert_idx = 0;
	u8     groupIdx   = 0;
	size_t readSize   = 0;
	struct ois_lens_info* oedata = NULL;
	struct RUMBAS4SW_LENS_INFO lens_info[LENS_INFO_GROUPS_MAX];
	struct timespec start_time, end_time;
	LOG_OIS_VERB("ois vsync process E");
	
	OIS_BUG_VOID(!work);
	ois = container_of(work, struct ois, vsync_work);
	OIS_BUG_VOID(!ois);
	client = ois->client;
	OIS_BUG_VOID(!client);
	OIS_BUG_VOID(!(ois->lens_info_buf));
	
	ktime_get_ts(&start_time);
	mutex_lock(&ois->op_lock);
	
	insert_idx = ois->lens_info_buf->insertidx;
	oedata  = &ois->lens_info_buf->buf[insert_idx];
	oedata->fcount = ois->vsync_info.ois_vsync_cnt;
	oedata->timestampboot = ois->vsync_info.sof_timestamp_boot;
	oedata->validnum = LENS_INFO_GROUPS_MAX;
	LOG_OIS_VERB("info fcount=%llu ts=%llu validnum=%d insert=%d",
		oedata->fcount, oedata->timestampboot, oedata->validnum, insert_idx);

	ois->vsync_info.ois_vsync_cnt++;
	
	readSize = oedata->validnum * sizeof(struct RUMBAS4SW_LENS_INFO);
	ret = ois_i2c_read_block(client, RUMBA_REG_OIS_EIS_BUF, (u8*)(&lens_info[0]), readSize);
	if (ret < 0) {
		LOG_OIS_ERR("ois user data write is fail, ret:%d", ret);
		goto p_err;
	}
	for (groupIdx = 0; groupIdx < oedata->validnum; ++groupIdx) {
		oedata->hallx[groupIdx] = lens_info[groupIdx].hallX;
		oedata->hally[groupIdx] = lens_info[groupIdx].hallY;
		oedata->ic_timecount[groupIdx] = lens_info[groupIdx].timeStamp;
		if(oedata->ic_timecount[groupIdx] == 0 && oedata->hallx[groupIdx]==0) {
			oedata->validnum = groupIdx;
			break;
		}
		LOG_OIS_INF("fcount %lu data[%d] timestamp %lu hallx %d hally %d",
			oedata->fcount, groupIdx,  oedata->ic_timecount[groupIdx],
			oedata->hallx[groupIdx], oedata->hally[groupIdx]);
	}
	if (++ois->lens_info_buf->insertidx == LENS_INFO_FRAMES_MAX)
		ois->lens_info_buf->insertidx = 0;	
	ktime_get_ts(&end_time);
	
	LOG_OIS_VERB("ois vsync process X, cost time %llu ms, validnumReal=%d", 
		(((u64)end_time.tv_sec * 1000 + end_time.tv_nsec / 1000000) - ((u64)start_time.tv_sec * 1000 + start_time.tv_nsec / 1000000)), oedata->validnum);
	
p_err:
	mutex_unlock(&ois->op_lock);
	return;
}

static int rumbas4sw_init_vsync_thread(struct ois *ois)
{
	int ret = 0;
	struct sched_param param = { .sched_priority =  MAX_RT_PRIO - 50 };

	if (NULL == ois->vsync_task) {
		kthread_init_worker(&ois->vsync_worker);
		kthread_init_work(&ois->vsync_work, rumbas4sw_vsync_work_fn);
		ois->vsync_task = kthread_run(kthread_worker_fn, &ois->vsync_worker, "rumbas4sw_vsync_process");
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
static int rumbas4sw_deinit_vsync_thread(struct ois *ois)
{
	int ret = 0;

	if (!IS_ERR_OR_NULL(ois->vsync_task)) {
		LOG_OIS_INF("stop ois vsync start(%p)", ois->vsync_task);
		ret = kthread_stop(ois->vsync_task);
		if (ret)
			LOG_OIS_ERR("vsync stop failed");
		ois->vsync_task = NULL;
		LOG_OIS_INF("stop ois vsync done");
	} else {
		LOG_OIS_INF("ois vsync already done(%p)", ois->vsync_task);
	}

	return ret;
}

static int rumbas4sw_vsync_signal(struct ois *ois, void *buf)
{
	int ret = 0;
	struct timespec curtime;
	struct ois_vsync_info *tempInfo = NULL;
	u64 module_idx = 0, vsync_cnt = 0;
	u32 tg_info = 0;

	OIS_BUG(!buf);
	OIS_BUG(!(ois->lens_info_buf));

	tempInfo = (struct ois_vsync_info *)buf;
	module_idx = tempInfo->module_idx;
	vsync_cnt = tempInfo->vsync_cnt;
	tg_info = get_tg_info() - 1;


	if (module_idx == tg_info) {
		curtime = ktime_to_timespec(tempInfo->sof_timestamp_boot);
		ois->vsync_info.sof_timestamp_boot = (u64)curtime.tv_sec * 1000000000 + curtime.tv_nsec;
		ois->vsync_info.vsync_cnt = vsync_cnt;
		ois->vsync_info.module_idx = module_idx;
		if (ois->vsync_task != NULL && 0 != vsync_debug) {
			kthread_queue_work(&ois->vsync_worker, &ois->vsync_work);
		}

		LOG_OIS_INF("signal vsync %llu timestamp %llu, module:%lu sat %d, tg_info %d",
			ois->vsync_info.vsync_cnt, ois->vsync_info.sof_timestamp_boot, module_idx, ois->sat_mode, tg_info);
	} else if (tg_info != 7) {
				curtime = ktime_to_timespec(tempInfo->sof_timestamp_boot);
		ois->vsync_info.sof_timestamp_boot = (u64)curtime.tv_sec * 1000000000 + curtime.tv_nsec;
		ois->vsync_info.vsync_cnt = vsync_cnt;
		ois->vsync_info.module_idx = module_idx;
		if (ois->vsync_task != NULL && 0 != vsync_debug) {
			kthread_queue_work(&ois->vsync_worker, &ois->vsync_work);
		}

		LOG_OIS_INF("signal vsync %llu timestamp %llu, module:%lu sat %d, tg_info %d",
			ois->vsync_info.vsync_cnt, ois->vsync_info.sof_timestamp_boot, module_idx, ois->sat_mode, tg_info);

	} else {
		LOG_OIS_INF("signal skip module:%lu sat %d, tg_info %d", module_idx, ois->sat_mode, tg_info);
	}

	return ret;
}

static int rumbas4sw_get_lens_info(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	u8  idx = 0;
	struct timespec start_time, end_time;

	ktime_get_ts(&start_time);
	
	OIS_BUG(!user_buf);
	mutex_lock(&ois->op_lock);
	
	ret = copy_to_user(user_buf, ois->lens_info_buf, sizeof(struct ois_lens_info_buf));
	if(ret) {
		LOG_OIS_ERR("copy_to_user fail(%d)", ret);
	}
	ktime_get_ts(&end_time);
	for(idx = 0; idx < LENS_INFO_FRAMES_MAX; ++idx) {
		LOG_OIS_VERB("fcount %lu data[0] timestamp %lu hallx %d hally %d", 
			ois->lens_info_buf->buf[idx].fcount, ois->lens_info_buf->buf[idx].ic_timecount[0],
			ois->lens_info_buf->buf[idx].hallx[0], ois->lens_info_buf->buf[idx].hally[0]);
	}
	LOG_OIS_INF("lens info copy %lu done, ret(%d), time cost %llu ms", ois->lens_info_buf->buf[idx].fcount, ret,
		(((u64)end_time.tv_sec * 1000 + end_time.tv_nsec/1000000) - ((u64)start_time.tv_sec * 1000 + start_time.tv_nsec/1000000)));
	mutex_unlock(&ois->op_lock);
	return ret;
}

static int rumbas4sw_get_fw_version(struct ois *ois, __user void *user_buf)
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

static int rumbas4sw_get_gyro_offset(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	struct ois_flash_info *flash_info = ois->flash_info;

	OIS_BUG(!flash_info);
	OIS_BUG(!user_buf);

	ret = copy_to_user(user_buf, &flash_info->caliInfo.gyroOffsetX, (2 * sizeof(int)));
	if (ret) {
		LOG_OIS_ERR("copy_to_user fail(%d)", ret);
	}

	LOG_OIS_INF("offset(%d, %d)", flash_info->caliInfo.gyroOffsetX, flash_info->caliInfo.gyroOffsetY);
	return ret;
}

static int rumbas4sw_get_gyro_gain(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	struct ois_flash_info *flash_info = ois->flash_info;

	OIS_BUG(!flash_info);
	OIS_BUG(!user_buf);

	ret = copy_to_user(user_buf, &flash_info->caliInfo.gyroGainX, (2 * sizeof(int)));
	if (ret) {
		LOG_OIS_ERR("copy_to_user fail(%d)", ret);
	}

	LOG_OIS_INF("gyro gain(0x%08x, 0x%08x)", flash_info->caliInfo.gyroGainX, flash_info->caliInfo.gyroGainY);
	return ret;
}

static int rumbas4sw_get_mode(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	struct ois_flash_info *flash_info = ois->flash_info;

	OIS_BUG(!flash_info);
	OIS_BUG(!user_buf);

	ret = copy_to_user(user_buf, &flash_info->mode, sizeof(int));
	if (ret) {
		LOG_OIS_ERR("copy_to_user fail(%d)", ret);
	}

	LOG_OIS_INF("ois mode(0x%x)", flash_info->mode);

	return ret;
}

static int rumbas4sw_flash_save(struct ois *ois)
{
	int ret = 0;
	struct i2c_client *client = ois->client;
	u8 rxData[4] = {0};
	u8 txData[4] = {0};
	u8 pre_mode = 0x00;
	int read_try = 20;

	OIS_BUG(!client);

	//save previous mode before cali
	ois_i2c_read_one_byte(client, RUMBA_REG_OIS_MODE, pre_mode, ret);

	//set and wait ic to idle state
	ret = rumbas4sw_set_idle(ois);
	if (!ret) {
		LOG_OIS_ERR("invalid ic status, skip calibration");
		goto p_err;
	}

	//result check data
	ois_i2c_write_one_byte(client, RUMBA_FLSWRTRESULT, 0xAA, ret);
	mdelay(2);

	//exec ois data area write
	ois_i2c_write_one_byte(client, RUMBA_REG_OIS_DATAWRITE, ENABLE, ret);
	mdelay(190);
	do {
		if (0 == read_try) {
			LOG_OIS_ERR("ois data area write timeout!");
			break;
		}
		ois_i2c_read_one_byte(client, RUMBA_REG_OIS_DATAWRITE, rxData[0], ret);
		read_try--;
		mdelay(5);
	} while (rxData[0] != 0x00);

	if (read_try)
		LOG_OIS_INF("write success(%d)", read_try);
	else
		goto p_err;

	//result check again
	ois_i2c_read_one_byte(client, RUMBA_FLSWRTRESULT, rxData[0], ret);
	if (rxData[0] != 0xAA) {
		LOG_OIS_ERR("fail to FLSWRT RESULT ret(%d), res(%d)", ret, rxData[0]);
		goto p_err;
	}

	//reset to pre-mode
	ret = rumbas4sw_set_mode(ois, pre_mode, 1);
	ois_i2c_write_one_byte(ois->client, RUMBA_REG_OIS_CTRL, ON, ret);

	LOG_OIS_INF("X ois flash save success(%d)", ret);
p_err:
	return ret;
}

static int rumbas4sw_set_gyro_gain(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	struct i2c_client *client = ois->client;
	int gain_param[3] = {0,};
	u8 rxData[4] = {0};
	u8 txData[4] = {0};
	int txGainVal = 0;
	u8 gainValue = 0x10;
	u16 addr = 0;
	u8 status = 0x00;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = copy_from_user(gain_param, user_buf, sizeof(gain_param));
	if (ret) {
		LOG_OIS_ERR("copy gain fail(%d)", ret);
		goto p_err;
	}

	ret = rumbas4sw_set_mode(ois, RUMBA_STILL_MODE, 1);

	ois_i2c_read_one_byte(client, RUMBA_REG_OIS_STS, status, ret);
	if (gain_param[0] > 0) {
		addr = RUMBA_REG_OIS_GGY;
		txGainVal = (gain_param[2]);
		gainValue = 0x20;
	} else {
		addr = RUMBA_REG_OIS_GGX;
		txGainVal = (gain_param[1]);
		gainValue = 0x10;
	}
	LOG_OIS_INF("rumba ois status %d, inputGain(0x%x)", status, txGainVal);

	txData[3] = (txGainVal >> 24) & 0xFF;
	txData[2] = (txGainVal >> 16) & 0xFF;
	txData[1] = (txGainVal >> 8) & 0xFF;
	txData[0] = (txGainVal)&0xFF;
	ret = ois_i2c_write_block(client, addr, txData, 4);
	I2COP_CHECK(ret);

	ois_i2c_write_one_byte(client, RUMBA_REG_OIS_GGADJ_CTRL, gainValue, ret);
	mdelay(30);

	//ret = rumbas4sw_flash_save(ois);
	//I2COP_CHECK(ret);

	ret = ois_i2c_read_block(client, addr, rxData, 4);

	LOG_OIS_INF("gain param(%d %d %d), set(addr=0x%04x value=%d) read (0x%x%x%x%x)",
		gain_param[0], gain_param[1], gain_param[2], addr, txGainVal, rxData[3],
		rxData[2], rxData[1], rxData[0]);

p_err:
	return ret;
}

static int rumbas4sw_set_offset_calibration(struct ois *ois)
{
	int ret = 0;
	s16 offsetx = 0x0000;
	s16 offsety = 0x0000;
	u16 repeatedCnt = 50;
	struct i2c_client *client = ois->client;
	struct ois_flash_info *flash_info = ois->flash_info;
	u8 rxData[4] = {0};
	u8 txData[4] = {0};
	LOG_OIS_INF("E");

	OIS_BUG(!client);
	OIS_BUG(!flash_info);

	ret = ois_i2c_read(client, RUMBA_REG_OIS_GOFFSETX, &offsetx);
	ret = ois_i2c_read(client, RUMBA_REG_OIS_GOFFSETY, &offsety);
	LOG_OIS_INF("before cal offset(%d %d) ret(%d)", (s16)OIS_LITTLE_ENDIAN_TRANS2_1(offsetx), (s16)OIS_LITTLE_ENDIAN_TRANS2_1(offsety), ret);

	mutex_lock(&ois->op_lock);

	//set and wait ic to idle state
	ret = rumbas4sw_set_idle(ois);
	if (!ret) {
		LOG_OIS_ERR("invalid ic status, skip calibration");
		goto p_err;
	}

	//start gyro offset cali
	ois_i2c_write_one_byte(client, RUMBA_REG_OIS_GCC_CTRL, ENABLE, ret);
	do {
		if (repeatedCnt == 0) {
			ret = -1;
			LOG_OIS_INF("wait calibration done timeout!");
			break;
		}
		mdelay(10);
		ois_i2c_read_one_byte(client, RUMBA_REG_OIS_GCC_CTRL, rxData[0], ret);
		repeatedCnt--;
	} while (rxData[0] != 0x00);

	ois_i2c_read_one_byte(client, RUMBA_REG_OIS_ERR, rxData[0], ret);
	if (rxData[0] & 0x0023) {
		LOG_OIS_INF("error in gyro offset calibration(%x)", rxData[0]);
		ret = -1;
		goto p_err;
	}

	rumbas4sw_flash_save(ois);

	ret = ois_i2c_read(client, RUMBA_REG_OIS_GOFFSETX, &offsetx);
	ret = ois_i2c_read(client, RUMBA_REG_OIS_GOFFSETY, &offsety);
	flash_info->imuInfo.gyroOffsetX = (s16)OIS_LITTLE_ENDIAN_TRANS2_1(offsetx);
	flash_info->imuInfo.gyroOffsetY = (s16)OIS_LITTLE_ENDIAN_TRANS2_1(offsety);

	LOG_OIS_INF("X: after cal offset(%d %d) ret(%d)", flash_info->imuInfo.gyroOffsetX, flash_info->imuInfo.gyroOffsetY, ret);

p_err:
	mutex_unlock(&ois->op_lock);
	return ret;
}

static int rumbas4sw_set_acc(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	struct ois_acc_param acc_info = {0};
	struct i2c_client *client = ois->client;
	u8 rxData[4] = {0};
	u8 txData[4] = {0};

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = copy_from_user(&acc_info, user_buf, sizeof(struct ois_acc_param));
	if (ret) {
		LOG_OIS_ERR("copy_from_user fail(%d)", ret);
		goto p_err;
	}

	if (acc_info.engineerMode == ENGINEER_MODE_OIS) {
		if (acc_info.accOn) {
			txData[3] = 0x43;
			txData[2] = 0x96;
			txData[1] = 0x00;
			txData[0] = 0x00;
			ois_i2c_write_one_byte(client, RUMBA_REG_OIS_ACCCTRL, ON, ret);
			mdelay(2);
			ret = ois_i2c_write_block(client, RUMBA_REG_OIS_ACCDIS, txData, 4);
		} else {
			txData[3] = 0x45;
			txData[2] = 0x9C;
			txData[1] = 0x40;
			txData[0] = 0x00;
			ois_i2c_write_one_byte(client, RUMBA_REG_OIS_ACCCTRL, OFF, ret);
			mdelay(2);
			ret = ois_i2c_write_block(client, RUMBA_REG_OIS_ACCDIS, txData, 4);
		}
		ret = ois_i2c_read_block(client, RUMBA_REG_OIS_ACCDIS, rxData, 4);
		LOG_OIS_VERB("readDistance:(%x|%x|%x|%x)", rxData[3], rxData[2], rxData[1], rxData[0]);
		ois_i2c_read_one_byte(client, RUMBA_REG_OIS_ACCCTRL, rxData[0], ret);
		LOG_OIS_VERB("ois set ACC status %d", rxData[0]);
	} else {
		if(acc_info.accOn == 1) {
			txData[3] = (acc_info.currFocusDistanceF >> 24) & 0xFF;
			txData[2] = (acc_info.currFocusDistanceF >> 16) & 0xFF;
			txData[1] = (acc_info.currFocusDistanceF >> 8) & 0xFF;
			txData[0] = (acc_info.currFocusDistanceF) & 0xFF;
			ret = ois_i2c_write_block(client, RUMBA_REG_OIS_ACCDIS, txData, 4);
			if (ret < 0) {
				LOG_OIS_ERR("set acc distance failed");
				goto p_err;
			}
			ois_i2c_write_one_byte(client, RUMBA_REG_OIS_ACCCTRL, ON, ret);
			
			ret = ois_i2c_read_block(ois->client, RUMBA_REG_OIS_ACCDIS, rxData, 4);
			LOG_OIS_VERB("rumba: set acc distance %x mm, tx[0]=%x, readDistance:(%x|%x|%x|%x), ret=%d",
				acc_info.currFocusDistanceF, txData[0], rxData[3], rxData[2], rxData[1], rxData[0], ret);
		} else {
			ois_i2c_write_one_byte(client, RUMBA_REG_OIS_ACCCTRL, OFF, ret);
		}
	}
	
p_err:
	return ret;
}

static int rumbas4sw_set_sinewave(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	struct ois_sinemode_parameter parameter = {0};
	struct i2c_client *client = ois->client;
	u8  txData[2] = {0}, ampx[2] = {0}, ampy[2] = {0};

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = rumbas4sw_set_mode(ois, RUMBA_SINE_WAVE_MODE, 1);
	
	ret = copy_from_user(&parameter, user_buf, sizeof(struct ois_sinemode_parameter));
	if (ret) {
		LOG_OIS_ERR("copy sine params fail(%d)", ret);
		goto p_err;
	}

	LOG_OIS_INF("set sinewave frequency(%d),amplitude(%d, %d), axis(%d)", parameter.frequency, parameter.amplitudeX,
		parameter.amplitudeY, parameter.axis);

	ois_i2c_write_one_byte(client, RUMBA_REG_OIS_SINCTRL, (u8)parameter.axis, ret);
	ois_i2c_write_one_byte(client, RUMBA_REG_OIS_SINFREQ, (u8)parameter.frequency, ret);

	ampx[1] = ((u16)(parameter.amplitudeX) >> 8) & 0xFF;
	ampx[0] = ((u16)(parameter.amplitudeX)) & 0xFF;

	ampy[1] = ((u16)(parameter.amplitudeY) >> 8) & 0xFF;
	ampy[0] = ((u16)(parameter.amplitudeY)) & 0xFF;

	ret = ois_i2c_write_block(client, RUMBA_REG_OIS_SINAMPX, ampx, 2);
	mdelay(1);
	ret = ois_i2c_write_block(client, RUMBA_REG_OIS_SINAMPY, ampy, 2);
	mdelay(1);

p_err:
	return ret;
}

static int rumbas4sw_set_circlewave(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	u8  rxData[4] = { 0 };
	u8  txData[4] = { 0 };
	int repeatedCnt = 100;
	int circleCheckResult = -1;
	struct i2c_client *client = ois->client;
	struct ois_circlemode_parameter parameter = { 0 };
	s16 errthreadhold = 0;
	u8 pre_mode = 0x00;
	s16 err_countx = 0, err_county = 0;
	s16 err_diffx = 0, err_diffy = 0;
	u16 sample_freq = 0;
	u8 circle_freq = 0, circle_num = 0, circle_skip = 0, start_pos=  0;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = copy_from_user(&parameter, user_buf, sizeof(struct ois_circlemode_parameter));
	if (ret) {
		LOG_OIS_ERR("copy_from_user of ois_circle_check_parameter is fail(%d)", ret);
		goto p_err;
	}

	ois_i2c_read_one_byte(client, RUMBA_REG_OIS_MODE, pre_mode, ret);
	LOG_OIS_INF("circle wave: threshhold(%d) errlimit(%d) freq(%d) angle(%d) cicrles(%d) sample(%d) skip(%d) amp(%d, %d) struct_size(%d) mode(%d)",
		parameter.threshhold, parameter.errCountLimit, parameter.circleFrequency, parameter.angleLimit, parameter.circleNum,
		parameter.sampleFrequency, parameter.circleSkipNum, parameter.amplitudeX, parameter.amplitudeY, sizeof(struct ois_circlemode_parameter), pre_mode);

	ois_i2c_write_one_byte(client, RUMBA_REG_OIS_CTRL, OFF, ret);
	if (ret < 0) {
		LOG_OIS_ERR("servo off set fail");
		goto p_err;
	}
	mdelay(2);
	ois_i2c_read_one_byte(ois->client, RUMBA_REG_OIS_STS, rxData[0], ret);
	if (ret < 0 || rxData[0] != RUMBA_IDLE) {
		LOG_OIS_ERR("idle status fail, ret(%d), status(%d)", ret, rxData[0]);
		goto p_err;
	}
    errthreadhold = parameter.threshhold;
    txData[1] = ((errthreadhold >> 8) & 0xFF);
    txData[0] = (errthreadhold & 0xFF);
	ret = ois_i2c_write_block(client, RUMBA_REG_CIRCLE_CHECK_THRES_PLUS, txData, 2);//MCSTH
	if (ret < 0) {
		LOG_OIS_ERR("MCSTH register(0x%04x) fail, ret(%d)", RUMBA_REG_CIRCLE_CHECK_THRES_PLUS, ret);
		goto p_err;
	}
    errthreadhold = 0 - errthreadhold;
    txData[1] = ((errthreadhold >> 8) & 0xFF);
    txData[0] = (errthreadhold & 0xFF);
    ret = ois_i2c_write_block(client, RUMBA_REG_CIRCLE_CHECK_THRES_MINUS, txData, 2);//MCSTH
	if (ret < 0) {
		LOG_OIS_ERR("MCSTH register(0x%04x) fail, ret(%d)", RUMBA_REG_CIRCLE_CHECK_THRES_MINUS, ret);
		goto p_err;
	}
	ois_i2c_write_one_byte(ois->client, RUMBA_REG_CIRCLE_ERROR_COUNT, parameter.errCountLimit, ret);
	if (ret < 0) {
		LOG_OIS_ERR("MCSERRC register(0x0053) fail, ret(%d)", ret);
		goto p_err;
	}
	ois_i2c_write_one_byte(ois->client, RUMBA_REG_CIRCLE_CHECK_FREQ, parameter.circleFrequency, ret);
	if (ret < 0) {
		LOG_OIS_ERR("MCSFREQ register(0x0054) fail, ret(%d)", ret);
		goto p_err;
	}
	ois_i2c_write_one_byte(ois->client, RUMBA_REG_CIRCLE_CHECK_ANGLE, parameter.angleLimit, ret);
	if (ret < 0) {
		LOG_OIS_ERR("MCSANGLE register(0x0055) fail, ret(%d)", ret);
		goto p_err;
	}
	ois_i2c_write_one_byte(ois->client, RUMBA_REG_CIRCLE_CHECK_NUM, parameter.circleNum, ret);
	if (ret < 0) {
		LOG_OIS_ERR("MCSNUM register(0x0057) fail, ret(%d)", ret);
		goto p_err;
	}
	ois_i2c_write_one_byte(ois->client, RUMBA_REG_CIRCLE_START_POS, parameter.startPosition, ret);
	if (ret < 0) {
		LOG_OIS_ERR("MCSNUM register(0x0057) fail, ret(%d)", ret);
		goto p_err;
	}

	sample_freq = (u16)parameter.sampleFrequency;
	txData[1] = ((sample_freq >> 8) & 0xFF);
    txData[0] = (sample_freq & 0xFF);
	ois_i2c_write_block(ois->client, RUMBA_REG_CIRCLE_SAMPLE_FREQ, txData, 2);
	if (ret < 0) {
		LOG_OIS_ERR("sample register(0x006E) fail, ret(%d)", ret);
		goto p_err;
	}
	ois_i2c_write_one_byte(ois->client, RUMBA_REG_CIRCLE_SKIP_COUNT, parameter.circleSkipNum, ret);
	if (ret < 0) {
		LOG_OIS_ERR("skip register(0x0056) fail, ret(%d)", ret);
		goto p_err;
	}
	ois_i2c_write_one_byte(ois->client, RUMBA_REG_CIRCLE_CHECK_XAMP, parameter.amplitudeX, ret);
	if (ret < 0) {
		LOG_OIS_ERR("XMCSAMP register(0x0268) fail, ret(%d)", ret);
		goto p_err;
	}
	ois_i2c_write_one_byte(ois->client, RUMBA_REG_CIRCLE_CHECK_YAMP, parameter.amplitudeY, ret);
	if (ret < 0) {
		LOG_OIS_ERR("YMCSAMP register(0x0269) fail, ret(%d)", ret);
		goto p_err;
	}

	ois_i2c_read_one_byte(ois->client, RUMBA_REG_CIRCLE_CHECK_FREQ, circle_freq, ret);
	ois_i2c_read_one_byte(ois->client, RUMBA_REG_CIRCLE_CHECK_NUM, circle_num, ret);
	ois_i2c_read_one_byte(ois->client, RUMBA_REG_CIRCLE_SKIP_COUNT, circle_skip, ret);
	ois_i2c_read_one_byte(ois->client, RUMBA_REG_CIRCLE_START_POS, start_pos, ret);
	ret = ois_i2c_read_block(client, RUMBA_REG_CIRCLE_SAMPLE_FREQ, rxData, 2);
	sample_freq = OIS_LITTLE_ENDIAN_TRANS2(rxData, 0);
	LOG_OIS_INF("circle init: freq %d num %d skip %d start %d sample %d", circle_freq, circle_num, circle_skip, start_pos, sample_freq);

	ois_i2c_write_one_byte(ois->client, RUMBA_REG_CIRCLE_CHECK_EN, 0x81, ret);
	if (ret < 0) {
		LOG_OIS_ERR("MCCTRL register(0x0050) fail, ret(%d)", ret);
		goto p_err;
	}
	mdelay(200);
	do {
		if (0 == repeatedCnt) {
			LOG_OIS_ERR("wait calibration done timeout!!!");
			ret = -1;
			goto p_err;
		}
		mdelay(500);
		ois_i2c_read_one_byte(ois->client, RUMBA_REG_CIRCLE_CHECK_EN, rxData[0], ret);
		repeatedCnt--;
	} while ((rxData[0] & 0x01) != 0x00);
	ois_i2c_read_one_byte(ois->client, RUMBA_REG_CIRCLE_CHECK_ERR, rxData[0], ret);
	if (ret < 0) {
		LOG_OIS_ERR("read check result fail, ret(%d), status(%d)", ret, rxData[0]);
		goto p_err;
	}
	if ((rxData[0] & 0x03) != 0x00) {
		circleCheckResult = 0x01;
	} else {
		circleCheckResult = 0x00;
	}
	//reset to pre-mode
	ret = rumbas4sw_set_mode(ois, pre_mode, 1);
	ois_i2c_write_one_byte(ois->client, RUMBA_REG_OIS_CTRL, ON, ret);
	LOG_OIS_INF("circles check finished, result(%d)", circleCheckResult);

	//return error info to caller
	ret = ois_i2c_read_block(client, RUMBA_REG_CIRCLE_ERR_COUNTX, rxData, 2);
	err_countx = OIS_LITTLE_ENDIAN_TRANS2(rxData, 0);
	ret = ois_i2c_read_block(client, RUMBA_REG_CIRCLE_ERR_COUNTY, rxData, 2);
	err_county = OIS_LITTLE_ENDIAN_TRANS2(rxData, 0);
	ret = ois_i2c_read_block(client, RUMBA_REG_CIRCLE_ERR_DIFFX, rxData, 2);
	err_diffx = OIS_LITTLE_ENDIAN_TRANS2(rxData, 0);
	ret = ois_i2c_read_block(client, RUMBA_REG_CIRCLE_ERR_DIFFY, rxData, 2);
	err_diffy = OIS_LITTLE_ENDIAN_TRANS2(rxData, 0);
	circleCheckResult = ((err_diffx << 16) & 0xFFFF0000) | (err_diffy & 0x0000FFFF);
	parameter.checkResult = circleCheckResult;
	LOG_OIS_INF("circle:count %d %d diff 0x%x %d 0x%x %d result 0x%x",
		err_countx, err_county, err_diffx, err_diffx, err_diffy, err_diffy, parameter.checkResult);

	ret = copy_to_user(user_buf, &parameter, sizeof(struct ois_circlemode_parameter));
	if (ret < 0) {
		ret = -EINVAL;
		LOG_OIS_ERR("copy_to_user of ois_circle_check_parameter is fail(%d)", ret);
		goto p_err;
	}
p_err:
	return ret;

}


static int rumbas4sw_set_target(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	struct ois_fixmode_parameter parameter = { 0 };
	struct i2c_client *client = ois->client;
	u8  targetx[2] = { 0 }, targety[2] = { 0 };

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = rumbas4sw_set_mode(ois, OIS_FIX_MODE, 0);

	ret = copy_from_user(&parameter, user_buf, sizeof(struct ois_fixmode_parameter));
	if (ret) {
		LOG_OIS_ERR("copy_from_user of ois_fixmode_parameter is fail(%d)", ret);
		goto p_err;
	}

	LOG_OIS_INF("set position(%d, %d)", parameter.targetX, parameter.targetY);
	targetx[1] = ((u16)(parameter.targetX) >> 8) & 0xFF;
	targetx[0] = ((u16)(parameter.targetX)) & 0xFF;

	targety[1] = ((u16)(parameter.targetY) >> 8) & 0xFF;
	targety[0] = ((u16)(parameter.targetY)) & 0xFF;

	ret = ois_i2c_write_block(client, RUMBA_REG_OIS_XTARGET, targetx, 2);
	mdelay(1);
	ret = ois_i2c_write_block(client, RUMBA_REG_OIS_YTARGET, targety, 2);
	mdelay(1);
p_err:
	return ret;

}

static int rumbas4sw_set_pantilt(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	u8 rxData[4] = {0};
	u8 txData[4] = {0};
	u8 val1, val2, val3, val4, val5;
	struct i2c_client *client = ois->client;
	struct ois_pantilt_param pantilt = {0,};
	u8 pre_mode = 0x00;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = copy_from_user(&pantilt, user_buf, sizeof(struct ois_pantilt_param));
	if (ret) {
		LOG_OIS_ERR("copy pantilt fail(%d)", ret);
		goto p_err;
	}

	ois_i2c_read_one_byte(client, RUMBA_REG_CIRCLE_PTAREA2, val1, ret);
	ois_i2c_read_one_byte(client, RUMBA_REG_CIRCLE_PTAREA3, val2, ret);
	ois_i2c_read_one_byte(client, RUMBA_REG_GYRO_PTAREA1_MODE0, val3, ret);
	ois_i2c_read_one_byte(client, RUMBA_REG_GYRO_PTAREA2_MODE0, val4, ret);
	ois_i2c_read_one_byte(client, RUMBA_REG_GYRO_PTAREA3_MODE0, val5, ret);
	ret = ois_i2c_read_block(client, RUMBA_REG_GYRO_PTCOEF_MODE0, rxData, 4);

	ois_i2c_read_one_byte(client, RUMBA_REG_OIS_MODE, pre_mode, ret);
	ois_i2c_write_one_byte(client, RUMBA_REG_OIS_CTRL, OFF, ret);
	mdelay(10);

	LOG_OIS_INF("rumba: before: 0x262(%x), 0x263(%x), 0x324(%x),0x325(%x), 0x326(%x), 0x31C(%x%x%x%x) mode(%d)",
			val1, val2, val3, val4, val5, rxData[3], rxData[2], rxData[1], rxData[0], pre_mode);

	if (ON == pantilt.on) {

	} else if (OFF == pantilt.on) {
		txData[3] = 0x3F;
		txData[2] = 0x80;
		txData[1] = 0x00;
		txData[0] = 0x00;
		ois_i2c_write_one_byte(client, RUMBA_REG_CIRCLE_PTAREA2, 0x64, ret);//100 for 100%
		ois_i2c_write_one_byte(client, RUMBA_REG_CIRCLE_PTAREA3, 0x64, ret);
		ois_i2c_write_one_byte(client, RUMBA_REG_GYRO_PTAREA1_MODE0, 0x64, ret);
		ois_i2c_write_one_byte(client, RUMBA_REG_GYRO_PTAREA2_MODE0, 0x64, ret);
		ois_i2c_write_one_byte(client, RUMBA_REG_GYRO_PTAREA3_MODE0, 0x64, ret);
		ret = ois_i2c_write_block(client, RUMBA_REG_GYRO_PTCOEF_MODE0, txData, 4);//0x3F800000 for 1.0f
	}

	ret = rumbas4sw_set_mode(ois, pre_mode, 1);
	ois_i2c_write_one_byte(ois->client, RUMBA_REG_OIS_CTRL, ON, ret);

p_err:
	ois_i2c_read_one_byte(client, RUMBA_REG_CIRCLE_PTAREA2, val1, ret);
	ois_i2c_read_one_byte(client, RUMBA_REG_CIRCLE_PTAREA3, val2, ret);
	ois_i2c_read_one_byte(client, RUMBA_REG_GYRO_PTAREA1_MODE0, val3, ret);
	ois_i2c_read_one_byte(client, RUMBA_REG_GYRO_PTAREA2_MODE0, val4, ret);
	ois_i2c_read_one_byte(client, RUMBA_REG_GYRO_PTAREA3_MODE0, val5, ret);
	ret = ois_i2c_read_block(client, RUMBA_REG_GYRO_PTCOEF_MODE0, rxData, 4);
	LOG_OIS_INF("rumba: after: 0x262(%x), 0x263(%x), 0x324(%x),0x325(%x), 0x326(%x), 0x31C(%x%x%x%x)",
		val1, val2, val3, val4, val5, rxData[3], rxData[2], rxData[1], rxData[0]);
	return ret;
}

static u16 rumbas4sw_fwCheckSum(const void *addr, u16 size)
{
	u16 checkSum = 0;
	u16 i;
	u16 *pAddr = (u16 *)(addr);
	for (i = 0; i < size / 2; i++) {
		checkSum += pAddr[i];
	}
	return checkSum;
}

static int rumbas4sw_initPID(struct ois *ois)
{
	int ret = 0;
	u8 rxData[4] = {0};
	u8 txData[4] = {0};

	OIS_BUG(!(ois->client));

	ois_i2c_write_one_byte(ois->client, RUMBA_REG_OIS_PIDPARAMINIT, 0x21, ret);
	mdelay(190);
	ois_i2c_read_one_byte(ois->client, RUMBA_REG_OIS_PIDPARAMINIT, rxData[0], ret);

	LOG_OIS_INF("current init pid %d", rxData[0]);

p_err:
	return ret;
}

static int rumbas4sw_flash_params_check(struct i2c_client *client, u32 start_addr, u16 size)
{
	int ret       = 0;
	u32 read_size = RUMBA_BLK_SIZE;
	u32 read_addr = start_addr;
	u8  param_data[RUMBA_BLK_SIZE] = {0x00, };
	u16 paramBlockIdx = 0, printIdx = 0, unitOffset = 0;

	OIS_BUG(!client);
	OIS_BUG(!(size > 0));

	for (paramBlockIdx = 0; paramBlockIdx < size - 1; ) {
		read_addr = start_addr + paramBlockIdx;
		ret = ois_i2c_read_block(client, read_addr, param_data, read_size);
		for (printIdx = 0; printIdx < read_size - 1;) {
			unitOffset = paramBlockIdx + printIdx;
			LOG_OIS_INF("update params: 0x%04x=0x%02x 0x%04x=0x%02x 0x%04x=0x%02x 0x%04x=0x%02x 0x%04x=0x%02x 0x%04x=0x%02x 0x%04x=0x%02x 0x%04x=0x%02x\n",
				(unitOffset), param_data[printIdx], (unitOffset + 1), param_data[printIdx + 1],
				(unitOffset + 2), param_data[printIdx + 2], (unitOffset + 3), param_data[printIdx + 3],
				(unitOffset + 4), param_data[printIdx + 4], (unitOffset + 5), param_data[printIdx + 5],
				(unitOffset + 6), param_data[printIdx + 6], (unitOffset + 7), param_data[printIdx + 7]);
			printIdx += 8;
		}
		paramBlockIdx += read_size;
	}

	return ret;
}

/*write fw to flash*/
static int rumbas4sw_fw_update(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	struct i2c_client *client = ois->client;
	const struct firmware *ois_fw = NULL;
	const u8 *fw_write_data = NULL;
	u32 newVersion = 0;
	u32 currVersion = 0;
	u16 checkSum = 0;
	u8 rxData[4] = {0}, txData[4] = {0};
	u16 block_idx = 0, write_idx = 0;
	const u16 txBufSize = 256;
	u8 settingData[txBufSize];
	int fw_size;
	int froce_version = 0x00;
	u8 pre_mode = 0x00;
	const char *fw_name;

	OIS_BUG(!client);
	OIS_BUG(!(ois->dev));

	LOG_OIS_INF("E");

	client->addr = RUMBAS4SW_SLAVE_ADDR >> 1;

	//mandatory update check
	if (NULL != user_buf) {
		ret = copy_from_user(&froce_version, user_buf, sizeof(int));
		if (ret) {
			LOG_OIS_ERR("copy_from_user fail(%d)", ret);
			goto p_err;
		}
		LOG_OIS_INF("force ois update(0x%04x)", froce_version);
	}

	ois_i2c_read_one_byte(client, RUMBA_REG_OIS_MODE, pre_mode, ret);

	ret = rumbas4sw_flash_params_check(client, RUMBA_PARAMS_BLOCK_ADDR, RUMBA_PARAMS_BLOCK_SIZE);

	//set and wait ic to idle state
	ret = rumbas4sw_set_idle(ois);
	if (!ret) {
		LOG_OIS_ERR("invalid ic status, skip fw update");
		goto p_err;
	}

	// load fw to kernel space
#if defined REAR_MAIN1
	if ('0' == ois->ccm_board_version[0])//pd2135
		fw_name = RUMBAS4SW_OIS_FW_NAME2;
	else
		fw_name = RUMBAS4SW_OIS_FW_NAME1;
#else
	fw_name = RUMBAS4SW_OIS_FW_NAME;
#endif
	ret = request_firmware(&ois_fw, fw_name, ois->dev);
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
	release_firmware(ois_fw);
	LOG_OIS_INF("ois_fw buffer %p, size %d fw %s", fw_write_data, fw_size, fw_name);

	//fw version check
	ret = ois_i2c_read_block(client, RUMBA_REG_OIS_FWVER, rxData, 4);
	currVersion = OIS_LITTLE_ENDIAN_TRANS4(rxData, 0);
	memcpy((void *)(&newVersion), (void *)(&fw_write_data[RUMBA_OIS_FW_ADDR_START]), 4);
	if (currVersion == newVersion) {
		LOG_OIS_INF("no need to update ois FW(%d)", currVersion);
		goto p_err;
	}
	LOG_OIS_INF("current FW %lu, new FW %lu", currVersion, newVersion);

	//update enable with every 256bytes write
	ois_i2c_write_one_byte(client, RUMBA_REG_FWUP_CTRL, 0x75, ret);
	mdelay(55);

	//Write user program data
	for (block_idx = 0; block_idx < (RUMBA_S4SW_FW_SIZE / txBufSize); block_idx++) {
		for (write_idx = 0; write_idx < txBufSize; write_idx++) {
			settingData[write_idx] = (fw_write_data + block_idx * txBufSize)[write_idx];
		}
		ret = ois_i2c_write_block(ois->client, RUMBA_REG_OIS_FLASH_BUF, settingData, txBufSize);
		if (ret) {
			LOG_OIS_ERR("OIS FW UPDATE write program error (%d)", ret);
			goto p_err;
		} else {
			LOG_OIS_INF("flash[%d] write: addr %lu success(%d)", block_idx, (block_idx * txBufSize), ret);
		}
		mdelay(10);
	}

	//check fw update error status
	ret = ois_i2c_read_block(ois->client, RUMBA_REG_FWUP_ERR, rxData, 2);
	if (*(u16 *)rxData != 0x0000) {
		LOG_OIS_ERR("Err ! F/W Update Error Status (0x%04x)", rxData);
		goto p_err;
	}

	//calculate checksum
	checkSum = rumbas4sw_fwCheckSum(fw_write_data, RUMBA_S4SW_FW_SIZE);
	txData[0] = (checkSum & 0x00FF);
	txData[1] = (checkSum & 0xFF00) >> 8;
	txData[2] = 0x00;
	txData[3] = 0x80;
	ret = ois_i2c_write_block(ois->client, RUMBA_REG_FWUP_CHKSUM, txData, 4);
	mdelay(190);

	//check fw update error status again
	ret = ois_i2c_read_block(ois->client, RUMBA_REG_FWUP_ERR, rxData, 2);
	if (*(u16 *)rxData != 0) {
		LOG_OIS_ERR("Err ! F/W Update Error Status (0x%04x)", rxData);
		goto p_err;
	}

	//init PID params
	rumbas4sw_initPID(ois);

	//check new fw version
	ret = ois_i2c_read_block(client, RUMBA_REG_OIS_FWVER, rxData, 4);
	currVersion = OIS_LITTLE_ENDIAN_TRANS4(rxData, 0);
	if (ret || (currVersion != newVersion)) {
		LOG_OIS_ERR("FW update fail(%lu, %lu), ret(%d)", currVersion, newVersion, ret);
		goto p_err;
	}

	//recover pre-mode
	ret = rumbas4sw_set_mode(ois, pre_mode, 1);
	ois_i2c_write_one_byte(ois->client, RUMBA_REG_OIS_CTRL, ON, ret);

	ret = rumbas4sw_flash_params_check(client, RUMBA_PARAMS_BLOCK_ADDR, RUMBA_PARAMS_BLOCK_SIZE);

	LOG_OIS_INF("X version %lu", currVersion);

p_err:
	if (fw_write_data) {
		LOG_OIS_INF("ois fw buffer(%p) release", fw_write_data);
		kfree(fw_write_data);
		fw_write_data = NULL;
	}
	return ret;
}

static int rumbas4sw_status_check(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	int glimitx;
	int glimity;
	int glimitc;
	u8 rxData[4] = { 0 };
	u8 coef[4] = { 0 };
	u16 u16_rxdata = 0x0000;
	u8 spi = 0x00;
	struct i2c_client     *client      = ois->client;
	struct ois_flash_info *flash_info = ois->flash_info;
	int active_ois;
	OIS_BUG(!client);
	OIS_BUG(!flash_info);

	ret = ois_i2c_read(client, RUMBA_REG_OIS_GYRO_RAW_X, &u16_rxdata);
	flash_info->imuInfo.gyroRawX = (s16)OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);
	ret = ois_i2c_read(client, RUMBA_REG_OIS_GYRO_RAW_Y, &u16_rxdata);
	flash_info->imuInfo.gyroRawY = (s16)OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);
	ret = ois_i2c_read(client, RUMBA_REG_OIS_GYRO_RAW_Z, &u16_rxdata);
	flash_info->imuInfo.gyroRawZ = (s16)OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);
	
	ret = ois_i2c_read(client, RUMBA_REG_OIS_ACC_RAW_X, &u16_rxdata);
	flash_info->imuInfo.accRawX = (s16)OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);
	ret = ois_i2c_read(client, RUMBA_REG_OIS_ACC_RAW_Y, &u16_rxdata);
	flash_info->imuInfo.accRawY = (s16)OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);
	ret = ois_i2c_read(client, RUMBA_REG_OIS_ACC_RAW_Z, &u16_rxdata);
	flash_info->imuInfo.accRawZ = (s16)OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);

	ret = ois_i2c_read(client, RUMBA_REG_OIS_GYRO_TAR_X, &u16_rxdata);
	flash_info->targetInfo.gyroTargetX = (s16)OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);
	ret = ois_i2c_read(client, RUMBA_REG_OIS_GYRO_TAR_Y, &u16_rxdata);
	flash_info->targetInfo.gyroTargetY = (s16)OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);
	ret = ois_i2c_read(client, RUMBA_REG_OIS_ACC_TAR_X, &u16_rxdata);
	flash_info->targetInfo.accTargetX = (s16)OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);
	ret = ois_i2c_read(client, RUMBA_REG_OIS_ACC_TAR_Y, &u16_rxdata);
	flash_info->targetInfo.accTargetY = (s16)OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);
	ret = ois_i2c_read(client, RUMBA_REG_OIS_SRX, &u16_rxdata);
	flash_info->targetInfo.totalTargetX = (s16)OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);
	ret = ois_i2c_read(client, RUMBA_REG_OIS_SRY, &u16_rxdata);
	flash_info->targetInfo.totalTargetY = (s16)OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);
	ret = ois_i2c_read(client, RUMBA_REG_OIS_HALL_X, &u16_rxdata);
	flash_info->hallInfo.lensPosX = OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);
	ret = ois_i2c_read(client, RUMBA_REG_OIS_HALL_Y, &u16_rxdata);
	flash_info->hallInfo.lensPosY = OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);
	ois_i2c_read_one_byte(client, RUMBA_REG_OIS_MONITOR_MODE, spi, ret);
	ois_i2c_read_one_byte(client, RUMBA_REG_OIS_ACTIVE_ON, rxData[0], ret);
	active_ois = rxData[0];
	ois_i2c_read_one_byte(client, RUMBA_REG_OIS_CTRL, rxData[0], ret);
	flash_info->serveOn = rxData[0];
	ois_i2c_read_one_byte(client, RUMBA_REG_OIS_MODE, rxData[0], ret);
	flash_info->mode = rxData[0];
	ois_i2c_read_one_byte(client, RUMBA_REG_OIS_ACCCTRL, rxData[0], ret);
	flash_info->accOn = rxData[0];
	ois_i2c_read_one_byte(client, RUMBA_REG_OIS_STS, rxData[0], ret);
	flash_info->status = rxData[0];
	ois_i2c_read_one_byte(client, RUMBA_REG_OIS_STATIONARY, rxData[0], ret);
	flash_info->tripodFlag = rxData[0];

	ret = ois_i2c_read(client, RUMBA_REG_GYRO_TARGET_LIMITX, &u16_rxdata);
	glimitx = OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);
	ret = ois_i2c_read(client, RUMBA_REG_GYRO_TARGET_LIMITY, &u16_rxdata);
	glimity = OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);
	ret = ois_i2c_read(client, RUMBA_REG_OIS_CIRCLELIMIT, &u16_rxdata);
	glimitc = OIS_LITTLE_ENDIAN_TRANS2_1(u16_rxdata);
	ret = ois_i2c_read_block(client, RUMBA_REG_OIS_GLCOEF3, coef, 4);

	LOG_OIS_INF("status_check info:ois en(%d), mode(%d), status(%d), tripodeFlag(%d), spi(0x%x) acc(%d), gyroRaw(%d, %d, %d), accRaw(%d, %d, %d), gyroTarget(%d, %d), accTarget(%d, %d), totalTarget(%d, %d), gyro limit(%d, %d, %d), coef3(0x%x%x%x%x), ACTIVE(%d)",
		flash_info->serveOn, flash_info->mode, flash_info->status, flash_info->tripodFlag, spi, flash_info->accOn,
		flash_info->imuInfo.gyroRawX, flash_info->imuInfo.gyroRawY, flash_info->imuInfo.gyroRawZ, flash_info->imuInfo.accRawX,
		flash_info->imuInfo.accRawY, flash_info->imuInfo.accRawZ, flash_info->targetInfo.gyroTargetX, 
		flash_info->targetInfo.gyroTargetY, flash_info->targetInfo.accTargetX, flash_info->targetInfo.accTargetY, 
		flash_info->targetInfo.totalTargetX, flash_info->targetInfo.totalTargetY, glimitx, glimity, glimitc, coef[3], coef[2], coef[1], coef[0], active_ois);
p_err:
	return ret;
}

//temp func for pd2085/pd2120
#if defined(CONFIG_MTK_CAM_PD2085) || defined(CONFIG_MTK_CAM_PD2120A)
static void format_otp_data_pd2085(struct ois_otp_info *ois_otp, u8 *otp_buf)
{
	s16                 s16_data    = 0x0000;
	u8                  circle_diff = 0;
	u8                  mirror      = 0;
	u8                  *sn_data    = NULL;
	u8                  *fuse_id    = NULL;
	u8                  data_size   = 0;
	u8                  idx         = 0;

	ois_otp->fwVersion = OIS_BIG_ENDIAN_TRANS4(otp_buf, OTP2ARR(0x2A07));
	ois_otp->gyroGainX = OIS_BIG_ENDIAN_TRANS4(otp_buf, OTP2ARR(0x2A34));
	ois_otp->gyroGainY = OIS_BIG_ENDIAN_TRANS4(otp_buf, OTP2ARR(0x2A38));
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, OTP2ARR(0x2A30));
	ois_otp->gyroOffsetX = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, OTP2ARR(0x2A32));
	ois_otp->gyroOffsetY = s16_data;
	ois_otp->hallMechCenterX = OIS_BIG_ENDIAN_TRANS2(otp_buf, OTP2ARR(0x2A25));
	ois_otp->hallMechCenterY = OIS_BIG_ENDIAN_TRANS2(otp_buf, OTP2ARR(0x2A27));
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, OTP2ARR(0x2A1F));
	ois_otp->hallXMin = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, OTP2ARR(0x2A23));
	ois_otp->hallYMin = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, OTP2ARR(0x2A1D));
	ois_otp->hallXMax = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, OTP2ARR(0x2A21));
	ois_otp->hallYMax = s16_data;
	ois_otp->tiltSRX = OIS_BIG_ENDIAN_TRANS2(otp_buf, OTP2ARR(0x2A44));
	ois_otp->tiltSRY = OIS_BIG_ENDIAN_TRANS2(otp_buf, OTP2ARR(0x2A46));
	ois_otp->accSRX = OIS_BIG_ENDIAN_TRANS2(otp_buf, OTP2ARR(0x2A64));
	ois_otp->accSRY = OIS_BIG_ENDIAN_TRANS2(otp_buf, OTP2ARR(0x2A66));
	circle_diff = otp_buf[OTP2ARR(0x2A48)];
	mirror      = otp_buf[OTP2ARR(0x2A49)];
	ois_otp->icType = RUMBAS4SW;

	data_size = 0x2A8C - 0x2A7B + 1;
	fuse_id = kzalloc(sizeof(u8) * data_size * 2 + 2, GFP_KERNEL);
	if (!fuse_id) {
		LOG_OIS_ERR("fuse id kzalloc failed\n");
		goto p_err;
	}
	for (idx = 0; idx < data_size; idx++)
		sprintf(&fuse_id[idx * 2], "%02x", otp_buf[0x0020 + 1 + idx]);

	data_size = 0x2AAE - 0x2AA0 + 1;
	sn_data = kzalloc(sizeof(u8) * data_size * 2 + 2, GFP_KERNEL);
	if (!sn_data) {
	      LOG_OIS_ERR("sn data kzalloc failed\n");
	      goto p_err;
	}
	for (idx = 0; idx < data_size; idx++)
		sprintf(&sn_data[idx * 2], "%02x", otp_buf[0x0045 + 1 + idx]);

	LOG_OIS_INF("ois otp:sn(0x%s) fuseId(0x%s) fwVer(%d) pola(%d %d %d %d) gyroGain(0x%x, 0x%x) gyroOffset(%d,%d) hall(%d,%d,%d,%d,%d,%d) SR(%d, %d, %d, %d)  circleResult(%d), mirror(%d)",
		sn_data, fuse_id, ois_otp->fwVersion, ois_otp->hallPolarity, ois_otp->gyroOrient, ois_otp->gyroPolarX, ois_otp->gyroPolarY,
		ois_otp->gyroGainX, ois_otp->gyroGainY, ois_otp->gyroOffsetX, ois_otp->gyroOffsetY,
		ois_otp->hallXMin, ois_otp->hallXMax, ois_otp->hallMechCenterX, ois_otp->hallYMin, ois_otp->hallYMax, ois_otp->hallMechCenterY,
		ois_otp->tiltSRX, ois_otp->tiltSRY, ois_otp->accSRX, ois_otp->accSRY, circle_diff, mirror);
p_err:
	if (fuse_id)
		kfree(fuse_id);
	if (sn_data)
		kfree(sn_data);
}
#elif defined(CONFIG_MTK_CAM_PD2135)
//temp func for pd2133/pd2135
static void format_otp_data_pd2135(struct ois_otp_info *ois_otp, u8 *otp_buf)
{
	s16                 s16_data    = 0x0000;
	u8                  circle_result = 0;
	u8                  mirror      = 0;
	u8                  *sn_data    = NULL;
	u8                  *fuse_id    = NULL;
	u8                  data_size   = 0;
	u8                  idx         = 0;
	u8                  fpc_flag    = 0x00;
	u8                  fpc_type    = 0x00;

	ois_otp->fwVersion = OIS_BIG_ENDIAN_TRANS4(otp_buf, 0x22A1);
	ois_otp->gyroGainX = OIS_BIG_ENDIAN_TRANS4(otp_buf, 0x22CE);
	ois_otp->gyroGainY = OIS_BIG_ENDIAN_TRANS4(otp_buf, 0x22D2);
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x22CA);
	ois_otp->gyroOffsetX = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x22CC);
	ois_otp->gyroOffsetY = s16_data;
	ois_otp->hallMechCenterX = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x22BF);
	ois_otp->hallMechCenterY = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x22C1);
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x22B9);
	ois_otp->hallXMin = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x22BD);
	ois_otp->hallYMin = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x22B7);
	ois_otp->hallXMax = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x22BB);
	ois_otp->hallYMax = s16_data;
	ois_otp->tiltSRX = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x22D6);
	ois_otp->tiltSRY = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x22D8);
	ois_otp->accSRX = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x22DA);
	ois_otp->accSRY = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x22DC);
	ois_otp->gyroPolarX = otp_buf[0x22E7];
	ois_otp->gyroPolarY = otp_buf[0x22E8];
	ois_otp->gyroOrient = otp_buf[0x22E9];
	ois_otp->hallPolarity = otp_buf[0x22A5];
	fpc_flag = otp_buf[0x0014];
	fpc_type = otp_buf[0x2422];
	mirror = otp_buf[0x22E6];
	circle_result = otp_buf[0x2421];

	ois_otp->icType = RUMBAS4SW;

	data_size = 0x2A8C - 0x2A7B + 1;
	fuse_id = kzalloc(sizeof(u8) * data_size * 2 + 2, GFP_KERNEL);
	if (!fuse_id) {
		LOG_OIS_ERR("fuse id kzalloc failed\n");
		goto p_err;
	}
	for (idx = 0; idx < data_size; idx++)
		sprintf(&fuse_id[idx * 2], "%02x", otp_buf[0x0020 + 1 + idx]);

	data_size = 0x2AAE - 0x2AA0 + 1;
	sn_data = kzalloc(sizeof(u8) * data_size * 2 + 2, GFP_KERNEL);
	if (!sn_data) {
	      LOG_OIS_ERR("sn data kzalloc failed\n");
	      goto p_err;
	}
	for (idx = 0; idx < data_size; idx++)
		sprintf(&sn_data[idx * 2], "%02x", otp_buf[0x0045 + 1 + idx]);

	LOG_OIS_INF("fpc flag %d type %d circleResult(%d), mirror(%d)", fpc_flag, fpc_type, circle_result, mirror);

	LOG_OIS_INF("ois otp:sn(0x%s) fuseId(0x%s) fwVer(%d) pola(%d %d %d %d) gyroGain(0x%x, 0x%x) gyroOffset(%d,%d) hall(%d,%d,%d,%d,%d,%d) SR(%d, %d, %d, %d)",
		sn_data, fuse_id, ois_otp->fwVersion, ois_otp->hallPolarity, ois_otp->gyroOrient, ois_otp->gyroPolarX, ois_otp->gyroPolarY,
		ois_otp->gyroGainX, ois_otp->gyroGainY, ois_otp->gyroOffsetX, ois_otp->gyroOffsetY,
		ois_otp->hallXMin, ois_otp->hallXMax, ois_otp->hallMechCenterX, ois_otp->hallYMin, ois_otp->hallYMax, ois_otp->hallMechCenterY,
		ois_otp->tiltSRX, ois_otp->tiltSRY, ois_otp->accSRX, ois_otp->accSRY);
p_err:
	if (fuse_id)
		kfree(fuse_id);
	if (sn_data)
		kfree(sn_data);
}
#endif

static int rumbas4sw_format_otp_data(struct ois *ois, void __user *user_buf)
{
	int                 ret         = 0;
	u8                  *otp_buf    = NULL;
	struct ois_otp_info *ois_otp    = NULL;

	OIS_BUG(!user_buf);
	OIS_BUG(!(ois->otp_info));

	ois_otp  = ois->otp_info;

	ois_otp->inited = 0x00;

#if defined(CONFIG_MTK_CAM_PD2085) || defined(CONFIG_MTK_CAM_PD2120A)
	otp_buf = vivo_otp_data_imx598pd2085;
	format_otp_data_pd2085(ois_otp, otp_buf);
#elif defined(CONFIG_MTK_CAM_PD2135)
	if ('0' == ois->ccm_board_version[0])//pd2135
		otp_buf = vivo_otp_data_imx766pd2135;
	else//pd2133
		otp_buf = vivo_otp_data_imx766pd2133;
	format_otp_data_pd2135(ois_otp, otp_buf);
#endif

	ret = copy_to_user(user_buf, ois_otp, sizeof(struct ois_otp_info));
	if (ret) {
		LOG_OIS_ERR("fail to copy otp info, ret(%d)\n", ret);
		goto p_err;
	}

	ois_otp->inited = 0x01;
p_err: 
	ois_otp = NULL;
	return ret;
}

static int rumbas4sw_set_stroke_limit(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	struct ois_stroke_limit_parameter parameter = { 0 };
	struct i2c_client     *client      = ois->client;
	u8 rxData[2] = { 0 }, txData[2] = { 0 };
	u8 mode;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);


	ret = copy_from_user(&parameter, user_buf, sizeof(struct ois_stroke_limit_parameter));
	if (ret) {
		LOG_OIS_ERR("copy_from_user of rumbas4sw_set_angle_limit is fail(%d)", ret);
		goto p_err;
	}

	LOG_OIS_INF("set angle limit enable(%d) axisX(%d), axisY(%d), axisCircle(%d)",
		parameter.enable, parameter.axisX, parameter.axisY, parameter.circle);

	ois_i2c_write_one_byte(client, RUMBA_REG_OIS_CTRL, OFF, ret);
	mdelay(20);
	txData[1] = (((u16)(parameter.circle)) >> 8) & 0xFF;
	txData[0] = (((u16)(parameter.circle)) )&0xFF;
	ret = ois_i2c_write_block(client, RUMBA_REG_OIS_CIRCLELIMIT, txData, 2);
	txData[1] = (((u16)(parameter.axisX))	>> 8) & 0xFF;
	txData[0] = (((u16)(parameter.axisX)) )&0xFF;
	ret = ois_i2c_write_block(client, RUMBA_REG_GYRO_TARGET_LIMITX, txData, 2);
	txData[1] = (((u16)(parameter.axisY))	>> 8) & 0xFF;
	txData[0] = (((u16)(parameter.axisY)) )&0xFF;
	ret = ois_i2c_write_block(client, RUMBA_REG_GYRO_TARGET_LIMITY, txData, 2);
	
	ois_i2c_write_one_byte(client, RUMBA_REG_OIS_CTRL, ON, ret);
	mdelay(1);
	ois_i2c_read_one_byte(client, RUMBA_REG_OIS_MODE, mode, ret);

	LOG_OIS_INF("X ois mode %d", mode);
p_err:
	return ret;

}

static int rumbas4sw_set_smooth(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	u8 rxData[4] = { 0 }, txData[4] = {0};
	struct i2c_client *client = ois->client;
	struct ois_smooth_info smooth = { 0, };
	u32 step1 = 0, step2 = 0;

	OIS_BUG(!client);

	ret = copy_from_user(&smooth, user_buf, sizeof(int));
	if (ret) {
		LOG_OIS_ERR("copy_from_user fail(%d)", ret);
		goto p_err;
	}

	ret = ois_i2c_read_block(client, RUMBA_REG_MODEX_TO_CENTER_TIME, rxData, 4);
	step1 = OIS_LITTLE_ENDIAN_TRANS4(rxData, 0);
	ret = ois_i2c_read_block(client, RUMBA_REG_CENTER_TO_MODEX_TIME, rxData, 4);
	step2 = OIS_LITTLE_ENDIAN_TRANS4(rxData, 0);

	LOG_OIS_INF("E smooth(%d 0x%x %d) before(0x%x 0x%x)",
		smooth.on, smooth.step, smooth.delay, step1, step2);

	if (MANUAL_ON == smooth.on) {
		txData[3] = (smooth.step >> 24) & 0xFF;
		txData[2] = (smooth.step >> 16) & 0xFF;
		txData[1] = (smooth.step >> 8) & 0xFF;
		txData[0] = (smooth.step) & 0xFF;
		ret = ois_i2c_write_block(client, RUMBA_REG_MODEX_TO_CENTER_TIME, txData, 4);
		ret = ois_i2c_write_block(client, RUMBA_REG_CENTER_TO_MODEX_TIME, txData, 4);
		if (ret < 0) {
			LOG_OIS_ERR("smooth change to set failed (%d)", ret);
		}
	} else if (DEFAULT_ON == smooth.on) {
		txData[3] = 0x3B;
		txData[2] = 0x03;
		txData[1] = 0x12;
		txData[0] = 0x00;// 1 second
		ret = ois_i2c_write_block(client, RUMBA_REG_MODEX_TO_CENTER_TIME, txData, 4);
		ret = ois_i2c_write_block(client, RUMBA_REG_CENTER_TO_MODEX_TIME, txData, 4);
		if (ret < 0) {
			LOG_OIS_ERR("smooth change to set failed (%d)", ret);
		}
	}

	ret = ois_i2c_read_block(client, RUMBA_REG_MODEX_TO_CENTER_TIME, rxData, 4);
	step1 = OIS_LITTLE_ENDIAN_TRANS4(rxData, 0);
	ret = ois_i2c_read_block(client, RUMBA_REG_CENTER_TO_MODEX_TIME, rxData, 4);
	step2 = OIS_LITTLE_ENDIAN_TRANS4(rxData, 0);
	LOG_OIS_INF("X smooth(0x%x 0x%x)", step1, step2);
p_err:
	return ret;
}

static int rumbas4sw_act_stroke_limit(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	int actois = 0;
	struct i2c_client     *client      = ois->client;
	u8 rxData[2] = { 0 }, txData[2] = { 0 };
	u8 needact;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);
#if defined REAR_MAIN1
	needact = 1;
#else
	needact = 0;
#endif
	if(!needact) {
		LOG_OIS_INF("this project dont support actois");
		return ret;
	}
	ret = copy_from_user(&actois, user_buf, sizeof(int));
	if (ret) {
		LOG_OIS_ERR("copy_from_user of rumbas4sw_set_angle_limit is fail(%d)", ret);
		goto p_err;
	}
	ret = ois_i2c_read_block(client, RUMBA_REG_OIS_GLCOEF3, rxData, 4);
	LOG_OIS_INF("X before coef3: 0x%x%x%x%x  actois: %d", rxData[3], rxData[2], rxData[1], rxData[0], actois);

	if(actois) {
		ois_i2c_write_one_byte(client, RUMBA_REG_OIS_ACTIVE_ON, ON, ret);
		rxData[3] = 0x3F; 
		rxData[2] = 0x2B;
		rxData[1] = 0x85;
		rxData[0] = 0x1F;// 1 second
		ret = ois_i2c_write_block(client, RUMBA_REG_OIS_GLCOEF3, rxData, 4);
	} else {
		ois_i2c_write_one_byte(client, RUMBA_REG_OIS_ACTIVE_ON, OFF, ret);
		rxData[3] = 0x3F; 
		rxData[2] = 0x80;
		rxData[1] = 0x00;
		rxData[0] = 0x00;// 1 second
		ret = ois_i2c_write_block(client, RUMBA_REG_OIS_GLCOEF3, rxData, 4);
	}
	ret = ois_i2c_read_block(client, RUMBA_REG_OIS_GLCOEF3, rxData, 4);
	LOG_OIS_INF("X after coef3: 0x%x%x%x%x  actois: %d", rxData[3], rxData[2], rxData[1], rxData[0], actois);
p_err:
	return ret;

}


static struct ois_core_ops rumbas4sw_ois_ops = {
	.ois_init = rumbas4sw_init,
	.ois_deinit = rumbas4sw_deinit,
	.ois_stream_on = rumbas4sw_stream_on,
	.ois_stream_off = rumbas4sw_stream_off,
	.ois_get_mode = rumbas4sw_get_mode,
	.ois_set_mode = rumbas4sw_set_mode,
	.ois_fw_update = rumbas4sw_fw_update,
	.ois_get_fw_version = rumbas4sw_get_fw_version,
	.ois_get_gyro_offset = rumbas4sw_get_gyro_offset,
	.ois_set_offset_calibration = rumbas4sw_set_offset_calibration,
	.ois_get_gyro_gain = rumbas4sw_get_gyro_gain,
	.ois_set_gyro_gain = rumbas4sw_set_gyro_gain,
	.ois_flash_save = rumbas4sw_flash_save,
	.ois_set_acc = rumbas4sw_set_acc,
	.ois_set_target = rumbas4sw_set_target,
	.ois_get_init_info = rumbas4sw_get_init_info,
	.ois_status_check = rumbas4sw_status_check,
	.ois_init_vsync_thread = rumbas4sw_init_vsync_thread,
	.ois_deinit_vsync_thread = rumbas4sw_deinit_vsync_thread,
	.ois_vsync_signal = rumbas4sw_vsync_signal,
	.ois_get_lens_info = rumbas4sw_get_lens_info,
	.ois_format_otp_data = rumbas4sw_format_otp_data,
	.ois_set_sinewave = rumbas4sw_set_sinewave,
	.ois_set_stroke_limit = rumbas4sw_set_stroke_limit,
	.ois_act_stroke_limit = rumbas4sw_act_stroke_limit,
	.ois_set_pantilt = rumbas4sw_set_pantilt,
	.ois_set_circlewave = rumbas4sw_set_circlewave,
	.ois_set_smooth = rumbas4sw_set_smooth,
	.ois_log_control = rumbas4sw_log_control,
	.ois_dependency_init = rumbas4sw_dependency_init,
};

/*ic entry expose to ois_core*/
void rumbas4sw_get_ops(struct ois *ois) 
{
	ois->ops = &rumbas4sw_ois_ops;
}
