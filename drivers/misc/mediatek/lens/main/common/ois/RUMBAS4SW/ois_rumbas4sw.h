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

#ifndef OIS_RUMBAS4SW_H
#define OIS_RUMBAS4SW_H

#include "ois_core.h"

#define RUMBAS4SW_OIS_FW_NAME "ois_rear_main.bin"
#define RUMBAS4SW_OIS_FW_NAME1 "ois_rear_main_1.bin"
#define RUMBAS4SW_OIS_FW_NAME2 "ois_rear_main_2.bin"

/* control registers define*/
#define RUMBA_REG_OIS_CTRL 0x0000
#define RUMBA_REG_OIS_STS 0x0001
#define RUMBA_REG_OIS_MODE 0x0002
#define RUMBA_REG_OIS_DATAWRITE 0x0003
#define RUMBA_REG_OIS_ERR 0x0004
#define RUMBA_REG_FWUP_ERR 0x0006
#define RUMBA_REG_FWUP_CHKSUM 0x0008
#define RUMBA_REG_FWUP_CTRL 0x000C
#define RUMBA_REG_OIS_GCC_CTRL 0x0014
#define RUMBA_REG_OIS_GGADJ_CTRL 0x0015
#define RUMBA_REG_OIS_XTARGET 0x0022
#define RUMBA_REG_OIS_YTARGET 0x0024
#define RUMBA_FLSWRTRESULT 0x0027
#define RUMBA_REG_OIS_STATIONARY 0x0035
#define RUMBA_REG_OIS_ACTIVE_ON 0x0040
#define RUMBA_REG_OIS_FWINFO_UPDATE 0x0080
#define RUMBA_REG_OIS_FWVER 0x00FC //4bytes
#define RUMBA_REG_OIS_EIS_BUF 0x0110 //EIS data buffer start addr
#define RUMBA_REG_OIS_FLASH_BUF 0x0100 //start addr of code flash data
#define RUMBA_REG_OIS_SRX 0x05A8
#define RUMBA_REG_OIS_SRY 0x05AA
#define RUMBA_REG_OIS_DFLSCTRL 0x000D //user data area control reg
#define RUMBA_REG_OIS_DFLSCMD 0x000E //user data area command reg
#define RUMBA_REG_OIS_PIDPARAMINIT 0x0036 //PID initialize reg
#define RUMBA_REG_OIS_SINCTRL 0x0018 // sinewave axis enable
#define RUMBA_REG_OIS_SINFREQ 0x0019 // sinewave frequency, 1-10
#define RUMBA_REG_OIS_SINAMPX 0x0038 // sinewave amplitude
#define RUMBA_REG_OIS_SINAMPY 0x003A // sinewave amplitude
#define RUMBA_REG_MODEX_TO_CENTER_TIME 0x02CC //modex to center mode smooth, 4bytes
#define RUMBA_REG_CENTER_TO_MODEX_TIME 0x02D0

/*Gyro related registers*/
#define RUMBA_REG_OIS_GYRO_RAW_X 0x0082
#define RUMBA_REG_OIS_GYRO_RAW_Y 0x0084
#define RUMBA_REG_OIS_GYRO_RAW_Z 0x0076
#define RUMBA_REG_OIS_GGX 0x0254 //gyro gainx
#define RUMBA_REG_OIS_GGY 0x0258
#define RUMBA_REG_OIS_GOFFSETX 0x0248 //gyro offsetx
#define RUMBA_REG_OIS_GOFFSETY 0x024A
#define RUMBA_REG_OIS_GLCOEF3 0x03A8 //gyro limit for mode3
#define RUMBA_REG_OIS_GYRO_TAR_X 0x0086
#define RUMBA_REG_OIS_GYRO_TAR_Y 0x0088
#define RUMBA_REG_OIS_X9B 0x027C //PID filter of X,4bytes
#define RUMBA_REG_OIS_Y9B 0x0294
#define RUMBA_REG_OIS_MONITOR_MODE 0x02FD
#define RUMBA_REG_GYRO_ORIENT 0x0242

/*Acc related registers*/
#define RUMBA_REG_OIS_ACCCTRL 0x0558
#define RUMBA_REG_OIS_ACCDIS 0x0588 //acc subject distance
#define RUMBA_REG_OIS_ACC_RAW_X 0x05A0
#define RUMBA_REG_OIS_ACC_RAW_Y 0x05A2
#define RUMBA_REG_OIS_ACC_RAW_Z 0x0078
#define RUMBA_REG_OIS_ACC_TAR_X 0x05A4
#define RUMBA_REG_OIS_ACC_TAR_Y 0x05A6
#define RUMBA_REG_OIS_ACCGAINX 0x0584 //acc gainx
#define RUMBA_REG_OIS_ACCGAINY 0x05E4
#define RUMBA_REG_OIS_ACCFGAINX 0x05F0 //acc fine gainx
#define RUMBA_REG_OIS_ACCFGAINY 0x05F4
#define RUMBA_REG_ACC_MIRROR 0x0559

/*Hall related registers*/
#define RUMBA_REG_XMECHCENTER 0x021A
#define RUMBA_REG_YMECHCENTER 0x021C
#define RUMBA_REG_OIS_HALLXMAX 0x0212
#define RUMBA_REG_OIS_HALLXMIM 0x0214
#define RUMBA_REG_OIS_HALLYMAX 0x0216
#define RUMBA_REG_OIS_HALLYMIN 0x0218
#define RUMBA_REG_HALLX_OFFSET 0x020E //hall offset of x-axis
#define RUMBA_REG_HALLY_OFFSET 0x020F
#define RUMBA_REG_HALLX_BIAS 0x0210 //hall bias value of X
#define RUMBA_REG_HALLY_BIAS 0x0211
#define RUMBA_REG_OIS_HALL_X 0x0072
#define RUMBA_REG_OIS_HALL_Y 0x0074

/*circle check registers*/
#define RUMBA_REG_CIRCLE_CHECK_EN 0x0050  // module circle check enable
#define RUMBA_REG_CIRCLE_CHECK_ERR 0x0051 // module circle check error register
#define RUMBA_REG_CIRCLE_CHECK_THRES_PLUS 0x006A // 16bit,threshold of the circle check diff(target<-->hall)
#define RUMBA_REG_CIRCLE_CHECK_THRES_MINUS 0x006C // 16bit, threshold of the circle check diff(target<-->hall)
#define RUMBA_REG_CIRCLE_ERROR_COUNT 0x0053 // error count beyond threshold,default=0
#define RUMBA_REG_CIRCLE_CHECK_FREQ 0x0054  // indicate how many circles we draw for each second,default=4
#define RUMBA_REG_CIRCLE_CHECK_NUM 0x0057 // number of circles measured during the test,default=1
#define RUMBA_REG_CIRCLE_CHECK_ANGLE 0x0055 // circle check amplitude angle
#define RUMBA_REG_CIRCLE_SKIP_COUNT 0x0056 // skip count for beginning several circles before actual test,defaul=1
#define RUMBA_REG_CIRCLE_START_POS 0x005B //start position of circle pre -process,default=27,means pre-process is from 270°-360°
#define RUMBA_REG_CIRCLE_PHASE_DIFF 0x0068 // 16bit phase diff between x&y axis
#define RUMBA_REG_CIRCLE_SAMPLE_FREQ 0x006E //16bit, hall readback sampling frequency, hz/s,must be a division of 2000s,default=50
#define RUMBA_REG_CIRCLE_CHECK_XAMP 0x0268 // module circle check amplitude of x-axis,default=0
#define RUMBA_REG_CIRCLE_CHECK_YAMP 0x0269
#define RUMBA_REG_CIRCLE_ERR_COUNTX 0x00E4 //count exit the threshold
#define RUMBA_REG_CIRCLE_ERR_COUNTY 0x00E6
#define RUMBA_REG_CIRCLE_ERR_DIFFX 0x00E8 //max diff between target&hall
#define RUMBA_REG_CIRCLE_ERR_DIFFY 0x00EA

/*pantilt related registers*/
#define RUMBA_REG_OIS_CIRCLELIMIT    0x0252 //2bytes
#define RUMBA_REG_OIS_ACC_LIMIT      0x058C //2bytes
#define RUMBA_REG_GYRO_TARGET_LIMITX 0x025C //4bytes
#define RUMBA_REG_GYRO_TARGET_LIMITY 0x025E
#define RUMBA_REG_CIRCLE_PTAREA2     0x0262 //circular pantilt correction area 2
#define RUMBA_REG_CIRCLE_PTAREA3     0x0263
#define RUMBA_REG_CIRCLE_PTSTEP      0x0264 //cirlular pantilt correction step,4bytes
#define RUMBA_REG_GYRO_PTAREA1_MODE0 0x0324 //gyro pantilt correction area 1 for mode0(0x10)
#define RUMBA_REG_GYRO_PTAREA2_MODE0 0x0325
#define RUMBA_REG_GYRO_PTAREA3_MODE0 0x0326
#define RUMBA_REG_GYRO_PTCOEF_MODE0  0x031C //gyro pantilt correction coefficient for mode0(0x10),4 bytes
#define RUMBA_REG_GYRO_PTSTEP_MODE0  0x0320 //gyro pantilt correction step for mode0(0x10),4 bytes
#define RUMBA_REG_GYRO_PTCOEF_MODE1  0x034C
#define RUMBA_REG_GYRO_PTSTEP_MODE1  0x0350
#define RUMBA_REG_GYRO_PTCOEF_MODE2  0x037C
#define RUMBA_REG_GYRO_PTSTEP_MODE2  0x0380
#define RUMBA_REG_GYRO_PTCOEF_MODE3  0x03AC
#define RUMBA_REG_GYRO_PTSTEP_MODE3  0x03B0
#define RUMBA_REG_GYRO_PTCOEF_MODE4  0x03DC
#define RUMBA_REG_GYRO_PTSTEP_MODE4  0x03E0


#define GYRO_COMMCHK_EN   0x08
#define GYRO_GAINX_EN     0x02
#define GYRO_GAINY_EN     0x04
#define OIS_RESET_REQ     0x80
#define FWUP_CTRL_256_SET 0x07
#define OFF               0x00
#define ON                0x01
#define ENABLE            0x01

/* common define */
#define RUMBA_S4SW_FW_SIZE        28672
#define RUMBA_OIS_FW_ADDR_START   0x6FF4  // 0x7FF4-0x7FF7
#define ENGINEER_MODE_OIS         (1U<<3)
#define RUMBA_BLK_SIZE            128     // block read/write size
#define RUMBA_PARAMS_BLOCK_ADDR   0x0000  //flash params block
#define RUMBA_PARAMS_BLOCK_SIZE   0x0600


/*ois mode*/
enum rumba_s4sw_mode {
	RUMBA_STILL_MODE = 0x10,
	RUMBA_VIDEO_MODE = 0x13, // need motify zhaoyuanpei
	RUMBA_ZOOM_MODE = 0x11,  // need motify
	RUMBA_CENTERING_MODE = 0x05,
	RUMBA_FIX_MODE = 0x02,
	RUMBA_SINE_WAVE_MODE = 0x03,
	RUMBA_SQUARE_WAVE_MODE = 0x04,
	RUMBA_CIR_WAVE_MODE = 0x03,
};

enum rumba_s4sw_status {
	RUMBA_INIT = 0x00,
	RUMBA_IDLE = 0x01,
	RUMBA_RUN = 0x02,
	RUMBA_HALL_CAL = 0x03,
	RUMBA_GYRO_CAL = 0x05,
	RUMBA_PWM_DUTY_FIXED = 0x08, //linear Drive value fixed state
	RUMBA_DFLS_UPDATE = 0x09, //user data range update state
	RUMBA_STANDBY = 0x0A,
	RUMBA_GYRO_ERR_CHK = 0x0B, //gyro communication error check state
	RUMBA_LOOP_GAIN_ADJ = 0x0E, //loop gain adjustment states
	RUMBA_OISDATA_WRITE = 0x11, //ois data area updating state
	RUMBA_PIDPARAM_INIT = 0x12,
	RUMBA_GYRO_WAKEUP_WAIT = 0x13,
};

struct RUMBAS4SW_LENS_INFO {
	u16 timeStamp;
	u16 hallX;
	u16 hallY;
};

#endif /* OIS_MAIN_H */
