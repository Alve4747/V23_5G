/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 s5kgd2mipiraw_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *0526 change Tr/Tf to +2tabs cwj
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/


#define PFX "SUB[0x0842]_camera_sensor"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__


#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "s5kgd2mipiraw_Sensor.h"
#include "../imgsensor_common.h"

#include "s5kgd2mipiraw_freq.h"
#include "../imgsensor_sensor.h"

/*
 * #define PK_DBG(format, args...) pr_debug(
 * PFX "[%s] " format, __func__, ##args)
 */
 
 /*vivo xuyuanwen add for PD2133 PD2135 board version start*/
#if defined(CONFIG_MTK_CAM_PD2135)
extern char *get_board_version(void);
static char *ccm_board_version = NULL;
#endif
/*vivo xuyuanwen add for PD2133 PD2135 board version end*/
 
static DEFINE_SPINLOCK(imgsensor_drv_lock);

static kal_uint16 hdr_le, hdr_me, hdr_se;

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = S5KGD2_SENSOR_ID,
		.checksum_value = 0xffb1ec31,

		.pre = {
		.pclk = 1592000000,
		.linelength = 16384,
		.framelength = 3236,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.mipi_pixel_rate = 358800000,
		.max_framerate = 300,
	},
	.cap = {	/* 16:11 6528X4896 @ 15fps 32M*/
		.pclk = 1592000000,
		.linelength = 19200,
		.framelength = 2752,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.mipi_pixel_rate = 358800000,
		.max_framerate = 300,

	},
	.normal_video = {	/*16:9 3264X1836 @ 30fps*/
		.pclk = 1592000000,
		.linelength = 18440,
		.framelength = 2880,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 1836,
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.mipi_pixel_rate = 358800000,
		.max_framerate = 300,
	},
	.hs_video = { /*not usehs_video 30fps*/
		.pclk = 1592000000,
		.linelength = 19200,
		.framelength = 2752,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.mipi_pixel_rate = 358800000,
		.max_framerate = 300,

		},
	.slim_video = { /* not use slim_video 30fps*/
		.pclk = 1592000000,
		.linelength = 19200,
		.framelength = 2752,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.mipi_pixel_rate = 358800000,
		.max_framerate = 300,

		},
	.custom1 = { /* 1632*1224 @ 30fps */
		.pclk = 1592000000,
		.linelength = 18000,
		.framelength = 2932,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 1632,
		.grabwindow_height = 1224,
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.mipi_pixel_rate = 180260000,
		.max_framerate = 300,
		},
	.custom2 = { /* 3264*1836 @ 30fps */
		.pclk = 1592000000,
		.linelength = 21000,
		.framelength = 2512,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 1836,
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.mipi_pixel_rate = 358800000,
		.max_framerate = 300,

	},
    .custom3 = {
		.pclk = 1592000000,
		.linelength = 18000,
		.framelength = 2932,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 1632,
		.grabwindow_height = 1224,
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.mipi_pixel_rate = 180260000,
		.max_framerate = 300,

	},
	.custom4 = {
		.pclk = 1592000000,
		.linelength = 18000,
		.framelength = 2932,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 1632,
		.grabwindow_height = 1224,
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.mipi_pixel_rate = 180260000,
		.max_framerate = 300,
		
	},
	.custom5 = {  /*remosaic 6528x4896*/
		.pclk = 1592000000,
		.linelength = 20608,
		.framelength = 5116,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 6528,
		.grabwindow_height = 4896,
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.mipi_pixel_rate = 606660000,
		.max_framerate = 150,

		
	},		
		.margin = 5,
		.min_shutter = 4,
		.min_gain = 64,
		.max_gain = 1024,
		.min_gain_iso = 100,
		.gain_step = 32,
		.gain_type = 2,
		.max_frame_length = 0xffff,
		.ae_shut_delay_frame = 0,
		.ae_sensor_gain_delay_frame = 0,
		.ae_ispGain_delay_frame = 2,
		.frame_time_delay_frame = 2,	/*sony sensor must be 3,non-sony sensor must be 2 , The delay frame of setting frame length  */
		.ihdr_support = 0,	  /*1, support; 0,not support*/
		.ihdr_le_firstline = 0,  /*1,le first; 0, se first*/
		.sensor_mode_num = 10,	  /*support sensor mode num*/

		.cap_delay_frame = 2,/*3 guanjd modify for cts*/
		.pre_delay_frame = 2,/*3 guanjd modify for cts*/
		.video_delay_frame = 3,
		.hs_video_delay_frame = 3,
		.slim_video_delay_frame = 3,
	    .custom1_delay_frame = 3,
	    .custom2_delay_frame = 3,
	    .custom3_delay_frame = 3,
		.custom4_delay_frame = 2,
		.custom5_delay_frame = 2,
	
		.isp_driving_current = ISP_DRIVING_4MA,
		.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
		.mipi_sensor_type = MIPI_OPHY_NCSI2, /*0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2*/
		.mipi_settle_delay_mode = 1, /*0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL*/
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_Gr,
		.mclk = 26,
		.mipi_lane_num = SENSOR_MIPI_2_LANE,
		.i2c_addr_table = {0x20, 0xff},
		.i2c_speed = 1000,
};


static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, /*IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video*/
	.shutter = 0x3D0,					/*current shutter*/
	.gain = 0x100,						/*current gain*/
	.dummy_pixel = 0,					/*current dummypixel*/
	.dummy_line = 0,					/*current dummyline*/
	.current_fps = 0,  /*full size current fps : 24fps for PIP, 30fps for Normal or ZSD*/
	.autoflicker_en = KAL_FALSE,  /*auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker*/
	.test_pattern = KAL_FALSE,		/*test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output*/
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,/*current scenario id*/
	.ihdr_mode = 0, /*sensor need support LE, SE with HDR feature*/
	.i2c_write_id = 0x20,
	.current_ae_effective_frame = 1,
	.freq_setting = 0,
  	.present_freq_setting = 0,

};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] = {
	{ 6528, 4896,	0,	0,	6528, 4896, 3264, 2448,	0000, 0000,	3264, 2448,	  0, 	0, 	3264, 2448}, /*Preview*/
	{ 6528, 4896,	0,	0,	6528, 4896, 3264, 2448,	0000, 0000,	3264, 2448,	  0, 	0, 	3264, 2448}, //capture	
	{ 6528, 4896,	0,612,  6528, 3672, 3264, 1836,	0000, 0000,	3264, 1836,	  0,	0, 	3264, 1836}, /*video*/
	{ 6528, 4896,	0,	0,	6528, 4896, 3264, 2448,	0000, 0000,	3264, 2448,	  0, 	0, 	3264, 2448}, /*hight speed video 120fps*/
	{ 6528, 4896,	0,	0,	6528, 4896, 3264, 2448,	0000, 0000,	3264, 2448,	  0, 	0, 	3264, 2448}, /*slim video  hs_video 240fps*/
	{ 6528, 4896,	0,	0,	6528, 4896, 1632, 1224,	0000, 0000,	1632, 1224,	  0, 	0, 	1632, 1224}, /* custom1*/
	{ 6528, 4896,	0,612,  6528, 3672, 3264, 1836,	0000, 0000,	3264, 1836,	  0,	0, 	3264, 1836}, /* custom2*/
	{ 6528, 4896,	0,	0,	6528, 4896, 1632, 1224,	0000, 0000,	1632, 1224,	  0, 	0, 	1632, 1224}, /* custom3*/
	{ 6528, 4896,	0,	0,	6528, 4896, 1632, 1224,	0000, 0000,	1632, 1224,	  0, 	0, 	1632, 1224}, /*custom4*/
	{ 6528, 4896,	0,	0,	6528, 4896, 6528, 4896,	0000, 0000,	6528, 4896,	  0,	0, 	6528, 4896}, /*custom5 hw-remosiac  remosaic*/
}; /*cpy from preview*/

static struct  SENSOR_RAWINFO_STRUCT imgsensor_raw_info = {
	 3264,//raw_weight 
 	 2448,//raw_height
	 2,//raw_dataBit
	 BAYER_GRBG,//raw_colorFilterValue
	 64,//raw_blackLevel
	 80.4,//raw_viewAngle
	 10,//raw_bitWidth
	 16//raw_maxSensorGain
};

static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3] = {
	/* Preview mode setting */
	{0x05, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x0B40, 0x086C, 0x00, 0x12, 0x0B40, 0x0002, /*VC0:raw, VC1:Embedded header, RAW8*/
	 0x00, 0x30, 0x0B40, 0x0018, 0x00, 0x00, 0x0E10, 0x0001},/*VC2:embedded footer, RAW8*/
	/* Capture mode setting */
	{0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x1680, 0x10D8, 0x00, 0x00, 0x0000, 0x0000,
	 0x00, 0x00, 0x0000, 0x0000, 0x00, 0x00, 0x0000, 0x0000},
	/* Video mode setting */
	{0x02, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x0B40, 0x086C, 0x00, 0x12, 0x0B40, 0x0002,
	 0x00, 0x30, 0x0B40, 0x0018, 0x00, 0x00, 0x0000, 0x0000},
};

/*hope add otp check start*/
static int vivo_otp_read_when_power_on;
extern int SUB_GD2_otp_read(void);
extern otp_error_code_t S5KGD2_OTP_ERROR_CODE;
MUINT32  sn_inf_sub_s5kgd2[13];  /*0 flag   1-12 data*/
MUINT32  material_inf_sub_s5kgd2[4]; 
extern u32 sensor_temperature[10];  
/*hope add otp check end*/

static kal_uint16 read_cmos_sensor_16_16(kal_uint32 addr)
{
	kal_uint16 get_byte= 0;
	char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	 /*kdSetI2CSpeed(imgsensor_info.i2c_speed); Add this func to set i2c speed by each sensor*/
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 2, imgsensor.i2c_write_id);
	return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}


static void write_cmos_sensor_16_16(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
	/* kdSetI2CSpeed(imgsensor_info.i2c_speed); Add this func to set i2c speed by each sensor*/
	iWriteRegI2C(pusendcmd , 4, imgsensor.i2c_write_id);
}

static kal_uint16 read_cmos_sensor_16_8(kal_uint16 addr)
{
	kal_uint16 get_byte= 0;
	char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	/*kdSetI2CSpeed(imgsensor_info.i2c_speed);  Add this func to set i2c speed by each sensor*/
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor_16_8(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
	 /* kdSetI2CSpeed(imgsensor_info.i2c_speed);Add this func to set i2c speed by each sensor*/
	iWriteRegI2C(pusendcmd , 3, imgsensor.i2c_write_id);
}

#define USE_TNP_BURST	0  /*samsung*/
#define MULTI_WRITE 1

#if MULTI_WRITE
#define I2C_BUFFER_LEN 225	/* trans# max is 255, each 3 bytes */
#else
#define I2C_BUFFER_LEN 4

#endif

static kal_uint16 table_write_cmos_sensor(kal_uint16 *para, kal_uint32 len)
{
	char puSendCmd[I2C_BUFFER_LEN];
	kal_uint32 tosend, IDX;
	kal_uint16 addr = 0, addr_last = 0, data;

	tosend = 0;
	IDX = 0;
	while (len > IDX) {
		addr = para[IDX];

		{
			puSendCmd[tosend++] = (char)(addr >> 8);
			puSendCmd[tosend++] = (char)(addr & 0xFF);
			data = para[IDX + 1];
			puSendCmd[tosend++] = (char)(data >> 8);
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;

		}
#if MULTI_WRITE
		/* Write when remain buffer size is less than 4 bytes or reach end of data */
		if ((I2C_BUFFER_LEN - tosend) < 4 || IDX == len || addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd, tosend, imgsensor.i2c_write_id,
								4, imgsensor_info.i2c_speed);
			tosend = 0;
		}
#else
		iWriteRegI2CTiming(puSendCmd, 4, imgsensor.i2c_write_id, imgsensor_info.i2c_speed);
		tosend = 0;

#endif
	}
	return 0;
}

#define I2C_BURST_LEN 1000
static bool sensor_register_is_consecutive(kal_uint16 *regs, kal_uint32 i, kal_uint32 size)
{
    if (regs[i] != regs[i + 2]){
        return false;
    }
    return true;
}
static kal_uint16 table_write_cmos_sensor_consecutive(kal_uint16 *regs, kal_uint32 size)
{
    char puSendCmd[I2C_BURST_LEN];
    kal_uint32 i, tosend;
    kal_uint16 addr = 0;
    kal_uint16 data = 0;
    kal_uint32 num_consecutive, IDX, start_IDX;
    kal_uint16 transfer_length;
    transfer_length = 0;
    num_consecutive = 1;
    IDX = 0;
    start_IDX = IDX;
    tosend = 0;
    pr_info("regs size: %d\n", size);
    if (!size || 0 != size % 2 ){
        pr_err("error reg size :%d", size);
        return -1;
    }
    while (IDX < size) {
        start_IDX = IDX;
        while ((IDX+2) < size) {
            if (sensor_register_is_consecutive(regs, IDX, size)){
                num_consecutive++;
                IDX+=2;
            } else {
                break;
            }
        }
        addr = regs[start_IDX];
        puSendCmd[tosend++] = (char)(addr >> 8);
        puSendCmd[tosend++] = (char)(addr & 0xFF);
        if (num_consecutive>1) {
            i = 0;
            while (i<num_consecutive)
            {
                data = regs[start_IDX+i*2+1];
                puSendCmd[tosend++]=(char)(data >> 8);
                puSendCmd[tosend++]=(char)(data & 0xFF);
                i++;
            }
            transfer_length = num_consecutive*2 + 2; //2 bytes for address
            iBurstWriteReg_multi(puSendCmd,
                    transfer_length,
                    imgsensor.i2c_write_id,
                    transfer_length,
                    imgsensor_info.i2c_speed);
            tosend = 0;
            num_consecutive = 1;
        }
        else {
            data = regs[IDX+1];
            puSendCmd[tosend++]=(char)(data >> 8);
            puSendCmd[tosend++]=(char)(data & 0xFF);
            iBurstWriteReg_multi(puSendCmd, tosend,
                          imgsensor.i2c_write_id, 4, imgsensor_info.i2c_speed);
            tosend = 0;
        }
        IDX+=2;
    }
    return 0;
}


static void set_dummy(void)
{
	PK_DBG("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	 write_cmos_sensor_16_16(0x0340, imgsensor.frame_length);
	 write_cmos_sensor_16_16(0x0342, imgsensor.line_length);
}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{

	kal_uint32 frame_length = imgsensor.frame_length;

	PK_DBG("framerate = %d, min framelength should enable %d \n", framerate,min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	if (frame_length >= imgsensor.min_frame_length)
		imgsensor.frame_length = frame_length;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

static void write_shutter(kal_uint16 shutter)
{

	kal_uint16 realtime_fps = 0;

	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	if (shutter < imgsensor_info.min_shutter) shutter = imgsensor_info.min_shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);
		else {
		/* Extend frame length*/
	        write_cmos_sensor_16_16(0x0340, imgsensor.frame_length);

	    }
	} else {
		/* Extend frame length*/
		write_cmos_sensor_16_16(0x0340, imgsensor.frame_length);

	}
	/* Update Shutter*/

	write_cmos_sensor_16_16(0x0202, shutter & 0xFFFF);
	PK_DBG("sensor_mode=%d\n", imgsensor.sensor_mode);
	PK_DBG("shutter = %d, framelength = %d\n", shutter, imgsensor.frame_length);

}	/*	write_shutter  */



/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint32 shutter)
{
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}	/*	set_shutter */

/*	write_shutter  */
static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length, kal_bool auto_extend_en)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	spin_lock(&imgsensor_drv_lock);
	/* Change frame time */
	if (frame_length > 1)
		dummy_line = frame_length - imgsensor.frame_length;
	imgsensor.frame_length = imgsensor.frame_length + dummy_line;

	/*  */
	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;

	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor_16_16(0x0340, imgsensor.frame_length & 0xFFFF);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor_16_16(0x0340, imgsensor.frame_length & 0xFFFF);
	}

	/* Update Shutter */
	write_cmos_sensor_16_16(0X0202, shutter & 0xFFFF);

	PK_DBG("shutter = %d, framelength = %d/%d, dummy_line= %d\n", shutter, imgsensor.frame_length,
		frame_length, dummy_line);

}				/*      write_shutter  */


static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0;

	reg_gain = gain / 2;
	return (kal_uint16) reg_gain;
}

/*************************************************************************
 * FUNCTION
 *	set_gain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *	iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	/*gain= 1024;for test*/
	/*return; for test*/

	if (gain < imgsensor_info.min_gain || gain > imgsensor_info.max_gain) {
		pr_debug("Error gain setting");

		if (gain < imgsensor_info.min_gain)
			gain = imgsensor_info.min_gain;
		else
			gain = imgsensor_info.max_gain;
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	PK_DBG("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);
  	write_cmos_sensor_16_16(0x0204,reg_gain);
  
	return gain;
}				/*      set_gain  */

static void set_mirror_flip(kal_uint8 image_mirror)
{
	switch (image_mirror) {

	    case IMAGE_NORMAL:

	        write_cmos_sensor_16_8(0x0101,0x00);   /* Gr*/
	        break;

	    case IMAGE_H_MIRROR:

	        write_cmos_sensor_16_8(0x0101,0x01);
	        break;

	    case IMAGE_V_MIRROR:

	        write_cmos_sensor_16_8(0x0101,0x02);
	        break;

	    case IMAGE_HV_MIRROR:

	        write_cmos_sensor_16_8(0x0101,0x03);/*Gb*/
	        break;
	    default:
	    PK_DBG("Error image_mirror setting\n");
	}
}

/*************************************************************************
 * FUNCTION
 *	night_mode
 *
 * DESCRIPTION
 *	This function night mode of sensor.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
#if 0
static void night_mode(kal_bool enable)
{
	/*No Need to implement this function*/
}				/*      night_mode      */
#endif

static kal_uint32 streaming_control(kal_bool enable)
{
	int timeout = (10000 / imgsensor.current_fps) + 1;
	int i = 0;
	int framecnt = 0;

	PK_DBG("streaming_enable(0= Sw Standby,1= streaming): %d\n", enable);
	if (enable) {
		write_cmos_sensor_16_8(0x0100, 0x01);
		mdelay(10);
	} else {
		write_cmos_sensor_16_8(0x0100, 0x00);
		for (i = 0; i < timeout; i++) {
			mdelay(5);
			framecnt = read_cmos_sensor_16_8(0x0005);
			if (framecnt == 0xFF) {
				PK_DBG(" Stream Off OK at i=%d.\n", i);
				return ERROR_NONE;
			}
		}
		PK_DBG("Stream Off Fail! framecnt= %d.\n", framecnt);
	}
	return ERROR_NONE;
}

#if USE_TNP_BURST
static const u16 uTnpArrayA[] = {
};
	
static const u16 uTnpArrayB[] = {
};
static const u16 uTnpArrayC[] = {
};
static const u16 uTnpArray_Face_A[] = {
};
static const u16 uTnpArray_Face_B[] = {
};  
#endif
static kal_uint16 addr_data_pair_init[] = {
	0x6028, 0x2001,
	0x602A, 0xD604,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0348,
	0x6F12, 0x90F8,
	0x6F12, 0xDA0C,
	0x6F12, 0x0128,
	0x6F12, 0x01D0,
	0x6F12, 0x00F0,
	0x6F12, 0x09B8,
	0x6F12, 0x7047,
	0x6F12, 0x2000,
	0x6F12, 0x12E0,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0548,
	0x6F12, 0x0449,
	0x6F12, 0x054A,
	0x6F12, 0xC0F8,
	0x6F12, 0x8C17,
	0x6F12, 0x511A,
	0x6F12, 0xC0F8,
	0x6F12, 0x9017,
	0x6F12, 0x00F0,
	0x6F12, 0x2AB9,
	0x6F12, 0x2001,
	0x6F12, 0xD9E8,
	0x6F12, 0x2001,
	0x6F12, 0x8400,
	0x6F12, 0x2002,
	0x6F12, 0x3600,
	0x6F12, 0x2DE9,
	0x6F12, 0xF041,
	0x6F12, 0xA44D,
	0x6F12, 0x0646,
	0x6F12, 0xA44F,
	0x6F12, 0xAC89,
	0x6F12, 0x2007,
	0x6F12, 0x08D5,
	0x6F12, 0xB7F8,
	0x6F12, 0xFE08,
	0x6F12, 0x28B1,
	0x6F12, 0x0022,
	0x6F12, 0x0821,
	0x6F12, 0x46F2,
	0x6F12, 0x1420,
	0x6F12, 0x00F0,
	0x6F12, 0x53F9,
	0x6F12, 0x24F0,
	0x6F12, 0x0100,
	0x6F12, 0x9E4C,
	0x6F12, 0xA4F8,
	0x6F12, 0x1802,
	0x6F12, 0x4FF4,
	0x6F12, 0x8068,
	0x6F12, 0x0022,
	0x6F12, 0x4146,
	0x6F12, 0x46F2,
	0x6F12, 0x3020,
	0x6F12, 0x00F0,
	0x6F12, 0x46F9,
	0x6F12, 0x9748,
	0x6F12, 0x7830,
	0x6F12, 0x00F0,
	0x6F12, 0x47F9,
	0x6F12, 0x00B1,
	0x6F12, 0x3EB1,
	0x6F12, 0xE888,
	0x6F12, 0xA4F8,
	0x6F12, 0x0E02,
	0x6F12, 0x00F0,
	0x6F12, 0x45F9,
	0x6F12, 0x3864,
	0x6F12, 0xBDE8,
	0x6F12, 0xF081,
	0x6F12, 0x46F2,
	0x6F12, 0x0E24,
	0x6F12, 0x0022,
	0x6F12, 0x4146,
	0x6F12, 0x2046,
	0x6F12, 0x00F0,
	0x6F12, 0x31F9,
	0x6F12, 0x0120,
	0x6F12, 0x00F0,
	0x6F12, 0x3DF9,
	0x6F12, 0x0122,
	0x6F12, 0x8021,
	0x6F12, 0x2046,
	0x6F12, 0x00F0,
	0x6F12, 0x29F9,
	0x6F12, 0xEA88,
	0x6F12, 0x4FF6,
	0x6F12, 0xFF51,
	0x6F12, 0x2046,
	0x6F12, 0x00F0,
	0x6F12, 0x37F9,
	0x6F12, 0xE4E7,
	0x6F12, 0x2DE9,
	0x6F12, 0xF04F,
	0x6F12, 0x8648,
	0x6F12, 0x91B0,
	0x6F12, 0x0568,
	0x6F12, 0x95F8,
	0x6F12, 0x3C73,
	0x6F12, 0x05F5,
	0x6F12, 0x4F75,
	0x6F12, 0xF807,
	0x6F12, 0x78D0,
	0x6F12, 0x0220,
	0x6F12, 0x00EA,
	0x6F12, 0x5704,
	0x6F12, 0x6888,
	0x6F12, 0x1090,
	0x6F12, 0xB0F5,
	0x6F12, 0x806F,
	0x6F12, 0x01D3,
	0x6F12, 0x44F0,
	0x6F12, 0x1004,
	0x6F12, 0x002C,
	0x6F12, 0x60D1,
	0x6F12, 0xB807,
	0x6F12, 0x52D5,
	0x6F12, 0x7C49,
	0x6F12, 0xB1F8,
	0x6F12, 0x0A01,
	0x6F12, 0x38B1,
	0x6F12, 0x0128,
	0x6F12, 0x0AD0,
	0x6F12, 0x0228,
	0x6F12, 0x16D0,
	0x6F12, 0x54F0,
	0x6F12, 0x8004,
	0x6F12, 0x0AD0,
	0x6F12, 0x52E0,
	0x6F12, 0x4FF0,
	0x6F12, 0x000B,
	0x6F12, 0x4FF4,
	0x6F12, 0x8030,
	0x6F12, 0x03E0,
	0x6F12, 0xB1F8,
	0x6F12, 0x0CB1,
	0x6F12, 0xB1F8,
	0x6F12, 0x0E01,
	0x6F12, 0x8246,
	0x6F12, 0x6946,
	0x6F12, 0x1098,
	0x6F12, 0x00F0,
	0x6F12, 0x08F9,
	0x6F12, 0x4028,
	0x6F12, 0x07D0,
	0x6F12, 0x44F0,
	0x6F12, 0x0104,
	0x6F12, 0x3FE0,
	0x6F12, 0xB1F8,
	0x6F12, 0x10B1,
	0x6F12, 0xB1F8,
	0x6F12, 0x1201,
	0x6F12, 0xF0E7,
	0x6F12, 0xE946,
	0x6F12, 0x05F1,
	0x6F12, 0x0408,
	0x6F12, 0x0026,
	0x6F12, 0xD8F8,
	0x6F12, 0x0010,
	0x6F12, 0xC1B1,
	0x6F12, 0xD9F8,
	0x6F12, 0x0000,
	0x6F12, 0x20B1,
	0x6F12, 0x8842,
	0x6F12, 0x13D0,
	0x6F12, 0x44F0,
	0x6F12, 0x0404,
	0x6F12, 0x10E0,
	0x6F12, 0x1098,
	0x6F12, 0x06EB,
	0x6F12, 0x8010,
	0x6F12, 0x8345,
	0x6F12, 0x09D8,
	0x6F12, 0x5045,
	0x6F12, 0x07D2,
	0x6F12, 0x4246,
	0x6F12, 0x0421,
	0x6F12, 0x00F0,
	0x6F12, 0xE9F8,
	0x6F12, 0x20B9,
	0x6F12, 0x44F0,
	0x6F12, 0x4004,
	0x6F12, 0x01E0,
	0x6F12, 0x44F0,
	0x6F12, 0x2004,
	0x6F12, 0x361D,
	0x6F12, 0x09F1,
	0x6F12, 0x0409,
	0x6F12, 0x08F1,
	0x6F12, 0x0408,
	0x6F12, 0x402E,
	0x6F12, 0xDCD3,
	0x6F12, 0x3806,
	0x6F12, 0x11D5,
	0x6F12, 0x4022,
	0x6F12, 0x0021,
	0x6F12, 0x281D,
	0x6F12, 0x00F0,
	0x6F12, 0xDAF8,
	0x6F12, 0x0BE0,
	0x6F12, 0x291D,
	0x6F12, 0x1098,
	0x6F12, 0x00F0,
	0x6F12, 0xCBF8,
	0x6F12, 0x4028,
	0x6F12, 0x05D0,
	0x6F12, 0x4022,
	0x6F12, 0x0021,
	0x6F12, 0x281D,
	0x6F12, 0x00F0,
	0x6F12, 0xCEF8,
	0x6F12, 0xBCE7,
	0x6F12, 0x00F0,
	0x6F12, 0xD0F8,
	0x6F12, 0x08B1,
	0x6F12, 0x44F0,
	0x6F12, 0x0804,
	0x6F12, 0x0CB1,
	0x6F12, 0x47F0,
	0x6F12, 0x0407,
	0x6F12, 0x27F0,
	0x6F12, 0x0100,
	0x6F12, 0x6C70,
	0x6F12, 0x2870,
	0x6F12, 0x11B0,
	0x6F12, 0xBDE8,
	0x6F12, 0xF08F,
	0x6F12, 0x70B5,
	0x6F12, 0x0446,
	0x6F12, 0x4548,
	0x6F12, 0x0022,
	0x6F12, 0x8068,
	0x6F12, 0x86B2,
	0x6F12, 0x050C,
	0x6F12, 0x3146,
	0x6F12, 0x2846,
	0x6F12, 0x00F0,
	0x6F12, 0x90F8,
	0x6F12, 0x2046,
	0x6F12, 0x00F0,
	0x6F12, 0xBAF8,
	0x6F12, 0x0122,
	0x6F12, 0x3146,
	0x6F12, 0x2846,
	0x6F12, 0x00F0,
	0x6F12, 0x88F8,
	0x6F12, 0x2188,
	0x6F12, 0x4FF0,
	0x6F12, 0x8040,
	0x6F12, 0xB0FB,
	0x6F12, 0xF1F1,
	0x6F12, 0x01F5,
	0x6F12, 0x0071,
	0x6F12, 0x890A,
	0x6F12, 0xE180,
	0x6F12, 0x6188,
	0x6F12, 0xB0FB,
	0x6F12, 0xF1F1,
	0x6F12, 0x01F5,
	0x6F12, 0x0071,
	0x6F12, 0x890A,
	0x6F12, 0x2181,
	0x6F12, 0xA188,
	0x6F12, 0xB0FB,
	0x6F12, 0xF1F0,
	0x6F12, 0x00F5,
	0x6F12, 0x0070,
	0x6F12, 0x800A,
	0x6F12, 0x6081,
	0x6F12, 0x70BD,
	0x6F12, 0x2DE9,
	0x6F12, 0xF84F,
	0x6F12, 0x8246,
	0x6F12, 0x2F48,
	0x6F12, 0x8946,
	0x6F12, 0x9046,
	0x6F12, 0xC168,
	0x6F12, 0x1E46,
	0x6F12, 0x0D0C,
	0x6F12, 0x8FB2,
	0x6F12, 0x0A9C,
	0x6F12, 0x0022,
	0x6F12, 0x3946,
	0x6F12, 0x2846,
	0x6F12, 0x00F0,
	0x6F12, 0x60F8,
	0x6F12, 0x3346,
	0x6F12, 0x4246,
	0x6F12, 0x4946,
	0x6F12, 0x5046,
	0x6F12, 0x0094,
	0x6F12, 0x00F0,
	0x6F12, 0x8BF8,
	0x6F12, 0x00F0,
	0x6F12, 0x8EF8,
	0x6F12, 0x10B1,
	0x6F12, 0xA088,
	0x6F12, 0x401E,
	0x6F12, 0xA080,
	0x6F12, 0x3946,
	0x6F12, 0x2846,
	0x6F12, 0xBDE8,
	0x6F12, 0xF84F,
	0x6F12, 0x0122,
	0x6F12, 0x00F0,
	0x6F12, 0x4CB8,
	0x6F12, 0x10B5,
	0x6F12, 0x0721,
	0x6F12, 0x0220,
	0x6F12, 0x00F0,
	0x6F12, 0x83F8,
	0x6F12, 0xBDE8,
	0x6F12, 0x1040,
	0x6F12, 0x4FF6,
	0x6F12, 0xFF71,
	0x6F12, 0x0A20,
	0x6F12, 0x00F0,
	0x6F12, 0x7CB8,
	0x6F12, 0x70B5,
	0x6F12, 0x194C,
	0x6F12, 0x0022,
	0x6F12, 0xAFF2,
	0x6F12, 0x4F21,
	0x6F12, 0x236A,
	0x6F12, 0x1748,
	0x6F12, 0x9847,
	0x6F12, 0x144D,
	0x6F12, 0x0022,
	0x6F12, 0x2860,
	0x6F12, 0xAFF2,
	0x6F12, 0xDB11,
	0x6F12, 0x236A,
	0x6F12, 0x1448,
	0x6F12, 0x9847,
	0x6F12, 0x6860,
	0x6F12, 0x0022,
	0x6F12, 0xAFF2,
	0x6F12, 0xDD01,
	0x6F12, 0x236A,
	0x6F12, 0x1248,
	0x6F12, 0x9847,
	0x6F12, 0xA860,
	0x6F12, 0x0022,
	0x6F12, 0xAFF2,
	0x6F12, 0x9301,
	0x6F12, 0x236A,
	0x6F12, 0x0F48,
	0x6F12, 0x9847,
	0x6F12, 0xE860,
	0x6F12, 0x0022,
	0x6F12, 0xAFF2,
	0x6F12, 0x5B01,
	0x6F12, 0x236A,
	0x6F12, 0x0D48,
	0x6F12, 0x9847,
	0x6F12, 0x2861,
	0x6F12, 0x70BD,
	0x6F12, 0x0000,
	0x6F12, 0x2000,
	0x6F12, 0x12B0,
	0x6F12, 0x2001,
	0x6F12, 0x8400,
	0x6F12, 0x4000,
	0x6F12, 0x6000,
	0x6F12, 0x2000,
	0x6F12, 0x0C10,
	0x6F12, 0x2000,
	0x6F12, 0x12E0,
	0x6F12, 0x2001,
	0x6F12, 0xD9B0,
	0x6F12, 0x2000,
	0x6F12, 0x8A80,
	0x6F12, 0x0001,
	0x6F12, 0x9C21,
	0x6F12, 0x0001,
	0x6F12, 0x8907,
	0x6F12, 0x0000,
	0x6F12, 0x4305,
	0x6F12, 0x0000,
	0x6F12, 0x11C9,
	0x6F12, 0x0001,
	0x6F12, 0x7D6F,
	0x6F12, 0x4BF2,
	0x6F12, 0x390C,
	0x6F12, 0xC0F2,
	0x6F12, 0x010C,
	0x6F12, 0x6047,
	0x6F12, 0x48F6,
	0x6F12, 0x8B5C,
	0x6F12, 0xC0F2,
	0x6F12, 0x010C,
	0x6F12, 0x6047,
	0x6F12, 0x44F6,
	0x6F12, 0x9B6C,
	0x6F12, 0xC0F2,
	0x6F12, 0x010C,
	0x6F12, 0x6047,
	0x6F12, 0x4AF6,
	0x6F12, 0xCF4C,
	0x6F12, 0xC0F2,
	0x6F12, 0x010C,
	0x6F12, 0x6047,
	0x6F12, 0x4BF2,
	0x6F12, 0x550C,
	0x6F12, 0xC0F2,
	0x6F12, 0x010C,
	0x6F12, 0x6047,
	0x6F12, 0x48F6,
	0x6F12, 0xFF0C,
	0x6F12, 0xC0F2,
	0x6F12, 0x010C,
	0x6F12, 0x6047,
	0x6F12, 0x46F6,
	0x6F12, 0x2B2C,
	0x6F12, 0xC0F2,
	0x6F12, 0x020C,
	0x6F12, 0x6047,
	0x6F12, 0x4BF2,
	0x6F12, 0x633C,
	0x6F12, 0xC0F2,
	0x6F12, 0x010C,
	0x6F12, 0x6047,
	0x6F12, 0x47F2,
	0x6F12, 0x2D4C,
	0x6F12, 0xC0F2,
	0x6F12, 0x010C,
	0x6F12, 0x6047,
	0x6F12, 0x44F2,
	0x6F12, 0x053C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x41F2,
	0x6F12, 0xC91C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x49F2,
	0x6F12, 0x374C,
	0x6F12, 0xC0F2,
	0x6F12, 0x020C,
	0x6F12, 0x6047,
	0x6F12, 0x47F2,
	0x6F12, 0x032C,
	0x6F12, 0xC0F2,
	0x6F12, 0x010C,
	0x6F12, 0x6047,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0842,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x01B1,
	0x6028, 0x2000,
	0x602A, 0x13BE,
	0x6F12, 0x0200,
	0x602A, 0x2A04,
	0x6F12, 0x5608,
	0x602A, 0x31EA,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x422A,
	0x6F12, 0x0008,
	0x6F12, 0x0808,
	0x6F12, 0x0808,
	0x6F12, 0x0808,
	0x6F12, 0x0801,
	0x6028, 0x2000,
	0x602A, 0x4208,
	0x6F12, 0x0004,
	0x6F12, 0x0C8D,
	0x602A, 0x219A,
	0x6F12, 0x6401,
	0x602A, 0x2B96,
	0x6F12, 0x0033,
	0x6F12, 0x0033,
	0x6F12, 0x0033,
	0x6F12, 0x0033,
	0x6F12, 0x0033,
	0x6F12, 0x0033,
	0x6F12, 0x0033,
	0x6F12, 0x0033,
	0x6F12, 0x0033,
	0x6F12, 0x0033,
	0x6F12, 0x0033,
	0x6F12, 0x0033,
	0x6F12, 0x0033,
	0x6F12, 0x0033,
	0x6F12, 0x0033,
	0x6F12, 0x0033,
	0x6F12, 0x0033,
	0x6F12, 0x0033,
	0x6F12, 0x0033,
	0x6F12, 0x0033,
	0x602A, 0x2BFC,
	0x6F12, 0x033F,
	0x6F12, 0x033F,
	0x6F12, 0x033F,
	0x6F12, 0x033F,
	0x6F12, 0x033F,
	0x6F12, 0x033F,
	0x6F12, 0x033F,
	0x6F12, 0x033F,
	0x6F12, 0x033F,
	0x6F12, 0x033F,
	0x6F12, 0x033F,
	0x6F12, 0x033F,
	0x6F12, 0x033F,
	0x6F12, 0x033F,
	0x6F12, 0x033F,
	0x6F12, 0x033F,
	0x6F12, 0x033F,
	0x6F12, 0x033F,
	0x6F12, 0x033F,
	0x6F12, 0x033F,
	0x602A, 0x31C8,
	0x6F12, 0x0101,
	0x602A, 0x2A00,
	0x6F12, 0x0100,
	0x6F12, 0x0480,
	0x602A, 0x29F8,
	0x6F12, 0x090F,
	0x602A, 0x29F2,
	0x6F12, 0x0305,
	0x602A, 0x29FA,
	0x6F12, 0x0203,
	0x602A, 0x5AC4,
	0x6F12, 0x0001,
	0x602A, 0x5B30,
	0x6F12, 0x0000,
	0x602A, 0x5B18,
	0x6F12, 0x0000,
	0x602A, 0x6CA6,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x6C96,
	0x6F12, 0x04B1,
	0x6F12, 0x04B1,
	0x6F12, 0x04B1,
	0x6F12, 0x04B1,
	0x6F12, 0x04B1,
	0x6F12, 0x04B1,
	0x602A, 0x6B16,
	0x6F12, 0x0021,
	0x6F12, 0x0021,
	0x6F12, 0x0021,
	0x6F12, 0x0021,
	0x6F12, 0x0021,
	0x6F12, 0x0021,
	0x602A, 0x6B06,
	0x6F12, 0x0015,
	0x6F12, 0x0015,
	0x6F12, 0x0015,
	0x6F12, 0x0015,
	0x6F12, 0x0015,
	0x6F12, 0x0015,
	0x602A, 0x6B36,
	0x6F12, 0x004C,
	0x6F12, 0x004C,
	0x6F12, 0x004C,
	0x6F12, 0x004C,
	0x6F12, 0x004C,
	0x6F12, 0x004C,
	0x602A, 0x6B26,
	0x6F12, 0x0001,
	0x6F12, 0x0001,
	0x6F12, 0x0001,
	0x6F12, 0x0001,
	0x6F12, 0x0001,
	0x6F12, 0x0001,
	0x602A, 0x6AF6,
	0x6F12, 0x06FA,
	0x6F12, 0x06FA,
	0x6F12, 0x06FA,
	0x6F12, 0x06FA,
	0x6F12, 0x06FA,
	0x6F12, 0x06FA,
	0x602A, 0x6AE6,
	0x6F12, 0x1753,
	0x6F12, 0x1753,
	0x6F12, 0x1753,
	0x6F12, 0x1753,
	0x6F12, 0x1753,
	0x6F12, 0x1753,
	0x602A, 0x6BD6,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x6BC6,
	0x6F12, 0x001B,
	0x6F12, 0x001B,
	0x6F12, 0x001B,
	0x6F12, 0x001B,
	0x6F12, 0x001B,
	0x6F12, 0x001B,
	0x602A, 0x6BF6,
	0x6F12, 0x0027,
	0x6F12, 0x0027,
	0x6F12, 0x0027,
	0x6F12, 0x0027,
	0x6F12, 0x0027,
	0x6F12, 0x0027,
	0x602A, 0x6BE6,
	0x6F12, 0x0075,
	0x6F12, 0x0075,
	0x6F12, 0x0075,
	0x6F12, 0x0075,
	0x6F12, 0x0075,
	0x6F12, 0x0075,
	0x602A, 0x6BB6,
	0x6F12, 0x0054,
	0x6F12, 0x0054,
	0x6F12, 0x0054,
	0x6F12, 0x0054,
	0x6F12, 0x0054,
	0x6F12, 0x0054,
	0x602A, 0x6BA6,
	0x6F12, 0x0579,
	0x6F12, 0x0579,
	0x6F12, 0x0579,
	0x6F12, 0x0579,
	0x6F12, 0x0579,
	0x6F12, 0x0579,
	0x602A, 0x6B76,
	0x6F12, 0x0010,
	0x6F12, 0x0010,
	0x6F12, 0x0010,
	0x6F12, 0x0010,
	0x6F12, 0x0010,
	0x6F12, 0x0010,
	0x602A, 0x6B66,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x6B96,
	0x6F12, 0x0001,
	0x6F12, 0x0001,
	0x6F12, 0x0001,
	0x6F12, 0x0001,
	0x6F12, 0x0001,
	0x6F12, 0x0001,
	0x602A, 0x6B86,
	0x6F12, 0x007D,
	0x6F12, 0x007D,
	0x6F12, 0x007D,
	0x6F12, 0x007D,
	0x6F12, 0x007D,
	0x6F12, 0x007D,
	0x602A, 0x6B56,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x6B46,
	0x6F12, 0x0082,
	0x6F12, 0x0082,
	0x6F12, 0x0082,
	0x6F12, 0x0082,
	0x6F12, 0x0082,
	0x6F12, 0x0082,
	0x602A, 0x6C36,
	0x6F12, 0x00B0,
	0x6F12, 0x00B0,
	0x6F12, 0x00B0,
	0x6F12, 0x00B0,
	0x6F12, 0x00B0,
	0x6F12, 0x00B0,
	0x602A, 0x6C46,
	0x6F12, 0x0025,
	0x6F12, 0x0025,
	0x6F12, 0x0025,
	0x6F12, 0x0025,
	0x6F12, 0x0025,
	0x6F12, 0x0025,
	0x602A, 0x5B3C,
	0x6F12, 0x0000,
	0x602A, 0x5B48,
	0x6F12, 0x0000,
	0x602A, 0x5B42,
	0x6F12, 0x012C,
	0x602A, 0x5B4E,
	0x6F12, 0x0000,
	0x602A, 0x5B5A,
	0x6F12, 0x0003,
	0x602A, 0x5B54,
	0x6F12, 0x0004,
	0x602A, 0x5B66,
	0x6F12, 0x0001,
	0x602A, 0x5B60,
	0x6F12, 0x0001,
	0x602A, 0x6C76,
	0x6F12, 0x0FFF,
	0x6F12, 0x0FFF,
	0x6F12, 0x0FFF,
	0x6F12, 0x0FFF,
	0x6F12, 0x0FFF,
	0x6F12, 0x0FFF,
	0x602A, 0x6C86,
	0x6F12, 0x0FFF,
	0x6F12, 0x0FFF,
	0x6F12, 0x0FFF,
	0x6F12, 0x0FFF,
	0x6F12, 0x0FFF,
	0x6F12, 0x0FFF,
	0x602A, 0x6C56,
	0x6F12, 0x0176,
	0x6F12, 0x0176,
	0x6F12, 0x0176,
	0x6F12, 0x0176,
	0x6F12, 0x0176,
	0x6F12, 0x0176,
	0x602A, 0x6C66,
	0x6F12, 0x0025,
	0x6F12, 0x0025,
	0x6F12, 0x0025,
	0x6F12, 0x0025,
	0x6F12, 0x0025,
	0x6F12, 0x0025,
	0x602A, 0x6AD6,
	0x6F12, 0x0190,
	0x6F12, 0x0190,
	0x6F12, 0x0190,
	0x6F12, 0x0190,
	0x6F12, 0x0190,
	0x6F12, 0x0190,
	0x602A, 0x5A40,
	0x6F12, 0x0001,
	0x602A, 0x5B36,
	0x6F12, 0x0000,
	0x602A, 0x5B78,
	0x6F12, 0x0000,
	0x602A, 0x5B7E,
	0x6F12, 0x0001,
	0x602A, 0x5B1E,
	0x6F12, 0x01B3,
	0x602A, 0x5B24,
	0x6F12, 0x0363,
	0x602A, 0x5B2A,
	0x6F12, 0x0007,
	0x602A, 0x5B0C,
	0x6F12, 0x0333,
	0x602A, 0x5B12,
	0x6F12, 0x0000,
	0x602A, 0x5B06,
	0x6F12, 0x0000,
	0x602A, 0x5AFA,
	0x6F12, 0x0000,
	0x602A, 0x5B00,
	0x6F12, 0x0000,
	0x602A, 0x5AF4,
	0x6F12, 0x00FC,
	0x602A, 0x6C16,
	0x6F12, 0x03EC,
	0x6F12, 0x03EC,
	0x6F12, 0x03EC,
	0x6F12, 0x03EC,
	0x6F12, 0x03EC,
	0x6F12, 0x03EC,
	0x602A, 0x5B72,
	0x6F12, 0x0007,
	0x602A, 0x5B8A,
	0x6F12, 0x0000,
	0x602A, 0x6C26,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x6C06,
	0x6F12, 0x035D,
	0x6F12, 0x035D,
	0x6F12, 0x035D,
	0x6F12, 0x035D,
	0x6F12, 0x035D,
	0x6F12, 0x035D,
	0x602A, 0x5AE8,
	0x6F12, 0x03FC,
	0x602A, 0x5AEE,
	0x6F12, 0x0000,
	0x602A, 0x5AE2,
	0x6F12, 0x012C,
	0x602A, 0x5AD6,
	0x6F12, 0x1984,
	0x602A, 0x5ADC,
	0x6F12, 0x3FFD,
	0x602A, 0x5AD0,
	0x6F12, 0x0035,
	0x602A, 0x5AB8,
	0x6F12, 0x014C,
	0x602A, 0x5ABE,
	0x6F12, 0x0001,
	0x602A, 0x5AB2,
	0x6F12, 0x0009,
	0x602A, 0x5AA6,
	0x6F12, 0x3FFE,
	0x602A, 0x5AAC,
	0x6F12, 0x00A2,
	0x602A, 0x5AA0,
	0x6F12, 0x184F,
	0x602A, 0x5A8E,
	0x6F12, 0x01D7,
	0x602A, 0x5A94,
	0x6F12, 0x03FF,
	0x602A, 0x5A88,
	0x6F12, 0x03FF,
	0x602A, 0x5A7C,
	0x6F12, 0x0002,
	0x602A, 0x5A82,
	0x6F12, 0x3FFE,
	0x602A, 0x5A76,
	0x6F12, 0x1ED7,
	0x602A, 0x5A64,
	0x6F12, 0x0004,
	0x602A, 0x5A6A,
	0x6F12, 0x0000,
	0x602A, 0x5A5E,
	0x6F12, 0x0000,
	0x602A, 0x5A52,
	0x6F12, 0x0000,
	0x602A, 0x5A58,
	0x6F12, 0x3E68,
	0x602A, 0x5A4C,
	0x6F12, 0x0000,
	0x602A, 0x5B84,
	0x6F12, 0x0000,
	0x602A, 0x5B72,
	0x6F12, 0x0000,
	0x602A, 0x2186,
	0x6F12, 0x0100,
	0x602A, 0x1310,
	0x6F12, 0x0F00,
	0x602A, 0x2188,
	0x6F12, 0x0650,
	0x602A, 0x218E,
	0x6F12, 0x0650,
	0x602A, 0x2194,
	0x6F12, 0x0100,
	0x602A, 0x3450,
	0x6F12, 0x193C,
	0x602A, 0x43C8,
	0x6F12, 0x0014,
	0x6F12, 0x000E,
	0x6F12, 0x003A,
	0x6F12, 0x0033,
	0x6F12, 0x0003,
	0x6F12, 0x0004,
	0x602A, 0x43DA,
	0x6F12, 0x19A0,
	0x6F12, 0x1348,
	0x6F12, 0x0138,
	0x6F12, 0x0140,
	0x6F12, 0x0110,
	0x6F12, 0x011C,
	0x6F12, 0x012C,
	0x6F12, 0x013C,
	0x6F12, 0x0150,
	0x6F12, 0x0168,
	0x6F12, 0x0180,
	0x6F12, 0x019C,
	0x6F12, 0x01BC,
	0x6F12, 0x01E4,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x012C,
	0x6F12, 0x0144,
	0x6F12, 0x0160,
	0x6F12, 0x0180,
	0x6F12, 0x01A8,
	0x6F12, 0x0214,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6028, 0x4000,
	0x0FE8, 0x4180,

};

static kal_uint16 addr_data_pair_preview[] = {
	0x6028, 0x2000,
	0x602A, 0x2A08,
	0x6F12, 0x0001,
	0x6F12, 0xA5E0,
	0x602A, 0x21F8,
	0x6F12, 0x1358,
	0x602A, 0x30CE,
	0x6F12, 0x01E4,
	0x602A, 0x8A00,
	0x6F12, 0x0001,
	0x602A, 0x3770,
	0x6F12, 0x0100,
	0x602A, 0x220E,
	0x6F12, 0x0005,
	0x6F12, 0x0005,
	0x602A, 0x31CC,
	0x6F12, 0x0019,
	0x602A, 0x31D2,
	0x6F12, 0x0012,
	0x602A, 0x31CA,
	0x6F12, 0x000A,
	0x602A, 0x221C,
	0x6F12, 0x0101,
	0x602A, 0x2A18,
	0x6F12, 0x0801,
	0x602A, 0x5090,
	0x6F12, 0x0E00,
	0x602A, 0x84F8,
	0x6F12, 0x0002,
	0x602A, 0x8508,
	0x6F12, 0x0200,
	0x6F12, 0x0000,
	0x602A, 0x5530,
	0x6F12, 0x0100,
	0x602A, 0x43B0,
	0x6F12, 0x0101,
	0x602A, 0x43D4,
	0x6F12, 0x0000,
	0x6F12, 0x0004,
	0x6F12, 0x13E8,
	0x602A, 0x2A0E,
	0x6F12, 0x1FCD,
	0x602A, 0x1346,
	0x6F12, 0x0018,
	0x602A, 0x2308,
	0x6F12, 0x005B,
	0x602A, 0x381A,
	0x6F12, 0x0000,
	0x6F12, 0x3FFF,
	0x602A, 0x3812,
	0x6F12, 0x0003,
	0x602A, 0x345E,
	0x6F12, 0x1F40,
	0x6F12, 0x1F47,
	0x602A, 0x3B1A,
	0x6F12, 0x000F,
	0x6F12, 0x000F,
	0x6F12, 0x000A,
	0x6F12, 0x000A,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x000A,
	0x6F12, 0x000A,
	0x6F12, 0x000F,
	0x6F12, 0x000F,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x3A9A,
	0x6F12, 0x0050,
	0x6F12, 0x0050,
	0x6F12, 0xFFC0,
	0x6F12, 0xFFC0,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xFFC0,
	0x6F12, 0xFFC0,
	0x6F12, 0x0050,
	0x6F12, 0x0050,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x39EE,
	0x6F12, 0x0100,
	0x6F12, 0x0100,
	0x6F12, 0x0200,
	0x6F12, 0x1020,
	0x6F12, 0x1000,
	0x602A, 0x2A1E,
	0x6F12, 0x0108,
	0x602A, 0x134C,
	0x6F12, 0x0001,
	0x602A, 0x30B8,
	0x6F12, 0xD020,
	0x602A, 0x2580,
	0x6F12, 0x000E,
	0x602A, 0x2584,
	0x6F12, 0x0226,
	0x602A, 0x8A10,
	0x6F12, 0x0000,
	0x602A, 0x8A6E,
	0x6F12, 0x0FA0,
	0x6F12, 0x0FA7,
	0x602A, 0x8A20,
	0x6F12, 0x0008,
	0x602A, 0x8A26,
	0x6F12, 0x0008,
	0x602A, 0x8A1E,
	0x6F12, 0x0080,
	0x602A, 0x8A68,
	0x6F12, 0x0200,
	0x602A, 0x8A3E,
	0x6F12, 0x03FF,
	0x602A, 0x8A44,
	0x6F12, 0x0080,
	0x602A, 0x1370,
	0x6F12, 0x0000,
	0x602A, 0x1378,
	0x6F12, 0x0003,
	0x602A, 0x2134,
	0x6F12, 0xFFFF,
	0x602A, 0x1FC0,
	0x6F12, 0x0100,
	0x6F12, 0x005F,
	0x602A, 0x1FC6,
	0x6F12, 0x0060,
	0x602A, 0x1FCC,
	0x6F12, 0x1C80,
	0x6F12, 0x1C82,
	0x6F12, 0x4000,
	0x6F12, 0xF4B2,
	0x6028, 0x4000,
	0xF484, 0x0080,
	0xF498, 0x0343,
	0xF496, 0x0200,
	0xF4B2, 0x1C82,
	0xF41C, 0x4116,
	0xF418, 0x001F,
	0xB606, 0x0400,
	0xF482, 0x800D,
	0x6028, 0x4000,
	0x623C, 0x0080,
	0x623E, 0x0000,
	0x6240, 0x0000,
	0xF4A6, 0x0016,
	0x0344, 0x0018,
	0x0346, 0x0018,
	0x0348, 0x1997,
	0x034A, 0x133F,
	0x034C, 0x0CC0,
	0x034E, 0x0990,
	0x0350, 0x0000,
	0x0352, 0x0002,
	0x0900, 0x2222,
	0x0380, 0x0002,
	0x0382, 0x0002,
	0x0384, 0x0002,
	0x0386, 0x0002,
	0x0400, 0x1010,
	0x0408, 0x0100,
	0x040A, 0x0100,
	0x040C, 0x0000,
	0x0110, 0x1002,
	0x0114, 0x0100,
	0x0116, 0x3000,
	0x0118, 0x0001,
	0x011A, 0x0000,
	0x011C, 0x0000,
	0x0136, 0x1A00,
	0x013E, 0x0000,
	0x0300, 0x0007,
	0x0302, 0x0001,
	0x0304, 0x0003,
	0x0306, 0x00A1,
	0x0308, 0x000B,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0003,
	0x0310, 0x00Cf,
	0x0312, 0x0001,
	0x0340, 0x0CA4,//FLL
	0x0342, 0x4000,//LLP
	0x0702, 0x0000,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x022C, 0x0100,
	0x0226, 0x0100,
	0x021E, 0x0000,
	0x080E, 0x0000,
	0x0B00, 0x0080,
	0x0B08, 0x0100,
	0x0D00, 0x0000,

};

static kal_uint16 addr_data_pair_capture[] = {
	0x6028, 0x2000,
	0x602A, 0x2A08,
	0x6F12, 0x0001,
	0x6F12, 0xA5E0,
	0x602A, 0x21F8,
	0x6F12, 0x1358,
	0x602A, 0x30CE,
	0x6F12, 0x01E4,
	0x602A, 0x8A00,
	0x6F12, 0x0001,
	0x602A, 0x3770,
	0x6F12, 0x0100,
	0x602A, 0x220E,
	0x6F12, 0x0005,
	0x6F12, 0x0005,
	0x602A, 0x31CC,
	0x6F12, 0x0019,
	0x602A, 0x31D2,
	0x6F12, 0x0012,
	0x602A, 0x31CA,
	0x6F12, 0x000A,
	0x602A, 0x221C,
	0x6F12, 0x0101,
	0x602A, 0x2A18,
	0x6F12, 0x0801,
	0x602A, 0x5090,
	0x6F12, 0x0E00,
	0x602A, 0x84F8,
	0x6F12, 0x0002,
	0x602A, 0x8508,
	0x6F12, 0x0200,
	0x6F12, 0x0000,
	0x602A, 0x5530,
	0x6F12, 0x0100,
	0x602A, 0x43B0,
	0x6F12, 0x0101,
	0x602A, 0x43D4,
	0x6F12, 0x0000,
	0x6F12, 0x0004,
	0x6F12, 0x13E8,
	0x602A, 0x2A0E,
	0x6F12, 0x1FCD,
	0x602A, 0x1346,
	0x6F12, 0x0018,
	0x602A, 0x2308,
	0x6F12, 0x005B,
	0x602A, 0x381A,
	0x6F12, 0x0000,
	0x6F12, 0x3FFF,
	0x602A, 0x3812,
	0x6F12, 0x0003,
	0x602A, 0x345E,
	0x6F12, 0x1F40,
	0x6F12, 0x1F47,
	0x602A, 0x3B1A,
	0x6F12, 0x000F,
	0x6F12, 0x000F,
	0x6F12, 0x000A,
	0x6F12, 0x000A,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x000A,
	0x6F12, 0x000A,
	0x6F12, 0x000F,
	0x6F12, 0x000F,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x3A9A,
	0x6F12, 0x0050,
	0x6F12, 0x0050,
	0x6F12, 0xFFC0,
	0x6F12, 0xFFC0,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xFFC0,
	0x6F12, 0xFFC0,
	0x6F12, 0x0050,
	0x6F12, 0x0050,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x39EE,
	0x6F12, 0x0100,
	0x6F12, 0x0100,
	0x6F12, 0x0200,
	0x6F12, 0x1020,
	0x6F12, 0x1000,
	0x602A, 0x2A1E,
	0x6F12, 0x0108,
	0x602A, 0x134C,
	0x6F12, 0x0001,
	0x602A, 0x30B8,
	0x6F12, 0xD020,
	0x602A, 0x2580,
	0x6F12, 0x000E,
	0x602A, 0x2584,
	0x6F12, 0x0226,
	0x602A, 0x8A10,
	0x6F12, 0x0000,
	0x602A, 0x8A6E,
	0x6F12, 0x0FA0,
	0x6F12, 0x0FA7,
	0x602A, 0x8A20,
	0x6F12, 0x0008,
	0x602A, 0x8A26,
	0x6F12, 0x0008,
	0x602A, 0x8A1E,
	0x6F12, 0x0080,
	0x602A, 0x8A68,
	0x6F12, 0x0200,
	0x602A, 0x8A3E,
	0x6F12, 0x03FF,
	0x602A, 0x8A44,
	0x6F12, 0x0080,
	0x602A, 0x1370,
	0x6F12, 0x0000,
	0x602A, 0x1378,
	0x6F12, 0x0003,
	0x602A, 0x2134,
	0x6F12, 0xFFFF,
	0x602A, 0x1FC0,
	0x6F12, 0x0100,
	0x6F12, 0x005F,
	0x602A, 0x1FC6,
	0x6F12, 0x0060,
	0x602A, 0x1FCC,
	0x6F12, 0x1C80,
	0x6F12, 0x1C82,
	0x6F12, 0x4000,
	0x6F12, 0xF4B2,
	0x6028, 0x4000,
	0xF484, 0x0080,
	0xF498, 0x0343,
	0xF496, 0x0200,
	0xF4B2, 0x1C82,
	0xF41C, 0x4116,
	0xF418, 0x001F,
	0xB606, 0x0400,
	0xF482, 0x800D,
	0x6028, 0x4000,
	0x623C, 0x0080,
	0x623E, 0x0000,
	0x6240, 0x0000,
	0xF4A6, 0x0016,
	0x0344, 0x0018,
	0x0346, 0x0018,
	0x0348, 0x1997,
	0x034A, 0x133F,
	0x034C, 0x0CC0,
	0x034E, 0x0990,
	0x0350, 0x0000,
	0x0352, 0x0002,
	0x0900, 0x2222,
	0x0380, 0x0002,
	0x0382, 0x0002,
	0x0384, 0x0002,
	0x0386, 0x0002,
	0x0400, 0x1010,
	0x0408, 0x0100,
	0x040A, 0x0100,
	0x040C, 0x0000,
	0x0110, 0x1002,
	0x0114, 0x0100,
	0x0116, 0x3000,
	0x0118, 0x0001,
	0x011A, 0x0000,
	0x011C, 0x0000,
	0x0136, 0x1A00,
	0x013E, 0x0000,
	0x0300, 0x0007,
	0x0302, 0x0001,
	0x0304, 0x0003,
	0x0306, 0x00A1,
	0x0308, 0x000B,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0003,
	0x0310, 0x00Cf,
	0x0312, 0x0001,
	0x0340, 0x0AC0,
	0x0342, 0x4B00,
	0x0702, 0x0000,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x022C, 0x0100,
	0x0226, 0x0100,
	0x021E, 0x0000,
	0x080E, 0x0000,
	0x0B00, 0x0080,
	0x0B08, 0x0100,
	0x0D00, 0x0000,

};

static kal_uint16 addr_data_pair_normal_video[] = {
	0x6028, 0x2000,
	0x602A, 0x2A08,
	0x6F12, 0x0001,
	0x6F12, 0xA5E0,
	0x602A, 0x21F8,
	0x6F12, 0x1358,
	0x602A, 0x30CE,
	0x6F12, 0x01E4,
	0x602A, 0x8A00,
	0x6F12, 0x0001,
	0x602A, 0x3770,
	0x6F12, 0x0100,
	0x602A, 0x220E,
	0x6F12, 0x0005,
	0x6F12, 0x0005,
	0x602A, 0x31CC,
	0x6F12, 0x0019,
	0x602A, 0x31D2,
	0x6F12, 0x0012,
	0x602A, 0x31CA,
	0x6F12, 0x000A,
	0x602A, 0x221C,
	0x6F12, 0x0101,
	0x602A, 0x2A18,
	0x6F12, 0x0801,
	0x602A, 0x5090,
	0x6F12, 0x0E00,
	0x602A, 0x84F8,
	0x6F12, 0x0002,
	0x602A, 0x8508,
	0x6F12, 0x0200,
	0x6F12, 0x0000,
	0x602A, 0x5530,
	0x6F12, 0x0100,
	0x602A, 0x43B0,
	0x6F12, 0x0101,
	0x602A, 0x43D4,
	0x6F12, 0x0000,
	0x6F12, 0x0004,
	0x6F12, 0x13E8,
	0x602A, 0x2A0E,
	0x6F12, 0x1FCD,
	0x602A, 0x1346,
	0x6F12, 0x0018,
	0x602A, 0x2308,
	0x6F12, 0x005B,
	0x602A, 0x381A,
	0x6F12, 0x0000,
	0x6F12, 0x3FFF,
	0x602A, 0x3812,
	0x6F12, 0x0003,
	0x602A, 0x345E,
	0x6F12, 0x1F40,
	0x6F12, 0x1F47,
	0x602A, 0x3B1A,
	0x6F12, 0x000F,
	0x6F12, 0x000F,
	0x6F12, 0x000A,
	0x6F12, 0x000A,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x000A,
	0x6F12, 0x000A,
	0x6F12, 0x000F,
	0x6F12, 0x000F,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x3A9A,
	0x6F12, 0x0050,
	0x6F12, 0x0050,
	0x6F12, 0xFFC0,
	0x6F12, 0xFFC0,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xFFC0,
	0x6F12, 0xFFC0,
	0x6F12, 0x0050,
	0x6F12, 0x0050,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x39EE,
	0x6F12, 0x0100,
	0x6F12, 0x0100,
	0x6F12, 0x0200,
	0x6F12, 0x1020,
	0x6F12, 0x1000,
	0x602A, 0x2A1E,
	0x6F12, 0x0108,
	0x602A, 0x134C,
	0x6F12, 0x0001,
	0x602A, 0x30B8,
	0x6F12, 0xD020,
	0x602A, 0x2580,
	0x6F12, 0x000E,
	0x602A, 0x2584,
	0x6F12, 0x0226,
	0x602A, 0x8A10,
	0x6F12, 0x0000,
	0x602A, 0x8A6E,
	0x6F12, 0x0FA0,
	0x6F12, 0x0FA7,
	0x602A, 0x8A20,
	0x6F12, 0x0008,
	0x602A, 0x8A26,
	0x6F12, 0x0008,
	0x602A, 0x8A1E,
	0x6F12, 0x0080,
	0x602A, 0x8A68,
	0x6F12, 0x0200,
	0x602A, 0x8A3E,
	0x6F12, 0x03FF,
	0x602A, 0x8A44,
	0x6F12, 0x0080,
	0x602A, 0x1370,
	0x6F12, 0x0000,
	0x602A, 0x1378,
	0x6F12, 0x0003,
	0x602A, 0x2134,
	0x6F12, 0xFFFF,
	0x602A, 0x1FC0,
	0x6F12, 0x0100,
	0x6F12, 0x005F,
	0x602A, 0x1FC6,
	0x6F12, 0x0060,
	0x602A, 0x1FCC,
	0x6F12, 0x1C80,
	0x6F12, 0x1C82,
	0x6F12, 0x4000,
	0x6F12, 0xF4B2,
	0x6028, 0x4000,
	0xF484, 0x0080,
	0xF498, 0x0343,
	0xF496, 0x0200,
	0xF4B2, 0x1C82,
	0xF41C, 0xA116,
	0xF418, 0x001F,
	0xB606, 0x0300,
	0xF482, 0x800D,
	0x6028, 0x4000,
	0x623C, 0x0080,
	0x623E, 0x0000,
	0x6240, 0x0000,
	0xF4A6, 0x0016,
	0x0344, 0x0018,
	0x0346, 0x0280,
	0x0348, 0x1997,
	0x034A, 0x10D7,
	0x034C, 0x0CC0,
	0x034E, 0x072C,
	0x0350, 0x0000,
	0x0352, 0x0000,
	0x0900, 0x2222,
	0x0380, 0x0002,
	0x0382, 0x0002,
	0x0384, 0x0002,
	0x0386, 0x0002,
	0x0400, 0x1010,
	0x0408, 0x0100,
	0x040A, 0x0100,
	0x040C, 0x0000,
	0x0110, 0x1002,
	0x0114, 0x0100,
	0x0116, 0x3000,
	0x0118, 0x0001,
	0x011A, 0x0000,
	0x011C, 0x0000,
	0x0136, 0x1A00,
	0x013E, 0x0000,
	0x0300, 0x0007,
	0x0302, 0x0001,
	0x0304, 0x0003,
	0x0306, 0x00A1,
	0x0308, 0x000B,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0003,
	0x0310, 0x00CF,
	0x0312, 0x0001,
	0x0340, 0x0B40,//FLL
	0x0342, 0x4808,//LLP
	0x0702, 0x0000,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x022C, 0x0100,
	0x0226, 0x0100,
	0x021E, 0x0000,
	0x080E, 0x0800,
	0x0B00, 0x0080,
	0x0B08, 0x0100,
	0x0D00, 0x0000,

};

static kal_uint16 addr_data_pair_hs_video[] = {
	0x6028, 0x2000,
	0x602A, 0x2A08,
	0x6F12, 0x0001,
	0x6F12, 0xA5E0,
	0x602A, 0x21F8,
	0x6F12, 0x1358,
	0x602A, 0x30CE,
	0x6F12, 0x01E4,
	0x602A, 0x8A00,
	0x6F12, 0x0001,
	0x602A, 0x3770,
	0x6F12, 0x0100,
	0x602A, 0x220E,
	0x6F12, 0x0005,
	0x6F12, 0x0005,
	0x602A, 0x31CC,
	0x6F12, 0x0019,
	0x602A, 0x31D2,
	0x6F12, 0x0012,
	0x602A, 0x31CA,
	0x6F12, 0x000A,
	0x602A, 0x221C,
	0x6F12, 0x0101,
	0x602A, 0x2A18,
	0x6F12, 0x0801,
	0x602A, 0x5090,
	0x6F12, 0x0E00,
	0x602A, 0x84F8,
	0x6F12, 0x0002,
	0x602A, 0x8508,
	0x6F12, 0x0200,
	0x6F12, 0x0000,
	0x602A, 0x5530,
	0x6F12, 0x0100,
	0x602A, 0x43B0,
	0x6F12, 0x0101,
	0x602A, 0x43D4,
	0x6F12, 0x0000,
	0x6F12, 0x0004,
	0x6F12, 0x13E8,
	0x602A, 0x2A0E,
	0x6F12, 0x1FCD,
	0x602A, 0x1346,
	0x6F12, 0x0018,
	0x602A, 0x2308,
	0x6F12, 0x005B,
	0x602A, 0x381A,
	0x6F12, 0x0000,
	0x6F12, 0x3FFF,
	0x602A, 0x3812,
	0x6F12, 0x0003,
	0x602A, 0x345E,
	0x6F12, 0x1F40,
	0x6F12, 0x1F47,
	0x602A, 0x3B1A,
	0x6F12, 0x000F,
	0x6F12, 0x000F,
	0x6F12, 0x000A,
	0x6F12, 0x000A,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x000A,
	0x6F12, 0x000A,
	0x6F12, 0x000F,
	0x6F12, 0x000F,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x3A9A,
	0x6F12, 0x0050,
	0x6F12, 0x0050,
	0x6F12, 0xFFC0,
	0x6F12, 0xFFC0,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xFFC0,
	0x6F12, 0xFFC0,
	0x6F12, 0x0050,
	0x6F12, 0x0050,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x39EE,
	0x6F12, 0x0100,
	0x6F12, 0x0100,
	0x6F12, 0x0200,
	0x6F12, 0x1020,
	0x6F12, 0x1000,
	0x602A, 0x2A1E,
	0x6F12, 0x0108,
	0x602A, 0x134C,
	0x6F12, 0x0001,
	0x602A, 0x30B8,
	0x6F12, 0xD020,
	0x602A, 0x2580,
	0x6F12, 0x000E,
	0x602A, 0x2584,
	0x6F12, 0x0226,
	0x602A, 0x8A10,
	0x6F12, 0x0000,
	0x602A, 0x8A6E,
	0x6F12, 0x0FA0,
	0x6F12, 0x0FA7,
	0x602A, 0x8A20,
	0x6F12, 0x0008,
	0x602A, 0x8A26,
	0x6F12, 0x0008,
	0x602A, 0x8A1E,
	0x6F12, 0x0080,
	0x602A, 0x8A68,
	0x6F12, 0x0200,
	0x602A, 0x8A3E,
	0x6F12, 0x03FF,
	0x602A, 0x8A44,
	0x6F12, 0x0080,
	0x602A, 0x1370,
	0x6F12, 0x0000,
	0x602A, 0x1378,
	0x6F12, 0x0003,
	0x602A, 0x2134,
	0x6F12, 0xFFFF,
	0x602A, 0x1FC0,
	0x6F12, 0x0100,
	0x6F12, 0x005F,
	0x602A, 0x1FC6,
	0x6F12, 0x0060,
	0x602A, 0x1FCC,
	0x6F12, 0x1C80,
	0x6F12, 0x1C82,
	0x6F12, 0x4000,
	0x6F12, 0xF4B2,
	0x6028, 0x4000,
	0xF484, 0x0080,
	0xF498, 0x0343,
	0xF496, 0x0200,
	0xF4B2, 0x1C82,
	0xF41C, 0x4116,
	0xF418, 0x001F,
	0xB606, 0x0400,
	0xF482, 0x800D,
	0x6028, 0x4000,
	0x623C, 0x0080,
	0x623E, 0x0000,
	0x6240, 0x0000,
	0xF4A6, 0x0016,
	0x0344, 0x0018,
	0x0346, 0x0018,
	0x0348, 0x1997,
	0x034A, 0x133F,
	0x034C, 0x0CC0,
	0x034E, 0x0990,
	0x0350, 0x0000,
	0x0352, 0x0002,
	0x0900, 0x2222,
	0x0380, 0x0002,
	0x0382, 0x0002,
	0x0384, 0x0002,
	0x0386, 0x0002,
	0x0400, 0x1010,
	0x0408, 0x0100,
	0x040A, 0x0100,
	0x040C, 0x0000,
	0x0110, 0x1002,
	0x0114, 0x0100,
	0x0116, 0x3000,
	0x0118, 0x0001,
	0x011A, 0x0000,
	0x011C, 0x0000,
	0x0136, 0x1A00,
	0x013E, 0x0000,
	0x0300, 0x0007,
	0x0302, 0x0001,
	0x0304, 0x0003,
	0x0306, 0x00A1,
	0x0308, 0x000B,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0003,
	0x0310, 0x00Cf,
	0x0312, 0x0001,
	0x0340, 0x0AC0,
	0x0342, 0x4B00,
	0x0702, 0x0000,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x022C, 0x0100,
	0x0226, 0x0100,
	0x021E, 0x0000,
	0x080E, 0x0000,
	0x0B00, 0x0080,
	0x0B08, 0x0100,
	0x0D00, 0x0000,

};

static kal_uint16 addr_data_pair_slim_video[] = {
	0x6028, 0x2000,
	0x602A, 0x2A08,
	0x6F12, 0x0001,
	0x6F12, 0xA5E0,
	0x602A, 0x21F8,
	0x6F12, 0x1358,
	0x602A, 0x30CE,
	0x6F12, 0x01E4,
	0x602A, 0x8A00,
	0x6F12, 0x0001,
	0x602A, 0x3770,
	0x6F12, 0x0100,
	0x602A, 0x220E,
	0x6F12, 0x0005,
	0x6F12, 0x0005,
	0x602A, 0x31CC,
	0x6F12, 0x0019,
	0x602A, 0x31D2,
	0x6F12, 0x0012,
	0x602A, 0x31CA,
	0x6F12, 0x000A,
	0x602A, 0x221C,
	0x6F12, 0x0101,
	0x602A, 0x2A18,
	0x6F12, 0x0801,
	0x602A, 0x5090,
	0x6F12, 0x0E00,
	0x602A, 0x84F8,
	0x6F12, 0x0002,
	0x602A, 0x8508,
	0x6F12, 0x0200,
	0x6F12, 0x0000,
	0x602A, 0x5530,
	0x6F12, 0x0100,
	0x602A, 0x43B0,
	0x6F12, 0x0101,
	0x602A, 0x43D4,
	0x6F12, 0x0000,
	0x6F12, 0x0004,
	0x6F12, 0x13E8,
	0x602A, 0x2A0E,
	0x6F12, 0x1FCD,
	0x602A, 0x1346,
	0x6F12, 0x0018,
	0x602A, 0x2308,
	0x6F12, 0x005B,
	0x602A, 0x381A,
	0x6F12, 0x0000,
	0x6F12, 0x3FFF,
	0x602A, 0x3812,
	0x6F12, 0x0003,
	0x602A, 0x345E,
	0x6F12, 0x1F40,
	0x6F12, 0x1F47,
	0x602A, 0x3B1A,
	0x6F12, 0x000F,
	0x6F12, 0x000F,
	0x6F12, 0x000A,
	0x6F12, 0x000A,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x000A,
	0x6F12, 0x000A,
	0x6F12, 0x000F,
	0x6F12, 0x000F,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x3A9A,
	0x6F12, 0x0050,
	0x6F12, 0x0050,
	0x6F12, 0xFFC0,
	0x6F12, 0xFFC0,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xFFC0,
	0x6F12, 0xFFC0,
	0x6F12, 0x0050,
	0x6F12, 0x0050,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x39EE,
	0x6F12, 0x0100,
	0x6F12, 0x0100,
	0x6F12, 0x0200,
	0x6F12, 0x1020,
	0x6F12, 0x1000,
	0x602A, 0x2A1E,
	0x6F12, 0x0108,
	0x602A, 0x134C,
	0x6F12, 0x0001,
	0x602A, 0x30B8,
	0x6F12, 0xD020,
	0x602A, 0x2580,
	0x6F12, 0x000E,
	0x602A, 0x2584,
	0x6F12, 0x0226,
	0x602A, 0x8A10,
	0x6F12, 0x0000,
	0x602A, 0x8A6E,
	0x6F12, 0x0FA0,
	0x6F12, 0x0FA7,
	0x602A, 0x8A20,
	0x6F12, 0x0008,
	0x602A, 0x8A26,
	0x6F12, 0x0008,
	0x602A, 0x8A1E,
	0x6F12, 0x0080,
	0x602A, 0x8A68,
	0x6F12, 0x0200,
	0x602A, 0x8A3E,
	0x6F12, 0x03FF,
	0x602A, 0x8A44,
	0x6F12, 0x0080,
	0x602A, 0x1370,
	0x6F12, 0x0000,
	0x602A, 0x1378,
	0x6F12, 0x0003,
	0x602A, 0x2134,
	0x6F12, 0xFFFF,
	0x602A, 0x1FC0,
	0x6F12, 0x0100,
	0x6F12, 0x005F,
	0x602A, 0x1FC6,
	0x6F12, 0x0060,
	0x602A, 0x1FCC,
	0x6F12, 0x1C80,
	0x6F12, 0x1C82,
	0x6F12, 0x4000,
	0x6F12, 0xF4B2,
	0x6028, 0x4000,
	0xF484, 0x0080,
	0xF498, 0x0343,
	0xF496, 0x0200,
	0xF4B2, 0x1C82,
	0xF41C, 0x4116,
	0xF418, 0x001F,
	0xB606, 0x0400,
	0xF482, 0x800D,
	0x6028, 0x4000,
	0x623C, 0x0080,
	0x623E, 0x0000,
	0x6240, 0x0000,
	0xF4A6, 0x0016,
	0x0344, 0x0018,
	0x0346, 0x0018,
	0x0348, 0x1997,
	0x034A, 0x133F,
	0x034C, 0x0CC0,
	0x034E, 0x0990,
	0x0350, 0x0000,
	0x0352, 0x0002,
	0x0900, 0x2222,
	0x0380, 0x0002,
	0x0382, 0x0002,
	0x0384, 0x0002,
	0x0386, 0x0002,
	0x0400, 0x1010,
	0x0408, 0x0100,
	0x040A, 0x0100,
	0x040C, 0x0000,
	0x0110, 0x1002,
	0x0114, 0x0100,
	0x0116, 0x3000,
	0x0118, 0x0001,
	0x011A, 0x0000,
	0x011C, 0x0000,
	0x0136, 0x1A00,
	0x013E, 0x0000,
	0x0300, 0x0007,
	0x0302, 0x0001,
	0x0304, 0x0003,
	0x0306, 0x00A1,
	0x0308, 0x000B,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0003,
	0x0310, 0x00Cf,
	0x0312, 0x0001,
	0x0340, 0x0AC0,
	0x0342, 0x4B00,
	0x0702, 0x0000,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x022C, 0x0100,
	0x0226, 0x0100,
	0x021E, 0x0000,
	0x080E, 0x0000,
	0x0B00, 0x0080,
	0x0B08, 0x0100,
	0x0D00, 0x0000,

};

/*4:3 3d-hdr binning mode */
static kal_uint16 addr_data_pair_custom1[] = {
0x6028,	0x2000,
0x602A,	0x2A08,
0x6F12,	0x0001,
0x6F12,	0xA5E0,
0x602A,	0x21F8,
0x6F12,	0x1358,
0x602A,	0x30CE,
0x6F12,	0x00E4,
0x602A,	0x8A00,
0x6F12,	0x0001,
0x602A,	0x3770,
0x6F12,	0x0100,
0x602A,	0x220E,
0x6F12,	0x0005,
0x6F12,	0x0005,
0x602A,	0x31CC,
0x6F12,	0x0019,
0x602A,	0x31D2,
0x6F12,	0x0012,
0x602A,	0x31CA,
0x6F12,	0x000A,
0x602A,	0x221C,
0x6F12,	0x0101,
0x602A,	0x2A18,
0x6F12,	0x1001,
0x602A,	0x5090,
0x6F12,	0x0E00,
0x602A,	0x84F8,
0x6F12,	0x0002,
0x602A,	0x8508,
0x6F12,	0x0200,
0x6F12,	0x0000,
0x602A,	0x5530,
0x6F12,	0x0100,
0x602A,	0x43B0,
0x6F12,	0x0101,
0x602A,	0x43D4,
0x6F12,	0x0000,
0x6F12,	0x0004,
0x6F12,	0x13E8,
0x602A,	0x2A0E,
0x6F12,	0x1FCD,
0x602A,	0x1346,
0x6F12,	0x0018,
0x602A,	0x2308,
0x6F12,	0x002F,
0x602A,	0x381A,
0x6F12,	0x0000,
0x6F12,	0x3FFF,
0x602A,	0x3812,
0x6F12,	0x0003,
0x602A,	0x345E,
0x6F12,	0x1F40,
0x6F12,	0x1F47,
0x602A,	0x3B1A,
0x6F12,	0x000F,
0x6F12,	0x000F,
0x6F12,	0x000A,
0x6F12,	0x000A,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x000A,
0x6F12,	0x000A,
0x6F12,	0x000F,
0x6F12,	0x000F,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x602A,	0x3A9A,
0x6F12,	0x0050,
0x6F12,	0x0050,
0x6F12,	0xFFC0,
0x6F12,	0xFFC0,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0xFFC0,
0x6F12,	0xFFC0,
0x6F12,	0x0050,
0x6F12,	0x0050,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x602A,	0x39EE,
0x6F12,	0x0100,
0x6F12,	0x0100,
0x6F12,	0x0200,
0x6F12,	0x1020,
0x6F12,	0x1000,
0x602A,	0x2A1E,
0x6F12,	0x0108,
0x602A,	0x134C,
0x6F12,	0x0001,
0x602A,	0x30B8,
0x6F12,	0xD020,
0x602A,	0x2580,
0x6F12,	0x000E,
0x602A,	0x2584,
0x6F12,	0x0226,
0x602A,	0x8A10,
0x6F12,	0x0000,
0x602A,	0x8A6E,
0x6F12,	0x0FA0,
0x6F12,	0x0FA7,
0x602A,	0x8A20,
0x6F12,	0x0008,
0x602A,	0x8A26,
0x6F12,	0x0008,
0x602A,	0x8A1E,
0x6F12,	0x0080,
0x602A,	0x8A68,
0x6F12,	0x0200,
0x602A,	0x8A3E,
0x6F12,	0x03FF,
0x602A,	0x8A44,
0x6F12,	0x0080,
0x602A,	0x1370,
0x6F12,	0x0000,
0x602A,	0x1378,
0x6F12,	0x0003,
0x602A,	0x2134,
0x6F12,	0xFFFF,
0x602A,	0x1FC0,
0x6F12,	0x0100,
0x6F12,	0x005F,
0x602A,	0x1FC6,
0x6F12,	0x0060,
0x602A,	0x1FCC,
0x6F12,	0x1C80,
0x6F12,	0x1C82,
0x6F12,	0x4000,
0x6F12,	0xF4B2,
0x6028,	0x4000,
0xF484,	0x0080,
0xF498,	0x0343,
0xF496,	0x0200,
0xF4B2,	0x1C82,
0xF41C,	0xC116,
0xF418,	0x001F,
0xB606,	0x0300,
0xF482,	0x800D,
0x6028,	0x4000,
0x623C,	0x0080,
0x623E,	0x0000,
0x6240,	0x0000,
0xF4A6,	0x0016,
0x0344,	0x0018,
0x0346,	0x001C,
0x0348,	0x1997,
0x034A,	0x133B,
0x034C,	0x0660,
0x034E,	0x04C8,
0x0350,	0x0000,
0x0352,	0x0000,
0x0900,	0x2324,
0x0380,	0x0002,
0x0382,	0x0002,
0x0384,	0x0004,
0x0386,	0x0004,
0x0400,	0x1010,
0x0408,	0x0200,
0x040A,	0x0100,
0x040C,	0x0000,
0x0110,	0x1002,
0x0114,	0x0100,
0x0116,	0x3000,
0x0118,	0x0001,
0x011A,	0x0000,
0x011C,	0x0000,
0x0136,	0x1A00,
0x013E,	0x0000,
0x0300,	0x0007,
0x0302,	0x0001,
0x0304,	0x0003,
0x0306,	0x00A1,
0x0308,	0x000B,
0x030A,	0x0001,
0x030C,	0x0000,
0x030E,	0x0003,
0x0310,	0x00D0,
0x0312,	0x0002,
0x0340,	0x0B74,
0x0342,	0x4650,
0x0702,	0x0000,
0x0202,	0x0100,
0x0200,	0x0100,
0x022C,	0x0100,
0x0226,	0x0100,
0x021E,	0x0000,
0x080E,	0x0800,
0x0B00,	0x0080,
0x0B08,	0x0100,
0x0D00,	0x0000,

};
/*16:9 bin0008ning mode */
static kal_uint16 addr_data_pair_custom2[] = {
0x6028,	0x2000,
0x602A,	0x2A08,
0x6F12,	0x0001,
0x6F12,	0xA5E0,
0x602A,	0x21F8,
0x6F12,	0x1358,
0x602A,	0x30CE,
0x6F12,	0x01E4,
0x602A,	0x8A00,
0x6F12,	0x0001,
0x602A,	0x3770,
0x6F12,	0x0100,
0x602A,	0x220E,
0x6F12,	0x0005,
0x6F12,	0x0005,
0x602A,	0x31CC,
0x6F12,	0x0019,
0x602A,	0x31D2,
0x6F12,	0x0012,
0x602A,	0x31CA,
0x6F12,	0x000A,
0x602A,	0x221C,
0x6F12,	0x0101,
0x602A,	0x2A18,
0x6F12,	0x0801,
0x602A,	0x5090,
0x6F12,	0x0E00,
0x602A,	0x84F8,
0x6F12,	0x0002,
0x602A,	0x8508,
0x6F12,	0x0200,
0x6F12,	0x0000,
0x602A,	0x5530,
0x6F12,	0x0100,
0x602A,	0x43B0,
0x6F12,	0x0101,
0x602A,	0x43D4,
0x6F12,	0x0000,
0x6F12,	0x0004,
0x6F12,	0x13E8,
0x602A,	0x2A0E,
0x6F12,	0x1FCD,
0x602A,	0x1346,
0x6F12,	0x0018,
0x602A,	0x2308,
0x6F12,	0x005B,
0x602A,	0x381A,
0x6F12,	0x0000,
0x6F12,	0x3FFF,
0x602A,	0x3812,
0x6F12,	0x0003,
0x602A,	0x345E,
0x6F12,	0x1F40,
0x6F12,	0x1F47,
0x602A,	0x3B1A,
0x6F12,	0x000F,
0x6F12,	0x000F,
0x6F12,	0x000A,
0x6F12,	0x000A,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x000A,
0x6F12,	0x000A,
0x6F12,	0x000F,
0x6F12,	0x000F,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x602A,	0x3A9A,
0x6F12,	0x0050,
0x6F12,	0x0050,
0x6F12,	0xFFC0,
0x6F12,	0xFFC0,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0xFFC0,
0x6F12,	0xFFC0,
0x6F12,	0x0050,
0x6F12,	0x0050,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x602A,	0x39EE,
0x6F12,	0x0100,
0x6F12,	0x0100,
0x6F12,	0x0200,
0x6F12,	0x1020,
0x6F12,	0x1000,
0x602A,	0x2A1E,
0x6F12,	0x0108,
0x602A,	0x134C,
0x6F12,	0x0001,
0x602A,	0x30B8,
0x6F12,	0xD020,
0x602A,	0x2580,
0x6F12,	0x000E,
0x602A,	0x2584,
0x6F12,	0x0226,
0x602A,	0x8A10,
0x6F12,	0x0000,
0x602A,	0x8A6E,
0x6F12,	0x0FA0,
0x6F12,	0x0FA7,
0x602A,	0x8A20,
0x6F12,	0x0008,
0x602A,	0x8A26,
0x6F12,	0x0008,
0x602A,	0x8A1E,
0x6F12,	0x0080,
0x602A,	0x8A68,
0x6F12,	0x0200,
0x602A,	0x8A3E,
0x6F12,	0x03FF,
0x602A,	0x8A44,
0x6F12,	0x0080,
0x602A,	0x1370,
0x6F12,	0x0000,
0x602A,	0x1378,
0x6F12,	0x0003,
0x602A,	0x2134,
0x6F12,	0xFFFF,
0x602A,	0x1FC0,
0x6F12,	0x0100,
0x6F12,	0x005F,
0x602A,	0x1FC6,
0x6F12,	0x0060,
0x602A,	0x1FCC,
0x6F12,	0x1C80,
0x6F12,	0x1C82,
0x6F12,	0x4000,
0x6F12,	0xF4B2,
0x6028,	0x4000,
0xF484,	0x0080,
0xF498,	0x0343,
0xF496,	0x0200,
0xF4B2,	0x1C82,
0xF41C,	0xA116,
0xF418,	0x001F,
0xB606,	0x0300,
0xF482,	0x800D,
0x6028,	0x4000,
0x623C,	0x0080,
0x623E,	0x0000,
0x6240,	0x0000,
0xF4A6,	0x0016,
0x0344,	0x0018,
0x0346,	0x0280,
0x0348,	0x1997,
0x034A,	0x10D7,
0x034C,	0x0CC0,
0x034E,	0x072C,
0x0350,	0x0000,
0x0352,	0x0000,
0x0900,	0x2222,
0x0380,	0x0002,
0x0382,	0x0002,
0x0384,	0x0002,
0x0386,	0x0002,
0x0400,	0x1010,
0x0408,	0x0100,
0x040A,	0x0100,
0x040C,	0x0000,
0x0110,	0x1002,
0x0114,	0x0100,
0x0116,	0x3000,
0x0118,	0x0001,
0x011A,	0x0000,
0x011C,	0x0000,
0x0136,	0x1A00,
0x013E,	0x0000,
0x0300,	0x0007,
0x0302,	0x0001,
0x0304,	0x0003,
0x0306,	0x00A1,
0x0308,	0x000B,
0x030A,	0x0001,
0x030C,	0x0000,
0x030E,	0x0003,
0x0310,	0x00CF,
0x0312,	0x0001,
0x0340,	0x09D0,
0x0342,	0x5208,
0x0702,	0x0000,
0x0202,	0x0100,
0x0200,	0x0100,
0x022C,	0x0100,
0x0226,	0x0100,
0x021E,	0x0000,
0x080E,	0x0800,
0x0B00,	0x0080,
0x0B08,	0x0100,
0x0D00,	0x0000,

};

//SUM 1632 X 1224 30fps
static kal_uint16 addr_data_pair_custom3[] = {
//1632x1224 
0x6028,	0x2000,
0x602A,	0x2A08,
0x6F12,	0x0001,
0x6F12,	0xA5E0,
0x602A,	0x21F8,
0x6F12,	0x1358,
0x602A,	0x30CE,
0x6F12,	0x00E4,
0x602A,	0x8A00,
0x6F12,	0x0001,
0x602A,	0x3770,
0x6F12,	0x0100,
0x602A,	0x220E,
0x6F12,	0x0005,
0x6F12,	0x0005,
0x602A,	0x31CC,
0x6F12,	0x0019,
0x602A,	0x31D2,
0x6F12,	0x0012,
0x602A,	0x31CA,
0x6F12,	0x000A,
0x602A,	0x221C,
0x6F12,	0x0101,
0x602A,	0x2A18,
0x6F12,	0x1001,
0x602A,	0x5090,
0x6F12,	0x0E00,
0x602A,	0x84F8,
0x6F12,	0x0002,
0x602A,	0x8508,
0x6F12,	0x0200,
0x6F12,	0x0000,
0x602A,	0x5530,
0x6F12,	0x0100,
0x602A,	0x43B0,
0x6F12,	0x0101,
0x602A,	0x43D4,
0x6F12,	0x0000,
0x6F12,	0x0004,
0x6F12,	0x13E8,
0x602A,	0x2A0E,
0x6F12,	0x1FCD,
0x602A,	0x1346,
0x6F12,	0x0018,
0x602A,	0x2308,
0x6F12,	0x002F,
0x602A,	0x381A,
0x6F12,	0x0000,
0x6F12,	0x3FFF,
0x602A,	0x3812,
0x6F12,	0x0003,
0x602A,	0x345E,
0x6F12,	0x1F40,
0x6F12,	0x1F47,
0x602A,	0x3B1A,
0x6F12,	0x000F,
0x6F12,	0x000F,
0x6F12,	0x000A,
0x6F12,	0x000A,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x000A,
0x6F12,	0x000A,
0x6F12,	0x000F,
0x6F12,	0x000F,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x602A,	0x3A9A,
0x6F12,	0x0050,
0x6F12,	0x0050,
0x6F12,	0xFFC0,
0x6F12,	0xFFC0,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0xFFC0,
0x6F12,	0xFFC0,
0x6F12,	0x0050,
0x6F12,	0x0050,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x602A,	0x39EE,
0x6F12,	0x0100,
0x6F12,	0x0100,
0x6F12,	0x0200,
0x6F12,	0x1020,
0x6F12,	0x1000,
0x602A,	0x2A1E,
0x6F12,	0x0108,
0x602A,	0x134C,
0x6F12,	0x0001,
0x602A,	0x30B8,
0x6F12,	0xD020,
0x602A,	0x2580,
0x6F12,	0x000E,
0x602A,	0x2584,
0x6F12,	0x0226,
0x602A,	0x8A10,
0x6F12,	0x0000,
0x602A,	0x8A6E,
0x6F12,	0x0FA0,
0x6F12,	0x0FA7,
0x602A,	0x8A20,
0x6F12,	0x0008,
0x602A,	0x8A26,
0x6F12,	0x0008,
0x602A,	0x8A1E,
0x6F12,	0x0080,
0x602A,	0x8A68,
0x6F12,	0x0200,
0x602A,	0x8A3E,
0x6F12,	0x03FF,
0x602A,	0x8A44,
0x6F12,	0x0080,
0x602A,	0x1370,
0x6F12,	0x0000,
0x602A,	0x1378,
0x6F12,	0x0003,
0x602A,	0x2134,
0x6F12,	0xFFFF,
0x602A,	0x1FC0,
0x6F12,	0x0100,
0x6F12,	0x005F,
0x602A,	0x1FC6,
0x6F12,	0x0060,
0x602A,	0x1FCC,
0x6F12,	0x1C80,
0x6F12,	0x1C82,
0x6F12,	0x4000,
0x6F12,	0xF4B2,
0x6028,	0x4000,
0xF484,	0x0080,
0xF498,	0x0343,
0xF496,	0x0200,
0xF4B2,	0x1C82,
0xF41C,	0xC116,
0xF418,	0x001F,
0xB606,	0x0300,
0xF482,	0x800D,
0x6028,	0x4000,
0x623C,	0x0080,
0x623E,	0x0000,
0x6240,	0x0000,
0xF4A6,	0x0016,
0x0344,	0x0018,
0x0346,	0x001C,
0x0348,	0x1997,
0x034A,	0x133B,
0x034C,	0x0660,
0x034E,	0x04C8,
0x0350,	0x0000,
0x0352,	0x0000,
0x0900,	0x2324,
0x0380,	0x0002,
0x0382,	0x0002,
0x0384,	0x0004,
0x0386,	0x0004,
0x0400,	0x1010,
0x0408,	0x0200,
0x040A,	0x0100,
0x040C,	0x0000,
0x0110,	0x1002,
0x0114,	0x0100,
0x0116,	0x3000,
0x0118,	0x0001,
0x011A,	0x0000,
0x011C,	0x0000,
0x0136,	0x1A00,
0x013E,	0x0000,
0x0300,	0x0007,
0x0302,	0x0001,
0x0304,	0x0003,
0x0306,	0x00A1,
0x0308,	0x000B,
0x030A,	0x0001,
0x030C,	0x0000,
0x030E,	0x0003,
0x0310,	0x00D0,
0x0312,	0x0002,
0x0340,	0x0B74,
0x0342,	0x4650,
0x0702,	0x0000,
0x0202,	0x0100,
0x0200,	0x0100,
0x022C,	0x0100,
0x0226,	0x0100,
0x021E,	0x0000,
0x080E,	0x0800,
0x0B00,	0x0080,
0x0B08,	0x0100,
0x0D00,	0x0000,
};	

static kal_uint16 addr_data_pair_custom4[] = {
 //1632x1224 
 0x6028,	0x2000,
0x602A,	0x2A08,
0x6F12,	0x0001,
0x6F12,	0xA5E0,
0x602A,	0x21F8,
0x6F12,	0x1358,
0x602A,	0x30CE,
0x6F12,	0x00E4,
0x602A,	0x8A00,
0x6F12,	0x0001,
0x602A,	0x3770,
0x6F12,	0x0100,
0x602A,	0x220E,
0x6F12,	0x0005,
0x6F12,	0x0005,
0x602A,	0x31CC,
0x6F12,	0x0019,
0x602A,	0x31D2,
0x6F12,	0x0012,
0x602A,	0x31CA,
0x6F12,	0x000A,
0x602A,	0x221C,
0x6F12,	0x0101,
0x602A,	0x2A18,
0x6F12,	0x1001,
0x602A,	0x5090,
0x6F12,	0x0E00,
0x602A,	0x84F8,
0x6F12,	0x0002,
0x602A,	0x8508,
0x6F12,	0x0200,
0x6F12,	0x0000,
0x602A,	0x5530,
0x6F12,	0x0100,
0x602A,	0x43B0,
0x6F12,	0x0101,
0x602A,	0x43D4,
0x6F12,	0x0000,
0x6F12,	0x0004,
0x6F12,	0x13E8,
0x602A,	0x2A0E,
0x6F12,	0x1FCD,
0x602A,	0x1346,
0x6F12,	0x0018,
0x602A,	0x2308,
0x6F12,	0x002F,
0x602A,	0x381A,
0x6F12,	0x0000,
0x6F12,	0x3FFF,
0x602A,	0x3812,
0x6F12,	0x0003,
0x602A,	0x345E,
0x6F12,	0x1F40,
0x6F12,	0x1F47,
0x602A,	0x3B1A,
0x6F12,	0x000F,
0x6F12,	0x000F,
0x6F12,	0x000A,
0x6F12,	0x000A,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x000A,
0x6F12,	0x000A,
0x6F12,	0x000F,
0x6F12,	0x000F,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x602A,	0x3A9A,
0x6F12,	0x0050,
0x6F12,	0x0050,
0x6F12,	0xFFC0,
0x6F12,	0xFFC0,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0xFFC0,
0x6F12,	0xFFC0,
0x6F12,	0x0050,
0x6F12,	0x0050,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x6F12,	0x0000,
0x602A,	0x39EE,
0x6F12,	0x0100,
0x6F12,	0x0100,
0x6F12,	0x0200,
0x6F12,	0x1020,
0x6F12,	0x1000,
0x602A,	0x2A1E,
0x6F12,	0x0108,
0x602A,	0x134C,
0x6F12,	0x0001,
0x602A,	0x30B8,
0x6F12,	0xD020,
0x602A,	0x2580,
0x6F12,	0x000E,
0x602A,	0x2584,
0x6F12,	0x0226,
0x602A,	0x8A10,
0x6F12,	0x0000,
0x602A,	0x8A6E,
0x6F12,	0x0FA0,
0x6F12,	0x0FA7,
0x602A,	0x8A20,
0x6F12,	0x0008,
0x602A,	0x8A26,
0x6F12,	0x0008,
0x602A,	0x8A1E,
0x6F12,	0x0080,
0x602A,	0x8A68,
0x6F12,	0x0200,
0x602A,	0x8A3E,
0x6F12,	0x03FF,
0x602A,	0x8A44,
0x6F12,	0x0080,
0x602A,	0x1370,
0x6F12,	0x0000,
0x602A,	0x1378,
0x6F12,	0x0003,
0x602A,	0x2134,
0x6F12,	0xFFFF,
0x602A,	0x1FC0,
0x6F12,	0x0100,
0x6F12,	0x005F,
0x602A,	0x1FC6,
0x6F12,	0x0060,
0x602A,	0x1FCC,
0x6F12,	0x1C80,
0x6F12,	0x1C82,
0x6F12,	0x4000,
0x6F12,	0xF4B2,
0x6028,	0x4000,
0xF484,	0x0080,
0xF498,	0x0343,
0xF496,	0x0200,
0xF4B2,	0x1C82,
0xF41C,	0xC116,
0xF418,	0x001F,
0xB606,	0x0300,
0xF482,	0x800D,
0x6028,	0x4000,
0x623C,	0x0080,
0x623E,	0x0000,
0x6240,	0x0000,
0xF4A6,	0x0016,
0x0344,	0x0018,
0x0346,	0x001C,
0x0348,	0x1997,
0x034A,	0x133B,
0x034C,	0x0660,
0x034E,	0x04C8,
0x0350,	0x0000,
0x0352,	0x0000,
0x0900,	0x2324,
0x0380,	0x0002,
0x0382,	0x0002,
0x0384,	0x0004,
0x0386,	0x0004,
0x0400,	0x1010,
0x0408,	0x0200,
0x040A,	0x0100,
0x040C,	0x0000,
0x0110,	0x1002,
0x0114,	0x0100,
0x0116,	0x3000,
0x0118,	0x0001,
0x011A,	0x0000,
0x011C,	0x0000,
0x0136,	0x1A00,
0x013E,	0x0000,
0x0300,	0x0007,
0x0302,	0x0001,
0x0304,	0x0003,
0x0306,	0x00A1,
0x0308,	0x000B,
0x030A,	0x0001,
0x030C,	0x0000,
0x030E,	0x0003,
0x0310,	0x00D0,
0x0312,	0x0002,
0x0340,	0x0B74,
0x0342,	0x4650,
0x0702,	0x0000,
0x0202,	0x0100,
0x0200,	0x0100,
0x022C,	0x0100,
0x0226,	0x0100,
0x021E,	0x0000,
0x080E,	0x0800,
0x0B00,	0x0080,
0x0B08,	0x0100,
0x0D00,	0x0000,	
 };	

//hw remosaic
static kal_uint16 addr_data_pair_custom5[] = {
	0x6028, 0x2000,
	0x602A, 0x2A08,
	0x6F12, 0x0001,
	0x6F12, 0xA5E0,
	0x602A, 0x21F8,
	0x6F12, 0x1358,
	0x602A, 0x30CE,
	0x6F12, 0x01E4,
	0x602A, 0x8A00,
	0x6F12, 0x0001,
	0x602A, 0x3770,
	0x6F12, 0x0100,
	0x602A, 0x220E,
	0x6F12, 0x0009,
	0x6F12, 0x0009,
	0x602A, 0x31CC,
	0x6F12, 0x0019,
	0x602A, 0x31D2,
	0x6F12, 0x0012,
	0x602A, 0x31CA,
	0x6F12, 0x000A,
	0x602A, 0x221C,
	0x6F12, 0x0100,
	0x602A, 0x2A18,
	0x6F12, 0x0801,
	0x602A, 0x5090,
	0x6F12, 0x0600,
	0x602A, 0x84F8,
	0x6F12, 0x0002,
	0x602A, 0x8508,
	0x6F12, 0x0200,
	0x6F12, 0x0000,
	0x602A, 0x5530,
	0x6F12, 0x0000,
	0x602A, 0x43B0,
	0x6F12, 0x0101,
	0x602A, 0x43D4,
	0x6F12, 0x1300,
	0x6F12, 0x0004,
	0x6F12, 0x13E8,
	0x602A, 0x2A0E,
	0x6F12, 0x1FCD,
	0x602A, 0x1346,
	0x6F12, 0x0018,
	0x602A, 0x2308,
	0x6F12, 0x005B,
	0x602A, 0x381A,
	0x6F12, 0x0000,
	0x6F12, 0x3FFF,
	0x602A, 0x3812,
	0x6F12, 0x0003,
	0x602A, 0x345E,
	0x6F12, 0x1F38,
	0x6F12, 0x1F3F,
	0x602A, 0x3B1A,
	0x6F12, 0x004F,
	0x6F12, 0x004F,
	0x6F12, 0x0050,
	0x6F12, 0x0050,
	0x6F12, 0x004F,
	0x6F12, 0x004F,
	0x6F12, 0x0050,
	0x6F12, 0x0050,
	0x6F12, 0x0050,
	0x6F12, 0x0050,
	0x6F12, 0x004F,
	0x6F12, 0x004F,
	0x6F12, 0x0050,
	0x6F12, 0x0050,
	0x6F12, 0x004F,
	0x6F12, 0x004F,
	0x602A, 0x3A9A,
	0x6F12, 0x0040,
	0x6F12, 0x0040,
	0x6F12, 0xFFE0,
	0x6F12, 0xFFE0,
	0x6F12, 0x0040,
	0x6F12, 0x0040,
	0x6F12, 0xFFE0,
	0x6F12, 0xFFE0,
	0x6F12, 0xFFE0,
	0x6F12, 0xFFE0,
	0x6F12, 0x0040,
	0x6F12, 0x0040,
	0x6F12, 0xFFE0,
	0x6F12, 0xFFE0,
	0x6F12, 0x0040,
	0x6F12, 0x0040,
	0x602A, 0x39EE,
	0x6F12, 0x0100,
	0x6F12, 0x0100,
	0x6F12, 0x0200,
	0x6F12, 0x1020,
	0x6F12, 0x1000,
	0x602A, 0x2A1E,
	0x6F12, 0x0108,
	0x602A, 0x134C,
	0x6F12, 0x0001,
	0x602A, 0x30B8,
	0x6F12, 0xD020,
	0x602A, 0x2580,
	0x6F12, 0x000E,
	0x602A, 0x2584,
	0x6F12, 0x0226,
	0x602A, 0x8A10,
	0x6F12, 0x0000,
	0x602A, 0x8A6E,
	0x6F12, 0x1F38,
	0x6F12, 0x1F3F,
	0x602A, 0x8A20,
	0x6F12, 0x0008,
	0x602A, 0x8A26,
	0x6F12, 0x0008,
	0x602A, 0x8A1E,
	0x6F12, 0x0080,
	0x602A, 0x8A68,
	0x6F12, 0x0200,
	0x602A, 0x8A3E,
	0x6F12, 0x03FF,
	0x602A, 0x8A44,
	0x6F12, 0x0080,
	0x602A, 0x1370,
	0x6F12, 0x0400,
	0x602A, 0x1378,
	0x6F12, 0x0084,
	0x602A, 0x2134,
	0x6F12, 0x00C6,
	0x602A, 0x1FC0,
	0x6F12, 0x0100,
	0x6F12, 0x005F,
	0x602A, 0x1FC6,
	0x6F12, 0x0060,
	0x602A, 0x1FCC,
	0x6F12, 0x1C80,
	0x6F12, 0x1C82,
	0x6F12, 0x4000,
	0x6F12, 0xF4B2,
	0x6028, 0x4000,
	0xF484, 0x0080,
	0xF498, 0x06A3,
	0xF496, 0x0200,
	0xF4B2, 0x1C82,
	0xF41C, 0x4916,
	0xF418, 0x001F,
	0xB606, 0x0300,
	0xF482, 0x800D,
	0x6028, 0x4000,
	0x623C, 0x8080,
	0x623E, 0x830D,
	0x6240, 0x0000,
	0xF4A6, 0x001D,
	0x0344, 0x0018,
	0x0346, 0x0018,
	0x0348, 0x1997,
	0x034A, 0x133F,
	0x034C, 0x1980,
	0x034E, 0x1320,
	0x0350, 0x0000,
	0x0352, 0x0004,
	0x0900, 0x0011,
	0x0380, 0x0001,
	0x0382, 0x0001,
	0x0384, 0x0001,
	0x0386, 0x0001,
	0x0400, 0x1010,
	0x0408, 0x0100,
	0x040A, 0x0100,
	0x040C, 0x0000,
	0x0110, 0x1002,
	0x0114, 0x0100,
	0x0116, 0x3000,
	0x0118, 0x0001,
	0x011A, 0x0000,
	0x011C, 0x0000,
	0x0136, 0x1A00,
	0x013E, 0x0000,
	0x0300, 0x0007,
	0x0302, 0x0001,
	0x0304, 0x0003,
	0x0306, 0x00A1,
	0x0308, 0x000B,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0003,
	0x0310, 0x00AF,
	0x0312, 0x0000,
	0x0340, 0x13FC,
	0x0342, 0x5080,
	0x0702, 0x0000,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x022C, 0x0100,
	0x0226, 0x0100,
	0x021E, 0x0000,
	0x080E, 0x0000,
	0x0B00, 0x0080,
	0x0B08, 0x0101,
	0x0D00, 0x0100,

};

static void sensor_init(void)
{
	/*Global setting */
	write_cmos_sensor_16_16(0x6028, 0x4000);
	write_cmos_sensor_16_16(0x0000, 0x0007);
	write_cmos_sensor_16_16(0x0000, 0x0842);
	write_cmos_sensor_16_16(0x001E, 0x3000);
	write_cmos_sensor_16_16(0x6028, 0x4000);
	write_cmos_sensor_16_16(0x6010, 0x0001);
	mdelay(4);
	write_cmos_sensor_16_16(0x6214, 0xFF7D);
	write_cmos_sensor_16_16(0x6218, 0x0000);
	mdelay(10);

	table_write_cmos_sensor_consecutive(addr_data_pair_init,
			   sizeof(addr_data_pair_init) / sizeof(kal_uint16));
}	/*	sensor_init  */


static void preview_setting(void)
{
	PK_DBG("4:3 binning size start\n");
	table_write_cmos_sensor(addr_data_pair_preview,
		   sizeof(addr_data_pair_preview) / sizeof(kal_uint16));
	PK_DBG("4:3 binning size end\n");
}	/*	preview_setting  */

/* Pll Setting - VCO = 280Mhz*/

static void capture_setting(kal_uint16 currefps)
{
	PK_DBG("full size start\n");
	table_write_cmos_sensor(addr_data_pair_capture,
		   sizeof(addr_data_pair_capture) / sizeof(kal_uint16));
	PK_DBG("full size  end\n");
}
static void normal_video_setting(void)
{
	PK_DBG("16:9 binning size start\n");
	table_write_cmos_sensor(addr_data_pair_normal_video,
		   sizeof(addr_data_pair_normal_video) / sizeof(kal_uint16));
	PK_DBG("16:9 binning size end\n");
}
static void hs_video_setting(void)
{
	table_write_cmos_sensor(addr_data_pair_hs_video,
		   sizeof(addr_data_pair_hs_video) / sizeof(kal_uint16));
}

static void slim_video_setting(void)
{
	table_write_cmos_sensor(addr_data_pair_slim_video,
		   sizeof(addr_data_pair_slim_video) / sizeof(kal_uint16));
}

static void custom1_setting(void)
{
	PK_DBG("4:3 HDR binning size start\n");
	table_write_cmos_sensor(addr_data_pair_custom1,
		   sizeof(addr_data_pair_custom1) / sizeof(kal_uint16));
	PK_DBG("4:3 HDR binning size end\n");
}

static void custom2_setting(void)
{
	PK_DBG("16:9 HDR binning size start\n");
	table_write_cmos_sensor(addr_data_pair_custom2,
		   sizeof(addr_data_pair_custom2) / sizeof(kal_uint16));
	PK_DBG("16:9 HDR binning size end\n");
}

static void custom3_setting(void)
{
	PK_DBG("4:3 1632X1224 size start\n");
	table_write_cmos_sensor(addr_data_pair_custom3,
		   sizeof(addr_data_pair_custom3) / sizeof(kal_uint16));
	PK_DBG("4:3 1632X1224 size end\n");
}

static void custom4_setting(void)
{
	PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_custom4,
		   sizeof(addr_data_pair_custom4) / sizeof(kal_uint16));
	PK_DBG("end \n");
}

static void custom5_setting(void)
{
	PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_custom5,
		   sizeof(addr_data_pair_custom5) / sizeof(kal_uint16));
	PK_DBG("end \n");
}
/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	*sensorID : return the sensor ID
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address*/
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = read_cmos_sensor_16_16(0x0000) + 1;

			PK_DBG("read out sensor id 0x%x \n",*sensor_id);
			if ( *sensor_id == imgsensor_info.sensor_id) {
				PK_DBG("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
				/*vivo lxd add for CameraEM otp errorcode*/
				pr_debug("start read eeprom ---vivo_otp_read_when_power_on = %d\n", vivo_otp_read_when_power_on);
				vivo_otp_read_when_power_on = SUB_GD2_otp_read();
				pr_debug("read eeprom ---vivo_otp_read_when_power_on = %d,MAIN_F8D1_OTP_ERROR_CODE=%d\n", vivo_otp_read_when_power_on, S5KGD2_OTP_ERROR_CODE);
				/*vivo lxd add end*/
				return ERROR_NONE;
			}
			PK_DBG("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id);
			retry--;
		} while(retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id !=  imgsensor_info.sensor_id) {
		/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF*/
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
 * FUNCTION
 *	open
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0;

	PK_DBG("%s", __func__);

	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address*/
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
	    		sensor_id = read_cmos_sensor_16_16(0x0000) + 1;

			if (sensor_id == imgsensor_info.sensor_id) {
				PK_DBG("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
				break;
			}
			PK_DBG("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	sensor_init();

	sensor_temperature[1] = 0;
	PK_DBG("sensor_temperature[1] = %d\n", sensor_temperature[1]);

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x3D0;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_mode = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}				/*      open  */



/*************************************************************************
 * FUNCTION
 *	close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 close(void)
{
	PK_DBG("E\n");
	sensor_temperature[1] = 0;
	PK_DBG("sensor_temperature[1] = %d\n", sensor_temperature[1]);

	/*No Need to implement this function*/

	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *	This function start the sensor preview.
 *
 * PARAMETERS
 *	*image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	PK_DBG("E imgsensor.hdr_mode=%d\n", imgsensor.ihdr_mode);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting();  /* 4:3 normal binning size */
	
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}				/*      preview   */

/*************************************************************************
 * FUNCTION
 *	capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	
	PK_DBG("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
	PK_DBG("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
				imgsensor.current_fps, imgsensor_info.cap.max_framerate/10);
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps);  /*hw-remosiac*/
		
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	PK_DBG("%s E\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	if (imgsensor.current_fps != imgsensor_info.normal_video.max_framerate)
	PK_DBG("Warning: current_fps %d fps is not support, so use normal_video's setting: %d fps!\n",
				imgsensor.current_fps, imgsensor_info.normal_video.max_framerate/10);
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	normal_video_setting();  /* 16:9 normal binning size */


	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}				/*      normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	PK_DBG("%s E\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}				/*      hs_video   */

static kal_uint32 slim_video(
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	PK_DBG("%s E\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}				/*      slim_video       */

static kal_uint32 Custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	PK_DBG("E imgsensor.ihdr_mode=%d\n", imgsensor.ihdr_mode);


	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	custom1_setting();  /* 4:3 normal binning size */


	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	custom1	 */

static kal_uint32 Custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	PK_DBG("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	imgsensor.pclk = imgsensor_info.custom2.pclk;
	/*imgsensor.video_mode = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.custom2.linelength;
	imgsensor.frame_length = imgsensor_info.custom2.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom2_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	custom2	 */

static kal_uint32 Custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,      
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)                       
{                                                                                
	PK_DBG("E\n");                                                                
                                                                                 
	spin_lock(&imgsensor_drv_lock);                                                
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;                                
	imgsensor.pclk = imgsensor_info.custom3.pclk;                                  
	imgsensor.line_length = imgsensor_info.custom3.linelength;                     
	imgsensor.frame_length = imgsensor_info.custom3.framelength;                   
	imgsensor.min_frame_length = imgsensor_info.custom3.framelength;               
	imgsensor.autoflicker_en = KAL_FALSE;                                          
	spin_unlock(&imgsensor_drv_lock);                                              
                                                                                 
	custom3_setting();                                    
	set_mirror_flip(imgsensor.mirror);                                             
                                                                                 
	return ERROR_NONE;                                                             
}	/*	custom3   */                                                               

 
 static kal_uint32 Custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,	  
					   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)						
 {																				  
	 PK_DBG("E\n"); 															   
																				  
	 spin_lock(&imgsensor_drv_lock);												
	 imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;								
	 imgsensor.pclk = imgsensor_info.custom4.pclk;									
	 imgsensor.line_length = imgsensor_info.custom4.linelength; 					
	 imgsensor.frame_length = imgsensor_info.custom4.framelength;					
	 imgsensor.min_frame_length = imgsensor_info.custom4.framelength;				
	 imgsensor.autoflicker_en = KAL_FALSE;											
	 spin_unlock(&imgsensor_drv_lock);												
																				  
	 custom4_setting();									
	 set_mirror_flip(imgsensor.mirror); 											
																				  
	 return ERROR_NONE; 															
 }	 /*  custom4   */

static kal_uint32 Custom5(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	PK_DBG("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
	imgsensor.pclk = imgsensor_info.custom5.pclk;
	imgsensor.line_length = imgsensor_info.custom5.linelength;
	imgsensor.frame_length = imgsensor_info.custom5.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	custom5_setting();									
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	custom5   */

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	PK_DBG("%s E\n", __func__);
	
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;

	sensor_resolution->SensorHighSpeedVideoWidth = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight = imgsensor_info.slim_video.grabwindow_height;

	sensor_resolution->SensorCustom1Width = imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height = imgsensor_info.custom1.grabwindow_height;

	sensor_resolution->SensorCustom2Width = imgsensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height = imgsensor_info.custom2.grabwindow_height;
	
	sensor_resolution->SensorCustom3Width = imgsensor_info.custom3.grabwindow_width;
	sensor_resolution->SensorCustom3Height = imgsensor_info.custom3.grabwindow_height;
	
	sensor_resolution->SensorCustom4Width = imgsensor_info.custom4.grabwindow_width;
	sensor_resolution->SensorCustom4Height =imgsensor_info.custom4.grabwindow_height;

	sensor_resolution->SensorCustom5Width = imgsensor_info.custom5.grabwindow_width;
	sensor_resolution->SensorCustom5Height =imgsensor_info.custom5.grabwindow_height;

	return ERROR_NONE;
}				/*      get_resolution  */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	/*PK_DBG("get_info -> scenario_id = %d\n", scenario_id);*/

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;

	/* not use */
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;

	/* inverse with datasheet */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;	/* not use */
	sensor_info->SensorResetActiveHigh = FALSE;	/* not use */
	sensor_info->SensorResetDelayCount = 5;	/* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;

	sensor_info->SensorOutputDataFormat =
		imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;

	sensor_info->HighSpeedVideoDelayFrame =
		imgsensor_info.hs_video_delay_frame;

	sensor_info->SlimVideoDelayFrame =
		imgsensor_info.slim_video_delay_frame;
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
	sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;
	sensor_info->Custom4DelayFrame = imgsensor_info.custom4_delay_frame;
	sensor_info->Custom5DelayFrame = imgsensor_info.custom5_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0;	/* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	/* The frame of setting shutter default 0 for TG int */
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;

	/* The frame of setting sensor gain*/
	sensor_info->AESensorGainDelayFrame =
				imgsensor_info.ae_sensor_gain_delay_frame;

	sensor_info->AEISPGainDelayFrame =
				imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->FrameTimeDelayFrame =
		imgsensor_info.frame_time_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	/* change pdaf support mode to pdaf VC mode */
	/*0: NO PDAF, 1: PDAF Raw Data mode, 2:PDAF VC mode */
	sensor_info->PDAF_Support = 0;
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3;	/* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2;	/* not use */
	sensor_info->SensorPixelClockCount = 3;	/* not use */
	sensor_info->SensorDataLatchCount = 2;	/* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

		sensor_info->SensorGrabStartX =
			imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.normal_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX =
			imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.slim_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

			break;
    case MSDK_SCENARIO_ID_CUSTOM2:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom2.mipi_data_lp2hs_settle_dc;

			break;
	case MSDK_SCENARIO_ID_CUSTOM3:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom3.mipi_data_lp2hs_settle_dc;

			break;
	case MSDK_SCENARIO_ID_CUSTOM4:
			sensor_info->SensorGrabStartX = imgsensor_info.custom4.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.custom4.starty;
		
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = 
				imgsensor_info.custom4.mipi_data_lp2hs_settle_dc; 
		
			break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		sensor_info->SensorGrabStartX = imgsensor_info.custom5.startx; 
		sensor_info->SensorGrabStartY = imgsensor_info.custom5.starty;
	
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = 
			imgsensor_info.custom5.mipi_data_lp2hs_settle_dc; 
	
		break;

	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}				/*      get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			  MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		Custom1(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		Custom2(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		Custom3(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		Custom4(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		Custom5(image_window, sensor_config_data);
		break;
	default:
		PK_DBG("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}				/* control() */

static kal_uint32 set_video_mode(UINT16 framerate)
{
	/* //PK_DBG("framerate = %d\n ", framerate); */
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		/* Dynamic frame rate */
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(
	kal_bool enable, UINT16 framerate)
{
	PK_DBG("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable)		/* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	else			/* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(
	enum MSDK_SCENARIO_ID_ENUM scenario_id,	MUINT32 framerate)
{
	kal_uint32 frame_length;

	PK_DBG("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		if (imgsensor.ihdr_mode) {
			frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ?
						(frame_length - imgsensor_info.custom1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		} else {
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
						(frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		if (imgsensor.ihdr_mode) {
			frame_length = imgsensor_info.custom2.pclk / framerate * 10 / imgsensor_info.custom2.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ?
						(frame_length - imgsensor_info.custom2.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		} else {
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10
						/ imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ?
						(frame_length - imgsensor_info.normal_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		PK_DBG("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
					framerate, imgsensor_info.cap.max_framerate/10);
		frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ?
						(frame_length - imgsensor_info.cap.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
	    set_dummy();
	    break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ?
						(frame_length - imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ?
						(frame_length - imgsensor_info.slim_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM1: /*same as preview */
		if (imgsensor.ihdr_mode) {
			frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ?
						(frame_length - imgsensor_info.custom1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		} else {
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
						(frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk / framerate * 10 / imgsensor_info.custom2.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ?
					(frame_length - imgsensor_info.custom2.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		frame_length = imgsensor_info.custom3.pclk / framerate * 10 / imgsensor_info.custom3.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom3.framelength) ?
					(frame_length - imgsensor_info.custom3.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom3.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;

	case MSDK_SCENARIO_ID_CUSTOM4:
		frame_length = imgsensor_info.custom4.pclk / framerate * 10 / imgsensor_info.custom4.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom4.framelength) ? (frame_length - imgsensor_info.custom4.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom4.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
		set_dummy();
		break;
	
	case MSDK_SCENARIO_ID_CUSTOM5:
		frame_length = imgsensor_info.custom5.pclk / framerate * 10 / imgsensor_info.custom5.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom5.framelength) ? (frame_length - imgsensor_info.custom5.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom5.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
		set_dummy();
		break;
	/* coding with  preview scenario by default */
	default:  /*coding with  preview scenario by default*/
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
					(frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		PK_DBG("error scenario_id = %d, we use preview scenario\n", scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(
	enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	/*PK_DBG("scenario_id = %d\n", scenario_id);*/

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		*framerate = imgsensor_info.custom1.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		*framerate = imgsensor_info.custom2.max_framerate;	
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		*framerate = imgsensor_info.custom3.max_framerate;	
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
	    *framerate = imgsensor_info.custom4.max_framerate;
	    break;
	case MSDK_SCENARIO_ID_CUSTOM5:
	    *framerate = imgsensor_info.custom5.max_framerate;
	    break;
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	PK_DBG("enable: %d\n", enable);

	if (enable) {
		/* 0x5E00[8]: 1 enable,  0 disable*/
		/* 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK*/
		write_cmos_sensor_16_16(0x0600, 0x0001);
	} else {
		/* 0x5E00[8]: 1 enable,  0 disable*/
		/* 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK*/
		write_cmos_sensor_16_16(0x0600,0x0000);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static void hdr_write_tri_shutter(kal_uint16 le, kal_uint16 me, kal_uint16 se)
{
	kal_uint16 realtime_fps = 0;

	PK_DBG("E! le:0x%x, me:0x%x, se:0x%x\n", le, me, se);
	spin_lock(&imgsensor_drv_lock);
	if (le > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = le + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	if (le < imgsensor_info.min_shutter)
		le = imgsensor_info.min_shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else
			write_cmos_sensor_16_16(0x0340, imgsensor.frame_length & 0xFFFF);
	} else {
		write_cmos_sensor_16_16(0x0340, imgsensor.frame_length & 0xFFFF);
	}

	hdr_le = le;
	hdr_me = me;
	hdr_se = se;
	///* Long exposure */
	//write_cmos_sensor_16_16(0x0226, le);
	///* Middle exposure */
	//write_cmos_sensor_16_16(0x022c, me);
	///* Short exposure */
	//write_cmos_sensor_16_16(0x0202, se);

	PK_DBG("imgsensor.frame_length:0x%x\n", imgsensor.frame_length);
	PK_DBG("L! le:0x%x, me:0x%x, se:0x%x\n", le, me, se);

}

static kal_uint16 gain2reg_dig(kal_uint16 gain, kal_uint16 g_gain)
{
	kal_uint16 dig_gain = 0x0;

	/* gain's base is 64, dig_gain's base is 256 */
	dig_gain = ((kal_uint32)gain*256/g_gain)&0xffff;
	PK_DBG("gain:%d,g_gain:%d,dig_gain=%d\n", gain, g_gain, dig_gain);
	return (kal_uint16)dig_gain;
}

static void hdr_write_tri_gain(kal_uint16 lg, kal_uint16 mg, kal_uint16 sg)
{
	kal_uint16 reg_lg_dig, reg_mg_dig, reg_sg_dig;
	kal_uint16 global_gain, reg_global_gain;

	/* should use analog(global) gain first */
	/* nosie would be obviously if totally use digital gain */
	global_gain = lg < mg?lg:mg;
	global_gain = global_gain < sg?global_gain:sg;

	if (global_gain < BASEGAIN || global_gain > 16 * BASEGAIN) {
		PK_DBG("Error gain setting");

		if (global_gain < BASEGAIN)
			global_gain = BASEGAIN;
		else if (global_gain > 16 * BASEGAIN)
			global_gain = 16 * BASEGAIN;
	}
	write_cmos_sensor_16_8(0x0104, 0x01);
	reg_global_gain = gain2reg(global_gain);
	reg_lg_dig = gain2reg_dig(lg, global_gain);
	reg_mg_dig = gain2reg_dig(mg, global_gain);
	reg_sg_dig = gain2reg_dig(sg, global_gain);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_global_gain;
	spin_unlock(&imgsensor_drv_lock);
	
	/* Long expo Gian- digital gain, step=256 */
	write_cmos_sensor_16_16(0x0230, reg_lg_dig);
	/* Medium expo Gian */
	write_cmos_sensor_16_16(0x0238, reg_mg_dig);
	/* Short expo Gian */
	write_cmos_sensor_16_16(0x020E, reg_sg_dig);
	/* global gain - step=32 */
	write_cmos_sensor_16_16(0x0204, reg_global_gain);
	
	write_cmos_sensor_16_16(0x0226, hdr_le);
	write_cmos_sensor_16_16(0x022c, hdr_me);
	write_cmos_sensor_16_16(0x0202, hdr_se);
	write_cmos_sensor_16_8(0x0104, 0x00);
	
	PK_DBG("lg:0x%x, reg_lg_dig:0x%x, mg:0x%x, reg_mg_dig:0x%x, sg:0x%x, reg_sg_dig:0x%x\n",
			lg, reg_lg_dig, mg, reg_mg_dig, sg, reg_sg_dig);
	PK_DBG("reg_global_gain:0x%x\n", reg_global_gain);

}


static kal_uint32 set_awb_gain(struct SET_SENSOR_AWB_GAIN *pSetSensorAWB)
{
	UINT32 rgain_32, grgain_32, gbgain_32, bgain_32, ggain_32;
	UINT32 BASE_HDR = 2184;
	UINT32 BASE_WB = 256;
	UINT16 reg_rgain, reg_ggain, reg_bgain;

	grgain_32 = pSetSensorAWB->ABS_GAIN_GR;
	rgain_32  = pSetSensorAWB->ABS_GAIN_R;
	bgain_32  = pSetSensorAWB->ABS_GAIN_B;
	gbgain_32 = pSetSensorAWB->ABS_GAIN_GB;
	ggain_32  = (grgain_32 + gbgain_32) >> 1; /*Gr_gain = Gb_gain */
	PK_DBG("[set_awb_gain] rgain:%d, ggain:%d, bgain:%d\n",
				rgain_32, ggain_32, bgain_32);

	/* set WB gain when HDR/remosaic*/
	reg_rgain = (rgain_32*BASE_WB/512)&0xffff;
	reg_ggain = (ggain_32*BASE_WB/512)&0xffff;
	reg_bgain = (bgain_32*BASE_WB/512)&0xffff;
	PK_DBG("[BASE_WB=256] reg_rgain:%d, reg_ggain:%d, reg_bgain:%d\n",
					reg_rgain, reg_ggain, reg_bgain);

	write_cmos_sensor_16_16(0x0D82, reg_rgain);
	write_cmos_sensor_16_16(0x0D84, reg_ggain);
	write_cmos_sensor_16_16(0x0D86, reg_bgain);

	/*set weight gain when HDR*/
	if (imgsensor.sensor_mode != IMGSENSOR_MODE_CAPTURE) {
		reg_rgain = (rgain_32*BASE_HDR*5/16/512)&0x1fff;  /*max value is 8192 */
		reg_ggain = (ggain_32*BASE_HDR*9/16/512)&0x1fff;
		reg_bgain = (bgain_32*BASE_HDR*2/16/512)&0x1fff;
		PK_DBG("[BASE_HDR=2184] reg_rgain:%d, reg_ggain:%d, reg_bgain:%d\n",
					reg_rgain, reg_ggain, reg_bgain);

		write_cmos_sensor_16_16(0x6028, 0x2000);
		write_cmos_sensor_16_16(0x602A, 0x4B9C);
		write_cmos_sensor_16_16(0x6F12, reg_rgain);
		write_cmos_sensor_16_16(0x6F12, reg_ggain);
		write_cmos_sensor_16_16(0x6F12, reg_bgain); /* short expo's gain */
		write_cmos_sensor_16_16(0x602A, 0x4BAA);
		write_cmos_sensor_16_16(0x6F12, reg_rgain);
		write_cmos_sensor_16_16(0x6F12, reg_ggain);
		write_cmos_sensor_16_16(0x6F12, reg_bgain); /* long expo's gain */
		write_cmos_sensor_16_16(0x602A, 0x4BB8);
		write_cmos_sensor_16_16(0x6F12, reg_rgain);
		write_cmos_sensor_16_16(0x6F12, reg_ggain);
		write_cmos_sensor_16_16(0x6F12, reg_bgain); /* medium expo's gain */
		write_cmos_sensor_16_16(0x602A, 0x4BC6);
		write_cmos_sensor_16_16(0x6F12, reg_rgain);
		write_cmos_sensor_16_16(0x6F12, reg_ggain);
		write_cmos_sensor_16_16(0x6F12, reg_bgain); /* mixed expo's gain */
	}

	return ERROR_NONE;
}
#if 1
static kal_uint32 get_sensor_temperature(void)
{
	kal_uint32	TMC_000A_Value=0,TMC_Value =0; 
	
	//write_cmos_sensor_16_16(0x6028, 0x4000);
	TMC_000A_Value=read_cmos_sensor_16_16(0x0020);
	PK_DBG("GD2 TMC 0X000A Value=0x%x\n", TMC_000A_Value);

	TMC_Value=(TMC_000A_Value>>8)&0xFF;
	
	TMC_Value = (TMC_Value > 100) ? 100: TMC_Value;

	sensor_temperature[1] = TMC_Value;
	PK_DBG("GD2 TMC 0X000A temperature Value=0x%x, sensor_temperature[1] =%d\n", TMC_Value, sensor_temperature[1]);
	
	return TMC_Value;
}
#endif
static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				  UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *) feature_para;
	/*INT32 *feature_return_para_i32 = (INT32 *) feature_para;*/
	struct SET_SENSOR_AWB_GAIN *pSetSensorAWB = (struct SET_SENSOR_AWB_GAIN *) feature_para;

	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
		struct SENSOR_RAWINFO_STRUCT *rawinfo;
	struct SENSOR_VC_INFO_STRUCT *pvcinfo;


	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data= (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	/*PK_DBG("feature_id = %d\n", feature_id);*/
	switch (feature_id) {
	case SENSOR_FEATURE_GET_AWB_REQ_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CUSTOM3:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CUSTOM5:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1;
			break;
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_gain;
		*(feature_data + 2) = imgsensor_info.max_gain;
		break;
	case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
		*(feature_data + 0) = imgsensor_info.min_gain_iso;
		*(feature_data + 1) = imgsensor_info.gain_step;
		*(feature_data + 2) = imgsensor_info.gain_type;
		break;
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
		switch (*feature_data) {
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(feature_data + 1) = 5;  /////  imgsensor_info.min_shutter
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(feature_data + 1) = 5;  /////  imgsensor_info.min_shutter
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(feature_data + 1) = 5;  /////  imgsensor_info.min_shutter
				break;

			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:

			 *(feature_data + 1) = 16;  /////  imgsensor_info.min_shutter
			 break;

			default:
				*(feature_data + 1) = 5;  /////  imgsensor_info.min_shutter
				break;
		}		
		
		break;
	case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
/*vivo xuyuanwen add for PD2133 PD2135 board version start*/
#if defined(CONFIG_MTK_CAM_PD2135)			
		ccm_board_version = get_board_version();
		PK_DBG("ccm_board_version = %s  ccm_board_version[0] = %c\n  ", ccm_board_version, ccm_board_version[0]);
		if('0' != ccm_board_version[0]){
			PK_DBG("curent board is PD2133 ccm_board_version = %c\n", ccm_board_version[0]);
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 2000000;
		}
		else{
			PK_DBG("curent board is PD2135 ccm_board_version = %c\n", ccm_board_version[0]);
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 2000000;
		}		
#else
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 8000000;
#endif
/*vivo xuyuanwen add for PD2133 PD2135 board version start*/
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.pclk;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.pclk;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.pclk;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom1.pclk;
			break;
	
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.pclk;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.cap.framelength << 16)
				+ imgsensor_info.cap.linelength;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.normal_video.framelength << 16)
				+ imgsensor_info.normal_video.linelength;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.hs_video.framelength << 16)
				+ imgsensor_info.hs_video.linelength;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.slim_video.framelength << 16)
				+ imgsensor_info.slim_video.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom1.framelength << 16)
				+ imgsensor_info.custom1.linelength;
			break;

		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
			break;
		}
		break;

	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
#if 0
		PK_DBG(
			"feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n",
			imgsensor.pclk, imgsensor.current_fps);
#endif
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
	/* night_mode((BOOL) *feature_data); no need to implement this mode */
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;

	case SENSOR_FEATURE_SET_REGISTER:
		PK_DBG("SENSOR_FEATURE_SET_REGISTER sensor_reg_data->RegAddr = 0x%x, sensor_reg_data->RegData = 0x%x\n", sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		write_cmos_sensor_16_16(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;

	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData = read_cmos_sensor_16_16(sensor_reg_data->RegAddr);
		PK_DBG("SENSOR_FEATURE_GET_REGISTER sensor_reg_data->RegAddr = 0x%x, sensor_reg_data->RegData = 0x%x\n", sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;

	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM or
		 * just return LENS_DRIVER_ID_DO_NOT_CARE
		 */
		/* if EEPROM does not exist in camera module. */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL) (*feature_data_16),
					*(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario(
	    (enum MSDK_SCENARIO_ID_ENUM) *feature_data, *(feature_data + 1));
		break;

	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario(
			(enum MSDK_SCENARIO_ID_ENUM) *(feature_data),
			  (MUINT32 *) (uintptr_t) (*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_GET_PDAF_DATA:
		PK_DBG("SENSOR_FEATURE_GET_PDAF_DATA\n");
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL) (*feature_data));
		break;

	/* for factory mode auto testing */
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		PK_DBG("current fps :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (UINT16)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_RAW_INFO:	
		pr_debug("SENSOR_FEATURE_GET_RAW_INFO scenarioId:%d\n",
			(UINT32) *feature_data);
		rawinfo = (struct SENSOR_RAWINFO_STRUCT *) (uintptr_t) (*(feature_data + 1));	
		memcpy((void *)rawinfo,
				(void *)&imgsensor_raw_info,
				sizeof(struct SENSOR_RAWINFO_STRUCT));
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		/* PK_DBG("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
		 *	(UINT32) *feature_data);
		 */

		wininfo =
	(struct SENSOR_WINSIZE_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[5],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[6],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[7],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			memcpy((void *)wininfo,	(void *)&imgsensor_winsize_info[8],	sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM5:
			memcpy((void *)wininfo,	(void *)&imgsensor_winsize_info[9],	sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
					break;
	case SENSOR_FEATURE_GET_PDAF_INFO:
		PK_DBG("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%lld\n", *feature_data);
		PDAFinfo= (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));

		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				//memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(SET_PD_BLOCK_INFO_T));
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM2:
		case MSDK_SCENARIO_ID_CUSTOM3:
		case MSDK_SCENARIO_ID_CUSTOM4:
		case MSDK_SCENARIO_ID_CUSTOM5:
			default:
				break;
		}
		break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:	
		switch (*(feature_data + 1)) {	/*2sum = 2; 4sum = 4; 4avg = 1 not 4cell sensor is 4avg*/
			
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM2:
		case MSDK_SCENARIO_ID_CUSTOM3:
		case MSDK_SCENARIO_ID_CUSTOM4:
			*feature_return_para_32 = 4;	/*BINNING_AVERAGED 2sum2avg*/
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		default:
			*feature_return_para_32 = 1; /*BINNING_NONE,*/ 
			break;
		}
		PK_DBG("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
			*feature_return_para_32);
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		PK_DBG("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%lld\n", *feature_data);
		/*PDAF capacity enable or not, 2p8 only full size support PDAF*/
		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:

				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0; /* video 16:9*/
				break;
			case MSDK_SCENARIO_ID_CUSTOM1:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0; /* video 16:9*/
				break;

			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
		}
		break;
/*
    case SENSOR_FEATURE_SET_PDAF:
        PK_DBG("PDAF mode :%d\n", *feature_data_16);
        imgsensor.pdaf_mode= *feature_data_16;
        break;
    */

	case SENSOR_FEATURE_SET_AWB_GAIN:
		set_awb_gain(pSetSensorAWB);
		break;	

    case SENSOR_FEATURE_GET_VC_INFO:
        PK_DBG("SENSOR_FEATURE_GET_VC_INFO %d\n", (UINT16)*feature_data);
        pvcinfo = (struct SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		    memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1], sizeof(struct SENSOR_VC_INFO_STRUCT));
		    break;
		case MSDK_SCENARIO_ID_CUSTOM1:
		    memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2], sizeof(struct SENSOR_VC_INFO_STRUCT));
		    break;		
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		default:
		    memcpy((void *)pvcinfo, (void *) &SENSOR_VC_INFO[0], sizeof(struct SENSOR_VC_INFO_STRUCT));
		    break;
		}
		break;  
		
	#if 1
	case SENSOR_FEATURE_GET_CUSTOM_INFO:
	    printk("SENSOR_FEATURE_GET_CUSTOM_INFO information type:%lld  MAIN_F8D1_OTP_ERROR_CODE:%d \n", *feature_data,S5KGD2_OTP_ERROR_CODE);
		switch (*feature_data) {
			case 0:    //info type: otp state
			PK_DBG("*feature_para_len = %d, sizeof(MUINT32)*13 + 2 =%ld, \n", *feature_para_len, sizeof(MUINT32)*13 + 2);
			if (*feature_para_len >= sizeof(MUINT32)*13 + 2) {
			    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = S5KGD2_OTP_ERROR_CODE;//otp_state
				memcpy( feature_data+2, sn_inf_sub_s5kgd2, sizeof(MUINT32)*13); 
				memcpy( feature_data+10, material_inf_sub_s5kgd2, sizeof(MUINT32)*4); 

				#if 0
						for (i = 0 ; i<12 ; i++ ){
						printk("sn_inf_sub_s5kgd2[%d]= 0x%x\n", i, sn_inf_sub_s5kgd2[i]);
						}
				#endif
			}
				break;
			}
			break;
	#endif

	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16) (*feature_data), (UINT16) (*(feature_data + 1)), (BOOL) (*(feature_data + 2)));
		break;

	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
		/*
		 * 1, if driver support new sw frame sync
		 * set_shutter_frame_length() support third para auto_extend_en
		 */
		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(feature_data + 1) = 1;
		    *(feature_data + 2) = 4;   ////imgsensor_info.margin
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(feature_data + 1) = 1;
		    *(feature_data + 2) = 4;   ////imgsensor_info.margin
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(feature_data + 1) = 1;
		    *(feature_data + 2) = 8;   ////imgsensor_info.margin
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(feature_data + 1) = 1;
		    *(feature_data + 2) = 8;   ////imgsensor_info.margin
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				*(feature_data + 1) = 1;
		    *(feature_data + 2) = 4;   ////imgsensor_info.margin
				break;
		  

			 
			default:
				*(feature_data + 1) = 1;
		    *(feature_data + 2) = 4;   ////imgsensor_info.margin
				break;
		}
		break;
		
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
		*feature_return_para_32 = get_sensor_temperature();
		*feature_para_len = 4;
		break;

	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		PK_DBG("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		PK_DBG("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n", *feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
	PK_DBG("SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE\n");
		memcpy(feature_return_para_32,
		&imgsensor.ae_frm_mode, sizeof(struct IMGSENSOR_AE_FRM_MODE));
		break;
	case SENSOR_FEATURE_SET_HDR_TRI_SHUTTER:
		PK_DBG("SENSOR_FEATURE_SET_HDR_TRI_SHUTTER LE=%d, SE=%d, ME=%d\n",(UINT16) *feature_data,(UINT16) *(feature_data + 1),(UINT16) *(feature_data + 2));
		hdr_write_tri_shutter((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
		break;
	case SENSOR_FEATURE_SET_HDR_TRI_GAIN:
		PK_DBG("SENSOR_FEATURE_SET_HDR_TRI_GAIN LGain=%d, SGain=%d, MGain=%d\n",(UINT16) *feature_data,(UINT16) *(feature_data + 1),(UINT16) *(feature_data + 2));
		hdr_write_tri_gain((UINT16)*feature_data, (UINT16)*(feature_data+1), (UINT16)*(feature_data+2));
		break;
 

	case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
		PK_DBG("SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE\n");
		*feature_return_para_32 =  imgsensor.current_ae_effective_frame;
		break;
	case SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY:
		/*
		  * SENSOR_VHDR_MODE_NONE  = 0x0,
		  * SENSOR_VHDR_MODE_IVHDR = 0x01,
		  * SENSOR_VHDR_MODE_MVHDR = 0x02,
		  * SENSOR_VHDR_MODE_ZVHDR = 0x09
		  * SENSOR_VHDR_MODE_4CELL_MVHDR = 0x0A
		*/
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM4:
		case MSDK_SCENARIO_ID_CUSTOM5:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x02;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
		case MSDK_SCENARIO_ID_CUSTOM3:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		default:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x0;
			break;
		}
		PK_DBG("SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY scenarioId:%llu, HDR:%llu\n", *feature_data, *(feature_data+1));
		break;
		
	case SENSOR_FEATURE_GET_PIXEL_RATE:
			switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.cap.pclk /
				(imgsensor_info.cap.linelength - 80))*
				imgsensor_info.cap.grabwindow_width;
		
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.normal_video.pclk /
				(imgsensor_info.normal_video.linelength - 80))*
				imgsensor_info.normal_video.grabwindow_width;
		
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.hs_video.pclk /
				(imgsensor_info.hs_video.linelength - 80))*
				imgsensor_info.hs_video.grabwindow_width;
		
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.slim_video.pclk /
				(imgsensor_info.slim_video.linelength - 80))*
				imgsensor_info.slim_video.grabwindow_width;
		
				break;
			
			case MSDK_SCENARIO_ID_CUSTOM1:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.custom1.pclk /
				(imgsensor_info.custom1.linelength - 80))*
				imgsensor_info.custom1.grabwindow_width;
		
				break;
			case MSDK_SCENARIO_ID_CUSTOM2:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.custom2.pclk /
				(imgsensor_info.custom2.linelength - 80))*
				imgsensor_info.custom2.grabwindow_width;
		
				break;
			case MSDK_SCENARIO_ID_CUSTOM3:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.custom3.pclk /
				(imgsensor_info.custom3.linelength - 80))*
				imgsensor_info.custom3.grabwindow_width;
				break;

			case MSDK_SCENARIO_ID_CUSTOM4:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.custom4.pclk /
				(imgsensor_info.custom4.linelength - 80))*
				imgsensor_info.custom4.grabwindow_width;
				break;

			case MSDK_SCENARIO_ID_CUSTOM5:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.custom5.pclk /
				(imgsensor_info.custom5.linelength - 80))*
				imgsensor_info.custom5.grabwindow_width;
				break;

			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				(imgsensor_info.pre.pclk /
				(imgsensor_info.pre.linelength - 80))*
				imgsensor_info.pre.grabwindow_width;
				break;
			}
			break;
		
		case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
		
			switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.cap.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
						if (imgsensor.ihdr_mode)
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.custom2.mipi_pixel_rate;
						else
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.normal_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.hs_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.slim_video.mipi_pixel_rate;
				break;
				
				case MSDK_SCENARIO_ID_CUSTOM1:
						if (imgsensor.ihdr_mode)
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.custom1.mipi_pixel_rate;
						else
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.pre.mipi_pixel_rate;
						break;
				case MSDK_SCENARIO_ID_CUSTOM2:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.custom2.mipi_pixel_rate;
						break;
				case MSDK_SCENARIO_ID_CUSTOM3:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.custom3.mipi_pixel_rate;
						break;
				case MSDK_SCENARIO_ID_CUSTOM4:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.custom4.mipi_pixel_rate;
						break;
				case MSDK_SCENARIO_ID_CUSTOM5:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.custom5.mipi_pixel_rate;
						break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.pre.mipi_pixel_rate;
				break;
			}
			break;

	default:
		break;
	}

	return ERROR_NONE;
}				/*      feature_control()  */

//#include "../imgsensor_hop.c"

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close,
	//hop,
  	//do_hop,
};

UINT32 S5KGD2_MIPI_RAW_SensorInit(
	struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}
