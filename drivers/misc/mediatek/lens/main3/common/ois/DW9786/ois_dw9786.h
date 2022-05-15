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


#ifndef OIS_DW9786_H
#define OIS_DW9786_H

#include "ois_core_tele.h"

#define DW9786_OIS_FW_NAME "dw9786FW.bin"

/* common define */
#define TIME_OVER -1
#define COMMAND_ERROR 1
#define EXECUTE_ERROR 2
#define CHECKSUM_ERROR 4
#define VERIFY_ERROR 8

#define FUNC_PASS 0
#define FUNC_FAIL -1

#define DW9786_SLAVE_ADDR           0x32
#define DW9786_MEM_ADDR             0x74
#define OFF                         0x0000
#define ON                          0x0001
#define OIS_ON                      0x0000  //servo on and ois on
#define SERVO_ON                    0x0001  //servo on and ois locked
#define XY_OIS_ON                   0x0001
#define XY_SERVO_ON                 0x0002
#define Z_SERVO_ON                  0x0001
#define OPERATION_MODE              0x0001
#define OIS_MODE                    0x0002
#define HALL_READ_DONE              0x0100
#define REG_DEAFULT_ON              0x8000
#define USER_WRITE_EN               0x56FA
#define OIS_SPI_MASTER              0x0001
#define OIS_SPI_INPUT               0x0002
#define OIS_GYRO_INITED             0x8001
#define OIS_GYRO_STOPPED            0x8002
#define ALL_RELEASE1                0x98AC
#define ALL_RELEASE2                0x70BD
#define FW_CHECKSUM_FLAG            0xCC33
#define CHIP_ID                     0x9781
#define DW9786_ERASE_PAGE           0x0008
#define DW9786_ERASE_START          0x0002
#define MTP_START_ADDR              0x8000
#define DW9786_FW_SIZE              20480   //20kb
#define DW9786_BLOCK_SIZE           512     //512bytes
#define DW9786_FW_CHECKSUM_OFFSET   20466
#define DW9786_SENCOND_CHIP_ID      0x0020
#define DW9871C_DEFAULT_SAMPLE_FREQ 0x04
#define DW9786_LENS_PACKET_SIZE     62      //byte
#define DW9786_PACKET_READ_DONE     0x0000
#define DW9786_MAX_LENS_INFO_SIZE   310      //62*5 bytes
#define DW9786_SHUTDOWN_MODE        0x0000   //shutdown mode
#define DW9786_STANDBY_MODE         0x0001   //standby mode
#define DW9786_SOFTPT_OFF           0xAC1E   //ALL_PT_OFF
#define DW9786_SOFTPT_ON            0x0000   //ALL_PT_ON
#define DW9786_IMONSEL_ON           0x0008   //IMON_MUX_SEL

#define FLASH_DATA_BIN_LEN          0xA000
#define DW9786_REF_MCS_CHECKSUM     0xF8F8F7DC
#define DW9786_PJT_VERSION          0x0202
#define DW9786_FW_VERSION           0x0402
#define DW9786_FW_DATE              0x0622

#define DW9786_PRODUCT_NUM          0x9786
/*-----------------Flash message define-----------------*/
#define DW9786_FMC_PAGE 20
#define DW9786_MCS_CODE 1
#define DW9786_MCS_LUT 2
#define DW9786_MCS_USER 3
#define DW9786_MCS_START_ADDRESS 0x6000
#define DW9786_MCS_PKT_SIZE 128
#define DW9786_MCS_SIZE_W 0x5000
#define DW9786_MCS_SIZE 0xA000
#define DW9786_MCS_CHECKSUM_SIZE 10240

/* registers define*/
#define DW9786_MEM_AREA                    0x3FFF
#define DW9786_USER_PROTECT_CTRL           0x97E6
#define DW9786_RELEASE_VER_H               0x9800
#define DW9786_RELEASE_VER_L               0x9802
#define DW9786_RELEASE_DATE                0x9804
#define DW9786_CHIP_ID                     0xB000
#define DW9786_PRODUCT_ID                  0xE018
#define DW9786_OIS_ON_OFF                  0xB010
#define DW9786_OIS_STATUS                  0xB020
#define DW9786_OIS_ACTIVE                  0xB022  // x&y axis active state
#define DW9786_AF_SERVO_ACTIVE             0xB024  // AF active state
#define DW9786_OIS_CTROL_MODE              0xB026  // control mode
#define DW9786_REG_OIS_TRIPODE_CTRL        0xB028  // trip mode
#define DW9786_GYRO_INIT_CMD               0xB02C  // gyro init
#define DW9786_GYRO_READ_CMD               0xB034  // gyro read start
#define DW9786_OIS_MODE_SELECT             0xB03A
#define DW9786_OIS_MODE_SWITCH             0xB0B0
#define DW9786_MODE_SWITCH_DELAY           0xB0B4
#define DW9786_MODE_SWITCH_STEP            0xB0B6
#define DW9786_LENS_POSX                   0xB102
#define DW9786_LENS_POSY                   0xB202
#define DW9786_AF_CODE                     0xB300
#define DW9786_REG_LENS_INFO_START         0xB350  //lens info start addr
#define DW9786_REG_INFO_SAMPLE_CTRL        0xB3A2  //[15:8] for hall and gyro data sample enable,refer to DW9786_SAMPLE_CTRL;[7:0] for sample frequecy
#define DW9786_DATA_READY                  0xB3A4
#define DW9786_REG_OIS_GYRO_OFFSETX        0xB4E4  // gyro offset
#define DW9786_REG_OIS_GYRO_OFFSETY        0xB5E4
#define DW9786_REG_OIS_GYRO_GAINX          0xB4E2
#define DW9786_REG_OIS_GYRO_GAINY          0xB5E2
#define DW9786_REG_OIS_GAIN_CHOOSE         0xB6A2
#define DW9786_GYRO_OFFSET_CAL_STATUS      0xB704
#define DW9786_CHIP_EN                     0xE000
#define DW9786_MCU_ACTIVE                  0xE004
#define DW9786_CODE_PT_RELEASE             0xE2F8
#define DW9786_SOFT_PROTECT                0xE2FC
#define DW9786_IMON_MUX_SEL                0xE164
#define DW9786_FLASH_MEM_SEL               0xED00
#define DW9786_FLASH_EARSE_ADDR            0xED08
#define DW9786_FLASH_EARSE_DATA            0xED0C
#define DW9786_FW_MEM_ADDR                 0xED28  //Memory address select
#define DW9786_FW_MEM_DATA                 0xED2C  //FW data write
#define DW9786_USER_PT_A                   0xEDB0
#define DW9786_MCS_LUT_ADDR                0xEDBC


/*struct define*/
enum DW9786_mode {
	DW9786_STILL_MODE       = 0x8000,
	DW9786_VIDEO_MODE       = 0x8001,
	DW9786_ZOOM_MODE        = 0x8002,
	DW9786_CENTERING_MODE   = 0x8003,
	DW9786_SINEWAVEY_MODE   = 0x8004,
	DW9786_SINEWAVEX_MODE   = 0x8005,
	DW9786_CIRCLEWAVE_MODE  = 0x8006,
};

enum DW9786_IMU {
	ICM20690 = 0x0000,
	LSM6DSM  = 0x0002,
	BMI260   = 0x0006,
	LSM6DSOQ = 0x0007,
};

enum DW9786_CMD_STATUS {
	OPERATE_START = 0x8000,
	OPERATE_ING   = 0x8001,
	OPERATE_DONE  = 0x8002,
};

enum DW9786_OPREATION_MODE {
	OP_FLASH_SAVE      = 0x00AA,
	OP_CAL_CHECKSUM    = 0x1000,
	OP_FW_CHECKSUM     = 0x2000,
	OP_OIS_MODE        = 0x8000,  //default
	OP_HALL_CAL        = 0x4012,
	OP_LOOP_GAIN_CAL   = 0x4013,
	OP_LENS_OFFSET_CAL = 0x4014,
	OP_GYRO_OFFSET_CAL = 0x4015,
};

enum DW9786_FW_TYPE {
	FW_MODULE = 0x8000,
	FW_VIVO   = 0x8001
};

enum DW9786_FLASH_SECTOR {
	SECTOR_0    = 0x0000,
	SECTOR_1    = 0x0008,
	SECTOR_2    = 0x0010,
	SECTOR_3    = 0x0018,
	SECTOR_4    = 0x0020,
	SECTOR_PAGE = 0x0027
};

enum DW9786_SAMPLE_CTRL {//8bit
	DISABLE_All = 0x00,
	ENABLE_HALL = 0x01,
	ENABLE_GYRO = 0x10,
	ENABLE_ALL  = 0x11
};

struct DW9786_LENS_INFO {
	u16 timeStamp;
	u16 hallX;
	u16 hallY;
};

#endif /* OIS_MAIN_H */
