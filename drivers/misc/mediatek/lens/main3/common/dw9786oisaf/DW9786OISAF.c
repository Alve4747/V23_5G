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

/*
 * DW9786OISAF voice coil motor driver
 *
 *
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>

#include "lens_info.h"

#define AF_DRVNAME "DW9786OISAF_DRV"
#define AF_I2C_SLAVE_ADDR 0x32
#define _SLV_OIS_ 0x32

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...)                                               \
	pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

static struct i2c_client *g_pstAF_I2Cclient;
static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;


static unsigned long g_u4AF_INF;
static unsigned long g_u4AF_MACRO = 1023;
static unsigned long g_u4TargetPosition;
static unsigned long g_u4CurrPosition;

static int s4AF_ReadReg(u16 a_uAddr, u16 *a_pu2Result)
{
	int i4RetValue = 0;
	char pBuff, temp;
	char puSendCmd[2] = {(char)(a_uAddr >> 8), (char)(a_uAddr &0xFF)};

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 2);

	if (i4RetValue < 0) {
		LOG_INF("I2C read - send failed!!\n");
		return -1;
	}

	i4RetValue = i2c_master_recv(g_pstAF_I2Cclient, &pBuff, 1);

	if (i4RetValue < 0) {
		LOG_INF("I2C read - recv failed!!\n");
		return -1;
	}
	
	temp = pBuff  >> 8;
	pBuff = pBuff <<8;
	pBuff = pBuff | temp;
	
	*a_pu2Result = pBuff;

	return 0;
}

static int s4AF_WriteReg(u16 a_u2Addr, u16 a_u2Data)
{
	int i4RetValue = 0;

	char puSendCmd[4] = {(char)(a_u2Addr >> 8), (char)(a_u2Addr &0xFF), (char)(a_u2Data >> 8), (char)(a_u2Data &0xFF)};

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 4);

	if (i4RetValue < 0) {
		LOG_INF("I2C write failed!!\n");
		return -1;
	}

	return 0;
}

/* DW9786OISAF OIS CODE START*/
int s4AF_WriteReg_DW9786OISAF(unsigned short i2c_id, unsigned char *a_pSendData,
			    unsigned short a_sizeSendData)
{
	int i4RetValue = 0;

	spin_lock(g_pAF_SpinLock);
	g_pstAF_I2Cclient->addr = i2c_id >> 1;
	spin_unlock(g_pAF_SpinLock);

	i4RetValue =
		i2c_master_send(g_pstAF_I2Cclient, a_pSendData, a_sizeSendData);

	if (i4RetValue != a_sizeSendData) {
		LOG_INF("I2C send failed!!, Addr = 0x%x, Data = 0x%x\n",
			a_pSendData[0], a_pSendData[1]);
		return -1;
	}

	return 0;
}

int s4AF_ReadReg_DW9786OISAF(unsigned short i2c_id, unsigned char *a_pSendData,
			   unsigned short a_sizeSendData,
			   unsigned char *a_pRecvData,
			   unsigned short a_sizeRecvData)
{
	int i4RetValue;
	struct i2c_msg msg[2];

	spin_lock(g_pAF_SpinLock);
	g_pstAF_I2Cclient->addr = i2c_id >> 1;
	spin_unlock(g_pAF_SpinLock);

	msg[0].addr = g_pstAF_I2Cclient->addr;
	msg[0].flags = 0;
	msg[0].len = a_sizeSendData;
	msg[0].buf = a_pSendData;

	msg[1].addr = g_pstAF_I2Cclient->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = a_sizeRecvData;
	msg[1].buf = a_pRecvData;

	i4RetValue =
		i2c_transfer(g_pstAF_I2Cclient->adapter, msg, ARRAY_SIZE(msg));

	if (i4RetValue != 2) {
		LOG_INF("I2C Read failed!!\n");
		return -1;
	}
	return 0;
}


void DW9786OISAF_WR_I2C(unsigned char slvadr, unsigned char size, unsigned char *dat)
{
	s4AF_WriteReg_DW9786OISAF(slvadr, dat, size);
}

unsigned short int DW9786OISAF_RD_I2C(unsigned char slvadr, unsigned char size,unsigned char *dat)
{
	unsigned short int read_data = 0;
	unsigned short int read_data_h = 0;

	s4AF_ReadReg_DW9786OISAF(slvadr, dat, 2,(unsigned char *)&read_data, 2);

	read_data_h = read_data >> 8;
	read_data = read_data << 8;
	read_data = read_data | read_data_h;

	return read_data;
}

void DW9786OISAF_I2C_OIS_per_write(unsigned short int u16_adr, unsigned short int u16_dat)
{
	unsigned char out[4];

	out[0] = (char)(u16_adr >> 8);
	out[1] = (char)(u16_adr & 0xFF);
	out[2] = (char)(u16_dat >> 8);
	out[3] = (char)(u16_dat & 0xFF);

	DW9786OISAF_WR_I2C(_SLV_OIS_, 4, out);

}

unsigned short int DW9786OISAF_I2C_OIS_per_read(unsigned short int u16_adr)
{

	unsigned char u08_dat[2];

	u08_dat[0] = (char)(u16_adr >> 8);; /*  */
	u08_dat[1] = (char)(u16_adr & 0xFF);	/* target address */
	
	return DW9786OISAF_RD_I2C(_SLV_OIS_, 2, u08_dat);
}


void DW9786OISAF_setOISMode(int Disable)
{

	LOG_INF("PosX = 0x%x \n",DW9786OISAF_I2C_OIS_per_read(0xB102));
	LOG_INF("PosY = 0x%x \n",DW9786OISAF_I2C_OIS_per_read(0xB202));
	
	
	LOG_INF("Gyro X = 0x%x \n",DW9786OISAF_I2C_OIS_per_read(0xB1A0));
	LOG_INF("Gyro Y = 0x%x \n",DW9786OISAF_I2C_OIS_per_read(0xB2A0));
	
	
	LOG_INF("VH = 0x%x \n",DW9786OISAF_I2C_OIS_per_read(0x9800));//V
	LOG_INF("VL = 0x%x \n",DW9786OISAF_I2C_OIS_per_read(0x9802)); //v
	LOG_INF("DATE = 0x%x \n",DW9786OISAF_I2C_OIS_per_read(0x9804)); //d

	
}


void DW9786OISAF_OIS_Standby(void)
{
	/* Gyro Off*/
	mdelay(10);
	DW9786OISAF_I2C_OIS_per_write(0xB02C, 0x0002); /* Stop Gyro*/
	mdelay(10);
	DW9786OISAF_I2C_OIS_per_write(0xB034, 0x0000); /* Stop*/ 	
}

/* MAIN OIS */
void DW9786OISAF_Main_OIS(void)
{
	LOG_INF("ois reset start\n");
#if 0
	mdelay(10);
	DW9786OISAF_I2C_OIS_per_write(0XE000, 0x0000); /*reset*/
	mdelay(2);
	DW9786OISAF_I2C_OIS_per_write(0XE000, 0x0001);/*Active mode(DSP ON)*/
	mdelay(5);/**/
	DW9786OISAF_I2C_OIS_per_write(0xE2FC, 0xAC1E);
	DW9786OISAF_I2C_OIS_per_write(0xE164, 0x0008);
	DW9786OISAF_I2C_OIS_per_write(0xE2FC, 0x0000);
	mdelay(1);/**/
	DW9786OISAF_I2C_OIS_per_write(0xE004, 0x0001);
	mdelay(50);
	/*User protection release*/
	LOG_INF("ois reset end, chip id = 0x%x\n",DW9786OISAF_I2C_OIS_per_read(0xB000)); //0X9786


	DW9786OISAF_I2C_OIS_per_write(0xB026, 0x0001); /* OIS ON */
	DW9786OISAF_I2C_OIS_per_write(0xB022, 0x0001); /* OIS ON */
	DW9786OISAF_I2C_OIS_per_write(0xB024, 0x0001); /* OIS ON */
	LOG_INF("OIS ON STATUS = 0x%x\n",DW9786OISAF_I2C_OIS_per_read(0xB020)); //0x1012

	LOG_INF("gyroInitOn start\n");
	mdelay(10);
	DW9786OISAF_I2C_OIS_per_write(0xB02C, 0x0001); /* SPI master*/
	mdelay(90);
	DW9786OISAF_I2C_OIS_per_write(0xB034, 0x0001); /* tart To Read Gyro*/
	LOG_INF("gyroInitOn end\n");
	mdelay(1);/**/

	DW9786OISAF_I2C_OIS_per_write(0xB3A2, 0x1104);//write sampling time to 4ms

	DW9786OISAF_I2C_OIS_per_write(0xB6A2, 0x0000);//choose main gain

	DW9786OISAF_I2C_OIS_per_write(0xB028, 0x8000);//trip mode on
#endif
}

static inline int setAFPara(__user struct stAF_MotorCmd *pstMotorCmd)
{
	struct stAF_MotorCmd stMotorCmd;

	if (copy_from_user(&stMotorCmd, pstMotorCmd, sizeof(stMotorCmd)))
		LOG_INF("copy to user failed when getting motor command\n");

	LOG_INF("Motor CmdID : %x\n", stMotorCmd.u4CmdID);

	LOG_INF("Motor Param : %x\n", stMotorCmd.u4Param);

	switch (stMotorCmd.u4CmdID) {
	case 1:
		DW9786OISAF_setOISMode(stMotorCmd.u4Param); /* 1 : disable */
		break;
	case 2:

		break;
	}

	return 0;
}

/* DW9786OISAF OIS CODE END*/


static inline int getAFInfo(__user struct stAF_MotorInfo *pstMotorInfo)
{
	struct stAF_MotorInfo stMotorInfo;

	stMotorInfo.u4MacroPosition = g_u4AF_MACRO;
	stMotorInfo.u4InfPosition = g_u4AF_INF;
	stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
	stMotorInfo.bIsSupportSR = 1;

	stMotorInfo.bIsMotorMoving = 1;

	if (*g_pAF_Opened >= 1)
		stMotorInfo.bIsMotorOpen = 1;
	else
		stMotorInfo.bIsMotorOpen = 0;

	if (copy_to_user(pstMotorInfo, &stMotorInfo, sizeof(struct stAF_MotorInfo)))
		LOG_INF("copy to user failed when getting motor information\n");

	return 0;
}

static int initdrv(void)
{
	
	LOG_INF("+\n");
	
	DW9786OISAF_Main_OIS();

	LOG_INF("-\n");
	return 0;
}


static inline int moveAF(unsigned long a_u4Position)
{
	int ret = 0;

	if ((a_u4Position > g_u4AF_MACRO) || (a_u4Position < g_u4AF_INF)) {
		LOG_INF("out of range\n");
		return -EINVAL;
	}

	if (*g_pAF_Opened == 1) {
		unsigned short InitPos;

		ret = s4AF_ReadReg(0xB300, &InitPos);

		if (ret == 0) {
			LOG_INF("Init Pos %6d\n", InitPos);

			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = (unsigned long)InitPos;
			spin_unlock(g_pAF_SpinLock);

		} else {
			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = 0;
			spin_unlock(g_pAF_SpinLock);
		}

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 2;
		spin_unlock(g_pAF_SpinLock);
	}


	spin_lock(g_pAF_SpinLock);
	g_u4TargetPosition = a_u4Position;
	spin_unlock(g_pAF_SpinLock);

	LOG_INF("move [curr] %d [target] %d\n", g_u4CurrPosition, g_u4TargetPosition); 


	if (s4AF_WriteReg(0xB300, (unsigned short)g_u4TargetPosition) == 0) {
		spin_lock(g_pAF_SpinLock);
		g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
		spin_unlock(g_pAF_SpinLock);
	} else {
		LOG_INF("set I2C failed when moving the motor\n");
		ret = -1;
	}

	return ret;
}

static inline int setAFInf(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_INF = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

static inline int setAFMacro(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_MACRO = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

/* ////////////////////////////////////////////////////////////// */
long DW9786OISAF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case AFIOC_G_MOTORINFO:
		i4RetValue = getAFInfo((__user struct stAF_MotorInfo *) (a_u4Param));
		break;

	case AFIOC_T_MOVETO:
		i4RetValue = moveAF(a_u4Param);
		break;

	case AFIOC_T_SETINFPOS:
		i4RetValue = setAFInf(a_u4Param);
		break;

	case AFIOC_T_SETMACROPOS:
		i4RetValue = setAFMacro(a_u4Param);
		break;
	case AFIOC_S_SETPARA:
		i4RetValue =
			setAFPara((__user struct stAF_MotorCmd *)(a_u4Param));
		break;


	default:
		LOG_INF("No CMD\n");
		i4RetValue = -EPERM;
		break;
	}

	return i4RetValue;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
int DW9786OISAF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	LOG_INF("Start\n");

	if (*g_pAF_Opened == 2){

		LOG_INF("Wait\n");
		s4AF_WriteReg(0xD013, 512);
		DW9786OISAF_OIS_Standby();
		msleep(20);
}
	
	if (*g_pAF_Opened) {
		LOG_INF("Free\n");

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 0;
		spin_unlock(g_pAF_SpinLock);
	}

	LOG_INF("End\n");

	return 0;
}

int DW9786OISAF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;

	
	initdrv();

	return 1;
}

int DW9786OISAF_GetFileName(unsigned char *pFileName)
{
	#if SUPPORT_GETTING_LENS_FOLDER_NAME
	char FilePath[256];
	char *FileString;

	sprintf(FilePath, "%s", __FILE__);
	FileString = strrchr(FilePath, '/');
	*FileString = '\0';
	FileString = (strrchr(FilePath, '/') + 1);
	strncpy(pFileName, FileString, AF_MOTOR_NAME);
	LOG_INF("FileName : %s\n", pFileName);
	#else
	pFileName[0] = '\0';
	#endif
	return 1;
}
