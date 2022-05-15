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

#ifndef _LENS_LIST_H

#define _LENS_LIST_H

#define BU24253AF_SetI2Cclient BU24253AF_SetI2Cclient_Main3
#define BU24253AF_Ioctl BU24253AF_Ioctl_Main3
#define BU24253AF_Release BU24253AF_Release_Main3
#define BU24253AF_PowerDown BU24253AF_PowerDown_Main3
#define BU24253AF_GetFileName BU24253AF_GetFileName_Main3
extern int BU24253AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				 spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long BU24253AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
			   unsigned long a_u4Param);
extern int BU24253AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int BU24253AF_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int BU24253AF_GetFileName(unsigned char *pFileName);

#define GT9772WIDEAF_SetI2Cclient GT9772WIDEAF_SetI2Cclient_Main3
#define GT9772WIDEAF_Ioctl GT9772WIDEAF_Ioctl_Main3
#define GT9772WIDEAF_Release GT9772WIDEAF_Release_Main3
#define GT9772WIDEAF_PowerDown GT9772WIDEAF_PowerDown_Main3
#define GT9772WIDEAF_GetFileName GT9772WIDEAF_GetFileName_Main3
extern int GT9772WIDEAF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long GT9772WIDEAF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
				unsigned long a_u4Param);
extern int GT9772WIDEAF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int GT9772WIDEAF_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int GT9772WIDEAF_GetFileName(unsigned char *pFileName);

#define GT9772AF_SetI2Cclient GT9772AF_SetI2Cclient_Main3
#define GT9772AF_Ioctl GT9772AF_Ioctl_Main3
#define GT9772AF_Release GT9772AF_Release_Main3
#define GT9772AF_PowerDown GT9772AF_PowerDown_Main3
#define GT9772AF_GetFileName GT9772AF_GetFileName_Main3
extern int GT9772AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long GT9772AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
				unsigned long a_u4Param);
extern int GT9772AF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int GT9772AF_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int GT9772AF_GetFileName(unsigned char *pFileName);

#define DW9786OISAF_SetI2Cclient DW9786OISAF_SetI2Cclient_Main3
#define DW9786OISAF_Ioctl DW9786OISAF_Ioctl_Main3
#define DW9786OISAF_Release DW9786OISAF_Release_Main3
#define DW9786OISAF_PowerDown DW9786OISAF_PowerDown_Main3
#define DW9786OISAF_GetFileName DW9786OISAF_GetFileName_Main3
extern int DW9786OISAF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
				spinlock_t *pAF_SpinLock, int *pAF_Opened);
extern long DW9786OISAF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
				unsigned long a_u4Param);
extern int DW9786OISAF_Release(struct inode *a_pstInode, struct file *a_pstFile);
extern int DW9786OISAF_PowerDown(struct i2c_client *pstAF_I2Cclient,
				int *pAF_Opened);
extern int DW9786OISAF_GetFileName(unsigned char *pFileName);

extern void AFRegulatorCtrl(int Stage);
#endif
