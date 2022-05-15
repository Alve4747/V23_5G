/*
 *  stmvl53l0.c - Linux kernel modules for STM LASER FlightSense Time-of-Flight sensor
 *
 *  Copyright (C) 2014 STMicroelectronics Imaging Division.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <asm/atomic.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif
#include <linux/laser_tof8801.h>
#include "laser.h"



#define PK_INF(fmt, args...)	 pr_info(LASER_DRVNAME "[%s] " fmt, __FUNCTION__, ##args)


static dev_t g_Laser_devno;
static struct cdev * g_pLaser_CharDrv = NULL;
static struct class *actuator_class = NULL;



static void Laser_Getdata(LaserInfo* LaserInfo)
{
	int ret;
	u32 distance_mm = 0;
	u32 confidence = 0;

	if( tof8801_get_status() == 0 ){
		PK_INF("tof8801_get_status fail\n");
		return ;
	}

	ret = tof8801_get_distance(&distance_mm, &confidence);
	if(ret)
		PK_INF("tof8801_get_distance fail\n");

	LaserInfo->u4LaserCurPos = distance_mm;
	LaserInfo->u4LaserStatus = confidence==0 ? STATUS_MOVE_MAX_RANGING_DIST : STATUS_RANGING_VALID;
	LaserInfo->u4LaserDMAX = LASER_MAXDISTANCE;

	PK_INF("LaserInfo->u4LaserCurPos = %d, LaserInfo->u4LaserStatus = %d, LaserInfo->u4LaserDMAX = %d,\n",LaserInfo->u4LaserCurPos, LaserInfo->u4LaserStatus, LaserInfo->u4LaserDMAX);

}



static long Laser_Ioctl(
	struct file * a_pstFile,
	unsigned int a_u4Command,
	unsigned long a_u4Param)
{
	long i4RetValue = 0;
	LaserInfo ParamVal;
	__user LaserInfo * p_u4Param;
	u32 state = 0;
	u32 ret = 0;


	switch(a_u4Command)
	{
	case LASER_IOCTL_INIT:	   /* init.  */

		if( tof8801_get_status() == 0 ){
			PK_INF("tof8801_get_status fail\n");
			break;
		}

		state = (u32)a_u4Param;
		PK_INF("LASER_IOCTL_INIT i =%d \n",state);
		if (state) {
			ret = tof8801_resume();
			if (ret)
			PK_INF("failed to resume tof8801 laser");
		} else {
			ret = tof8801_suspend();
   		if (ret)
			PK_INF("failed to suspend tof8801 laser");
		}
		break;

	case LASER_IOCTL_GETDATA:
		PK_INF("LASER_IOCTL_GETDATA \n");
		p_u4Param = (__user LaserInfo *)a_u4Param;
		Laser_Getdata(&ParamVal);

		if(copy_to_user(p_u4Param , &ParamVal , sizeof(LaserInfo)))
		{
			PK_INF("copy to user failed when getting motor information \n");
		}
		break;

	default :
		PK_INF("No CMD \n");
		i4RetValue = -EPERM;
		break;
	}

	return i4RetValue;
}

static int Laser_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
	int ret = 0;

	PK_INF("Start \n");

	if( tof8801_get_status() == 0 ){
		PK_INF("tof8801_get_status fail\n");
		return -EPERM;
	}

	ret = tof8801_resume();
	if (ret)
		PK_INF("failed to resume tof8801 laser");

	PK_INF("End \n");

	return ret;

}

static int Laser_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
	int ret;

	PK_INF("Start \n");

	if( tof8801_get_status() == 0 ){
		PK_INF("tof8801_get_status fail\n");
		return -EPERM;
	}

	ret = tof8801_suspend();
	if (ret)
		PK_INF("failed to suspend tof8801 laser");

	PK_INF("End \n");

	return ret;

}

static const struct file_operations g_stLaser_fops =
{
	.owner = THIS_MODULE,
	.open = Laser_Open,
	.release = Laser_Release,
	.unlocked_ioctl = Laser_Ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = Laser_Ioctl,
#endif
};

inline static int Register_Laser_CharDrv(void)
{

	static struct device* laser_device = NULL;

	if( alloc_chrdev_region(&g_Laser_devno, 0, 1,LASER_DRVNAME) )
	{
		PK_INF("Allocate device no failed\n");
		return -EAGAIN;
	}

	g_pLaser_CharDrv = cdev_alloc();
	if(NULL == g_pLaser_CharDrv)
	{
		unregister_chrdev_region(g_Laser_devno, 1);
		PK_INF("Allocate mem for kobject failed\n");
		return -ENOMEM;
	}

	cdev_init(g_pLaser_CharDrv, &g_stLaser_fops);
	g_pLaser_CharDrv->owner = THIS_MODULE;

	if(cdev_add(g_pLaser_CharDrv, g_Laser_devno, 1))
	{
		PK_INF("Attatch file operation failed\n");
		unregister_chrdev_region(g_Laser_devno, 1);
		return -EAGAIN;
	}

	actuator_class = class_create(THIS_MODULE, LASER_DRIVER_CLASS_NAME);
	if (IS_ERR(actuator_class))
	{
		int ret = PTR_ERR(actuator_class);
		PK_INF("Unable to create class, err = %d\n", ret);
		return ret;
	}

	laser_device = device_create(actuator_class, NULL, g_Laser_devno, NULL, LASER_DRVNAME);
	if(NULL == laser_device)
	{
		return -EIO;
	}


	PK_INF("End\n");
	return 0;
}

inline static void UnRegister_Laser_CharDrv(void)
{
	PK_INF("Start\n");
	cdev_del(g_pLaser_CharDrv);
	unregister_chrdev_region(g_Laser_devno, 1);
	device_destroy(actuator_class, g_Laser_devno);
	class_destroy(actuator_class);
	PK_INF("End\n");
}


static int Laser_probe(struct platform_device *pdev)
{
	int i4RetValue = 0;

	PK_INF("enter Laser_probe\n");

	i4RetValue = Register_Laser_CharDrv();
	if(i4RetValue){
		PK_INF(" register char device failed!\n");
		return i4RetValue;
	}

	PK_INF("exit Laser_probe\n");
	return 0;
}


static const struct of_device_id laser_gpio_of_match[] = {
	{.compatible = "mediatek,laser"},
	{},
};
MODULE_DEVICE_TABLE(of, laser_gpio_of_match);

static struct platform_driver g_stLaser_Driver = {
	.probe		= Laser_probe,
	.driver		= {
		.name	= PLATFORM_DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = laser_gpio_of_match,
	}
};

static int __init LASER_init(void)
{
	if(platform_driver_register(&g_stLaser_Driver))
	{
		PK_INF("Failed to register Laser driver\n");
		return -ENODEV;
	}

	PK_INF("LASER_init sucess\n");
	return 0;
}

static void __exit LASER_exit(void)
{
	platform_driver_unregister(&g_stLaser_Driver);
}

module_init(LASER_init);
module_exit(LASER_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("STMicroelectronics Imaging Division");
MODULE_DESCRIPTION("ST FlightSense Time-of-Flight sensor driver");


