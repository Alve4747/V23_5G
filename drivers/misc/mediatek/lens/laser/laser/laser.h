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


#define LASER_MAGIC 'A'

#define LASER_IOCTL_INIT 			_IOW(LASER_MAGIC, 	0x01, int)
#define LASER_IOCTL_GETOFFCALB		_IOR(LASER_MAGIC, 	0x02, int)
#define LASER_IOCTL_GETXTALKCALB	_IOR(LASER_MAGIC, 	0x03, int)
#define LASER_IOCTL_SETOFFCALB		_IOW(LASER_MAGIC, 	0x04, int)
#define LASER_IOCTL_SETXTALKCALB	_IOW(LASER_MAGIC, 	0x05, int)
#define LASER_IOCTL_GETDATA 		_IOR(LASER_MAGIC, 	0x0a, LaserInfo)

#define LASER_DRVNAME 		"laser"
#define LASER_MAXDISTANCE 	8000



#define PLATFORM_DRIVER_NAME 	"laser_actuator"
#define LASER_DRIVER_CLASS_NAME "laser"



typedef enum
{
	STATUS_RANGING_VALID		 = 0x0,      // reference laser ranging distance
	STATUS_MOVE_DMAX		 = 0x1,          // Search range [DMAX  : infinity]
	STATUS_MOVE_MAX_RANGING_DIST	 = 0x2,  // Search range [xx cm : infinity], according to the laser max ranging distance
	STATUS_NOT_REFERENCE		 = 0x3
} LASER_STATUS_T;

typedef struct
{
	int u4LaserCurPos;	//current position
	int u4LaserStatus;	//laser status
	int u4LaserDMAX;	//DMAX
} LaserInfo;

