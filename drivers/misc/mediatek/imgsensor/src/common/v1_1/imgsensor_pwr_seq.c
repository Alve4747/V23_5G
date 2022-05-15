/*
 * Copyright (C) 2017 MediaTek Inc.
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

#include "kd_imgsensor.h"


#include "imgsensor_hw.h"
#include "imgsensor_cfg_table.h"

/* Legacy design */
struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence[] = {

/*ss123 start sensor porting PD2102F_EX*/
#if defined(CONFIG_MTK_CAM_PD2102F_EX)
#if defined(OV64B40PD2102_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV64B40PD2102_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 2},
			{AVDD, Vol_2800, 2},
			{DVDD, Vol_1100, 2},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 10},
			{AFVDD, Vol_2800, 100},
		},
	},
#endif

#if defined(OV8856PD2102_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV8856PD2102_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{PDN, Vol_High, 0},// pdn is DVDD pin in new PD2102 HW
			{DVDD, Vol_1200, 1},// keep this for old PD2102 HW
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 3}
		},
	},
#endif

#if defined(OV32ALQ_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV32ALQ_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_High, 1},
			{DOVDD, Vol_1800, 1},
			{RST, Vol_High, 6},
			{SensorMCLK, Vol_High, 2},
		},
	},
#endif
#if defined(S5KGW3SP13PD2102_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KGW3SP13PD2102_MIPI_RAW,
		{
			{RST, Vol_Low, 2},
			{DOVDD, Vol_1800, 2},
			{DVDD, Vol_1050, 2},
			{AVDD, Vol_2800, 2},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 5}
		},
	},
#endif

#if defined(GC08A3PD2102_MIPI_RAW)
	{
		SENSOR_DRVNAME_GC08A3PD2102_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1200, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(S5KJD1_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KJD1_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DVDD, Vol_1100, 0},
			{AVDD, Vol_2800, 0},
			{AFVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 0},
			{PDN, Vol_High, 0},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(IMX586_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX586_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 0},
			{AFVDD, Vol_2800, 0},
			{DVDD, Vol_1100, 0},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 3}
		},
	},
#endif
#if defined(IMX319_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX319_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 0},
			{AFVDD, Vol_2800, 0},
			{DVDD, Vol_1100, 0},
			{DOVDD, Vol_1800, 1},
			{SensorMCLK, Vol_High, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(S5K3M5SX_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K3M5SX_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DVDD, Vol_1100, 0},
			{AVDD, Vol_2800, 0},
			{AFVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 1},
			{RST, Vol_High, 2},
			{SensorMCLK, Vol_High, 1}
		},
	},
#endif
#endif
/*ss123 end sensor porting PD2102F_EX*/

/*ss123 start for PD2069F_EX */
#if defined(CONFIG_MTK_CAM_PD2069F_EX)

#if defined(OV64B40_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV64B40_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 2},
			{AVDD, Vol_2800, 2},
			{DVDD, Vol_1100, 2},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 10},
			{AFVDD, Vol_2800, 1},
		},
	},
#endif

#if defined(OV02B10PD2069V1_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV02B10PD2069V1_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 10},
			{RST, Vol_High, 1},
			{SensorMCLK, Vol_High, 10},
		},
	},
#endif
#if defined(S5K3P9SP04PD2069_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K3P9SP04PD2069_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1100, 1},
			{DOVDD, Vol_1800, 1},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 3}
		},
	},
#endif
#if defined(OV8856PD2069_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV8856PD2069_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1200, 1},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 3}
		},
	},
#endif

#if defined(S5KGM1STPD2069CF_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KGM1STPD2069CF_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1100, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 3},
			{AFVDD, Vol_2800, 100}
		},
	},
#endif

#if defined(OV13B10PD2069CF_MIPI_RAW)
		{
			SENSOR_DRVNAME_OV13B10PD2069CF_MIPI_RAW,
			{
				{SensorMCLK, Vol_High,1},
				{DOVDD, Vol_1800, 1},
				{AVDD, Vol_2800, 1},
				{DVDD, Vol_1200, 1},
				{RST, Vol_Low, 1, Vol_Low, 2},
				{RST, Vol_High, 2},
				{AFVDD_DEF, Vol_2800, 1},
			},
		},
#endif

#if defined(HI846PD2069_MIPI_RAW)
	{
		SENSOR_DRVNAME_HI846PD2069_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 2},
			{AVDD, Vol_2800, 2},
			{DVDD, Vol_1200, 2},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 3}
		},
	},
#endif

#if defined(GC02M1BPD2069CF_MIPI_MONO)
	{
		SENSOR_DRVNAME_GC02M1BPD2069CF_MIPI_MONO,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#endif
/*ss123 end for PD2069F_EX */


#if defined(IMX766_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX766_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 3},
#ifdef CONFIG_REGULATOR_RT5133
			{AVDD1, Vol_1800, 0},
#endif
			{AFVDD, Vol_2800, 3},
			{DVDD, Vol_1100, 4},
			{DOVDD, Vol_1800, 1},
			{SensorMCLK, Vol_High, 6},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(CONFIG_MTK_CAM_PD2123)
#if defined(S5KGW3SP13PD2123_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KGW3SP13PD2123_MIPI_RAW,
		{
		
			{RST, Vol_Low, 2},
			{DOVDD, Vol_1800, 2},
			{DVDD, Vol_1050, 2},
			{AVDD, Vol_2800, 2},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 5}
		},
	},
#endif	

#if defined(S5KJN1SQ03PD2131_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KJN1SQ03PD2131_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1050, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 5},
		},
	},
#endif
#if defined(IMX355PD2131_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX355PD2131_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1200, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(OV02B10PD2131_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV02B10PD2131_MIPI_RAW,
		{
			{RST, Vol_Low, 2},
			{DOVDD, Vol_1800, 2},
			{AVDD, Vol_2800, 2},
			{SensorMCLK, Vol_High, 5},
			{RST, Vol_High, 9}
		},
	},
#endif
#endif

#if defined(CONFIG_MTK_CAM_PD2167F_EX)
#if defined(S5KGW1SD03PD2167F_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KGW1SD03PD2167F_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DVDD, Vol_1000, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(S5KJN1SQ03PD2167F_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KJN1SQ03PD2167F_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 2},
			{DVDD, Vol_1050, 2},
			{AVDD, Vol_2800, 7},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 3},
			//{AFVDD, Vol_2800, 2},
		},
	},
#endif
#if defined(S5KJNVSQ04PD2167F_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KJNVSQ04PD2167F_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 10},
			{DVDD, Vol_1050, 10},
			{AVDD, Vol_2800, 7},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 3},
			//{AFVDD, Vol_2800, 2},
		},
	},
#endif
#if defined(OV8856PD2167F_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV8856PD2167F_MIPI_RAW,
		{
			{RST, Vol_Low, 2},
			{DOVDD, Vol_1800, 2},
			{AVDD, Vol_2800, 2},
			{DVDD, Vol_1200, 2},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 3}
		},
	},
#endif
#if defined(HI846PD2167F_MIPI_RAW)
	{
		SENSOR_DRVNAME_HI846PD2167F_MIPI_RAW,
		{
			{RST, Vol_Low, 2},
			{AVDD, Vol_2800, 2},
			{DVDD, Vol_1200, 2},
			{DOVDD, Vol_1800, 2},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(OV02B10PD2167F_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV02B10PD2167F_MIPI_RAW,
		{
			{RST, Vol_Low, 2},
			{DOVDD, Vol_1800, 2},
			{AVDD, Vol_2800, 5},
			{RST, Vol_High, 2},
			{SensorMCLK, Vol_High, 9}
		},
	},
#endif
#endif


#if defined(CONFIG_MTK_CAM_PD2166)
#if defined(S5KJN1SQ03PD2166_MIPI_RAW)
{
	SENSOR_DRVNAME_S5KJN1SQ03PD2166_MIPI_RAW,
	{
		{RST, Vol_Low, 1},
		{DOVDD, Vol_1800, 1},
		{DVDD, Vol_1050, 1},
		{AVDD, Vol_2800, 4},
		{SensorMCLK, Vol_High, 2},
		{RST, Vol_High, 3},
		{AFVDD_DEF, Vol_2800, 2},
	},
},
#endif
#if defined(OV13B10PD2166_MIPI_RAW)
{
	SENSOR_DRVNAME_OV13B10PD2166_MIPI_RAW,
	{
		{SensorMCLK, Vol_High,1},
		{DOVDD, Vol_1800, 1},
		{AVDD, Vol_2800, 1},
		{DVDD, Vol_1200, 1},
		{RST, Vol_Low, 1, Vol_Low, 2},
		{RST, Vol_High, 2},
		{AFVDD_DEF, Vol_2800, 1},
	},
},
#endif
#if defined(HI1634QPD2166_MIPI_RAW)
{
	SENSOR_DRVNAME_HI1634QPD2166_MIPI_RAW,
	{
		{RST, Vol_Low, 1},
		{DOVDD, Vol_1800, 5},
		{AVDD, Vol_2800, 2},
		{DVDD, Vol_1100, 1},
		{SensorMCLK, Vol_High, 1},
		{RST, Vol_High, 1}
	},
},
#endif
#if defined(S5K4H7PD2166_MIPI_RAW)
{
	SENSOR_DRVNAME_S5K4H7PD2166_MIPI_RAW,
	{
		{RST, Vol_Low, 1},
		{SensorMCLK, Vol_High, 1},
		{AVDD, Vol_2800, 1},
		{DVDD, Vol_1200, 1},
                {DOVDD, Vol_1800, 1},
		{RST, Vol_High, 5}
	},
},
#endif
#if defined(GC02M1PD2166_MIPI_RAW)
{
	SENSOR_DRVNAME_GC02M1PD2166_MIPI_RAW,
	{
		{RST, Vol_Low, 1},
		{DOVDD, Vol_1800, 1},
		{AVDD, Vol_2800, 0},
		{SensorMCLK, Vol_High, 1},
		{RST, Vol_High, 2}
	},
},
#endif
#if defined(GC02M1BPD2166_MIPI_MONO)
{
	SENSOR_DRVNAME_GC02M1BPD2166_MIPI_MONO,
	{
		{RST, Vol_Low, 1},
		{DOVDD, Vol_1800, 1},
		{AVDD, Vol_2800, 0},
		{SensorMCLK, Vol_High, 1},
		{RST, Vol_High, 2}
	},
},
#endif
#endif

#if defined(CONFIG_MTK_CAM_PD2166A)
#if defined(S5KJN1SQ03PD2166A_MIPI_RAW)
{
	SENSOR_DRVNAME_S5KJN1SQ03PD2166A_MIPI_RAW,
	{
		{RST, Vol_Low, 1},
		{DOVDD, Vol_1800, 1},
		{DVDD, Vol_1050, 1},
		{AVDD, Vol_2800, 4},
		{SensorMCLK, Vol_High, 2},
		{RST, Vol_High, 3},
		{AFVDD_DEF, Vol_2800, 2},
	},
},
#endif
#if defined(OV13B10PD2166A_MIPI_RAW)
{
	SENSOR_DRVNAME_OV13B10PD2166A_MIPI_RAW,
	{
		{SensorMCLK, Vol_High,1},
		{DOVDD, Vol_1800, 1},
		{AVDD, Vol_2800, 1},
		{DVDD, Vol_1200, 1},
		{RST, Vol_Low, 1, Vol_Low, 2},
		{RST, Vol_High, 2},
		{AFVDD_DEF, Vol_2800, 1},
	},
},
#endif
#if defined(OV13B10V1PD2166A_MIPI_RAW)
{
	SENSOR_DRVNAME_OV13B10V1PD2166A_MIPI_RAW,
	{
		{SensorMCLK, Vol_High,1},
		{DOVDD, Vol_1800, 1},
		{AVDD, Vol_2800, 1},
		{DVDD, Vol_1200, 1},
		{RST, Vol_Low, 1, Vol_Low, 2},
		{RST, Vol_High, 2},
		{AFVDD_DEF, Vol_2800, 1},
	},
},
#endif
#if defined(HI1634QPD2166A_MIPI_RAW)
{
	SENSOR_DRVNAME_HI1634QPD2166A_MIPI_RAW,
	{
		{RST, Vol_Low, 1},
		{DOVDD, Vol_1800, 5},
		{AVDD, Vol_2800, 2},
		{DVDD, Vol_1100, 1},
		{SensorMCLK, Vol_High, 1},
		{RST, Vol_High, 1}
	},
},
#endif
#if defined(S5K4H7PD2166A_MIPI_RAW)
{
	SENSOR_DRVNAME_S5K4H7PD2166A_MIPI_RAW,
	{
		{RST, Vol_Low, 1},
		{SensorMCLK, Vol_High, 1},
		{AVDD, Vol_2800, 1},
		{DVDD, Vol_1200, 1},
                {DOVDD, Vol_1800, 1},
		{RST, Vol_High, 5}
	},
},
#endif
#if defined(S5K4H7V1PD2166A_MIPI_RAW)
{
	SENSOR_DRVNAME_S5K4H7V1PD2166A_MIPI_RAW,
	{
		{RST, Vol_Low, 1},
		{SensorMCLK, Vol_High, 1},
		{AVDD, Vol_2800, 1},
		{DVDD, Vol_1200, 1},
                {DOVDD, Vol_1800, 1},
		{RST, Vol_High, 5}
	},
},
#endif
#if defined(GC02M1PD2166A_MIPI_RAW)
{
	SENSOR_DRVNAME_GC02M1PD2166A_MIPI_RAW,
	{
		{RST, Vol_Low, 1},
		{DOVDD, Vol_1800, 1},
		{AVDD, Vol_2800, 0},
		{SensorMCLK, Vol_High, 1},
		{RST, Vol_High, 2}
	},
},
#endif
/*
#if defined(GC02M1BPD2166A_MIPI_MONO)
{
	SENSOR_DRVNAME_GC02M1BPD2166A_MIPI_MONO,
	{
		{RST, Vol_Low, 1},
		{DOVDD, Vol_1800, 1},
		{AVDD, Vol_2800, 0},
		{SensorMCLK, Vol_High, 1},
		{RST, Vol_High, 2}
	},
},
#endif
*/
#endif

#if defined(CONFIG_MTK_CAM_PD2159F_EX) // ----CONFIG_MTK_CAM_PD2159 start---
#if defined(S5KJN1SQ03PD2159_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KJN1SQ03PD2159_MIPI_RAW,
		{
#ifdef CONFIG_REGULATOR_FAN53870
            {PLUGIC_EN, Vol_High, 2},
#endif
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1050, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 3},
 			{AFVDD, Vol_2800, 2},
		},
	},
#endif
#if defined(S5KGH1SM24PD2159_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KGH1SM24PD2159_MIPI_RAW,
		{
#ifdef CONFIG_REGULATOR_FAN53870
            {PLUGIC_EN, Vol_High, 2},
#endif
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 2},
			{AVDD, Vol_2800, 2},
			{DVDD, Vol_1050, 2},
			{RST, Vol_High, 3},
			{SensorMCLK, Vol_High, 2},
			{AFVDD, Vol_2800, 2},
		},
	},
#endif
#if defined(OV8856PD2159_MIPI_RAW)
	{	SENSOR_DRVNAME_OV8856PD2159_MIPI_RAW,
		{
#ifdef CONFIG_REGULATOR_FAN53870
            {PLUGIC_EN, Vol_High, 2},
#endif
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1200, 1},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 3}
		},
	},
#endif
#if defined(GC02M1PD2159_MIPI_RAW)
	{	SENSOR_DRVNAME_GC02M1PD2159_MIPI_RAW,
		{
#ifdef CONFIG_REGULATOR_FAN53870
            {PLUGIC_EN, Vol_High, 2},
#endif
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 3}
		},
	},
#endif
#endif// ----CONFIG_MTK_CAM_PD2159 end---
//add by vivo zhangyinpeng for pd2163 camera start
#if defined(S5KHM2SP03PD2163_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KHM2SP03PD2163_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 2},
			{AVDD, Vol_2800, 2},
			{DVDD, Vol_1100, 2},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 10}
		},
	},
#endif
#if defined(S5KJN1SQ03PD2163_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KJN1SQ03PD2163_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1050, 1},
			{AVDD, Vol_2800, 1},
			{RST, Vol_High, 3},
			{SensorMCLK, Vol_High, 10},
		},
	},
#endif
#if defined(OV8856PD2163_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV8856PD2163_MIPI_RAW,
		{
			{RST, Vol_Low, 2},
			{DOVDD, Vol_1800, 2},
			{AVDD, Vol_2800, 2},
			{DVDD, Vol_1200, 2},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 3}
		},
	},
#endif
#if defined(HI846PD2163_MIPI_RAW)
		{
			SENSOR_DRVNAME_HI846PD2163_MIPI_RAW,
			{
				{RST, Vol_Low, 2},
				{AVDD, Vol_2800, 2},
				{DVDD, Vol_1200, 2},
				{DOVDD, Vol_1800, 2},
				{SensorMCLK, Vol_High, 2},
				{RST, Vol_High, 2}
			},
		},
#endif
#if defined(OV02B10PD2163_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV02B10PD2163_MIPI_RAW,
		{
			{RST, Vol_Low, 2},
			{DOVDD, Vol_1800, 2},
			{AVDD, Vol_2800, 2},
			{SensorMCLK, Vol_High, 5},
			{RST, Vol_High, 9}
		},
	},
#endif
//add by vivo zhangyinpeng for pd2163 camera end
#if defined(S5KGW3SP13PD2083_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KGW3SP13PD2083_MIPI_RAW,
		{
#ifdef CONFIG_REGULATOR_FAN53870
			{PLUGIC_EN, Vol_High, 2},
#endif
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1100, 1},
			{AVDD, Vol_2800, 1},
			{OISVDD, Vol_2800, 1},
			{OISVM, Vol_2800, 1},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(S5KGH1SM24PD2083_MIPI_RAW)
		{
		   SENSOR_DRVNAME_S5KGH1SM24PD2083_MIPI_RAW,
		   {
#ifdef CONFIG_REGULATOR_FAN53870
			{PLUGIC_EN, Vol_High, 2},
#endif
			   {RST, Vol_Low, 1},
			   {DOVDD, Vol_1800, 1},
			   {AVDD, Vol_2800, 1},
			   {DVDD, Vol_1050, 1},
			   {OISVDD, Vol_2800, 1},
//			   {OISVM, Vol_2800, 1},
			   {SensorMCLK, Vol_High, 1},
			   {RST, Vol_High, 5}
		   },
		},
#endif
#if defined(OV8856PD2083_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV8856PD2083_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1200, 1},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 3}
		},
	},
#endif
#if defined(OV02B10PD2083_MIPI_RAW)
		{
			SENSOR_DRVNAME_OV02B10PD2083_MIPI_RAW,
			{
				{RST, Vol_Low, 1},
				{DOVDD, Vol_1800, 1},
				{AVDD, Vol_2800, 1},
				{SensorMCLK, Vol_High, 5},
				{RST, Vol_High, 15}
			},
		},
#endif
#if defined(S5KJD1_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KJD1_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DVDD, Vol_1100, 0},
			{AVDD, Vol_2800, 0},
			{AFVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 0},
			{PDN, Vol_High, 0},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(IMX586_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX586_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 0},
#ifdef CONFIG_REGULATOR_RT5133
			{AVDD1, Vol_1800, 0},
#endif
		//	{AFVDD, Vol_2800, 0},
			{DVDD, Vol_1100, 0},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 3}
		},
	},
#endif
#if defined(IMX766PD2133_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX766PD2133_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DVDD, Vol_High, 1},
			{DOVDD, Vol_1800, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 3}
		},
	},
#endif
#if defined(IMX766PD2135_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX766PD2135_MIPI_RAW,
		{
#ifdef CONFIG_REGULATOR_FAN53870
			{PLUGIC_EN, Vol_1800, 2},
#endif
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 1},
			{AVDD_1P8, Vol_1800, 1},
			{DVDD, Vol_1180, 1},
			{DOVDD, Vol_1800, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 3}
		},
	},
#endif
#if defined(IMX663_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX663_MIPI_RAW,
		{
#ifdef CONFIG_REGULATOR_FAN53870
			{PLUGIC_EN, Vol_1800, 2},
#endif
		  {RST, Vol_Low, 1},
		  {AVDD, Vol_2800, 1},
		  {DVDD, Vol_1800, 1},
		  {DOVDD, Vol_1800,1},
		  {SensorMCLK, Vol_High, 2},
		  {RST, Vol_High, 5}
		},
	},
#endif
#if defined(IMX663PD2133_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX663PD2133_MIPI_RAW,
		{
		  {RST, Vol_Low, 1},
		  {AVDD, Vol_2800, 1},
		  {DVDD, Vol_High, 1},
		  {DOVDD, Vol_1800,1},
		  {SensorMCLK, Vol_High, 2},
		  {RST, Vol_High, 5}
		},
	},
#endif
#if defined(OV08A10_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV08A10_MIPI_RAW,
		{
#ifdef CONFIG_REGULATOR_FAN53870
			{PLUGIC_EN, Vol_1800, 2},
#endif
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1800, 1},
			{DOVDD, Vol_1800, 1},
			{OISVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 3}
		},
	},
#endif
#if defined(OV48B_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV48B_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 5},
			//{AFVDD, Vol_2800, 2},
			{RST, Vol_High, 5},
		},
	},
#endif
#if defined(S5K3P9SP_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DVDD, Vol_1100, 1},
			{AVDD, Vol_2800, 1},
			{DOVDD, Vol_1800, 0},
			{SensorMCLK, Vol_High, 0},
			//{AFVDD, Vol_2800, 5},
			{RST, Vol_High, 2},
		},
	},
#endif
#if defined(IMX319_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX319_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 0},
			{AFVDD, Vol_2800, 0},
			{DVDD, Vol_1100, 0},
			{DOVDD, Vol_1800, 1},
			{SensorMCLK, Vol_High, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(S5K3M5SX_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K3M5SX_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DVDD, Vol_1100, 1},
			{AVDD, Vol_2800, 1},
			//{AFVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 1},
			{RST, Vol_High, 2},
			{SensorMCLK, Vol_High, 1}
		},
	},
#endif
#if defined(GC02M0_MIPI_RAW)
	{
		SENSOR_DRVNAME_GC02M0_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{DOVDD, Vol_1800, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(IMX519_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX519_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{AVDD, Vol_2800, 0},
			{AFVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{DOVDD, Vol_1800, 1},
			{SensorMCLK, Vol_High, 5},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 1}
		},
	},
#endif
#if defined(IMX519DUAL_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX519DUAL_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{AVDD, Vol_2800, 0},
			{AFVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{DOVDD, Vol_1800, 1},
			{SensorMCLK, Vol_High, 5},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 1}
		},
	},
#endif
#if defined(IMX499_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX499_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{AVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 0},
			{DVDD, Vol_1100, 0},
			{AFVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 10}
		},
	},
#endif
#if defined(IMX481_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX481_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
#ifdef CONFIG_REGULATOR_RT5133
			//To trigger ex-LDO output 2.8V
			{AVDD, Vol_1800, 0},
#else
			// PMIC output 2.8V
			{AVDD, Vol_2800, 0},
#endif
			{DOVDD, Vol_1800, 0},
#ifdef CONFIG_REGULATOR_RT5133
			//To trigger ex-LDO output 1.1V
			{DVDD, Vol_1800, 0},
#else
			//PMIC output 1.1V
			{DVDD, Vol_1100, 0},
#endif
//			{AFVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 10}
		},
	},
#endif
#if defined(IMX576_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX576_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{AVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 0},
			{DVDD, Vol_1100, 1}, /*data sheet 1050*/
			{SensorMCLK, Vol_High, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 8}
		},
	},
#endif
#if defined(IMX350_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX350_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{AVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 0},
			{DVDD, Vol_1200, 5},
			{SensorMCLK, Vol_High, 5},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(IMX398_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX398_MIPI_RAW,
		{
			{SensorMCLK, Vol_Low, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{AVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 0},
			{DVDD, Vol_1100, 0},
			{AFVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 1},
		},
	},
#endif
#if defined(OV23850_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV23850_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 2},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 5},
		},
	},
#endif
#if defined(OV16885_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV16885_MIPI_RAW,
		{
			{PDN, Vol_Low, 1},
			{RST, Vol_Low, 1},
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1200, 5},
			{PDN, Vol_High, 2},
			{RST, Vol_High, 2},
		},
	},
#endif
#if defined(OV05A20_MIPI_RAW)
		{
			SENSOR_DRVNAME_OV05A20_MIPI_RAW,
			{
				{SensorMCLK, Vol_High, 1},
				{RST, Vol_Low, 1},
				{AVDD, Vol_2800, 10},
				{DOVDD, Vol_1800, 5},
				{DVDD, Vol_1200, 5},
				{RST, Vol_High, 15}
			},
		},
#endif

#if defined(IMX386_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX386_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1100, 0},
			{DOVDD, Vol_1800, 1},
			{AFVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 2},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 10},
		},
	},
#endif
#if defined(IMX386_MIPI_MONO)
	{
		SENSOR_DRVNAME_IMX386_MIPI_MONO,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1100, 0},
			{DOVDD, Vol_1800, 1},
			{AFVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 2},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 10},
		},
	},
#endif
#if defined(IMX376_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX376_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1200, 5},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(IMX338_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX338_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 1},
			{AVDD, Vol_2500, 0},
			{DOVDD, Vol_1800, 0},
			{DVDD, Vol_1100, 0},
			{AFVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(S5K2LQSX_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K2LQSX_MIPI_RAW,
		{
			{PDN, Vol_Low, 1},
			{RST, Vol_Low, 1},
			{SensorMCLK, Vol_High, 4},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1000, 1},
			{DOVDD, Vol_1800, 0},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 0},
		},
	},
#endif
#if defined(S5KGD2_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KGD2_MIPI_RAW,
		{
#ifdef CONFIG_REGULATOR_FAN53870
			{PLUGIC_EN, Vol_1800, 2},
#endif
			{RST, Vol_Low, 8},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1000, 1},
			{DOVDD, Vol_1800, 1},
			{OISVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 8}
		},
	},
#endif
#if defined(S5K3L6WIDEPD2133_MIPI_RAW)
		{
			SENSOR_DRVNAME_S5K3L6WIDEPD2133_MIPI_RAW,
			 {
				  {RST, Vol_Low, 1},
				  {SensorMCLK, Vol_High, 1},
				  {DOVDD, Vol_1800,5},
				  {AVDD, Vol_2800, 2},
				  {DVDD, Vol_1050, 1},
				  {RST, Vol_High, 5}
			 },
		},
#endif
#if defined(S5K3L6WIDEPD2135_MIPI_RAW)
		{
			SENSOR_DRVNAME_S5K3L6WIDEPD2135_MIPI_RAW,
			 {
#ifdef CONFIG_REGULATOR_FAN53870
			{PLUGIC_EN, Vol_1800, 2},
#endif
				  {RST, Vol_Low, 1},
				  {SensorMCLK, Vol_High, 1},
				  {DOVDD, Vol_1800,5},
				  {AVDD, Vol_2800, 2},
				  {DVDD, Vol_1800, 1},
				  {RST, Vol_High, 5}
			 },
		},
#endif
#if defined(RUMBAS4SWOIS_MIPI_RAW)
		{
			SENSOR_DRVNAME_RUMBAS4SWOIS_MIPI_RAW,
			 {
				  {RST, Vol_Low, 1},
				  {DOVDD, Vol_1800,1},
				  {OISVDD_DEPENDENCY, Vol_2800, 1},		 /*vivo hope add for ois listening mode 2021.5.20*/
				  {OISVDD, Vol_2800, 1},
				  {SensorMCLK, Vol_High, 1},
				  {RST, Vol_High, 1}
			 },
		},
#endif
#if defined(RUMBAS4SWOIS_MIPI_RAW)
		{
			SENSOR_DRVNAME_RUMBAS4SWOIS_MIPI_RAW_3V1,
			 {
				  {RST, Vol_Low, 1},
				  {DOVDD, Vol_1800,1},
				  {OISVDD_DEPENDENCY, Vol_2800, 1},		 /*vivo hope add for ois listening mode 2021.5.20*/
				  {OISVDD, Vol_3100, 1},
				  {SensorMCLK, Vol_High, 1},
				  {RST, Vol_High, 1}
			 },
		},
#endif
#if defined(S5K4H7_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K4H7_MIPI_RAW,
		{
			{PDN, Vol_Low, 1},
			{RST, Vol_Low, 1},
			{SensorMCLK, Vol_High, 4},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1000, 1},
			{DOVDD, Vol_1800, 0},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 0},
		},
	},
#endif
#if defined(S5K4E6_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K4E6_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2900, 0},
			{DVDD, Vol_1200, 2},
			{AFVDD, Vol_2800, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(S5K2T7SP_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K2T7SP_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1000, 0},
			{SensorMCLK, Vol_High, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 2},
		},
	},
#endif
#if defined(S5K3P8SP_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K3P8SP_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1000, 0},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_Low, 4},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 1},
			{RST, Vol_High, 0},
		},
	},
#endif
#if defined(S5K3P8SX_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K3P8SX_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{SensorMCLK, Vol_High, 1},
			{DVDD, Vol_1000, 0},
			{AVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 0},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 2},
		},
	},
#endif
#if defined(S5K3M2_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K3M2_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_Low, 4},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 1},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(S5K3P3SX_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K3P3SX_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_Low, 4},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 1},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(S5K5E2YA_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K5E2YA_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_Low, 4},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 1},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(S5K4ECGX_MIPI_YUV)
	{
		SENSOR_DRVNAME_S5K4ECGX_MIPI_YUV,
		{
			{DVDD, Vol_1200, 1},
			{AVDD, Vol_2800, 1},
			{DOVDD, Vol_1800, 1},
			{AFVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 1},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(OV16880_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV16880_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1200, 5},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(S5K2P7_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K2P7_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1000, 1},
			{DOVDD, Vol_1800, 1},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_Low, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 1},
			{RST, Vol_High, 0},
		},
	},
#endif
#if defined(S5K2P8_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K2P8_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_Low, 4},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 1},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(IMX258_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX258_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(IMX258_MIPI_MONO)
	{
		SENSOR_DRVNAME_IMX258_MIPI_MONO,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(IMX377_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX377_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(OV8858_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV8858_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1200, 5},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(OV8856_MIPI_RAW)
	{SENSOR_DRVNAME_OV8856_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 2},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 5},
		},
	},
#endif
#if defined(S5K2X8_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K2X8_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(IMX214_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX214_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 0},
			{DVDD, Vol_1000, 0},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 1}
		},
	},
#endif
#if defined(IMX214_MIPI_MONO)
	{
		SENSOR_DRVNAME_IMX214_MIPI_MONO,
		{
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 0},
			{DVDD, Vol_1000, 0},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 1}
		},
	},
#endif
#if defined(IMX230_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX230_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 10},
			{DOVDD, Vol_1800, 10},
			{DVDD, Vol_1200, 10},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(S5K3L8_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K3L8_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(IMX362_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX362_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 10},
			{DOVDD, Vol_1800, 10},
			{DVDD, Vol_1200, 10},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(S5K2L7_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K2L7_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1000, 0},
			{AFVDD, Vol_2800, 3},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(IMX318_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX318_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{AVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 5},
			{SensorMCLK, Vol_High, 5},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(OV8865_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV8865_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 5},
			{RST, Vol_Low, 5},
			{DOVDD, Vol_1800, 5},
			{AVDD, Vol_2800, 5},
			{DVDD, Vol_1200, 5},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_High, 5},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(IMX219_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX219_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 10},
			{DOVDD, Vol_1800, 10},
			{DVDD, Vol_1000, 10},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(S5K3M3_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K3M3_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1000, 0},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(OV5670_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV5670_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 5},
			{RST, Vol_Low, 5},
			{DOVDD, Vol_1800, 5},
			{AVDD, Vol_2800, 5},
			{DVDD, Vol_1200, 5},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_High, 5},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(OV5670_MIPI_RAW_2)
	{
		SENSOR_DRVNAME_OV5670_MIPI_RAW_2,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 5},
			{RST, Vol_Low, 5},
			{DOVDD, Vol_1800, 5},
			{AVDD, Vol_2800, 5},
			{DVDD, Vol_1200, 5},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_High, 5},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(OV2281_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV2281_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 0},
			{RST, Vol_Low, 10},
			{RST, Vol_High, 5},
			{PDN, Vol_Low, 5},
			{PDN, Vol_High, 5},
		},
	},
#endif
#if defined(OV20880_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV20880_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1100, 1},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(OV5645_MIPI_YUV)
	{
		SENSOR_DRVNAME_OV5645_MIPI_YUV,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 5},
			{PDN, Vol_High, 1},
			{RST, Vol_High, 10}
		},
	},
#endif
#if defined(S5K5E9_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K5E9_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 2},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 10}
		},
	},
#endif
#if defined(S5KGD1SP_MIPI_RAW)
		{
			SENSOR_DRVNAME_S5KGD1SP_MIPI_RAW,
			{
				{RST, Vol_Low, 1},
				{AVDD, Vol_2800, 0},
				{DVDD, Vol_1100, 0},
				{DOVDD, Vol_1800, 1},
				{SensorMCLK, Vol_High, 1},
				{RST, Vol_High, 2}
			},
		},
#endif
#if defined(HI846_MIPI_RAW)
		{
			SENSOR_DRVNAME_HI846_MIPI_RAW,
			{
				{RST, Vol_Low, 1},
				{AVDD, Vol_2800, 0},
				{DVDD, Vol_1200, 0},
				{DOVDD, Vol_1800, 1},
				{SensorMCLK, Vol_High, 1},
				{RST, Vol_High, 2}
			},
		},
#endif
#if defined(GC02M0_MIPI_RAW)
		{
			SENSOR_DRVNAME_GC02M0_MIPI_RAW,
			{
				{RST, Vol_Low, 1},
				{AVDD, Vol_2800, 0},
				{DVDD, Vol_1200, 0},
				{DOVDD, Vol_1800, 1},
				{SensorMCLK, Vol_High, 1},
				{RST, Vol_High, 2}
			},
		},
#endif
#if defined(OV02A10_MIPI_MONO)
		{
			SENSOR_DRVNAME_OV02A10_MIPI_MONO,
			{
				{RST, Vol_High, 1},
				{AVDD, Vol_2800, 0},
			/*main3 has no dvdd, compatible with sub2*/
				{DVDD, Vol_1200, 0},
				{DOVDD, Vol_1800, 0},
				{SensorMCLK, Vol_High, 5},
				{RST, Vol_Low, 9}
			},
		},
#endif
#if defined(IMX686_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX686_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{AVDD, Vol_2900, 0},
		/*in alph.dts file, pin avdd controls two gpio pins*/
			/*{AVDD_1, Vol_1800, 0},*/
			{DVDD, Vol_1100, 0},
			{DOVDD, Vol_1800, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 1}
		},
	},
#endif
#if defined(IMX616_MIPI_RAW)
		{
			SENSOR_DRVNAME_IMX616_MIPI_RAW,
			{
				{RST, Vol_Low, 1},
				{AVDD, Vol_2900, 0},
				{DVDD, Vol_1100, 0},
				{DOVDD, Vol_1800, 1},
				{SensorMCLK, Vol_High, 1},
				{RST, Vol_High, 2}
			},
		},
#endif
#if defined(IMX355_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX355_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1200, 1},
			{AVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(OV13B10_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV13B10_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{DOVDD, Vol_1800, 1},
			{RST, Vol_High, 5},
			{SensorMCLK, Vol_High, 1},
		},
	},
#endif
#if defined(OV48C_MIPI_RAW)
		{
			SENSOR_DRVNAME_OV48C_MIPI_RAW,
			{
				{RST, Vol_Low, 1},
				{SensorMCLK, Vol_High, 0},
				{DOVDD, Vol_1800, 0},
				{AVDD, Vol_2800, 0},
				{DVDD, Vol_1200, 5},
				{RST, Vol_High, 5},
			},
		},
#endif
#if defined(OV16A10_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV16A10_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{DOVDD, Vol_1800, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2},
		},
	},
#endif
#if defined(GC8054_MIPI_RAW)
	{
		SENSOR_DRVNAME_GC8054_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{RST, Vol_Low,  1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1200, 1},
			{AVDD, Vol_2800, 1},
			{RST, Vol_High, 1},
			//{AFVDD, Vol_Low, 5}
		},
	},
#endif
#if defined(GC02M0B_MIPI_MONO)
	{
		SENSOR_DRVNAME_GC02M0B_MIPI_MONO,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(GC02M1B_MIPI_MONO)
	{
		SENSOR_DRVNAME_GC02M1B_MIPI_MONO,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(GC02M0B_MIPI_MONO1)
	{
		SENSOR_DRVNAME_GC02M0B_MIPI_MONO1,
		{
			{RST, Vol_Low, 1},
			//{DVDD, Vol_1200, 0},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(GC02M0B_MIPI_MONO2)
	{
		SENSOR_DRVNAME_GC02M0B_MIPI_MONO2,
		{
			{RST, Vol_Low, 1},
			//{DVDD, Vol_1200, 0},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(GC02K0B_MIPI_MONO)
	{
		SENSOR_DRVNAME_GC02K0B_MIPI_MONO,
		{
			{RST, Vol_Low, 1},
			//{DVDD, Vol_1200, 0},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(OV02B10_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV02B10_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 9},
			{RST, Vol_High, 1}
		},
	},
#endif

	/* add new sensor before this line */
	{NULL,},
};

