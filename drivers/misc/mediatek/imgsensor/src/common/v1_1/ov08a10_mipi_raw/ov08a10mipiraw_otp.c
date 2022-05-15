#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "../imgsensor_i2c.h"
#include "ov08a10mipiraw_Sensor.h"

#define PFX "MAIN3[844]_EEPROM_OTP"
#define LOG_INF(format,  args...)	pr_debug(PFX "[%s] " format,  __FUNCTION__,  ##args)

/**************/
/****vivo cfx add start****/
extern void kdSetI2CSpeed(u16 i2cSpeed);
/*unsigned char PDAF_data[1310];*/
#define VIVO_OTP_DATA_SIZE 0x1A40 /*sizeof(moduleinfo)+sizeof(awb)+sizeof(af)+sizeof(lsc)*/
/*#define AF_RANGE_GOT*/  /****if got the range of AF_inf and AF_mac , open this define!!!!****/
#define VIVO_EEPROM_WRITE_ID 0x74
#define VIVO_I2C_SPEED 1000
#define VIVO_MAX_OFFSET 0x3A40
/*#define VIVO_VENDOR_SUNNY 0x01*/
#define VIVO_VENDOR 0x08
/*#define VIVO_VENDOR_OFILM 0x09*/
/*#define VIVO_VENDOR_LENS_ID 0x11*/
#define VIVO_SUNNY_LENS_ID 0x09
#define VIVO_VENDOR_VCM_ID 0x10
#define VIVO_VENDOR_DRIVERIC_ID 0x08
#define VIVO_VENDOR_PLATFORM_ID 0x04

unsigned char otp_data_ov08a10[VIVO_OTP_DATA_SIZE];
static unsigned const int ModuleInfo_addr = 0x0000;
static unsigned const int ModuleInfo_checksum_addr = 0x001f;
static unsigned const int FuseIDInfo_addr = 0x0020;
static unsigned const int FuseIDInfo_checksum_addr = 0x0044;
static unsigned const int SNInfo_addr = 0x0045;
static unsigned const int SNInfo_checksum_addr = 0x005F;
static unsigned const int Af_addr = 0x0790;
static unsigned const int Af_checksum_addr = 0x079F;
static unsigned const int Awb_addr = 0x0060;
static unsigned const int Awb_checksum_addr = 0x009F;
static unsigned const int Lsc_addr = 0x0D00;
static unsigned const int Lsc_checksum_addr = 0x144F;
static unsigned const int PD_Proc1_addr = 0x1450;
static unsigned const int PD_Proc1_checksum_addr = 0x164F;
static unsigned const int PD_Proc2_addr = 0x1650;
static unsigned const int PD_Proc2_checksum_addr = 0x1A3F;


#ifdef AF_RANGE_GOT
unsigned const int AF_inf_golden = 0;/*320;*/
unsigned const int AF_mac_golden = 0;/*680;*/
#endif
static int checksum;
extern otp_error_code_t OV08A10_OTP_ERROR_CODE;
extern MUINT32  sn_inf_ov08a10[13];
extern MUINT32  material_inf_ov08a10[4];

static void write_otp_sensor_16_16(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
	/* kdSetI2CSpeed(imgsensor_info.i2c_speed); Add this func to set i2c speed by each sensor*/
	iWriteRegI2C(pusendcmd , 4, 0x32);
}


static bool vivo_read_eeprom(kal_uint16 addr,  BYTE *data)
{
	int i=0,num=addr;
	char pu_send_cmd[2] = {(char)(addr >> 8),  (char)(addr & 0xFF)};
	if (addr > VIVO_MAX_OFFSET) { /*VIVO_MAX_OFFSET*/
		return false;
	}
	kdSetI2CSpeed(VIVO_I2C_SPEED);

	while(i*4096<VIVO_OTP_DATA_SIZE){
		if (iReadRegI2C(pu_send_cmd,  2,  (u8 *)(data+i*4096),  4096,  VIVO_EEPROM_WRITE_ID) < 0) {
			return false;
		}
		i++;
		num+=4096;
		pu_send_cmd[0]=(char)(num >> 8);
		pu_send_cmd[1]=(char)(num & 0xFF);
	}

	if(num<VIVO_OTP_DATA_SIZE){
		if (iReadRegI2C(pu_send_cmd,  2,  (u8 *)(data+i*4096),  VIVO_OTP_DATA_SIZE-4096*i,  VIVO_EEPROM_WRITE_ID) < 0) {
			return false;
		}
	}
   return true;
}

//extern unsigned int is_atboot;
/*guojunzheng add*/
int ov08a10_otp_read(void)
{
	int i = 0;
	int offset = 0x2000;
	int check_if_group_valid = 0;
	int R_unit = 0, B_unit = 0, G_unit = 0, R_golden = 0, B_golden = 0, G_golden = 0;
	int R_unit_low = 0, B_unit_low = 0, G_unit_low = 0, R_golden_low = 0, B_golden_low = 0, G_golden_low = 0;

	#ifdef AF_RANGE_GOT
	int diff_inf = 0,  diff_mac = 0,  diff_inf_macro = 0;
	int AF_inf = 0,  AF_mac = 0;
	#endif

	long long t1, t2, t3, t4, t5, t6, t, temp;
	OV08A10_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;

	write_otp_sensor_16_16(0xE000, 0x0001);

	mdelay(10);


	if (!vivo_read_eeprom(offset,  otp_data_ov08a10)) {
			OV08A10_OTP_ERROR_CODE = OTP_ERROR_CODE_EMPTY;
			LOG_INF("read_vivo_eeprom 0x%0x fail \n", offset);
			return 0;
	}

/*	for(i = 0; i < VIVO_OTP_DATA_SIZE; i++) {
		LOG_INF("read_vivo_eeprom Data[0x%0x]:0x%x\n", i,  otp_data_ov08a10[i]);
	}*/

	/*check_if_group_valid = otp_data_ov08a10[ModuleInfo_addr] | otp_data_ov08a10[Awb_addr] | otp_data_ov08a10[Af_addr] | otp_data_ov08a10[Lsc_addr] | otp_data_ov08a10[PD_Proc1_addr] | otp_data_ov08a10[PD_Proc2_addr];*/
	if ((0x01 == otp_data_ov08a10[ModuleInfo_addr]) &&
		(0x01 == otp_data_ov08a10[FuseIDInfo_addr]) &&
		(0x01 == otp_data_ov08a10[SNInfo_addr]) &&
		(0x01 == otp_data_ov08a10[Awb_addr]) &&
		(0x01 == otp_data_ov08a10[Lsc_addr]) &&
		(0x01 == otp_data_ov08a10[Af_addr]) &&
		(0x01 == otp_data_ov08a10[PD_Proc1_addr]) &&
		(0x01 == otp_data_ov08a10[PD_Proc2_addr])) {
		check_if_group_valid = 0x01;
		LOG_INF("0x01 is valid.check_if_group_valid:%d\n", check_if_group_valid);
	}

	if (check_if_group_valid == 0x01) { /****all the data is valid****/
		/****ModuleInfo****/
		if (  (VIVO_VENDOR != otp_data_ov08a10[0x01])) {
			OV08A10_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("Module ID error!!!    otp_error_code:%d\n", OV08A10_OTP_ERROR_CODE);
			return 0;
		} else if (( (VIVO_SUNNY_LENS_ID != otp_data_ov08a10[0x0008])) || 
			(VIVO_VENDOR_VCM_ID != otp_data_ov08a10[0x0009]) || 
			(VIVO_VENDOR_DRIVERIC_ID != otp_data_ov08a10[0x000A]) || 
			(VIVO_VENDOR_PLATFORM_ID != otp_data_ov08a10[0x0002])) {
			
			OV08A10_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF(": Platform ID or Lens or VCM ID or Driver_IC ID  Error!!!    otp_error_code:%d\n", OV08A10_OTP_ERROR_CODE);
			return 0;
		} else if ((0x00 != otp_data_ov08a10[0x000B]) || 
			(0x00 != otp_data_ov08a10[0x000C]) || 
			(0x00 != otp_data_ov08a10[0x000D]) || 
			(0x00 != otp_data_ov08a10[0x000E])) {
			OV08A10_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF(": calibration version  Error!!!    Read version:0x%2x%2x%2x%2x\n", 
				otp_data_ov08a10[0x000B], otp_data_ov08a10[0x000C], otp_data_ov08a10[0x000D], otp_data_ov08a10[0x000E]);
			return 0;
		}
		
		/****ModuleInfo_checksum****/
		checksum = 0;
		for (i = ModuleInfo_addr+1; i < ModuleInfo_checksum_addr; i++) {
			checksum += otp_data_ov08a10[i];
			}
			checksum = checksum % 0xff+1;
		if (otp_data_ov08a10[ModuleInfo_checksum_addr] != checksum) {
			OV08A10_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("ModuleInfo_checksum error!!!   otp_error_code:%d\n", OV08A10_OTP_ERROR_CODE);
			return 0;
			}
		
		/****material info start****/
			material_inf_ov08a10[0] = (MUINT32)otp_data_ov08a10[0x0F];
			material_inf_ov08a10[1] = (MUINT32)otp_data_ov08a10[0x10];
			material_inf_ov08a10[2] = (MUINT32)otp_data_ov08a10[0x11];
			material_inf_ov08a10[3] = (MUINT32)otp_data_ov08a10[0x12];
			LOG_INF("material_inf_ov08a10[0] = 0x%x, material_inf_ov08a10[1] = 0x%x, material_inf_ov08a10[2] = 0x%x,material_inf_ov08a10[3] = 0x%x\n",
				material_inf_ov08a10[0]  , material_inf_ov08a10[1],  material_inf_ov08a10[2], material_inf_ov08a10[3]);
		/****material info end****/


		/****Fuse id_checksum****/
		checksum = 0;
		for (i = FuseIDInfo_addr+1; i < FuseIDInfo_checksum_addr; i++) {
			checksum += otp_data_ov08a10[i];
			}
			checksum = checksum % 0xff+1;
		if (otp_data_ov08a10[FuseIDInfo_checksum_addr] != checksum) {
			OV08A10_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("Fuse id_checksum error!!!   otp_error_code:%d\n", OV08A10_OTP_ERROR_CODE);
			return 0;
		}

		
		/****SN_checksum****/
		checksum = 0;
		for (i = SNInfo_addr+1; i < SNInfo_checksum_addr; i++) {
			/*LOG_INF("otp_data_ov08a10[0x%x] = 0x%x\n", i, otp_data_ov08a10[i]);*/
			checksum += otp_data_ov08a10[i];
			}
			checksum = checksum % 0xff+1;
		if (otp_data_ov08a10[SNInfo_checksum_addr] != checksum) {
			OV08A10_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("SN_checksum error!!!   otp_error_code:%d\n", OV08A10_OTP_ERROR_CODE);
			return 0;
		}
		sn_inf_ov08a10[0] = 0x01;
		for (i = 0; i < 12; i++) {
			sn_inf_ov08a10[i +1] = (MUINT32)otp_data_ov08a10[i + SNInfo_addr+1];
			/*LOG_INF("sn_inf_ov08a10[%d] = 0x%x, otp_data_ov08a10[0x%x] = 0x%x\n", i+1  , sn_inf_ov08a10[i +1],  i +SN_addr+1, otp_data_ov08a10[i + SN_addr+1]);*/
		}

		/****AWB_checksum****/
		checksum = 0;
		for (i = Awb_addr+1; i < Awb_checksum_addr; i++) {
			checksum += otp_data_ov08a10[i];
			}
			checksum = checksum % 0xff+1;
		if (otp_data_ov08a10[Awb_checksum_addr] != checksum) {
			OV08A10_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("AWB_checksum error!!!   otp_error_code:%d\n", OV08A10_OTP_ERROR_CODE);
			return 0;
			}


		/****AF_checksum****/
		checksum = 0;
		for (i = Af_addr+1; i < Af_checksum_addr; i++) {
			checksum += otp_data_ov08a10[i];
			}
			checksum = checksum % 0xff+1;
		if (otp_data_ov08a10[Af_checksum_addr] != checksum) {
			OV08A10_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("AF_checksum error!!!   otp_error_code:%d\n", OV08A10_OTP_ERROR_CODE);
			return 0;
			}

		
		/****LSC_checksum****/
		checksum = 0;
		for (i = Lsc_addr+1; i < Lsc_checksum_addr; i++) {
			checksum += otp_data_ov08a10[i];
			}
			checksum = checksum % 0xff+1;
		if (otp_data_ov08a10[Lsc_checksum_addr] != checksum) {
			OV08A10_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("LSC_checksum error!!!   otp_error_code:%d\n", OV08A10_OTP_ERROR_CODE);
			return 0;
		}


		/****PDAF_proc1_checksum****/
		checksum = 0;
		for (i = PD_Proc1_addr+1; i < PD_Proc1_checksum_addr; i++) {
			checksum += otp_data_ov08a10[i];
		}
			checksum = checksum % 0xff+1;
		if (otp_data_ov08a10[PD_Proc1_checksum_addr] != checksum) {
			OV08A10_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("PDAF_data_checksum error!!!   otp_error_code:%d\n", OV08A10_OTP_ERROR_CODE);
			return 0;
			}

		/****PDAF_proc2_checksum****/
		checksum = 0;
		for (i = PD_Proc2_addr+1; i < PD_Proc2_checksum_addr; i++) {
			checksum += otp_data_ov08a10[i];
		}
			checksum = checksum % 0xff+1;
		if (otp_data_ov08a10[PD_Proc2_checksum_addr] != checksum) {
			OV08A10_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("PDAF_data_checksum error!!!   otp_error_code:%d\n", OV08A10_OTP_ERROR_CODE);
			return 0;
		}

		


		/****check if awb out of range[high cct]****/
		R_unit = otp_data_ov08a10[Awb_addr+1];
		R_unit = (R_unit << 8) | (otp_data_ov08a10[Awb_addr+2]);
		B_unit = otp_data_ov08a10[Awb_addr+3];
		B_unit = (B_unit << 8) | (otp_data_ov08a10[Awb_addr+4]);
		G_unit = otp_data_ov08a10[Awb_addr+5];
		G_unit = (G_unit << 8) | (otp_data_ov08a10[Awb_addr+6]);

		R_golden = otp_data_ov08a10[Awb_addr+7];
		R_golden = (R_golden << 8) | (otp_data_ov08a10[Awb_addr+8]);
		B_golden = otp_data_ov08a10[Awb_addr+9];
		B_golden = (B_golden << 8) | (otp_data_ov08a10[Awb_addr+10]);
		G_golden = otp_data_ov08a10[Awb_addr+11];
		G_golden = (G_golden << 8) | (otp_data_ov08a10[Awb_addr+12]);
		 
		/****awb_range = pow(pow(R_unit/R_golden-1, 2)+pow(B_unit/B_golden-1, 2)+pow(G_unit/G_golden-1, 2), 0.5);****/
		/****t = 1024^2 * (R_unit-R_golden)^2 /(R_golden)^2 + 1024^2 * (B_unit-B_golden)^2 /(B_golden)^2 + 1024^2 * (G_unit-G_golden)^2 /(G_golden)^2 < (10% * 1024^2)****/
		LOG_INF("cfx_add:R_unit=%d, R_golden=%d, B_unit=%d, B_golden=%d, G_unit=%d, G_golden=%d\n", R_unit, R_golden, B_unit, B_golden, G_unit, G_golden);
		t1 = 1024*1024*(R_unit-R_golden)*(R_unit-R_golden);
		t2 = R_golden*R_golden;
		t3 = 1048576*(G_unit-G_golden)*(G_unit-G_golden);
		t4 = G_golden*G_golden;
		t5 = 1048576*(B_unit-B_golden)*(B_unit-B_golden);
		t6 = B_golden*B_golden;
		temp = t1/t2 + t3/t4 + t5/t6 ;
		t = temp - 10485;
		LOG_INF("cfx_add:t1 = %lld , t2 = %lld , t3 = %lld , t4 = %lld , t5 = %lld , t6 = %lld , temp = %lld , t = %lld\n", t1, t2, t3, t4, t5, t6, temp, t);
		if (t > 0) {
			OV08A10_OTP_ERROR_CODE = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
			LOG_INF("AWB[high cct] out of range error!!!This module range^2 *1024^2 is %lld%%   otp_error_code:%d\n", temp, OV08A10_OTP_ERROR_CODE);
			return 0;
		}
		/****check if awb out of range[low cct]****/
		R_unit_low = otp_data_ov08a10[Awb_addr+13];
		R_unit_low = (R_unit_low << 8) | (otp_data_ov08a10[Awb_addr+14]);
		B_unit_low = otp_data_ov08a10[Awb_addr+15];
		B_unit_low = (B_unit_low << 8) | (otp_data_ov08a10[Awb_addr+16]);
		G_unit_low = otp_data_ov08a10[Awb_addr+17];
		G_unit_low = (G_unit_low << 8) | (otp_data_ov08a10[Awb_addr+18]);

		R_golden_low = otp_data_ov08a10[Awb_addr+19];
		R_golden_low = (R_golden_low << 8) | (otp_data_ov08a10[Awb_addr+20]);
		B_golden_low = otp_data_ov08a10[Awb_addr+21];
		B_golden_low = (B_golden_low << 8) | (otp_data_ov08a10[Awb_addr+22]);
		G_golden_low = otp_data_ov08a10[Awb_addr+23];
		G_golden_low = (G_golden_low << 8) | (otp_data_ov08a10[Awb_addr+24]);

		/****awb_range = pow(pow(R_unit/R_golden-1, 2)+pow(B_unit/B_golden-1, 2)+pow(G_unit/G_golden-1, 2), 0.5);****/
		/****t = 1024^2 * (R_unit-R_golden)^2 /(R_golden)^2 + 1024^2 * (B_unit-B_golden)^2 /(B_golden)^2 + 1024^2 * (G_unit-G_golden)^2 /(G_golden)^2 < (10% * 1024^2)****/
		LOG_INF("cfx_add:R_unit_low=%d, R_golden_low=%d, B_unit_low=%d, B_golden_low=%d, G_unit_low=%d, G_golden_low=%d\n", R_unit_low, R_golden_low, B_unit_low, B_golden_low, G_unit_low, G_golden_low);
		t1 = 1024*1024*(R_unit_low-R_golden_low)*(R_unit_low-R_golden_low);
		t2 = R_golden_low*R_golden_low;
		t3 = 1048576*(G_unit_low-G_golden_low)*(G_unit_low-G_golden_low);
		t4 = G_golden_low*G_golden_low;
		t5 = 1048576*(B_unit_low-B_golden_low)*(B_unit_low-B_golden_low);
		t6 = B_golden_low*B_golden_low;
		temp = t1/t2 + t3/t4 + t5/t6 ;
		t = temp - 10485;
		LOG_INF("cfx_add:t1 = %lld , t2 = %lld , t3 = %lld , t4 = %lld , t5 = %lld , t6 = %lld , temp = %lld , t = %lld\n", t1, t2, t3, t4, t5, t6, temp, t);
		if (t > 0) {
			OV08A10_OTP_ERROR_CODE = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
			LOG_INF("AWB[low cct] out of range error!!!This module range^2 *1024^2 is %lld%%   otp_error_code:%d\n", temp, OV08A10_OTP_ERROR_CODE);
			return 0;
		}

		#ifdef AF_RANGE_GOT
		/*******check if AF out of range******/

		AF_inf  =  otp_data_ov08a10[Af_addr+2];
		AF_inf = (AF_inf << 8) | (otp_data_ov08a10[Af_addr+3]);

		AF_mac = otp_data_ov08a10[Af_addr+4];
		AF_mac = (AF_mac << 8) | (otp_data_ov08a10[Af_addr+5]);

		diff_inf = (AF_inf - AF_inf_golden) > 0 ? (AF_inf - AF_inf_golden) : (AF_inf_golden - AF_inf);
		diff_mac = (AF_mac - AF_mac_golden) > 0 ? (AF_mac - AF_mac_golden) : (AF_mac_golden - AF_mac);
		diff_inf_macro = AF_mac - AF_inf;
		if (diff_inf > 70 || diff_mac > 80 || diff_inf_macro > 450 || diff_inf_macro < 250) {  /*AF code out of range*/
			OV08A10_OTP_ERROR_CODE = OTP_ERROR_CODE_AF_OUT_OF_RANGE;
			LOG_INF("AF out of range error!!!   otp_error_code:%d\n", OV08A10_OTP_ERROR_CODE);
			return 0;
		}
		#endif

		/*cfx add for pdaf data start 20161223*/
		/*for(i = 0;i < 1372;i++)
		{
			if(i == 0)
				LOG_INF(" read_S5K2L8_PDAF_data");
			if(i < 496)
				PDAF_data[i] = otp_data_ov08a10[PD_Proc1_addr+i+1];
			else //i >= 496
				PDAF_data[i] = otp_data_ov08a10[PD_Proc1_addr+i+3];
		}*/
		/*copy pdaf data*/
		/*memcpy(PDAF_data, &otp_data_ov08a10[PDAF_addr+1], PDAF_checksum_addr-PDAF_addr-1);*/
		/*cfx add for pdaf data end*/
		OV08A10_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
		return 1;
	} else {
		OV08A10_OTP_ERROR_CODE = OTP_ERROR_CODE_GROUP_INVALID;
		LOG_INF(" invalid otp data. error!!!   otp_error_code:%d\n", OV08A10_OTP_ERROR_CODE);
		return 0;
	}
}
/*vivo cfx add end*/
