/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2018, The Linux Foundation. All rights reserved.
 */
#ifndef FSA4480_I2C_H
#define FSA4480_I2C_H

#include <linux/of.h>
#include <linux/notifier.h>
//#include <max20328-i2c.h>

enum fsa_function {
	FSA_MIC_GND_SWAP,
	FSA_USBC_AUIDO_HP_ON,
	FSA_USBC_AUIDO_HP_OFF,
	FSA_USBC_ORIENTATION_CC1,
	FSA_USBC_ORIENTATION_CC2,
	FSA_USBC_DISPLAYPORT_DISCONNECTED,
	FSA_USBC_FAST_CHARGE_SELECT,
	FSA_USBC_FAST_CHARGE_EXIT,
	FSA_USBC_SWITCH_ENABLE,
	FSA_USBC_SWITCH_DISABLE,
	FSA_USBC_SWITCH_SBU_DIRECT_CONNECT,	//SBU1_H=SBU1¡¢SBU2_H=SBU2
	FSA_USBC_SWITCH_SBU_FLIP_CONNECT,	//SBU1_H=SBU2¡¢SBU2_H=SBU1
	FSA_USBC_SWITCH_SBU_HIZ,
	FSA_USBC_AUDIO_REPORT_IN,
	FSA_USBC_AUDIO_REPORT_REMOVE,
	FSA_EVENT_MAX,
};


enum fsa4480_regs_def {
	FSA4480_REG_DEVID    		= 0x00,
	FSA4480_REG_OVP_INT_MASK	= 0x01,
	FSA4480_REG_OVP_INT_FLAG	= 0x02,
	FSA4480_REG_OVP_STA 		= 0x03,
	FSA4480_REG_SW_EN  			= 0x04,
	FSA4480_REG_SW_SEL 			= 0x05,
	FSA4480_REG_SW_STA0 		= 0x06,
	FSA4480_REG_SW_STA1 		= 0x07,
	FSA4480_REG_AUDIO_L 		= 0x08,
	FSA4480_REG_AUDIO_R 		= 0x09,
	FSA4480_REG_MIC_SW			= 0x0a,
	FSA4480_REG_SENSE_SW		= 0x0b,
	FSA4480_REG_AUDIO_G 		= 0x0c,
	FSA4480_REG_RL_DELAY		= 0x0d,
	FSA4480_REG_ML_DELAY		= 0x0e,
	FSA4480_REG_SL_DELAY		= 0x0f,
	FSA4480_REG_GL_DELAY		= 0x10,
	FSA4480_REG_AUDIO_STA		= 0x11,
	FSA4480_REG_FUNC_EN 		= 0x12,
	FSA4480_REG_RES_DET_SET 	= 0x13,
	FSA4480_REG_RES_DET_VAL 	= 0x14,
	FSA4480_REG_RES_DET_THD 	= 0x15,
	FSA4480_REG_RES_DET_IVAL	= 0x16,
	FSA4480_REG_AUDIO_JACK_STA	= 0x17,
	FSA4480_REG_DET_INT 		= 0x18,
	FSA4480_REG_DET_INT_MASK	= 0x19,
	FSA4480_REG_AUDIO_RGE1		= 0x1a,
	FSA4480_REG_AUDIO_RGE2		= 0x1b,
	FSA4480_REG_MIC_THD0		= 0x1c,
	FSA4480_REG_MIC_THD1		= 0x1d,
	FSA4480_REG_I2C_RESET		= 0x1e,
	FSA4480_REG_CS_SET			= 0x1f,
	FSA4480_REG_CS_E0           = 0xe0,
	FSA4480_REG_CS_E1 			= 0xe1,
	FSA4480_REG_CS_E2			= 0xe2,
	FSA4480_REG_CS_E3 			= 0xe3,
	FSA4480_REG_CS_E4			= 0xe4,
	FSA4480_REG_CS_E5			= 0xe5,
	FSA4480_REG_CS_E6 			= 0xe6,
	FSA4480_REG_CS_E7 			= 0xe7,
	FSA4480_REG_CS_E8 			= 0xe8,
	FSA4480_REG_CS_E9 			= 0xe9,
	FSA4480_REG_CS_EA			= 0xea,
	FSA4480_REG_CS_EB			= 0xeb,
	FSA4480_REG_CS_EC 			= 0xec,
	FSA4480_REG_CS_ED			= 0xed,
	FSA4480_REG_CS_EE			= 0xee,
	FSA4480_REG_CS_EF			= 0xef,
	FSA4480_REG_CS_F0			= 0xf0,
	FSA4480_REG_CS_F1			= 0xf1,
	FSA4480_REG_CS_F2			= 0xf2,
	FSA4480_REG_MAX  			= FSA4480_REG_CS_F2,
};



static u8 fsa4480_regs[] = {
	FSA4480_REG_DEVID,
	FSA4480_REG_OVP_INT_MASK,
	FSA4480_REG_OVP_INT_FLAG,
	FSA4480_REG_OVP_STA,
	FSA4480_REG_SW_EN,
	FSA4480_REG_SW_SEL,
	FSA4480_REG_SW_STA0,
	FSA4480_REG_SW_STA1,
	FSA4480_REG_AUDIO_L,
	FSA4480_REG_AUDIO_R,
	FSA4480_REG_MIC_SW,
	FSA4480_REG_SENSE_SW,
	FSA4480_REG_AUDIO_G,
	FSA4480_REG_RL_DELAY,
	FSA4480_REG_ML_DELAY,
	FSA4480_REG_SL_DELAY,
	FSA4480_REG_GL_DELAY,
	FSA4480_REG_AUDIO_STA,
	FSA4480_REG_FUNC_EN,
	FSA4480_REG_RES_DET_SET,
	FSA4480_REG_RES_DET_VAL,
	FSA4480_REG_RES_DET_THD,
	FSA4480_REG_RES_DET_IVAL,
	FSA4480_REG_AUDIO_JACK_STA,
	FSA4480_REG_DET_INT,
	FSA4480_REG_DET_INT_MASK,
	FSA4480_REG_AUDIO_RGE1,
	FSA4480_REG_AUDIO_RGE2,
	FSA4480_REG_MIC_THD0,
	FSA4480_REG_MIC_THD1,
	FSA4480_REG_I2C_RESET,
	FSA4480_REG_CS_SET,
	FSA4480_REG_CS_E0,
	FSA4480_REG_CS_E1,
	FSA4480_REG_CS_E2,
	FSA4480_REG_CS_E3,
	FSA4480_REG_CS_E4,
	FSA4480_REG_CS_E5,
	FSA4480_REG_CS_E6,
	FSA4480_REG_CS_E7,
	FSA4480_REG_CS_E8,
	FSA4480_REG_CS_E9,
	FSA4480_REG_CS_EA,
	FSA4480_REG_CS_EB,
	FSA4480_REG_CS_EC,
	FSA4480_REG_CS_ED,
	FSA4480_REG_CS_EE,
	FSA4480_REG_CS_EF,
	FSA4480_REG_CS_F0,
	FSA4480_REG_CS_F1,
	FSA4480_REG_CS_F2,
	FSA4480_REG_MAX,
};


/*FSA4480 RES DETECTION PIN SETTING*/
enum {
	CC_IN__PIN = 0,
	DP_R__PIN,
	DN_L__PIN,
	SBU1__PIN,
	SBU2__PIN,

	ERROR__PIN,	
};

static const char * const RES_detection_pin[] = {
	"CC_IN__PIN",
	"DP_R__PIN",
	"DN_L__PIN",
	"SBU1__PIN",
	"SBU2__PIN",

	"ERROR__PIN",
};


#if 1

int fsa4480_reg_notifier(struct notifier_block *nb,
			 struct device_node *node);
int fsa4480_unreg_notifier(struct notifier_block *nb,
			   struct device_node *node);
/* Add this func for AT CMD and failure detection by vivo audio team@fanyongxiang.
 * return -1: Not support;
 * return 0: No peripheral;
 * return 1: Insert peripheral.
 */
int get_usbc_peripheral_status(void);
/* Add this func for Mic and Gnd switch status to AT CMD by vivo audio team@fanyongxiang.
 * return -1: Not support;
 * return 1: MG_SR->GSNS;
 * return 2: GM_SR->GSNS.
 */
int get_usbc_mg_status(void);
#else
static inline int fsa4480_switch_event(struct device_node *node,
				       enum fsa_function event)
{
	return 0;
}

static inline int fsa4480_reg_notifier(struct notifier_block *nb,
				       struct device_node *node)
{
	return 0;
}

static inline int fsa4480_unreg_notifier(struct notifier_block *nb,
					 struct device_node *node)
{
	return 0;
}
#endif /* CONFIG_QCOM_FSA4480_I2C */

int get_usb_connecter_pin_resistance_value(int pin_sel);

#endif /* FSA4480_I2C_H */

