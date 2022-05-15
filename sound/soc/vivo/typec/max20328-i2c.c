
/*
 * Copyright (C) 2020 Samsung System LSI, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

//#define pr_fmt(fmt) "%s(): " fmt, __func__

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/regmap.h>

#include <fsa4480-i2c.h>
#include <max20328-i2c.h>

#ifdef pr_debug
#undef pr_debug
#endif
#define pr_debug pr_info

#ifdef dev_dbg
#undef dev_dbg
#endif
#define dev_dbg dev_info

#define MAX20328B_VERSION	"v0.0"
unsigned int delay_bias = 30;
unsigned int delay_switch = 30;

char test_value[1024];
static struct i2c_client *local_i2c;
static bool is_audio_adapter;

int max20328_is_ready;
int mmax_is_unuseirq;

/*IC type*/
enum {
	MAX20328,
	FSA4480
};

static int regD_val, regE_val;

static struct max20328_priv *usbc_switch_mmax_priv;


struct max20328_priv {
	struct regmap *regmap;
	struct device *dev;
	struct power_supply *usb_psy;
	struct notifier_block psy_nb;
	struct votable *drp_mode_votable;
	atomic_t usbc_mode;
	struct work_struct usbc_analog_work;
	struct work_struct usbc_removed_work;
	struct work_struct max20328_irq_handler_work;
	struct blocking_notifier_head max20328_notifier;
	struct regulator *vdda33;
	wait_queue_head_t irq_waitq;
	bool power_enabled;
	struct mutex usbc_switch_lock;
	//struct wake_lock usbc_wake_lock;
	struct kobject *switch_kobj;
	struct kobject *kobj;
	int mmax_en;
	int mmax_int;
	int mmax_int_irq;
	int dev_gpio;
	int dev_gpio_irq;
	int current_plug;
	int gpio_detection_type;
	int current_switch_dev;
	int usbc_ic_mode;
	int is_mostest;
	int is_unuseirq;
};

struct max20328_reg_val {
	u16 reg;
	u8 val;
};

static const struct regmap_config max20328_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX20328_REG_MAX,
};

static const struct max20328_reg_val mmax_reg_i2c_defaults[] = {
	{MAX20328_SW_DEFLT1, 0x40}, /* 0x0D*/
	{MAX20328_SW_DEFLT2, 0x00}, /* 0x0E*/
	{MAX20328_ADC_CONTROL2, 0xF0}, /* 0x0A*/
	{MAX20328_CONTROL2, 0x00}, /* 0x07*/
	{MAX20328_CONTROL3, 0x00}, /* 0x08*/
	{MAX20328_ADC_CONTROL1, 0x30}, /* 0x09*/
	{MAX20328_CONTROL1, 0x13}, /* 0x06*/
};

static const struct max20328_reg_val fsa_reg_i2c_defaults[] = {
	{FSA4480_REG_SW_SEL, 0x18}, /* 0x05*/
	{FSA4480_REG_SW_EN, 0x98}, /* 0x04*/
};

int fsa4480_switch_event(struct device_node *node,
			 enum max_function event);
int fsa4480_switch_mode_event(enum max_function event);

static void max20328_usbc_switch_enable(struct max20328_priv *mmax_priv, bool enable)
{
	unsigned int val = 0, reg_06 = 0;
	int ret;

	if (!mmax_priv->regmap) {
		dev_err(mmax_priv->dev, "%s: regmap invalid\n", __func__);
		return;
	}

	val = enable ? 0x13 : 0x03;
	ret = regmap_write(mmax_priv->regmap, 0x06, val);
	if (ret)
		dev_err(mmax_priv->dev, "%s: failed %d\n", __func__, ret);
	usleep_range(5 * 1000, 5 * 1000);
	regmap_read(mmax_priv->regmap, 0x06, &reg_06);
	dev_dbg(mmax_priv->dev, "%s: enable (%d) reg_0x06 (0x%x)\n",
			__func__, enable, reg_06);
}

void max20328_usbc_set_switch_mode(int mode)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;

	if (!mmax_priv) {
		pr_err("%s: max20328 mmax_priv is null\n", __func__);
		return;
	}

	if (!mmax_priv->regmap) {
		dev_err(mmax_priv->dev, "%s: regmap invalid\n", __func__);
		return;
	}
	atomic_set(&(mmax_priv->usbc_mode), mode);
	mutex_lock(&mmax_priv->usbc_switch_lock);
	dev_dbg(mmax_priv->dev, "%s: %s mode (%d)\n", __func__, mmax_priv->usbc_ic_mode ? "FSA4480" : "MAX20328", mode);

	switch (mmax_priv->usbc_ic_mode) {
	case MAX20328:
		switch (mode) {
		case POWER_SUPPLY_TYPEC_NONE: /* USB mode */
			if(mmax_priv->is_mostest){
				regmap_write(mmax_priv->regmap, 0x0E, 0x40); /* DEF register2 set 00 */
				regmap_write(mmax_priv->regmap, 0x0D, 0x47); /* DEF register1 set TOP side closed in data connection, bottom side is open */
			} else {
				regmap_write(mmax_priv->regmap, 0x0E, 0x00); /* DEF register2 set 00 */
				regmap_write(mmax_priv->regmap, 0x0D, 0x40); /* DEF register1 set TOP side closed in data connection, bottom side is open */
			}
			regmap_write(mmax_priv->regmap, 0x07, 0x00); /* CONTROL2 register, switch state NOT Force mode nor follow MODE[0:2] */
			regmap_write(mmax_priv->regmap, 0x08, 0x00); /* CONTROL3 register, force value is not use, anyway default it. */
			regmap_write(mmax_priv->regmap, 0x09, 0x30); /* ADC CONTROL1, ADC is always off on USB MODE */
			regmap_write(mmax_priv->regmap, 0x06, 0x13); /* CONTROL1 register, switch enable, default programmable with registers 0x0D and 0x0E */
			break;
		case POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER:
			regmap_write(mmax_priv->regmap, 0x0D, 0x03); /* DEF register */
			regmap_write(mmax_priv->regmap, 0x0E, 0x10); /* DEF register2 */
			regmap_write(mmax_priv->regmap, 0x07, 0x02); /* CONTROL2 register */
			regmap_write(mmax_priv->regmap, 0x08, 0x00); /* CONTROL3 register */
			regmap_write(mmax_priv->regmap, 0x09, 0x00); /* ADC CONTROL1, ADC is always off */
			regmap_write(mmax_priv->regmap, 0x06, 0x13); /* CONTROL1 register, switch enable, single Audio accessory */
			break;
		default:
			break;
		}
		break;

	case FSA4480:
		switch (mode) {
		case POWER_SUPPLY_TYPEC_NONE: /* USB mode */
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_SEL, 0x18);
			if (mmax_priv->is_mostest)
				regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x9f);
			else
				regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x98);
			break;
		case POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER:
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_SEL, 0x00);
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x87);
			break;
		default:
			break;
		}
		break;

	default:
		break;
	}

	mutex_unlock(&mmax_priv->usbc_switch_lock);
}

EXPORT_SYMBOL(max20328_usbc_set_switch_mode);


/* wangkai add */
// flag = true, charger Plug in, DP_T/DM_T pull down, disconnect DP_AP --> DP_T
// flag = false, charger Plug out, no need to set it
void max20328_switch_DPDM_low(bool flag)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;

	unsigned int val = 0;

	if (!flag)
		return;

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: max20328 mmax_priv is null\n", __func__);
		return;
	}
	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: max20328 regmap is null\n", __func__);
		return;
	}

	mutex_lock(&mmax_priv->usbc_switch_lock);
	regmap_read(mmax_priv->regmap, 0x0D, &val);
	printk("%s read 0x0D register is 0x%2x (before)\n", __func__, val);
	val = (val & 0x3f);
	printk("%s write 0x%2x to 0x0D register\n", __func__, val);
	regmap_write(mmax_priv->regmap, 0x0D, val);
	regmap_read(mmax_priv->regmap, 0x0D, &val);
	printk("%s read 0x0D register is 0x%2x (after)\n", __func__, val);
	mutex_unlock(&mmax_priv->usbc_switch_lock);
}
EXPORT_SYMBOL(max20328_switch_DPDM_low);

void max20328_save_0x0D_0x0E_reg(void)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: max20328 mmax_priv is null\n", __func__);
		regD_val = 0;
		regE_val = 0;
		return;
	}
	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: max20328 regmap is null\n", __func__);
		regD_val = 0;
		regE_val = 0;
		return;
	}

	mutex_lock(&mmax_priv->usbc_switch_lock);
	regmap_read(mmax_priv->regmap, 0x0D, &regD_val);
	regmap_read(mmax_priv->regmap, 0x0E, &regE_val);
	pr_info("%s: 0x0D's val = 0x%02x, 0x0E's val = 0x%02x\n", __func__, regD_val, regE_val);
	mutex_unlock(&mmax_priv->usbc_switch_lock);
}
EXPORT_SYMBOL(max20328_save_0x0D_0x0E_reg);

/* end */
int max20328_switch_mode_event(enum max_function event)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	union power_supply_propval pval = {0, };
	//bool is_audio_adapter;
	unsigned int val = 0;
	unsigned int val_0x0D = 0, val_0x0E = 0;

	if (!mmax_priv) {
		pr_err("%s: max20328 mmax_priv is null\n", __func__);
		return -EINVAL;
	}
	if (!mmax_priv->regmap) {
		pr_err("%s: max20328 regmap is null\n", __func__);
		return -EINVAL;
	}

	//max20328_usbc_wait_event_wake(mmax_priv);

	mutex_lock(&mmax_priv->usbc_switch_lock);

	if (atomic_read(&(mmax_priv->usbc_mode)) == POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER)
		is_audio_adapter = true;
	else
		is_audio_adapter = false;

	dev_dbg(mmax_priv->dev, "%s: %s: event: %d, mode: %d, is_audio_adapter: %d.\n",
			__func__, mmax_priv->usbc_ic_mode ? "FSA4480":"MAX20328", event, atomic_read(&(mmax_priv->usbc_mode)), is_audio_adapter);

	switch (mmax_priv->usbc_ic_mode) {
	case MAX20328:
		switch (event) {
		case MAX_MIC_GND_SWAP:
			if (is_audio_adapter) {
				regmap_read(mmax_priv->regmap, 0x0D, &val);
				if ((val & 0x0f) == 0x07) {
					val = 0x03 | (val & 0xf0);
					regmap_write(mmax_priv->regmap, 0x0D, val);
					regmap_write(mmax_priv->regmap, 0x0E, 0x10);
				} else {
					val = 0x07 | (val & 0xf0);
					regmap_write(mmax_priv->regmap, 0x0D, val);
					regmap_write(mmax_priv->regmap, 0x0E, 0x40);
				}
			}
			break;
		case MAX_USBC_AUIDO_HP_ON:
			if (is_audio_adapter) {
				regmap_read(mmax_priv->regmap, 0x0D, &val);
				if ((val & 0x0f) == 0x03)
					regmap_write(mmax_priv->regmap, 0x0E, 0x10);
				else
					regmap_write(mmax_priv->regmap, 0x0E, 0x40);
				usleep_range(2000, 2020);
				val = 0xa0 | (val & 0x0f);
				regmap_write(mmax_priv->regmap, 0x0D, val);
			}
			break;
		case MAX_USBC_AUIDO_HP_OFF:
			if (is_audio_adapter) {
				regmap_read(mmax_priv->regmap, 0x0D, &val);
				val = val & 0x0f;
				regmap_write(mmax_priv->regmap, 0x0D, val);
				usleep_range(2000, 2020);
				regmap_write(mmax_priv->regmap, 0x0E, 0x00);
			}
			break;
		case MAX_USBC_ORIENTATION_CC2:
			regmap_write(mmax_priv->regmap, 0x06, 0x34);
			break;
		case MAX_USBC_DISPLAYPORT_DISCONNECTED:
			regmap_write(mmax_priv->regmap, 0x06, 0x14);
			break;
		case MAX_USBC_FAST_CHARGE_SELECT:
			if (!is_audio_adapter)
				regmap_write(mmax_priv->regmap, 0x0D, 0x10);
			break;
		case MAX_USBC_FAST_CHARGE_EXIT:
			if (!is_audio_adapter) {
				if(mmax_priv->is_mostest){
					regmap_write(mmax_priv->regmap, 0x0D, 0x47);
				} else {
					regmap_write(mmax_priv->regmap, 0x0D, 0x40);
				}
			}
			break;
		case MAX_USBC_SWITCH_ENABLE:
			if (!is_audio_adapter)
				max20328_usbc_switch_enable(mmax_priv, true);
			break;
		case MAX_USBC_SWITCH_DISABLE:
			if (!is_audio_adapter)
				max20328_usbc_switch_enable(mmax_priv, false);
			break;
		case MAX_USBC_SWITCH_SBU_DIRECT_CONNECT:
			if (!is_audio_adapter) {
				regmap_write(mmax_priv->regmap, 0x0D, 0x14);
				regmap_write(mmax_priv->regmap, 0x0E, 0x02);
			}
			break;
		case MAX_USBC_SWITCH_SBU_HIZ:
			if (!is_audio_adapter) {
				regmap_write(mmax_priv->regmap, 0x0D, 0x10);
				regmap_write(mmax_priv->regmap, 0x0E, 0x00);
			}
			break;
		case MAX_USBC_AUDIO_REPORT_IN:
			if (is_audio_adapter) {
				memset(&pval, 0, sizeof(pval));
				pval.intval = true;
			}
			break;
		case MAX_USBC_AUDIO_REPORT_REMOVE:
			memset(&pval, 0, sizeof(pval));
			pval.intval = false;
			break;
		default:
			break;
		}
		regmap_read(mmax_priv->regmap, 0x0D, &val_0x0D);
		regmap_read(mmax_priv->regmap, 0x0E, &val_0x0E);
		dev_dbg(mmax_priv->dev, "%s: max20328: val_0x0D = 0x%x, val_0x0E = 0x%x, is_audio_adapter %d\n",
				__func__, val_0x0D, val_0x0E, is_audio_adapter);
		break;

	case FSA4480:
		switch (event) {
		case MAX_MIC_GND_SWAP:
			if (is_audio_adapter) {
				regmap_read(mmax_priv->regmap, FSA4480_REG_SW_SEL, &val);
				if ((val & 0x07) == 0x07) {
					regmap_write(mmax_priv->regmap, FSA4480_REG_SW_SEL, 0x00);
					/* regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x87); */
				} else {
					regmap_write(mmax_priv->regmap, FSA4480_REG_SW_SEL, 0x07);
					/* regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x87); */
				}
			}
			break;
		case MAX_USBC_AUIDO_HP_ON:
			if (is_audio_adapter) {
				regmap_read(mmax_priv->regmap, FSA4480_REG_SW_EN, &val);
				val = 0x04 | (val & 0x9f);
				regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, val);
				usleep_range(2000, 2020);
				val = 0x18 | (val & 0x9f);
				regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, val);
			}
			break;
		case MAX_USBC_AUIDO_HP_OFF:
			if (is_audio_adapter) {
				regmap_read(mmax_priv->regmap, FSA4480_REG_SW_EN, &val);
				val = val & 0xe7;
				regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, val);
				usleep_range(2000, 2020);
				val = val & 0xe3;
				regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, val);
			}
			break;
		case MAX_USBC_ORIENTATION_CC2:
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_SEL, 0x18);
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x98);
			break;
		case MAX_USBC_DISPLAYPORT_DISCONNECTED:
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_SEL, 0x18);
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x98);
			break;
		case MAX_USBC_FAST_CHARGE_SELECT:
			if (!is_audio_adapter) {
				regmap_write(mmax_priv->regmap, FSA4480_REG_SW_SEL, 0x18);
				regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x98);
			}
			break;
		case MAX_USBC_FAST_CHARGE_EXIT:
			if (!is_audio_adapter) {
				regmap_write(mmax_priv->regmap, FSA4480_REG_SW_SEL, 0x18);
				if (mmax_priv->is_mostest)
					regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x9f);
				else
					regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x98);
			}
			break;
		/*
		case MAX_USBC_SWITCH_ENABLE:
			if (!is_audio_adapter) {
				regmap_write(mmax_priv->regmap, FSA4480_REG_SW_SEL, 0x18);
				regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x98);
			}
			break;
		case MAX_USBC_SWITCH_DISABLE:
			if (!is_audio_adapter) {
				regmap_write(mmax_priv->regmap, FSA4480_REG_SW_SEL, 0x18);
				regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x98);
			}
			break;
		*/
		case FSA_USBC_SWITCH_SBU_DIRECT_CONNECT:
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_SEL, 0x18);
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0xF8);
			break;
		case FSA_USBC_SWITCH_SBU_FLIP_CONNECT:
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_SEL, 0x78);
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0xF8);
			break;
		case FSA_USBC_SWITCH_SBU_HIZ:
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_SEL, 0x18);
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x98);
			break;
		case MAX_USBC_AUDIO_REPORT_IN:
			if (is_audio_adapter) {
				memset(&pval, 0, sizeof(pval));
				pval.intval = true;
			}
			break;
		case MAX_USBC_AUDIO_REPORT_REMOVE:
			memset(&pval, 0, sizeof(pval));
			pval.intval = false;
			break;
		default:
			break;
		}
		regmap_read(mmax_priv->regmap, FSA4480_REG_SW_SEL, &val_0x0D);
		regmap_read(mmax_priv->regmap, FSA4480_REG_SW_EN, &val_0x0E);
		dev_dbg(mmax_priv->dev, "%s: fsa4480: val_sel = 0x%x,val_en = 0x%x is_audio_adapter %d\n",
				__func__, val_0x0D, val_0x0E, is_audio_adapter);
		break;

	default:
		break;
	}

	mutex_unlock(&mmax_priv->usbc_switch_lock);

	return 0;
}

EXPORT_SYMBOL(max20328_switch_mode_event);

/*
 * fsa4480_switch_event - configure MMAX switch position based on event
 *
 * @node - phandle node to max20328 device
 * @event - fsa_function enum
 *
 * Returns int on whether the switch happened or not
 */
int fsa4480_switch_event(struct device_node *node,
			 enum max_function event)
{
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct max20328_priv *mmax_priv;

	if (!client)
		return -EINVAL;

	mmax_priv = (struct max20328_priv *)i2c_get_clientdata(client);
	if (!mmax_priv)
		return -EINVAL;
	if (!mmax_priv->regmap)
		return -EINVAL;

	max20328_switch_mode_event(event);

	return 0;
}
EXPORT_SYMBOL(fsa4480_switch_event);

int fsa4480_switch_mode_event(enum max_function event)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;

	if (!mmax_priv) {
		pr_err("%s: max20328 mmax_priv is null\n", __func__);
		return -EINVAL;
	}
	if (!mmax_priv->regmap) {
		pr_err("%s: max20328 regmap is null\n", __func__);
		return -EINVAL;
	}

	max20328_switch_mode_event(event);

	return 0;
}
EXPORT_SYMBOL(fsa4480_switch_mode_event);

int max20328_switch_status_restore(void)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	unsigned int data;
	int i;

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: max20328 mmax_priv is null\n", __func__);
		return -EINVAL;
	}
	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: max20328 regmap is null\n", __func__);
		return -EINVAL;
	}

	regmap_write(mmax_priv->regmap, 0x0D, regD_val); /* DEF register */
	regmap_write(mmax_priv->regmap, 0x0E, regE_val); /* DEF register2 */
	regmap_write(mmax_priv->regmap, 0x07, 0x02); /* CONTROL2 register */
	regmap_write(mmax_priv->regmap, 0x08, 0x00); /* CONTROL3 register */
	regmap_write(mmax_priv->regmap, 0x09, 0x00); /* ADC CONTROL1, ADC is always off */
	regmap_write(mmax_priv->regmap, 0x06, 0x13); /* CONTROL1 register, switch enable, single Audio accessory */
	for (i = 0; i < sizeof(max20328_regs); i++) {
		regmap_read(mmax_priv->regmap, max20328_regs[i], &data);
		pr_info("%s: reg[0x%02x]: 0x%02x.\n", __func__, max20328_regs[i], data);
	}

	return 0;
}
EXPORT_SYMBOL(max20328_switch_status_restore);

int get_usbc_mg_status(void)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	unsigned int val_0x0D = 0, val_0x0E = 0, val = 0;
	int ret = -1;

	if (!mmax_priv) {
		pr_err("%s: mmax container invalid\n", __func__);
		return ret;
	}

	if (!mmax_priv->regmap) {
		pr_err("%s: max20328 regmap is null\n", __func__);
		return ret;
	}

	mutex_lock(&mmax_priv->usbc_switch_lock);
	switch (mmax_priv->usbc_ic_mode) {
	case MAX20328:
		regmap_read(mmax_priv->regmap, MAX20328_SW_DEFLT1, &val_0x0D);
		regmap_read(mmax_priv->regmap, MAX20328_SW_DEFLT2, &val_0x0E);
		pr_info("%s: max20328: val_0x0D: 0x%x, val_0x0E: 0x%x\n",
				__func__, val_0x0D, val_0x0E);
		if ((val_0x0D & 0xf) == 0x7) {
			ret = 1;
		} else if ((val_0x0D & 0xf) == 0x3) {
			ret = 2;
		}

		pr_err("%s: get_usbc_mg_status %d\n", __func__, ret);
		break;
	case FSA4480:
		regmap_read(mmax_priv->regmap, FSA4480_REG_SW_SEL, &val);
		pr_info("%s: fas4480: val_sel: 0x%x\n", __func__, val);
		if ((val & 0x07) == 0x07) {
			ret = 1;
		} else if ((val & 0x07) == 0x00) {
			ret = 2;
		}
		break;
	default:
		break;
	}
	mutex_unlock(&mmax_priv->usbc_switch_lock);

	return ret;
}
EXPORT_SYMBOL(get_usbc_mg_status);

static int max20328_ic_check_to_repair(struct max20328_priv *mmax_priv)
{
	int ret = 0, reg_ed = 0, repair_ok = 0;

	mutex_lock(&mmax_priv->usbc_switch_lock);
	ret = regmap_read(mmax_priv->regmap, FSA4480_REG_CS_ED, &reg_ed);
	if (reg_ed & 0x03) {
		pr_info("%s: reg_ed %d, chip damage, start to repair\n", __func__, reg_ed);
		regmap_write(mmax_priv->regmap, FSA4480_REG_CS_E0, 0x01);
		regmap_write(mmax_priv->regmap, FSA4480_REG_CS_EA, 0x07);
		regmap_write(mmax_priv->regmap, FSA4480_REG_CS_F0, 0x0b);
		regmap_write(mmax_priv->regmap, FSA4480_REG_CS_F1, 0xf7);
		regmap_write(mmax_priv->regmap, FSA4480_REG_CS_F2, 0x08);
		regmap_write(mmax_priv->regmap, FSA4480_REG_CS_E3, 0x07);
		regmap_write(mmax_priv->regmap, FSA4480_REG_CS_E2, 0x02);
		usleep_range(1 * 1000, 1 * 1000);
		ret = regmap_read(mmax_priv->regmap, FSA4480_REG_CS_ED, &reg_ed);
		if (reg_ed == 0x08) {
			repair_ok = 1;
			pr_info("%s: chip repair successful\n", __func__);
		} else {
			pr_info("%s: chip repair fail, reg_ed %d\n", __func__, reg_ed);
		}
		regmap_write(mmax_priv->regmap, FSA4480_REG_CS_E0, 0x00);

	} else {
		repair_ok = 2;
		pr_info("%s: reg_ed %d, chip may not damage, continue\n", __func__, reg_ed);
	}
	mutex_unlock(&mmax_priv->usbc_switch_lock);

	return repair_ok;
}

static void max20328_update_reg_defaults(struct max20328_priv *mmax_priv)
{
	u8 i;

	if (!mmax_priv)
		return;

	switch (mmax_priv->usbc_ic_mode) {
	case MAX20328:
		for (i = 0; i < ARRAY_SIZE(mmax_reg_i2c_defaults); i++)
			regmap_write(mmax_priv->regmap, mmax_reg_i2c_defaults[i].reg,
					   mmax_reg_i2c_defaults[i].val);

		if(mmax_priv->is_mostest){
			regmap_write(mmax_priv->regmap, 0x0d, 0x47);
			regmap_write(mmax_priv->regmap, 0x0e, 0x40);
		}

		break;
	case FSA4480:
		/* i2c reset */
		regmap_write(mmax_priv->regmap, FSA4480_REG_I2C_RESET, 0x01);
		usleep_range(1 * 1000, 1 * 1000);
		for (i = 0; i < 3; i++) {
			if (max20328_ic_check_to_repair(mmax_priv))
				break;
			usleep_range(5 * 1000, 5 * 1000);
		}
		usleep_range(1 * 1000, 1 * 1000);
		for (i = 0; i < ARRAY_SIZE(fsa_reg_i2c_defaults); i++)
			regmap_write(mmax_priv->regmap, fsa_reg_i2c_defaults[i].reg,
					   fsa_reg_i2c_defaults[i].val);

		if (mmax_priv->is_mostest)
			regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x9f);

		break;
	default:
		break;
	}
}
/*
 * max20328_notify_mos
 * audio_v : archer add for voice mos test
*/
int max20328_notify_mos(int state)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	if (!mmax_priv)
		return -EINVAL;
	if (!mmax_priv->regmap)
		return -EINVAL;

	mmax_priv->is_mostest = state;

	max20328_update_reg_defaults(mmax_priv);

	return 0;
}
EXPORT_SYMBOL(max20328_notify_mos);

static ssize_t mos_show (struct kobject *kobj, struct kobj_attribute *attr, char *buffer)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	const int size = 600;
	int n = 0;

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: Invalid regmap.\n ", __func__);
		return -EFAULT;
	}

	n += scnprintf(buffer+n, size-n, "getprop vendor.audio.vivo.mos.test : %s\n", mmax_priv->is_mostest ? "true":"false");

	buffer[n] = 0;
	return n;
}

static ssize_t mos_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *ubuf, size_t cnt)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	unsigned int kbuf[2];
	char *temp;
	int ret = 0;

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: max20328 mmax_priv is null\n", __func__);
		return -EINVAL;
	}
	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: max20328 regmap is null\n", __func__);
		return -EINVAL;
	}

	temp = kmalloc(cnt, GFP_KERNEL);
	if (!temp) {
		return 0;
	}

	memcpy(temp, ubuf, cnt);
	ret = sscanf(temp, "%x", &kbuf[0]);
	if (!ret) {
		kfree(temp);
		return 0;
	}

	pr_info("%s: kbuf[0]: %x cnt: %d\n",
			__func__, kbuf[0], (int)cnt);

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		kfree(temp);
		return 0;
	}

	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: Invalid regmap.\n ", __func__);
		kfree(temp);
		return 0;
	}

	if (kbuf[0] == true) {
		mmax_priv->is_mostest = true;
	} else {
		mmax_priv->is_mostest = false;
	}
	max20328_update_reg_defaults(mmax_priv);

	kfree(temp);
	return cnt;
}

#if 1
static ssize_t max20328_reg_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *ubuf, size_t count)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	unsigned int kbuf[2];
	int reg_max = 0;
	char *temp;

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: max20328 mmax_priv is null\n", __func__);
		return -EINVAL;
	}
	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: max20328 regmap is null\n", __func__);
		return -EINVAL;
	}

	reg_max = (mmax_priv->usbc_ic_mode == MAX20328) ? sizeof(max20328_regs) : sizeof(fsa4480_regs);
	if (count > 32) {
		pr_err("%s: Invalid count (%d).\n", __func__, count);
		return -EINVAL;
	}

	temp = kmalloc(count, GFP_KERNEL);
	if (!temp) {
		pr_err("%s: Invalid data.\n", __func__);
		return -ENOMEM;
	}

	memcpy(temp, ubuf, count);
	sscanf(temp, "%x %x", &kbuf[0], &kbuf[1]);


	pr_info("%s: kbuf[0]: %x, kbuf[1]: %x cnt: %d\n",
		__func__, kbuf[0], kbuf[1], (int)count);

	if (IS_ERR_OR_NULL(mmax_priv)) {
		kfree(temp);
		pr_err("%s: mmax_priv is invalid.\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		kfree(temp);
		pr_err("%s: Invalid regmap.\n ", __func__);
		return -EFAULT;
	}

	if (kbuf[0] <= reg_max) {
		mutex_lock(&mmax_priv->usbc_switch_lock);
		regmap_write(mmax_priv->regmap, kbuf[0], kbuf[1]);
		mutex_unlock(&mmax_priv->usbc_switch_lock);
	} else {
		pr_err("%s: reg addr 0x%x out of range.\n", __func__, kbuf[0]);
	}

	kfree(temp);
	return count;
}


static ssize_t max20328_reg_show(struct kobject *kobj, struct kobj_attribute *attr, char *ubuf)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	const int size = 1024;
	int n = 0, i;
	unsigned int data;
	u8 *regs = max20328_regs;
	u8 regs_size = sizeof(max20328_regs);

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: max20328 mmax_priv is null\n", __func__);
		return -EINVAL;
	}
	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: max20328 regmap is null\n", __func__);
		return -EINVAL;
	}

	switch (mmax_priv->usbc_ic_mode) {
	case MAX20328:
		regs = max20328_regs;
		regs_size = sizeof(max20328_regs);
		break;
	case FSA4480:
		regs = fsa4480_regs;
		regs_size = sizeof(fsa4480_regs);
		break;
	default:
		break;
	}

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: Invalid regmap.\n ", __func__);
		return -EFAULT;
	}

	mutex_lock(&mmax_priv->usbc_switch_lock);
	for (i = 0; i < regs_size; i++) {
		regmap_read(mmax_priv->regmap, regs[i], &data);
		n += scnprintf(ubuf+n, size-n, "0x%02x: 0x%02x\n", regs[i], data);
		pr_info("%s: reg[0x%02x]: 0x%02x.\n", __func__, regs[i], data);
	}
	mutex_unlock(&mmax_priv->usbc_switch_lock);

	ubuf[n] = 0;

	return n;
}

static ssize_t max20328_i2c_show(struct kobject *kobj, struct kobj_attribute *attr, char *ubuf)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	struct i2c_client *i2c = NULL;
	const int size = 512;
	int n = 0, ret = 0, reg_01 = 0;

	pr_info("%s: i2c read enter.\n", __func__);

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(mmax_priv->dev)) {
		pr_err("%s: Invalid client.\n ", __func__);
		return -EFAULT;
	}

	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: Invalid regmap.\n ", __func__);
		return -EFAULT;
	}

	i2c = to_i2c_client(mmax_priv->dev);

	switch (mmax_priv->usbc_ic_mode) {
	case MAX20328:
		mutex_lock(&mmax_priv->usbc_switch_lock);
		ret = regmap_read(mmax_priv->regmap, MAX20328_DEVICE_ID, &reg_01);
		mutex_unlock(&mmax_priv->usbc_switch_lock);
		n += scnprintf(ubuf+n, size-n, "MAX20328-0x%x %s\n",
			i2c->addr, ((ret < 0) || !(reg_01 & 0x80)) ? "ERROR" : "OK");
		break;
	case FSA4480:
		mutex_lock(&mmax_priv->usbc_switch_lock);
		ret = regmap_read(mmax_priv->regmap, FSA4480_REG_DEVID, &reg_01);
		mutex_unlock(&mmax_priv->usbc_switch_lock);
		n += scnprintf(ubuf+n, size-n, "FSA4480-0x%x %s\n",
			i2c->addr, ((ret < 0) || !(reg_01 & 0x08)) ? "ERROR" : "OK");
		break;
	default:
		break;
	}

	ubuf[n] = 0;
	return n;
}

#define MAX20328_REG_FILE "/data/engineermode/max20328_reg"

static int max20328_reg_save(char *buffer, int count)
{
	struct file *pfile = NULL;
	mm_segment_t old_fs;
	int ret = 0;
	loff_t pos = 0;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	pfile = filp_open(MAX20328_REG_FILE, O_RDWR | O_CREAT, 0666);
	if (!IS_ERR(pfile)) {
		pr_info("%s: save buffer value %x %x %x\n", __func__,
			*buffer, *(buffer + sizeof(int)), *(buffer + 2 * sizeof(int)));
		vfs_write(pfile, buffer, (sizeof(int) * count), &pos);
		filp_close(pfile, NULL);
	} else {
		pr_err("%s: %s open failed!\n", __func__, MAX20328_REG_FILE);
		ret = -1;
	}

	set_fs(old_fs);
	return ret;
}

static int max20328_reg_get(char *buffer, int count)
{
	struct file *pfile = NULL;
	mm_segment_t old_fs;
	int ret = 0;
	loff_t pos = 0;

	*buffer = 0;
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	pfile = filp_open(MAX20328_REG_FILE, O_RDONLY, 0);
	if (!IS_ERR_OR_NULL(pfile)) {
		vfs_read(pfile, (char *)buffer, (sizeof(int) * count), &pos);
		pr_info("%s: get buffer value %x %x %x\n", __func__,
			*buffer, *(buffer + sizeof(int)), *(buffer + 2 * sizeof(int)));
		filp_close(pfile, NULL);
	} else {
		pr_err("%s: %s open failed!\n", __func__, MAX20328_REG_FILE);
		ret = -1;
	}

	set_fs(old_fs);
	return ret;
}

static ssize_t max20328_switch_show(struct kobject *kobj, struct kobj_attribute *attr, char *ubuf)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	struct i2c_client *i2c = NULL;
	const int size = 512;
	int n = 0, ret = 0, check_eb_err = 0, i = 0;
	int reg_06 = 0, reg_07 = 0, old_reg_eb[3] = {0}, reg_eb[3] = {0};

	pr_info("%s: i2c read enter.\n", __func__);

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(mmax_priv->dev)) {
		pr_err("%s: Invalid client.\n ", __func__);
		return -EFAULT;
	}

	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: Invalid regmap.\n ", __func__);
		return -EFAULT;
	}

	i2c = to_i2c_client(mmax_priv->dev);

	switch (mmax_priv->usbc_ic_mode) {
	case MAX20328:
		n += scnprintf(ubuf+n, size-n, "%s\n",
			i2c->addr, (ret < 0) ? "ERROR" : "OK");
		break;
	case FSA4480:
		mutex_lock(&mmax_priv->usbc_switch_lock);
		ret = regmap_read(mmax_priv->regmap, FSA4480_REG_CS_EB, &reg_eb[0]);
		ret = regmap_read(mmax_priv->regmap, FSA4480_REG_CS_EC, &reg_eb[1]);
		ret = regmap_read(mmax_priv->regmap, FSA4480_REG_CS_ED, &reg_eb[2]);
		ret = max20328_reg_get((char *)old_reg_eb, 3);
		if (ret < 0) {
			ret = max20328_reg_save((char *)reg_eb, 3);
			if (ret < 0) {
				check_eb_err = 1;
			}
		} else {
			for (i = 0; i < 3; i++) {
				if (reg_eb[i] != old_reg_eb[i]) {
					check_eb_err = 1;
					pr_info("%s: reg_eb[%d]=0x%02x old_reg_eb[%d]=0x%02x\n",
						__func__, i, reg_eb[i], i, old_reg_eb[i]);
				}
			}
		}
		pr_info("%s: check_eb_err %d\n", __func__, check_eb_err);
		ret = regmap_read(mmax_priv->regmap, FSA4480_REG_SW_STA0, &reg_06);
		ret = regmap_read(mmax_priv->regmap, FSA4480_REG_SW_STA1, &reg_07);
		pr_info("%s: FSA4480: SW_STA0 0x%x SW_STA1 0x%x\n", __func__, reg_06, reg_07);
		regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x9f);
		usleep_range(5 * 1000, 5 * 1000);
		ret = regmap_read(mmax_priv->regmap, FSA4480_REG_SW_STA0, &reg_06);
		ret = regmap_read(mmax_priv->regmap, FSA4480_REG_SW_STA1, &reg_07);
		pr_info("%s: FSA4480: SW_STA0 0x%x SW_STA1 0x%x\n", __func__, reg_06, reg_07);
		regmap_write(mmax_priv->regmap, FSA4480_REG_SW_EN, 0x98);
		mutex_unlock(&mmax_priv->usbc_switch_lock);
		n += scnprintf(ubuf+n, size-n, "%s reg_06 0x%02x reg_07 0x%02x reg_eb 0x%02x reg_ec 0x%02x reg_ed 0x%02x\n",
			((ret < 0) || (reg_06 != 0x15) || (reg_07 != 0x0a) || check_eb_err) ? "ERROR" : "OK",
			reg_06, reg_07, reg_eb[0], reg_eb[1], reg_eb[2]);
		break;
	default:
		break;
	}

	ubuf[n] = 0;
	return n;
}

static ssize_t max20328_repair_show(struct kobject *kobj, struct kobj_attribute *attr, char *ubuf)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	struct i2c_client *i2c = NULL;
	const int size = 512;
	int n = 0, ret = 0, repair_ok = 0, reg_ed = 0;

	pr_info("%s: i2c read enter.\n", __func__);

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(mmax_priv->dev)) {
		pr_err("%s: Invalid client.\n ", __func__);
		return -EFAULT;
	}

	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: Invalid regmap.\n ", __func__);
		return -EFAULT;
	}

	i2c = to_i2c_client(mmax_priv->dev);

	switch (mmax_priv->usbc_ic_mode) {
	case MAX20328:
		n += scnprintf(ubuf+n, size-n, "%s not support\n",
			!repair_ok ? "OK" : "ERROR");
		break;
	case FSA4480:
		repair_ok = max20328_ic_check_to_repair(mmax_priv);
		ret = regmap_read(mmax_priv->regmap, FSA4480_REG_CS_ED, &reg_ed);
		n += scnprintf(ubuf+n, size-n, "%s reg_ed 0x%02x\n",
			repair_ok ? "OK" : "ERROR", reg_ed);
		break;
	default:
		break;
	}

	ubuf[n] = 0;
	return n;
}

static ssize_t max20328_unuseirq_show(struct kobject *kobj, struct kobj_attribute *attr, char *ubuf)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	const int size = 512;
	int n = 0;

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return -EINVAL;
	}

	n += scnprintf(ubuf+n, size-n, "is_unuseirq: %d\n", mmax_priv->is_unuseirq);

	ubuf[n] = 0;
	return n;
}

static ssize_t max20328_unuseirq_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *ubuf, size_t cnt)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	unsigned int kbuf[2];
	char *temp;
	int ret = 0;

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return 0;
	}

	temp = kmalloc(cnt, GFP_KERNEL);
	if (IS_ERR_OR_NULL(temp)) {
		pr_err("%s: kmalloc failed.\n", __func__);
		return 0;
	}

	memcpy(temp, ubuf, cnt);
	ret = sscanf(temp, "%x", &kbuf[0]);
	if (!ret) {
		pr_err("%s: sscanf fail.\n", __func__);
		kfree(temp);
		return 0;
	}

	pr_info("%s: kbuf[0]: %x cnt: %d\n",
		__func__, kbuf[0], (int)cnt);

	mutex_lock(&mmax_priv->usbc_switch_lock);
	if (kbuf[0] == 1) {
		mmax_priv->is_unuseirq = 1;
		mmax_is_unuseirq = 1;
#ifdef CONFIG_VIVO_CHARGING_NEW_ARCH
		if (get_typec_drp())
			vote_typec_drp(ACCDET_TCPC_VOTER, false);

		vote_drp_enable(false);
#endif
	} else {
		mmax_priv->is_unuseirq = 0;
		mmax_is_unuseirq = 0;
#ifdef CONFIG_VIVO_CHARGING_NEW_ARCH
		vote_drp_enable(true);
#endif
	}
	mutex_unlock(&mmax_priv->usbc_switch_lock);

	kfree(temp);
	return cnt;
}


static struct kobj_attribute dev_attr_reg =
	__ATTR(reg, 0664, max20328_reg_show, max20328_reg_store);
static struct kobj_attribute dev_attr_i2c =
	__ATTR(i2c, 0664, max20328_i2c_show, NULL);
static struct kobj_attribute dev_attr_switch =
	__ATTR(switch, 0664, max20328_switch_show, NULL);
static struct kobj_attribute dev_attr_repair =
	__ATTR(repair, 0664, max20328_repair_show, NULL);
static struct kobj_attribute dev_attr_unuseirq =
	__ATTR(unuseirq, 0664, max20328_unuseirq_show, max20328_unuseirq_store);
static struct kobj_attribute dev_attr_mos =
	__ATTR(mos, 0664, mos_show, mos_store);

static struct attribute *sys_node_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_i2c.attr,
	&dev_attr_switch.attr,
	&dev_attr_repair.attr,
	&dev_attr_unuseirq.attr,
	&dev_attr_mos.attr,
	NULL
};


static struct attribute_group node_attribute_group = {
	.name = NULL,		/* put in device directory */
	.attrs = sys_node_attributes
};


static int class_attr_create(struct kobject *kobj)
{
	int ret = -1;
	char name[64];

	scnprintf(name, 48, "audio-max20328");

	kobj = kobject_create_and_add(name, kernel_kobj);
	if (!kobj) {
		pr_err("%s: kobject_create_and_add %s faild\n", __func__, name);
		return 0;
	}

	ret = sysfs_create_group(kobj, &node_attribute_group);
	if (ret) {
		kobject_del(kobj);
		kobj = NULL;
		pr_err("%s: sysfs_create_group %s faild\n", __func__, name);
	}

	pr_info("%s: sysfs create name successful\n", __func__, name);
	return ret;
}

static int class_attr_remove(struct kobject *kobj)
{
	if (kobj) {
		sysfs_remove_group(kobj, &node_attribute_group);
		kobject_del(kobj);
		kobj = NULL;
	}
	return 0;
}

#endif

static const struct i2c_device_id max20328b_i2c_id[] = {
	{ "max20328b", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max20328b_i2c_id);

static struct of_device_id max20328b_dt_match[] = {
	{ .compatible = "maxin,max20328",},
	{ },
};


static int max20328b_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct max20328_priv *mmax_priv;
	struct pinctrl *max_pinctrl;
	struct pinctrl_state *pins_rst;
	int err;
	int ret;

	mmax_priv = devm_kzalloc(&i2c->dev, sizeof(*mmax_priv),
				GFP_KERNEL);
	if (!mmax_priv)
		return -ENOMEM;


	mmax_priv->dev = &i2c->dev;
	mutex_init(&mmax_priv->usbc_switch_lock);

	usbc_switch_mmax_priv = mmax_priv;
	mmax_priv->is_unuseirq = 0;

	/*
	 * Create a simple kobject with the name of "kobject_example",
	 * located under /sys/kernel/
	 *
	 * As this is a simple directory, no uevent will be sent to
	 * userspace.  That is why this function should not be used for
	 * any type of dynamic kobjects, where the name and number are
	 * not known ahead of time.
	 */

	local_i2c = i2c;

	if (of_find_property(mmax_priv->dev->of_node, "max,usbc-ic-mode", NULL)) {
		err = of_property_read_u32(mmax_priv->dev->of_node,
					"max,usbc-ic-mode", &mmax_priv->usbc_ic_mode);
		if (err < 0) {
			pr_info("%s: read property failed %d\n", __func__, err);
		} else {
			pr_info("%s: usbc_ic_mode:%s\n", __func__, mmax_priv->usbc_ic_mode == MAX20328 ? "MAX20328" : "FSA4480");
		}
	}

	max_pinctrl = devm_pinctrl_get(&i2c->dev);
	if (IS_ERR(max_pinctrl)) {
		ret = PTR_ERR(max_pinctrl);
		dev_notice(&i2c->dev, "get max_pinctrl fail.\n");
	} else {
		pins_rst = pinctrl_lookup_state(max_pinctrl, "typec_rst_set_state");
		if (IS_ERR(pins_rst)) {
			ret = PTR_ERR(pins_rst);
			dev_notice(&i2c->dev, "lookup rst pinctrl fail\n");
		} else {
			pinctrl_select_state(max_pinctrl, pins_rst);
			dev_notice(&i2c->dev, "lookup rst pinctrl set to gpio input mode\n");
		}
	}

	mmax_priv->regmap = devm_regmap_init_i2c(i2c, &max20328_regmap_config);
	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: Failed to initialize regmap:\n", __func__);
		if (!mmax_priv->regmap) {
			err = -EINVAL;
			pr_err("%s: Failed to initialize regmap: %d\n", __func__, err);
		}
		err = PTR_ERR(mmax_priv->regmap);
		pr_err("%s: Failed to initialize regmap ptr err: %d\n", __func__, err);
	}

	max20328_update_reg_defaults(mmax_priv);

	class_attr_create(mmax_priv->kobj);
	atomic_set(&(mmax_priv->usbc_mode), 0);
	max20328_is_ready = 1;

	pr_info("%s Probe completed successfully! \n", __func__);

	return 0;

}

static int max20328b_i2c_remove(struct i2c_client *i2c)
{
	struct max20328_priv *mmax_priv = (struct max20328_priv *)i2c_get_clientdata(i2c);
	if (!mmax_priv)
		return -EINVAL;

	dev_dbg(mmax_priv->dev, "%s\n", __func__);

	class_attr_remove(mmax_priv->kobj);

	max20328_usbc_set_switch_mode(POWER_SUPPLY_TYPEC_NONE);

	kobject_put(usbc_switch_mmax_priv->switch_kobj);
	usbc_switch_mmax_priv = NULL;
	max20328_is_ready = 0;
	return 0;
}


static struct i2c_driver max20328_i2c_driver = {
	.driver = {
		.name = "max20328-driver",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(max20328b_dt_match),
	},
	.probe =    max20328b_i2c_probe,
	.remove =   max20328b_i2c_remove,
	.id_table = max20328b_i2c_id,
};

static int __init max20328b_i2c_init(void)
{
	int rc;

	rc = i2c_add_driver(&max20328_i2c_driver);
	if (rc)
		pr_err("max20328: Failed to register I2C driver: %d\n", rc);

	return rc;
}

module_init(max20328b_i2c_init);

static void __exit max20328b_i2c_exit(void)
{
	i2c_del_driver(&max20328_i2c_driver);
}

module_exit(max20328b_i2c_exit);

MODULE_DESCRIPTION("MAX20328 I2C driver");
MODULE_LICENSE("GPL v2");
