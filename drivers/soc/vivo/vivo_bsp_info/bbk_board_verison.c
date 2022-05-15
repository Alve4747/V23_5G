// SPDX-License-Identifier: GPL-2.0
/*
 * bbk_boards_version.c
 *
 * Copyright (C) 2021 Vivo, Inc.
 * Author: wugaocheng <wugaocheng@vivo.com>
 *
 * Description: cali the hardware board version and show it
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
//#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/kobject.h>
#include <linux/delay.h>

#include "../drivers/pinctrl/mediatek/pinctrl-mtk-common-v2.h"
#include "../drivers/pinctrl/core.h"
#include "../drivers/gpio/gpiolib.h"

#define ID_GPIO_MAX_LEN    20
#define PCB_GPIO_MAX_LEN   4
#define PCB_MAP_MAX_LEN    32	/* 2^(PCB_GPIO_MAX_LEN+1) */
#define PCB_VER_MAX_LEN    8	/* EX: MP_0.1 */

struct boards_version_data {
	unsigned int gpio_nums;
	unsigned int pcb_gpio_nums;
	unsigned int pcb_maps_nums;
	unsigned int gpios[ID_GPIO_MAX_LEN];
	unsigned int pcb_gpios[PCB_GPIO_MAX_LEN];
	unsigned int pcb_version_map[PCB_MAP_MAX_LEN];
	char board_version[ID_GPIO_MAX_LEN];
	char pcb_version[PCB_VER_MAX_LEN];
	char model_value[ID_GPIO_MAX_LEN*11+20];
};

static struct boards_version_data *bv_data;

#include <linux/bbk_drivers_info.h>
char *get_board_version(void)
{
	return bv_data?bv_data->board_version:NULL;
}
EXPORT_SYMBOL_GPL(get_board_version);

static ssize_t board_version_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, bv_data->board_version);
}

static ssize_t model_value_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, bv_data->model_value);
}

static ssize_t pcb_version_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, bv_data->pcb_version);
}

struct kobj_attribute board_version =
		__ATTR(board_version, S_IRUGO, board_version_show, NULL);
struct kobj_attribute model_value =
		__ATTR(model_value, S_IRUGO, model_value_show, NULL);
struct kobj_attribute pcb_version =
		__ATTR(pcb_version, S_IRUGO, pcb_version_show, NULL);

static int boards_version_parse_dt(struct device *dev, 
				struct boards_version_data *bvdata)
{
	struct device_node *np = dev->of_node;

	if (of_property_read_u32(np, "gpio_nums", &bvdata->gpio_nums)) {
		dev_err(dev, "boards version fdt gpio_nums get failed\n");
		return -EFAULT;
	}

	if (0 == bvdata->gpio_nums || bvdata->gpio_nums > ID_GPIO_MAX_LEN) {
		dev_err(dev, "boards version gpio nums(%d) is not valid\n",
							bvdata->gpio_nums);
		return -EINVAL;
	}

	if (of_property_read_u32_array(np, "gpios", bvdata->gpios, bvdata->gpio_nums)) {
		dev_err(dev, "boards version fdt gpios get failed\n");
		return -EFAULT;
	}

	/* pcb version */
	if (of_property_read_u32(np, "pcb_gpio_nums", &bvdata->pcb_gpio_nums)) {
		dev_err(dev, "boards pcb_gpio nums get failed\n");
		bvdata->pcb_gpio_nums = 0;
	}

	if (0 == bvdata->pcb_gpio_nums || bvdata->pcb_gpio_nums > PCB_GPIO_MAX_LEN) {
		dev_info(dev, "boards pcb_version is used default\n");
		return 0;
	}

	if (of_property_read_u32_array(np, "pcb_gpios", bvdata->pcb_gpios, 
							bvdata->pcb_gpio_nums)) {
		dev_err(dev, "boards pcb gpios get fail\n");
		return -EFAULT;
	}

	if (of_property_read_u32(np, "pcb_maps_nums", 
							&bvdata->pcb_maps_nums)) {
		dev_err(dev, "boards pcb_maps nums get failed\n");
		return -EFAULT;
	}

	if (0 == bvdata->pcb_maps_nums || bvdata->pcb_maps_nums > PCB_MAP_MAX_LEN) {
		dev_err(dev, "boards pcb_maps nums(%d) is not valid\n",
							bvdata->pcb_maps_nums);
		return -EINVAL;
	}

	if (of_property_read_u32_array(np, "pcb_gpios_maps", bvdata->pcb_version_map, 
							bvdata->pcb_maps_nums)) {
		dev_err(dev, "boards pcb version map get fail\n");
		return -EFAULT;
	}

	return 0;
}

static void boards_version_set(struct device *dev, 
				struct boards_version_data *bvdata)
{
	struct pinctrl_state *state = NULL;
	struct pinctrl       *bv_pinctl = dev->pins->p;
	struct pinctrl_setting *setting = NULL;
	struct mtk_pinctrl          *hw = NULL;
	const struct mtk_pin_desc *desc = NULL;
	struct gpio_desc     *gpio_desc = NULL;
	int ret = 0;
	int gpio_val = 0;
	unsigned int count = 0;
	unsigned int i = 0;

	if (NULL == bv_pinctl || NULL == bv_pinctl->state) {
		dev_err(dev, "no found bv pinctrl\n");
		return;
	}

	state = bv_pinctl->state;
	list_for_each_entry(setting, &(state->settings), node) {
		if (setting->type == PIN_MAP_TYPE_CONFIGS_GROUP)
			break;
	}

	if (!setting || !(setting->pctldev)) {
		dev_err(dev, "can not find pinctl node\n");
		return;
	}

	hw = pinctrl_dev_get_drvdata(setting->pctldev);
	for (i = 0; i < bvdata->gpio_nums; i++) {
		// enable pull up
		desc = (const struct mtk_pin_desc *)&hw->soc->pins[bvdata->gpios[i]];
		ret |= hw->soc->bias_set_combo(hw, desc, 1, 1);
		mdelay(1);
		gpio_desc = gpiochip_get_desc(&hw->chip, bvdata->gpios[i]);
		gpio_val = gpio_get_value(desc_to_gpio(gpio_desc));
		bvdata->board_version[i] = gpio_val + '0';

		// enable pull down
		if (0 == gpio_val)
			ret |= hw->soc->bias_set_combo(hw, desc, 0, 1);

		count += sprintf(&bvdata->model_value[count], "GPIO-%d-%c,", 
						bvdata->gpios[i], bvdata->board_version[i]);
	}
	bvdata->board_version[bvdata->gpio_nums] = '\0';
	/* bvdata->model_value[count-1] is ',' so must set to '\0'  */
	bvdata->model_value[count-1] = '\0';

	count = 0;
	if (0 == bvdata->pcb_gpio_nums) {
		count += snprintf(bvdata->pcb_version, PCB_VER_MAX_LEN, "MP_0.1");
	} else {
		int indx = 0;

		for (i = 0; i < bvdata->pcb_gpio_nums; i++) {
			// enable pull up
			desc = (const struct mtk_pin_desc *)&hw->soc->pins[bvdata->pcb_gpios[i]];
			ret |= hw->soc->bias_set_combo(hw, desc, 1, 1);
			mdelay(1);
			gpio_desc = gpiochip_get_desc(&hw->chip, bvdata->pcb_gpios[i]);
			gpio_val = gpio_get_value(desc_to_gpio(gpio_desc));
			indx += gpio_val;

			// enable pull down
			if (0 == gpio_val)
				ret |= hw->soc->bias_set_combo(hw, desc, 0, 1);
		}

		for (i = 0; i < bvdata->pcb_maps_nums; i += 2) {
			if (bvdata->pcb_version_map[i] == indx) {
				count += snprintf(bvdata->pcb_version, PCB_VER_MAX_LEN,
						"MP_0.%d",
						bvdata->pcb_version_map[i+1]);
				break;
			}
		}

		if (i >= bvdata->pcb_maps_nums)
			count += snprintf(bvdata->pcb_version, PCB_VER_MAX_LEN, "MP_0.1");
	}
	bvdata->pcb_version[count] = '\0';

	if (ret)
		dev_err(dev, "read gpio failed\n");
}

static int boards_version_probe(struct platform_device *pdev)
{
	int ret = 0;

	bv_data = kzalloc(sizeof(struct boards_version_data), GFP_KERNEL);
	if (!bv_data) {
		dev_err(&pdev->dev, "bv_data kzalloc fail\n");
		return -ENOMEM;
	}

	ret = boards_version_parse_dt(&pdev->dev, bv_data);
	if (ret < 0) {
		dev_err(&pdev->dev, "boards version parse dt fail\n");
		goto free_pdata;
	}

	boards_version_set(&pdev->dev, bv_data);
	platform_set_drvdata(pdev, bv_data);

	return 0;

free_pdata:
	kfree(bv_data);
	return ret;
}

static int boards_version_remove(struct platform_device *pdev)
{
	struct boards_version_data *data = platform_get_drvdata(pdev);

	kfree(data);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id board_match_table[] = {
	{ .compatible = "board-version",},
	{},
};
#endif

struct platform_driver boards_version_driver = {
	.probe      = boards_version_probe,
	.remove     = boards_version_remove,
	.driver     = {
		.name   = "board-version",
		.owner  = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = board_match_table,
#endif
	},
};

