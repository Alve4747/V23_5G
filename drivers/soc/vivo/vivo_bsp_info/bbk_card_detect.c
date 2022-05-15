 // SPDX-License-Identifier: GPL-2.0
 /*
  * bbk_card_detect.c
  *
  * Copyright (C) 2021 Vivo, Inc.
  * Author: wugaocheng <wugaocheng@vivo.com>
  *
  * Description: detect SIM1 SIM2 TF-CARD inster remove state
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

#define HIGH_AS_INSERT 1
#define LOW_AS_INSERT 0

struct card_detect_data {
	struct kobject kobj;
	bool sim1_card_flag;
	bool sim2_card_flag;
//	bool tf_card_flag;

	unsigned int sim1_gpio;
	unsigned int sim2_gpio;
//	unsigned int tf_card_gpio;

	unsigned int sim1_insert_stat;
	unsigned int sim2_insert_stat;
//	unsigned int tf_card_default_stat;

};
static struct card_detect_data *cd_data;

static ssize_t card_slot_inserted_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	int gpio_state = -1;
	int count = 0;
	int flag = 0;

	if (cd_data->sim1_card_flag) {
		gpio_state = gpio_get_value(cd_data->sim1_gpio);
		if (gpio_state < 0) {
			count += snprintf(&buf[count], PAGE_SIZE, "sim1 gpio %d is invalid\n",
							cd_data->sim1_gpio);
			return -EIO;
		} else if (gpio_state != cd_data->sim1_insert_stat) {
			count += snprintf(&buf[count], PAGE_SIZE, "sim1 gpio %d is open drain\n",
							cd_data->sim1_gpio);
			flag = -1;
		}
	}

	if (cd_data->sim2_card_flag) {
		gpio_state = gpio_get_value(cd_data->sim2_gpio);
		if (gpio_state < 0) {
			count += snprintf(&buf[count], PAGE_SIZE, "sim2 gpio %d is invalid\n",
							cd_data->sim2_gpio);
			return -EIO;
		} else if (gpio_state != cd_data->sim2_insert_stat) {
			count += snprintf(&buf[count], PAGE_SIZE, "sim2 gpio %d is open drain\n",
							cd_data->sim2_gpio);
			flag = -1;
		}
	}

	if (flag == 0)
		count += snprintf(&buf[count], PAGE_SIZE, "OK\n");
	else
		count += snprintf(&buf[count], PAGE_SIZE, "ERROR\n");

	return count;
}

static ssize_t card_slot_removed_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	int gpio_state = -1;
	int count = 0;
	int flag = 0;

	if (cd_data->sim1_card_flag) {
		gpio_state = gpio_get_value(cd_data->sim1_gpio);
		if (gpio_state < 0) {
			count += snprintf(&buf[count], PAGE_SIZE, "sim1 gpio %d is invalid\n",
							cd_data->sim1_gpio);
			return count;
		} else if (gpio_state == cd_data->sim1_insert_stat) {
			count += snprintf(&buf[count], PAGE_SIZE, "sim1 gpio %d is short\n",
							cd_data->sim1_gpio);
			flag = -1;
		}
	}

	if (cd_data->sim2_card_flag) {
		gpio_state = gpio_get_value(cd_data->sim2_gpio);
		if (gpio_state < 0) {
			count += snprintf(&buf[count], PAGE_SIZE, "sim2 gpio %d is invalid\n",
							cd_data->sim2_gpio);
			return count;
		} else if (gpio_state == cd_data->sim2_insert_stat) {
			count += snprintf(&buf[count], PAGE_SIZE, "sim2 gpio %d is short\n",
							cd_data->sim2_gpio);
			flag = -1;
		}
	}

	if (flag == 0)
		count += snprintf(&buf[count], PAGE_SIZE, "OK\n");
	else
		count += snprintf(&buf[count], PAGE_SIZE, "ERROR\n");

	return count;
}

static ssize_t card_slot_status_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	int gpio_state = -1;
	int count = 0;

	if (cd_data->sim1_card_flag) {
		gpio_state = gpio_get_value(cd_data->sim1_gpio);
		if (gpio_state < 0) {
			count += snprintf(&buf[count], PAGE_SIZE, "sim1-gpio %d is invalid\n",
							cd_data->sim1_gpio);
			goto err;
		} else if (gpio_state == cd_data->sim1_insert_stat) {  /* short */
			count += snprintf(&buf[count], PAGE_SIZE, "GPIO%d-SIM1:1,",
							cd_data->sim1_gpio);
		} else if (gpio_state != cd_data->sim1_insert_stat) {  /*open drain */
			count += snprintf(&buf[count], PAGE_SIZE, "GPIO%d-SIM1:0,",
							cd_data->sim1_gpio);
		}
	}

	if (cd_data->sim2_card_flag) {
		gpio_state = gpio_get_value(cd_data->sim2_gpio);
		if (gpio_state < 0) {
			count += snprintf(&buf[count], PAGE_SIZE, "sim2-gpio %d is invalid\n",
							cd_data->sim2_gpio);
			goto err;
		} else if (gpio_state == cd_data->sim2_insert_stat) {
			count += snprintf(&buf[count], PAGE_SIZE, "GPIO%d-SIM2:1",
							cd_data->sim2_gpio);
		} else if (gpio_state != cd_data->sim2_insert_stat) {
			count += snprintf(&buf[count], PAGE_SIZE, "GPIO%d-SIM2:0",
							cd_data->sim2_gpio);
		}
	}

	return count;
err:
	return -EIO;
}

struct kobj_attribute card_slot_inserted_check =
		__ATTR(card_slot_inserted_check, S_IRUGO, card_slot_inserted_show, NULL);
struct kobj_attribute card_slot_removed_check =
		__ATTR(card_slot_removed_check, S_IRUGO, card_slot_removed_show, NULL);
struct kobj_attribute card_slot_status_check =
		__ATTR(card_slot_status_check, S_IRUGO, card_slot_status_show, NULL);

static int card_detect_parse_dt(struct device *dev, struct card_detect_data *cdata)
{
	struct device_node *np = dev->of_node;
	bool flag = false;
	unsigned  int gpio_flags;

	cdata->sim1_card_flag = of_property_read_bool(np, "vivo,sim1-need-detect");
	cdata->sim2_card_flag = of_property_read_bool(np, "vivo,sim2-need-detect");

	if (cdata->sim1_card_flag) {
		flag = of_property_read_bool(np, "vivo,sim1-detect-high");
		if (flag)
			cdata->sim1_insert_stat = HIGH_AS_INSERT;
		else
			cdata->sim1_insert_stat = LOW_AS_INSERT;

		cdata->sim1_gpio = of_get_named_gpio_flags(np,
				"card-detect-sim1,gpios", 0, &gpio_flags);
		if (!gpio_is_valid(cdata->sim1_gpio)) {
			dev_err(dev, "sim detect gpio %d is not valid\n", cdata->sim1_gpio);
			return -EINVAL;
		}
		dev_err(dev, "sim detect gpio %d\n", cdata->sim1_gpio);
	}

	if (cdata->sim2_card_flag) {
		flag = of_property_read_bool(np, "vivo,sim2-detect-high");
		if (flag)
			cdata->sim2_insert_stat = HIGH_AS_INSERT;
		else
			cdata->sim2_insert_stat = LOW_AS_INSERT;

		cdata->sim2_gpio = of_get_named_gpio_flags(np,
					"card-detect-sim2,gpios", 0, &gpio_flags);
		if (!gpio_is_valid(cdata->sim2_gpio)) {
			dev_err(dev, "sim detect gpio %d is not valid\n", cdata->sim1_gpio);
			return -EINVAL;
		}
		dev_err(dev, "sim detect gpio %d\n", cdata->sim2_gpio);
	}

	return 0;
}

static int card_detect_probe(struct platform_device *pdev)
{
	int ret = 0;

	cd_data = kzalloc(sizeof(struct card_detect_data), GFP_KERNEL);
	if (!cd_data) {
		dev_err(&pdev->dev, "cd_data kzalloc fail\n");
		return -ENOMEM;
	}

	ret = card_detect_parse_dt(&pdev->dev, cd_data);
	if (ret < 0) {
		dev_err(&pdev->dev, "card detect parse dt fail\n");
		goto free_pdata;
	}

	platform_set_drvdata(pdev, cd_data);

	return 0;

free_pdata:
	kfree(cd_data);
	return ret;
}

static int card_detect_remove(struct platform_device *pdev)
{
	struct card_detect_data *data = platform_get_drvdata(pdev);

	kfree(data);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id card_match_table[] = {
	{ .compatible = "card-detect",},
	{},
};
#endif

struct platform_driver card_detect_driver = {
	.probe      = card_detect_probe,
	.remove     = card_detect_remove,
	.driver     = {
		.name   = "card-detect",
		.owner  = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = card_match_table,
#endif
	},
};

