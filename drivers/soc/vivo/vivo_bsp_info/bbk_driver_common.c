// SPDX-License-Identifier: GPL-2.0
/*
 * bbk_driver_common.c
 *
 * Copyright (C) 2021 Vivo, Inc.
 * Author: wugaocheng <wugaocheng@vivo.com>
 *
 * Description: create sysfile node for board version and
 *              card detected.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/kobject.h>
#include <linux/string.h>

#define TAG "bbk_board_driver"
static ssize_t bbk_info_show(struct kobject *k, 
			struct attribute *attr, 
			char *buf)
{
	struct kobj_attribute *kobj_attr;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);
	if (kobj_attr->show)
		return kobj_attr->show(k, kobj_attr, buf);

	return -EIO;
}

static ssize_t bbk_info_store(struct kobject *k, 
			struct attribute *attr,
			const char *buf, size_t count)
{
	struct kobj_attribute *kobj_attr;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);
	if (kobj_attr->store)
		return kobj_attr->store(k, kobj_attr, buf, count);

	return -EIO;
}

extern struct kobj_attribute board_version;
extern struct kobj_attribute model_value;
extern struct kobj_attribute pcb_version;
extern struct kobj_attribute card_slot_inserted_check;
extern struct kobj_attribute card_slot_removed_check;
extern struct kobj_attribute card_slot_status_check;

static struct attribute *bbk_info_attrs[] = {
	/* board version */
	&board_version.attr,
	&model_value.attr,
	&pcb_version.attr,
	/* card detected */
	&card_slot_inserted_check.attr,
	&card_slot_removed_check.attr,
	&card_slot_status_check.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static const struct sysfs_ops bbk_info_sysfsops = {
	.show = bbk_info_show,
	.store = bbk_info_store,
};

static struct kobj_type bbk_info_ktype = {
	.sysfs_ops = &bbk_info_sysfsops,
	.default_attrs = bbk_info_attrs,
};
struct kobject bbk_info_kobj;

static int creat_sys_files(void)
{
	int ret;

	ret = kobject_init_and_add(&bbk_info_kobj, &bbk_info_ktype, NULL, "devs_list");
	if (ret) {
		pr_err(KERN_ERR "%s %s:Create kobjetct error!\n", TAG, __func__);
		return -EINVAL;
	}
	return 0;
}

extern struct platform_driver boards_version_driver;
extern struct platform_driver card_detect_driver;
static int __init bbk_info_driver_init(void)
{
	int ret = 0;

	ret = creat_sys_files();
	if (ret) {
		pr_err(KERN_ERR "%s %s:creat sysfs files failed!\n", TAG, __func__);
		return ret;
	}

	ret = platform_driver_register(&boards_version_driver);
	ret |= platform_driver_register(&card_detect_driver);

	if (ret) {
		kobject_put(&bbk_info_kobj);
		pr_err(KERN_ERR "%s %s:register failed!\n", TAG, __func__);
	}

	return ret;
}

static void __exit bbk_info_driver_exit(void)
{
	kobject_put(&bbk_info_kobj);
	platform_driver_unregister(&boards_version_driver);
	platform_driver_unregister(&card_detect_driver);
}

module_init(bbk_info_driver_init);
module_exit(bbk_info_driver_exit);

MODULE_AUTHOR("wugaocheng <wugaocheng@vivo.com>");
MODULE_DESCRIPTION("Hardware Boards Version & Card state detect");
MODULE_LICENSE("GPL v2");

