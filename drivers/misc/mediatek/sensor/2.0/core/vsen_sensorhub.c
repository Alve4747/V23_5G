/*
 * Copyright (C) 2019 VSEN SENSOR TEAM
 *
 */

#include <linux/kobject.h>
#include <linux/miscdevice.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include "../mtk_nanohub/mtk_nanohub.h"
#include "vsen_sensorhub.h"

//#define SENSOR_CMD_DUMP

#define VSEN_LOG_TAG              "[vsen_sensorhub] "
#define VSEN_ERR(fmt, args...)     pr_err(VSEN_LOG_TAG fmt, ##args)
#define VSEN_INFO(fmt, args...)    pr_info(VSEN_LOG_TAG fmt, ##args)
#define VSEN_DEBUG(fmt, args...)   pr_debug(VSEN_LOG_TAG fmt, ##args)

static DEFINE_MUTEX(ioctrl_mutex);

extern int vsen_nanohub_selftest(int sensor_type);

static void vsen_sensorhub_dump_request(struct SCP_SENSOR_HUB_VSEN_CMD_REQ *req)
{
#ifdef SENSOR_CMD_DUMP
	VSEN_INFO("dump_cmd sensor_type %d cmd 0x%x\n", req->sensorType, req->cmd);
	VSEN_INFO("dump_cmd data: %d %d %d %d %d %d %d %d %d\n",
				req->data[0], req->data[1], req->data[2],
				req->data[3], req->data[4], req->data[5],
				req->data[6], req->data[7], req->data[8]);
#endif
	return;
}

static void vsen_sensorhub_init_request(struct SCP_SENSOR_HUB_VSEN_CMD_REQ *req)
{
	memset(req, 0x00, sizeof(*req));
}

extern int mtk_nanohub_req_send(union SCP_SENSOR_HUB_DATA *data);
static int vsen_sensorhub_send_request(struct SCP_SENSOR_HUB_VSEN_CMD_REQ *req)
{
	int err = 0;

	err = mtk_nanohub_req_send((union SCP_SENSOR_HUB_DATA *)req);
	if (err)
		VSEN_INFO("mtk_nanohub_req_send %d\n", err);
	vsen_sensorhub_dump_request(req);

	return err;
}

static int vsen_sensorhub_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static ssize_t vsen_sensorhub_write(struct file *file, const char *buffer,
				size_t length, loff_t *offset)
{
	return 0;
}

static int vsen_sensorhub_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long vsen_sensorhub_unlocked_ioctl(struct file *file, unsigned int cmd,
					 unsigned long arg)
{
	void __user *ptr = (void __user *)arg;
	int user_param[VSEN_USER_SPACE_CMD_DATA_SIZE] = {0};
	int ret = 0;
	struct SCP_SENSOR_HUB_VSEN_CMD_REQ req;

	mutex_lock(&ioctrl_mutex);
	switch (cmd) {
	case VSEN_SENSOR_HUB_SENSOR_CMD:
		if (copy_from_user(&user_param, ptr, sizeof(user_param))) {
			ret = -EFAULT;
			goto exit_hub_ioctrl;
		}
		VSEN_INFO("VSEN_SENSOR_HUB_SENSOR_CMD begin %d 0x%x 0x%x\n", user_param[0], user_param[1], user_param[2]);

		vsen_sensorhub_init_request(&req);
		req.sensorType = (uint8_t)(user_param[0]&0xFF);
		req.action = SENSOR_HUB_VSEN_CMD;
		req.cmd =  (uint32_t)(user_param[1]&0xFFFF);

		if ((req.cmd == SENSOR_COMMAND_ACC_SELF_TEST) ||
			(req.cmd == SENSOR_COMMAND_GYRO_SELF_TEST) ||
			(req.cmd == SENSOR_COMMAND_MAG_SELF_TEST)) {
			user_param[2] = vsen_nanohub_selftest(req.sensorType);
			VSEN_ERR("do selftest sensor:%d result:%d\n", req.sensorType, user_param[2]);
		} else {
			memcpy(&(req.data), &(user_param[2]), sizeof(req.data));
			ret = vsen_sensorhub_send_request(&req);
			if (ret < 0) {
				VSEN_ERR("VSEN_SENSOR_HUB_SENSOR_CMD ipc fail (%d) %d 0x%x %d %d %d\n", ret,
						user_param[0], user_param[1], user_param[2],
						user_param[3], user_param[4]);
				goto exit_hub_ioctrl;
			}

			if ((req.sensorType == (uint8_t)(user_param[0]&0xFF)) && (req.cmd == (uint32_t)(user_param[1]&0xFFFF))) {
				memcpy(&(user_param[2]), &(req.data), sizeof(req.data));
			} else {
				VSEN_ERR("req error: sensorType %d (%d) cmd:0x%x (0x%x)\n", req.sensorType, user_param[0], req.cmd, user_param[1]);
				ret = -EFAULT;
				goto exit_hub_ioctrl;
			}
		}

		if (copy_to_user(ptr, &user_param, sizeof(user_param))) {
			ret = -EFAULT;
			goto exit_hub_ioctrl;
		}
		VSEN_INFO("VSEN_SENSOR_HUB_SENSOR_CMD end %d 0x%x %d %d %d\n", user_param[0], user_param[1],
									user_param[2], user_param[3], user_param[4]);
		break;

	default:
		VSEN_ERR("unknown IOCTL: 0x%08x\n", cmd);
		ret = -ENOIOCTLCMD;
		break;
	}

exit_hub_ioctrl:
	mutex_unlock(&ioctrl_mutex);
	return ret;
}

#ifdef CONFIG_COMPAT
static long vsen_sensorhub_compat_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	return 0;
}
#endif

static const struct file_operations _vsen_sensorhub_fops = {
	.open = vsen_sensorhub_open,
	.write = vsen_sensorhub_write,
	.release = vsen_sensorhub_release,
	.unlocked_ioctl = vsen_sensorhub_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = vsen_sensorhub_compat_ioctl,
#endif
};

static struct miscdevice vsen_sensorhub_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "vivo_sensorhub",
	.fops = &_vsen_sensorhub_fops,
};

static int __init vsen_sensorhub_init(void)
{
	int ret = 0;

	ret = misc_register(&vsen_sensorhub_device);
	if (ret) {
		VSEN_ERR("register misc failed\n");
		ret = -1;
	}
	VSEN_INFO("misc %d\n", ret);

	return ret;
}

static void __exit vsen_sensorhub_cleanup(void)
{
	misc_deregister(&vsen_sensorhub_device);
}

module_init(vsen_sensorhub_init);
module_exit(vsen_sensorhub_cleanup);

MODULE_AUTHOR("Yang Ruibin@VSEN SENSOR TEAM");
MODULE_LICENSE("GPL");
