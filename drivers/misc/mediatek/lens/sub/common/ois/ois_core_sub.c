/*
 * Copyright (C) 2015 MediaTek Inc.
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

#include "ois_core_sub.h"

/*ois log define*/
int log_ois_level_sub;
//module_param(log_ois_level_sub, int, 0644);
EXPORT_SYMBOL_GPL(log_ois_level_sub);

struct ois *ois_instance_sub = NULL;
#ifdef CONFIG_MTK_CAM_PD2083F_EX
extern struct ois *ois_instance;
#endif
int ois_interface_create_sub(struct i2c_client *client, struct device *dev, const char *ois_name)
{
	int ret = 0;
	log_ois_level_sub = OIS_LOG_INFO;
	OIS_BUG(!client);
	OIS_BUG(!ois_name);
	LOG_OIS_INF("ois name = %s", ois_name);

	if (ois_instance_sub) {
		LOG_OIS_INF("ois create already done");
		goto p_err;
	}

	ois_instance_sub = (struct ois*)(kzalloc(sizeof(struct ois), GFP_KERNEL));
	if (!ois_instance_sub) {
		LOG_OIS_ERR("ois create fail");
		goto p_err;
	}

	mutex_init(&ois_instance_sub->op_lock);
	mutex_init(&ois_instance_sub->init_lock);
	ois_instance_sub->dev = dev;
	ois_instance_sub->slave_mode = 0x00;
	ois_instance_sub->dependency = NULL;
	ois_instance_sub->client = (struct i2c_client*)(kzalloc(sizeof(struct i2c_client), GFP_KERNEL));
	if (!ois_instance_sub->client) {
		LOG_OIS_ERR("ois client buffer alloc fail");
		goto p_err_client;
	}
	memcpy((void*)ois_instance_sub->client, (void*)client, sizeof(struct i2c_client));

	if (!strncmp(ois_name, "LC898129", 6)) {
		//dw9786_get_ops(ois_instance_sub);
		//lc898129_get_ops(ois_instance_sub);
#if defined(CONFIG_MTK_CAM_PD2083F_EX)
		lc898129_get_ops(ois_instance_sub);
#endif
	} else {
		LOG_OIS_ERR("unsupport ic type(%s)", ois_name);
		goto p_err_ops;
	}

	ois_instance_sub->flash_info = (struct ois_flash_info *)(kzalloc(sizeof(struct ois_flash_info), GFP_KERNEL));
	if (!ois_instance_sub->flash_info) {
		LOG_OIS_ERR("ois_status_check_buf kzalloc failed(%d)\n", ret);
		ois_instance_sub->flash_info = NULL;
		goto p_err_flash;
	}

	//alloc otp info buffer
	ois_instance_sub->otp_info = (struct ois_otp_info *)(kzalloc(sizeof(struct ois_otp_info), GFP_KERNEL));
	if (!ois_instance_sub->otp_info) {
		LOG_OIS_ERR("ois otp data kzalloc failed(%d)\n", ret);
		goto p_err_otp;
	}

	//alloc o+e info buffer
	ois_instance_sub->lens_info_buf = kzalloc(sizeof(struct ois_lens_info_buf), GFP_KERNEL);
	if (!ois_instance_sub->lens_info_buf) {
		LOG_OIS_ERR("alloc o2e buffer fail");
		goto p_err_o2e;
	};

	LOG_OIS_INF("ois(%s instance=%p client=%p lens_info=%p) create success",
		ois_name, ois_instance_sub, ois_instance_sub->client, ois_instance_sub->lens_info_buf);

	return ret;

p_err_o2e:
	if (ois_instance_sub->otp_info) {
		LOG_OIS_INF("release ois otp(%p)", ois_instance_sub->otp_info);
		kfree(ois_instance_sub->flash_info);
		ois_instance_sub->flash_info = NULL;
	}
p_err_otp:
	if (ois_instance_sub->flash_info) {
		LOG_OIS_INF("release ois flash(%p)", ois_instance_sub->flash_info);
		kfree(ois_instance_sub->flash_info);
		ois_instance_sub->flash_info = NULL;
	}
p_err_flash:
p_err_ops:
	if (ois_instance_sub->client) {
		LOG_OIS_INF("release ois client(%p)", ois_instance_sub->client);
		kfree(ois_instance_sub->client);
		ois_instance_sub->client = NULL;
	}
p_err_client:
	if (ois_instance_sub) {
		LOG_OIS_INF("client fail, release ois buffer(%p)", ois_instance_sub);
		kfree(ois_instance_sub);
		ois_instance_sub = NULL;
	}
p_err:
	return ret;
}

int ois_interface_destroy_sub()
{
	int ret = 0;

	if (ois_instance_sub) {
		if (ois_instance_sub->flash_info) {
			LOG_OIS_INF("release ois flash info buffer(%p)", ois_instance_sub->flash_info);
			kfree(ois_instance_sub->flash_info);
			ois_instance_sub->flash_info = NULL;
		}
		if (ois_instance_sub->lens_info_buf) {
			LOG_OIS_INF("release oe buffer(%p)", ois_instance_sub->lens_info_buf);
			kfree(ois_instance_sub->lens_info_buf);
			ois_instance_sub->lens_info_buf = NULL;
		}
		if (ois_instance_sub->client) {
			LOG_OIS_INF("release ois client buffer(%p)", ois_instance_sub->client);
			kfree(ois_instance_sub->client);
			ois_instance_sub->client = NULL;
		}

		LOG_OIS_INF("release ois(%p)", ois_instance_sub);
		kfree(ois_instance_sub);
		ois_instance_sub = NULL;
	} else {
		LOG_OIS_INF("ois been destroyed");
	}

	return ret;
}
#ifdef CONFIG_MTK_CAM_PD2083F_EX
static void ois_multiInstance_transaction_check()
{
	//common init
	ois_instance_sub->slave_mode = 0x00;
	ois_instance_sub->dependency = NULL;

	if ( NULL != ois_instance) {//dw already on as master,add dw operation as dependency of lc
		ois_instance->slave_mode = 0x01;
		ois_instance->dependency = NULL;
		ois_instance_sub->dependency = ois_instance;
	}

	LOG_OIS_INF("salve %d dependency %p", ois_instance_sub->slave_mode, ois_instance_sub->dependency);
}
#endif
static long ois_dispatcher_sub(struct ois *ois, unsigned int ioc_command, __user void *buf)
{
	long ret = 0;
	int mode = 0x0000;
	int log_level = OIS_LOG_INFO;
	struct ois_af_drift_param drift = {-1, 0, 0};

	OIS_BUG(!ois);
	OIS_BUG(!(ois->ops));

	switch (ioc_command) {
	case AFIOC_X_OIS_SETMODE: {
		OIS_BUG(!buf);
		ret = copy_from_user(&mode, buf, sizeof(int));
		if (ret) {
			LOG_OIS_ERR("copy mode fail(%d)", ret);
			goto p_err;
		}
		ret = CALL_OISOPS(ois, ois_set_mode, mode);
		break;
	}
	case AFIOC_X_OIS_SETACC: {
		ret = CALL_OISOPS(ois, ois_set_acc, buf);
		break;
	}
	case AFIOC_X_OIS_STATUSCHECK: {
		ret = CALL_OISOPS(ois, ois_status_check, buf);
		break;
	}
	case AFIOC_X_OIS_SETGYROGAIN: {
		ret = CALL_OISOPS(ois, ois_set_gyro_gain, buf);
		break;
	}
	case AFIOC_X_OIS_SETFIXMODE: {
		ret = CALL_OISOPS(ois, ois_set_target, buf);
		break;
	}
	case AFIOC_X_OIS_SETSINEMODE: {
		ret = CALL_OISOPS(ois, ois_set_sinewave, buf);
		break;
	}
	case AFIOC_X_OIS_SETCIRCLEMODE: {
		ret = CALL_OISOPS(ois, ois_set_circlewave, buf);
		break;
	}
	case AFIOC_X_OIS_SETSTROKELIMIT: {
		ret = CALL_OISOPS(ois, ois_set_stroke_limit, buf);
		break;
	}
	case AFIOC_X_OIS_SETPANTILT: {
		ret = CALL_OISOPS(ois, ois_set_pantilt, buf);
		break;
	}
	case AFIOC_X_OIS_GETMODE: {
		ret = CALL_OISOPS(ois, ois_get_mode, buf);
		break;
	}
	case AFIOC_X_OIS_GETINITINFO: {
		ret = CALL_OISOPS(ois, ois_get_init_info, buf);
		break;
	}
	case AFIOC_X_OIS_GETFWVERSION: {
		ret = CALL_OISOPS(ois, ois_get_fw_version, buf);
		break;
	}
	case AFIOC_X_OIS_GETGYROOFFSET: {
		ret = CALL_OISOPS(ois, ois_get_gyro_offset, buf);
		break;
	}
	case AFIOC_X_OIS_GETGYROGAIN: {
		ret = CALL_OISOPS(ois, ois_get_gyro_gain, buf);
		break;
	}
	case AFIOC_X_OIS_GETLENSINFO: {
		ret = CALL_OISOPS(ois, ois_get_lens_info, buf);
		break;
	}
	case AFIOC_X_OIS_GETOTPINFO: {
		ret = CALL_OISOPS(ois, ois_format_otp_data, buf);
		break;
	}
	case AFIOC_X_OIS_OFFSETCAL: {
		ret = CALL_OISOPS(ois, ois_set_offset_calibration);
		break;
	}
	case AFIOC_X_OIS_FWUPDATE: {
		ret = CALL_OISOPS(ois, ois_fw_update, buf);
		break;
	}
	case AFIOC_X_OIS_FLASHSAVE: {
		ret = CALL_OISOPS(ois, ois_flash_save);
		break;
	}
	case AFIOC_X_OIS_INIT: {
#ifdef CONFIG_MTK_CAM_PD2083F_EX
		ois_multiInstance_transaction_check();
#endif
		ret = CALL_OISOPS(ois, ois_init);
		ret = CALL_OISOPS(ois, ois_init_vsync_thread);
		break;
	}
	case AFIOC_X_OIS_DEINIT: {
		ret = CALL_OISOPS(ois, ois_deinit_vsync_thread);
		ret = CALL_OISOPS(ois, ois_deinit);
		break;
	}
	case AFIOC_X_OIS_STREAMON: {
		ret = CALL_OISOPS(ois, ois_stream_on);
		break;
	}
	case AFIOC_X_OIS_STREAMOFF: {
		ret = CALL_OISOPS(ois, ois_stream_off);
		break;
	}
	case AFIOC_X_OIS_SETTRIPOD: {
		ret = CALL_OISOPS(ois, ois_set_tripod, buf);
		break;
	}
	case AFICO_X_OIS_SETSMOOTH: {
		ret = CALL_OISOPS(ois, ois_set_smooth, buf);
		break;
	}
	case AFICO_X_OIS_SETAFDRIFT: {
		OIS_BUG(!buf);
		ret = copy_from_user(&drift, buf, sizeof(drift));
		if (ret) {
			LOG_OIS_ERR("copy drift fail(%d)", ret);
			goto p_err;
		}
		ret = CALL_OISOPS(ois, ois_af_crosstalk_compensation, &drift);
		break;
	}
#if defined(CONFIG_MTK_CAM_PD2072)
	case AFIOC_T_MOVETO:
	case AFIOC_T_SETINFPOS:
	case AFIOC_T_SETMACROPOS: {
		drift.currFocusDac = (int)buf;
		ret = CALL_OISOPS(ois, ois_af_crosstalk_compensation, &drift);
		break;
	}
#endif
	case AFICO_X_OIS_SETSERVO: {
		ret = CALL_OISOPS(ois, ois_set_servo, buf);
		break;
	}
	case AFICO_X_OIS_VSYNC: {
		ret = CALL_OISOPS(ois, ois_vsync_signal, buf);
		break;
	}
	case AFICO_X_OIS_LOG_LEVEL: {
		OIS_BUG(!buf);
		ret = copy_from_user(&log_level, buf, sizeof(int));
		if (ret) {
			LOG_OIS_ERR("copy log fail(%d)", ret);
			goto p_err;
		}
		ret = CALL_OISOPS(ois, ois_log_control, log_level);
		break;
	}
	case AFICO_X_OIS_DEPENDENCY_INIT: {
		ret = CALL_OISOPS(ois, ois_dependency_init);
		break;
	}
	case AFICO_X_OIS_ACTSTROKELIMIT:
	default:
		LOG_OIS_INF("unsupport command(%08x)", ioc_command);
		goto p_err;
	}
	//LOG_OIS_INF("execute ois command(%08x) result(%d)", ioc_command, ret);

p_err:
	return ret;
}

long ois_interface_dispatcher_sub(unsigned int ioc_command, void *buf)
{
	return ois_dispatcher_sub(ois_instance_sub, ioc_command, buf);
}