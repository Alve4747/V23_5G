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

#include "ois_core.h"

/*ois log define*/
int log_ois_level;
//module_param(log_ois_level, int, 0644);
EXPORT_SYMBOL_GPL(log_ois_level);
extern char *get_board_version(void);
struct ois *ois_instance = NULL;
#ifdef CONFIG_MTK_CAM_PD2083F_EX
extern struct ois *ois_instance_sub;
#endif

static void ois_dependency_init_thread(struct ois *ois)
{
	bool ret = 0;
	LOG_OIS_INF("E");
	ret = schedule_work(&ois->dependency_work);
	LOG_OIS_INF("X(%d)", ret);
}

void ois_dependency_init_work(struct work_struct *data)
{
	struct ois *ois;
	int ret = 0;

	LOG_OIS_INF("E");

	OIS_BUG_VOID(!data);

	ois = container_of(data, struct ois, dependency_work);

	OIS_BUG_VOID(!ois);

	ret = CALL_OISOPS(ois, ois_dependency_init);
	if (!ret)
		ois->dependency_ready = 1;
	else
		LOG_OIS_ERR("dependency ois not ready");

	LOG_OIS_INF("X(depednency %d ret %d)", ois->dependency_ready, ret);
}

#ifdef CONFIG_MTK_CAM_PD2083F_EX
static void ois_multiInstance_transaction_check()
{
	//common init
	ois_instance->slave_mode = 0x00;
	ois_instance->dependency = NULL;

	if (NULL != ois_instance_sub) {//lc already on as master, init dw as slave
		ois_instance ->slave_mode = 0x01;
	}

	LOG_OIS_INF("salve %d dependency %p", ois_instance->slave_mode, ois_instance->dependency);
}
#endif
static long ois_dispatcher(struct ois *ois, unsigned int ioc_command, __user void *buf)
{
	long ret = 0;
	int mode = 0x0000;
	int log_level = OIS_LOG_INFO;
	struct ois_af_drift_param drift = {-1, 0, 0};

	OIS_BUG(!ois);
	OIS_BUG(!(ois->ops));

	if (!ois->ready_check && AFICO_X_OIS_VSYNC == ioc_command) {
		LOG_OIS_INF("ois not ready(%d)!", ois->ready_check);
		goto p_err;
	}

	switch (ioc_command) {
	case AFIOC_X_OIS_SETMODE: {
		OIS_BUG(!buf);
		ret = copy_from_user(&mode, buf, sizeof(int));
		if (ret) {
			LOG_OIS_ERR("copy mode fail(%d)", ret);
			goto p_err;
		}
		ret = CALL_OISOPS(ois, ois_set_mode, mode, 0);
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
		mutex_lock(&ois->init_lock);
		flush_work(&ois->dependency_work);
#ifdef CONFIG_MTK_CAM_PD2083F_EX
		ois_multiInstance_transaction_check();
#endif
		ret = CALL_OISOPS(ois, ois_init);
		if (!ret) {
			ois->ready_check = 1;
			ret = CALL_OISOPS(ois, ois_init_vsync_thread);
		} else
			LOG_OIS_ERR("ois not ready, skip vsync process");
		mutex_unlock(&ois->init_lock);
		break;
	}
	case AFIOC_X_OIS_DEINIT: {
		mutex_lock(&ois_instance->init_lock);
		if (1 == ois->ready_check) {
			ret = CALL_OISOPS(ois, ois_deinit_vsync_thread);
			ois->ready_check = 0;
			ret = CALL_OISOPS(ois, ois_deinit);
		} else {
			LOG_OIS_INF("ois already deinit done");
		}
		mutex_unlock(&ois_instance->init_lock);
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
		ois_dependency_init_thread(ois);
		break;
	}
	case AFICO_X_OIS_ACTSTROKELIMIT: {
		ret = CALL_OISOPS(ois, ois_act_stroke_limit, buf);
		break;
	}
	default:
		//LOG_OIS_INF("unsupport command(%08x)", ioc_command);
		goto p_err;
	}
	//LOG_OIS_INF("execute ois command(%08x) result(%d)", ioc_command, ret);

p_err:
	return ret;
}

long ois_interface_dispatcher(unsigned int ioc_command, void *buf)
{
	return ois_dispatcher(ois_instance, ioc_command, buf);
}

/*************************************************************************
 * FUNCTION
 *	ois_interface_create
 *
 * DESCRIPTION
 *	create specific ois driver instance
 *
 * PARAMETERS
 *	*client : i2c client from sensor or af device
 *  *dev: device ptr from sensor or af
 *  *ois_name: specific ois driver we should create, for multi-instance use in future
 *  is_boot: 0 for normal process, 1 for boot process
 *
 * RETURNS
 *	0 for success, else value for fail
 *
 *
 *************************************************************************/
int ois_interface_create(struct i2c_client *client, struct device *dev, const char *ois_name, int is_boot, int sat_mode)
{
	int ret = 0;
	log_ois_level = OIS_LOG_INFO;

	OIS_BUG(!client);
	OIS_BUG(!ois_name);

	LOG_OIS_INF("E(%s)", ois_name);

	if (ois_instance) {
		LOG_OIS_INF("ois create already done");
		/*
		if (dev) {
			ois_instance->dev = dev;
			LOG_OIS_INF("update dev(%p)", dev);
		}
		*/
		goto p_err;
	}

	//alloc instance
	ois_instance = (struct ois *)(kzalloc(sizeof(struct ois), GFP_KERNEL));
	if (!ois_instance) {
		LOG_OIS_ERR("ois create fail");
		goto p_err;
	}

	mutex_init(&ois_instance->op_lock);
	mutex_init(&ois_instance->init_lock);
	ois_instance->dev = dev;
	ois_instance->ready_check = 0;
	ois_instance->dependency_ready = 0;
	ois_instance->sat_mode = sat_mode;
	ois_instance->slave_mode = 0x00;
	ois_instance->dependency = NULL;
	INIT_WORK(&ois_instance->dependency_work, ois_dependency_init_work);

	//alloc i2c client
	ois_instance->client = (struct i2c_client *)(kzalloc(sizeof(struct i2c_client), GFP_KERNEL));
	if (!ois_instance->client) {
		LOG_OIS_ERR("ois client buffer alloc fail");
		goto p_err_client;
	}
	memcpy((void *)ois_instance->client, (void *)client, sizeof(struct i2c_client));

	//get ops interfaces
	if (!strncmp(ois_name, "DW9781C", 7)) {
#if defined(CONFIG_MTK_CAM_PD2072) || defined(CONFIG_MTK_CAM_PD2083F_EX)
		dw9781c_get_ops(ois_instance);
#endif
	} else if (!strncmp(ois_name, "RUMBAS4SW", 9)) {
#if defined(CONFIG_MTK_CAM_PD2085) || defined(CONFIG_MTK_CAM_PD2120A) || defined(CONFIG_MTK_CAM_PD2133) || defined(CONFIG_MTK_CAM_PD2135)
		rumbas4sw_get_ops(ois_instance);
#endif
	} else {
		LOG_OIS_ERR("unsupport ic type(%s)", ois_name);
		goto p_err_ops;
	}

	//alloc ic flash info buffer
	ois_instance->flash_info = (struct ois_flash_info *)(kzalloc(sizeof(struct ois_flash_info), GFP_KERNEL));
	if (!ois_instance->flash_info) {
		LOG_OIS_ERR("ois_status_check_buf kzalloc failed(%d)\n", ret);
		ois_instance->flash_info = NULL;
		goto p_err_flash;
	}

	//alloc otp info buffer
	ois_instance->otp_info = (struct ois_otp_info *)(kzalloc(sizeof(struct ois_otp_info), GFP_KERNEL));
	if (!ois_instance->otp_info) {
		LOG_OIS_ERR("ois otp data kzalloc failed(%d)\n", ret);
		goto p_err_otp;
	}

	//alloc o+e buffer
	ois_instance->lens_info_buf = kzalloc(sizeof(struct ois_lens_info_buf), GFP_KERNEL);
	if (!ois_instance->lens_info_buf) {
		LOG_OIS_ERR("alloc o2e buffer fail");
		goto p_err_o2e;
	};

#if defined(CONFIG_MTK_CAM_PD2135)
	ois_instance->ccm_board_version = get_board_version();
	if (NULL != ois_instance->ccm_board_version) {
		LOG_OIS_INF("ccm_board_version = %s  ccm_board_version[0] = %c\n  ", ois_instance->ccm_board_version, ois_instance->ccm_board_version[0]);
	} else {
		LOG_OIS_INF("ccm_board_version is NULL\n");
	}
#endif

	LOG_OIS_INF("ois(%s instance=%p client=%p lens_info=%p sat=%d) create success",
		ois_name, ois_instance, ois_instance->client, ois_instance->lens_info_buf, ois_instance->sat_mode);

	//check if need to init dependent ic first(for gyro monitor)
	if (!is_boot)
		ois_dispatcher(ois_instance, AFICO_X_OIS_DEPENDENCY_INIT, NULL);
	else
		LOG_OIS_INF("boot process");

	return ret;
p_err_o2e:
	if (ois_instance->otp_info) {
		LOG_OIS_INF("release ois otp(%p)", ois_instance->otp_info);
		kfree(ois_instance->flash_info);
		ois_instance->flash_info = NULL;
	}
p_err_otp:
	if (ois_instance->flash_info) {
		LOG_OIS_INF("release ois flash(%p)", ois_instance->flash_info);
		kfree(ois_instance->flash_info);
		ois_instance->flash_info = NULL;
	}
p_err_flash:
p_err_ops:
	if (ois_instance->client) {
		LOG_OIS_INF("release ois client(%p)", ois_instance->client);
		kfree(ois_instance->client);
		ois_instance->client = NULL;
	}
p_err_client:
	if (ois_instance) {
		LOG_OIS_INF("release ois buffer(%p)", ois_instance);
		kfree(ois_instance);
		ois_instance = NULL;
	}
p_err:
	return ret;
}

int ois_interface_destroy(void)
{
	int ret = 0;

	LOG_OIS_INF("E");

	OIS_BUG(!ois_instance);

	flush_work(&ois_instance->dependency_work);
	mutex_lock(&ois_instance->init_lock);
	if (1 == ois_instance->ready_check) {//the abnormal case if camera did not call deinit before destroy
		ret = CALL_OISOPS(ois_instance, ois_deinit_vsync_thread);
		ois_instance->ready_check = 0;
		ret = CALL_OISOPS(ois_instance, ois_deinit);
	}
	else
		LOG_OIS_INF("ois already deinit done");
	mutex_unlock(&ois_instance->init_lock);

	if (ois_instance->lens_info_buf) {
		LOG_OIS_INF("release oe buffer(%p)", ois_instance->lens_info_buf);
		kfree(ois_instance->lens_info_buf);
		ois_instance->lens_info_buf = NULL;
	}

	if (ois_instance->otp_info) {
		LOG_OIS_INF("release ois otp buffer(%p)", ois_instance->otp_info);
		kfree(ois_instance->otp_info);
		ois_instance->otp_info = NULL;
	}

	if (ois_instance->flash_info) {
		LOG_OIS_INF("release ois flash info buffer(%p)", ois_instance->flash_info);
		kfree(ois_instance->flash_info);
		ois_instance->flash_info = NULL;
	}

	if (ois_instance->client) {
		LOG_OIS_INF("release ois client buffer(%p)", ois_instance->client);
		kfree(ois_instance->client);
		ois_instance->client = NULL;
	}

	LOG_OIS_INF("release ois(%p)", ois_instance);
	kfree(ois_instance);
	ois_instance = NULL;

	LOG_OIS_INF("X");

	return ret;
}
