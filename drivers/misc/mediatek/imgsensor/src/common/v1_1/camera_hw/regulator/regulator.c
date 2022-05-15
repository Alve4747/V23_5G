/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include "regulator.h"
#ifdef IMGSENSOR_OC_ENABLE
#include <mt-plat/aee.h>
#include <asm/siginfo.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include "upmu_common.h"
#endif


static const int regulator_voltage[] = {
	REGULATOR_VOLTAGE_0,
	REGULATOR_VOLTAGE_1000,
	REGULATOR_VOLTAGE_1050,
	REGULATOR_VOLTAGE_1100,
	REGULATOR_VOLTAGE_1180,
	REGULATOR_VOLTAGE_1200,
	REGULATOR_VOLTAGE_1210,
	REGULATOR_VOLTAGE_1220,
	REGULATOR_VOLTAGE_1500,
	REGULATOR_VOLTAGE_1800,
	REGULATOR_VOLTAGE_2500,
	REGULATOR_VOLTAGE_2800,
	REGULATOR_VOLTAGE_2900,
	REGULATOR_VOLTAGE_3100,
};

struct REGULATOR_CTRL regulator_control[REGULATOR_TYPE_MAX_NUM] = {
	{"vcama"},
	{"vcama_1p8"},
	{"vcamois"},
#if defined(CONFIG_MTK_CAM_PD2135)
	{"vcamois_dependency"},	/*vivo hope add for ois listening mode 2021.5.20*/
#else
	{"vmois"},
#endif
#ifdef CONFIG_REGULATOR_FAN53870
	{"plugic_en"},
#endif
#ifdef CONFIG_REGULATOR_RT5133
	{"vcama1"},
#endif
    {"vcamaf_def"},
	{"vcamd"},
	{"vcamio"},
};

#if defined(CONFIG_MTK_CAM_PD2163)
struct REGULATOR_CTRL regulator_control_b[REGULATOR_TYPE_MAX_NUM] = {
	{"vcama_b"},
	{"vcama_1p8_b"},
	{"vcamois_b"},
	{"vcamd_b"},
	{"vcamio_b"},
};
#endif
static struct REGULATOR reg_instance;

#ifdef IMGSENSOR_OC_ENABLE
static int vcamio_oc;
enum IMGSENSOR_RETURN imgsensor_oc_interrupt_enable(
	enum IMGSENSOR_SENSOR_IDX sensor_idx, bool enable)
{
	struct regulator *preg = NULL;
	struct device *pdevice =
		&gimgsensor.hw.common.pplatform_device->dev;

	gimgsensor.status.oc = 0;

	if (enable && !vcamio_oc) {
		mdelay(5);
		if (sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN ||
			sensor_idx == IMGSENSOR_SENSOR_IDX_SUB ||
			sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN2) {

			preg = regulator_get(pdevice, "vcamio");
			if (preg && regulator_is_enabled(preg)) {
				pmic_enable_interrupt(INT_VCAMIO_OC, 1,
						      "camera");
				vcamio_oc = 1;
				regulator_put(preg);
				pr_debug(
					"[regulator] %s INT_VCAMIO_OC %d, idx %d\n",
					__func__, enable, sensor_idx);
			} else
				pr_debug(
					"[regulator] %s fail to enable INT_VCAMIO_OC idx %d\n",
					__func__, sensor_idx);
		}
		rcu_read_lock();
		reg_instance.pid = current->tgid;
		rcu_read_unlock();
	} else if(!enable && vcamio_oc){	
		/* Disable interrupt before power off */
		if (sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN ||
			sensor_idx == IMGSENSOR_SENSOR_IDX_SUB ||
			sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN2) {
			pmic_enable_interrupt(INT_VCAMIO_OC, 0, "camera");
			reg_instance.pid = -1;
			vcamio_oc = 0;
			pr_debug(
				"[regulator] %s INT_VCAMIO_OC %d, idx %d\n",
				__func__, enable, sensor_idx);
		}
	}

	return IMGSENSOR_RETURN_SUCCESS;
}

#define OCP_DAEMON_RESTART_MAX_NUM 100 // form hardware design
static unsigned int camio_ocp_occur_count;
static void _vcamio_oc_handler(void)
{
	pr_debug("[regulator]%s enter vcamio oc %d\n",
		__func__,
		gimgsensor.status.oc);
	gimgsensor.status.oc = 1;
	vcamio_oc = 0;

   //����һ��count ����,��һ��kill������������Ҫ800ms�����ӣ�����kill�������ˣ����������ڻᵽ5s�����ӣ�����100�δ����5���ӵ�ʱ�䣬���������ô���������ΪӲ�����ˣ�ֱ��return  ��Ҫ�����������ܴ�camera��
   if (camio_ocp_occur_count >= OCP_DAEMON_RESTART_MAX_NUM) {
        pr_info("ocp daemon restart count is %u >= max num %u, just return\n",
                     camio_ocp_occur_count, OCP_DAEMON_RESTART_MAX_NUM);
        return;
   }

	aee_kernel_warning("Imgsensor vcamio OC", "Over current");
	if (reg_instance.pid != -1 && pid_task(find_get_pid(reg_instance.pid),PIDTYPE_PID) != NULL){
		force_sig(SIGKILL,
			pid_task(find_get_pid(reg_instance.pid), PIDTYPE_PID));
		reg_instance.pid = -1;
       camio_ocp_occur_count++;
	}
}

enum IMGSENSOR_RETURN imgsensor_oc_init(void)
{
	/* Register your interrupt handler of OC interrupt at first */
	/* pmic_register_interrupt_callback(INT_VCAMA_OC, imgsensor_oc_handler1); 
	pmic_register_interrupt_callback(INT_VCAMD_OC, imgsensor_oc_handler2); */
	pmic_register_interrupt_callback(INT_VCAMIO_OC, _vcamio_oc_handler);

	gimgsensor.status.oc  = 0;
	gimgsensor.imgsensor_oc_irq_enable = imgsensor_oc_interrupt_enable;
	reg_instance.pid = -1;
    camio_ocp_occur_count = 0;
    //vcamio_oc = 0;
	printk("[regulator]imgsensor_oc_init %d\n",vcamio_oc);
	
	return IMGSENSOR_RETURN_SUCCESS;
}
#endif

static enum IMGSENSOR_RETURN regulator_init(
	void *pinstance,
	struct IMGSENSOR_HW_DEVICE_COMMON *pcommon)
{
	struct REGULATOR *preg = (struct REGULATOR *)pinstance;
	int type, idx, ret = 0;
	char str_regulator_name[LENGTH_FOR_SNPRINTF];
#if (defined(CONFIG_MTK_CAM_PD2163))
	struct REGULATOR_CTRL *regulator_c = NULL;
	regulator_c = IS_ERR(regulator_get_optional(&pcommon->pplatform_device->dev, "cam0_vcama")) == 0 ? regulator_control:regulator_control_b;
	//pr_err("==hope cam0_vcama = %d", IS_ERR(regulator_get_optional(&pcommon->pplatform_device->dev, "cam0_vcama")));
#endif

	for (idx = IMGSENSOR_SENSOR_IDX_MIN_NUM;
		idx < IMGSENSOR_SENSOR_IDX_MAX_NUM;
		idx++) {
		for (type = 0;
			type < REGULATOR_TYPE_MAX_NUM;
			type++) {
			memset(str_regulator_name, 0,
				sizeof(str_regulator_name));
			ret = snprintf(str_regulator_name,
				sizeof(str_regulator_name),
				"cam%d_%s",
				idx,
#if (defined(CONFIG_MTK_CAM_PD2163))
				regulator_c[type].pregulator_type);
#else
				regulator_control[type].pregulator_type);
#endif
			if (ret < 0)
				return ret;
			preg->pregulator[idx][type] = regulator_get_optional(
					&pcommon->pplatform_device->dev,
					str_regulator_name);
			if (IS_ERR(preg->pregulator[idx][type])) {
				//add  use fan53870  boot up then  reboot by changcheng@vivo.com
				preg->pregulator[idx][type] = NULL;
				//add  use fan53870  boot up then  reboot by changcheng@vivo.com
				pr_err("ERROR: regulator[%d][%d]  %s fail!\n",
						idx, type, str_regulator_name);
			}
			atomic_set(&preg->enable_cnt[idx][type], 0);
		}
	}
#ifdef IMGSENSOR_OC_ENABLE
	imgsensor_oc_init();
#endif	
	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN regulator_release(void *pinstance)
{
	struct REGULATOR *preg = (struct REGULATOR *)pinstance;
	int type, idx;
	struct regulator *pregulator = NULL;
	atomic_t *enable_cnt = NULL;

	for (idx = IMGSENSOR_SENSOR_IDX_MIN_NUM;
		idx < IMGSENSOR_SENSOR_IDX_MAX_NUM;
		idx++) {

		for (type = 0; type < REGULATOR_TYPE_MAX_NUM; type++) {
			pregulator = preg->pregulator[idx][type];
			enable_cnt = &preg->enable_cnt[idx][type];
			if (pregulator != NULL) {
				for (; atomic_read(enable_cnt) > 0; ) {
					regulator_disable(pregulator);
					atomic_dec(enable_cnt);
				}
			}
		}
	}
	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN regulator_set(
	void *pinstance,
	enum IMGSENSOR_SENSOR_IDX   sensor_idx,
	enum IMGSENSOR_HW_PIN       pin,
	enum IMGSENSOR_HW_PIN_STATE pin_state)
{
	struct regulator     *pregulator;
	struct REGULATOR     *preg = (struct REGULATOR *)pinstance;
	int reg_type_offset;
	atomic_t             *enable_cnt;


	if (pin > IMGSENSOR_HW_PIN_DOVDD   ||
	    pin < IMGSENSOR_HW_PIN_AVDD    ||
	    pin_state < IMGSENSOR_HW_PIN_STATE_LEVEL_0 ||
	    pin_state >= IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH ||
	    sensor_idx < 0) {
		PK_PR_ERR("pin error ,out range");
		return IMGSENSOR_RETURN_ERROR;
		}

	reg_type_offset = REGULATOR_TYPE_VCAMA;

	pregulator = preg->pregulator[(unsigned int)sensor_idx][
		reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD];

	enable_cnt = &preg->enable_cnt[(unsigned int)sensor_idx][
		reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD];

	if (pregulator) {
		if (pin_state != IMGSENSOR_HW_PIN_STATE_LEVEL_0) {
			if (regulator_set_voltage(pregulator,
				regulator_voltage[
				pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0],
				regulator_voltage[
				pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0])) {

				PK_PR_ERR(
				  "[regulator]fail to regulator_set_voltage, powertype:%d powerId:%d\n",
				  pin,
				  regulator_voltage[
				  pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]);
			}
			if (regulator_enable(pregulator)) {
				PK_PR_ERR(
				"[regulator]fail to regulator_enable, powertype:%d powerId:%d\n",
				pin,
				regulator_voltage[
				  pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]);
				return IMGSENSOR_RETURN_ERROR;
			}
			atomic_inc(enable_cnt);
		} else {
			if (regulator_is_enabled(pregulator))
				PK_DBG("[regulator]%d is enabled\n", pin);
//vivo zahngxiaodong add for avdd grouned
#if defined(CONFIG_MTK_CAM_PD2159F_EX)
            else if(sensor_idx == 0 && pin == IMGSENSOR_HW_PIN_AVDD)
            {
                    gimgsensor.status.reserved = 1;
            }
#elif defined(CONFIG_MTK_CAM_PD2167F_EX)
      else if(sensor_idx == 1 && pin == IMGSENSOR_HW_PIN_AVDD)
            {
                    gimgsensor.status.reserved = 1;
            }

#endif
//vivo zahngxiaodong add for avdd grouned
			if (regulator_disable(pregulator)) {
				PK_PR_ERR(
					"[regulator]fail to regulator_disable, powertype: %d\n",
					pin);
				return IMGSENSOR_RETURN_ERROR;
			}
			atomic_dec(enable_cnt);
		}
	} else {
		PK_PR_ERR("regulator == NULL %d %d %d\n",
				reg_type_offset,
				pin,
				IMGSENSOR_HW_PIN_AVDD);
	}

	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN regulator_dump(void *pinstance)
{
	struct REGULATOR *preg = (struct REGULATOR *)pinstance;
	int i, j;
	int enable = 0;

	for (j = IMGSENSOR_SENSOR_IDX_MIN_NUM;
		j < IMGSENSOR_SENSOR_IDX_MAX_NUM;
		j++) {

		for (i = REGULATOR_TYPE_VCAMA;
		i < REGULATOR_TYPE_MAX_NUM;
		i++) {
			if (!preg->pregulator[j][i])
				continue;

			if (regulator_is_enabled(preg->pregulator[j][i]) &&
				atomic_read(&preg->enable_cnt[j][i]) != 0)
				enable = 1;
			else
				enable = 0;

			PK_DBG("[sensor_dump][regulator] index= %d, %s = %d, enable = %d\n",
				j,
				regulator_control[i].pregulator_type,
				regulator_get_voltage(preg->pregulator[j][i]),
				enable);
		}
	}
	return IMGSENSOR_RETURN_SUCCESS;
}

static struct IMGSENSOR_HW_DEVICE device = {
	.id        = IMGSENSOR_HW_ID_REGULATOR,
	.pinstance = (void *)&reg_instance,
	.init      = regulator_init,
	.set       = regulator_set,
	.release   = regulator_release,
	.dump      = regulator_dump
};

enum IMGSENSOR_RETURN imgsensor_hw_regulator_open(
	struct IMGSENSOR_HW_DEVICE **pdevice)
{
	*pdevice = &device;
	return IMGSENSOR_RETURN_SUCCESS;
}

