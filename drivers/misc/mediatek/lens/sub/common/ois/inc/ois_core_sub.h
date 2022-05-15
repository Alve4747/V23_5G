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


#ifndef OIS_VIVO_H
#define OIS_VIVO_H

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/platform_data/nanohub.h>
#include <linux/kthread.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <asm-generic/int-ll64.h>
#include <linux/of_platform.h>
#include <linux/firmware.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/completion.h>
#include <uapi/linux/sched/types.h>
#include "lens_info.h"

/*common define*/
#define LENS_INFO_FRAMES_MAX 8
#define LENS_INFO_GROUPS_MAX 11

#define OIS_BUG(condition) \
	{ \
		if (unlikely(condition)) { \
			pr_info("[BUG][%s] %s:%d(%s)\n", __FILE__, __func__, __LINE__, #condition); \
			return -EINVAL; \
		} \
	}
#define OIS_BUG_VOID(condition)								\
	{											\
		if (unlikely(condition)) {							\
			pr_info("[BUG][%s] %s:%d(%s)\n", __FILE__, __func__, __LINE__, #condition);\
			return;									\
		}										\
	}
#define CALL_OISOPS(s, op, args...)                                         \
	(((s)->ops->op) ? ((s)->ops->op((s), ##args)) : 0)

#define OIS_BIG_ENDIAN_TRANS2(data, offset) \
	((data[offset] << 8) | (data[offset+1]))

#define OIS_BIG_ENDIAN_TRANS4(data, offset) \
	((data[offset] << 24) | (data[offset+1] << 16) | (data[offset+2] << 8) | data[offset+3])

#define OIS_LITTLE_ENDIAN_TRANS2(data, offset) \
		((data[offset]) | (data[offset+1] << 8))

#define OIS_LITTLE_ENDIAN_TRANS4(data, offset) \
		((data[offset]) | (data[offset+1] << 8) | (data[offset+2] << 16) | (data[offset+3] << 24))

//ois log
extern int log_ois_level;
#define LOG_OIS_ERR(format, args...) \
	do { \
		pr_err("[%s][%d] " format, __func__, __LINE__, ##args); \
	} while (0)

#define LOG_OIS_INF(format, args...) \
	do { \
		if (log_ois_level > OIS_LOG_ERROR) \
			pr_info("[%s][%d] " format, __func__, __LINE__, ##args); \
	} while (0)

#define LOG_OIS_VERB(format, args...) \
	do { \
		if (log_ois_level > OIS_LOG_INFO) \
			pr_info("[%s][%d] " format, __func__, __LINE__, ##args); \
	} while (0)
#define I2COP_CHECK(cond) \
{ \
	if (cond) { \
		LOG_OIS_ERR("ois i2c fail(%d)", cond); \
		goto p_err; \
	} \
}

/*struct define*/
struct ois_lens_info {
	u64 fcount;       //vsync countr fom kernel
	u64 frame_id;     //frame_id from hal
	u64 timestampboot;
	u8  validnum;
	u16 ic_timecount[LENS_INFO_GROUPS_MAX];
	s16 hallx[LENS_INFO_GROUPS_MAX];
	s16 hally[LENS_INFO_GROUPS_MAX];
};

struct ois_lens_info_buf {
	u8                   insertidx;
	struct ois_lens_info buf[LENS_INFO_FRAMES_MAX];
};

struct ois_flash_addr_table {
	u16 servoon_ctrl;
	u16 mode_ctrl;
	u16 acc_ctrl;
	u16 acc_gainx;
	u16 acc_gainy;
	u16 acc_bindwidth;
	u16 acc_polarity;
	u16 acc_odr;
	u16 acc_on;
	u16 acc_full_scale_select;
	u16 gyro_gainx;
	u16 gyro_gainy;
	u16 gyro_offsetx;
	u16 gyro_offsety;
	u16 gyro_rawx;
	u16 gyro_rawy;
	u16 gyro_odr;
	u16 gyro_polarity;
	u16 gyro_bindwidth;
	u16 gyro_full_scale_select;  //gyro range
	u16 gyro_temp;               //gyro temperature
	u16 gyro_on;
	u16 target_posx;
	u16 target_posy;
	u16 hall_polarity;
	u16 hall_offsetx;
	u16 hal_offsety;
	u16 hall_gainx;
	u16 hal_gainy;
	u16 hallx;
	u16 hally;
	u16 acc_limitx;
	u16 acc_limity;
	u16 gyro_limitx;
	u16 gyro_limity;
	u16 total_limitx;
	u16 total_limity;
	u16 loop_gainx;
	u16 loop_gainy;
	u16 p_gainx;
	u16 p_gainy;
	u16 i_gainx;
	u16 i_gainy;
	u16 d_gainx;
	u16 d_gainy;
};
struct ois_otp_addr_table {
	u16 start_addr;  //start addr of eeprom data
	u16 end_addr;
	u16 sn_code_start;
	u16 sn_code_end;
	u16 fuse_id_start;
	u16 fuse_id_end;
	u16 flag;
	u16 fw_version;
	u16 hall_polarity;
	u16 hall_maxx;
	u16 hall_maxy;
	u16 hall_minx;
	u16 hall_miny;
	u16 hall_mech_centerx;
	u16 hall_mech_centery;
	u16 hall_gainx;
	u16 hall_gainy;
	u16 hall_offsetx;
	u16 hall_offsety;
	u16 gyro_polarityx;
	u16 gyro_polarityy;
	u16 gyro_orient;
	u16 gyro_offsetx;
	u16 gyro_offsety;
	u16 gyro_gainx;
	u16 gyro_gainy;
	u16 loop_gainx;
	u16 loop_gainy;
	u16 tilt_srx;
	u16 tilt_sry;
	u16 shift_srx;
	u16 shift_sry;
	u16 check_sum;
};

struct ois_otp_info {
	int fwVersion;
	int tiltSRX;
	int tiltSRY;
	int accSRX;
	int accSRY;
	int gyroOffsetX;
	int gyroOffsetY;
	int hallGainX;
	int hallGainY;
	int hallOffsetX;
	int hallOffsetY;
	int hallMechCenterX;
	int hallMechCenterY;
	int hallXMax;
	int hallXMin;
	int hallYMax;
	int hallYMin;
	int gyroGainX;
	int gyroGainY;
	int loopGainX;
	int loopGainY;
	int icType;
	int gyroPolarX;
	int gyroPolarY;
	int hallPolarity;
	int gyroOrient;
	int accMirror;
	int OISName;
	int inited;
};

struct ois_fw_info {
	int version;
	int type;
	int date;
};

struct ois_imu_info {
	int imuType;
	int imuReadEn;
	int spiMode;
	int gyroOffsetX;
	int gyroOffsetY;
	int accGainX;
	int accGainY;
	int gyroGainX;
	int gyroGainY;
	int accFineGainX;
	int accFineGainY;
	int gyroRawX;
	int gyroRawY;
	int gyroRawZ;
	int accRawX;
	int accRawY;
	int accRawZ;
};

struct ois_target_info     {
	int gyroTargetX;
	int gyroTargetY;
	int accTargetX;
	int accTargetY;
	int totalTargetX;
	int totalTargetY;
};

struct ois_hall_info {
	int pantiltOn;
	int lensPosX;
	int lensPosY;
	int gyroLimitX;
	int gyroLimitY;
	int accLimitX;
	int accLimitY;
	int totalLimitX;
	int totalLimitY;
	int totalLimitCircle;
};

struct ois_smooth_info {
	int on;
	int step;
	int delay;
};

struct ois_flash_info {
	int chipEn;
	int dspEn;
	int serveOn;
	int logicReset;
	int mode;
	int writeAuthority;
	int accOn;
	int status;
	int tripodFlag;
	int driftCompOn;
	struct ois_fw_info fwInfo;
	struct ois_imu_info imuInfo;
	struct ois_target_info targetInfo;
	struct ois_hall_info hallInfo;
	struct ois_smooth_info smoothInfo;
	struct ois_otp_info caliInfo;
};

struct ois_circlemode_parameter
{
	s16 threshhold;
	s16 circleNum;
	s16 errCountLimit;
	s16 circleFrequency;
	s16 sampleFrequency;
	s16 circleSkipNum;
	s16 startPosition;
	s16 angleLimit;
	s16 amplitudeX;
	s16 amplitudeY;
	int checkResult;
};

struct ois_fixmode_parameter {
	s8  mannualEnable;
	int targetX;
	int targetY;
	int hallX;
	int hallY;
};

struct ois_sinemode_parameter {
	int axis;       //1 for x;2 for y; 3 for x&y
	int frequency;
	int amplitudeX;
	int amplitudeY;
};

struct ois_stroke_limit_parameter {
	int enable;
	int axisX;
	int axisY;
	int circle;
};

struct ois_acc_param {
	int  accOn;
	int  supportLaser;
	int  preFocusDistance;
	int  currFocusDistance;
	int  currFocusDistanceF;
	int  engineerMode;
};

struct ois_af_drift_param {
	int  driftCompOn;  //AF drift compensation
	int  preFocusDac;
	int  currFocusDac;
};

struct ois_pantilt_param {
	int on;
	int startX;
	int startY;
	int limitY;
	int limitX;
	int limitCircle;
	int step;
};

/**
 * module's interrupt , each module should have its own isr.
 * note:
 * mapping to ISP_IRQ_TYPE_ENUM
 */
enum ois_vsync_idx {
	OIS_VSYNC_TYPE_INT_CAM_A_ST,
	OIS_VSYNC_TYPE_INT_CAM_B_ST,
	OIS_VSYNC_TYPE_INT_CAM_C_ST,
	OIS_VSYNC_TYPE_INT_CAMSV_0_ST,
	OIS_VSYNC_TYPE_INT_CAMSV_1_ST,
	OIS_VSYNC_TYPE_INT_CAMSV_2_ST,
	OIS_VSYNC_TYPE_INT_CAMSV_3_ST,
	OIS_VSYNC_TYPE_INT_CAMSV_4_ST,
	OIS_VSYNC_TYPE_INT_CAMSV_5_ST,
	OIS_VSYNC_TYPE_INT_CAMSV_6_ST,
	OIS_VSYNC_TYPE_INT_CAMSV_7_ST,
	OIS_VSYNC_TYPE_AMOUNT
};

struct ois_vsync_info {
	u64 module_idx;
	u64 vsync_cnt ;
	u64 sof_timestamp_boot ;
	u64 ois_vsync_cnt;
};

struct ois {
	u32                          mode;
	bool                         inited;
	struct i2c_client            *client;
	struct device                *dev;
	const struct ois_core_ops    *ops;
	struct mutex                 init_lock;
	struct mutex                 op_lock;
	struct ois_lens_info_buf     *lens_info_buf;
	spinlock_t                   ois_vsync_lock;
	struct task_struct           *vsync_task;
	struct kthread_worker        vsync_worker;
	struct kthread_work          vsync_work;
	struct ois_flash_info        *flash_info;
	struct ois_otp_info          *otp_info;
	int                          max_fps;
	struct ois_vsync_info        vsync_info;
	char                         *ccm_board_version;
	int                          ready_check;
	int                          dependency_ready;
	struct work_struct           dependency_work;//add for 5x init
	u32                          sat_mode;
	int                          slave_mode; //0 master,1 slave,tutorial info for current ois instance
	void*                        dependency; //pre ois instance which current ois instance dependent on
};

/*ois mode definition of hal should match with this*/
enum OIS_MODE {
	OIS_MODE_START      = -1,
	OIS_CENTER_MODE     = 0,
	OIS_STILL_MODE      = 1,
	OIS_VIDEO_MODE      = 2,
	OIS_ZOOM_MODE       = 3,
	OIS_FIX_MODE        = 4,
	OIS_SINEWAVE_MODE   = 5,
	OIS_CIRCLEWAVE_MODE = 6,
	OIS_ACTIVE_CEN_ON   = 7,
	OIS_ACTIVE_CEN_OFF  = 8,
	OIS_MODE_END        = 9,
};

enum is_ois_type {
	DW9781C = 10,
	RUMBAS4SW,
	DW9786,
	LC898129,
};

enum OIS_LOG_LEVEL {
	OIS_LOG_START = 0,
	OIS_LOG_ERROR = 1,
	OIS_LOG_INFO  = 2,
	OIS_LOG_VERB  = 3,
	OIS_LOG_END   = 4,
};

enum SAT_MODE {
	SAT_DISABLE = 0,
	SAT_ENABLE = 1,
	EG_MODE  = 2,
};

//Quad-phase switch for ois common feature control
enum ois_switch_type {
    DEFAULT_OFF = 0,
    DEFAULT_ON = 1,
    MANUAL_OFF = 2,
    MANUAL_ON = 3
};

enum target_vertex {
	VERTEX_DEFAULT  = 0,
	LEFTTOP         = 1,
	LEFTBOTTOM      = 2,
	RIGHTTOP        = 3,
	RIGHTBOTTOM     = 4,
	VERTEX_END      = 5,
};


/* ois module external interfaces define*/
int ois_interface_create_sub(struct i2c_client *client, struct device *pdev, const char *ois_name);
int ois_interface_destroy_sub(void);
long ois_interface_dispatcher_sub(unsigned int ioc_command, __user void *buf);

/*ois util fucs define*/
int ois_i2c_write_32_sub(struct i2c_client *client, u16 addr, u32 data);
int ois_i2c_write_32_sub(struct i2c_client *client, u16 addr, u32 data);
int ois_i2c_write_block_sub(struct i2c_client *client, u16 addr, const u8 *data, size_t size);
int ois_i2c_read_32_sub(struct i2c_client *client, u16 addr, u32 *data);
int ois_i2c_read_sub(struct i2c_client *client, u16 addr, u16 *data);
int ois_i2c_read_block_sub(struct i2c_client *client, u16 addr, u8 *data, size_t size);


/*specific ois ic interface expose to ois_core*/
void lc898129_get_ops(struct ois *ois);
//void dw9786_get_ops(struct ois *ois);

/*ois internal operation funcs define*/
struct ois_core_ops {
	int (*ois_init)(struct ois *ois);
	int (*ois_deinit)(struct ois *ois);
	int (*ois_get_mode)(struct ois *ois, void __user *user_buf);
	//is_internal:0 indicate generic ois mode, 1 indicate ois internal mode
	int (*ois_set_mode)(struct ois *ois, int mode);
	int (*ois_stream_on)(struct ois *ois);
	int (*ois_stream_off)(struct ois *ois);
	int (*ois_fw_update)(struct ois *ois, void __user *user_buf);
	int (*ois_get_fw_version)(struct ois *ois, void __user *user_buf);
	int (*ois_get_gyro_offset)(struct ois *ois, void __user *user_buf);
	int (*ois_set_offset_calibration)(struct ois *ois);
	int (*ois_get_gyro_gain)(struct ois *ois, __user void *user_buf);
	int (*ois_set_gyro_gain)(struct ois *ois, __user void* user_buf);
	int (*ois_flash_save)(struct ois *ois);
	int (*ois_set_acc)(struct ois *ois, void __user *user_buf);
	int (*ois_set_target)(struct ois *ois, void __user *user_buf);
	int (*ois_get_init_info)(struct ois *ois, void __user *user_buf);
	int (*ois_status_check)(struct ois *ois, void __user *user_buf);
	int (*ois_init_vsync_thread)(struct ois *ois);
	int (*ois_deinit_vsync_thread)(struct ois *ois);
	int (*ois_vsync_signal)(struct ois *ois, void *buf);
	int (*ois_get_lens_info)(struct ois *ois, void __user *user_buf);
	int (*ois_format_otp_data)(struct ois *ois, void __user *user_buf);
	int (*ois_set_sinewave)(struct ois *ois, void __user *user_buf);
	int (*ois_set_stroke_limit)(struct ois *ois, void __user *user_buf);
	int (*ois_set_pantilt)(struct ois *ois, void __user *user_buf);
	int (*ois_reset)(struct ois *ois);
	int (*ois_set_smooth)(struct ois *ois, __user void *user_buf);
	int (*ois_set_tripod)(struct ois *ois, __user void *user_buf);
	int (*ois_set_circlewave)(struct ois *ois, void __user *user_buf);
    int (*ois_af_crosstalk_compensation)(struct ois *ois, struct ois_af_drift_param *drift);
	int (*ois_set_servo)(struct ois *ois, __user void *user_buf);
	int (*ois_log_control)(struct ois *ois, int level);
	int (*ois_dependency_init)(struct ois *ois);
};

#endif /* OIS_SUB_VIVO_H */
