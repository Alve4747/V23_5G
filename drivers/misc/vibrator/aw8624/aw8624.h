#ifndef _AW8624_H_
#define _AW8624_H_

/*********************************************************
*
* aw8624.h
*
 ********************************************************/
#include <linux/regmap.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/version.h>
#include <sound/control.h>
#include <sound/soc.h>
#include <linux/leds.h>
#include "../lra-wakeup.h"
#include "../vivo_haptic_core.h"

#define AW8624_CHIPID                   0x24

#define AW8624_VBAT_REFER                   4200
#define AW8624_VBAT_MIN                     3000
#define AW8624_VBAT_MAX                     4500
#define HAPTIC_BATTERY_VOLTAGE              4000

#define AW8624_HAPTIC_F0_COEFF              260     //2.604167

#define SEQ_WAIT_UNIT    10000 //us
#define AW8624_DUOBLE_CLICK_DELTA      30000 //us

#define AW8624_SEQUENCER_SIZE          8

enum aw8624_haptic_ram_vbat_comp_mode {
	AW8624_HAPTIC_RAM_VBAT_COMP_DISABLE = 0,
	AW8624_HAPTIC_RAM_VBAT_COMP_ENABLE = 1,
};

enum aw8624_haptic_work_mode {
	AW8624_HAPTIC_STANDBY_MODE = 0,
	AW8624_HAPTIC_RAM_MODE = 1,
	AW8624_HAPTIC_RTP_MODE = 2,
	AW8624_HAPTIC_TRIG_MODE = 3,
	AW8624_HAPTIC_CONT_MODE = 4,
	AW8624_HAPTIC_RAM_LOOP_MODE = 5,
};


enum aw8624_haptic_clock_type {
	AW8624_HAPTIC_CLOCK_CALI_F0 = 0,
	AW8624_HAPTIC_CLOCK_CALI_OSC_STANDARD = 1,

};

enum aw8624_haptic_pwm_mode {
	AW8624_PWM_48K = 0,
	AW8624_PWM_24K = 1,
	AW8624_PWM_12K = 2,
};
	
enum aw8624_haptic_vbat_comp_mode {
	AW8624_HAPTIC_VBAT_SW_COMP_MODE = 0,
	AW8624_HAPTIC_VBAT_HW_COMP_MODE = 1,
};


enum aw8624_haptic_activate_mode {
  AW8624_HAPTIC_ACTIVATE_RAM_MODE = 0,
  AW8624_HAPTIC_ACTIVATE_CONT_MODE = 1,
  AW8624_HAPTIC_ACTIVATE_RTP_MODE = 2,
  AW8624_HAPTIC_ACTIVATE_RAM_LOOP_MODE = 3,
};

enum aw8624_haptic_cali_lra {
	AW8624_HAPTIC_ZERO = 0,
	AW8624_HAPTIC_F0_CALI_LRA = 1,
	AW8624_HAPTIC_RTP_CALI_LRA = 2,
};

enum haptics_custom_effect_param {
	CUSTOM_DATA_EFFECT_IDX,
	CUSTOM_DATA_TIMEOUT_SEC_IDX,
	CUSTOM_DATA_TIMEOUT_MSEC_IDX,
	CUSTOM_DATA_LEN,
};

enum play_type {
	RAM_TYPE = 0,
	RTP_TYPE = 1,
	TIME_TYPE = 2,
	CONT_TYPE = 3,
	RTP_MMAP_TYPE = 4,
};

enum aw8624_haptic_f0_flag {
	AW8624_HAPTIC_LRA_F0 = 0,
	AW8624_HAPTIC_CALI_F0 = 1,
};


//effect scene struct
struct scene_effect_info {
	u16		scene_id;
	u16		effect_id;
	u16		real_vmax;
};

struct ram {
	unsigned int len;
	unsigned int check_sum;
	unsigned int base_addr;
	unsigned char version;
	unsigned char ram_shift;
	unsigned char baseaddr_shift;
};

struct aw8624_play_info {
	enum play_type type;
	u32 ram_id;//用于ram模式
	u32 vmax;
	u32 times_ms;//用于返回ram和rtp模式波形时长
	u32 playLength;//用于时长播放
	char rtp_file[128];//用于rtp模式
};


//数据类型用u32，与dts的数据类型保持一致
struct haptic_wavefrom_info {
	u32 idx;
	u32 ram_id;
	u32 vmax;
	u32 times_ms;
	bool rtp_enable;
	const char *rtp_file_name;
};

struct lra_info_for_cali {
	u32 AW8624_HAPTIC_F0_PRE;  // 170Hz
	u32 AW8624_HAPTIC_F0_CALI_PERCEN;      // -7%~7%
	u32 AW8624_HAPTIC_CONT_DRV_LVL;   // 71*6.1/256=1.69v
	u32 AW8624_HAPTIC_CONT_DRV_LVL_OV;    // 125*6.1/256=2.98v
	u32 AW8624_HAPTIC_CONT_TD;
	u32 AW8624_HAPTIC_CONT_ZC_THR;
	u32 AW8624_HAPTIC_CONT_NUM_BRK;
	u32 AW8624_HAPTIC_RATED_VOLTAGE; //mv-Vp
 };
 
struct aw8624_container{
	unsigned int len;
	unsigned char data[];
};

#define AW8624_LRA_0619                    619
#define AW8624_LRA_0832                    832
#define AW8624_LRA_1040                    1040
#define AW8624_LRA_0815                    815


struct aw8624 {
	struct i2c_client *i2c;
	struct device *dev;
	struct led_classdev cdev;
	struct haptic_misc *hm;

	struct mutex lock;
	struct mutex rtp_lock;

	struct hrtimer timer;
	struct work_struct vibrator_work;
	struct work_struct rtp_work;
	struct delayed_work ram_work;
	struct delayed_work stop_work;

	struct ram ram;

	struct aw8624_play_info play;
	unsigned int rtp_cnt;
	unsigned int rtp_file_num;

	unsigned char chipid;
	unsigned char chipid_flag;
	unsigned char play_mode;
	unsigned char activate_mode;

	int state;
	int duration;
	int amplitude;
	int vmax;
	int gain;
	int f0_value;

	volatile unsigned int osc_cali_data; //标准的osc时钟频率偏移，osc校准获取
	volatile unsigned int f0_cali_data; //校准f0时获取的时钟频率偏移

	unsigned int f0;
	unsigned int f0_pre;
	unsigned int cont_f0;
	unsigned int cont_td;
	unsigned int cont_zc_thr;
	unsigned char cont_drv_lvl;
	unsigned char cont_drv_lvl_ov;
	unsigned char cont_num_brk;
	unsigned char max_pos_beme;
	unsigned char max_neg_beme;
	unsigned char f0_cali_flag;

	unsigned char ram_vbat_comp;
	unsigned int lra;

	struct mutex bus_lock;
	struct mutex rtp_check_lock;

	struct wakeup_source *wklock;

	// Play and stop time interval
	ktime_t begin;
	ktime_t cancel;

// DTS Information
	int reset_gpio;
	int irq_gpio;
	u32 resistance_min;
	u32 resistance_max;
	u32 freq_min;
	u32 freq_max;

	u32 lra_information;  //板子马达型号
	struct lra_info_for_cali lra_info; //板子马达校准配置
	int cali_f0;

	bool no_trigger;
	bool add_suffix;
	bool rtp_mmap_page_alloc_flag; // true : allco sucess, false: alloc failed
	volatile unsigned char rtp_init;
	unsigned char ram_init;

	// dts effect
	int effects_count;
	struct haptic_wavefrom_info *effect_list;
	int default_vmax;

	struct scene_effect_info *base_scene_list;
	int base_scene_count;
	struct scene_effect_info *ext_scene_list;
	int ext_scene_count;
};

#endif
