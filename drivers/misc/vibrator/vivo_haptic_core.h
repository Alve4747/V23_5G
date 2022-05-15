#ifndef __HAPTIC_CORE_H__
#define __HAPTIC_CORE_H__

/* only used for vivo haptic core */
#define HAPTIC_CORE_IOCTL_MAGIC           'v'

#define HAPTIC_CORE_GET_VIB_COUNT         _IOR(HAPTIC_CORE_IOCTL_MAGIC, 11, int*)
/*****************************************************/

#define HAPTIC_IOCTL_MAGIC                'h'

#define HAPTIC_UPLOAD                     _IOWR(HAPTIC_IOCTL_MAGIC, 11, struct haptic_effect*)
#define HAPTIC_PLAYBACK                   _IOWR(HAPTIC_IOCTL_MAGIC, 12, int)
#define HAPTIC_STOP                       _IOWR(HAPTIC_IOCTL_MAGIC, 13, int)
#define HAPTIC_GAIN                       _IOWR(HAPTIC_IOCTL_MAGIC, 14, int)
#define HAPTIC_TRIGGER_INTENSITY          _IOW(HAPTIC_IOCTL_MAGIC, 15, int)
#define HAPTIC_JUDGE_EFFECT_SUPPORT       _IOWR(HAPTIC_IOCTL_MAGIC, 17, int*)
#define HAPTIC_GET_MOTOR_TYPE             _IOWR(HAPTIC_IOCTL_MAGIC, 18, int*)
#define HAPTIC_GET_F0                     _IOWR(HAPTIC_IOCTL_MAGIC, 19, int*) // 线性马达的实际f0，而非理论值
#define HAPTIC_DISPATCH_CALI_PARAM        _IOWR(HAPTIC_IOCTL_MAGIC, 20, struct haptic_cali_param *)
#define HAPTIC_GET_DRIVER_IC_TYPE         _IOWR(HAPTIC_IOCTL_MAGIC, 21, enum ic_type *)

#define HAPTIC_SUPPORT_BITMASK            _IOWR(HAPTIC_IOCTL_MAGIC, 16, unsigned char *)

/* only used for aac richtap begin */

#define RICHTAP_IOCTL_GROUP 0x52
#define RICHTAP_GET_HWINFO          _IO(RICHTAP_IOCTL_GROUP, 0x03)
#define RICHTAP_SET_FREQ            _IO(RICHTAP_IOCTL_GROUP, 0x04)
#define RICHTAP_SETTING_GAIN        _IO(RICHTAP_IOCTL_GROUP, 0x05)
#define RICHTAP_OFF_MODE            _IO(RICHTAP_IOCTL_GROUP, 0x06)
#define RICHTAP_TIMEOUT_MODE        _IO(RICHTAP_IOCTL_GROUP, 0x07)
#define RICHTAP_RAM_MODE            _IO(RICHTAP_IOCTL_GROUP, 0x08)
#define RICHTAP_RTP_MODE            _IO(RICHTAP_IOCTL_GROUP, 0x09)
#define RICHTAP_STREAM_MODE         _IO(RICHTAP_IOCTL_GROUP, 0x0A)
#define RICHTAP_UPDATE_RAM          _IO(RICHTAP_IOCTL_GROUP, 0x10)
#define RICHTAP_GET_F0              _IO(RICHTAP_IOCTL_GROUP, 0x11)
#define RICHTAP_STOP_MODE           _IO(RICHTAP_IOCTL_GROUP, 0x12)

/* only used for aac richtap end */


/* support bit mask*/
#define HAPTIC_MASK_BIT_SUPPORT_EFFECT    0x00
#define HAPTIC_MASK_BIT_SUPPORT_GAIN      0x01
#define HAPTIC_MASK_BIT_TRIGGER_INTENSITY 0x07
#define HAPTIC_MASK_BIT_AAC_RICHTAP       0x06

#define HAPTIC_CNT                        0x08

enum haptic_effect_type {
	HAPTIC_CONSTANT,
	HAPTIC_CUSTOM,
	HAPTIC_RTP_STREAM,
};

enum haptic_write_event_type {
	HAPTIC_PLAY_EVENT,
	HAPTIC_GAIN_EVENT,
};

struct haptic_write_event {
	__s16           type;
	__s16           code;
	__s32           value;
};

struct custom_fifo_data {
	uint32_t   	    effect_id;
	uint32_t        length;
	uint32_t        play_rate_hz;
	const int8_t    *data;
};

struct haptic_effect {
	enum haptic_effect_type      type;
	__s16                 id; // caller identifier
	__s16                 magnitude;

	__s16                 length; // only for constant mode
	__u32                 data_count; // only for rtp stream mode
	__u32                 custom_len;
	__s16 __user         *custom_data;

};

enum ic_type {
	AW8624 = 0,
	AW86224,
	AW8697,
	AW86917,
	AWINIC_MAX = 10,
	DRV2625,
	TI_MAX = 20,
	VIB_NONE = 0xFF,
};

enum ic_calibration_data_len {
	AWINIC_LEN = 2,
	TI_LEN = 4
};

struct haptic_cali_awinic {
	int f0;
	int f0_offset;
};

struct haptic_cali_ti {
	int f0;
	int CalComp;
	int CalBemf;
	int CalGain;
};

struct haptic_cali_param {
	enum ic_calibration_data_len calibration_data_len;
	union {
		struct haptic_cali_awinic awinic;
		struct haptic_cali_ti ti;
	} u;
};

struct haptic_rtp_container{
	unsigned int len;
	unsigned char data[];
};


/* ============================================================================ */
/* ============= above same as hidl vivo_haptic_core.h ======================== */
/* ============================================================================ */

#include <linux/fs.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/types.h>

enum effect_channel {
	UPLOAD = 0,
	AAC_RTP_MODE,
	AAC_STREAM_MODE,
};


struct reg_debug {
	u16 start_addr;
	int count;
	bool set_flag;
};

/**
 * struct user_status - represents current calling status for userspace
 * @user_id: allocted for current calling
 * @current_owner: current calling open the dev node
 */
struct user_status {
	int user_id;
	struct file *current_owner;
	enum effect_channel channel_flag; // only used for aac richtap

};


/**
 * struct haptic_handle - represents a Operating collection for IC
 * @name: name of miscdev handle
 * @dev: driver model view of IC driver device(eg:form IC probe)
 * @upload: complete the preparation work of vibration playback，(Must be implemented)
 * @playback: start or stop playing，(Must be implemented)
 * @erase: the work of cleaning up (maybe not use)
 * @set_gain: set the gain of vibration
 * @set_trigger_intensity: set the gain of trigger vibration
 * @judge_effect_support: Determine whether the current effect is supported in DTS
 * @get_f0:get motor actual f0
 * @init_dev: when the device is opened, initialize the device
 * @hap_bit: bitmap of functions that ic has
 * @chip: point to IC information structure
 * @private: reserve
 */

struct haptic_handle {
	const char     *name;
	struct device  *dev;

	int (*upload)(struct haptic_handle *hh, struct haptic_effect *effect);
	int (*playback)(struct haptic_handle *hh, int value);
	int (*erase)(struct haptic_handle *hh);
	void (*set_gain)(struct haptic_handle *hh, u16 gain);
	void (*set_trigger_intensity)(struct haptic_handle *hh, int gain);
	int (*judge_effect_support)(struct haptic_handle *hp, int16_t effectNo);
	void (*get_motor_type)(struct haptic_handle *hh, int *motorType);
	void (*get_f0)(struct haptic_handle *hh, int *f0);
	void (*set_cali_params)(struct haptic_handle *hh, struct haptic_cali_param *cali_params);
	void (*get_driver_ic)(struct haptic_handle *hh, enum ic_type *driver_ic);

	void (*init_dev)(struct haptic_handle *hh);
	void (*destroy)(struct haptic_handle *hh);

	// AT and Debug
	int (*f0_calibration)(struct haptic_handle *hh, char *buf);
	int (*f0_test)(struct haptic_handle *hh, char *buf);
	int (*impedance_test)(struct haptic_handle *hh, char *buf);

	int (*dump_reg)(struct haptic_handle *hh, char *buf, u16 start_addr, int count);
	int (*set_reg)(struct haptic_handle *hh, u16 addr, u8 val);

	// extend for AAC RICHTAP begin

	void (*get_ic_identity_NO)(struct haptic_handle *hh, uint8_t *identity_NO);
	void (*get_motor_f0)(struct haptic_handle *hh, int *f0);
	void (*set_gain_reg_direct)(struct haptic_handle *hh, int gain_reg_val);
	int (*play_rtp_less_than_FIFO)(struct haptic_handle *hh, void __user *p);
	int (*play_rtp_from_cycle_buffer)(struct haptic_handle *hh);
	int (*rtp_stop)(struct haptic_handle *hh);

	// extend for AAC RICHTAP end

	unsigned long  hap_bit[BITS_TO_LONGS(HAPTIC_CNT)];
	void           *chip;
	void           *private;
	struct haptic_rtp_container *haptic_container;
	struct mmap_buf_format      *start_buf; // only for AAC RICHTAP

};


/**
 * struct haptic_misc - represents a haptic device
 * @misc: miscdev structure
 * @handle: Operator functions that drive the IC,
 *          and relevant information
 * @lock: mutex lock for fops
 * @devt: device number of the miscdev
 * @open: times of opening the miscdev
 * @list: used to mount it to haptic_list
 * @user_status: record the current state of the call
 * @usr_bit: bitmap that records ID assigned to the caller
 * @private_data: reserve
 */

#define DYNAMIC_USER_IDS (0xFF + 1)
struct haptic_misc {
	struct miscdevice     misc;
	const char            *name;
	struct haptic_handle  *handle;
	struct mutex          lock;
	dev_t                 devt;
	int                   open;
	struct list_head      list;
	struct user_status    user_status;
	unsigned long         usr_bit[BITS_TO_LONGS(DYNAMIC_USER_IDS)];
	void                  *private_data;
	struct reg_debug      reg_debug;
	char                  *m_buf;
};


extern int haptic_miscdev_register(struct haptic_misc *hm);
extern int haptic_miscdev_unregister(struct haptic_misc *hm);
extern int haptic_handle_create(struct haptic_misc *hap_misc_dev, const char *name);
extern void haptic_device_set_capcity(struct haptic_handle *hh, int bit);
extern void haptic_handle_destroy(struct haptic_handle *hh);




#endif

