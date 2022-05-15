#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/serial_core.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>

#include <../mediatek/common/mtk-sp-spk-amp.h>
#include "vivo_smartpa_cali.h"

/* Mutex to serialize DSP read/write commands*/
static struct mutex routing_lock;

static struct i2c_client *smartpa_debug_client;
static unsigned char last_addr;

/* vendor ops for speaker calibration */
static struct vivo_cali_ops *local_ops;

#define VIVO_DEV_NAME   "smartpa"
#define VIVO_CTL_IOC_MAGIC  'T'
#define VIVO_IOCTL_SPK_REST  _IOW(VIVO_CTL_IOC_MAGIC, 0x01, int)
#define VIVO_IOCTL_SPK_INTS   _IOR(VIVO_CTL_IOC_MAGIC, 0x02, struct smartpa_msg)
#define VIVO_IOCTL_SPK_INTT  _IOW(VIVO_CTL_IOC_MAGIC, 0x03, int)
#define VIVO_IOCTL_SPK_RFDES 	_IOR(VIVO_CTL_IOC_MAGIC, 0x04, struct smartpa_msg)
#define VIVO_IOCTL_SPK_CHCK _IOR(VIVO_CTL_IOC_MAGIC, 0x05, int)
#define VIVO_IOCTL_SPK_PRARS _IOR(VIVO_CTL_IOC_MAGIC, 0x06, struct smartpa_prars)
#define VIVO_IOCTL_SPK_ADDR  _IOW(VIVO_CTL_IOC_MAGIC, 0x07, unsigned char)
#define VIVO_IOCTL_SPK_MTP_BACKUP _IOR(VIVO_CTL_IOC_MAGIC, 0x08, int)

/*
extern int vivo_smartpa_check_calib_dbg(void);
extern int vivo_smartpa_init_dbg(char *buffer, int size);
extern int vivo_smartpa_read_freq_dbg(char *buffer, int size);
extern void vivo_smartpa_read_prars_dbg(int temp[5], unsigned char addr);
extern void vivo_smartpa_get_client(struct i2c_client **client, unsigned char addr);
*/

static int afe_smartamp_get_set(uint8_t *data_buff, uint32_t param_id,
	uint8_t get_set, uint8_t length)
{
	int32_t ret = 0;
	int32_t rd_length = 0;
	struct mtk_apr apr_buff;
	int32_t i = 0;

	pr_info("[SmartPA:%s] get_set %d param_id %d length %d",
		__func__, get_set, param_id, length);
	if (length > AP_2_DSP_PAYLOAD_SIZE*4) {
		pr_err("[SmartPA:%s] Out of bound length %d", length);
		return -1;
	}

	switch (get_set) {
	case AP_2_DSP_SET_PARAM:
		{
			apr_buff.param_id = param_id;
			pr_err("[SmartPA:%s] AP_2_DSP_SET_PARAM param_id %d", __func__, param_id);
			memcpy(apr_buff.data, data_buff, length);
			ret = mtk_spk_send_ipi_buf_to_dsp((void *)&apr_buff, length+sizeof(param_id));
		}
		break;
	case AP_2_DSP_GET_PARAM:
		{
			apr_buff.param_id = param_id;
			pr_info("[SmartPA:%s] AP_2_DSP_GET_PARAM param_id %d", __func__, param_id);
			memset(apr_buff.data, 0, length);
			//update param_id firstly, since param_id can not be sent by get_buf
			ret = mtk_spk_send_ipi_buf_to_dsp((void *)&apr_buff, sizeof(param_id));
			if (ret == 0) {
				ret = mtk_spk_recv_ipi_buf_from_dsp((void *)&apr_buff, length+sizeof(param_id), &rd_length);
				pr_err("[SmartPA:%s] legen-AP_2_DSP_GET rd_length %d, %lld", __func__, rd_length, (unsigned long)(rd_length-sizeof(param_id)));
				if ((ret == 0) && (rd_length <= AP_2_DSP_PAYLOAD_SIZE*4+sizeof(param_id)) && (rd_length >= sizeof(param_id))) {
					memcpy(data_buff, apr_buff.data, rd_length-sizeof(param_id));
				}
			}

			//For Debug
			for (i = 0; i < length/4; i++)
				pr_err("[SmartPA:%s] apr_buff.data[%d] = 0x%0x", __func__, i, apr_buff.data[i]);

			break;
		}
	case AP_2_DSP_SEND_PARAM: /* wangkai add, to pass cali info (struct CALIBRATION_RX_) to DSP */
		{
			apr_buff.param_id = param_id;
			pr_info("[SmartPA:%s] AP_2_DSP_SEND_PARAM param_id %d", __func__, param_id);
			memcpy((void *)&(apr_buff.calib_param), data_buff, length);
			ret = mtk_spk_send_ipi_buf_to_dsp((void *)&apr_buff, sizeof(struct mtk_apr));
			if (ret < 0) {
				pr_err("[SmarPA] %s: mtk_spk_send_ipi_buf_to_dsp, param send error!\n", __func__);
			} else {
				pr_info("[SmartPA] %s: mtk_spk_send_ipi_buf_to_dsp, param send successful!\n", __func__);
			}
		}
		break;
	default:
		{
			break;
		}
	}

	return ret;
}

int mtk_afe_smartamp_algo_ctrl(uint8_t *data_buff, uint32_t param_id,
	uint8_t get_set, uint8_t length)
{
	int ret = 0;
	mutex_lock(&routing_lock);
	ret = afe_smartamp_get_set(data_buff, param_id,
		get_set, length);
	mutex_unlock(&routing_lock);
	return ret;
}

static ssize_t vivo_smartpa_debug_read (struct file *file,
	char __user *buf, size_t count, loff_t *offset)
{
	return 0;
/*
	char *tmp;
	int ret;
	smartpa_get_client(&smartpa_debug_client, last_addr);
	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL)
		return -ENOMEM;

	ret = i2c_master_recv(smartpa_debug_client, tmp, count);
	if (ret >= 0)
		ret = copy_to_user(buf, tmp, count) ? -EFAULT : ret;
	else 
		printk("[SmartPA]%s: transfer error %d\n", __func__, ret);		
	kfree(tmp);
	return ret;
*/
}
 
static ssize_t vivo_smartpa_debug_write (struct file *file,
	const char __user *buf, size_t count, loff_t *offset)
{
	return 0;
/*
	pr_info("%s have not complete it\n", __func__);
	return 0;

	char *tmp;
	int ret;

	smartpa_get_client(&smartpa_debug_client, last_addr);
	tmp = memdup_user(buf, count);
	if (IS_ERR(tmp)) 
		return PTR_ERR(tmp);
	ret = i2c_master_send(smartpa_debug_client, tmp, count);
	if (ret < 0)
	  printk("[SmartPA]%s: transfer error %d\n", __func__, ret);	
	kfree(tmp);
	return ret;
*/
}

static long  vivo_smartpa_debug_ioctl (struct file *file,
	unsigned int cmd, unsigned long arg)
{
    int  ret = 0, check = 0;
	int temp[5] = {0};
	struct smartpa_msg msg;
	struct smartpa_prars prars;

	memset(&prars, 0, sizeof(struct smartpa_prars));
	memset(&msg, 0, sizeof(struct smartpa_msg));
	switch (cmd) {
	/* Reset MTP */
	case VIVO_IOCTL_SPK_REST:
		printk("smartpa_ioctl SPK_REST\n");
		//max989xx_reset_mtp_dbg();
		break;
	/* calibrate */
	case VIVO_IOCTL_SPK_INTS:
		printk("smartpa_ioctl SPK_INTS\n");
		if (!local_ops || !local_ops->vivo_smartpa_init_dbg) {
			printk("[SmartPA] %s: local_ops or local_ops->vivo_smartpa_init_dbg is NULL\n", __func__);
			msg.msg_result = -1;
		} else {
			check = local_ops->vivo_smartpa_init_dbg(msg.msgs, MSGS_SIZE);
			msg.msg_result = check;
		}
		ret = copy_to_user((void *)arg, &msg, sizeof(struct smartpa_msg));
		break;
	case VIVO_IOCTL_SPK_INTT:
		printk("smartpa_ioctl SPK_INT\n");
		break;
	case VIVO_IOCTL_SPK_RFDES:
		//usleep_range(10*1000, 10*1000);
		printk("smartpa_ioctl SPK_ReadFDes\n");
		if (!local_ops || !local_ops->vivo_smartpa_read_freq_dbg) {
			printk("[SmartPA] %s: local_ops or local_ops->vivo_smartpa_read_freq_dbg is NULL\n", __func__);
			msg.msg_result = -1;
		} else {
			check = local_ops->vivo_smartpa_read_freq_dbg(msg.msgs, MSGS_SIZE);
			msg.msg_result = check;
		}
		ret = copy_to_user((void *)arg, &msg, sizeof(struct smartpa_msg));
		break;
	/* checkmtp */
	case VIVO_IOCTL_SPK_CHCK:
		printk("smartpa_ioctl SPK Check MtpEx\n");
		if (!local_ops || !local_ops->vivo_smartpa_check_calib_dbg) {
			printk("[SmartPA] %s: local_ops or local_ops->vivo_smartpa_read_freq_dbg is NULL\n", __func__);
			msg.msg_result = -1;
		} else {
			check = local_ops->vivo_smartpa_check_calib_dbg();
			msg.msg_result = check;
			pr_info("%s check %d.\n", __func__, check);
		}
		ret = copy_to_user((__user int *)arg, &check, sizeof(int));
		break;
	case VIVO_IOCTL_SPK_PRARS:
		//printk("smartpa_ioctl SPK Read f0 and Qt\n");
		//vivo_smartpa_read_prars_dbg(temp, last_addr);
		//prars.fRes_max = temp[0];
		//prars.fRes_min = temp[1];
		//prars.Qt = temp[2];
		//prars.impedance_max = temp[3];
		//prars.impedance_min = temp[4];

		//ret = copy_to_user((void *)arg, &prars, sizeof(struct smartpa_prars));
		//pr_info("smartpa_ioctl %d %d %d\n", temp[0], temp[1], temp[2]);
		pr_info("%s have not complete it\n", __func__);
		break;
	case VIVO_IOCTL_SPK_ADDR:
		//ret = copy_from_user(&last_addr, (void __user *)arg, sizeof(unsigned char));
		//printk("smartpa_ioctl addr %x\n", last_addr);
		pr_info("%s: have not complete it\n", __func__);
		break;
	case VIVO_IOCTL_SPK_MTP_BACKUP:
	//	check = max989xx_debug_mtp_backup(0);
		pr_info("%s have not complete it\n", __func__);
	//	ret = copy_to_user((__user int*)arg, &check, sizeof(int));
	default:
		printk("smartpa Fail IOCTL command no such ioctl cmd = %x\n", cmd);
		ret = -1;
		break;
    }

    return ret;
}

static int vivo_smartpa_debug_open(
	struct inode *inode, struct file *file)
{
	printk("[SmartPA]%s\n", __func__);
	return 0;
}

static int vivo_smartpa_debug_release(
	struct inode *inode, struct file *file)
{
	printk("[SmartPA]%s\n", __func__);
	return 0;
}

static const struct file_operations vivo_smartpa_debug_fileops = {
	.owner = THIS_MODULE,
	.open  = vivo_smartpa_debug_open,
	.read  = vivo_smartpa_debug_read,
	.write = vivo_smartpa_debug_write,
	.unlocked_ioctl = vivo_smartpa_debug_ioctl,
	.release = vivo_smartpa_debug_release,
};

static struct miscdevice vivo_smartpa_debug_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = VIVO_DEV_NAME,
	.fops = &vivo_smartpa_debug_fileops,
};

int vivo_smartpa_debug_probe(struct vivo_cali_ops *ops)
{
	int err = 0;

	printk("%s\n", __func__);

	if (!ops) {
		printk("%s: smartpa_device register failed for not implementing ops\n", __func__);
		return 0;
	}

	local_ops = ops;

	err = misc_register(&vivo_smartpa_debug_device);
	if (err) {
		printk("%s: smartpa_device register failed\n", __func__);
		return err;
	}

	return 0;
}

MODULE_DESCRIPTION("smartpa debug driver");
MODULE_AUTHOR("chenjinquan <chenjinquan@vivo.com>");
MODULE_LICENSE("GPL");
