#ifndef _FP_LINUX_DIRVER_H_
#define _FP_LINUX_DIRVER_H_
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <asm/irq.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>

#define NVT_MAJOR				100 /* assigned */
#define N_MINORS                32
#define NVT_DEV_NAME  "nvt_device"
#define NVT_FP "nvt_fp"
#define NVT_DEBUG (1)

#define FPR_IOC_MAGIC                   'f'
#define FPR_IOC_RST_CTRL                _IOW(FPR_IOC_MAGIC, 5, int)
#define FPR_IOC_ENABLE_INTERRUPT        _IOW(FPR_IOC_MAGIC, 20, int)
#define FPR_IOC_V3_GPIO_CTRL            _IOW(FPR_IOC_MAGIC, 25, int)
#define FPR_IOC_REQUEST_RESOURCE        _IOW(FPR_IOC_MAGIC, 30, int)
#define FPR_IOC_RELEASE_RESOURCE        _IOW(FPR_IOC_MAGIC, 35, int)

#define FPR_IOC_SPICLK_ENABLE           _IOW(FPR_IOC_MAGIC, 40, int)
#define FPR_IOC_SPICLK_DISABLE          _IOW(FPR_IOC_MAGIC, 45, int)
#define FPR_IOC_SPIPIN_SETTING          _IOW(FPR_IOC_MAGIC, 50, int)
#define FPR_IOC_SPIPIN_PULLLOW          _IOW(FPR_IOC_MAGIC, 55, int)

#if NVT_DEBUG
#  define NVT_DBG_DBG(fmt, args...)    pr_err("[%s] %s %d: " fmt, NVT_FP, __func__, __LINE__, ##args)
#else
#  define NVT_DBG_DBG(fmt, args...)    pr_info("[%s] %s %d: " fmt, NVT_FP, __func__, __LINE__, ##args)
#endif

#define NVT_DBG_ERR(fmt, args...)    pr_err("[%s] %s %d: " fmt, NVT_FP, __func__, __LINE__, ##args)

/* ------------------------- Structure ------------------------------*/
struct novatek_data {
	dev_t devt;
	spinlock_t spi_lock;
	struct spi_device  *spi;	
	struct platform_device *pd;
	struct device *device;
	struct list_head device_entry;
	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex buf_lock;
	unsigned users;
	u8 *buffer;
	bool vdd_use_gpio;
	bool vdd_use_pmic;
	bool vddio_use_gpio;
	bool vddio_use_pmic;
	u32  vdd_en_gpio;
	u32  vddio_en_gpio;
	struct regulator *vdd;
	struct regulator *vddio;
    struct input_dev	*input_dev;
	bool property_navigation_enable;
	struct regulator *fp_reg;
	bool pwr_by_gpio;
#ifdef CONFIG_OF
	struct pinctrl *pinctrl_gpios;
	struct pinctrl *pinctrl_gpios_spi;
	struct pinctrl_state *pins_irq;
	struct pinctrl_state *pins_reset_high, *pins_reset_low;
	struct pinctrl_state *pins_vcc_high, *pins_vcc_low;
	struct pinctrl_state *pins_miso_spi, *pins_miso_pullhigh, *pins_miso_pulllow;
	struct pinctrl_state *pins_mosi_spi, *pins_mosi_pullhigh, *pins_mosi_pulllow;
	struct pinctrl_state *pins_cs_spi, *pins_cs_pullhigh, *pins_cs_pulllow;
	struct pinctrl_state *pins_clk_spi, *pins_clk_pullhigh, *pins_clk_pulllow;
#endif	
};

extern void vfp_spi_clk_enable(uint8_t bonoff);
int novatek_remove(struct platform_device *pdev);
int nvt_power_onoff(int power_onoff);
void nvt_reset(void);
int nvt_request_resoure(void);
int nvt_release_resoure(void);
int novatek_spi_pin(int spi_pulllow);
int novatek_probe(struct platform_device *pdev);
int nvt_parse_dt(struct novatek_data *data);
int novatek_open(struct inode *inode, struct file *filp);
int novatek_release(struct inode *inode, struct file *filp);

#endif
