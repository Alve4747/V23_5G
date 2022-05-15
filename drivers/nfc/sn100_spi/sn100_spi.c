#include <linux/slab.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/fb.h>

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/moduleparam.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#else
#include <linux/notifier.h>
#endif

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#endif

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#ifdef CONFIG_MTK_CLKMGR
#include "mach/mt_clkmgr.h"
#else
#include <linux/clk.h>
#endif

#include <net/sock.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/miscdevice.h>

#define NFC_DEV_NAME "mediatek,nxp-sn100"

extern void mt_spi_enable_master_clk(struct spi_device *spidev);
extern void mt_spi_disable_master_clk(struct spi_device *spidev);

int log_level = 1;
static struct spi_device *spi_dev;
static bool spiclk_en;

#define sn100_info(level, fmt, args...) do { \
	if (level >= 1) {\
		pr_info("[sn100 spi] " fmt, ##args); \
	} \
} while (0)

#define FUNC_ENTRY()  \
	sn100_info(log_level, "%s, %d, enter\n", __func__, __LINE__)
#define FUNC_EXIT()  \
	sn100_info(log_level, "%s, %d, exit\n", __func__, __LINE__)


static int sn100_remove(struct spi_device *spi);

struct pinctrl *pinctrl_gpios;
struct pinctrl_state *pins_irq;
struct pinctrl_state *pins_miso_spi;
struct pinctrl_state *pins_mosi_spi;
struct pinctrl_state *pins_cs_spi;
struct pinctrl_state *pins_clk_spi;
#ifdef CONFIG_OF

#ifdef CONFIG_SEC_NFC

#define SEC_NFC_MAGIC   'S'
#define SEC_ESE_COLD_RESET      _IOW(SEC_NFC_MAGIC, 6, unsigned int)
#define SEC_SWP_SLEEP  _IOW(SEC_NFC_MAGIC, 7, unsigned int)
#define SEC_SWP_WAKEUP  _IOW(SEC_NFC_MAGIC, 8, unsigned int)
#define SEC_SPI_CLK_ENABLE  _IOW(SEC_NFC_MAGIC, 10, unsigned int)
#define SEC_SPI_CLK_DISABLE  _IOW(SEC_NFC_MAGIC, 11, unsigned int)

extern int trig_cold_reset(void);
extern int trig_nfc_wakeup(void);
extern int trig_nfc_sleep(void);

#endif


static const struct of_device_id sn100_of_match[] = {
	{ .compatible = "mediatek,sn100-spi", },
	{ .compatible = "mediatek,nxp-sn100", },
	{},
};
MODULE_DEVICE_TABLE(of, sn100_of_match);
#endif
static int nfc_get_gpio_dts_info(void)
{
	int ret = 0;

	struct device_node *node = NULL;
	struct platform_device *pdev = NULL;

	sn100_info(log_level, "nfc_get_gpio_dts_info enter\n");
	node = of_find_compatible_node(NULL, NULL, "mediatek,nxp-sn100");
	if (node) {
		pdev = of_find_device_by_node(node);
		if (pdev) {
			pinctrl_gpios = devm_pinctrl_get(&pdev->dev);
			if (IS_ERR(pinctrl_gpios)) {
				ret = PTR_ERR(pinctrl_gpios);
				pr_err("%s can't find nfc pinctrl ret:%d\n", __func__, ret);
				return ret;
			}
		} else {
			pr_err("%s platform device is null\n", __func__);
			return -1;
		}
	} else {
		pr_err("%s device node is null\n", __func__);
		return -1;
	}

	pins_miso_spi = pinctrl_lookup_state(pinctrl_gpios, "miso_spi");
	if (IS_ERR(pins_miso_spi)) {
		ret = PTR_ERR(pins_miso_spi);
		pr_err("%s can't find pinctrl miso_spi ret=%d\n", __func__, ret);
		return ret;
	}
	ret = pinctrl_select_state(pinctrl_gpios, pins_miso_spi);
	if (ret != 0) {
		pr_err("%s pinctrl_select_state pins_miso_spi ret =%d\n", __func__, ret);
	}
	pins_mosi_spi = pinctrl_lookup_state(pinctrl_gpios, "mosi_spi");
	if (IS_ERR(pins_mosi_spi)) {
		ret = PTR_ERR(pins_mosi_spi);
		pr_err("%s can't find pinctrl pins_mosi_spi\n", __func__);
		return ret;
	}
	ret = pinctrl_select_state(pinctrl_gpios, pins_mosi_spi);
	if (ret != 0) {
		pr_err("%s pinctrl_select_state pins_mosi_spi ret =%d\n", __func__, ret);
	}
	pins_cs_spi = pinctrl_lookup_state(pinctrl_gpios, "cs_spi");
	if (IS_ERR(pins_cs_spi)) {
		ret = PTR_ERR(pins_cs_spi);
		pr_err("%s can't find pinctrl cs_spi ret=%d\n", __func__, ret);
		return ret;
	}
	ret = pinctrl_select_state(pinctrl_gpios, pins_cs_spi);
	if (ret != 0) {
		pr_err("%s pinctrl_select_state pins_cs_spi ret =%d\n", __func__, ret);
	}
	pins_clk_spi = pinctrl_lookup_state(pinctrl_gpios, "clk_spi");
	if (IS_ERR(pins_clk_spi)) {
		ret = PTR_ERR(pins_clk_spi);
		pr_err("%s can't find pinctrl clk_spi ret=%d\n", __func__, ret);
		return ret;
	}
	ret = pinctrl_select_state(pinctrl_gpios, pins_clk_spi);
	if (ret != 0) {
		pr_err("%s pinctrl_select_state pins_clk_spi ret =%d\n", __func__, ret);
	}
	return ret;
}

//static void sn100_clk_enable(struct spi_device *spi, u8 bonoff)
bool sn100_clk_enable(u8 bonoff)
{
	FUNC_ENTRY();

	if (NULL == spi_dev) {
		pr_err("%s spi_device is NULL\n", __func__);
		return false;
	}

	if (spiclk_en && bonoff)
		return true;
	if (!spiclk_en && !bonoff)
		return true;

#ifdef CONFIG_MTK_CLKMGR
	if (bonoff)
		enable_clock(MT_CG_PERI_SPI1, "spi");
	else
		disable_clock(MT_CG_PERI_SPI1, "spi");
#else
	if (bonoff) {
		mt_spi_enable_master_clk(spi_dev);
		spiclk_en = true;
		sn100_info(log_level, "enable spi clk\n");
	} else {
		mt_spi_disable_master_clk(spi_dev);
		spiclk_en = false;
		sn100_info(log_level, "disable spi clk\n");
	}
#endif
	return true;
	FUNC_EXIT();
}

bool sn100_get_spiclk_status(void)
{
	return spiclk_en;
}

#ifdef CONFIG_SEC_NFC
int ese_power_reset(void)
{
	/*Add Reset Sequence here*/
	return trig_cold_reset();
}

static int sec_ese_power_open(struct inode *inode, struct file *file)
{
    pr_info("sec_ese_power_open\n");
    return 0;
}

static int sec_ese_power_close(struct inode *inodep, struct file *filp)
{
    pr_info("sec_ese_power_close\n");
    return 0;
}

static ssize_t sec_ese_power_write(struct file *file, const char __user *buf, size_t len, loff_t *ppos)
{

    pr_info("sec_ese_power_write \n");
    return 0;
}

static long sec_ese_power_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int status = 0;
	int ret = 0;
    pr_info("sec_ese_power_ioctl \n");
    switch (cmd) {
    case SEC_SWP_SLEEP:
		ret = trig_nfc_sleep();
		break;
    case SEC_SWP_WAKEUP:

		ret = trig_nfc_wakeup();
		break;

    case SEC_ESE_COLD_RESET:
		ret = ese_power_reset();
		break;
	case SEC_SPI_CLK_ENABLE:
		if (sn100_clk_enable(1) == false) {
			pr_err("%s spi clk enable error \n", __func__);
			ret = -1;
		}
		break;
	case SEC_SPI_CLK_DISABLE:
		if (sn100_clk_enable(0) == false) {
			pr_err("%s spi clk disable error \n", __func__);
			ret = -1;
		}
		break;
	}
	return ret;
}

static const struct file_operations sec_ese_power_fops = {
    .owner      = THIS_MODULE,
    .write      = sec_ese_power_write,
    .open       = sec_ese_power_open,
    .release    = sec_ese_power_close,
    .unlocked_ioctl = sec_ese_power_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = sec_ese_power_ioctl,
#endif
    .llseek     = no_llseek,
};

struct miscdevice sec_ese_power_miscdevice = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "sec-esepwr",
    .fops = &sec_ese_power_fops,
};

#endif


static int sn100_probe(struct spi_device *spi)
{
	int status = -EINVAL;
	FUNC_ENTRY();

	spiclk_en = false;
	if (NULL == spi) {
		pr_err("%s spi_device is NULL\n", __func__);
		goto err;
	}

	spi_dev = spi;
	status = nfc_get_gpio_dts_info();
	if (status) {
		status = -EINVAL;
		pr_err("%s nfc_get_gpio_dts_info error\n", __func__);
		goto err;
	}
	//sn100_clk_enable(spi, 1);//clk on

#ifdef CONFIG_SEC_NFC
	status = misc_register(&sec_ese_power_miscdevice);
    if (status) {
		pr_info("can't register eSE power driver :(\n");
		goto err;
	}
#endif
err:
	FUNC_EXIT();
	return status;
}

static int ese_power_suspend(struct device *dev)
{
	pr_info("ese_power_suspend\n");
	return 0;
}
static int ese_power_resume(struct device *dev)
{
	pr_info("ese_power_resume\n");
	//sn100_clk_enable(1);
	return 0;
}

static SIMPLE_DEV_PM_OPS(ese_power_pm_ops, ese_power_suspend, ese_power_resume);


static struct spi_driver sn100_spi_driver = {
	.driver = {
		.pm = &ese_power_pm_ops,
		.name = NFC_DEV_NAME,
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = sn100_of_match,
#endif
	},
	.probe = sn100_probe,
	.remove = sn100_remove,
};

static int sn100_remove(struct spi_device *spi)
{
	FUNC_ENTRY();
	spi_unregister_driver(&sn100_spi_driver);
	spi_dev = NULL;
	FUNC_EXIT();
	return  0;
}

static int __init sn100_spi_init(void)
{
	int status = 0;

	FUNC_ENTRY();

	status = spi_register_driver(&sn100_spi_driver);
	if (status < 0) {
		pr_err("%s Failed to register SPI driver.\n", __func__);
		return -EINVAL;
	}

	FUNC_EXIT();
	return status;
}
/* module_init(gf_init); */
late_initcall(sn100_spi_init);

static void __exit sn100_spi_exit(void)
{
	FUNC_ENTRY();
	spi_unregister_driver(&sn100_spi_driver);
	spi_dev = NULL;
	FUNC_EXIT();
}
module_exit(sn100_spi_exit);


MODULE_AUTHOR("nxp");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:sn100_spi");
