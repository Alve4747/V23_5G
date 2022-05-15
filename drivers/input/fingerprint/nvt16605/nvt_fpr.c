#include <linux/interrupt.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#endif
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/fb.h>


#if defined(CONFIG_BBK_FP_ID) || defined(CONFIG_BBK_FP_MODULE)
#include "../fp_id.h"
#endif

#include "nvt_fpr.h"

#define pr_fmt(fmt)		"[FP_KERN] " KBUILD_MODNAME ": " fmt

unsigned int bufsiz = 4096;
static int g_resource_requested;
struct class *novatek_class;
struct novatek_data *g_data;

LIST_HEAD(device_list);
DEFINE_MUTEX(device_list_lock);
DECLARE_BITMAP(minors, N_MINORS);

int nvt_power_onoff(int power_onoff)
{
	int rc = 0;
    struct novatek_data *nvt = g_data;
	NVT_DBG_DBG("[nvt] %s   power_onoff = %d \n", __func__, power_onoff);
	if (power_onoff) {
		if (nvt->vdd_use_gpio) {
			rc = gpio_direction_output(nvt->vdd_en_gpio, 1);
			if (rc) {
				NVT_DBG_ERR("nt16602 vdd power on fail.\n");
				return -EIO;
			}
		}

		if (nvt->vdd_use_pmic) {
			if (regulator_is_enabled(nvt->vdd)) {
				NVT_DBG_ERR("%s, vdd state:on,don't set repeatedly!\n", __func__);
				return rc;
			}

			rc = regulator_set_load(nvt->vdd, 600000);
			if (rc) {
				NVT_DBG_ERR("%s: vdd regulator_set_load(uA_load=%d) failed. rc=%d\n",
				__func__, 1000, rc);
				return rc;
			}

			rc = regulator_count_voltages(nvt->vdd);
			if (rc <= 0) {
				NVT_DBG_ERR("%s: regulator_count_voltages failed. rc=%d\n", __func__, rc);
				return rc;
			}
			
			rc = regulator_set_voltage(nvt->vdd, 3300000, 3300000);
			if (rc) {
				NVT_DBG_ERR(KERN_ERR "nt16602:Unable to set voltage on vdd, rc=%d\n", __func__, rc);
				return rc;
			}
			
			rc = regulator_enable(nvt->vdd);
			if (rc) {
				NVT_DBG_ERR(KERN_ERR "nt16602:Unable to enable regulator");
				return rc;
			}
		}

		// VDDIO ON
		if (nvt->vddio_use_gpio) {
			rc = gpio_direction_output(nvt->vddio_en_gpio, 1);
			if (rc) {
				NVT_DBG_ERR("nt16602 vddio power on fail.\n");
				return -EIO;
			}
			NVT_DBG_DBG("vddio_use_gpio power enable successfully\n");
		}

		if (nvt->vddio_use_pmic) {
			if (regulator_is_enabled(nvt->vddio)) {
				NVT_DBG_ERR("%s, vddio state:on,don't set repeatedly!\n", __func__);
				return rc;
			}

			rc = regulator_set_load(nvt->vddio, 600000);
			if (rc < 0) {
				NVT_DBG_ERR("%s: vddio regulator_set_load(uA_load=%d) failed. rc=%d\n",
				__func__, 1000, rc);
				return rc;
			}

			rc = regulator_count_voltages(nvt->vddio);
			if (rc <= 0) {
				NVT_DBG_ERR(KERN_ERR "%s: regulator_count_voltages failed, rc=%d\n", __func__, rc);
				return rc;
			}
		
			rc = regulator_set_voltage(nvt->vddio, 1800000, 1800000);
			if (rc) {
				NVT_DBG_ERR(KERN_ERR "nt16602:Unable to set voltage on vddio, rc=%d\n", __func__, rc);
				return rc;
			}
			
			rc = regulator_enable(nvt->vddio);
			if (rc) {
				NVT_DBG_ERR(KERN_ERR "nt16602:Unable to enable regulator");
				return rc;
			}
		}
	} else {
		//reset low
		pinctrl_select_state(nvt->pinctrl_gpios, nvt->pins_reset_low);

		// VDD OFF
		if (nvt->vdd_use_gpio) {
			rc = gpio_direction_output(nvt->vdd_en_gpio, 0);
			if (rc) {
				NVT_DBG_ERR("nt16602 vdd power off fail.\n");
				return -EIO;
			}
		}

		if (nvt->vdd_use_pmic) {
			rc = regulator_set_load(nvt->vdd, 0);
			if (rc < 0) {
				NVT_DBG_ERR("%s: vdd regulator_set_load(uA_load=%d) failed. rc=%d\n",
				__func__, 0, rc);
				return rc;
			}
			if (regulator_is_enabled(nvt->vdd)) {
				regulator_disable(nvt->vdd);
			}
			NVT_DBG_ERR(KERN_ERR "nt16602: disable  vdd %d\n", rc);
		}

		// VDDIO OFF
		if (nvt->vddio_use_gpio) {
			rc = gpio_direction_output(nvt->vddio_en_gpio, 0);
			if (rc) {
				NVT_DBG_ERR("nt16602 vddio power off fail.\n");
				return -EIO;
			}
			NVT_DBG_DBG("vddio_use_gpio power enable successfully\n");
		}

		if (nvt->vddio_use_pmic) {
			rc = regulator_set_load(nvt->vddio, 0);
			if (rc < 0) {
				NVT_DBG_ERR("%s: vddio regulator_set_load(uA_load=%d) failed. rc=%d\n",
					__func__, 0, rc);
				return rc;
			}
			if (regulator_is_enabled(nvt->vddio)) {
				regulator_disable(nvt->vddio);
			}
			NVT_DBG_ERR(KERN_ERR "nt16602: disable  vddio %d\n", rc);
		}
	}

	return rc;
}
void nvt_reset(void)
{
    struct novatek_data *nvt = g_data;
	pinctrl_select_state(nvt->pinctrl_gpios, nvt->pins_reset_low);
	mdelay(2);
	pinctrl_select_state(nvt->pinctrl_gpios, nvt->pins_reset_high);
}

int novatek_spi_pin(int spi_pulllow)
{
    struct novatek_data *novatek = g_data;
	NVT_DBG_DBG("%s spi_pulllow = %d\n", __func__, spi_pulllow);
	if (spi_pulllow) {
		pinctrl_select_state(novatek->pinctrl_gpios, novatek->pins_miso_pulllow);
		pinctrl_select_state(novatek->pinctrl_gpios, novatek->pins_mosi_pulllow);
		pinctrl_select_state(novatek->pinctrl_gpios, novatek->pins_cs_pulllow);
		pinctrl_select_state(novatek->pinctrl_gpios, novatek->pins_clk_pulllow);
	} else {
		pinctrl_select_state(novatek->pinctrl_gpios, novatek->pins_miso_spi);
		pinctrl_select_state(novatek->pinctrl_gpios, novatek->pins_mosi_spi);
		pinctrl_select_state(novatek->pinctrl_gpios, novatek->pins_cs_spi);
		pinctrl_select_state(novatek->pinctrl_gpios, novatek->pins_clk_spi);
	}
	return 0;
}

int nvt_request_resoure(void)
{
    struct novatek_data *data = g_data;
    int ret = 0;
	if (g_resource_requested == 0) {
		ret = nvt_parse_dt(data);
		if (ret) {
			NVT_DBG_ERR("parse dts failed, ret=%d\n", ret);
			return ret;
		}
		// request power io
		NVT_DBG_DBG(KERN_ERR "%s request power resource \n", __func__);
		if (data->vdd_use_gpio) {
			ret = gpio_request(data->vdd_en_gpio, "gpio_vdd_en");
			if (ret) {
				NVT_DBG_ERR("Failed to request VDD_EN GPIO. ret = %d,number=%d\n", ret, data->vdd_en_gpio);
				return -ENODEV;
			}
		}
		if (data->vdd_use_pmic) {
			data->vdd = regulator_get(&data->pd->dev, "vfp");
			if (IS_ERR(data->vdd)) {
			NVT_DBG_ERR("%s Unable to get vdd\n", __func__);
			return -EINVAL;
			} else {
				NVT_DBG_ERR("%s Success to get vdd\n", __func__);
			}
		}

		if (data->vddio_use_gpio) {
			ret = gpio_request(data->vddio_en_gpio, "gpio_vddio_en");
			if (ret) {
				NVT_DBG_ERR("%s Failed to request VDDIO_EN GPIO. ret = %d\n", __func__, ret);
				return -EIO;
			}
		}
		if (data->vddio_use_pmic) {
			data->vddio = regulator_get(&data->pd->dev, "vddio");
			if (IS_ERR(data->vddio)) {
				NVT_DBG_ERR("%s Unable to get vddio\n", __func__);
				return -EINVAL;
			} else {
				NVT_DBG_ERR("%s Success to get vddio\n", __func__);
			}
		}

		NVT_DBG_DBG(KERN_ERR "%s request reset state \n", __func__);
		if (data->pd) {
			data->pinctrl_gpios = devm_pinctrl_get(&data->pd->dev);
			if (IS_ERR(data->pinctrl_gpios)) {
				ret = PTR_ERR(data->pinctrl_gpios);
				NVT_DBG_ERR(KERN_ERR "%s can't find fingerprint pinctrl\n", __func__);
				return ret;
			}
			data->pins_reset_high = pinctrl_lookup_state(data->pinctrl_gpios, "reset_high");
			if (IS_ERR(data->pins_reset_high)) {
				ret = PTR_ERR(data->pins_reset_high);
				NVT_DBG_ERR("%s can't find fingerprint pinctrl pins_reset_high\n", __func__);
				return ret;
			}

			data->pins_reset_low = pinctrl_lookup_state(data->pinctrl_gpios, "reset_low");
			if (IS_ERR(data->pins_reset_low)) {
				ret = PTR_ERR(data->pins_reset_low);
				NVT_DBG_ERR("%s can't find fingerprint pinctrl pins_reset_low\n", __func__);
				return ret;
			}

			data->pins_irq = pinctrl_lookup_state(data->pinctrl_gpios, "fingerprint_irq");
			if (IS_ERR(data->pins_irq)) {
				ret = PTR_ERR(data->pins_irq);
				NVT_DBG_ERR("%s can't find fingerprint pinctrl fingerprint_irq\n", __func__);
				return ret;
			}
   
		}	

		//spi state
		NVT_DBG_DBG(KERN_ERR "%s request spi state \n", __func__);
		if (data->pd) {
			data->pins_mosi_spi = pinctrl_lookup_state(data->pinctrl_gpios, "mosi_spi");
			if (IS_ERR(data->pins_mosi_spi)) {
				ret = PTR_ERR(data->pins_mosi_spi);
				NVT_DBG_ERR("%s can't find fingerprint pinctrl pins_mosi_spi\n", __func__);
				return ret;
			}
			data->pins_mosi_pullhigh = pinctrl_lookup_state(data->pinctrl_gpios, "mosi_pullhigh");
			if (IS_ERR(data->pins_mosi_pullhigh)) {
				ret = PTR_ERR(data->pins_mosi_pullhigh);
				NVT_DBG_ERR("%s can't find fingerprint pinctrl mosi_pullhigh\n", __func__);
				return ret;
			}
			data->pins_mosi_pulllow = pinctrl_lookup_state(data->pinctrl_gpios, "mosi_pulllow");
			if (IS_ERR(data->pins_mosi_pulllow)) {
				ret = PTR_ERR(data->pins_mosi_pulllow);
				NVT_DBG_ERR("%s can't find fingerprint pinctrl mosi_pulllow\n", __func__);
				return ret;
			}
			data->pins_miso_spi = pinctrl_lookup_state(data->pinctrl_gpios, "miso_spi");
			if (IS_ERR(data->pins_miso_spi)) {
				ret = PTR_ERR(data->pins_miso_spi);
				NVT_DBG_ERR("%s can't find fingerprint pinctrl pins_miso_spi\n", __func__);
				return ret;
			}
			data->pins_miso_pullhigh = pinctrl_lookup_state(data->pinctrl_gpios, "miso_pullhigh");
			if (IS_ERR(data->pins_miso_pullhigh)) {
				ret = PTR_ERR(data->pins_miso_pullhigh);
				NVT_DBG_ERR("%s can't find fingerprint pinctrl miso_pullhigh\n", __func__);
				return ret;
			}
			data->pins_miso_pulllow = pinctrl_lookup_state(data->pinctrl_gpios, "miso_pulllow");
			if (IS_ERR(data->pins_miso_pulllow)) {
				ret = PTR_ERR(data->pins_miso_pulllow);
				NVT_DBG_ERR("%s can't find fingerprint pinctrl miso_pulllow\n", __func__);
				return ret;
			}
			data->pins_cs_spi = pinctrl_lookup_state(data->pinctrl_gpios, "cs_spi");
			if (IS_ERR(data->pins_cs_spi)) {
				ret = PTR_ERR(data->pins_cs_spi);
				NVT_DBG_ERR("%s can't find fingerprint pinctrl pins_cs_spi\n", __func__);
				return ret;
			}
			data->pins_cs_pullhigh = pinctrl_lookup_state(data->pinctrl_gpios, "cs_pullhigh");
			if (IS_ERR(data->pins_cs_pullhigh)) {
				ret = PTR_ERR(data->pins_cs_pullhigh);
				NVT_DBG_ERR("%s can't find fingerprint pinctrl cs_pullhigh\n", __func__);
				return ret;
			}
			data->pins_cs_pulllow = pinctrl_lookup_state(data->pinctrl_gpios, "cs_pulllow");
			if (IS_ERR(data->pins_cs_pulllow)) {
				ret = PTR_ERR(data->pins_cs_pulllow);
				NVT_DBG_ERR("%s can't find fingerprint pinctrl cs_pulllow\n", __func__);
				return ret;
			}
			data->pins_clk_spi = pinctrl_lookup_state(data->pinctrl_gpios, "clk_spi");
			if (IS_ERR(data->pins_clk_spi)) {
				ret = PTR_ERR(data->pins_clk_spi);
				NVT_DBG_ERR("%s can't find fingerprint pinctrl pins_clk_spi\n", __func__);
				return ret;
			}
			data->pins_clk_pullhigh = pinctrl_lookup_state(data->pinctrl_gpios, "clk_pullhigh");
			if (IS_ERR(data->pins_clk_pullhigh)) {
				ret = PTR_ERR(data->pins_clk_pullhigh);
				NVT_DBG_ERR("%s can't find fingerprint pinctrl clk_pullhigh\n", __func__);
				return ret;
			}
			data->pins_clk_pulllow = pinctrl_lookup_state(data->pinctrl_gpios, "clk_pulllow");
			if (IS_ERR(data->pins_clk_pulllow)) {
				ret = PTR_ERR(data->pins_clk_pulllow);
				NVT_DBG_ERR("%s can't find fingerprint pinctrl clk_pulllow\n", __func__);
				return ret;
			}
		}

		g_resource_requested = 1;
		NVT_DBG_DBG(" %s request resource successfully! \n", __func__);
	} else {
		NVT_DBG_DBG("resource is requested already.");
	}

    return 0;
}

int nvt_release_resoure(void)
{
    struct novatek_data *data = g_data;

    if (g_resource_requested == 1) {

		// release power
		if (data->vdd_use_gpio) {
			if (gpio_is_valid(data->vdd_en_gpio)) {
				gpio_free(data->vdd_en_gpio);
				NVT_DBG_DBG("remove vdd_en_gpio success\n");
			}
		}
		if (data->vddio_use_gpio) {
			if (gpio_is_valid(data->vddio_en_gpio)) {
				gpio_free(data->vddio_en_gpio);
				NVT_DBG_DBG("remove vdd_en_gpio success\n");
			}
		}
		if (data->vdd_use_pmic) {
			if (regulator_is_enabled(data->vdd)) {
				regulator_disable(data->vdd);
			}
			devm_regulator_put(data->vdd);
		}
		if (data->vddio_use_pmic) {
			if (regulator_is_enabled(data->vddio)) {
				regulator_disable(data->vddio);
			}
			devm_regulator_put(data->vddio);
		}

		//release pinctl
		if (data->pinctrl_gpios) {
			NVT_DBG_DBG(" %s : devm_pinctrl_put \n", __func__);
			devm_pinctrl_put(data->pinctrl_gpios);
			data->pinctrl_gpios = NULL;
		}

		g_resource_requested = 0;
		NVT_DBG_DBG(" %s release resource successfully! \n", __func__);
	} else {
		NVT_DBG_ERR("resource not request, can not release resource.\n");
	}

	return 0;
}

int nvt_parse_dt(struct novatek_data *data)
{
#ifdef CONFIG_OF
	int ret = 0;
	struct device_node *node = NULL;
	struct platform_device *pdev = NULL;
	bool test = false;
	NVT_DBG_DBG(KERN_ERR "%s, from dts pinctrl\n", __func__);

	if (data->pd->dev.of_node) {

		data->vdd_use_gpio = of_property_read_bool(data->pd->dev.of_node, "fp,vdd_use_gpio");
		data->vdd_use_pmic = of_property_read_bool(data->pd->dev.of_node, "fp,vdd_use_pmic");
		data->vddio_use_gpio = of_property_read_bool(data->pd->dev.of_node, "fp,vddio_use_gpio");
		data->vddio_use_pmic = of_property_read_bool(data->pd->dev.of_node, "fp,vddio_use_pmic");

		NVT_DBG_DBG(KERN_ERR "%s vdd_use_gpio %d\n", __func__, data->vdd_use_gpio);
		NVT_DBG_DBG(KERN_ERR "%s vdd_use_pmic %d\n", __func__, data->vdd_use_pmic);
		NVT_DBG_DBG(KERN_ERR "%s vddio_use_gpio %d\n", __func__, data->vddio_use_gpio);
		NVT_DBG_DBG(KERN_ERR "%s vddio_use_pmic %d\n", __func__, data->vddio_use_pmic);
	}
	//gpio
	if (data->vdd_use_gpio) {
		data->vdd_en_gpio = of_get_named_gpio(data->pd->dev.of_node, "fp,gpio_vdd_en", 0);
		if (!gpio_is_valid(data->vdd_en_gpio)) {
			NVT_DBG_ERR("VDD_EN GPIO is invalid.\n");
			return -ENODEV;
		}
	}
	// VDDIO-gpio
	if (data->vddio_use_gpio) {
		data->vddio_en_gpio = of_get_named_gpio(data->pd->dev.of_node, "fp,gpio_vddio_en", 0);
		if (!gpio_is_valid(data->vddio_en_gpio)) {
			NVT_DBG_ERR("%s VDDIO_EN GPIO is invalid.\n", __func__);
			return -EIO;
		} else {
			NVT_DBG_ERR("%s VDDIO_EN GPIO is %d.\n", __func__, data->vddio_en_gpio);
		}
	}
#endif
	NVT_DBG_DBG("%s is successful\n", __func__);
	return 0;
}

long novatek_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int retval = 0;
	int val = 0;

	if (_IOC_TYPE(cmd) != FPR_IOC_MAGIC) {
		NVT_DBG_ERR("_IOC_TYPE(cmd) != FPR_IOC_MAGIC\n");
		return -ENOTTY;
	}

	switch (cmd) {
	case FPR_IOC_V3_GPIO_CTRL:
		if (copy_from_user(&val, (void __user *)arg, _IOC_SIZE(cmd))) {
			NVT_DBG_ERR("copy from user\n");
			return -ENOTTY;
		}
		NVT_DBG_DBG("[nvt] %s: val = %d\n", __func__, val);
		nvt_power_onoff(val); // power setting. 1 = on, 0 = off.
		break;
	case FPR_IOC_RST_CTRL:
		nvt_reset();
		NVT_DBG_DBG("[nvt] %s: nvt_reset\n", __func__);
		break;
	case FPR_IOC_SPIPIN_SETTING:
		NVT_DBG_DBG("%smodify to spi mode\n", __func__);
		novatek_spi_pin(0);
		break;
	case FPR_IOC_SPIPIN_PULLLOW:
		NVT_DBG_DBG("%s  spi_pulllow to gpio mode\n", __func__);
		novatek_spi_pin(1);
		break;
	case FPR_IOC_SPICLK_ENABLE:
		NVT_DBG_DBG("novatek enable spi");
		vfp_spi_clk_enable(1);
		break;
	case FPR_IOC_SPICLK_DISABLE:
		NVT_DBG_DBG("novatek disable spi\n");
		vfp_spi_clk_enable(0);
		break;
	case FPR_IOC_REQUEST_RESOURCE:
		NVT_DBG_DBG("novatek request resource\n");
		retval = nvt_request_resoure();
		break;
	case FPR_IOC_RELEASE_RESOURCE:
		NVT_DBG_DBG("novatek release resource\n");
		retval = nvt_release_resoure();
		break;
	default:
		NVT_DBG_ERR("novatek unknow ioctl cmd\n");
		retval = -ENOTTY;
		break;
	}
	NVT_DBG_DBG(" %s done  \n", __func__);
	return retval;
}

int novatek_release(struct inode *inode, struct file *filp)
{
	struct novatek_data *novatek;
	NVT_DBG_DBG("%s\n", __func__);
	mutex_lock(&device_list_lock);
	novatek = filp->private_data;
	filp->private_data = NULL;
	/* last close? */
	novatek->users--;
	if (novatek->users == 0) {
		int	dofree;
		kfree(novatek->buffer);
		novatek->buffer = NULL;
		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&novatek->spi_lock);
		dofree = (novatek->pd == NULL);
		spin_unlock_irq(&novatek->spi_lock);
		if (dofree)
			kfree(novatek);
	}
	mutex_unlock(&device_list_lock);
	return 0;
}

int novatek_open(struct inode *inode, struct file *filp)
{
	struct novatek_data *novatek;
	int status = -ENXIO;
	NVT_DBG_DBG("%s\n", __func__);
	mutex_lock(&device_list_lock);
	list_for_each_entry(novatek, &device_list, device_entry) {
		if (novatek->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status == 0) {
		if (novatek->buffer == NULL) {
			novatek->buffer = kmalloc(bufsiz, GFP_KERNEL);
			if (novatek->buffer == NULL) {
				NVT_DBG_ERR("open/ENOMEM\n");
				status = -ENOMEM;
			}
		}
		if (status == 0) {
			novatek->users++;
			filp->private_data = novatek;
			nonseekable_open(inode, filp);
		}
	} else {
		NVT_DBG_ERR("%s nothing for minor %d\n", __func__, iminor(inode));
	}
	mutex_unlock(&device_list_lock);
	return status;
}

const struct file_operations novatek_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = novatek_ioctl,
	.open = novatek_open,
	.release =	novatek_release,
};

int novatek_remove(struct platform_device *pdev)
{
	NVT_DBG_DBG("%s\n", __func__);
	device_destroy(novatek_class, g_data->devt);
	list_del(&g_data->device_entry);
	class_destroy(novatek_class);
	unregister_chrdev(NVT_MAJOR, NVT_DEV_NAME);
	g_data = NULL;
}
int novatek_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct novatek_data *novatek;
	int status = 0;
	unsigned long minor;

	NVT_DBG_DBG("%s initial\n", __func__);

	status = register_chrdev(NVT_MAJOR, NVT_DEV_NAME, &novatek_fops);
	if (status < 0) {
			NVT_DBG_ERR("%s register_chrdev error.\n", __func__);
			return status;
	}

	novatek_class = class_create(THIS_MODULE, NVT_DEV_NAME);
	if (IS_ERR(novatek_class)) {
		NVT_DBG_ERR("%s class_create error.\n", __func__);
		unregister_chrdev(NVT_MAJOR, NVT_DEV_NAME);
		return PTR_ERR(novatek_class);
	}
	/* Allocate driver data */
	novatek = kzalloc(sizeof(struct novatek_data), GFP_KERNEL);
	if (novatek == NULL) {
		NVT_DBG_ERR("%s - Failed to kzalloc\n", __func__);
		return -ENOMEM;
	}

	/* Initialize the driver data */
	novatek->pd = pdev;
	g_data = novatek;
	//wake_lock_init(&nvt_wake_lock, WAKE_LOCK_SUSPEND, "nvt_wake_lock");
	NVT_DBG_DBG("novatek_module_probe\n");
	spin_lock_init(&novatek->spi_lock);
	mutex_init(&novatek->buf_lock);
	mutex_init(&device_list_lock);

	INIT_LIST_HEAD(&novatek->device_entry);

	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_MINORS);
    novatek->devt = MKDEV(NVT_MAJOR, minor);
    novatek->device = device_create(novatek_class, &pdev->dev, novatek->devt, novatek, NVT_DEV_NAME);
    status = IS_ERR(novatek->device) ? PTR_ERR(novatek->device) : 0;
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&novatek->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	if (status == 0) {
		dev_set_drvdata(dev, novatek);
	} else {
		goto novatek_probe_failed;
	}
	NVT_DBG_DBG("%s : initialize success %d\n", __func__, status);
	return status;
novatek_probe_failed:
	device_destroy(novatek_class, novatek->devt);
	class_destroy(novatek_class);
	return status;
}

struct of_device_id novatek_match_table[] = {
	{ .compatible = "mediatek,nvt-fp",},
	{},
};
MODULE_DEVICE_TABLE(of, novatek_match_table);

struct platform_driver novatek_driver = {
	.driver = {
		.name		= NVT_DEV_NAME,
		.owner		= THIS_MODULE,
		.of_match_table = novatek_match_table,
	},
	.probe =    novatek_probe,
	.remove =   novatek_remove,
};

int __init nvt_fpr_init(void)
{
	int status = 0;

	status = platform_driver_register(&novatek_driver);
	if (status < 0) {
		NVT_DBG_ERR("register platform driver fail%s\n", __func__);
		status = -EINVAL;
	}
    return status;
}

void __exit nvt_fpr_exit(void)
{
	  platform_driver_unregister(&novatek_driver);
}

late_initcall(nvt_fpr_init);
module_exit(nvt_fpr_exit);

MODULE_AUTHOR("Novatek");
MODULE_DESCRIPTION("Platform driver for Novatek");
MODULE_LICENSE("GPL");
