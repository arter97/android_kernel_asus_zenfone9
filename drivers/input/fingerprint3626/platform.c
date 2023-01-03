/*
 * platform indepent driver interface
 * Copyright (C) 2016 Goodix
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <drm/drm_panel.h>
#include <linux/of.h>

#include "gf_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

/*
int parse_dt_panel(struct gf_dev *gf_dev)
{
	int i;
	int count;
    struct device_node *panel_node = NULL;
	struct drm_panel *panel;

    struct device *dev = &gf_dev->spi->dev;
	struct device_node *node = dev->of_node;
	pr_err("gf:parse_dt_panel start\n");
	count = of_count_phandle_with_args(node, "panel", NULL);
	pr_err("gf:parse_dt_panel , count = %d \n" , count);
	if (count <= 0)
		return -1;

	for (i = 0; i < count; i++) {
		panel_node = of_parse_phandle(node, "panel", i);
		if( panel_node == NULL ){
			pr_err("gf:parse_dt_panel ,panel_node == NULL\n");
		}
		panel = of_drm_find_panel(panel_node);
		if( panel == NULL ){
			pr_err("gf:parse_dt_panel ,panel == NULL\n");
		}
		of_node_put(panel_node);
		pr_err("gf:parse_dt_panel ---64---\n");
		if (!IS_ERR(panel)) {
			gf_dev->active_panel_asus = panel;
			pr_err("gf:parse_dt_panel , return 0\n");
			return 0;
		}else{
			pr_err("gf:parse_dt_panel , %d\n",PTR_ERR(panel));
		}
	}
    pr_err("gf:parse_dt_panel end\n");
	return 0;
}
*/

int gf_parse_dts(struct gf_dev *gf_dev)
{
	int rc = 0;
	struct device *dev = &gf_dev->spi->dev;
	struct device_node *np = dev->of_node;

	pr_info("[GF][%s] +++\n", __func__);

	// request irq gpio
	gf_dev->irq_gpio = of_get_named_gpio(np, "fp-gpio-irq", 0);
	if (gf_dev->irq_gpio < 0) {
		pr_err("[GF][%s] falied to get irq gpio!\n", __func__);
		return gf_dev->irq_gpio;
	}
	rc = devm_gpio_request(dev, gf_dev->irq_gpio, "goodix_irq");
	if (rc) {
		pr_err("[GF][%s] failed to request irq gpio, rc = %d\n", __func__, rc);
		goto err_irq;
	}
	gpio_direction_input(gf_dev->irq_gpio);
	pr_info("[GF][%s] gf_dev->irq_gpio = %d\n", __func__, gf_dev->irq_gpio);

	// request reset gpio
	gf_dev->reset_gpio = of_get_named_gpio(np, "fp-gpio-reset", 0);
	if (gf_dev->reset_gpio < 0) {
		pr_err("[GF][%s] falied to get reset gpio!\n", __func__);
		return gf_dev->reset_gpio;
	}
	rc = devm_gpio_request(dev, gf_dev->reset_gpio, "goodix_reset");
	if (rc) {
		pr_err("[GF][%s] failed to request reset gpio, rc = %d\n", __func__, rc);
		goto err_irq;
	}
	gpio_direction_output(gf_dev->reset_gpio, 0);
	pr_info("[GF][%s] gf_dev->reset_gpio = %d\n", __func__, gf_dev->reset_gpio);

	pr_info("[GF][%s] ---\n", __func__);
	return rc;
err_irq:
	devm_gpio_free(dev, gf_dev->irq_gpio);

	return rc;
}

void gf_cleanup(struct gf_dev *gf_dev)
{
	pr_info("[info] %s\n", __func__);

	if (gpio_is_valid(gf_dev->irq_gpio)) {
		gpio_free(gf_dev->irq_gpio);
		pr_info("remove irq_gpio success\n");
	}
	if (gpio_is_valid(gf_dev->reset_gpio)) {
		gpio_free(gf_dev->reset_gpio);
		pr_info("remove reset_gpio success\n");
	}
}

int gf_power_on(struct gf_dev *gf_dev)
{
	int rc = 0;
	int ret = 0;

	pr_err("[GF][%s] +++\n", __func__);

	/* TODO: add your power control here */
	gf_dev->vcc = regulator_get(&gf_dev->spi->dev, "vcc");
	if (regulator_count_voltages(gf_dev->vcc) > 0) {
		ret = regulator_set_voltage(gf_dev->vcc, 3000000, 3000000);
		if (ret) {
			pr_err("[GF][%s] pm8010j ldo3 regulator set_vtg failed, ret=%d\n", __func__, ret);
			regulator_put(gf_dev->vcc);
		}
	}
	ret = regulator_enable(gf_dev->vcc);
	if (ret) {
		pr_err("[GF][%s] enable pm8010j ldo3 regulator failed, ret=%d\n", __func__, ret);
	} else {
		pr_info("[GF][%s] enable pm8010j ldo3 regulator successfully!\n", __func__);
	}

	msleep(15);

	if (gpio_is_valid(gf_dev->reset_gpio)) {
		rc = gpio_direction_output(gf_dev->reset_gpio, 1);
		if (rc)
			pr_info("[GF][%s] reset gpio enable failed, rc = %d\n", __func__, rc);
		else
			pr_err("[GF][%s] reset gpio enable!\n", __func__);
	}

	msleep(3);
	gpio_set_value(gf_dev->reset_gpio, 1);

	pr_err("[GF][%s] ---\n", __func__);
	return rc;
}

int gf_power_off(struct gf_dev *gf_dev)
{
	int rc = 0;
	int ret = 0;

	if (!gf_dev) {
		pr_err("[GF][%s] Input buff is NULL.\n", __func__);
		return -ENODEV;
	}

	//if (gf_dev->power_enabled) {
	//	gf_dev->power_enabled = 0;
		/* TODO: add your power control here */
		gpio_direction_output(gf_dev->reset_gpio, 0);
		msleep(10);
		gf_dev->vcc = regulator_get(&gf_dev->spi->dev, "vcc");
		ret = regulator_disable(gf_dev->vcc);
		if (ret) {
			pr_err("[GF][%s] disable pm8010j ldo3 regulator failed, ret=%d\n", __func__, ret);
		} else {
			pr_info("[GF][%s] disable pm8010j ldo3 regulator successfully!\n", __func__);
		}
	if (regulator_count_voltages(gf_dev->vcc) > 0)
		regulator_set_voltage(gf_dev->vcc, 0, 3000000);
	regulator_put(gf_dev->vcc);
	//}
	return rc;
}

int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
	if (!gf_dev) {
		pr_err("Input buff is NULL.\n");
		return -ENODEV;
	}
	pr_debug("[GF][%s] gf hw reset\n", __func__);
	gpio_direction_output(gf_dev->reset_gpio, 1);
	gpio_set_value(gf_dev->reset_gpio, 0);
	msleep(3);
	gpio_set_value(gf_dev->reset_gpio, 1);
	msleep(delay_ms);
	return 0;
}

int gf_irq_num(struct gf_dev *gf_dev)
{
	if (!gf_dev) {
		pr_err("Input buff is NULL.\n");
		return -ENODEV;
	} else {
		return gpio_to_irq(gf_dev->irq_gpio);
	}
}

// AI2202 BSP +++
static ssize_t gf_charger_mode_store(
    struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct gf_dev *gf_dev = &gf;
	int tmp = buf[0] - '0';
	pr_err("[GF][%s] store buf is %d\n", __func__, tmp);
	if (tmp == 1) {
		pr_info("[GF][%s] Is in charger mode, Power off the fingerprint sensor", __func__);
		gf_dev->irq = gf_irq_num(gf_dev);
		disable_irq(gf_dev->irq);
		gf_power_off(gf_dev);
		gf_dev->device_available = 0;
	}
	pr_err("[GF][%s] store buf is %d\n", __func__, tmp);
	return count;
}

static DEVICE_ATTR(gf_charger_mode, S_IRUGO | S_IWUSR, NULL, gf_charger_mode_store);
static struct attribute *gf_attributes[] = {
    &dev_attr_gf_charger_mode.attr,
    NULL
};

static struct attribute_group asus_gf_attribute_group = {
    .attrs = gf_attributes
};

int asus_gf_create_sysfs(struct gf_dev *gf_dev)
{
	int ret = 0;

	ret = sysfs_create_group(&gf_dev->spi->dev.kobj, &asus_gf_attribute_group);
	if (ret) {
		pr_err("[GF][%s] asus_create_group() failed!!\n", __func__);
		sysfs_remove_group(&gf_dev->spi->dev.kobj, &asus_gf_attribute_group);
		return -ENOMEM;
	} else {
		pr_info("[GF][%s] asus_create_group() succeeded!!\n", __func__);
	}

	return ret;
}

int asus_gf_remove_sysfs(struct gf_dev *gf_dev)
{
    sysfs_remove_group(&gf_dev->spi->dev.kobj, &asus_gf_attribute_group);
    return 0;
}
// AI2202 BSP ---
