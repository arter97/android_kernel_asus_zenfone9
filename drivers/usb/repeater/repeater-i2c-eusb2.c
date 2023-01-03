// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/qti-regmap-debugfs.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/usb/repeater.h>
#include <linux/usb/role.h>

#define EUSB2_3P0_VOL_MIN			3075000 /* uV */
#define EUSB2_3P0_VOL_MAX			3300000 /* uV */
#define EUSB2_3P0_HPM_LOAD			3500	/* uA */

#define EUSB2_1P8_VOL_MIN			1800000 /* uV */
#define EUSB2_1P8_VOL_MAX			1800000 /* uV */
#define EUSB2_1P8_HPM_LOAD			32000	/* uA */

/* NXP eUSB2 repeater registers */
#define RESET_CONTROL			0x01
#define LINK_CONTROL1			0x02
#define LINK_CONTROL2			0x03
#define eUSB2_RX_CONTROL		0x04
#define eUSB2_TX_CONTROL		0x05
#define USB2_RX_CONTROL			0x06
#define USB2_TX_CONTROL1		0x07
#define USB2_TX_CONTROL2		0x08
#define USB2_HS_TERMINATION		0x09
#define RAP_SIGNATURE			0x0D
#define VDX_CONTROL			0x0E
#define DEVICE_STATUS			0x0F
#define LINK_STATUS			0x10
#define REVISION_ID			0x13
#define CHIP_ID_0			0x14
#define CHIP_ID_1			0x15
#define CHIP_ID_2			0x16

/* TI eUSB2 repeater registers */
#define GPIO0_CONFIG			0x00
#define GPIO1_CONFIG			0x40
#define UART_PORT1			0x50
#define EXTRA_PORT1			0x51
#define U_TX_ADJUST_PORT1		0x70
#define U_HS_TX_PRE_EMPHASIS_P1		0x71
#define U_RX_ADJUST_PORT1		0x72
#define U_DISCONNECT_SQUELCH_PORT1	0x73
#define E_HS_TX_PRE_EMPHASIS_P1		0x77
#define E_TX_ADJUST_PORT1		0x78
#define E_RX_ADJUST_PORT1		0x79
#define REV_ID				0xB0
#define GLOBAL_CONFIG			0xB2
#define INT_ENABLE_1			0xB3
#define INT_ENABLE_2			0xB4
#define BC_CONTROL			0xB6
#define BC_STATUS_1			0xB7
#define INT_STATUS_1			0xA3
#define INT_STATUS_2			0xA4

enum usb_role usb_current_role;

void repeater_set_usb_role(enum usb_role role)
{
	usb_current_role = role;
}EXPORT_SYMBOL_GPL(repeater_set_usb_role);

enum eusb2_repeater_type {
	TI_REPEATER,
	NXP_REPEATER,
};

struct i2c_repeater_chip {
	enum eusb2_repeater_type repeater_type;
};

struct eusb2_repeater {
	struct device			*dev;
	struct usb_repeater		ur;
	struct regmap			*regmap;
	const struct i2c_repeater_chip	*chip;
	u16				reg_base;
	struct regulator		*vdd18;
	struct regulator		*vdd3;
	bool				power_enabled;

	struct gpio_desc		*reset_gpiod;
	u32				*param_override_seq;
	u8				param_override_seq_cnt;
	u8 reg_addr;

	u8 override_RESET_CONTROL;//0x01
	u8 override_LINK_CONTROL1;//0x02
	u8 override_LINK_CONTROL2;//0x03
	u8 override_eUSB2_RX_CONTROL;//0x04
	u8 override_eUSB2_TX_CONTROL;//0x05
	u8 override_USB2_RX_CONTROL;//0x06
	u8 override_USB2_TX_CONTROL1;//0x07
	u8 override_USB2_TX_CONTROL2;//0x08
	u8 override_USB2_HS_TERMINATION;//0x09
	u8 override_RAP_SIGNATURE;//0x0D
	u8 override_VDX_CONTROL;//0x0E
};

static const struct regmap_config eusb2_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
};

static int eusb2_i2c_read_reg(struct eusb2_repeater *er, u8 reg, u8 *val)
{
	int ret;
	unsigned int reg_val;

	ret = regmap_read(er->regmap, reg, &reg_val);
	if (ret < 0) {
		dev_err(er->dev, "[eusb2_repeater] Failed to read reg:0x%02x ret=%d\n", reg, ret);
		return ret;
	}

	*val = reg_val;
	dev_info(er->dev, "[eusb2_repeater] read reg:0x%02x val:0x%02x\n", reg, *val);

	return 0;
}

static int eusb2_i2c_write_reg(struct eusb2_repeater *er, u8 reg, u8 val)
{
	int ret;

	ret = regmap_write(er->regmap, reg, val);
	if (ret < 0) {
		dev_err(er->dev, "[eusb2_repeater] failed to write 0x%02x to reg: 0x%02x ret=%d\n", val, reg, ret);
		return ret;
	}

	dev_info(er->dev, "[eusb2_repeater] write reg:0x%02x val:0x%02x\n", reg, val);

	return 0;
}

static ssize_t addr_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct eusb2_repeater *er = dev_get_drvdata(dev);

	return sprintf(buf, "0x%02x\n", er->reg_addr);
}
static ssize_t addr_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	struct eusb2_repeater *er = dev_get_drvdata(dev);

	sscanf(buf, "%x", &er->reg_addr);

	return size;
}
static DEVICE_ATTR_RW(addr);

static ssize_t data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct eusb2_repeater *er = dev_get_drvdata(dev);
	u8 reg_val;

	eusb2_i2c_read_reg(er, er->reg_addr, &reg_val);

	return sprintf(buf, "0x%02x\n", reg_val);
}
static ssize_t data_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	struct eusb2_repeater *er = dev_get_drvdata(dev);
	u8 reg_val;

	sscanf(buf, "%x", &reg_val);
	eusb2_i2c_write_reg(er, er->reg_addr, reg_val);

	if (er->reg_addr == RESET_CONTROL)//0x01
		er->override_RESET_CONTROL = reg_val;
	if (er->reg_addr == LINK_CONTROL1)//0x02
		er->override_LINK_CONTROL1 = reg_val;
	if (er->reg_addr == LINK_CONTROL2)//0x03
		er->override_LINK_CONTROL2 = reg_val;
	if (er->reg_addr == eUSB2_RX_CONTROL)//0x04
		er->override_eUSB2_RX_CONTROL = reg_val;
	if (er->reg_addr == eUSB2_TX_CONTROL)//0x05
		er->override_eUSB2_TX_CONTROL = reg_val;
	if (er->reg_addr == USB2_RX_CONTROL)//0x06
		er->override_USB2_RX_CONTROL = reg_val;
	if (er->reg_addr == USB2_TX_CONTROL1)//0x07
		er->override_USB2_TX_CONTROL1 = reg_val;
	if (er->reg_addr == USB2_TX_CONTROL2)//0x08
		er->override_USB2_TX_CONTROL2 = reg_val;
	if (er->reg_addr == USB2_HS_TERMINATION)//0x09
		er->override_USB2_HS_TERMINATION = reg_val;
	if (er->reg_addr == RAP_SIGNATURE)//0x0D
		er->override_RAP_SIGNATURE = reg_val;
	if (er->reg_addr == VDX_CONTROL)//0x0E
		er->override_VDX_CONTROL = reg_val;

	dev_info(er->dev, "[eusb2_repeater] data_store addr:0x%02x data:0x%02x\n", er->reg_addr, reg_val);

	return size;
}
static DEVICE_ATTR_RW(data);

static void eusb2_repeater_update_seq(struct eusb2_repeater *er, u32 *seq, u8 cnt)
{
	int i;

	dev_info(er->ur.dev, "[eusb2_repeater] param override seq count:%d\n", cnt);
	for (i = 0; i < cnt; i = i+2) {
		dev_info(er->ur.dev, "[eusb2_repeater] write 0x%02x to 0x%02x\n", seq[i], seq[i+1]);
		eusb2_i2c_write_reg(er, seq[i+1], seq[i]);
	}
}

static int eusb2_repeater_power(struct eusb2_repeater *er, bool on)
{
	int ret = 0;

	dev_dbg(er->ur.dev, "%s turn %s regulators. power_enabled:%d\n",
			__func__, on ? "on" : "off", er->power_enabled);

	if (er->power_enabled == on) {
		dev_dbg(er->ur.dev, "regulators are already ON.\n");
		return 0;
	}

	if (!on)
		goto disable_vdd3;

	ret = regulator_set_load(er->vdd18, EUSB2_1P8_HPM_LOAD);
	if (ret < 0) {
		dev_err(er->ur.dev, "Unable to set HPM of vdd12:%d\n", ret);
		goto err_vdd18;
	}

	ret = regulator_set_voltage(er->vdd18, EUSB2_1P8_VOL_MIN,
						EUSB2_1P8_VOL_MAX);
	if (ret) {
		dev_err(er->ur.dev,
				"Unable to set voltage for vdd18:%d\n", ret);
		goto put_vdd18_lpm;
	}

	ret = regulator_enable(er->vdd18);
	if (ret) {
		dev_err(er->ur.dev, "Unable to enable vdd18:%d\n", ret);
		goto unset_vdd18;
	}

	ret = regulator_set_load(er->vdd3, EUSB2_3P0_HPM_LOAD);
	if (ret < 0) {
		dev_err(er->ur.dev, "Unable to set HPM of vdd3:%d\n", ret);
		goto disable_vdd18;
	}

	ret = regulator_set_voltage(er->vdd3, EUSB2_3P0_VOL_MIN,
						EUSB2_3P0_VOL_MAX);
	if (ret) {
		dev_err(er->ur.dev,
				"Unable to set voltage for vdd3:%d\n", ret);
		goto put_vdd3_lpm;
	}

	ret = regulator_enable(er->vdd3);
	if (ret) {
		dev_err(er->ur.dev, "Unable to enable vdd3:%d\n", ret);
		goto unset_vdd3;
	}

	er->power_enabled = true;
	pr_debug("%s(): eUSB2 repeater egulators are turned ON.\n", __func__);
	return ret;

disable_vdd3:
	ret = regulator_disable(er->vdd3);
	if (ret)
		dev_err(er->ur.dev, "Unable to disable vdd3:%d\n", ret);

unset_vdd3:
	ret = regulator_set_voltage(er->vdd3, 0, EUSB2_3P0_VOL_MAX);
	if (ret)
		dev_err(er->ur.dev,
			"Unable to set (0) voltage for vdd3:%d\n", ret);

put_vdd3_lpm:
	ret = regulator_set_load(er->vdd3, 0);
	if (ret < 0)
		dev_err(er->ur.dev, "Unable to set (0) HPM of vdd3\n");

disable_vdd18:
	ret = regulator_disable(er->vdd18);
	if (ret)
		dev_err(er->ur.dev, "Unable to disable vdd18:%d\n", ret);

unset_vdd18:
	ret = regulator_set_voltage(er->vdd18, 0, EUSB2_1P8_VOL_MAX);
	if (ret)
		dev_err(er->ur.dev,
			"Unable to set (0) voltage for vdd18:%d\n", ret);

put_vdd18_lpm:
	ret = regulator_set_load(er->vdd18, 0);
	if (ret < 0)
		dev_err(er->ur.dev, "Unable to set LPM of vdd18\n");

	/* case handling when regulator turning on failed */
	if (!er->power_enabled)
		return -EINVAL;

err_vdd18:
	er->power_enabled = false;
	dev_dbg(er->ur.dev, "eUSB2 repeater's regulators are turned OFF.\n");
	return ret;
}

static int eusb2_repeater_init(struct usb_repeater *ur)
{
	struct eusb2_repeater *er =
			container_of(ur, struct eusb2_repeater, ur);
	const struct i2c_repeater_chip *chip = er->chip;
	u8 reg_val;

	switch (chip->repeater_type) {
	case TI_REPEATER:
		eusb2_i2c_read_reg(er, REV_ID, &reg_val);
		/* If the repeater revision is B1 disable auto-resume WA */
		if (reg_val == 0x03)
			ur->flags |= UR_AUTO_RESUME_SUPPORTED;
		break;
	case NXP_REPEATER:
		eusb2_i2c_read_reg(er, REVISION_ID, &reg_val);
		break;
	default:
		dev_err(er->ur.dev, "Invalid repeater\n");
	}

	dev_info(er->ur.dev, "eUSB2 repeater version = 0x%x ur->flags:0x%x\n", reg_val, ur->flags);

	/* override init sequence using devicetree based values */
	if (er->param_override_seq_cnt)
		eusb2_repeater_update_seq(er, er->param_override_seq,
					er->param_override_seq_cnt);

	if (usb_current_role == USB_ROLE_HOST) {
		dev_info(er->ur.dev, "[eusb2_repeater] hardcode override host params\n", __func__);
		eusb2_i2c_write_reg(er, USB2_TX_CONTROL1, 0x02); //0x07
		eusb2_i2c_write_reg(er, USB2_TX_CONTROL2, 0x62); //0x08
	}

	dev_info(er->ur.dev, "[eusb2_repeater] dynamic override params\n", __func__);
	if (er->override_RESET_CONTROL)//0x01
		eusb2_i2c_write_reg(er, RESET_CONTROL, er->override_RESET_CONTROL);
	if (er->override_LINK_CONTROL1)//0x02
		eusb2_i2c_write_reg(er, LINK_CONTROL1, er->override_LINK_CONTROL1);
	if (er->override_LINK_CONTROL2)//0x03
		eusb2_i2c_write_reg(er, LINK_CONTROL2, er->override_LINK_CONTROL2);
	if (er->override_eUSB2_RX_CONTROL)//0x04
		eusb2_i2c_write_reg(er, eUSB2_RX_CONTROL, er->override_eUSB2_RX_CONTROL);
	if (er->override_eUSB2_TX_CONTROL)//0x05
		eusb2_i2c_write_reg(er, eUSB2_TX_CONTROL, er->override_eUSB2_TX_CONTROL);
	if (er->override_USB2_RX_CONTROL)//0x06
		eusb2_i2c_write_reg(er, USB2_RX_CONTROL, er->override_USB2_RX_CONTROL);
	if (er->override_USB2_TX_CONTROL1)//0x07
		eusb2_i2c_write_reg(er, USB2_TX_CONTROL1, er->override_USB2_TX_CONTROL1);
	if (er->override_USB2_TX_CONTROL2)//0x08
		eusb2_i2c_write_reg(er, USB2_TX_CONTROL2, er->override_USB2_TX_CONTROL2);
	if (er->override_USB2_HS_TERMINATION)//0x09
		eusb2_i2c_write_reg(er, USB2_HS_TERMINATION, er->override_USB2_HS_TERMINATION);
	if (er->override_RAP_SIGNATURE)//0x0D
		eusb2_i2c_write_reg(er, RAP_SIGNATURE, er->override_RAP_SIGNATURE);
	if (er->override_VDX_CONTROL)//0x0E
		eusb2_i2c_write_reg(er, VDX_CONTROL, er->override_VDX_CONTROL);

	dev_info(er->ur.dev, "[eusb2_repeater] init done\n");

	return 0;
}

static int eusb2_repeater_reset(struct usb_repeater *ur, bool bring_out_of_reset)
{
	struct eusb2_repeater *er =
			container_of(ur, struct eusb2_repeater, ur);

	dev_info(ur->dev, "[eusb2_repeater] reset gpio:%s\n",
			bring_out_of_reset ? "assert" : "deassert");
	gpiod_set_value_cansleep(er->reset_gpiod, bring_out_of_reset);
	return 0;
}

static int eusb2_repeater_powerup(struct usb_repeater *ur)
{
	struct eusb2_repeater *er =
			container_of(ur, struct eusb2_repeater, ur);

	return eusb2_repeater_power(er, true);
}

static int eusb2_repeater_powerdown(struct usb_repeater *ur)
{
	struct eusb2_repeater *er =
			container_of(ur, struct eusb2_repeater, ur);

	return eusb2_repeater_power(er, false);
}

static struct i2c_repeater_chip repeater_chip[] = {
	[NXP_REPEATER] = {
		.repeater_type = NXP_REPEATER,
	},
	[TI_REPEATER] = {
		.repeater_type = TI_REPEATER,
	}
};

static const struct of_device_id eusb2_repeater_id_table[] = {
	{
		.compatible = "nxp,eusb2-repeater",
		.data = &repeater_chip[NXP_REPEATER]
	},
	{
		.compatible = "ti,eusb2-repeater",
		.data = &repeater_chip[TI_REPEATER]
	},
	{ },
};
MODULE_DEVICE_TABLE(of, eusb2_repeater_id_table);

static int eusb2_repeater_i2c_probe(struct i2c_client *client)
{
	struct eusb2_repeater *er;
	struct device *dev = &client->dev;
	const struct of_device_id *match;
	int ret = 0, num_elem;

	er = devm_kzalloc(dev, sizeof(*er), GFP_KERNEL);
	if (!er) {
		ret = -ENOMEM;
		goto err_probe;
	}

	er->dev = dev;
	match = of_match_node(eusb2_repeater_id_table, dev->of_node);
	er->chip = match->data;

	er->regmap = devm_regmap_init_i2c(client, &eusb2_i2c_regmap);
	if (!er->regmap) {
		dev_err(dev, "failed to allocate register map\n");
		ret = -EINVAL;
		goto err_probe;
	}

	devm_regmap_qti_debugfs_register(er->dev, er->regmap);
	i2c_set_clientdata(client, er);

	ret = of_property_read_u16(dev->of_node, "reg", &er->reg_base);
	if (ret < 0) {
		dev_err(dev, "failed to get reg base address:%d\n", ret);
		goto err_probe;
	}

	er->vdd3 = devm_regulator_get(dev, "vdd3");
	if (IS_ERR(er->vdd3)) {
		dev_err(dev, "unable to get vdd3 supply\n");
		ret = PTR_ERR(er->vdd3);
		goto err_probe;
	}

	er->vdd18 = devm_regulator_get(dev, "vdd18");
	if (IS_ERR(er->vdd18)) {
		dev_err(dev, "unable to get vdd18 supply\n");
		ret = PTR_ERR(er->vdd18);
		goto err_probe;
	}

	er->reset_gpiod = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(er->reset_gpiod)) {
		ret = PTR_ERR(er->reset_gpiod);
		goto err_probe;
	}

	num_elem = of_property_count_elems_of_size(dev->of_node, "qcom,param-override-seq",
				sizeof(*er->param_override_seq));
	if (num_elem > 0) {
		if (num_elem % 2) {
			dev_err(dev, "invalid param_override_seq_len\n");
			ret = -EINVAL;
			goto err_probe;
		}

		er->param_override_seq_cnt = num_elem;
		er->param_override_seq = devm_kcalloc(dev,
				er->param_override_seq_cnt,
				sizeof(*er->param_override_seq), GFP_KERNEL);
		if (!er->param_override_seq) {
			ret = -ENOMEM;
			goto err_probe;
		}

		ret = of_property_read_u32_array(dev->of_node,
				"qcom,param-override-seq",
				er->param_override_seq,
				er->param_override_seq_cnt);
		if (ret) {
			dev_err(dev, "qcom,param-override-seq read failed %d\n",
									ret);
			goto err_probe;
		}
	}


	er->ur.dev = dev;

	er->ur.init		= eusb2_repeater_init;
	er->ur.reset		= eusb2_repeater_reset;
	er->ur.powerup		= eusb2_repeater_powerup;
	er->ur.powerdown	= eusb2_repeater_powerdown;

	ret = usb_add_repeater_dev(&er->ur);
	if (ret)
		goto err_probe;

	ret = device_create_file(&client->dev, &dev_attr_addr);
	ret = device_create_file(&client->dev, &dev_attr_data);

	dev_info(dev, "[eusb2_repeater] probe done\n");
	return ret;

err_probe:
	return ret;
}

static int eusb2_repeater_i2c_remove(struct i2c_client *client)
{
	struct eusb2_repeater *er = i2c_get_clientdata(client);

	if (!er)
		return 0;

	usb_remove_repeater_dev(&er->ur);
	eusb2_repeater_power(er, false);
	return 0;
}

static struct i2c_driver eusb2_i2c_repeater_driver = {
	.probe_new	= eusb2_repeater_i2c_probe,
	.remove		= eusb2_repeater_i2c_remove,
	.driver = {
		.name	= "eusb2-repeater",
		.of_match_table = of_match_ptr(eusb2_repeater_id_table),
	},
};

module_i2c_driver(eusb2_i2c_repeater_driver);

MODULE_DESCRIPTION("eUSB2 i2c repeater driver");
MODULE_LICENSE("GPL v2");
