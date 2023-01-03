/*
 * drivers/usb/misc/tusb212.c - tusb212 usb redriver driver
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/role.h>

/* DEFINE REGISTERs */
#define ACB_LVL     0x01
#define CFG_ACTIVE  0x03
#define DCB_LVL     0x0E
#define RX_SEN      0x25

#define TUSB212_I2C_NAME "tusb212"
#define tusb212_addr 0x2C

enum tusb212_mode{
	TUSB212_DISABLE = 0,
	TUSB212_ENABLE = 1,
};

struct gpio_control {
	u32 REDRIVER_RESET;
	//u32 REDRIVER_CD;
};


struct tusb212 {
	struct device		*dev;

	/* debugfs entries */
	struct dentry		*root;
	u8			reg_ACB_LVL;
	u8			reg_CFG_ACTIVE;
	u8			reg_DCB_LVL;
	u8			reg_RX_SEN;
};

int current_redriver_mode;
static int current_redriver_vcc;

struct i2c_client *redriver_client;
struct gpio_control *redriver_gpio_ctrl;
struct tusb212 *redriver_tusb212;
struct regulator *vcc_redriver;

static int tusb212_register_write(char reg, u8 data);
static int tusb212_register_read(char reg, char *data);
static void tusb212_gpio_init(struct tusb212 *redriver);
static int tusb212_reset_n(int val);
int tusb212_enable(int mode);
enum usb_role usb_btm_role;

void tusb212_set_usb_btm_role(enum usb_role role)
{
	usb_btm_role = role;
}EXPORT_SYMBOL_GPL(tusb212_set_usb_btm_role);

static ssize_t mode_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (current_redriver_mode == TUSB212_DISABLE)
		return sprintf(buf, "%s", "disable\n");
	else if (current_redriver_mode == TUSB212_ENABLE)
		return sprintf(buf, "%s", "enable\n");

	return 0;
}
static ssize_t mode_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	if (!strncmp(buf, "enable", 6)) {
		tusb212_enable(TUSB212_ENABLE);
		pr_info("tusb212 driver enable\n");
	} else if (!strncmp(buf, "disable", 7)) {
		tusb212_enable(TUSB212_DISABLE);
		pr_info("tusb212 driver disable\n");
	}

	return size;
}
static DEVICE_ATTR_RW(mode);

static ssize_t ACB_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	char acb;
	tusb212_register_read(ACB_LVL, &acb);
	pr_info("%s ACB_LVL: 0x%02x \n", __func__, acb);
	return sprintf(buf, "0x%02x", acb);
}
static DEVICE_ATTR_RO(ACB);

static ssize_t DCB_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	char dcb;
	tusb212_register_read(DCB_LVL, &dcb);
	pr_info("%s DCB_LVL: 0x%02x \n", __func__, dcb);
	return sprintf(buf, "0x%02x", dcb);
}
static DEVICE_ATTR_RO(DCB);

static int tusb212_register_write(char reg, u8 data)
{
	int ret;
	char buf[2];
	struct i2c_msg msg[] = {
		{
			.addr = redriver_client->addr,
			.flags = 0,
			.len = 2,
			.buf = buf,
		},
	};

	buf[0] = reg;
	buf[1] = data;

	ret = i2c_transfer(redriver_client->adapter, msg, 1);

	if (ret < 0)
		printk("%s: failed to write i2c addr=%x reg=%x\n",	__func__, redriver_client->addr, reg);

	return ret;
}

static int tusb212_register_read(char reg, char *data)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr = redriver_client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = redriver_client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = data,
		},
	};

	ret = i2c_transfer(redriver_client->adapter, msgs, 2);

	if (ret < 0)
		printk("%s: failed to read i2c addr=%x reg=%x\n",	__func__, redriver_client->addr, reg);

	return ret;
}

static void tusb212_create_debugfs(struct tusb212 *redriver)
{
	redriver->root = debugfs_create_dir(dev_name(redriver->dev), NULL);
	debugfs_create_x8("reg_ACB_LVL", 0644, redriver->root, &redriver->reg_ACB_LVL);
	//debugfs_create_x8("reg_CFG_ACTIVE", 0644, redriver->root, &redriver->reg_CFG_ACTIVE);
	debugfs_create_x8("reg_DCB_LVL", 0644, redriver->root, &redriver->reg_DCB_LVL);
	debugfs_create_x8("reg_RX_SEN", 0644, redriver->root, &redriver->reg_RX_SEN);
}

static int tusb212_reset_n(int val)
{
	int ret = 0;

	pr_info("%s %d\n", __func__, val);
	ret = gpio_direction_output(redriver_gpio_ctrl->REDRIVER_RESET, val);
	if (ret) {
		pr_err("tusb212 failed to control REDRIVER_RESET\n");
	}
	return 0;
}

int tusb212_enable(int mode)
{
	int ret = 0;
	char data,acb,dcb,rx;
	char cfg;

	pr_info("%s %d\n", __func__, mode);
	if(!redriver_tusb212){
		pr_err("%s tusb212 not ready!\n", __func__);
		return 0;
	}

	switch (mode) {
		case TUSB212_DISABLE:
			pr_info("%s disable redriver, close vcc, reset high\n", __func__);
			//redriver disable +++
			if (!IS_ERR_OR_NULL(vcc_redriver) && current_redriver_vcc == 1) {
					ret = regulator_disable(vcc_redriver);
				if (ret) {
					pr_err("usb2_redriver, disable vcc_redriver regulator failed, ret=%d\n", ret);
				} else {
					pr_info("usb2_redriver, vcc_redriver disable\n");
					current_redriver_vcc = 0;
				}
			}
			tusb212_reset_n(1);
			//redriver disable ---

			current_redriver_mode=0;
			break;
		case TUSB212_ENABLE:
			pr_info("%s enable redriver, open vcc, reset low, restore parameters\n", __func__);

			//redriver enable +++
			if (!IS_ERR_OR_NULL(vcc_redriver) && current_redriver_vcc == 0) {
				ret = regulator_enable(vcc_redriver);
				if (ret) {
					pr_err("usb2_redriver, enable vcc_redriver regulator failed, ret=%d\n", ret);
				} else {
					pr_info("usb2_redriver, vcc_redriver enable\n");
					current_redriver_vcc = 1;
				}
			}
			usleep_range(100, 200); //Tstable > 100us
			tusb212_reset_n(0);
			//redriver enable ---

			msleep(200);

			tusb212_register_read(CFG_ACTIVE, &cfg);
			tusb212_register_write(CFG_ACTIVE, cfg|=1);
			tusb212_register_read(CFG_ACTIVE, &cfg);
			pr_info("[%s] config mode, CFG_ACTIVE: 0x%x \n", __func__, cfg);

			//read default and set tuning eye diagram param
			tusb212_register_read(ACB_LVL, &acb);
			pr_info("[%s] default ACB_LVL: 0x%x \n", __func__, acb);
			tusb212_register_read(DCB_LVL, &dcb);
			pr_info("[%s] default DCB_LVL: 0x%x \n", __func__, dcb);
			tusb212_register_read(RX_SEN, &rx);
			pr_info("[%s] default RX_SEN: 0x%x \n", __func__, rx);

			if(dcb==0x6){ //tusb216
				pr_info("[%s] tusb216\n", __func__);
				if (usb_btm_role == USB_ROLE_HOST){
					pr_info("[%s] usb_btm_role == USB_ROLE_HOST\n", __func__);
					tusb212_register_write(ACB_LVL, (acb&~(0xf0))|(0xf0));
					tusb212_register_read(ACB_LVL, &acb);
					pr_info("[%s] tune ACB_LVL: 0x%x \n", __func__, acb);

					tusb212_register_write(DCB_LVL, (dcb&~(0x0f))|(0x06));
					tusb212_register_read(DCB_LVL, &dcb);
					pr_info("[%s] tune DCB_LVL: 0x%x \n", __func__, dcb);
				} else if (usb_btm_role == USB_ROLE_DEVICE){
					pr_info("[%s] usb_btm_role == USB_ROLE_DEVICE\n", __func__);
					tusb212_register_write(ACB_LVL, (acb&~(0xf0))|(0x30));
					tusb212_register_read(ACB_LVL, &acb);
					pr_info("[%s] tune ACB_LVL: 0x%x \n", __func__, acb);

					tusb212_register_write(DCB_LVL, (dcb&~(0x0f))|(0x06));
					tusb212_register_read(DCB_LVL, &dcb);
					pr_info("[%s] tune DCB_LVL: 0x%x \n", __func__, dcb);
				}
			} else if(dcb==0xb){ //tusb212
				pr_info("[%s] tusb212\n", __func__);
				if (usb_btm_role == USB_ROLE_HOST){
					pr_info("[%s] usb_btm_role == USB_ROLE_HOST\n", __func__);
					tusb212_register_write(ACB_LVL, (acb&~(0x70))|(0x78));
					tusb212_register_read(ACB_LVL, &acb);
					pr_info("[%s] tune ACB_LVL: 0x%x \n", __func__, acb);

					tusb212_register_write(DCB_LVL, (dcb&~(0x07))|(0x0b));
					tusb212_register_read(DCB_LVL, &dcb);
					pr_info("[%s] tune DCB_LVL: 0x%x \n", __func__, dcb);
				} else if (usb_btm_role == USB_ROLE_DEVICE){
					pr_info("[%s] usb_btm_role == USB_ROLE_DEVICE\n", __func__);
					tusb212_register_write(ACB_LVL, (acb&~(0x70))|(0x08));
					tusb212_register_read(ACB_LVL, &acb);
					pr_info("[%s] tune ACB_LVL: 0x%x \n", __func__, acb);

					tusb212_register_write(DCB_LVL, (dcb&~(0x07))|(0x0b));
					tusb212_register_read(DCB_LVL, &dcb);
					pr_info("[%s] tune DCB_LVL: 0x%x \n", __func__, dcb);
				}
			} else if(dcb==0xd){ //tusb214
				pr_info("[%s] tusb214\n", __func__);
				if (usb_btm_role == USB_ROLE_HOST){
					pr_info("[%s] usb_btm_role == USB_ROLE_HOST\n", __func__);
					tusb212_register_write(ACB_LVL, (acb&~(0x70))|(0x78));
					tusb212_register_read(ACB_LVL, &acb);
					pr_info("[%s] tune ACB_LVL: 0x%x \n", __func__, acb);

					tusb212_register_write(DCB_LVL, (dcb&~(0x07))|(0x0b));
					tusb212_register_read(DCB_LVL, &dcb);
					pr_info("[%s] tune DCB_LVL: 0x%x \n", __func__, dcb);
				} else if (usb_btm_role == USB_ROLE_DEVICE){
					pr_info("[%s] usb_btm_role == USB_ROLE_DEVICE\n", __func__);
					tusb212_register_write(ACB_LVL, (acb&~(0x70))|(0x08));
					tusb212_register_read(ACB_LVL, &acb);
					pr_info("[%s] tune ACB_LVL: 0x%x \n", __func__, acb);

					tusb212_register_write(DCB_LVL, (dcb&~(0x07))|(0x0b));
					tusb212_register_read(DCB_LVL, &dcb);
					pr_info("[%s] tune DCB_LVL: 0x%x \n", __func__, dcb);
				}
			}

			//overide eye diagram param
			if (redriver_tusb212->reg_ACB_LVL) {
				tusb212_register_write(ACB_LVL, redriver_tusb212->reg_ACB_LVL);
				tusb212_register_read(ACB_LVL, &data);
				pr_info("[%s] overide ACB_LVL: 0x%x\n", __func__, data);
			}
			if (redriver_tusb212->reg_DCB_LVL) {
				tusb212_register_write(DCB_LVL, redriver_tusb212->reg_DCB_LVL);
				tusb212_register_read(DCB_LVL, &data);
				pr_info("[%s] overide DCB_LVL: 0x%x\n", __func__, data);
			}
			if (redriver_tusb212->reg_RX_SEN) {
				tusb212_register_write(RX_SEN, redriver_tusb212->reg_RX_SEN);
				tusb212_register_read(RX_SEN, &data);
				pr_info("[%s] overide RX_SEN: 0x%x\n", __func__, data);
			}
			tusb212_register_write(CFG_ACTIVE, cfg&=~1);
			tusb212_register_read(CFG_ACTIVE, &cfg);
			pr_info("[%s] normal mode, CFG_ACTIVE: 0x%x \n", __func__, cfg);

			current_redriver_mode=1;
			break;
		}

	pr_info("%s done\n", __func__);
	return ret;
}
EXPORT_SYMBOL(tusb212_enable);

static void tusb212_gpio_init(struct tusb212 *redriver)
{
	int ret = 0;

	pr_info("%s\n", __func__);
	redriver_gpio_ctrl->REDRIVER_RESET = of_get_named_gpio(redriver->dev->of_node, "REDRIVER_RESET", 0);
	ret = gpio_request(redriver_gpio_ctrl->REDRIVER_RESET, "REDRIVER_RESET");
	if (ret)
		pr_info("%s, failed to request REDRIVER_RESET\n", __func__);

}

int tusb212_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tusb212 *redriver;
	int err = 0;

	pr_info("%s\n", __func__);

	//i2c init
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		pr_err("%s i2c bus does not support tusb212\n", __func__);
	}

	redriver = devm_kzalloc(&client->dev, sizeof(struct tusb212), GFP_KERNEL);
	if (!redriver)
		return -ENOMEM;

	redriver_client = client;
	i2c_set_clientdata(client, redriver);
	redriver_client->addr = tusb212_addr;

	redriver->dev = &client->dev;

	//gpio init
	redriver_gpio_ctrl = devm_kzalloc(&client->dev, sizeof(*redriver_gpio_ctrl), GFP_KERNEL);
	if (!redriver_gpio_ctrl)
		return -ENOMEM;

	tusb212_gpio_init(redriver);

	//vcc init
	vcc_redriver = devm_regulator_get(redriver->dev, "vcc_redriver");
	if (!IS_ERR_OR_NULL(vcc_redriver)) {
		if (regulator_count_voltages(vcc_redriver) > 0) {
			err = regulator_set_voltage(vcc_redriver, 3300000, 3300000);
			if (err) {
				pr_err("%s vcc_redriver regulator set_vtg failed, err=%d\n", __func__, err);
				regulator_put(vcc_redriver);
			}
		}
	} else {
		pr_err("%s vcc_redriver IS_ERR_OR_NULL\n", __func__);
	}
	current_redriver_vcc = 0;

	pr_info("%s redriver default disable\n", __func__);
	tusb212_enable(TUSB212_DISABLE);

	err = device_create_file(&client->dev, &dev_attr_mode);
	err = device_create_file(&client->dev, &dev_attr_ACB);
	err = device_create_file(&client->dev, &dev_attr_DCB);

	tusb212_create_debugfs(redriver);

	redriver_tusb212 = redriver;

	return err;

}

static int tusb212_remove(struct i2c_client *client)
{
	return 0;
}

static const struct of_device_id tusb212_match_table[] = {
	{ .compatible = "redriver,tusb212",},
	{ },
};

static const struct i2c_device_id tusb212_id[] = {
	{ TUSB212_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver tusb212_driver = {
	.probe		= tusb212_probe,
	.remove		= tusb212_remove,
	.id_table = tusb212_id,
	.driver		= {
		.name		= TUSB212_I2C_NAME,
		.owner		= THIS_MODULE,
		.of_match_table = of_match_ptr(tusb212_match_table),
	},
};

static int __init tusb212_init(void)
{
	int ret = 0;

	ret = i2c_add_driver(&tusb212_driver);
	if (ret != 0)
		pr_err("tusb212 I2C add driver error ret %d\n", ret);
	return ret;
}
module_init(tusb212_init);

static void __exit tusb212_exit(void)
{
	i2c_del_driver(&tusb212_driver);
}
module_exit(tusb212_exit);

MODULE_DESCRIPTION("tusb212 USB redriver driver");
MODULE_LICENSE("GPL");
