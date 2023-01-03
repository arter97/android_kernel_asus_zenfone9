// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * cs35l45-i2c.c -- CS35L45 I2C driver
 *
 * Copyright 2019 Cirrus Logic, Inc.
 *
 * Author: James Schulman <james.schulman@cirrus.com>
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>

#include "wm_adsp.h"
#include "cs35l45.h"
#include <sound/cs35l45.h>

static int i2c_adapter_nr = -1;

static int cs35l45_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct cs35l45_private *cs35l45;
	struct device *dev = &client->dev;
	const struct i2c_adapter_quirks *quirks;
	int ret;
	int i = 0, retry_count = 50; /* ASUS_BSP Paul +++ */

	printk("%s: 1\n", __func__);

	/* ASUS_BSP Paul +++ */
	if (of_property_read_bool(dev->of_node, "asus,dummy")) {
		dev_err(dev, "dummy driver for EVB");
		return 0;
	}
	/* ASUS_BSP Paul --- */

	printk("%s: 2\n", __func__);
	cs35l45 = devm_kzalloc(dev, sizeof(struct cs35l45_private), GFP_KERNEL);
	if (cs35l45 == NULL)
		return -ENOMEM;

	printk("%s: 3\n", __func__);
	i2c_set_clientdata(client, cs35l45);
	cs35l45->regmap = devm_regmap_init_i2c(client, &cs35l45_i2c_regmap);
	if (IS_ERR(cs35l45->regmap)) {
		ret = PTR_ERR(cs35l45->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}

	printk("%s: 4\n", __func__);
	cs35l45->dev = dev;
	cs35l45->irq = client->irq;
	cs35l45->bus_type = CONTROL_BUS_I2C;
	cs35l45->i2c_addr = client->addr;
	i2c_adapter_nr = cs35l45->i2c_adapter_nr = client->adapter->nr;
	printk("%s: i2c_adapter_nr of cs35l45 = %d  Kernel driver version linux-5.10-20211008\n", __func__, cs35l45->i2c_adapter_nr);

	quirks = client->adapter->quirks;
	if (quirks != NULL)
		cs35l45->max_quirks_read_nwords = (int) quirks->max_read_len / 4;

	ret = cs35l45_probe(cs35l45);
	if (ret < 0) {
		dev_err(dev, "Failed device probe: %d\n", ret);
		return ret;
	}

	printk("%s: 5\n", __func__);
	usleep_range(2000, 2100);

	ret = cs35l45_initialize(cs35l45);
	/* ASUS_BSP Paul +++ */
	while (ret < 0 && i < retry_count) {
		msleep(100);
		ret = cs35l45_initialize(cs35l45);
		i++;
	}
	/* ASUS_BSP Paul --- */
	if (ret < 0) {
		dev_err(dev, "Failed device initialization: %d\n", ret);
		goto fail;
	}

	printk("%s: 6\n", __func__);
	if (ret < 0) {
		dev_err(dev, "Failed device initialization: %d , still return status OK for factory special usecase\n", ret); //Austin+++
		//return ret;
	}

	printk("%s: 7\n", __func__);
	return 0;
fail:
	cs35l45_remove(cs35l45);
	return ret;
}

int cs35l45_i2c_get_adapter_nr(int *nr)
{
	int ret = 0;

	if (i2c_adapter_nr < 0 || i2c_adapter_nr > 16) {
		ret = -1;
		printk("Failed to get i2c_adapter_nr: %d\n", ret);
	} else {
		*nr = i2c_adapter_nr;
		printk("cs35l45 i2c_adapter_nr:%d\n",*nr);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(cs35l45_i2c_get_adapter_nr);

static int cs35l45_i2c_remove(struct i2c_client *client)
{
	struct cs35l45_private *cs35l45 = i2c_get_clientdata(client);

	return cs35l45_remove(cs35l45);
}

static const struct of_device_id cs35l45_dt_match[] = {
	{.compatible = "cirrus,cs35l45"},
	{},
};
/* MODULE_DEVICE_TABLE(of, cs35l45_dt_match); */

static const struct i2c_device_id cs35l45_id_i2c[] = {
	{"cs35l45", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, cs35l45_id_i2c);

static struct i2c_driver cs35l45_i2c_driver = {
	.driver = {
		.name		= "cs35l45",
		.of_match_table = of_match_ptr(cs35l45_dt_match),
	},
	.id_table	= cs35l45_id_i2c,
	.probe		= cs35l45_i2c_probe,
	.remove		= cs35l45_i2c_remove,
};
/* module_i2c_driver(cs35l45_i2c_driver); */

static int __init cs35l45_i2c_init(void)
{
        int ret = 0;
printk("[cs35l45] cs35l45 I2C driver\n");
        ret = i2c_add_driver(&cs35l45_i2c_driver);
        if (ret) {
                printk("%s: fail to add cs35l45 device into i2c\n", __func__);
                return ret;
        }
printk("[cs35l45] cs35l45 I2C driver return\n");
        return 0;
}

module_init(cs35l45_i2c_init);

static void __exit cs35l45_i2c_exit(void)
{
        i2c_del_driver(&cs35l45_i2c_driver);
}

module_exit(cs35l45_i2c_exit);

MODULE_DESCRIPTION("I2C CS35L45 driver");
MODULE_AUTHOR("James Schulman, Cirrus Logic Inc, <james.schulman@cirrus.com>");
MODULE_LICENSE("GPL");
