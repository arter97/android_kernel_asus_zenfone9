// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/extcon/extcon.c - External Connector (extcon) framework.
 *
 * Copyright (C) 2015 Samsung Electronics
 * Author: Chanwoo Choi <cw00.choi@samsung.com>
 *
 * Copyright (C) 2012 Samsung Electronics
 * Author: Donggeun Kim <dg77.kim@samsung.com>
 * Author: MyungJoo Ham <myungjoo.ham@samsung.com>
 *
 * based on android/drivers/switch/switch_class.c
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#include "extcon-asus.h"

static struct class *extcon_class;


static ssize_t state_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct extcon_asus_dev *edev = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", edev->state);
}
static DEVICE_ATTR_RO(state);

static ssize_t name_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct extcon_asus_dev *edev = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", edev->name);
}
static DEVICE_ATTR_RO(name);


static bool asus_is_extcon_changed(struct extcon_asus_dev *edev, int new_state)
{
	return (edev->state != new_state);
}

int asus_extcon_sync(struct extcon_asus_dev *edev)
{
	char name_buf[120];
	char state_buf[120];
	char *prop_buf;
	char *envp[3];
	int env_offset = 0;
	int length;
	unsigned long flags;

	if (!edev)
		return -EINVAL;

	spin_lock_irqsave(&edev->lock, flags);

	/* This could be in interrupt handler */
	prop_buf = (char *)get_zeroed_page(GFP_ATOMIC);
	if (!prop_buf) {
		/* Unlock early before uevent */
		spin_unlock_irqrestore(&edev->lock, flags);

		dev_err(&edev->dev, "out of memory in extcon_set_state\n");
		kobject_uevent(&edev->dev.kobj, KOBJ_CHANGE);

		return -ENOMEM;
	}

	length = name_show(&edev->dev, NULL, prop_buf);
	if (length > 0) {
		if (prop_buf[length - 1] == '\n')
			prop_buf[length - 1] = 0;
		snprintf(name_buf, sizeof(name_buf), "NAME=%s", prop_buf);
		envp[env_offset++] = name_buf;
	}

	length = state_show(&edev->dev, NULL, prop_buf);
	if (length > 0) {
		if (prop_buf[length - 1] == '\n')
			prop_buf[length - 1] = 0;
		snprintf(state_buf, sizeof(state_buf), "STATE=%s", prop_buf);
		envp[env_offset++] = state_buf;
	}
	envp[env_offset] = NULL;

	/* Unlock early before uevent */
	spin_unlock_irqrestore(&edev->lock, flags);
	kobject_uevent_env(&edev->dev.kobj, KOBJ_CHANGE, envp);
	free_page((unsigned long)prop_buf);

	return 0;
}
EXPORT_SYMBOL_GPL(asus_extcon_sync);

bool boot_completed_flag = 1;
int asus_extcon_set_state(struct extcon_asus_dev *edev, int cable_state)
{
	unsigned long flags;

	if (!edev)
		return -EINVAL;

	spin_lock_irqsave(&edev->lock, flags);

	/* Check whether the external connector's state is changed. */
	if (!asus_is_extcon_changed(edev, cable_state) || !boot_completed_flag)
		goto out;

	/* Don't check mutual exclusiveness & property since the state no longer represents multi-cable. */
	/* Update the state for a external connector. */
	edev->state = cable_state;
out:
	spin_unlock_irqrestore(&edev->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(asus_extcon_set_state);

int asus_extcon_set_state_sync(struct extcon_asus_dev *edev, int cable_state)
{
	int ret;
	unsigned long flags;

	if (edev == NULL) {
		printk("%s: Skip to set extcon(edev is NULL). \n", __func__);
		return 0;
	}
	/* Check whether the external connector's state is changed. */
	spin_lock_irqsave(&edev->lock, flags);
	ret = asus_is_extcon_changed(edev, cable_state);
	spin_unlock_irqrestore(&edev->lock, flags);
	if (!ret)
		return 0;

	printk("[BAT][CHG] asus_extcon_set_state %d\n", cable_state);
	ret = asus_extcon_set_state(edev, cable_state);
	if (ret < 0)
		return ret;

	return asus_extcon_sync(edev);
}
EXPORT_SYMBOL_GPL(asus_extcon_set_state_sync);



static struct attribute *extcon_attrs[] = {
	&dev_attr_state.attr,
	&dev_attr_name.attr,
	NULL,
};
ATTRIBUTE_GROUPS(extcon);

static int create_extcon_class(void)
{
	if (!extcon_class) {
		extcon_class = class_create(THIS_MODULE, "extcon-asus");
		if (IS_ERR(extcon_class))
			return PTR_ERR(extcon_class);
		extcon_class->dev_groups = extcon_groups;
	}

	return 0;
}

static void extcon_asus_dev_release(struct device *dev)
{
}


/*
 * extcon_asus_dev_allocate() - Allocate the memory of extcon device.
 *
 * Note that this function allocates the memory for extcon device
 * and initialize default setting for the extcon device.
 *
 * Returns the pointer memory of allocated extcon_asus_dev if success
 * or ERR_PTR(err) if fail.
 */
struct extcon_asus_dev *extcon_asus_dev_allocate(void)
{
	struct extcon_asus_dev *edev;

printk("extcon_asus_dev_allocate ++");
	edev = kzalloc(sizeof(*edev), GFP_KERNEL);
	if (!edev)
		return ERR_PTR(-ENOMEM);


	return edev;
}
EXPORT_SYMBOL_GPL(extcon_asus_dev_allocate);

/*
 * extcon_asus_dev_free() - Free the memory of extcon device.
 * @edev:	the extcon device
 */
void extcon_asus_dev_free(struct extcon_asus_dev *edev)
{
	kfree(edev);
}
EXPORT_SYMBOL_GPL(extcon_asus_dev_free);

/**
 * extcon_asus_dev_register() - Register an new extcon device
 * @edev:	the extcon device to be registered
 *
 *
 * Returns 0 if success or error number if fail.
 */
int extcon_asus_dev_register(struct extcon_asus_dev *edev)
{
	int ret;
	static atomic_t edev_no = ATOMIC_INIT(-1);

	if (!extcon_class) {
		ret = create_extcon_class();
		if (ret < 0)
			return ret;
	}

	edev->dev.class = extcon_class;
	edev->dev.release = extcon_asus_dev_release;


	dev_set_name(&edev->dev, edev->name,
		(unsigned long)atomic_inc_return(&edev_no));

	ret = device_register(&edev->dev);
	if (ret) {
		put_device(&edev->dev);
		goto err_dev;
	}

	spin_lock_init(&edev->lock);

	dev_set_drvdata(&edev->dev, edev);
	edev->state = 0;

	return 0;

err_dev:
	return ret;
}
EXPORT_SYMBOL_GPL(extcon_asus_dev_register);

/**
 * extcon_asus_dev_unregister() - Unregister the extcon device.
 * @edev:	the extcon device to be unregistered.
 *
 * Note that this does not call kfree(edev) because edev was not allocated
 * by this class.
 */
void extcon_asus_dev_unregister(struct extcon_asus_dev *edev)
{
	if (!edev)
		return;

	if (IS_ERR_OR_NULL(get_device(&edev->dev))) {
		dev_err(&edev->dev, "Failed to unregister extcon_asus_dev (%s)\n",
				dev_name(&edev->dev));
		return;
	}

	device_unregister(&edev->dev);


	put_device(&edev->dev);
}
EXPORT_SYMBOL_GPL(extcon_asus_dev_unregister);

int asus_extcon_get_state(struct extcon_asus_dev *edev)
{
	int state = edev->state;

	if (!edev)
		return -EINVAL;

	return state;
}
EXPORT_SYMBOL_GPL(asus_extcon_get_state);



static int __init extcon_asus_class_init(void)
{
	return create_extcon_class();
}
module_init(extcon_asus_class_init);

static void __exit extcon_asus_class_exit(void)
{
	class_destroy(extcon_class);
}
module_exit(extcon_asus_class_exit);

MODULE_AUTHOR("Chanwoo Choi <cw00.choi@samsung.com>");
MODULE_AUTHOR("MyungJoo Ham <myungjoo.ham@samsung.com>");
MODULE_DESCRIPTION("External Connector (extcon) framework");
MODULE_LICENSE("GPL v2");
