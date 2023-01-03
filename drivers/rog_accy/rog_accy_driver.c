/* Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Edit by ASUS Deeo, deeo_ho@asus.com
 * V10
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/hidraw.h>
#include <linux/usb.h>
#include <linux/time.h>

#include "rog_accy_driver.h"

void rog_accy_uevent(void);

static u8 pogo_mutex_state = 0;

/*
 * 	gDongleType
 * 	- 1: Error
 * 	0 	: No Insert
 * 	1 	: InBox5
 *  2   : ERROR
 *  3   : Other
 * 	11 	: Fan Dongle 6
 * 255  : Default status
 */
uint8_t gDongleType=0;
EXPORT_SYMBOL(gDongleType);

/*
 * 	gPanelStatusForHdcpWork
 * 	0 	: Phone panel off
 * 	1 	: Phone panel on
 */
uint8_t gPanelStatusForHdcpWork=0;
EXPORT_SYMBOL(gPanelStatusForHdcpWork);

static ssize_t sync_state_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int ret = 0;
	u32 val;

	ret = kstrtou32(data, 10, &val);
	if (ret)
		return ret;

	printk("[ROG_ACCY][EXTCON] old state : %d, new state : %d\n", accy_extcon->state, val);
	switch(val) {
		case EXTRA_DOCK_STATE_UNDOCKED:
			printk("[ROG_ACCY][EXTCON] EXTRA_DOCK_STATE_UNDOCKED\n");
			asus_extcon_set_state_sync(accy_extcon, EXTRA_DOCK_STATE_UNDOCKED);
		break;
		case EXTRA_DOCK_STATE_ASUS_INBOX:
			printk("[ROG_ACCY][EXTCON] EXTRA_DOCK_STATE_ASUS_INBOX\n");
			asus_extcon_set_state_sync(accy_extcon, EXTRA_DOCK_STATE_ASUS_INBOX);
		break;
		case EXTRA_DOCK_STATE_ASUS_STATION:
			printk("[ROG_ACCY][EXTCON] EXTRA_DOCK_STATE_ASUS_STATION\n");
			asus_extcon_set_state_sync(accy_extcon, EXTRA_DOCK_STATE_ASUS_STATION);
		break;
		case EXTRA_DOCK_STATE_ASUS_DT:
			printk("[ROG_ACCY][EXTCON] EXTRA_DOCK_STATE_ASUS_DT\n");
			asus_extcon_set_state_sync(accy_extcon, EXTRA_DOCK_STATE_ASUS_DT);
		break;
		case EXTRA_DOCK_STATE_BACKGROUND_PAD:
			printk("[ROG_ACCY][EXTCON] EXTRA_DOCK_STATE_BACKGROUND_PAD\n");
			asus_extcon_set_state_sync(accy_extcon, EXTRA_DOCK_STATE_BACKGROUND_PAD);
		break;
		case EXTRA_DOCK_STATE_ASUS_2nd_INBOX:
			printk("[ROG_ACCY][EXTCON] EXTRA_DOCK_STATE_ASUS_2nd_INBOX\n");
			asus_extcon_set_state_sync(accy_extcon, EXTRA_DOCK_STATE_ASUS_2nd_INBOX);
		break;

		case EXTRA_DOCK_STATE_ASUS_OTHER:
		default:
			printk("[ROG_ACCY][EXTCON] EXTRA_DOCK_STATE_ASUS_OTHER\n");
			asus_extcon_set_state_sync(accy_extcon, EXTRA_DOCK_STATE_ASUS_OTHER);
		break;
	}

	pogo_mutex_state = 0;
	//printk("[ROG_ACCY] pogo_sema up!!! %d\n", val);
	//up(&g_rog_accy_data->pogo_sema);
	return count;
}

static ssize_t sync_state_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	printk("[ROG_ACCY][EXTCON] current state : %d\n", accy_extcon->state);

	return snprintf(buf, PAGE_SIZE,"%d\n", accy_extcon->state);
}

void rog_accy_uevent(void){

	u8 type;
	type = gDongleType;

	if (type == Dongle_default_status){
		printk("[ROG_ACCY] type = 255, fake dongle type\n");
		return;
	}

	if (pogo_mutex_state && type == Dongle_NO_INSERT){
		printk("[ROG_ACCY] type : %d, pogo_mutex_state : %d, force unlock!!\n", type, pogo_mutex_state);
		pogo_mutex_state = 0;
		//printk("[ROG_ACCY] pogo_sema up, %d!!!\n", type);
		//up(&g_rog_accy_data->pogo_sema);
	}

	//down(&g_rog_accy_data->pogo_sema);
	//printk("[ROG_ACCY] pogo_sema down, %d!!!\n", type);
	//pogo_mutex_state = 1;
	kobject_uevent(&g_rog_accy_data->dev->kobj, KOBJ_CHANGE);
}
EXPORT_SYMBOL(rog_accy_uevent);

enum asus_dongle_type pre_dongletype = 255;

static ssize_t gDongleType_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	printk("[ROG_ACCY] gDongleType_show : %d\n", gDongleType);
	return sprintf(buf, "%d\n", gDongleType);
}

static ssize_t gDongleType_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int val;
	sscanf(data, "%d", &val);
	printk("[ROG_ACCY] gDongleType_store : %d\n", val);

	switch(val) {
		case Dongle_NO_INSERT:
			gDongleType = Dongle_NO_INSERT;
		break;
		case Dongle_INBOX5:
			gDongleType = Dongle_INBOX5;
		break;
		case Dongle_FANDG6:
			gDongleType = Dongle_FANDG6;
		break;
		case Dongle_ERROR:
			gDongleType = Dongle_ERROR;
		break;
		case Dongle_Others:
		default:
			printk("[ROG_ACCY] NO Recognize Dongle Type!!! Set gDongleType as Dongle_Others!\n");
			gDongleType = Dongle_Others;
			break;
	}

	if (pogo_mutex_state)
	{
		printk("[ROG_ACCY] pogo_mutex_state : %d, skip send uevent.\n", pogo_mutex_state);
		return count;
	}

	//down(&g_rog_accy_data->pogo_sema);
	//printk("[ROG_ACCY] pogo_sema down!!! %d\n", val);
	//pogo_mutex_state = 1;

	rog_accy_uevent();
	kobject_uevent(&g_rog_accy_data->dev->kobj, KOBJ_CHANGE);
	return count;
}

void vph_output_side_port(int val)
{
	if (val > 0) {
		printk("[ROG_ACCY] VPH_CTRL[%d] set HIGH.\n", g_rog_accy_data->vph_ctrl);
		gpio_set_value(g_rog_accy_data->vph_ctrl, 1);
	}else {
		printk("[ROG_ACCY] VPH_CTRL[%d] set LOW.\n", g_rog_accy_data->vph_ctrl);
		gpio_set_value(g_rog_accy_data->vph_ctrl, 0);
	}
}
EXPORT_SYMBOL_GPL(vph_output_side_port);

static ssize_t vph_ctrl_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	printk("[ROG_ACCY] VPH_CTRL[%d] = %s\n", g_rog_accy_data->vph_ctrl, (gpio_get_value(g_rog_accy_data->vph_ctrl) == 0)?"LOW":"HIGH");
	return snprintf(buf, PAGE_SIZE,"%d\n", gpio_get_value(g_rog_accy_data->vph_ctrl));
}

static ssize_t vph_ctrl_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int ret = 0;
	u32 val;

	ret = kstrtou32(data, 10, &val);
	if (ret)
		return ret;

	if (val > 0) {
		printk("[ROG_ACCY][%s] VPH_CTRL[%d] set HIGH.\n", __func__, g_rog_accy_data->vph_ctrl);
		vph_output_side_port(1);
	}else {
		printk("[ROG_ACCY][%s] VPH_CTRL[%d] set LOW.\n", __func__, g_rog_accy_data->vph_ctrl);
		vph_output_side_port(0);
	}

	return count;
}

static ssize_t pogo_mutex_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int ret = 0;
	u32 val;

	ret = kstrtou32(data, 10, &val);
	if (ret)
		return ret;

	if (val > 0) {
		if(0) {
			printk("[ROG_ACCY] pogo_mutex is already lock.\n");
		} else {
			//down(&g_rog_accy_data->pogo_sema);
			printk("[ROG_ACCY] pogo_sema down!!!\n");

			//pogo_mutex_state = 1;
		}
	}else {
		if(1) {
			pogo_mutex_state = 0;
			printk("[ROG_ACCY] pogo_sema up!!!\n");
			//up(&g_rog_accy_data->pogo_sema);
		} else {
			printk("[ROG_ACCY] pogo_mutex is already unlock.\n");
		}
	}

	return count;
}

static ssize_t pogo_mutex_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{
	printk("[ROG_ACCY] pogo_mutex_show : %d\n", pogo_mutex_state);

	return snprintf(buf, PAGE_SIZE,"%d\n", pogo_mutex_state);
}

static DEVICE_ATTR(gDongleType, S_IRUGO | S_IWUSR, gDongleType_show, gDongleType_store);
static DEVICE_ATTR(VPH_CTRL, S_IRUGO | S_IWUSR, vph_ctrl_show, vph_ctrl_store);
static DEVICE_ATTR(pogo_mutex, S_IRUGO | S_IWUSR, pogo_mutex_show, pogo_mutex_store);
static DEVICE_ATTR(sync_state, S_IRUGO | S_IWUSR, sync_state_show, sync_state_store);

static struct attribute *rog_accy_attrs[] = {
	&dev_attr_gDongleType.attr,
	&dev_attr_VPH_CTRL.attr,
	&dev_attr_pogo_mutex.attr,
	&dev_attr_sync_state.attr,
	NULL
};

const struct attribute_group rog_accy_group = {
	.attrs = rog_accy_attrs,
};

/*
extern struct hidraw *rog6_inbox_hidraw;
void hid_switch_usb_autosuspend(bool flag){
	struct hid_device *hdev;
	struct usb_interface *intf;

	if (rog6_inbox_hidraw == NULL || g_rog_accy_data->lock) {
		printk("[ROG_ACCY] rog6_inbox_hidraw is NULL or lock %d\n", g_rog_accy_data->lock);
		return;
	}

	hdev = rog6_inbox_hidraw->hid;
	intf = to_usb_interface(hdev->dev.parent);

	printk("[ROG_ACCY] hid_swithc_usb_autosuspend %d\n", flag);
	if(flag) {
		//usb_enable_autosuspend(interface_to_usbdev(intf));
	}else {
		//usb_disable_autosuspend(interface_to_usbdev(intf));
	}

	return;
}
*/

void FANDG_connect(int option)
{
	int extcon_val = EXTRA_DOCK_STATE_UNDOCKED;

	switch(option) {
		case 0:
			printk("[ROG_ACCY][EXTCON] EXTRA_DOCK_STATE_UNDOCKED, PDID %s\n", (FANDG_PDID_detect == true)?"detect":"not detect");
			#ifdef ASUS_AI2201_PROJECT
			if (!FANDG_USBID_detect) {
				gDongleType = Dongle_NO_INSERT;
				extcon_val = EXTRA_DOCK_STATE_UNDOCKED;
				// Turn off VPH & HUB Mode
				//vph_output_side_port(0);	//HW removed
				//Inbox_role_switch(0);
				break;
			}else {
				printk("[ROG_ACCY][EXTCON] PDID %s, USBID %s, skip!\n", (FANDG_PDID_detect == true)?"detect":"non-detect", (FANDG_USBID_detect == true)?"detect":"non-detect");
				return;
			}
			#else
			gDongleType = Dongle_NO_INSERT;
			extcon_val = EXTRA_DOCK_STATE_UNDOCKED;
			#endif
		break;
		case 1:
			printk("[ROG_ACCY][EXTCON] EXTRA_DOCK_STATE_ASUS_2nd_INBOX, PDID %s\n", (FANDG_PDID_detect == true)?"detect":"not detect");
			#ifdef ASUS_AI2201_PROJECT
			if (FANDG_USBID_detect) {
				gDongleType = Dongle_FANDG6;
				extcon_val = EXTRA_DOCK_STATE_ASUS_2nd_INBOX;
			}else {
				printk("[ROG_ACCY][EXTCON] PDID %s, USBID %s, skip!\n", (FANDG_PDID_detect == true)?"detect":"non-detect", (FANDG_USBID_detect == true)?"detect":"non-detect");
				return;
			}
			#else
			gDongleType = Dongle_FANDG6;
			extcon_val = EXTRA_DOCK_STATE_ASUS_2nd_INBOX;
			#endif
		break;
		default:
			printk("[ROG_ACCY][EXTCON] unknow option %d, ignore!!!\n", option);
			return;
		break;
	};

	if (extcon_val != accy_extcon->state){
		kobject_uevent(&g_rog_accy_data->dev->kobj, KOBJ_CHANGE);
		msleep(50);
		asus_extcon_set_state_sync(accy_extcon, extcon_val);
	}else
		printk("[ROG_ACCY][EXTCON] Last state %d, Current State %d, No changed, skip!\n", accy_extcon->state, extcon_val);
}
EXPORT_SYMBOL_GPL(FANDG_connect);

static int rog_accy_parse_dt(struct device *dev, struct rog_accy_data *rog_accy_device){
	struct device_node *np = dev->of_node;
	int retval=0;

	// Set VPH_CTRL
	rog_accy_device->vph_ctrl = of_get_named_gpio(np, "accy,vph_ctrl", 0);
	if ( gpio_is_valid(rog_accy_device->vph_ctrl) ) {
		printk("[ROG_ACCY] Request VPH_CTRL config.\n");
		retval = gpio_request(rog_accy_device->vph_ctrl, "VPH_CTRL");
		if (retval)
			printk("[ROG_ACCY] VPH_CTRL gpio_request, err %d\n", retval);

		printk("[ROG_ACCY] VPH_CTRL default off.\n");
		retval = gpio_direction_output(rog_accy_device->vph_ctrl, 0);
		if (retval)
			printk("[ROG_ACCY] VPH_CTRL output high, err %d\n", retval);

		gpio_set_value(rog_accy_device->vph_ctrl, 0);
	}

	// Parse pinctrl state
	if ( gpio_is_valid(rog_accy_device->vph_ctrl) ){
		printk("[ROG_ACCY] Get the pinctrl node.\n");
		// Get the pinctrl node
		rog_accy_device->pinctrl = devm_pinctrl_get(dev);
		if (IS_ERR_OR_NULL(rog_accy_device->pinctrl)) {
		     dev_err(dev, "%s: Failed to get pinctrl\n", __func__);
		}

		// Get the default state
		printk("[ROG_ACCY] Get pinctrl defaul state.\n");
		rog_accy_device->pins_default = pinctrl_lookup_state(rog_accy_device->pinctrl, "vph_ctrl_default");
		if (IS_ERR_OR_NULL(rog_accy_device->pins_default)) {
			dev_err(dev, "%s: Failed to get pinctrl active state\n", __func__);
		}

		// Set the default state
		printk("[ROG_ACCY] Set defaul state.\n");
		retval = pinctrl_select_state(rog_accy_device->pinctrl, rog_accy_device->pins_default);
		if (retval)
			printk("[ROG_ACCY] pinctrl_select_state err:%d\n", retval);
	}

	return 0;
}

static int rog_accy_probe(struct platform_device *pdev)
{
	int status = 0;
	struct device *dev = &pdev->dev;
	struct rog_accy_data *rog_accy_device;
	int retval;
//	enum asus_dongle_type type = Dongle_default_status;

	printk("[ROG_ACCY] rog_accy_probe.\n");
	rog_accy_device = kzalloc(sizeof(*rog_accy_device), GFP_KERNEL);
	if (rog_accy_device == NULL) {
		printk("[ROG_ACCY] alloc ROG_ACCY data fail.\r\n");
		goto kmalloc_failed;
	}

	rog_accy_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(rog_accy_class)) {
		printk("[ROG_ACCY] rog_accy_probe: class_create() is failed - unregister chrdev.\n");
		goto class_create_failed;
	}

	dev = device_create(rog_accy_class, &pdev->dev,
			    rog_accy_device->devt, rog_accy_device, "dongle");
	status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	printk("[ROG_ACCY] rog_accy_probe: device_create() status %d\n", status);

	status = sysfs_create_group(&pdev->dev.kobj, &rog_accy_group);
	printk("[ROG_ACCY] rog_accy_probe: sysfs_create_group() status %d\n", status);

// Extcon file node register
	accy_extcon = extcon_asus_dev_allocate();
	if (IS_ERR(accy_extcon)) {
		status = PTR_ERR(accy_extcon);
		printk("[ROG_ACCY] failed to allocate accy_extcon status %d\n", status);
	}
	accy_extcon->name = "dock";  // assing extcon class name

	status = extcon_asus_dev_register(accy_extcon);
	if (status < 0) {
		printk("[ROG_ACCY] failed to register accy_extcon status %d\n", status);
	}

// Parse platform data from dtsi
	retval = rog_accy_parse_dt(&pdev->dev, rog_accy_device);
	if (retval) {
		printk("[ROG_ACCY] rog_accy_parse_dt get fail !!!\n");
		goto skip_pinctrl;
	}

    sema_init(&rog_accy_device->pogo_sema, 1);

	rog_accy_device->lock = false;
	rog_accy_device->dev = &pdev->dev;
	g_rog_accy_data = rog_accy_device;

	return 0;

skip_pinctrl:
class_create_failed:
kmalloc_failed:
	return -1;
}

static int rog_accy_remove(struct platform_device *pdev)
{
	printk("[ROG_ACCY] rog_accy_remove.\n");
	return 0;
}

int rog_accy_suspend(struct device *dev)
{
	return 0;
}

int rog_accy_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops rog_accy_pm_ops = {
	.suspend	= rog_accy_suspend,
	.resume	= rog_accy_resume,
};

static struct of_device_id dongle_match_table[] = {
	{ .compatible = "asus:rog_accy",},
	{ },
};

static struct platform_driver rog_accy_driver = {
	.driver = {
		.name = "rog_accy",
		.owner = THIS_MODULE,
		.pm	= &rog_accy_pm_ops,
		.of_match_table = dongle_match_table,
	},
	.probe         	= rog_accy_probe,
	.remove			= rog_accy_remove,
};

static int __init rog_accy_init(void)
{
	int ret;

	ret = platform_driver_register(&rog_accy_driver);
	if (ret != 0) {
		printk("[ROG_ACCY] rog_accy_init fail, Error : %d\n", ret);
	}

	return ret;
}
module_init(rog_accy_init);

static void __exit rog_accy_exit(void)
{
	platform_driver_unregister(&rog_accy_driver);
}
module_exit(rog_accy_exit);

MODULE_AUTHOR("ASUS Deeo Ho");
MODULE_DESCRIPTION("ROG Phone ACCY driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("asus:rog_accy");
