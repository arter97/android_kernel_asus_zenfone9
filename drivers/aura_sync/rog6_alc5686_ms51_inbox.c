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
#include <linux/hid.h>
#include <linux/usb.h>
#include <linux/time.h>
#include <linux/firmware.h>
#define MS51_2LED_FW_PATH "ASUS_ROG_FAN6_2LED.bin"
#define MS51_3LED_FW_PATH "ASUS_ROG_FAN6_3LED.bin"

#include "rog6_alc5686_ms51_inbox.h"

struct delayed_work	disable_autosuspend_work;

static u32 hid_report_id_aprom(u8 addr)
{
	u32 report_id = 0;

	switch (addr){
		case addr_0x16:
			report_id = 0x0B;
		break;
		case addr_0x18:
			report_id = 0x0E;
		break;
		case addr_0x75:
			report_id = 0x10;
		break;
		case addr_0x40:
			report_id = 0x11;
		break;
		default:
			printk("[ROG6_INBOX] unknown addr.\n");
		break;
	}

	return report_id;
}

static u32 hid_report_id_long_cmd(u8 addr)
{
	u32 report_id = 0;

	switch (addr){
		case addr_0x16:
			report_id = 0x0C;
		break;
		case addr_0x18:
			report_id = 0x0F;
		break;
		default:
			printk("[ROG6_INBOX] unknown addr.\n");
		break;
	}

	return report_id;
}

static int asus_usb_hid_write_aprom(u8 repor_id, u8 *cmd, int cmd_len)
{
	struct hid_device *hdev;
	int ret = 0;
//	int i = 0;
	char *buffer;

	if (rog6_inbox_hidraw == NULL) {
		printk("[ROG6_INBOX] rog6_inbox_hidraw is NULL !\n");
		return -1;
	}

	buffer = kzalloc(0x06, GFP_KERNEL);
	memset(buffer,0,6);
	hdev = rog6_inbox_hidraw->hid;

	buffer[0] = repor_id;
	buffer[1] = 0x2;		//WRITE CMD
	buffer[2] = cmd_len;	//CMD LEN
	memcpy(&(buffer[3]), cmd, cmd_len);

//	for ( i=0; i<FEATURE_WRITE_COMMAND_SIZE; i++ )
//		printk("[ROG6_INBOX][%s] buffer[%d] = 0x%02x\n", __func__, i, buffer[i]);

	hid_hw_power(hdev, PM_HINT_FULLON);

	ret = hid_hw_raw_request(hdev, buffer[0], buffer, FEATURE_WRITE_COMMAND_SIZE,
					HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	if (ret < 0)
		printk("[ROG6_INBOX] hid_hw_raw_request fail: ret = %d\n", ret);

	hid_hw_power(hdev, PM_HINT_NORMAL);
	kfree(buffer);

	return ret;
}

static int asus_usb_hid_read_aprom(u8 repor_id, u8 *cmd, int cmd_len, u8 *data)
{
	struct hid_device *hdev;
	int ret = 0;
//	int i = 0;
	char *buffer;

	if (rog6_inbox_hidraw == NULL) {
		printk("[ROG6_INBOX] rog6_inbox_hidraw is NULL !\n");
		return -1;
	}

	buffer = kzalloc(FEATURE_WRITE_COMMAND_SIZE, GFP_KERNEL);
	memset(buffer,0,FEATURE_WRITE_COMMAND_SIZE);
	hdev = rog6_inbox_hidraw->hid;

	buffer[0] = repor_id;
	buffer[1] = 0x1;		//READ CMD
	buffer[2] = cmd_len;	//CMD_LEN
	memcpy(&(buffer[3]), cmd, cmd_len);

//	for ( i=0; i<FEATURE_WRITE_COMMAND_SIZE; i++ )
//		printk("[ROG6_INBOX][%s] buffer[%d] = 0x%02x\n", __func__, i, buffer[i]);

	hid_hw_power(hdev, PM_HINT_FULLON);
	ret = hid_hw_raw_request(hdev, buffer[0], buffer, FEATURE_WRITE_COMMAND_SIZE,
					HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	if (ret < 0)
		printk("[ROG6_INBOX] hid_hw_raw_request HID_REQ_SET_REPORT fail: ret = %d\n", ret);

	msleep(10);
	ret = hid_hw_raw_request(hdev, buffer[0], data, FEATURE_READ_COMMAND_SIZE,
					HID_FEATURE_REPORT, HID_REQ_GET_REPORT);
	if (ret < 0)
		printk("[ROG6_INBOX] hid_hw_raw_request HID_REQ_GET_REPORT fail: ret = %d\n", ret);

	hid_hw_power(hdev, PM_HINT_NORMAL);

//	for ( i=0; i<FEATURE_READ_COMMAND_SIZE; i++ )
//		printk("[ROG6_INBOX][%s] data[%d] = 0x%02x\n", __func__, i, data[i]);

	kfree(buffer);

	return ret;
}

static int asus_usb_hid_write_log_cmd(u8 repor_id, u8 *cmd, int cmd_len)
{
	struct hid_device *hdev;
	int ret = 0;
//	int i = 0;
	u8 *buffer;

	if (rog6_inbox_hidraw == NULL) {
		printk("[ROG6_INBOX] rog6_inbox_hidraw is NULL !\n");
		return -1;
	}

	buffer = kzalloc(FEATURE_WRITE_LONG_COMMAND_SIZE, GFP_KERNEL);
	memset(buffer,0,FEATURE_WRITE_LONG_COMMAND_SIZE);
	hdev = rog6_inbox_hidraw->hid;

	buffer[0] = repor_id;
	buffer[1] = 0x2;		// WRITE CMD
	buffer[2] = cmd_len;	// CMD_LEN
	buffer[3] = 0;			// Fix 0x00
	memcpy(&(buffer[4]), cmd, cmd_len);

//	for ( i=0; i<FEATURE_WRITE_LONG_COMMAND_SIZE; i++ )
//		printk("[ROG6_INBOX][%s] buffer[%d] = 0x%02x\n", __func__, i, buffer[i]);

	hid_hw_power(hdev, PM_HINT_FULLON);
	ret = hid_hw_raw_request(hdev, buffer[0], buffer, FEATURE_WRITE_LONG_COMMAND_SIZE,
					HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	if (ret < 0)
		printk("[ROG6_INBOX] hid_hw_raw_request fail: ret = %d\n", ret);

	hid_hw_power(hdev, PM_HINT_NORMAL);
	kfree(buffer);

	return ret;
}

static int asus_usb_hid_read_long_cmd(u8 repor_id, u8 *cmd, int cmd_len, u8 *data, int data_len)
{
	struct hid_device *hdev;
	int ret = 0;
//	int i = 0;
	char *buffer;

	if (rog6_inbox_hidraw == NULL) {
		printk("[ROG6_INBOX] rog6_inbox_hidraw is NULL !\n");
		return -1;
	}

	buffer = kzalloc(FEATURE_WRITE_LONG_COMMAND_SIZE, GFP_KERNEL);
	memset(buffer,0,FEATURE_WRITE_LONG_COMMAND_SIZE);
	hdev = rog6_inbox_hidraw->hid;

	buffer[0] = repor_id;
	buffer[1] = 0x1;		//READ CMD
	buffer[2] = cmd_len;	//CMD_LEN
	buffer[3] = data_len;	//DATA_LEN
	memcpy(&(buffer[4]), cmd, cmd_len);

//	for ( i=0; i<FEATURE_WRITE_LONG_COMMAND_SIZE; i++ )
//		printk("[ROG6_INBOX][%s] buffer[%d] = 0x%02x\n", __func__, i, buffer[i]);

	hid_hw_power(hdev, PM_HINT_FULLON);
	ret = hid_hw_raw_request(hdev, buffer[0], buffer, FEATURE_WRITE_LONG_COMMAND_SIZE,
					HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	if (ret < 0)
		printk("[ROG6_INBOX] hid_hw_raw_request HID_REQ_SET_REPORT fail: ret = %d\n", ret);

	msleep(10);
	ret = hid_hw_raw_request(hdev, buffer[0], data, FEATURE_READ_LONG_COMMAND_SIZE,
					HID_FEATURE_REPORT, HID_REQ_GET_REPORT);
	if (ret < 0)
		printk("[ROG6_INBOX] hid_hw_raw_request HID_REQ_GET_REPORT fail: ret = %d\n", ret);

	hid_hw_power(hdev, PM_HINT_NORMAL);

//	for ( i=0; i<FEATURE_READ_LONG_COMMAND_SIZE; i++ )
//		printk("[ROG6_INBOX][%s] data[%d] = 0x%02x\n", __func__, i, data[i]);

	kfree(buffer);

	return ret;
}

static ssize_t gpio8_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	struct hid_device *hdev;
	u8 *buffer;
	u8 key_state_tmp1, key_state_tmp2;

	if (rog6_inbox_hidraw == NULL) {
		printk("[ROG6_INBOX] rog6_inbox_hidraw is NULL !\n");
		return -1;
	}

	key_state_tmp1 = key_state;

	buffer = kzalloc(2, GFP_KERNEL);
	memset(buffer, 0, 2);

	hdev = rog6_inbox_hidraw->hid;

	buffer[0] = 0x0A;
	buffer[1] = 0x08;

	hid_hw_power(hdev, PM_HINT_FULLON);
	ret = hid_hw_raw_request(hdev, buffer[0], buffer, sizeof(buffer),
					HID_OUTPUT_REPORT, HID_REQ_SET_REPORT);
	if (ret < 0)
		printk("[ROG6_INBOX] hid_hw_raw_request fail: ret = %d\n", ret);
	hid_hw_power(hdev, PM_HINT_NORMAL);

	kfree(buffer);

	key_state_tmp2 = key_state;
	key_state = key_state_tmp1;
	printk("[ROG6_INBOX] gpio8_show : %x\n", key_state_tmp2);
	return snprintf(buf, PAGE_SIZE,"%x\n", key_state_tmp2);
}

static ssize_t gpio8_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 reg_val;
	int ret;
	struct hid_device *hdev;
	u8 *buffer;

	if (rog6_inbox_hidraw == NULL) {
		printk("[ROG6_INBOX] rog6_inbox_hidraw is NULL !\n");
		return -1;
	}

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	printk("[ROG6_INBOX] %s, reg_val %d\n", __func__, reg_val);

	buffer = kzalloc(2, GFP_KERNEL);
	memset(buffer,0,2);

	buffer[0] = 0xA;
	if (reg_val > 0)
		buffer[1] = 0x01;
	else
		buffer[1] = 0x00;

	hdev = rog6_inbox_hidraw->hid;

	hid_hw_power(hdev, PM_HINT_FULLON);
	ret = hid_hw_raw_request(hdev, buffer[0], buffer, sizeof(buffer),
					HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	if (ret < 0)
		printk("[ROG6_INBOX] hid_hw_raw_request fail: ret = %d\n", ret);

	hid_hw_power(hdev, PM_HINT_NORMAL);

	kfree(buffer);

	return count;
}

static ssize_t gpio9_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	struct hid_device *hdev;
	u8 *buffer;
	u8 key_state_tmp1, key_state_tmp2;

	if (rog6_inbox_hidraw == NULL) {
		printk("[ROG6_INBOX] rog6_inbox_hidraw is NULL !\n");
		return -1;
	}

	key_state_tmp1 = key_state;

	buffer = kzalloc(2, GFP_KERNEL);
	memset(buffer, 0, 2);

	hdev = rog6_inbox_hidraw->hid;

	buffer[0] = 0x0A;
	buffer[1] = 0x09;

	hid_hw_power(hdev, PM_HINT_FULLON);
	ret = hid_hw_raw_request(hdev, buffer[0], buffer, sizeof(buffer),
					HID_OUTPUT_REPORT, HID_REQ_SET_REPORT);
	if (ret < 0)
		printk("[ROG6_INBOX] hid_hw_raw_request fail: ret = %d\n", ret);
	hid_hw_power(hdev, PM_HINT_NORMAL);

	kfree(buffer);

	key_state_tmp2 = key_state;
	key_state = key_state_tmp1;
	printk("[ROG6_INBOX] gpio9_show : %x\n", key_state_tmp2);
	return snprintf(buf, PAGE_SIZE,"%x\n", key_state_tmp2);
}

static ssize_t gpio9_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 reg_val;
	int ret;
	struct hid_device *hdev;
	u8 *buffer;

	if (rog6_inbox_hidraw == NULL) {
		printk("[ROG6_INBOX] rog6_inbox_hidraw is NULL !\n");
		return -1;
	}

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	printk("[ROG6_INBOX] %s, reg_val %d\n", __func__, reg_val);

	buffer = kzalloc(2, GFP_KERNEL);
	memset(buffer,0,2);

	buffer[0] = 0xA;
	if (reg_val > 0)
		buffer[1] = 0x91;
	else
		buffer[1] = 0x90;

	hdev = rog6_inbox_hidraw->hid;

	hid_hw_power(hdev, PM_HINT_FULLON);
	ret = hid_hw_raw_request(hdev, buffer[0], buffer, sizeof(buffer),
					HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	if (ret < 0)
		printk("[ROG6_INBOX] hid_hw_raw_request fail: ret = %d\n", ret);
	hid_hw_power(hdev, PM_HINT_NORMAL);
	kfree(buffer);

	return count;
}

static ssize_t gpio10_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	struct hid_device *hdev;
	u8 *buffer;
	u8 key_state_tmp1, key_state_tmp2;

	if (rog6_inbox_hidraw == NULL) {
		printk("[ROG6_INBOX] rog6_inbox_hidraw is NULL !\n");
		return -1;
	}

	key_state_tmp1 = key_state;

	buffer = kzalloc(2, GFP_KERNEL);
	memset(buffer, 0, 2);

	hdev = rog6_inbox_hidraw->hid;

	buffer[0] = 0x0A;
	buffer[1] = 0x0A;

	hid_hw_power(hdev, PM_HINT_FULLON);
	ret = hid_hw_raw_request(hdev, buffer[0], buffer, sizeof(buffer),
					HID_OUTPUT_REPORT, HID_REQ_SET_REPORT);
	if (ret < 0)
		printk("[ROG6_INBOX] hid_hw_raw_request fail: ret = %d\n", ret);
	hid_hw_power(hdev, PM_HINT_NORMAL);

	kfree(buffer);

	key_state_tmp2 = key_state;
	key_state = key_state_tmp1;
	printk("[ROG6_INBOX] gpio10_show : %x\n", key_state_tmp2);
	return snprintf(buf, PAGE_SIZE,"%x\n", key_state_tmp2);

}

static ssize_t gpio10_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 reg_val;
	int ret;
	struct hid_device *hdev;
	u8 *buffer;

	if (rog6_inbox_hidraw == NULL) {
		printk("[ROG6_INBOX] rog6_inbox_hidraw is NULL !\n");
		return -1;
	}

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	printk("[ROG6_INBOX] %s, reg_val %d\n", __func__, reg_val);

	buffer = kzalloc(2, GFP_KERNEL);
	memset(buffer,0,2);

	buffer[0] = 0xA;
	if (reg_val > 0)
		buffer[1] = 0xA1;
	else
		buffer[1] = 0xA0;

	hdev = rog6_inbox_hidraw->hid;

	hid_hw_power(hdev, PM_HINT_FULLON);
	ret = hid_hw_raw_request(hdev, buffer[0], buffer, sizeof(buffer),
					HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	if (ret < 0)
		printk("[ROG6_INBOX] hid_hw_raw_request fail: ret = %d\n", ret);
	hid_hw_power(hdev, PM_HINT_NORMAL);
	kfree(buffer);

	return count;
}

static ssize_t gpio11_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	struct hid_device *hdev;
	u8 *buffer;
	u8 key_state_tmp1, key_state_tmp2;

	if (rog6_inbox_hidraw == NULL) {
		printk("[ROG6_INBOX] rog6_inbox_hidraw is NULL !\n");
		return -1;
	}

	key_state_tmp1 = key_state;

	buffer = kzalloc(2, GFP_KERNEL);
	memset(buffer, 0, 2);

	hdev = rog6_inbox_hidraw->hid;

	buffer[0] = 0x0A;
	buffer[1] = 0x0B;

	hid_hw_power(hdev, PM_HINT_FULLON);
	ret = hid_hw_raw_request(hdev, buffer[0], buffer, sizeof(buffer),
					HID_OUTPUT_REPORT, HID_REQ_SET_REPORT);
	if (ret < 0)
		printk("[ROG6_INBOX] hid_hw_raw_request fail: ret = %d\n", ret);
	hid_hw_power(hdev, PM_HINT_NORMAL);

	kfree(buffer);

	key_state_tmp2 = key_state;
	key_state = key_state_tmp1;
	printk("[ROG6_INBOX] gpio11_show : %x\n", key_state_tmp2);
	return snprintf(buf, PAGE_SIZE,"%x\n", key_state_tmp2);

}

static ssize_t gpio11_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 reg_val;
	int ret;
	struct hid_device *hdev;
	u8 *buffer;

	if (rog6_inbox_hidraw == NULL) {
		printk("[ROG6_INBOX] rog6_inbox_hidraw is NULL !\n");
		return -1;
	}

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	printk("[ROG6_INBOX] %s, reg_val %d\n", __func__, reg_val);

	buffer = kzalloc(2, GFP_KERNEL);
	memset(buffer,0,2);

	buffer[0] = 0xA;
	if (reg_val > 0)
		buffer[1] = 0xB1;
	else
		buffer[1] = 0xB0;

	hdev = rog6_inbox_hidraw->hid;

	hid_hw_power(hdev, PM_HINT_FULLON);
	ret = hid_hw_raw_request(hdev, buffer[0], buffer, sizeof(buffer),
					HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	if (ret < 0)
		printk("[ROG6_INBOX] hid_hw_raw_request fail: ret = %d\n", ret);
	hid_hw_power(hdev, PM_HINT_NORMAL);
	kfree(buffer);

	return count;
}

static ssize_t led_test_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned char data[3] = {0};
	u32 reg_val;
	int err = 0;

	err = kstrtou32(buf, 10, &reg_val);
	if (err)
		return count;

	printk("[ROG6_INBOX] %s, reg_val %d\n", __func__, reg_val);

	data[0] = 0x80;
	data[1] = 0x21;
	data[2] = 0x04;

	g_2led_mode = 0x04;

	err = asus_usb_hid_write_aprom(hid_report_id_aprom(IC_switch), data, 3);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

	msleep(10);

	data[0] = 0x80;
	data[1] = 0x2F;
	data[2] = 0x01;

	err = asus_usb_hid_write_aprom(hid_report_id_aprom(IC_switch), data, 3);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

	return count;
}

static ssize_t red_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	//printk("[ROG6_INBOX] %s reg_val = %d.\n", __func__, reg_val);
	tmp = DIV_ROUND_CLOSEST(reg_val*g_2led_red_max, 255);
	//printk("[ROG6_INBOX] %s tmp = %d.\n", __func__, tmp);

	data[0] = 0x80;
	data[1] = 0x10;
	data[2] = tmp;

	err = asus_usb_hid_write_aprom(hid_report_id_aprom(IC_switch), data, 3);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

	msleep(10);

	data[1] = 0x13;
	err = asus_usb_hid_write_aprom(hid_report_id_aprom(IC_switch), data, 3);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

	if (IC_switch == addr_0x16){
		g_2led_red = tmp;
	}else if (IC_switch == addr_0x18){
		msleep(10);
		data[1] = 0x16;
		err = asus_usb_hid_write_aprom(hid_report_id_aprom(IC_switch), data, 3);
		if (err < 0)
			printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

		g_3led_red = tmp;
	}
	return count;
}

static ssize_t red_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int err = 0;
	u8 *data;
	u8 cmd[3] = {0};
	u8 val;

	data = kzalloc(3, GFP_KERNEL);
	memset(data, 0, 3);

	cmd[0] = 0x80;
	cmd[1] = 0x10;
	cmd[2] = 0x00;

	err = asus_usb_hid_read_aprom(hid_report_id_aprom(IC_switch), cmd, 2, data);
	if (err < 0)
		printk("[ROG6_INBOX] asus_usb_hid_read_aprom:err %d\n", err);

	//printk("[ROG6_INBOX][%s] %x, %x, %x\n", __func__, data[0], data[1], data[2]);

	val = data[1];
	kfree(data);

	printk("[ROG6_INBOX][%d] IC:%d, PWM:0x%02x (%d)\n", __func__, IC_switch, val, val);
	return snprintf(buf, PAGE_SIZE,"%d\n", val);
}

static ssize_t green_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	//printk("[ROG6_INBOX] %s reg_val = %d.\n", __func__, reg_val);
	tmp = DIV_ROUND_CLOSEST(reg_val*g_2led_green_max, 255);
	//printk("[ROG6_INBOX] %s tmp = %d.\n", __func__, tmp);

	data[0] = 0x80;
	data[1] = 0x11;
	data[2] = tmp;

	err = asus_usb_hid_write_aprom(hid_report_id_aprom(IC_switch), data, 3);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

	msleep(10);

	data[1] = 0x14;
	err = asus_usb_hid_write_aprom(hid_report_id_aprom(IC_switch), data, 3);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

	if (IC_switch == addr_0x16){
		g_2led_green = tmp;
	}else if (IC_switch == addr_0x18){
		msleep(10);
		data[1] = 0x17;
		err = asus_usb_hid_write_aprom(hid_report_id_aprom(IC_switch), data, 3);
		if (err < 0)
			printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

		g_3led_green = tmp;
	}

	return count;
}

static ssize_t green_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int err = 0;
	u8 *data;
	u8 cmd[3] = {0};
	u8 val;

	data = kzalloc(3, GFP_KERNEL);
	memset(data, 0, 3);

	cmd[0] = 0x80;
	cmd[1] = 0x11;
	cmd[2] = 0x00;

	err = asus_usb_hid_read_aprom(hid_report_id_aprom(IC_switch), cmd, 2, data);
	if (err < 0)
		printk("[ROG6_INBOX] asus_usb_hid_read_aprom:err %d\n", err);

	//printk("[ROG6_INBOX][%s] %x, %x, %x\n", __func__, data[0], data[1], data[2]);

	val = data[1];
	kfree(data);

	printk("[ROG6_INBOX][%d] IC:%d, PWM:0x%02x (%d)\n", __func__, IC_switch, val, val);
	return snprintf(buf, PAGE_SIZE,"%d\n", val);
}

static ssize_t blue_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	//printk("[ROG6_INBOX] %s reg_val = %d.\n", __func__, reg_val);
	tmp = DIV_ROUND_CLOSEST(reg_val*g_2led_blue_max, 255);
	//printk("[ROG6_INBOX] %s tmp = %d.\n", __func__, tmp);

	data[0] = 0x80;
	data[1] = 0x12;
	data[2] = tmp;

	err = asus_usb_hid_write_aprom(hid_report_id_aprom(IC_switch), data, 3);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

	msleep(10);

	data[1] = 0x15;
	err = asus_usb_hid_write_aprom(hid_report_id_aprom(IC_switch), data, 3);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

	if (IC_switch == addr_0x16){
		g_2led_blue = tmp;
	}else if (IC_switch == addr_0x18){
		msleep(10);
		data[1] = 0x18;
		err = asus_usb_hid_write_aprom(hid_report_id_aprom(IC_switch), data, 3);
		if (err < 0)
			printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

		g_3led_blue = tmp;
	}

	return count;
}

static ssize_t blue_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int err = 0;
	u8 *data;
	u8 cmd[3] = {0};
	u8 val;

	data = kzalloc(3, GFP_KERNEL);
	memset(data, 0, 3);

	cmd[0] = 0x80;
	cmd[1] = 0x12;
	cmd[2] = 0x00;

	err = asus_usb_hid_read_aprom(hid_report_id_aprom(IC_switch), cmd, 2, data);
	if (err < 0)
		printk("[ROG6_INBOX] asus_usb_hid_read_aprom:err %d\n", err);

	//printk("[ROG6_INBOX][%s] %x, %x, %x\n", __func__, data[0], data[1], data[2]);

	val = data[1];
	kfree(data);

	printk("[ROG6_INBOX][%d] IC:%d, PWM:0x%02x (%d)\n", __func__, IC_switch, val, val);
	return snprintf(buf, PAGE_SIZE,"%d\n", val);
}

static ssize_t red1_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	//printk("[ROG6_INBOX] %s reg_val = %d.\n", __func__, reg_val);
	tmp = DIV_ROUND_CLOSEST(reg_val*g_2led_red_max, 255);
	//printk("[ROG6_INBOX] %s tmp = %d.\n", __func__, tmp);

	data[0] = 0x80;
	data[1] = 0x13;
	data[2] = tmp;

	err = asus_usb_hid_write_aprom(hid_report_id_aprom(IC_switch), data, 3);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

	return count;
}

static ssize_t red1_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int err = 0;
	u8 *data;
	u8 cmd[3] = {0};
	u8 val;

	data = kzalloc(3, GFP_KERNEL);
	memset(data, 0, 3);

	cmd[0] = 0x80;
	cmd[1] = 0x13;
	cmd[2] = 0x00;

	err = asus_usb_hid_read_aprom(hid_report_id_aprom(IC_switch), cmd, 2, data);
	if (err < 0)
		printk("[ROG6_INBOX] asus_usb_hid_read_aprom:err %d\n", err);

	//printk("[ROG6_INBOX][%s] %x, %x, %x\n", __func__, data[0], data[1], data[2]);

	val = data[1];
	kfree(data);

	printk("[ROG6_INBOX][%d] IC:%d, PWM:0x%02x (%d)\n", __func__, IC_switch, val, val);
	return snprintf(buf, PAGE_SIZE,"%d\n", val);
}

static ssize_t green1_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	//printk("[ROG6_INBOX] %s reg_val = %d.\n", __func__, reg_val);
	tmp = DIV_ROUND_CLOSEST(reg_val*g_2led_green_max, 255);
	//printk("[ROG6_INBOX] %s tmp = %d.\n", __func__, tmp);

	data[0] = 0x80;
	data[1] = 0x14;
	data[2] = tmp;

	err = asus_usb_hid_write_aprom(hid_report_id_aprom(IC_switch), data, 3);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

	return count;
}

static ssize_t green1_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int err = 0;
	u8 *data;
	u8 cmd[3] = {0};
	u8 val;

	data = kzalloc(3, GFP_KERNEL);
	memset(data, 0, 3);

	cmd[0] = 0x80;
	cmd[1] = 0x14;
	cmd[2] = 0x00;

	err = asus_usb_hid_read_aprom(hid_report_id_aprom(IC_switch), cmd, 2, data);
	if (err < 0)
		printk("[ROG6_INBOX] asus_usb_hid_read_aprom:err %d\n", err);

	//printk("[ROG6_INBOX][%s] %x, %x, %x\n", __func__, data[0], data[1], data[2]);

	val = data[1];
	kfree(data);

	printk("[ROG6_INBOX][%d] IC:%d, PWM:0x%02x (%d)\n", __func__, IC_switch, val, val);
	return snprintf(buf, PAGE_SIZE,"%d\n", val);
}

static ssize_t blue1_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	//printk("[ROG6_INBOX] %s reg_val = %d.\n", __func__, reg_val);
	tmp = DIV_ROUND_CLOSEST(reg_val*g_2led_blue_max, 255);
	//printk("[ROG6_INBOX] %s tmp = %d.\n", __func__, tmp);

	data[0] = 0x80;
	data[1] = 0x15;
	data[2] = tmp;

	err = asus_usb_hid_write_aprom(hid_report_id_aprom(IC_switch), data, 3);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

	return count;
}

static ssize_t blue1_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int err = 0;
	u8 *data;
	u8 cmd[3] = {0};
	u8 val;

	data = kzalloc(3, GFP_KERNEL);
	memset(data, 0, 3);


	cmd[0] = 0x80;
	cmd[1] = 0x15;
	cmd[2] = 0x00;

	err = asus_usb_hid_read_aprom(hid_report_id_aprom(IC_switch), cmd, 2, data);
	if (err < 0)
		printk("[ROG6_INBOX] asus_usb_hid_read_aprom:err %d\n", err);

	//printk("[ROG6_INBOX][%s] %x, %x, %x\n", __func__, data[0], data[1], data[2]);

	val = data[1];
	kfree(data);

	printk("[ROG6_INBOX][%d] IC:%d, PWM:0x%02x (%d)\n", __func__, IC_switch, val, val);
	return snprintf(buf, PAGE_SIZE,"%d\n", val);
}

static ssize_t red2_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	if (IC_switch == addr_0x16){
		printk("[ROG6_INBOX] addr_0x16 not support LED3\n");
		return count;
	}

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	//printk("[ROG6_INBOX] %s reg_val = %d.\n", __func__, reg_val);
	tmp = DIV_ROUND_CLOSEST(reg_val*g_2led_red_max, 255);
	//printk("[ROG6_INBOX] %s tmp = %d.\n", __func__, tmp);

	data[0] = 0x80;
	data[1] = 0x16;
	data[2] = tmp;

	err = asus_usb_hid_write_aprom(hid_report_id_aprom(addr_0x18), data, 3);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

	return count;
}

static ssize_t red2_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int err = 0;
	u8 *data;
	u8 cmd[3] = {0};
	u8 val;

	if (IC_switch == addr_0x16){
		return snprintf(buf, PAGE_SIZE,"Not support on addr_0x16.\n");
	}

	data = kzalloc(3, GFP_KERNEL);
	memset(data, 0, 3);

	cmd[0] = 0x80;
	cmd[1] = 0x16;
	cmd[2] = 0x00;

	err = asus_usb_hid_read_aprom(hid_report_id_aprom(IC_switch), cmd, 2, data);
	if (err < 0)
		printk("[ROG6_INBOX] asus_usb_hid_read_aprom:err %d\n", err);

	//printk("[ROG6_INBOX][%s] %x, %x, %x\n", __func__, data[0], data[1], data[2]);

	val = data[1];
	kfree(data);

	printk("[ROG6_INBOX][%d] IC:%d, PWM:0x%02x (%d)\n", __func__, IC_switch, val, val);
	return snprintf(buf, PAGE_SIZE,"%d\n", val);
}

static ssize_t green2_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	if (IC_switch == addr_0x16){
		printk("[ROG6_INBOX] addr_0x16 not support LED3\n");
		return count;
	}

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	//printk("[ROG6_INBOX] %s reg_val = %d.\n", __func__, reg_val);
	tmp = DIV_ROUND_CLOSEST(reg_val*g_2led_red_max, 255);
	//printk("[ROG6_INBOX] %s tmp = %d.\n", __func__, tmp);

	data[0] = 0x80;
	data[1] = 0x17;
	data[2] = tmp;

	err = asus_usb_hid_write_aprom(hid_report_id_aprom(addr_0x18), data, 3);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

	return count;
}

static ssize_t green2_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int err = 0;
	u8 *data;
	u8 cmd[3] = {0};
	u8 val;

	if (IC_switch == addr_0x16){
		return snprintf(buf, PAGE_SIZE,"Not support on addr_0x16.\n");
	}

	data = kzalloc(3, GFP_KERNEL);
	memset(data, 0, 3);

	cmd[0] = 0x80;
	cmd[1] = 0x17;
	cmd[2] = 0x00;

	err = asus_usb_hid_read_aprom(hid_report_id_aprom(IC_switch), cmd, 2, data);
	if (err < 0)
		printk("[ROG6_INBOX] asus_usb_hid_read_aprom:err %d\n", err);

	//printk("[ROG6_INBOX][%s] %x, %x, %x\n", __func__, data[0], data[1], data[2]);

	val = data[1];
	kfree(data);

	printk("[ROG6_INBOX][%d] IC:%d, PWM:0x%02x (%d)\n", __func__, IC_switch, val, val);
	return snprintf(buf, PAGE_SIZE,"%d\n", val);
}

static ssize_t blue2_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	if (IC_switch == addr_0x16){
		printk("[ROG6_INBOX] addr_0x16 not support LED3\n");
		return count;
	}

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	//printk("[ROG6_INBOX] %s reg_val = %d.\n", __func__, reg_val);
	tmp = DIV_ROUND_CLOSEST(reg_val*g_2led_red_max, 255);
	//printk("[ROG6_INBOX] %s tmp = %d.\n", __func__, tmp);

	data[0] = 0x80;
	data[1] = 0x18;
	data[2] = tmp;

	err = asus_usb_hid_write_aprom(hid_report_id_aprom(addr_0x18), data, 3);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

	return count;
}

static ssize_t blue2_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int err = 0;
	u8 *data;
	u8 cmd[3] = {0};
	u8 val;

	if (IC_switch == addr_0x16){
		return snprintf(buf, PAGE_SIZE,"Not support on addr_0x16.\n");
	}

	data = kzalloc(3, GFP_KERNEL);
	memset(data, 0, 3);

	cmd[0] = 0x80;
	cmd[1] = 0x18;
	cmd[2] = 0x00;

	err = asus_usb_hid_read_aprom(hid_report_id_aprom(IC_switch), cmd, 2, data);
	if (err < 0)
		printk("[ROG6_INBOX] asus_usb_hid_read_aprom:err %d\n", err);

	//printk("[ROG6_INBOX][%s] %x, %x, %x\n", __func__, data[0], data[1], data[2]);

	val = data[1];
	kfree(data);

	printk("[ROG6_INBOX][%d] IC:%d, PWM:0x%02x (%d)\n", __func__, IC_switch, val, val);
	return snprintf(buf, PAGE_SIZE,"%d\n", val);
}

static ssize_t apply_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE,"%d\n", g_2led_apply);
}

static ssize_t apply_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &val);
	if (ret) {
		g_2led_apply = -1;
		return count;
	}

	//printk("[ROG6_INBOX] apply_store: %d\n", val);
	g_2led_apply = 0;

	if (val) {
		data[0] = 0x80;
		data[1] = 0x2F;
		data[2] = 0x01;

		err = asus_usb_hid_write_aprom(hid_report_id_aprom(IC_switch), data, 3);
		if (err < 0) {
			printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);
			g_2led_apply = -1;
			return count;
		}

		if (IC_switch == addr_0x16)
			printk("[ROG6_INBOX] Send apply. IC:%d, RGB:%d %d %d, mode:%d, speed:%d, led_on:%d\n", IC_switch, g_2led_red, g_2led_green, g_2led_blue, g_2led_mode, g_2led_speed, g_led_on);
		else if (IC_switch == addr_0x18)
			printk("[ROG6_INBOX] Send apply. IC:%d, RGB:%d %d %d, mode:%d, speed:%d, led_on:%d\n", IC_switch, g_3led_red, g_3led_green, g_3led_blue, g_3led_mode, g_3led_speed, g_led_on);
	}
	else {
		printk("[ROG6_INBOX] don't send apply command\n");
	}

	return count;
}

static ssize_t mode_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	u8 *data;
	u8 cmd[3] = {0};
	u8 val;

	data = kzalloc(3, GFP_KERNEL);
	memset(data, 0, 3);

	cmd[0] = 0x80;
	cmd[1] = 0x21;
	cmd[2] = 0x00;

	ret = asus_usb_hid_read_aprom(hid_report_id_aprom(IC_switch), cmd, 2, data);
	if (ret < 0)
		printk("[ROG6_INBOX] asus_usb_hid_read_aprom:err %d\n", ret);

	//printk("[ROG6_INBOX][%s] %x, %x, %x\n", __func__, data[0], data[1], data[2]);

	val = data[1];
	kfree(data);

	printk("[ROG6_INBOX][%s] IC:%d, Mode:%d\n", __func__, IC_switch, val);
	return snprintf(buf, PAGE_SIZE,"%d\n", val);
}

static ssize_t set_frame(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;

	if (val > 255){
		printk("[ROG6_INBOX] Frame should not over 255.\n");
		return count;
	}
	//printk("[ROG6_INBOX][%s] %d\n", __func__, val);

	data[0] = 0x80;
	data[1] = 0xF2;
	data[2] = val;

	err = asus_usb_hid_write_aprom(hid_report_id_aprom(IC_switch), data, 3);
	if (err < 0) {
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);
		return count;
	}

	return count;
}

static ssize_t get_frame(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	u8 *data;
	u8 cmd[3] = {0};
	u8 val;

	data = kzalloc(3, GFP_KERNEL);
	memset(data, 0, 3);

	cmd[0] = 0x80;
	cmd[1] = 0xF3;
	cmd[2] = 0x00;

	ret = asus_usb_hid_read_aprom(hid_report_id_aprom(IC_switch), cmd, 2, data);
	if (ret < 0)
		printk("[ROG6_INBOX] asus_usb_hid_read_aprom:err %d\n", ret);

	//printk("[ROG6_INBOX][%s] %x, %x, %x\n", __func__, data[0], data[1], data[2]);

	val = data[1];
	kfree(data);

	printk("[ROG6_INBOX][%s] IC:%d, Frame:%d\n", __func__, IC_switch, val);
	return snprintf(buf, PAGE_SIZE,"%d\n", val);
}

static ssize_t set_speed(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;

	if (val != 254 && val != 255 && val != 0){
		printk("[ROG6_INBOX] speed should be 0, 255, 254\n");
		return count;
	}
	//printk("[ROG6_INBOX][%s] %d\n", __func__, val);

	data[0] = 0x80;
	data[1] = 0x22;
	data[2] = val;

	err = asus_usb_hid_write_aprom(hid_report_id_aprom(IC_switch), data, 3);
	if (err < 0) {
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);
		return count;
	}

	if (IC_switch == addr_0x16)
		g_2led_speed = val;
	else if (IC_switch == addr_0x18)
		g_3led_speed = val;

	return count;
}

static ssize_t get_speed(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	u8 *data;
	u8 cmd[3] = {0};
	u8 val;

	data = kzalloc(3, GFP_KERNEL);
	memset(data, 0, 3);

	cmd[0] = 0x80;
	cmd[1] = 0x22;
	cmd[2] = 0x00;

	ret = asus_usb_hid_read_aprom(hid_report_id_aprom(IC_switch), cmd, 2, data);
	if (ret < 0)
		printk("[ROG6_INBOX] asus_usb_hid_read_aprom:err %d\n", ret);

	//printk("[ROG6_INBOX][%s] %x, %x, %x\n", __func__, data[0], data[1], data[2]);

	val = data[1];
	kfree(data);

	printk("[ROG6_INBOX][%s] IC:%d, Speed:%d\n", __func__, IC_switch, val);
	return snprintf(buf, PAGE_SIZE,"%d\n", val);
}

static ssize_t set_cali_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int red_val = 0, green_val = 0, blue_val = 0;

	sscanf(buf, "%d %d %d", &red_val, &green_val, &blue_val);
	printk("[ROG6_INBOX][%d] %d, %d, %d\n", __func__, red_val, green_val, blue_val);

	if (IC_switch == addr_0x16){
		g_2led_red_max = red_val;
		g_2led_green_max = green_val;
		g_2led_blue_max = blue_val;
	}else if (IC_switch == addr_0x18){
		g_3led_red_max = red_val;
		g_3led_green_max = green_val;
		g_3led_blue_max = blue_val;
	}

	return count;
}

static ssize_t get_cali_data(struct device *dev, struct device_attribute *attr,char *buf)
{
	if (IC_switch == addr_0x16){
		printk("[ROG6_INBOX] IC:%d, R:%d, G:%d, B:%d\n", IC_switch, g_2led_red_max, g_2led_green_max, g_2led_blue_max);
		return snprintf(buf, PAGE_SIZE,"R:%d, G:%d, B:%d\n", g_2led_red_max, g_2led_green_max, g_2led_blue_max);
	}else if (IC_switch == addr_0x18){
		printk("[ROG6_INBOX] IC:%d, R:%d, G:%d, B:%d\n", IC_switch, g_3led_red_max, g_3led_green_max, g_3led_blue_max);
		return snprintf(buf, PAGE_SIZE,"R:%d, G:%d, B:%d\n", g_3led_red_max, g_3led_green_max, g_3led_blue_max);
	}

	return snprintf(buf, PAGE_SIZE,"IC:%d, Choose wrong IC.\n", IC_switch);
}

static ssize_t mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;

	//printk("[ROG6_INBOX] mode_store: %d\n", val);

	data[0] = 0x80;
	data[1] = 0x21;
	data[2] = val;

	err = asus_usb_hid_write_aprom(hid_report_id_aprom(IC_switch), data, 3);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

	if(IC_switch == addr_0x16){
		g_2led_mode = val;
		g_2led_mode2 = val;
	}else if(IC_switch == addr_0x18) {
		g_3led_mode = val;
		g_3led_mode2 = val;
	}

	if(!val){
		g_2led_mode = 0;
		g_2led_mode2 = 0;
		g_3led_mode = 0;
		g_3led_mode2 = 0;
	}

	return count;
}

static ssize_t fw_mode_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	u8 *data;
	u8 cmd[3] = {0};
	u8 val;

	data = kzalloc(3, GFP_KERNEL);
	memset(data, 0, 3);

	cmd[0] = 0xCA;
	cmd[1] = 0x00;
	cmd[2] = 0x00;

	ret = asus_usb_hid_read_aprom(hid_report_id_aprom(IC_switch), cmd, 1, data);
	if (ret < 0)
		printk("[ROG6_INBOX] asus_usb_hid_read_aprom:err %d\n", ret);

	//printk("[ROG6_INBOX][%s] %x, %x, %x\n", __func__, data[0], data[1], data[2]);

	val = data[1];
	kfree(data);

	printk("[ROG6_INBOX][%s] IC:%d, FW Mode:%d\n", __func__, IC_switch, val);
	return snprintf(buf, PAGE_SIZE,"%d\n", val);
}

static ssize_t fw_ver_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	u8 *data;
	u8 cmd[3] = {0};
	u8 val[3] = {0};

	data = kzalloc(3, GFP_KERNEL);
	memset(data, 0, 3);

	cmd[0] = 0xCB;
	cmd[1] = 0x01;
	cmd[2] = 0x00;

	ret = asus_usb_hid_read_aprom(hid_report_id_aprom(IC_switch), cmd, 2, data);
	if (ret < 0)
		printk("[ROG6_INBOX] asus_usb_hid_read_aprom:err %d\n", ret);

	//printk("[ROG6_INBOX][%s] %x, %x, %x\n", __func__, data[0], data[1], data[2]);

	memcpy(val, data, FEATURE_READ_COMMAND_SIZE);
	kfree(data);

	printk("[ROG6_INBOX][%s] IC:%d, FW_VER:0x%02x%02x\n", __func__, IC_switch, val[1], val[2]);
	return snprintf(buf, PAGE_SIZE,"0x%02x%02x\n", val[1], val[2]);
}
/*
static int ms51_GetFirmwareSize(char *firmware_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];

	printk("[ROG6_INBOX] ms51_GetFirmwareSize.\n");
	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s", firmware_name);
	if (NULL == pfile) {
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

static int ms51_ReadFirmware(char *fw_name, unsigned char *fw_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	int ret = 0;

	printk("[ROG6_INBOX] ms51_ReadFirmware.\n");
	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s", fw_name);
	if (NULL == pfile) {
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	pos = 0;
	ret = kernel_read(pfile, fw_buf, fsize, &pos);
	if (ret<0)
		pr_err("Read file %s fail\n", filepath);

	filp_close(pfile, NULL);
	return 0;
}
*/
static int ms51_UpdateFirmware(const unsigned char *fw_buf,int fwsize)
{
	int err = 0;
	unsigned char *buf;
	short addr;
	int count = 0;

	buf = kmalloc(sizeof(unsigned char)*49, GFP_DMA);
	if (!buf) {
		printk("unable to allocate key input memory\n");
		return -ENOMEM;
	}

	//erase--remove this because we will send all 13kb data (add 0 to the end)
	//err = ms51_fw_erase(client);
	//if (err !=1)
		//printk("[AURA_MS51_INBOX] ms51_fw_erase :err %d\n", err);
	//msleep(500);
	//printk("[AURA_MS51_INBOX] after erase :\n");

	//flash

	//first write
	memset(buf,0,sizeof(unsigned char)*48);
	buf[0] = 0xA0;
	buf[13] = 0x34;
	memcpy(&(buf[16]),fw_buf+0,32);

	printk("[ROG6_INBOX][%s] num=0\n", __func__);
	err = asus_usb_hid_write_log_cmd(hid_report_id_long_cmd(IC_switch), buf, 48);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_log_cmd: err %d\n", __func__, err);

	msleep(1000);

	//the follwing write
	for(addr = 32; addr < 13*1024; addr = addr+32){
		memset(buf,0,sizeof(unsigned char)*48);
		buf[0] = 0xA0;
		if(addr <= fwsize-32){
			printk("if: addr = %d\n", addr);
			memcpy(&(buf[16]),fw_buf+addr,32);
			count = 48;
		}else{
			printk("else: addr = %d\n", addr);
			if(addr >= fwsize){
				memset(&(buf[16]),0,sizeof(unsigned char)*32);
				count = 16;
			}else{
				memcpy(&(buf[16]),fw_buf+addr,fwsize-addr);
				//memset(&(buf[16+fwsize-addr]),0,sizeof(unsigned char)*(32-fwsize+addr));
				count = 16 + (fwsize-addr);
			}
		}

		printk("[ROG6_INBOX][%s] num=%d, count=%d\n", __func__, addr/32, count);
		err = asus_usb_hid_write_log_cmd(hid_report_id_long_cmd(IC_switch), buf, count);
		if (err < 0)
			printk("[ROG6_INBOX][%s] asus_usb_hid_write_log_cmd: err %d\n", __func__, err);

		msleep(10);
	}//end for

	printk("[ROG6_INBOX] ms51_UpdateFirmware finished.\n");
	kfree(buf);
	return 0;
}

static ssize_t fw_update_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *buf, size_t count)
{
	int err = 0;
	const struct firmware *fw = NULL;

	// read firmware from fw_path which is defined by boot parameter "firmware_class.path"
	if(IC_switch == 1){
		printk("[ROG6_INBOX] Request MS51_2LED_FW_PATH...\n");
		err = request_firmware(&fw, MS51_2LED_FW_PATH, dev);
	}else if(IC_switch == 2) {
		printk("[ROG6_INBOX] Request MS51_3LED_FW_PATH...\n");
		err = request_firmware(&fw, MS51_3LED_FW_PATH, dev);
	}else {
		printk("[ROG6_INBOX] IC switch error, %d !!!\n", IC_switch);
		return -ENOENT;
	}

	if (err) {
		printk("[ROG6_INBOX] Error: request_firmware failed!!!\n");
		return -ENOENT;
	}

	mutex_lock(&ms51_mutex);
	err = ms51_UpdateFirmware(fw->data,fw->size);
	if(err)
		printk("[ROG6_INBOX] ms51_UpdateFirmware, err %d\n", err);

	mutex_unlock(&ms51_mutex);
	release_firmware(fw);
	return count;

}

static ssize_t fw_update_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	if(IC_switch == 1){
		printk("[ROG6_INBOX] assign %s\n", MS51_2LED_FW_PATH);
		return snprintf(buf, PAGE_SIZE, "%s\n", MS51_2LED_FW_PATH);
	}else if(IC_switch == 2) {
		printk("[ROG6_INBOX] assign %s\n", MS51_3LED_FW_PATH);
		return snprintf(buf, PAGE_SIZE, "%s\n", MS51_3LED_FW_PATH);
	}else {
		printk("[ROG6_INBOX] FW assign error, IC switch %d !!!\n", IC_switch);
		return snprintf(buf, PAGE_SIZE, "Assign Error\n");
	}
}

static ssize_t ap2ld_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;

	printk("[ROG6_INBOX] AP to LD.\n");

	data[0] = 0xCB;
	data[1] = 0x02;
	data[2] = 0x00;

	err = asus_usb_hid_write_aprom(hid_report_id_aprom(IC_switch), data, 2);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

	return count;
}

static ssize_t ld2ap_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;

	printk("[ROG6_INBOX] LD to AP.\n");

	data[0] = 0xAB;
	data[1] = 0x00;
	data[2] = 0x00;

	err = asus_usb_hid_write_aprom(hid_report_id_aprom(IC_switch), data, 2);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

	return count;
}

static ssize_t led_on_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;

	//printk("[ROG6_INBOX] led_on: %d\n", val);
	g_led_on = (val==1)?0x01:0x00;

	data[0] = 0x60;
	data[1] = 0x06;		// MS51 GPIO P0.6
	data[2] = g_led_on;

	err = asus_usb_hid_write_aprom(hid_report_id_aprom(addr_0x16), data, 3);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

	return count;
}

static ssize_t led_on_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", g_led_on);
}

static ssize_t door_on_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;

	//printk("[ROG6_INBOX] door_on: %d\n", val);
	g_door_on = (val==1)?0x01:0x00;

	data[0] = 0x60;
	data[1] = 0x06;		// MS51 GPIO P0.6
	data[2] = g_door_on;

	err = asus_usb_hid_write_aprom(hid_report_id_aprom(addr_0x18), data, 3);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

	return count;
}

static ssize_t door_on_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", g_door_on);
}

static ssize_t logo_on_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;

	//printk("[ROG6_INBOX] door_on: %d\n", val);
	g_logo_on = (val==1)?0x01:0x00;

	data[0] = 0x60;
	data[1] = 0x07;		// MS51 GPIO P0.7
	data[2] = g_logo_on;

	err = asus_usb_hid_write_aprom(hid_report_id_aprom(addr_0x18), data, 3);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

	return count;
}

static ssize_t logo_on_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", g_logo_on);
}

static int fan_enable(unsigned char option)
{
	int err = 0;
	unsigned char data[3] = {0};

	option = (option==1)?0x01:0x00;
	printk("[ROG6_INBOX] FAN %s\n", (option==1)?"Enable":"Disable");

	data[0] = 0x60;
	data[1] = 0x07;		// MS51 GPIO P0.7
	data[2] = option;

	err = asus_usb_hid_write_aprom(hid_report_id_aprom(addr_0x16), data, 3);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write err: %d\n", __func__, err);

	return err;
}

static int fan_pwm(unsigned char pwm)
{
	int err = 0;
	unsigned char data[3] = {0};

	printk("[ROG6_INBOX] FAN PWM: %d\n", pwm);

	if (pwm < 0 || pwm > 255) {
		printk("[ROG6_INBOX][%s] input value error: %d (0 ~ 255)\n", __func__, pwm);
		return -1;
	}

	data[0] = 0x60;
	data[1] = 0x01;
	data[2] = pwm;

	err = asus_usb_hid_write_aprom(hid_report_id_aprom(addr_0x16), data, 3);
	if (err < 0)
		printk("[ROG6_INBOX][%s]: asus_usb_hid_write err: %d\n", __func__, err);

	return err;
}

static ssize_t fan_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	ssize_t ret;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;

	//printk("[ROG6_INBOX] fan_enable_store: %d\n", val);

	if (val > 0)
		fan_enable(1);
	else
		fan_enable(0);

	return count;
}

//+++inbox user fan
static ssize_t inbox_user_fan(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int num = 99;
	u32  tmp;
	int err = 0;

	mutex_lock(&update_lock);

	sscanf(buf, "%d", &num);
	printk("[INBOX_FAN] %s: %d", __func__, num);

	switch (num) {
		case 0:
			tmp = 0;
			err = fan_pwm(tmp);
			msleep(30);
			err = fan_enable(0);
			break;
		case 1:
			tmp = 127;
			err = fan_pwm(tmp);
			msleep(30);
			err = fan_enable(1);
			break;
		case 2:
			tmp = 135;
			err = fan_pwm(tmp);
			msleep(30);
			err = fan_enable(1);
			break;
		case 3:
			tmp = 163;
			err = fan_pwm(tmp);
			msleep(30);
			err = fan_enable(1);
			break;
		case 4:
			tmp = 171;
			err = fan_pwm(tmp);
			msleep(30);
			err = fan_enable(1);
			break;
		default:
			printk("[INBOX_FAN] %s :mode isn't 0-4, unsupport\n", __func__);

	}
	msleep(500); //Wait 0.5s
	mutex_unlock(&update_lock);
	printk("%s ---", __func__);
	return size;
}

//+++inbox thermal fan
static ssize_t inbox_thermal_fan(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int num = 99;
	u32 tmp;
	int err = 0;

	mutex_lock(&update_lock);
	sscanf(buf, "%d", &num);
	printk("[INBOX_FAN] %s: %d", __func__, num);

	switch (num) {
		case 0:
			tmp = 0;
			err = fan_pwm(tmp);
			err = fan_enable(1);
			break;
		case 1:
			tmp = 127;
			err = fan_pwm(tmp);
			err = fan_enable(1);
			break;
		case 2:
			tmp = 135;
			err = fan_pwm(tmp);
			msleep(30);
			err = fan_enable(1);
			break;
		case 3:
			tmp = 163;
			err = fan_pwm(tmp);
			msleep(30);
			err = fan_enable(1);
			break;
		case 4:
			tmp = 171;
			err = fan_pwm(tmp);
			msleep(30);
			err = fan_enable(1);
			break;
		default:
			printk("[INBOX_FAN][%s] mode isn't 0-4, unsupport\n",__func__);
	}
	msleep(500); //Wait 0.5s
	mutex_unlock(&update_lock);
	printk("%s ---", __func__);
	return size;
}
//---inbox thermal fan

static ssize_t fan_rpm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int err = 0;
	u8 *data;
	u8 cmd[3] = {0};
	int raw_data = 0;
	long rpm = 0;

	data = kzalloc(3, GFP_KERNEL);
	memset(data, 0, 3);

	cmd[0] = 0x60;
	cmd[1] = 0x02;
	cmd[2] = 0x00;

	err = asus_usb_hid_write_aprom(hid_report_id_aprom(addr_0x16), cmd, 2);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

	msleep(10);

	cmd[0] = 0xCC;
	cmd[1] = 0x00;
	cmd[2] = 0x00;

	err = asus_usb_hid_read_aprom(hid_report_id_aprom(addr_0x16), cmd, 1, data);
	if (err < 0)
		printk("[ROG6_INBOX] asus_usb_hid_read_aprom:err %d\n", err);

	//printk("[ROG6_INBOX][%s] %02x, %02x, %02x\n", __func__, data[0], data[1], data[2]);
	raw_data = data[1]*256 + data[2];
	kfree(data);
	//printk("[ROG6_INBOX][%s] %d\n", __func__, raw_data);

	if (raw_data != 0) {
		rpm = (24 * 1000000) >> 9;
		do_div(rpm,raw_data);
		printk("[ROG6_INBOX][FAN RPM] %ld\n", rpm*30);
		return snprintf(buf, PAGE_SIZE,"%ld\n", rpm*30);
	} else {
		printk("[ROG6_INBOX][FAN RPM] raw data abnormal.\n");
		return snprintf(buf, PAGE_SIZE,"%d\n", raw_data);
	}
}

static ssize_t fan_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int err = 0;
	ssize_t ret;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;

	err = fan_pwm(val);

	return count;
}

static ssize_t fan_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	u8 *data;
	u8 cmd[3] = {0};
	u8 val;

	data = kzalloc(3, GFP_KERNEL);
	memset(data, 0, 3);

	cmd[0] = 0x60;
	cmd[1] = 0x01;
	cmd[2] = 0x00;

	ret = asus_usb_hid_read_aprom(hid_report_id_aprom(addr_0x16), cmd, 2, data);
	if (ret < 0)
		printk("[ROG6_INBOX] asus_usb_hid_read_aprom:err %d\n", ret);

	//printk("[ROG6_INBOX][%s] %x, %x, %x\n", __func__, data[0], data[1], data[2]);

	val = data[1];
	kfree(data);

	printk("[ROG6_INBOX][%s] %d\n", __func__, val);
	return snprintf(buf, PAGE_SIZE,"%d\n", val);
}

static ssize_t unique_id_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int err = 0;
	u8 *data;
	u8 cmd[48] = {0};
	u8 val[FEATURE_READ_LONG_COMMAND_SIZE] = {0};

	data = kzalloc(FEATURE_READ_LONG_COMMAND_SIZE, GFP_KERNEL);
	memset(data,0,FEATURE_READ_LONG_COMMAND_SIZE);

	cmd[0] = 0xCB;
	cmd[1] = 0x3;

	err = asus_usb_hid_write_log_cmd(hid_report_id_long_cmd(addr_0x16), cmd, 2);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_log_cmd:err %d\n", __func__, err);

	msleep(10);

	cmd[0] = 0xCC;
	cmd[1] = 0x00;

	err = asus_usb_hid_read_long_cmd(hid_report_id_long_cmd(addr_0x16), cmd, 1, data, 12);
	if (err < 0)
		printk("[ROG6_INBOX] asus_usb_hid_read_long_cmd:err %d\n", err);

/*
	printk("[ROG6_INBOX][%s] %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x\n", __func__,
		data[0], data[1], data[2], data[3], data[4], data[5], data[6],
		data[7], data[8], data[9], data[10], data[11], data[12], data[13]);
*/
	memcpy(val, data, FEATURE_READ_LONG_COMMAND_SIZE);
	kfree(data);

	printk("[ROG6_INBOX] Unique ID : 0x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
		val[1], val[2], val[3], val[4], val[5], val[6], val[7], val[8], val[9], val[10], val[11], val[12]);

	return snprintf(buf, PAGE_SIZE,"0x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
		val[1], val[2], val[3], val[4], val[5], val[6],
		val[7], val[8], val[9], val[10], val[11], val[12]);
}

static ssize_t mode2_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	if(IC_switch == addr_0x16)
		return snprintf(buf, PAGE_SIZE,"%d\n", g_2led_mode2);
	else if(IC_switch == addr_0x18)
		return snprintf(buf, PAGE_SIZE,"%d\n", g_3led_mode2);

	return snprintf(buf, PAGE_SIZE,"IC:%d, choose wrong IC.\n", IC_switch);
}

static ssize_t mode2_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned char rgb[RGB_MAX] = {0};
	unsigned char rainbow_mode = 0;
	unsigned char mode2 = 0;
	int err = 0;
	int i = 0, n = 0, rgb_num = 0;
	long rgb_tmp = 0;
	int ntokens = 0;
	const char *cp = buf;
	const char *buf_tmp;
	unsigned char data[3] = {0};

	sscanf(buf, "%d", &mode2);

	while ((cp = strpbrk(cp + 1, ","))){
		ntokens++;  //the number of ","
		//printk("[ROG6_INBOX] mode2_store %s.\n", cp);
	}

	//printk("[ROG6_INBOX][%s] mode2=%d ntokens=%d, buf=%s\n", __func__, mode2, ntokens, buf);
	if(ntokens > 6){
		printk("[ROG6_INBOX][%s] Too many ntokens %d, Maxium can't over 6\n", __func__, ntokens);
		g_2led_mode2=-1;
		return count;
	}

	cp=buf;
	while((cp = strpbrk(cp, ",")))  //goto the ",".
	{
		cp++; // go after the ','
		while(*cp != ',' && *cp != '\0' && *cp !='\n')
		{
			if(*cp==' '){
				cp++; //skip the ' '
			}else{
				buf_tmp = cp;
				rgb_tmp = 0;
				sscanf(buf_tmp, "%x",&rgb_tmp);
				rgb[rgb_num++] = (rgb_tmp >> 16)&0xFF;
				rgb[rgb_num++] = (rgb_tmp >> 8)&0xFF;
				rgb[rgb_num++] = rgb_tmp & 0xFF;
				break;
			}
		}
	}

	if(rgb_num != ntokens*3){
		printk("[ROG6_INBOX][%s] Wrong input. rgb_num (%d) != ntokens*3 (%d*3)\n", __func__, rgb_num, ntokens);
		g_2led_mode2=-1;
		return count;
	}
/*
	printk("[ROG6_INBOX][%s] rgb_num = %d \n",__func__, rgb_num);
	for(i=0;i<rgb_num;i++)
		printk("[ROG6_INBOX][%s] rgb[%d] = 0x%x \n",__func__, i, rgb[i]);
*/
	switch(mode2){
		case 0: //closed
			rainbow_mode = 0;
			break;
		case 1: //6 color rainbow
			rainbow_mode = 0x7;
			break;
		case 2: //static
			rainbow_mode = 1;
			break;
		case 3: //breath at the same time
			rainbow_mode = 0x2;
			break;
		case 4: //breath at different time
			rainbow_mode = 0x11;
			break;
		case 5: //breath only one led
			rainbow_mode = 0x10;
			break;
		case 6: //commet
			rainbow_mode = 0x12;
			break;
		case 7: //flash and dash
			rainbow_mode = 0x14;
			break;
		case 8: //commet in different direction direction
			rainbow_mode = 0x13;
			break;
		case 9: //flash and dash in different direction
			rainbow_mode = 0x15;
			break;
		case 10: //6 color in different direction
			rainbow_mode = 0x8;
			break;
		case 11: //6 color in different direction
			rainbow_mode = 0xF;
			break;
		case 12: //LED1 static, LED2 & LED3 breath
			rainbow_mode = 0x1F;
			break;
		case 13: //LED2 static, LED1 & LED3 breath
			rainbow_mode = 0x2F;
			break;
		case 14: //LED3 static, LED1 & LED2 breath
			rainbow_mode = 0x3F;
			break;
		case 15: //LED1 breath, LED2 & LED3 static
			rainbow_mode = 0x4F;
			break;
		case 16: //LED2 breath, LED1 & LED3 static
			rainbow_mode = 0x5F;
			break;
		case 17: //LED3 breath, LED1 & LED2 static
			rainbow_mode = 0x6F;
			break;
		case 18: //LED1 & LED3 breath, LED2 breath at different time
			rainbow_mode = 0x11;
			break;
		case 19: //LED1 & LED2 breath, LED3 breath at different time
			rainbow_mode = 0x21;
			break;
		case 20: //LED2 & LED3 breath, LED1 breath at different time
			rainbow_mode = 0x31;
			break;
		case 21: //Flow breath 01 (slow)
			rainbow_mode = 0x16;
			break;
		case 22: //Flow breath 02 (Fast)
			rainbow_mode = 0x17;
			break;
	}

	if ( (IC_switch == addr_0x16) && ((mode2 == 12) || (mode2 == 13)) ){
		printk("[ROG6_INBOX][%s] 2LED MS51 not support mode2 %d\n", __func__, mode2);
		return count;
	}

	switch(rainbow_mode){
		case 0:  //mode 0
			break;
		case 0x1: //static
		case 0x2: //breath at the same time
		case 0xF: //breath one led
		case 0x10: //breath one led
		case 0x11: //breath at the different time //LED1 & LED3 breath, LED2 breath at different time
		case 0x1f://LED1 static, LED2 & LED3 breath
		case 0x2f://LED2 static, LED1 & LED3 breath
		case 0x3f://LED3 static, LED1 & LED2 breath
		case 0x4f://LED1 breath, LED2 & LED3 static
		case 0x5f://LED2 breath, LED1 & LED3 static
		case 0x6f://LED3 breath, LED1 & LED2 static
		case 0x21://LED1 & LED2 breath, LED3 breath at different time
		case 0x31://LED1 & LED3 breath, LED1 breath at different time
			if( ((ntokens != 2) && (IC_switch == addr_0x16)) || ((ntokens != 3) && (IC_switch == addr_0x18))){
				printk("[ROG6_INBOX][%s] Wrong input. ntokens(%d) != %d\n", __func__, ntokens, (IC_switch == addr_0x16)?2:3);
				g_2led_mode2 = -1;
				return count;
			}
			//sscanf(buf, "%x, %x %x %x,%x %x %x", &rainbow_mode,&rgb[0],&rgb[1],&rgb[2],&rgb[3],&rgb[4],&rgb[5]);
			//printk("[ROG6_INBOX] mode2_store,static two leds. mode=0x%x,client->addr:0x%02x.\n", rainbow_mode,client->addr);

			for(i=0; i<=ntokens*3; i++){
				data[0] = 0x80;
				data[1] = (0x10 + i);
				data[2] = rgb[i];

				err = asus_usb_hid_write_aprom(hid_report_id_aprom(IC_switch), data, 3);
				if (err < 0) {
					g_2led_mode2 = -1;
					printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);
				}
			}
			break;
		case 0x7:
		case 0x8://6 colors rainbow
			if(ntokens != 6){
				printk("[ROG6_INBOX][%s] Wrong input. ntokens(%d) != 6\n", __func__, ntokens);
				g_2led_mode2=-1;
				return count;
			}

			for(i=0; i<ntokens; i++){
				data[0] = 0xD0 + i;
				for(n=0; n<3; n++){
					data[1] = n;
					data[2] = rgb[3*i+n];

					err = asus_usb_hid_write_aprom(hid_report_id_aprom(IC_switch), data, 3);
					if (err < 0){
						g_2led_mode2=-1;
						printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);
					}

				}
			}
			break;
		case 0x12://comet
		case 0x13://comet in different direction
		case 0x14://flash and dash
		case 0x15://flash and dash in different direction
		case 0x16://Flow breath 01 (slow)
		case 0x17://Flow breath 02 (fast)
			if( ((ntokens != 2) && (IC_switch == addr_0x16)) || ((ntokens != 3) && (IC_switch == addr_0x18))){
				printk("[ROG6_INBOX][%s] Wrong input. ntokens(%d) != %d\n", __func__, ntokens, (IC_switch == addr_0x16)?2:3);
				return count;
			}

			//sscanf(buf, "%x, %x %x %x", &rainbow_mode,&rgb[0],&rgb[1],&rgb[2]);
			//printk("[AURA_MS51_INBOX] mode2_store,comet or flash and dash. mode=0x%x,client->addr:0x%x.\n", rainbow_mode,client->addr);

			for(i=0; i<ntokens; i++){
				data[0] = 0xDB + i;
				for(n=0; n<=2; n++){
					data[1] = n;
					data[2] = rgb[3*i+n];

					err = asus_usb_hid_write_aprom(hid_report_id_aprom(IC_switch), data, 3);
					if (err < 0) {
						g_2led_mode2 = -1;
						printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);
					}
				}
			}
			break;
		default:
			break;
	}

	// send mode command
	data[0] = 0x80;
	data[1] = 0x21;
	data[2] = rainbow_mode;

	err = asus_usb_hid_write_aprom(hid_report_id_aprom(IC_switch), data, 3);
	if (err < 0) {
		g_2led_mode2 = -1;
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);
	}

	if(IC_switch == addr_0x16){
		g_2led_mode2 = mode2;
		g_2led_mode = (u8)rainbow_mode;
	}else if(IC_switch == addr_0x18){
		g_3led_mode2 = mode2;
		g_3led_mode = (u8)rainbow_mode;
	}

	return count;
}

static ssize_t key_state_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	printk("[ROG6_INBOX][%s] 0x%02x\n", __func__, key_state);
	return snprintf(buf, PAGE_SIZE,"0x%02x\n", key_state);
}

static ssize_t cooling_en_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;

	g_cooling_en = (val==1)?0x01:0x00;
	printk("[ROG6_INBOX] Cooler Module %s\n", (g_cooling_en==1)?"Enable":"Disable");

	data[0] = 0x60;
	data[1] = 0x05;		// MS51 GPIO P0.5
	data[2] = g_cooling_en;

	err = asus_usb_hid_write_aprom(hid_report_id_aprom(addr_0x16), data, 3);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

	return count;
}

static ssize_t cooling_en_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", g_cooling_en);
}

static ssize_t cooling_stage_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int err = 0;
	ssize_t ret;
	unsigned char data[2] = {0};

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;

	if(val >= 0 && val < 128)
		g_cooling_stage = val;
	else {
		printk("[ROG6_INBOX][%s] error value %d, should between 0 ~ 127\n", __func__, val);
		return count;
	}

	data[0] = 0x04;
	data[1] = g_cooling_stage;

	err = asus_usb_hid_write_aprom(hid_report_id_aprom(addr_0x75), data, 2);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

	return count;
}

static ssize_t cooling_stage_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int err = 0;
	u8 *data;
	u8 cmd[3] = {0};
	u8 val;

	data = kzalloc(3, GFP_KERNEL);
	memset(data, 0, 3);

	cmd[0] = 0x04;
	cmd[1] = 0x00;
	cmd[2] = 0x00;

	err = asus_usb_hid_read_aprom(hid_report_id_aprom(addr_0x75), cmd, 1, data);
	if (err < 0)
		printk("[ROG6_INBOX] asus_usb_hid_read_aprom:err %d\n", err);

	//printk("[ROG6_INBOX] cooling_stage_show : %02x, %02x, %02x\n", data[0], data[1], data[2]);
	printk("[ROG6_INBOX][%s] 0x%02x\n", __func__, data[1]);

	val = data[1];
	kfree(data);

	return snprintf(buf, PAGE_SIZE,"0x%02x\n", val);
}

static ssize_t RT6160_DEVID_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int err = 0;
	u8 *data;
	u8 cmd[3] = {0};
	u8 val;

	data = kzalloc(3, GFP_KERNEL);
	memset(data, 0, 3);

	cmd[0] = 0x03;
	cmd[1] = 0x00;
	cmd[2] = 0x00;

	err = asus_usb_hid_read_aprom(hid_report_id_aprom(addr_0x75), cmd, 1, data);
	if (err < 0)
		printk("[ROG6_INBOX] asus_usb_hid_read_aprom:err %d\n", err);

	//printk("[ROG6_INBOX][%s] %02x, %02x, %02x\n", __func__, data[0], data[1], data[2]);
	printk("[ROG6_INBOX] RT6160 DEVID : 0x%02x\n", data[1]);

	val = data[1];
	kfree(data);

	return snprintf(buf, PAGE_SIZE,"0x%02x\n", val);
}

static ssize_t HDC2010_MANID_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int err = 0;
	u8 *data;
	u8 cmd[3] = {0};
	u8 val_L = 0x0, val_H = 0x0;

	data = kzalloc(3, GFP_KERNEL);
	memset(data, 0, 3);

	cmd[0] = 0xFC;
	cmd[1] = 0x00;
	cmd[2] = 0x00;

	err = asus_usb_hid_read_aprom(hid_report_id_aprom(addr_0x40), cmd, 1, data);
	if (err < 0)
		printk("[ROG6_INBOX] asus_usb_hid_read_aprom:err %d\n", err);

	//printk("[ROG6_INBOX][%s] %02x, %02x, %02x\n", __func__, data[0], data[1], data[2]);
	printk("[ROG6_INBOX] HDC2010 MANID REG[0xFC]: 0x%02x\n", data[1]);
	val_L = data[1];

	memset(data, 0, 3);

	cmd[0] = 0xFD;
	cmd[1] = 0x00;
	cmd[2] = 0x00;

	err = asus_usb_hid_read_aprom(hid_report_id_aprom(addr_0x40), cmd, 1, data);
	if (err < 0)
		printk("[ROG6_INBOX] asus_usb_hid_read_aprom:err %d\n", err);

	//printk("[ROG6_INBOX][%s] %02x, %02x, %02x\n", __func__, data[0], data[1], data[2]);
	printk("[ROG6_INBOX] HDC2010 MANID REG[0xFD]: 0x%02x\n", data[1]);
	val_H = data[1];

	kfree(data);

	return snprintf(buf, PAGE_SIZE,"0x%02x%02x\n", val_H, val_L);
}

static ssize_t HDC2010_DEVID_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int err = 0;
	u8 *data;
	u8 cmd[3] = {0};
	u8 val_L = 0x0, val_H = 0x0;

	data = kzalloc(3, GFP_KERNEL);
	memset(data, 0, 3);

	cmd[0] = 0xFE;
	cmd[1] = 0x00;
	cmd[2] = 0x00;

	err = asus_usb_hid_read_aprom(hid_report_id_aprom(addr_0x40), cmd, 1, data);
	if (err < 0)
		printk("[ROG6_INBOX] asus_usb_hid_read_aprom:err %d\n", err);

	//printk("[ROG6_INBOX][%s] %02x, %02x, %02x\n", __func__, data[0], data[1], data[2]);
	printk("[ROG6_INBOX] HDC2010 DEVID REG[0xFE]: 0x%02x\n", data[1]);
	val_L = data[1];

	memset(data, 0, 3);

	cmd[0] = 0xFF;
	cmd[1] = 0x00;
	cmd[2] = 0x00;

	err = asus_usb_hid_read_aprom(hid_report_id_aprom(addr_0x40), cmd, 1, data);
	if (err < 0)
		printk("[ROG6_INBOX] asus_usb_hid_read_aprom:err %d\n", err);

	//printk("[ROG6_INBOX][%s] %02x, %02x, %02x\n", __func__, data[0], data[1], data[2]);
	printk("[ROG6_INBOX] HDC2010 DEVID REG[0xFF]: 0x%02x\n", data[1]);
	val_H = data[1];

	kfree(data);

	return snprintf(buf, PAGE_SIZE,"0x%02x%02x\n", val_H, val_L);
}

static ssize_t measure_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int err = 0;
	ssize_t ret;
	unsigned char data[2] = {0};

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;

	g_TH_meaure = (val==1)?0x1:0x0;

	// 0x20 : 1/60Hz, 0x30 : 1/10Hz, 0x50 : 1Hz
	data[0] = 0x0E;
	data[1] = 0x50;

	err = asus_usb_hid_write_aprom(hid_report_id_aprom(addr_0x40), data, 2);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

	msleep(10);

	// 0x0 : no action, 0x1 : start
	data[0] = 0x0F;
	data[1] = g_TH_meaure;

	err = asus_usb_hid_write_aprom(hid_report_id_aprom(addr_0x40), data, 2);
	if (err < 0)
		printk("[ROG6_INBOX][%s] asus_usb_hid_write_aprom:err %d\n", __func__, err);

	return count;
}

static ssize_t measure_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", g_TH_meaure);
}

static ssize_t temperature_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int err = 0;
	u8 *data;
	u8 cmd[3] = {0};
	u8 LSB = 0, MSB = 0x0;
	u32 temperature = 0;

	data = kzalloc(3, GFP_KERNEL);
	memset(data, 0, 3);

	cmd[0] = 0x0;

	err = asus_usb_hid_read_aprom(hid_report_id_aprom(addr_0x40), cmd, 1, data);
	if (err < 0)
		printk("[ROG6_INBOX] asus_usb_hid_read_aprom:err %d\n", err);

	LSB = data[1];
	//printk("[ROG6_INBOX][%s] REG[0x%02x] %02x, %02x, %02x\n", __func__, cmd[0], data[0], data[1], data[2]);

	msleep(10);
	memset(data, 0, 3);
	cmd[0] = 0x1;

	err = asus_usb_hid_read_aprom(hid_report_id_aprom(addr_0x40), cmd, 1, data);
	if (err < 0)
		printk("[ROG6_INBOX] asus_usb_hid_read_aprom:err %d\n", err);

	MSB = data[1];
	//printk("[ROG6_INBOX][%s] REG[0x%02x] %02x, %02x, %02x\n", __func__, cmd[0], data[0], data[1], data[2]);

	kfree(data);

	temperature = ((((MSB << 8) + LSB)*160) >> 16)-40;
	printk("[ROG6_INBOX][%s] %d\n", __func__, temperature);

	return snprintf(buf, PAGE_SIZE,"%d\n", temperature);
}

static ssize_t humidity_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int err = 0;
	u8 *data;
	u8 cmd[3] = {0};
	u8 LSB = 0, MSB = 0x0;
	u32 humidity = 0;

	data = kzalloc(3, GFP_KERNEL);
	memset(data, 0, 3);

	cmd[0] = 0x2;

	err = asus_usb_hid_read_aprom(hid_report_id_aprom(addr_0x40), cmd, 1, data);
	if (err < 0)
		printk("[ROG6_INBOX] asus_usb_hid_read_aprom:err %d\n", err);

	LSB = data[1];
	//printk("[ROG6_INBOX][%s] REG[0x%02x] %02x, %02x, %02x\n", __func__, cmd[0], data[0], data[1], data[2]);

	msleep(10);
	memset(data, 0, 3);
	cmd[0] = 0x3;

	err = asus_usb_hid_read_aprom(hid_report_id_aprom(addr_0x40), cmd, 1, data);
	if (err < 0)
		printk("[ROG6_INBOX] asus_usb_hid_read_aprom:err %d\n", err);

	MSB = data[1];
	//printk("[ROG6_INBOX][%s] REG[0x%02x] %02x, %02x, %02x\n", __func__, cmd[0], data[0], data[1], data[2]);

	kfree(data);

	humidity = ((((MSB << 8) + LSB)*100) >> 16);
	printk("[ROG6_INBOX][%s] %d\n", __func__, humidity);

	return snprintf(buf, PAGE_SIZE,"%d\n", humidity);
}

static ssize_t ic_switch_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	switch (IC_switch){
		case addr_0x16:
			printk("[ROG6_INBOX] Choose addr_0x16.\n");
		break;
		case addr_0x18:
			printk("[ROG6_INBOX] Choose addr_0x18.\n");
		break;
		case addr_0x75:
			printk("[ROG6_INBOX] Choose addr_0x75.\n");
		break;
		case addr_0x40:
			printk("[ROG6_INBOX] Choose addr_0x40.\n");
		break;
		default:
			printk("[ROG6_INBOX] unknown addr.\n");
		break;
	}

	return snprintf(buf, PAGE_SIZE,"%d\n", IC_switch);
}

static ssize_t ic_switch_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;

	sscanf(buf, "%d", &val);

	if (val == 1)
		IC_switch = addr_0x16;
	else if (val == 2)
		IC_switch = addr_0x18;
	else if (val == 3)
		IC_switch = addr_0x75;
	else if (val == 4)
		IC_switch = addr_0x40;
	else{
		printk("[ROG6_INBOX] Input error I2C address.\n");
	}

	//printk("[ROG6_INBOX] IC switch 0x%x\n", IC_switch);
	return count;
}

static DEVICE_ATTR(gpio8, 0664, gpio8_show, gpio8_store);
static DEVICE_ATTR(gpio9, 0664, gpio9_show, gpio9_store);
static DEVICE_ATTR(gpio10, 0664, gpio10_show, gpio10_store);
static DEVICE_ATTR(gpio11, 0664, gpio11_show, gpio11_store);
static DEVICE_ATTR(led_test, 0664, NULL, led_test_store);
static DEVICE_ATTR(red_pwm, 0664, red_pwm_show, red_pwm_store);
static DEVICE_ATTR(green_pwm, 0664, green_pwm_show, green_pwm_store);
static DEVICE_ATTR(blue_pwm, 0664, blue_pwm_show, blue_pwm_store);
static DEVICE_ATTR(red1_pwm, 0664, red1_pwm_show, red1_pwm_store);
static DEVICE_ATTR(green1_pwm, 0664, green1_pwm_show, green1_pwm_store);
static DEVICE_ATTR(blue1_pwm, 0664, blue1_pwm_show, blue1_pwm_store);
static DEVICE_ATTR(red2_pwm, 0664, red2_pwm_show, red2_pwm_store);
static DEVICE_ATTR(green2_pwm, 0664, green2_pwm_show, green2_pwm_store);
static DEVICE_ATTR(blue2_pwm, 0664, blue2_pwm_show, blue2_pwm_store);
static DEVICE_ATTR(apply, 0664, apply_show, apply_store);
static DEVICE_ATTR(mode, 0664, mode_show, mode_store);
static DEVICE_ATTR(frame, 0664, get_frame, set_frame);
static DEVICE_ATTR(speed, 0664, get_speed, set_speed);
static DEVICE_ATTR(Calibration, 0664, get_cali_data, set_cali_data);
static DEVICE_ATTR(fw_mode, 0664, fw_mode_show, NULL);
static DEVICE_ATTR(fw_ver, 0664, fw_ver_show, NULL);
static DEVICE_ATTR(fw_update, 0664, fw_update_show, fw_update_store);
static DEVICE_ATTR(ap2ld, 0664, NULL, ap2ld_store);
static DEVICE_ATTR(ld2ap, 0664, NULL, ld2ap_store);
static DEVICE_ATTR(led_on, 0664, led_on_show, led_on_store);
static DEVICE_ATTR(door_on, 0664, door_on_show, door_on_store);
static DEVICE_ATTR(logo_on, 0664, logo_on_show, logo_on_store);
static DEVICE_ATTR(fan_enable, 0664, NULL, fan_enable_store);
static DEVICE_ATTR(fan_RPM, 0664, fan_rpm_show, NULL);
static DEVICE_ATTR(fan_PWM, 0664, fan_pwm_show, fan_pwm_store);
static DEVICE_ATTR(inbox_user_type, 0664, NULL, inbox_user_fan);
static DEVICE_ATTR(inbox_thermal_type, 0664, NULL, inbox_thermal_fan);
static DEVICE_ATTR(unique_id, 0664, unique_id_show, NULL);
static DEVICE_ATTR(mode2, 0664, mode2_show, mode2_store);
static DEVICE_ATTR(key_state, 0664, key_state_show, NULL);
static DEVICE_ATTR(cooling_en, 0664, cooling_en_show, cooling_en_store);
static DEVICE_ATTR(cooling_stage, 0664, cooling_stage_show, cooling_stage_store);
static DEVICE_ATTR(cooling_DEVID, 0664, RT6160_DEVID_show, NULL);
static DEVICE_ATTR(HDC2010_MANID, 0664, HDC2010_MANID_show, NULL);
static DEVICE_ATTR(HDC2010_DEVID, 0664, HDC2010_DEVID_show, NULL);
static DEVICE_ATTR(measure, 0664, measure_show, measure_store);
static DEVICE_ATTR(temperature, 0664, temperature_show, NULL);
static DEVICE_ATTR(humidity, 0664, humidity_show, NULL);
static DEVICE_ATTR(ic_switch, 0664, ic_switch_show, ic_switch_store);

static struct attribute *pwm_attrs[] = {
	&dev_attr_gpio8.attr,
	&dev_attr_gpio9.attr,
	&dev_attr_gpio10.attr,
	&dev_attr_gpio11.attr,
	&dev_attr_led_test.attr,
	&dev_attr_red_pwm.attr,
	&dev_attr_green_pwm.attr,
	&dev_attr_blue_pwm.attr,
	&dev_attr_red1_pwm.attr,
	&dev_attr_green1_pwm.attr,
	&dev_attr_blue1_pwm.attr,
	&dev_attr_red2_pwm.attr,
	&dev_attr_green2_pwm.attr,
	&dev_attr_blue2_pwm.attr,
	&dev_attr_apply.attr,
	&dev_attr_mode.attr,
	&dev_attr_frame.attr,
	&dev_attr_speed.attr,
	&dev_attr_Calibration.attr,
	&dev_attr_fw_mode.attr,
	&dev_attr_fw_ver.attr,
	&dev_attr_fw_update.attr,
	&dev_attr_ap2ld.attr,
	&dev_attr_ld2ap.attr,
	&dev_attr_led_on.attr,
	&dev_attr_door_on.attr,
	&dev_attr_logo_on.attr,
	&dev_attr_fan_enable.attr,
	&dev_attr_fan_RPM.attr,
	&dev_attr_fan_PWM.attr,
	&dev_attr_inbox_user_type.attr,
	&dev_attr_inbox_thermal_type.attr,
	&dev_attr_unique_id.attr,
	&dev_attr_mode2.attr,
	&dev_attr_key_state.attr,
	&dev_attr_cooling_en.attr,
	&dev_attr_cooling_stage.attr,
	&dev_attr_cooling_DEVID.attr,
	&dev_attr_HDC2010_MANID.attr,
	&dev_attr_HDC2010_DEVID.attr,
	&dev_attr_measure.attr,
	&dev_attr_temperature.attr,
	&dev_attr_humidity.attr,
	&dev_attr_ic_switch.attr,
	NULL
};

static const struct attribute_group pwm_attr_group = {
	.attrs = pwm_attrs,
};

static void aura_sync_set(struct led_classdev *led,
			      enum led_brightness brightness)
{
}

static enum led_brightness aura_sync_get(struct led_classdev *led_cdev)
{
	struct inbox_drvdata *data;

	data = container_of(led_cdev, struct inbox_drvdata, led);

	return data->led.brightness;
}

static int aura_sync_register(struct device *dev, struct inbox_drvdata *data)
{
	data->led.name = "aura_inbox";

	data->led.brightness = LED_OFF;
	data->led.max_brightness = LED_HALF;
	data->led.default_trigger = "none";
	data->led.brightness_set = aura_sync_set;
	data->led.brightness_get = aura_sync_get;

	return led_classdev_register(dev, &data->led);
}
static void aura_sync_unregister(struct inbox_drvdata *data)
{
	led_classdev_unregister(&data->led);
}

#ifdef CONFIG_PM
static int rog6_inbox_usb_resume(struct hid_device *hdev)
{
	if(g_Charger_mode) {
		printk("[ROG6_INBOX] In charger mode, stop rog6_inbox_usb_resume\n");
		return 0;
	}

	printk("%s\n", __func__);
	return 0;
}

static int rog6_inbox_usb_suspend(struct hid_device *hdev, pm_message_t message)
{
	if(g_Charger_mode) {
		printk("[ROG6_INBOX] In charger mode, stop rog6_inbox_usb_suspend\n");
		return 0;
	}

	printk("%s\n", __func__);
	return 0;
}
#endif /* CONFIG_PM */

static int rog6_inbox_usb_raw_event(struct hid_device *hdev,
		struct hid_report *report, u8 *data, int size)
{
	return 0;
}

void disable_autosuspend_worker(struct work_struct *work)
{
	if (rog6_inbox_hidraw == NULL) {
		printk("[ROG6_INBOX] %s : rog6_inbox_hidraw is NULL.\n", __func__);
		return;
	}

	//usb_disable_autosuspend(interface_to_usbdev(to_usb_interface(rog6_inbox_hidraw->hid->dev.parent)));
}

static int rog6_inbox_usb_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	int ret = 0;
	struct inbox_drvdata *drvdata;

	if(g_Charger_mode) {
		printk("[ROG6_INBOX] In charger mode, stop rog6_inbox_usb_probe\n");
		return 0;
	}

	// default control 0x16
	IC_switch = addr_0x16;

	printk("[ROG6_INBOX] hid->name : %s\n", hdev->name);
	printk("[ROG6_INBOX] hid->vendor  : 0x%04x\n", hdev->vendor);
	printk("[ROG6_INBOX] hid->product : 0x%02x\n", hdev->product);
	//ASUSEvtlog("[ROG6_INBOX] Inbox connect\n");

	drvdata = devm_kzalloc(&hdev->dev, sizeof(*drvdata), GFP_KERNEL);
	if (drvdata == NULL) {
		hid_err(hdev, "[ROG6_INBOX] Can't alloc drvdata\n");
		return -ENOMEM;
	}
	hid_set_drvdata(hdev, drvdata);

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "[ROG6_INBOX] parse failed\n");
		goto err_free;
	}

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret) {
		hid_err(hdev, "[ROG6_INBOX] hw start failed\n");
		goto err_free;
	}

	rog6_inbox_hidraw = hdev->hidraw;

	mutex_init(&ms51_mutex);

	// Register sys class
	ret = aura_sync_register(&hdev->dev, drvdata);
	if (ret) {
		hid_err(hdev, "[ROG6_INBOX] aura_sync_register failed\n");
		goto err_free;
	}
	ret = sysfs_create_group(&drvdata->led.dev->kobj, &pwm_attr_group);
	if (ret)
		goto unregister;

	mutex_init(&update_lock);

// Set global variable
	g_2led_red_max = 255;
	g_2led_green_max = 255;
	g_2led_blue_max = 255;
	g_3led_red_max = 255;
	g_3led_green_max = 255;
	g_3led_blue_max = 255;

	g_2led_red=-1;
	g_2led_green=-1;
	g_2led_blue=-1;
	g_2led_mode=-1;
	g_2led_speed=-1;
	g_3led_red=-1;
	g_3led_green=-1;
	g_3led_blue=-1;
	g_3led_mode=-1;
	g_3led_speed=-1;

	g_led_on=-1;
	g_door_on=-1;
	g_logo_on=-1;

	INIT_DELAYED_WORK(&disable_autosuspend_work, disable_autosuspend_worker);
	schedule_delayed_work(&disable_autosuspend_work, msecs_to_jiffies(1000));

	device_init_wakeup(&interface_to_usbdev(to_usb_interface(hdev->dev.parent))->dev, true);

#if defined ASUS_AI2201_PROJECT || defined ASUS_AI2202_PROJECT
	FANDG_USBID_detect = true;
	FANDG_connect(1);
#endif

	return 0;

unregister:
	aura_sync_unregister(drvdata);
err_free:
	printk("[ROG6_INBOX] rog6_inbox_usb_probe fail.\n");
	hid_hw_stop(hdev);
	return ret;
}

static void rog6_inbox_usb_remove(struct hid_device *hdev)
{
	struct inbox_drvdata *drvdata = dev_get_drvdata(&hdev->dev);;

	if(g_Charger_mode) {
		printk("[ROG6_INBOX] In charger mode, stop rog6_inbox_usb_remove\n");
		return;
	}

	printk("[ROG6_INBOX] rog6_inbox_usb_remove\n");
#if defined ASUS_AI2201_PROJECT || defined ASUS_AI2202_PROJECT
	//ASUSEvtlog("[ROG6_INBOX] Inbox disconnect\n");
	FANDG_USBID_detect = false;
	FANDG_connect(0);
#endif

	sysfs_remove_group(&drvdata->led.dev->kobj, &pwm_attr_group);
	aura_sync_unregister(drvdata);
	rog6_inbox_hidraw = NULL;
	hid_hw_stop(hdev);
}

static struct hid_device_id rog6_inbox_idtable[] = {
	{ HID_USB_DEVICE(0x0BDA, 0x4A96),
		.driver_data = 0 },
	{ HID_USB_DEVICE(0x0BDA, 0x4BF0),
		.driver_data = 0 },
	{ HID_USB_DEVICE(0x0BDA, 0x4A80),
		.driver_data = 0 },
	{ }
};
MODULE_DEVICE_TABLE(hid, rog6_inbox_idtable);

static struct hid_driver rog6_inbox_hid_driver = {
	.name		= "rog6_inbox",
	.id_table		= rog6_inbox_idtable,
	.probe			= rog6_inbox_usb_probe,
	.remove			= rog6_inbox_usb_remove,
	.raw_event		= rog6_inbox_usb_raw_event,
#ifdef CONFIG_PM
	.suspend        = rog6_inbox_usb_suspend,
	.resume			= rog6_inbox_usb_resume,
#endif
};

static int __init rog6_inbox_usb_init(void)
{
	printk("[ROG6_INBOX] rog6_inbox_usb_init\n");
	return hid_register_driver(&rog6_inbox_hid_driver);
}

static void __exit rog6_inbox_usb_exit(void)
{
	printk("[ROG6_INBOX] rog6_inbox_usb_exit\n");
	hid_unregister_driver(&rog6_inbox_hid_driver);
}

module_init(rog6_inbox_usb_init);
module_exit(rog6_inbox_usb_exit);

MODULE_AUTHOR("ASUS Deeo");
MODULE_DESCRIPTION("ROG6 INBOX HID Interface");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("ASUS:ROG6 IBOX HID driver");
