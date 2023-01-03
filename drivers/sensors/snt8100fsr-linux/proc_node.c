#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/uaccess.h>

#define CONFIG_I2C_MODULE
#include <linux/i2c.h>
#include "device.h"
#include "memory.h"
#include "serial_bus.h"
#include "main.h"
#include "event.h"
#include "hardware.h"
#include "sysfs.h"
#include "utils.h"
#include "config.h"
#include "debug.h"
#include "sonacomm.h"
#include "workqueue.h"
#include "irq.h"

#include "locking.h"
#include "customize.h"
#include "file_control.h"
#include "proc_node.h"
#include <linux/timer.h>

void Reset_Func(struct work_struct *work);

/* Read config bank */
int cust_write_registers(void *dev, int reg, int num, void *value);
int cust_read_registers(void *dev, int reg, int num, void *value);

/*
	param. fw_loading_status:
	0: when charger/recorver or the other mode, grip fw will load fail
	1: load fw success
int fw_loading_status = 0;
*/

/* init 1V2_2V8 power status */
static int g_snt_power_state = 1;

static int SntSensor_miscOpen(struct inode *inode, struct file *file);
static int SntSensor_miscRelease(struct inode *inode, struct file *file);
static long SntSensor_miscIoctl(struct file *file, unsigned int cmd, unsigned long arg);

struct file_operations sentons_snt_fops = {
	.owner = THIS_MODULE,
	.open = SntSensor_miscOpen,
	.release = SntSensor_miscRelease,
	.unlocked_ioctl = SntSensor_miscIoctl,
	.compat_ioctl = SntSensor_miscIoctl
};
struct miscdevice sentons_snt_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "sentons_grip_sensor",
	.fops = &sentons_snt_fops
};

/*************** ASUS BSP Clay: ioctl +++ *******************/
#define ASUS_GRIP_SENSOR_DATA_SIZE 3
#define ASUS_GRIP_SENSOR_D1TEST_DATA_SIZE	784
#define ASUS_GRIP_SENSOR_NAME_SIZE 32
#define ASUS_GRIP_SENSOR_IOC_MAGIC                      ('L')///< Grip sensor ioctl magic number
#define ASUS_GRIP_SENSOR_IOCTL_ONOFF           _IOR(ASUS_GRIP_SENSOR_IOC_MAGIC, 1, int)	///< Grip sensor ioctl command - Set on/off
#define ASUS_GRIP_SENSOR_IOCTL_SET_FRAM_RATE           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 2, int)	///< Grip sensor ioctl command - Set frame rate
#define ASUS_GRIP_SENSOR_IOCTL_SET_PRESSURE_THRESHOLD           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 3, int)	///< Grip sensor ioctl command - Set pressure threshold
#define ASUS_GRIP_SENSOR_IOCTL_GET_DEBUG_MODE           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 4, int)	///< Grip sensor ioctl command - Set Debug Mode
#define ASUS_GRIP_SENSOR_IOCTL_DATA_READ           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 5, int[ASUS_GRIP_SENSOR_DATA_SIZE])	///< Grip sensor ioctl command - Data Read
#define ASUS_GRIP_SENSOR_IOCTL_MODULE_NAME           _IOR(ASUS_GRIP_SENSOR_IOC_MAGIC, 6, char[ASUS_GRIP_SENSOR_NAME_SIZE])	///< GRIP sensor ioctl command - Get module name
#define ASUS_GRIP_SENSOR_IOCTL_D1TEST_DATA_READ           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 7, unsigned char[ASUS_GRIP_SENSOR_D1TEST_DATA_SIZE])	///< Grip sensor ioctl command - D1Test Data Read
#define ASUS_GRIP_SENSOR_IOCTL_BAR0_TEST           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 8, int[2])	///< Grip sensor ioctl command - Bar Test
#define ASUS_GRIP_SENSOR_IOCTL_BAR1_TEST           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 9, int[2])	///< Grip sensor ioctl command - Bar Test
#define ASUS_GRIP_SENSOR_IOCTL_I2C_TEST           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 10, bool)	///< Grip sensor ioctl command - I2C Test
#define ASUS_GRIP_SENSOR_IOCTL_BAR0_STATUS           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 11, bool)	///< Grip sensor ioctl command - Bar Status
#define ASUS_GRIP_SENSOR_IOCTL_BAR1_STATUS           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 12, bool)	///< Grip sensor ioctl command - Bar Status
#define ASUS_GRIP_SENSOR_IOCTL_BAR_TEST_FORCE           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 13, int)	///< Grip sensor ioctl command - Bar Test Force value
#define ASUS_GRIP_SENSOR_IOCTL_BAR_TEST_TOLERANCE           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 14, int)	///< Grip sensor ioctl command - Bar Test tolerance %
#define ASUS_GRIP_SENSOR_IOCTL_SWITCH_ONOFF           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 15, int)	///< Grip sensor ioctl command - Bar Test Force value
#define ASUS_GRIP_SENSOR_IOCTL_GET_ONOFF           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 16, bool)	///< Grip sensor ioctl command - Bar Test tolerance %
#define ASUS_GRIP_SENSOR_IOCTL_BAR_TEST_HI_TOLERANCE           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 17, int)	///< Grip sensor ioctl command - Bar Test Hi tolerance %
#define ASUS_GRIP_SENSOR_IOCTL_BAR_TEST_LO_TOLERANCE           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 18, int)	///< Grip sensor ioctl command - Bar Test Lo tolerance %
#define ASUS_GRIP_SENSOR_IOCTL_BAR0_DYNAMIC_TEST           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 19, int[2])	///< Grip sensor ioctl command - Bar0 Test
#define ASUS_GRIP_SENSOR_IOCTL_BAR1_DYNAMIC_TEST           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 20, int[2])	///< Grip sensor ioctl command - Bar1 Test
#define ASUS_GRIP_SENSOR_IOCTL_ID_STATE           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 21, int)	///< Grip sensor ioctl command - get id pin status
#define ASUS_GRIP_SENSOR_IOCTL_LOAD_FW           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 22, unsigned char[512])	///< Grip sensor ioctl command - READ_FW_DATA
#define ASUS_GRIP_SENSOR_IOCTL_LOAD_FW_SIZE           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 23, int)	///< Grip sensor ioctl command - READ_FW_DATA_SIZE

static void print_current_report(int i){
	PRINT_INFO("%u %u %u %u %u %u %u\n",
				snt8100fsr_g->track_reports_frame,
				snt8100fsr_g->track_reports[i].bar_id,
				snt8100fsr_g->track_reports[i].trk_id,
				snt8100fsr_g->track_reports[i].force_lvl,
				snt8100fsr_g->track_reports[i].top,
				snt8100fsr_g->track_reports[i].center,
				snt8100fsr_g->track_reports[i].bottom);
}

static int check_report_force(int force, int force_tolerance, int force_test){
	int force_top, force_floor;
	if(force_tolerance == 100){
		return 1;
	}else if((force_tolerance > 0) && (force_tolerance < 100)){
		force_top = force_test * (100 + force_tolerance) /100;
		force_floor = force_test * (100 - force_tolerance) /100;
	}else{
		force_top = force_test;
		force_floor = force_test;
	}
	PRINT_INFO("force check: force = %d, threshould = %d, tolerance = %d percent, top = %d, floor= %d",
				force, force_test, force_tolerance, force_top, force_floor);
	if(force_test > 0){
		if(force >= force_floor && force <= force_top){
			return 1;
		}else{
			return 0;
		}
	} else {
		return 1;
	}
}

static int check_report_force_trans(int force, int force_test, int hi_tolerance, int lo_tolerance){
	int force_top, force_floor;
	if(force_test > 255){
		force_test = 255;
	}
	if(hi_tolerance == 100 && lo_tolerance == 100){
		return 1;
	}else if((hi_tolerance >= 0) && (hi_tolerance <= 100) &&
	(lo_tolerance > 0) && (lo_tolerance < 100)){
		force_top = force_test * (100 + hi_tolerance) /100;
		force_floor = force_test * (100 - lo_tolerance) /100;
	}else{
		force_top = force_test;
		force_floor = force_test;
	}
	PRINT_INFO("force check: force = %d, threshold=%d, top = %d, floor= %d, tolerance:hi= %d, lo=%d percent",
				force, force_test, force_top, force_floor, hi_tolerance, lo_tolerance);
	if(force > 0){
		if(force >= force_floor && force <= force_top){
			return 1;
		}else{
			return 0;
		}
	} else {
		return 1;
	}
}

static int SntSensor_miscOpen(struct inode *inode, struct file *file)
{
	PRINT_INFO("misc test");
#if 0
	int ret;
	if(!snt8100fsr_g){
		PRINT_CRIT("%s: null pointer, probe might not finish!", __func__);
	}
	PRINT_FUNC();
	// We don't mutex lock here, due to write_register locking
	PRINT_DEBUG("Setting frame rate to %d",
				snt8100fsr_g->suspended_frame_rate);
	ret = write_register(snt8100fsr_g,
					MAX_REGISTER_ADDRESS,
					&snt8100fsr_g->suspended_frame_rate);
	if (ret) {
		PRINT_CRIT("write_register(REGISTER_FRAME_RATE) failed");
	}

	PRINT_DEBUG("done");
#endif
	return 0;
}

static int SntSensor_miscRelease(struct inode *inode, struct file *file)
{
	//int ret;
	if(!snt8100fsr_g){
		PRINT_CRIT("%s: null pointer, probe might not finish!", __func__);
	}
	PRINT_FUNC();

	PRINT_DEBUG("done");
	return 0;
}

static int d1test_size = 784;
extern struct sc_command *sc_cmd;
static long long int grip_fw_size_for_each_payload[6] = {56, 58136, 7480, 32552, 32568, 16936};
uint8_t g_grip_fw_data_keep[6][58136];
extern int snt_read_sc_rsp(struct snt8100fsr *snt8100fsr);
static long SntSensor_miscIoctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0, i = 0;
	int snt_frame_rate = 0;
	int pressure_threshold = 0;
	int bar_test_result[2];
	int cur_frame_rate=50, i2c_flag = 1, grip_en_status = 0, switch_en;
	int fpc_status = 1, id_status = -1;
	static int force_tolerance=0, force_hi_tolerance=0, force_lo_tolerance=0;
	static int force_test = 0;
	static uint16_t RegRead_t = 0;
	unsigned char d1test_ioctl[784];
	/* fw data load */
	unsigned char fw_data[512];
	static int fw_data_size = 0;
	static long long int fw_read_total_count = 0;

	int dataSNT[ASUS_GRIP_SENSOR_DATA_SIZE];
	char nameSNT[ASUS_GRIP_SENSOR_NAME_SIZE];
	switch (cmd) {
		case ASUS_GRIP_SENSOR_IOCTL_SET_FRAM_RATE:
			ret = copy_from_user(&snt_frame_rate, (int __user*)arg, sizeof(snt_frame_rate));
			if( ret < 0) {
				PRINT_CRIT("%s: cmd = SET_FRAM_RATE, copy_from_user error(%d)\n", __func__, snt8100fsr_g->frame_rate);
				goto end;
			}
			snt8100fsr_g->frame_rate = snt_frame_rate;
			break;
		case ASUS_GRIP_SENSOR_IOCTL_SET_PRESSURE_THRESHOLD:
			ret = copy_from_user(&pressure_threshold, (int __user*)arg, sizeof(pressure_threshold));
			if( ret < 0) {
				PRINT_CRIT("%s: cmd = SET_PRESSURE_THRESHOLD, copy_from_user error(%d)\n", __func__, snt8100fsr_g->frame_rate);
				goto end;
			}
			snt8100fsr_g->pressure_threshold = pressure_threshold;
			break;
		case ASUS_GRIP_SENSOR_IOCTL_DATA_READ:
			dataSNT[0] = snt8100fsr_g->frame_rate;
			dataSNT[1] = snt8100fsr_g->pressure_threshold;
			dataSNT[2] = snt8100fsr_g->suspended_frame_rate;
			PRINT_INFO("%s: cmd = DATA_READ, data[0] = %d, data[1] = %d, data[2] = %d\n"
					 , __func__, dataSNT[0], dataSNT[1], dataSNT[2]);
			ret = copy_to_user((int __user*)arg, &dataSNT, sizeof(dataSNT));
			break;
		case ASUS_GRIP_SENSOR_IOCTL_D1TEST_DATA_READ:
			MUTEX_LOCK(&snt8100fsr_g->sb_lock);
			ret = snt_read_sc_rsp(snt8100fsr_g);
			if(log_d1test_file != NULL) {
				//Clear char array
				memset(d1test_ioctl, 0, sizeof(d1test_ioctl));
				//each sc_cmd->data[] is 4bytes
				for(i = 0; i < (d1test_size/4); i++){
					strcat(&d1test_ioctl[4*i], (unsigned char*)&sc_cmd->data[i]);
					PRINT_DEBUG("IOCTL: data[%d]: %s", i, (unsigned char*)&sc_cmd->data[i]);
				}
			}
			mutex_unlock(&snt8100fsr_g->sb_lock);

			ret = copy_to_user((int __user*)arg, &d1test_ioctl, sizeof(d1test_ioctl));
			PRINT_DEBUG("IOCTL: done");
			break;
		case ASUS_GRIP_SENSOR_IOCTL_MODULE_NAME:
			snprintf(nameSNT, sizeof(nameSNT), "%s", SYSFS_NAME);
			PRINT_INFO("%s: cmd = MODULE_NAME, name = %s\n", __func__, nameSNT);
			ret = copy_to_user((int __user*)arg, &nameSNT, sizeof(nameSNT));
			break;
		case ASUS_GRIP_SENSOR_IOCTL_BAR0_TEST:
			bar_test_result[0] = 0;
			bar_test_result[1] = 0;
			ret = read_register(snt8100fsr_g, REGISTER_SQUEEZE_FORCECALIB, &RegRead_t);
			RegRead_t = RegRead_t & 0x00FF;
			force_test = (int)RegRead_t;
			for (i = 0; i < snt8100fsr_g->track_reports_count; i++) {
				if(snt8100fsr_g->track_reports[i].bar_id == 0 && snt8100fsr_g->track_reports[i].force_lvl != 0){
					//center 56~166
					if(snt8100fsr_g->track_reports[i].center > 0 && snt8100fsr_g->track_reports[i].center < 9999 ){
						bar_test_result[1] = snt8100fsr_g->track_reports[i].force_lvl;
						bar_test_result[0] = check_report_force(bar_test_result[1], force_tolerance, force_test);
						print_current_report(i);
						break;
					}
				}
			}
			PRINT_INFO("Bar_0 Result = %d, Force = %d", bar_test_result[0], bar_test_result[1]);
			ret = copy_to_user((int __user*)arg, &bar_test_result, sizeof(bar_test_result));
			break;
		case ASUS_GRIP_SENSOR_IOCTL_BAR1_TEST:
			bar_test_result[0] = 0;
			bar_test_result[1] = 0;
			ret = read_register(snt8100fsr_g, REGISTER_SQUEEZE_FORCECALIB, &RegRead_t);
			RegRead_t = RegRead_t >> 8;
			force_test = (int)RegRead_t;
			for (i = 0; i < snt8100fsr_g->track_reports_count; i++) {
				if(snt8100fsr_g->track_reports[i].bar_id == 1 && snt8100fsr_g->track_reports[i].force_lvl != 0){
					//center 92~604
					if(snt8100fsr_g->track_reports[i].center > 0 && snt8100fsr_g->track_reports[i].center < 9999 ){
						bar_test_result[1] = snt8100fsr_g->track_reports[i].force_lvl;
						bar_test_result[0] = check_report_force(bar_test_result[1], force_tolerance, force_test);
						print_current_report(i);
						break;
					}
				}
			}
			PRINT_INFO("Bar_1 Result = %d, Force = %d", bar_test_result[0], bar_test_result[1]);
			ret = copy_to_user((int __user*)arg, &bar_test_result, sizeof(bar_test_result));
			break;
		case ASUS_GRIP_SENSOR_IOCTL_BAR_TEST_FORCE:
			ret = copy_from_user(&force_test, (int __user*)arg, sizeof(force_test));
			if( ret < 0) {
				PRINT_CRIT("%s: cmd = TEST_FORCE, copy_from_user error(%d)\n", __func__, force_test);
				goto end;
			}
			PRINT_INFO("set bar_test force = %d", force_test);
			break;
		case ASUS_GRIP_SENSOR_IOCTL_BAR_TEST_TOLERANCE:
			ret = copy_from_user(&force_tolerance, (int __user*)arg, sizeof(force_tolerance));
			if( ret < 0) {
				PRINT_CRIT("%s: cmd = ASUS_GRIP_SENSOR_IOCTL_BAR_TEST_TOLERANCE, copy_from_user error(%d)\n", __func__, force_tolerance);
				goto end;
			}
			PRINT_INFO("set bar_test tolerance = %d", force_tolerance);
			break;
		case ASUS_GRIP_SENSOR_IOCTL_BAR_TEST_HI_TOLERANCE:
			ret = copy_from_user(&force_hi_tolerance, (int __user*)arg, sizeof(force_hi_tolerance));
			if( ret < 0) {
				PRINT_CRIT("%s: cmd = HI_TOLERANCE, copy_from_user error(%d)\n", __func__, force_hi_tolerance);
				goto end;
			}
			PRINT_INFO("set bar_test hi tolerance = %d", force_hi_tolerance);
			break;
		case ASUS_GRIP_SENSOR_IOCTL_BAR_TEST_LO_TOLERANCE:
			ret = copy_from_user(&force_lo_tolerance, (int __user*)arg, sizeof(force_lo_tolerance));
			if( ret < 0) {
				PRINT_CRIT("%s: cmd = LO_TOLERANCE, copy_from_user error(%d)\n", __func__, force_lo_tolerance);
				goto end;
			}
			PRINT_INFO("set bar_test lo tolerance = %d", force_lo_tolerance);
			break;

		case ASUS_GRIP_SENSOR_IOCTL_BAR0_DYNAMIC_TEST:
			bar_test_result[0] = 0;
			bar_test_result[1] = 0;
			for (i = 0; i < snt8100fsr_g->track_reports_count; i++) {
				if(snt8100fsr_g->track_reports[i].bar_id == 0 && snt8100fsr_g->track_reports[i].force_lvl != 0){
					if(snt8100fsr_g->track_reports[i].center > 0 && snt8100fsr_g->track_reports[i].center < 9999 ){
						bar_test_result[1] = snt8100fsr_g->track_reports[i].force_lvl;
						if(bar_test_result[1]>255){
							bar_test_result[1] = 255;
						}
						bar_test_result[0] = check_report_force_trans(bar_test_result[1], force_test, force_hi_tolerance, force_lo_tolerance);
						print_current_report(i);
						break;
					}
				}
			}
			PRINT_INFO("Bar_0 Result = %d, Force = %d", bar_test_result[0], bar_test_result[1]);
			ret = copy_to_user((int __user*)arg, &bar_test_result, sizeof(bar_test_result));
			break;
		case ASUS_GRIP_SENSOR_IOCTL_BAR1_DYNAMIC_TEST:
			bar_test_result[0] = 0;
			bar_test_result[1] = 0;
			for (i = 0; i < snt8100fsr_g->track_reports_count; i++) {
				if(snt8100fsr_g->track_reports[i].bar_id == 1 && snt8100fsr_g->track_reports[i].force_lvl != 0){
					if(snt8100fsr_g->track_reports[i].center > 0 && snt8100fsr_g->track_reports[i].center < 9999 ){
						bar_test_result[1] = snt8100fsr_g->track_reports[i].force_lvl * 255;
						if(bar_test_result[1]>255){
							bar_test_result[1] = 255;
						}
						bar_test_result[0] = check_report_force_trans(bar_test_result[1], force_test, force_hi_tolerance, force_lo_tolerance);
						print_current_report(i);
						break;
					}
				}
			}
			PRINT_INFO("Bar_1 Result = %d, Force = %d", bar_test_result[0], bar_test_result[1]);
			ret = copy_to_user((int __user*)arg, &bar_test_result, sizeof(bar_test_result));
			break;
		case ASUS_GRIP_SENSOR_IOCTL_I2C_TEST:
				Wait_Wake_For_RegW();
			if(snt8100fsr_g->grip_fw_loading_status == true){
				ret = read_register (snt8100fsr_g,
					REGISTER_FRAME_RATE,
					&cur_frame_rate);
				if(ret < 0) {
					PRINT_ERR("Grip I2c no ack");
					i2c_flag = 0;
				}
			}else{
				i2c_flag = 0;
			}
			PRINT_INFO("I2C status = %d, frame_rate=%d", i2c_flag, cur_frame_rate);
			ret = copy_to_user((int __user*)arg, &i2c_flag, sizeof(i2c_flag));
			break;
		case ASUS_GRIP_SENSOR_IOCTL_GET_ONOFF:
			grip_en_status = g_snt_power_state;
			ret = copy_to_user((int __user*)arg, &grip_en_status, sizeof(i2c_flag));
			break;
		case ASUS_GRIP_SENSOR_IOCTL_SWITCH_ONOFF:
			ret = copy_from_user(&switch_en, (int __user*)arg, sizeof(switch_en));
			if( ret < 0) {
				PRINT_CRIT("%s: cmd = ASUS_GRIP_SENSOR_IOCTL_SWITCH_ONOFF, copy_from_user error(%d)\n", __func__, switch_en);
							goto end;
			}
			if(switch_en == 0 && g_snt_power_state == 1){
				Wait_Wake_For_RegW();
			}
			Power_Control(switch_en);
			PRINT_INFO("set power = %d", switch_en);
			break;
		case ASUS_GRIP_SENSOR_IOCTL_BAR0_STATUS:
			Wait_Wake_For_RegW();
				if(Health_Check(0x0003)!=0){
				fpc_status = 0;
				}
			ret = copy_to_user((int __user*)arg, &fpc_status, sizeof(fpc_status));
			break;
		case ASUS_GRIP_SENSOR_IOCTL_BAR1_STATUS:
			Wait_Wake_For_RegW();
				if(Health_Check(0x003C)!=0){
				fpc_status = 0;
				}
			ret = copy_to_user((int __user*)arg, &fpc_status, sizeof(fpc_status));
			break;
		case ASUS_GRIP_SENSOR_IOCTL_ID_STATE:
			id_status = gpio_get_value(snt8100fsr_g->snt_id_num);
			PRINT_INFO("id_status=%d", id_status);
			ret = copy_to_user((int __user*)arg, &id_status, sizeof(id_status));
                        break;
		case ASUS_GRIP_SENSOR_IOCTL_LOAD_FW:
			if(snt8100fsr_g->chip_reset_flag == GRIP_RST_FW_DL){
MUTEX_LOCK(&snt8100fsr_g->ap_lock);
				snt8100fsr_g->service_load_fw_status = true;
				ret = copy_from_user(&fw_data, (int __user*)arg, sizeof(fw_data));
				if(snt8100fsr_g->payload_count == 0 && fw_data_size == 8){
					fw_data[0] = 0x34;
					fw_data[4] = 0xfe;
					fw_data[5] = 0x06;
					PRINT_INFO("write special case in first payload1");
				}
				if(snt8100fsr_g->payload_count == 0){
					for(i = 0; i < fw_data_size; i++){
						if(i%8==0 && (i+7) < fw_data_size){
							i+=7;
							pr_info("snt: index=%d, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x",
								i,
								fw_data[i-7],
								fw_data[i-6],
								fw_data[i-5],
								fw_data[i-4],
								fw_data[i-3],
								fw_data[i-2],
								fw_data[i-1],
								fw_data[i]);
						}
					}
				}
				if (fw_data_size % SPI_FIFO_SIZE) {
					PRINT_ERR("Size not multiple of %d", SPI_FIFO_SIZE);
					return E_BADSIZE;
				}
				ret = sb_read_and_write(snt8100fsr_g, fw_data_size, fw_data, NULL);
				if (ret == FIRMWARE_ALREADY_LOADED_RESULT) {
					PRINT_NOTICE("Existing firmware already loaded...");
					snt8100fsr_g->chip_reset_flag = GRIP_FW_DL_END;
					snt8100fsr_g->grip_fw_loading_status = false;
					snt8100fsr_g->payload_count = 0;
					snt8100fsr_g->service_load_fw_status = false;
					fw_read_total_count = 0;
				} else if (ret) {
					PRINT_ERR("sb_write() failed");
					snt8100fsr_g->chip_reset_flag = GRIP_FW_DL_END;
					snt8100fsr_g->grip_fw_loading_status = false;
					snt8100fsr_g->payload_count = 0;
					snt8100fsr_g->service_load_fw_status = false;
					fw_read_total_count = 0;
				}else{
				}
				fw_read_total_count += fw_data_size;
				PRINT_INFO("snt: fw_read_total_count=%d, payload_count=%d", fw_read_total_count, snt8100fsr_g->payload_count);
				if(fw_read_total_count >= grip_fw_size_for_each_payload[snt8100fsr_g->payload_count]){
					if(snt8100fsr_g->payload_count == 5){
						PRINT_INFO("payload over 6, return 0");
						snt8100fsr_g->chip_reset_flag = GRIP_FW_DL_END;
						snt8100fsr_g->grip_fw_loading_status = true;
						snt8100fsr_g->payload_count = 0;
					}else{
						snt8100fsr_g->payload_count++;
						msleep(1000);
					}
					fw_read_total_count = 0;
				}
mutex_unlock(&snt8100fsr_g->ap_lock);
			}else{
					//PRINT_CRIT("wrong chip_reset_flag%d", snt8100fsr_g->chip_reset_flag);
			}
			break;
		case ASUS_GRIP_SENSOR_IOCTL_LOAD_FW_SIZE:
			ret = copy_from_user(&fw_data_size, (int __user*)arg, sizeof(fw_data_size));
			//pr_info("snt: fw_data_size=%d",fw_data_size);
			break;
		default:
			ret = -1;
			PRINT_INFO("%s: default\n", __func__);
	}
 end:
	return ret;
}

int sntSensor_miscRegister(void)
{
	int rtn = 0;
	/* in sys/class/misc/ */
	rtn = misc_register(&sentons_snt_misc);
	if (rtn < 0) {
		PRINT_CRIT("[%s] Unable to register misc deive\n", __func__);
		misc_deregister(&sentons_snt_misc);
	}
	return rtn;
}
/*************** ASUS BSP Clay: ioctl --- *******************/

/*************** ASUS BSP Clay: proc file +++ *******************/

void grip_frame_rate_func(int val){
	int ret;
	uint16_t RegRead_t = 0;
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	Wait_Wake_For_RegW();
	if(val == 10 || val == 20 || val == 25 || val == 40 || val == 50 || val == 80 || val == 100){
		PRINT_DEBUG("val = %d", val);
		if(grip_status_g->G_DPC_STATUS==1){
			Grip_DPC_status_g->High = val;
			RegRead_t = val;
			snt8100fsr_g->frame_rate = val;
			ret = write_register(snt8100fsr_g, REGISTER_DPC_HIGH_FRAME, &RegRead_t);
			if(ret < 0) {
				PRINT_ERR("Write reg 0x%X faill", REGISTER_DPC_HIGH_FRAME);
			}else{
				PRINT_INFO("Write DPC High: 0x%x", RegRead_t);
			}
		}else{
			snt8100fsr_g->frame_rate = val;
			ret = write_register(snt8100fsr_g, REGISTER_FRAME_RATE, &snt8100fsr_g->frame_rate);
			if(ret < 0) {
				PRINT_ERR("Write reg 0x%X faill", REGISTER_FRAME_RATE);
			}else{
				PRINT_INFO("Write frame rate: 0x%x", RegRead_t);
			}
		}
		mutex_unlock(&snt8100fsr_g->ap_lock);
	}else{
		PRINT_INFO("Not in defined frame rate range, skip val = %d", val);
		mutex_unlock(&snt8100fsr_g->ap_lock);
		return;
	}
}
/*+++BSP Clay proc asusGripDebug Interface+++*/
static int g_debugMode=0;
int asusGripDebug_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", g_debugMode);
	return 0;
}
int asusGripDebug_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, asusGripDebug_proc_read, NULL);
}

ssize_t asusGripDebug_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	int count=0, val[2] = { -1, -1};
	char *token = (char *) kmalloc(sizeof(char), GFP_KERNEL);
	char *s = (char *) kmalloc(sizeof(char), GFP_KERNEL);

	Wait_Wake_For_RegW();
	if (len > 256) {
		len = 256;
	}

	if (copy_from_user(s, buff, len)) {
		return -EFAULT;
	}
	//strlcpy(s, messages, sizeof(len));

	do {
		token = strsep(&s, " ");
		if(token!= NULL)
			val[count] = (int)simple_strtol(token, NULL, 10);
		else
			break;
		count++;
	}while(token != NULL);

	if(count ==2){
		PRINT_INFO("val=%d, %d", val[0], val[1]);
	}else{
		PRINT_INFO("count = %d, Do nothing!", count);
	}

	if(token != NULL)
		kfree(token);
	if(s != NULL)
		kfree(s);
	return len;
}
void create_asusGripDebug_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= asusGripDebug_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= asusGripDebug_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = asusGripDebug_proc_open,
		.write = asusGripDebug_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/Grip_Debug_Flag", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
/*---BSP Clay proc asusGripDebug Interface---*/


/*Calibration File read operation */
static int Grip_Calibration_raw_data_proc_read(struct seq_file *buf, void *v)
{
	int ret, i;


	MUTEX_LOCK(&snt8100fsr_g->sb_lock);

	ret = snt_read_sc_rsp(snt8100fsr_g);
	if(log_d1test_file != NULL) {
		//each sc_cmd->data[] is 4bytes
		for(i = 0; i < (d1test_size/4); i++){
			PRINT_INFO("IOCTL: data[%d]: %s", i, (unsigned char*)&sc_cmd->data[i]);
			seq_printf(buf, "%s", (unsigned char*)&sc_cmd->data[i]);
		}
	}
	mutex_unlock(&snt8100fsr_g->sb_lock);

	PRINT_INFO("proc_data: done");

	return 0;
}

static int Grip_Calibration_raw_data_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Calibration_raw_data_proc_read, NULL);
}

void create_Grip_Calibration_raw_data_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Calibration_raw_data_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= NULL,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Calibration_raw_data_proc_open,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/Grip_D1test", 0666, NULL, &proc_fops);
	if (!proc_file) {
		PRINT_ERR("%s failed!\n", __func__);
	}
	return;
}
/*Calibration File read operation*/

/* +++ BSP Clay proc i2c check +++ */
static int Grip_I2c_Check_proc_read(struct seq_file *buf, void *v)
{
	int ret, i2c_status;
	bool flag = 1;
	Wait_Wake_For_RegW();
	ret = read_register (snt8100fsr_g, REGISTER_FRAME_RATE, &i2c_status);
	if(ret < 0) {
		PRINT_ERR("Grip I2c no ack");
		flag = 0;
		goto Report;
	}

Report:
	if(flag == 1){
		seq_printf(buf, "%d\n", flag);
	}else{
		seq_printf(buf, "%d\n", flag);
	}
	return 0;
}
static int Grip_I2c_Check_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_I2c_Check_proc_read, NULL);
}
void create_Grip_I2c_Check_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_I2c_Check_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= NULL,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_I2c_Check_proc_open,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/Grip_I2c_Check", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
/* --- BSP Clay proc i2c check --- */
static ssize_t Grip_ReadK_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{

	int val;
	char messages[2048];
	/*
	if(g_Charger_mode){
		PRINT_INFO("Charger mode, do nothing");
		return len;
	}
	*/
	if (len > 2048) {
		len = 2048;
	}
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	val = (int)simple_strtol(messages, NULL, 10);

	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	Wait_Wake_For_RegW();
	//Grip_Chip_IRQ_EN(1);

	//PRINT_INFO("Str: %s, %zu bytes", messages, len);
	enable_boot_init_reg_req(snt8100fsr_g, messages, len);
	Into_DeepSleep_fun();
	mutex_unlock(&snt8100fsr_g->ap_lock);
	Grip_ReEnable_Squeeze_Check();
	return len;
}

static int Grip_ReadK_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "0\n");
	return 0;
}
static int Grip_ReadK_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_ReadK_proc_read, NULL);
}
void create_Grip_ReadK_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_ReadK_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_ReadK_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_ReadK_proc_open,
		.write = Grip_ReadK_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/Grip_ReadK", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

/* +++ BSP Clay proc FPC check --- */
/* +++ BSP Clay Disable wake_lock and grip event +++ */
bool wake_lock_disable_flag = 0;;
static int Grip_Disable_WakeLock_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "wake_lock_evt_flag: %d\n", wake_lock_disable_flag);
	return 0;
}

static ssize_t Grip_Disable_WakeLock_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	int val, ret, reg_en = 0;
	char messages[256];
	if (len > 256) {
		len = 256;
	}
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	val = (int)simple_strtol(messages, NULL, 10);
	wake_lock_disable_flag = val;
	if(wake_lock_disable_flag == 0){
		reg_en = 1;
	}else{
		__pm_relax(snt8100fsr_g->snt_wakelock);
	}
	Wait_Wake_For_RegW();
	ret = read_register(snt8100fsr_g, REGISTER_ENABLE, &reg_en);
	if(ret < 0) {
		PRINT_ERR("Grip register_enable write fail");
	}else{
		PRINT_INFO("reg_en = %d", reg_en);
	}

	return len;
}

static int Grip_Disable_WakeLock_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Disable_WakeLock_proc_read, NULL);
}

void create_Grip_Disable_WakeLock_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Disable_WakeLock_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Disable_WakeLock_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Disable_WakeLock_proc_open,
		.write = Grip_Disable_WakeLock_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/Grip_Disable_WakeLock", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
/* --- BSP Clay R/W Temp register value --- */
/* +++ BSP Clay Frame Rate setting +++ */
static int Grip_frame_proc_read(struct seq_file *buf, void *v)
{
	if(grip_status_g->G_DPC_STATUS==1)
		seq_printf(buf, "%d\n", Grip_DPC_status_g->High);
	else
		seq_printf(buf, "%d\n", snt8100fsr_g->frame_rate);
	return 0;
}
static ssize_t Grip_frame_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	int val;
	char messages[256];

	memset(messages, 0, sizeof(messages));
	if (len > 256) {
		len = 256;
	}
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	val = (int)simple_strtol(messages, NULL, 10);
	grip_frame_rate_func(val);
	return len;
}

static int Grip_frame_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_frame_proc_read, NULL);
}

void create_Grip_frame_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_frame_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_frame_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_frame_proc_open,
		.write = Grip_frame_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_frame_rate", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

/* --- BSP Clay Frame Rate setting --- */
//==============Enable Interface=============//
static int Grip_raw_en_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", grip_status_g->G_RAW_EN);
	return 0;
}
static ssize_t Grip_raw_en_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	int val;
	char messages[256];

	memset(messages, 0, sizeof(messages));
	if (len > 256) {
		len = 256;
	}
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	val = (int)simple_strtol(messages, NULL, 10);

	if(grip_status_g->G_RAW_EN==val){
		PRINT_INFO("repeat, skip it");
		return len;
	}
		grip_raw_enable_func(val);
	return len;
}

static int Grip_raw_en_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_raw_en_proc_read, NULL);
}

void create_Grip_raw_en_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_raw_en_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_raw_en_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_raw_en_proc_open,
		.write = Grip_raw_en_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_raw_en", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

/*+++ Sensor Grip Gesture +++ */
static int Grip_en_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", grip_status_g->G_EN);
	return 0;
}
static ssize_t Grip_en_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	/*
	int val;
	char messages[256];

	memset(messages, 0, sizeof(messages));
	if (len > 256) {
		len = 256;
	}
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	val = (int)simple_strtol(messages, NULL, 10);
	grip_enable_func_noLock(val);
	*/
	PRINT_INFO("Do nothing");
	return len;
}

static int Grip_en_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_en_proc_read, NULL);
}

void create_Grip_en_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_en_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_en_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_en_proc_open,
		.write = Grip_en_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_en", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

//Tap Sense Enable Interface
int Grip_Tap_Sense_En_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", grip_status_g->G_TAP_SENSE_SET);
	return 0;
}

ssize_t Grip_Tap_Sense_En_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	int val;
	char messages[256];

	memset(messages, 0, sizeof(messages));
	if (len > 256) {
		len = 256;
	}
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	val = (int)simple_strtol(messages, NULL, 10);
	if(grip_status_g->G_TAP_SENSE_SET ==val){
		PRINT_INFO("repeat, skip it");
		return len;
	}
	grip_tap_sense_enable_func(val);
	return len;
}

int Grip_Tap_Sense_En_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Tap_Sense_En_proc_read, NULL);
}

void create_Grip_Tap_Sense_En_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Tap_Sense_En_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Tap_Sense_En_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Tap_Sense_En_proc_open,
		.write = Grip_Tap_Sense_En_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_tap_sense_en", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
/* --- BSP Clay Grip Gesture --- */

//=======================================================
//=======================================================
//=======================================================
static int parse_result[2] = {-1};
static char *parse_token = NULL;
static char *string_temp = NULL;
static int Grip_node_parse_string(const char __user *buff, size_t len, int *reg_val){
	int count=0, val[2] = { -1, -1};
	int result=-1;
	char message[256];

	memset(message, 0, sizeof(message));
	if(parse_token == NULL){
		PRINT_DEBUG("parse_token KMALLOC MEMORY!!!!");
		parse_token = (char *) kmalloc(sizeof(char), GFP_KERNEL);
	}
	if(string_temp == NULL){
		PRINT_DEBUG("string_temp KMALLOC MEMORY!!!!");
		parse_token = (char *) kmalloc(sizeof(char), GFP_KERNEL);
	}

	memset(parse_result, -1, sizeof(*parse_result));

	if (len > 256) {
		len = 256;
	}
	if (copy_from_user(message, buff, len)) {
		result = -EFAULT;
		goto Final;
	}
	string_temp = message;

	do {
		if(count > 0){
			if(parse_token[0] > 57 || parse_token[0] < 48){
				PRINT_ERR("1. count=%d, parse_token=%s, string_temp=%s", count, parse_token, string_temp);
				break;
			}
		}

		parse_token = strsep(&string_temp, " ");
		val[count] = (int)simple_strtol(parse_token, NULL, 10);
		PRINT_DEBUG("count=%d, val[%d]=%s, parse_token=%s, string_temp=%s", count, val[count], parse_token, string_temp);
		count++;
		if(string_temp == NULL){
			PRINT_DEBUG("string_temp == NULL");
			break;
		}else if(string_temp[0] > 57 || string_temp[0] < 48){
			PRINT_ERR("2. count=%d, parse_token=%s, string_temp=%s", count, parse_token, string_temp);
			break;
		}
	}while(parse_token != NULL);

	if(count ==2){
		if(reg_val[val[0]]==val[1]){
			PRINT_DEBUG("repeat, skip it, input1=%d, input2=%d", val[0], val[1]);
			goto Final;
		}else
			result = 1;

		PRINT_DEBUG("val=%d, %d", val[0], val[1]);
		parse_result[0] = val[0];
		parse_result[1] = val[1];
	}else{
		PRINT_ERR("Error input count = %d, Do nothing!", count);
	}

Final:
	PRINT_DEBUG("Done");
	return result;
}
int Grip_Tap_En_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d,%d,%d\n",
		grip_status_g->G_TAP_EN[0],
		grip_status_g->G_TAP_EN[1],
		grip_status_g->G_TAP_EN[2],
		grip_status_g->G_TAP_EN[3]);
	return 0;
}

ssize_t Grip_Tap_En_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_TAP_EN) > 0)
		grip_tapX_enable_func(parse_result[1], parse_result[0], &TAP_BIT0[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");

	return len;
}

int Grip_Tap_En_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Tap_En_proc_read, NULL);
}

void create_Grip_Tap_En_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Tap_En_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Tap_En_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Tap_En_proc_open,
		.write = Grip_Tap_En_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_tap_en", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Tap_Force_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d,%d,%d\n",
		grip_status_g->G_TAP_FORCE[0],
		grip_status_g->G_TAP_FORCE[1],
		grip_status_g->G_TAP_FORCE[2],
		grip_status_g->G_TAP_FORCE[3]);
	return 0;
}

ssize_t Grip_Tap_Force_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_TAP_FORCE) > 0)
		grip_tapX_force_func(parse_result[1], parse_result[0], &TAP_BIT0[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Tap_Force_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Tap_Force_proc_read, NULL);
}

void create_Grip_Tap_Force_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Tap_Force_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Tap_Force_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Tap_Force_proc_open,
		.write = Grip_Tap_Force_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_tap_force", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}


int Grip_Tap_min_pos_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d,%d,%d\n",
		grip_status_g->G_TAP_MIN_POS[0],
		grip_status_g->G_TAP_MIN_POS[1],
		grip_status_g->G_TAP_MIN_POS[2],
		grip_status_g->G_TAP_MIN_POS[3]);
	return 0;
}

ssize_t Grip_Tap_min_pos_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_TAP_MIN_POS) > 0)
		grip_tapX_min_position_func(parse_result[1], parse_result[0], &TAP_BIT2[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Tap_min_pos_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Tap_min_pos_proc_read, NULL);
}

void create_Grip_Tap_min_pos_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Tap_min_pos_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Tap_min_pos_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Tap_min_pos_proc_open,
		.write = Grip_Tap_min_pos_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_tap_min_position", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Tap_max_pos_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d,%d,%d\n",
		grip_status_g->G_TAP_MAX_POS[0],
		grip_status_g->G_TAP_MAX_POS[1],
		grip_status_g->G_TAP_MAX_POS[2],
		grip_status_g->G_TAP_MAX_POS[3]);
	return 0;
}

ssize_t Grip_Tap_max_pos_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_TAP_MAX_POS) > 0)
		grip_tapX_max_position_func(parse_result[1], parse_result[0], &TAP_BIT3[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Tap_max_pos_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Tap_max_pos_proc_read, NULL);
}

void create_Grip_Tap_max_pos_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Tap_max_pos_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Tap_max_pos_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Tap_max_pos_proc_open,
		.write = Grip_Tap_max_pos_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_tap_max_position", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Tap_slope_window_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d,%d,%d\n",
		grip_status_g->G_TAP_SLOPE_WINDOW[0],
		grip_status_g->G_TAP_SLOPE_WINDOW[1],
		grip_status_g->G_TAP_SLOPE_WINDOW[2],
		grip_status_g->G_TAP_SLOPE_WINDOW[3]);
	return 0;
}

ssize_t Grip_Tap_slope_window_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_TAP_SLOPE_WINDOW) > 0)
		grip_tapX_slope_window_func(parse_result[1], parse_result[0], &TAP_BIT1[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Tap_slope_window_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Tap_slope_window_proc_read, NULL);
}

void create_Grip_Tap_slope_window_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Tap_slope_window_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Tap_slope_window_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Tap_slope_window_proc_open,
		.write = Grip_Tap_slope_window_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_tap_slope_window", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Tap_slope_tap_force_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d,%d,%d\n",
		grip_status_g->G_TAP_SLOPE_TAP_FORCE[0],
		grip_status_g->G_TAP_SLOPE_TAP_FORCE[1],
		grip_status_g->G_TAP_SLOPE_TAP_FORCE[2],
		grip_status_g->G_TAP_SLOPE_TAP_FORCE[3]);
	return 0;
}

ssize_t Grip_Tap_slope_tap_force_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_TAP_SLOPE_TAP_FORCE) > 0)
		grip_tapX_slope_tap_force_func(parse_result[1], parse_result[0], &TAP_BIT5[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Tap_slope_tap_force_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Tap_slope_tap_force_proc_read, NULL);
}

void create_Grip_Tap_slope_tap_force_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Tap_slope_tap_force_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Tap_slope_tap_force_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Tap_slope_tap_force_proc_open,
		.write = Grip_Tap_slope_tap_force_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_tap_slope_tap_force", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Tap_slope_release_force_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d,%d,%d\n",
		grip_status_g->G_TAP_SLOPE_RELEASE_FORCE[0],
		grip_status_g->G_TAP_SLOPE_RELEASE_FORCE[1],
		grip_status_g->G_TAP_SLOPE_RELEASE_FORCE[2],
		grip_status_g->G_TAP_SLOPE_RELEASE_FORCE[3]);
	return 0;
}

ssize_t Grip_Tap_slope_release_force_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_TAP_SLOPE_RELEASE_FORCE) > 0)
		grip_tapX_slope_release_force_func(parse_result[1], parse_result[0], &TAP_BIT5[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Tap_slope_release_force_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Tap_slope_release_force_proc_read, NULL);
}

void create_Grip_Tap_slope_release_force_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Tap_slope_release_force_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Tap_slope_release_force_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Tap_slope_release_force_proc_open,
		.write = Grip_Tap_slope_release_force_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_tap_slope_release_force", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Tap_delta_tap_force_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d,%d,%d\n",
		grip_status_g->G_TAP_DELTA_TAP_FORCE[0],
		grip_status_g->G_TAP_DELTA_TAP_FORCE[1],
		grip_status_g->G_TAP_DELTA_TAP_FORCE[2],
		grip_status_g->G_TAP_DELTA_TAP_FORCE[3]);
	return 0;
}

ssize_t Grip_Tap_delta_tap_force_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_TAP_DELTA_TAP_FORCE) > 0)
		grip_tapX_delta_tap_force_func(parse_result[1], parse_result[0], &TAP_BIT4[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Tap_delta_tap_force_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Tap_delta_tap_force_proc_read, NULL);
}

void create_Grip_Tap_delta_tap_force_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Tap_delta_tap_force_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Tap_delta_tap_force_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Tap_delta_tap_force_proc_open,
		.write = Grip_Tap_delta_tap_force_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_tap_delta_tap_force", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Tap_delta_release_force_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d,%d,%d\n",
		grip_status_g->G_TAP_DELTA_RELEASE_FORCE[0],
		grip_status_g->G_TAP_DELTA_RELEASE_FORCE[1],
		grip_status_g->G_TAP_DELTA_RELEASE_FORCE[2],
		grip_status_g->G_TAP_DELTA_RELEASE_FORCE[3]);
	return 0;
}

ssize_t Grip_Tap_delta_release_force_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_TAP_DELTA_RELEASE_FORCE) > 0)
		grip_tapX_delta_release_force_func(parse_result[1], parse_result[0], &TAP_BIT4[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Tap_delta_release_force_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Tap_delta_release_force_proc_read, NULL);
}

void create_Grip_Tap_delta_release_force_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Tap_delta_release_force_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Tap_delta_release_force_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Tap_delta_release_force_proc_open,
		.write = Grip_Tap_delta_release_force_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_tap_delta_release_force", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Tap_vib_en_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d,%d,%d\n",
		grip_status_g->G_TAP_VIB_EN[0],
		grip_status_g->G_TAP_VIB_EN[1],
		grip_status_g->G_TAP_VIB_EN[2],
		grip_status_g->G_TAP_VIB_EN[3]);
	return 0;
}

ssize_t Grip_Tap_vib_en_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_TAP_VIB_EN) > 0)
		grip_tapX_vibrator_enable_func(parse_result[1], parse_result[0]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Tap_vib_en_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Tap_vib_en_proc_read, NULL);
}

void create_Grip_Tap_vib_en_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Tap_vib_en_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Tap_vib_en_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Tap_vib_en_proc_open,
		.write = Grip_Tap_vib_en_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_tap_vib_en", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Tap_vib_repeat_en_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d,%d,%d\n",
		grip_status_g->G_TAP_VIB_REPEAT_EN[0],
		grip_status_g->G_TAP_VIB_REPEAT_EN[1],
		grip_status_g->G_TAP_VIB_REPEAT_EN[2],
		grip_status_g->G_TAP_VIB_REPEAT_EN[3]);
	return 0;
}

ssize_t Grip_Tap_vib_repeat_en_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_TAP_VIB_REPEAT_EN) > 0)
		grip_tapX_vibrator_repeat_enable_func(parse_result[1], parse_result[0]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Tap_vib_repeat_en_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Tap_vib_repeat_en_proc_read, NULL);
}

void create_Grip_Tap_vib_repeat_en_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Tap_vib_repeat_en_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Tap_vib_repeat_en_proc_write,
		.proc_release	= single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_tap_vib_repeat_en", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Tap_vib_repeat_dur_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d,%d,%d\n",
		grip_status_g->G_TAP_VIB_REPEAT_DUR[0],
		grip_status_g->G_TAP_VIB_REPEAT_DUR[1],
		grip_status_g->G_TAP_VIB_REPEAT_DUR[2],
		grip_status_g->G_TAP_VIB_REPEAT_DUR[3]);
	return 0;
}

ssize_t Grip_Tap_vib_repeat_dur_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_TAP_VIB_REPEAT_DUR) > 0){
		grip_tapX_vibrator_repeat_dur_func(parse_result[1], parse_result[0]);

	}else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Tap_vib_repeat_dur_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Tap_vib_repeat_dur_proc_read, NULL);
}

void create_Grip_Tap_vib_repeat_dur_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Tap_vib_repeat_dur_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Tap_vib_repeat_dur_proc_write,
		.proc_release	= single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_tap_vib_repeat_dur", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

//============================================================
//=======================TAP Part Done!!!==========================
//============================================================
int Grip_Squeeze_en_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n",
		grip_status_g->G_SQUEEZE_EN[0],
		grip_status_g->G_SQUEEZE_EN[1]);
	return 0;
}

ssize_t Grip_Squeeze_en_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SQUEEZE_EN) > 0)
		grip_squeezeX_enable_func(parse_result[1], parse_result[0], &SQ_BIT0[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Squeeze_en_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Squeeze_en_proc_read, NULL);
}

void create_Grip_Squeeze_en_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Squeeze_en_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Squeeze_en_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Squeeze_en_proc_open,
		.write = Grip_Squeeze_en_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_squeeze_en", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Squeeze_force_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n",
		grip_status_g->G_SQUEEZE_FORCE[0],
		grip_status_g->G_SQUEEZE_FORCE[1]);
	return 0;
}

ssize_t Grip_Squeeze_force_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SQUEEZE_FORCE) > 0)
		grip_squeezeX_force_func(parse_result[1], parse_result[0], &SQ_BIT0[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Squeeze_force_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Squeeze_force_proc_read, NULL);
}

void create_Grip_Squeeze_force_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Squeeze_force_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Squeeze_force_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Squeeze_force_proc_open,
		.write = Grip_Squeeze_force_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_squeeze_force", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Squeeze_short_dur_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n",
		grip_status_g->G_SQUEEZE_SHORT[0],
		grip_status_g->G_SQUEEZE_SHORT[1]);
	return 0;
}

ssize_t Grip_Squeeze_short_dur_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SQUEEZE_SHORT) > 0)
		grip_squeezeX_short_dur_func(parse_result[1], parse_result[0], &SQ_BIT5[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Squeeze_short_dur_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Squeeze_short_dur_proc_read, NULL);
}

void create_Grip_Squeeze_short_dur_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Squeeze_short_dur_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Squeeze_short_dur_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Squeeze_short_dur_proc_open,
		.write = Grip_Squeeze_short_dur_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_squeeze_short_dur", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Squeeze_long_dur_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n",
		grip_status_g->G_SQUEEZE_LONG[0],
		grip_status_g->G_SQUEEZE_LONG[1]);
	return 0;
}

ssize_t Grip_Squeeze_long_dur_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SQUEEZE_LONG) > 0)
		grip_squeezeX_long_dur_func(parse_result[1], parse_result[0], &SQ_BIT5[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Squeeze_long_dur_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Squeeze_long_dur_proc_read, NULL);
}

void create_Grip_Squeeze_long_dur_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Squeeze_long_dur_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Squeeze_long_dur_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Squeeze_long_dur_proc_open,
		.write = Grip_Squeeze_long_dur_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_squeeze_long_dur", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Squeeze_BarA_drop_rate_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n",
		grip_status_g->G_SQUEEZE_BARA_DROP_RATE[0],
		grip_status_g->G_SQUEEZE_BARA_DROP_RATE[1]);
	return 0;
}

ssize_t Grip_Squeeze_BarA_drop_rate_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SQUEEZE_BARA_DROP_RATE) > 0)
		grip_squeezeX_BarA_drop_rate_func(parse_result[1], parse_result[0], &SQ_BIT7[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Squeeze_BarA_drop_rate_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Squeeze_BarA_drop_rate_proc_read, NULL);
}

void create_Grip_Squeeze_BarA_drop_rate_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Squeeze_BarA_drop_rate_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Squeeze_BarA_drop_rate_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Squeeze_drop_rate_proc_open,
		.write = Grip_Squeeze_drop_rate_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_squeeze_bara_drop_rate", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Squeeze_BarA_drop_rate_acc_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n",
		grip_status_g->G_SQUEEZE_BARA_DROP_RATE_ACC[0],
		grip_status_g->G_SQUEEZE_BARA_DROP_RATE_ACC[1]);
	return 0;
}

ssize_t Grip_Squeeze_BarA_drop_rate_acc_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SQUEEZE_BARA_DROP_RATE_ACC) > 0)
		grip_squeezeX_BarA_drop_rate_acc_func(parse_result[1], parse_result[0], &SQ_BIT7[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Squeeze_BarA_drop_rate_acc_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Squeeze_BarA_drop_rate_acc_proc_read, NULL);
}

void create_Grip_Squeeze_BarA_drop_rate_acc_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Squeeze_BarA_drop_rate_acc_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Squeeze_BarA_drop_rate_acc_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Squeeze_drop_total_proc_open,
		.write = Grip_Squeeze_drop_total_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_squeeze_bara_drop_rate_acc", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Squeeze_BarA_up_rate_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n",
		grip_status_g->G_SQUEEZE_BARA_UP_RATE[0],
		grip_status_g->G_SQUEEZE_BARA_UP_RATE[1]);
	return 0;
}

ssize_t Grip_Squeeze_BarA_up_rate_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SQUEEZE_BARA_UP_RATE) > 0)
		grip_squeezeX_BarA_up_rate_func(parse_result[1], parse_result[0], &SQ_BIT6[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Squeeze_BarA_up_rate_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Squeeze_BarA_up_rate_proc_read, NULL);
}

void create_Grip_Squeeze_BarA_up_rate_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Squeeze_BarA_up_rate_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Squeeze_BarA_up_rate_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Squeeze_up_rate_proc_open,
		.write = Grip_Squeeze_up_rate_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_squeeze_bara_up_rate", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Squeeze_BarA_up_rate_acc_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n",
		grip_status_g->G_SQUEEZE_BARA_UP_RATE_ACC[0],
		grip_status_g->G_SQUEEZE_BARA_UP_RATE_ACC[1]);
	return 0;
}

ssize_t Grip_Squeeze_BarA_up_rate_acc_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SQUEEZE_BARA_UP_RATE_ACC) > 0)
		grip_squeezeX_BarA_up_rate_acc_func(parse_result[1], parse_result[0], &SQ_BIT6[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Squeeze_BarA_up_rate_acc_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Squeeze_BarA_up_rate_acc_proc_read, NULL);
}

void create_Grip_Squeeze_BarA_up_rate_acc_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Squeeze_BarA_up_rate_acc_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Squeeze_BarA_up_rate_acc_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Squeeze_up_total_proc_open,
		.write = Grip_Squeeze_up_total_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_squeeze_bara_up_rate_acc", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}


int Grip_Squeeze_BarB_drop_rate_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n",
		grip_status_g->G_SQUEEZE_BARB_DROP_RATE[0],
		grip_status_g->G_SQUEEZE_BARB_DROP_RATE[1]);
	return 0;
}

ssize_t Grip_Squeeze_BarB_drop_rate_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SQUEEZE_BARB_DROP_RATE) > 0)
		grip_squeezeX_BarB_drop_rate_func(parse_result[1], parse_result[0], &SQ_BIT7[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Squeeze_BarB_drop_rate_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Squeeze_BarB_drop_rate_proc_read, NULL);
}

void create_Grip_Squeeze_BarB_drop_rate_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Squeeze_BarB_drop_rate_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Squeeze_BarB_drop_rate_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Squeeze_drop_rate_proc_open,
		.write = Grip_Squeeze_drop_rate_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_squeeze_barb_drop_rate", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Squeeze_BarB_drop_rate_acc_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n",
		grip_status_g->G_SQUEEZE_BARB_DROP_RATE_ACC[0],
		grip_status_g->G_SQUEEZE_BARB_DROP_RATE_ACC[1]);
	return 0;
}

ssize_t Grip_Squeeze_BarB_drop_rate_acc_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SQUEEZE_BARB_DROP_RATE_ACC) > 0)
		grip_squeezeX_BarB_drop_rate_acc_func(parse_result[1], parse_result[0], &SQ_BIT7[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Squeeze_BarB_drop_rate_acc_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Squeeze_BarB_drop_rate_acc_proc_read, NULL);
}

void create_Grip_Squeeze_BarB_drop_rate_acc_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Squeeze_BarB_drop_rate_acc_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Squeeze_BarB_drop_rate_acc_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Squeeze_drop_total_proc_open,
		.write = Grip_Squeeze_drop_total_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_squeeze_barb_drop_rate_acc", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Squeeze_BarB_up_rate_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n",
		grip_status_g->G_SQUEEZE_BARB_UP_RATE[0],
		grip_status_g->G_SQUEEZE_BARB_UP_RATE[1]);
	return 0;
}

ssize_t Grip_Squeeze_BarB_up_rate_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SQUEEZE_BARB_UP_RATE) > 0)
		grip_squeezeX_BarB_up_rate_func(parse_result[1], parse_result[0], &SQ_BIT6[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Squeeze_BarB_up_rate_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Squeeze_BarB_up_rate_proc_read, NULL);
}

void create_Grip_Squeeze_BarB_up_rate_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Squeeze_BarB_up_rate_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Squeeze_BarB_up_rate_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Squeeze_up_rate_proc_open,
		.write = Grip_Squeeze_up_rate_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_squeeze_barb_up_rate", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Squeeze_BarB_up_rate_acc_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n",
		grip_status_g->G_SQUEEZE_BARB_UP_RATE_ACC[0],
		grip_status_g->G_SQUEEZE_BARB_UP_RATE_ACC[1]);
	return 0;
}

ssize_t Grip_Squeeze_BarB_up_rate_acc_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SQUEEZE_BARB_UP_RATE_ACC) > 0)
		grip_squeezeX_BarB_up_rate_acc_func(parse_result[1], parse_result[0], &SQ_BIT6[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Squeeze_BarB_up_rate_acc_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Squeeze_BarB_up_rate_acc_proc_read, NULL);
}

void create_Grip_Squeeze_BarB_up_rate_acc_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Squeeze_BarB_up_rate_acc_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Squeeze_BarB_up_rate_acc_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Squeeze_up_total_proc_open,
		.write = Grip_Squeeze_up_total_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_squeeze_barb_up_rate_acc", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

//============================================================
//=======================SQUEEZE Part Done!!!==========================
//============================================================
int Grip_Slide_en_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n",
		grip_status_g->G_SLIDE_EN[0],
		grip_status_g->G_SLIDE_EN[1]);
	return 0;
}

ssize_t Grip_Slide_en_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SLIDE_EN) > 0)
		grip_slideX_enable_func(parse_result[1], parse_result[0], &SLIDE_BIT0[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Slide_en_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Slide_en_proc_read, NULL);
}

void create_Grip_Slide_en_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Slide_en_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Slide_en_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Slide_en_proc_open,
		.write = Grip_Slide_en_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_slide_en", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Slide_dist_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n",
		grip_status_g->G_SLIDE_DIST[0],
		grip_status_g->G_SLIDE_DIST[1]);
	return 0;
}

ssize_t Grip_Slide_dist_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SLIDE_DIST) > 0)
		grip_slideX_dist_func(parse_result[1], parse_result[0], &SLIDE_BIT3[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");

	return len;
}

int Grip_Slide_dist_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Slide_dist_proc_read, NULL);
}

void create_Grip_Slide_dist_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Slide_dist_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Slide_dist_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Slide_dist_proc_open,
		.write = Grip_Slide_dist_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_slide_dist", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Slide_2nd_dist_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n",
		grip_status_g->G_SLIDE_2ND_DIST[0],
		grip_status_g->G_SLIDE_2ND_DIST[1]);
	return 0;
}

ssize_t Grip_Slide_2nd_dist_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SLIDE_2ND_DIST) > 0)
		grip_slideX_2nd_dist_func(parse_result[1], parse_result[0], &SLIDE_BIT4[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");

	return len;
}

int Grip_Slide_2nd_dist_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Slide_2nd_dist_proc_read, NULL);
}

void create_Grip_Slide_2nd_dist_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Slide_2nd_dist_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Slide_2nd_dist_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Slide_2nd_dist_proc_open,
		.write = Grip_Slide_2nd_dist_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_slide_2nd_dist", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Slide_force_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n",
		grip_status_g->G_SLIDE_FORCE[0],
		grip_status_g->G_SLIDE_FORCE[1]);
	return 0;
}

ssize_t Grip_Slide_force_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SLIDE_FORCE) > 0)
		grip_slideX_force_func(parse_result[1], parse_result[0], &SLIDE_BIT3[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");

	return len;
}

int Grip_Slide_force_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Slide_force_proc_read, NULL);
}

void create_Grip_Slide_force_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Slide_force_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Slide_force_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Slide_force_proc_open,
		.write = Grip_Slide_force_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_slide_force", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Slide_min_pos_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n",
		grip_status_g->G_SLIDE_MIN_POS[0],
		grip_status_g->G_SLIDE_MIN_POS[1]);
	return 0;
}

ssize_t Grip_Slide_min_pos_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SLIDE_MIN_POS) > 0)
		grip_slideX_min_position_func(parse_result[1], parse_result[0], &SLIDE_BIT1[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");

	return len;
}

int Grip_Slide_min_pos_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Slide_min_pos_proc_read, NULL);
}

void create_Grip_Slide_min_pos_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Slide_min_pos_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Slide_min_pos_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Slide_min_pos_proc_open,
		.write = Grip_Slide_min_pos_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_slide_min_pos", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Slide_max_pos_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n",
		grip_status_g->G_SLIDE_MAX_POS[0],
		grip_status_g->G_SLIDE_MAX_POS[1]);
	return 0;
}

ssize_t Grip_Slide_max_pos_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SLIDE_MAX_POS) > 0)
		grip_slideX_max_position_func(parse_result[1], parse_result[0], &SLIDE_BIT2[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");

	return len;
}

int Grip_Slide_max_pos_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Slide_max_pos_proc_read, NULL);
}

void create_Grip_Slide_max_pos_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Slide_max_pos_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Slide_max_pos_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Slide_max_pos_proc_open,
		.write = Grip_Slide_max_pos_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_slide_max_pos", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Slide_vib_en_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d,%d,%d\n",
		grip_status_g->G_SLIDE_VIB_EN[0],
		grip_status_g->G_SLIDE_VIB_EN[1]);
	return 0;
}

ssize_t Grip_Slide_vib_en_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SLIDE_VIB_EN) > 0)
		grip_slideX_vibrator_enable_func(parse_result[1], parse_result[0]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Slide_vib_en_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Slide_vib_en_proc_read, NULL);
}

void create_Grip_Slide_vib_en_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Slide_vib_en_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Slide_vib_en_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Slide_tap_priority_proc_open,
		.write = Grip_Slide_tap_priority_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_slide_vib_en", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Slide_tap_priority_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d,%d,%d\n",
		grip_status_g->G_SLIDE_TAP_PRIORITY[0],
		grip_status_g->G_SLIDE_TAP_PRIORITY[1]);
	return 0;
}

ssize_t Grip_Slide_tap_priority_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SLIDE_TAP_PRIORITY) > 0)
		grip_slideX_tap_priority_func(parse_result[1], parse_result[0], &SLIDE_BIT0[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Slide_tap_priority_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Slide_tap_priority_proc_read, NULL);
}

void create_Grip_Slide_tap_priority_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Slide_tap_priority_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Slide_tap_priority_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Slide_tap_priority_proc_open,
		.write = Grip_Slide_tap_priority_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_slide_tap_priority", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

//============================================================
//=======================SLIDE Part Done!!!==========================
//============================================================

int Grip_Swipe_en_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n",
		grip_status_g->G_SWIPE_EN[0],
		grip_status_g->G_SWIPE_EN[1]);
	return 0;
}

ssize_t Grip_Swipe_en_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SWIPE_EN) > 0)
		grip_swipeX_enable_func(parse_result[1], parse_result[0], &SWIPE_BIT0[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Swipe_en_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Swipe_en_proc_read, NULL);
}

void create_Grip_Swipe_en_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Swipe_en_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Swipe_en_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Swipe_en_proc_open,
		.write = Grip_Swipe_en_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_swipe_en", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
int Grip_Swipe_velocity_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n",
		grip_status_g->G_SWIPE_VELOCITY[0],
		grip_status_g->G_SWIPE_VELOCITY[1]);
	return 0;
}

ssize_t Grip_Swipe_velocity_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SWIPE_VELOCITY) > 0)
		grip_swipeX_velocity_func(parse_result[1], parse_result[0], &SWIPE_BIT0[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Swipe_velocity_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Swipe_velocity_proc_read, NULL);
}

void create_Grip_Swipe_velocity_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Swipe_velocity_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Swipe_velocity_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Swipe_velocity_proc_open,
		.write = Grip_Swipe_velocity_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_swipe_v", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Swipe_len_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n",
		grip_status_g->G_SWIPE_LEN[0],
		grip_status_g->G_SWIPE_LEN[1]);
	return 0;
}

ssize_t Grip_Swipe_len_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SWIPE_LEN) > 0)
		grip_swipeX_len_func(parse_result[1], parse_result[0], &SWIPE_BIT3[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Swipe_len_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Swipe_len_proc_read, NULL);
}

void create_Grip_Swipe_len_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Swipe_len_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Swipe_len_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Swipe_len_proc_open,
		.write = Grip_Swipe_len_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_swipe_len", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Swipe_min_pos_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n",
		grip_status_g->G_SWIPE_MIN_POS[0],
		grip_status_g->G_SWIPE_MIN_POS[1]);
	return 0;
}

ssize_t Grip_Swipe_min_pos_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SWIPE_MIN_POS) > 0)
		grip_swipeX_min_position_func(parse_result[1], parse_result[0], &SWIPE_BIT1[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Swipe_min_pos_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Swipe_min_pos_proc_read, NULL);
}

void create_Grip_Swipe_min_pos_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Swipe_min_pos_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Swipe_min_pos_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Swipe_min_pos_proc_open,
		.write = Grip_Swipe_min_pos_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_swipe_min_pos", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Swipe_max_pos_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n",
		grip_status_g->G_SWIPE_MAX_POS[0],
		grip_status_g->G_SWIPE_MAX_POS[1]);
	return 0;
}

ssize_t Grip_Swipe_max_pos_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SWIPE_MAX_POS) > 0)
		grip_swipeX_max_position_func(parse_result[1], parse_result[0], &SWIPE_BIT2[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Swipe_max_pos_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Swipe_max_pos_proc_read, NULL);
}

void create_Grip_Swipe_max_pos_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Swipe_max_pos_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_Swipe_max_pos_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Swipe_max_pos_proc_open,
		.write = Grip_Swipe_max_pos_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file = proc_create("driver/grip_swipe_max_pos", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
//============================================================
//=======================SWIPE Part Done!!!==========================
//============================================================

//==========Gesture Thershold Interface=======//
int fw_version = 0;
static char *product_string;
static int Grip_FW_RESULT_proc_read(struct seq_file *buf, void *v)
{
	if(snt8100fsr_g->grip_fw_loading_status == true){
		seq_printf(buf, "0xffff\n");
	}else{
		seq_printf(buf, "0x0\n");
	}
	return 0;
}

static ssize_t Grip_FW_RESULT_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	int val;
	char messages[256];

	memset(messages, 0, sizeof(messages));
	if (len > 256) {
		len = 256;
	}
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	val = (int)simple_strtol(messages, NULL, 10);
	return len;
}

static int Grip_FW_RESULT_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_FW_RESULT_proc_read, NULL);
}

void create_Grip_FW_RESULT_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_FW_RESULT_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_FW_RESULT_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_FW_RESULT_proc_open,
		.write = Grip_FW_RESULT_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file =
	proc_create("driver/grip_fw_result", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
static int Grip_FW_VER_proc_read(struct seq_file *buf, void *v)
{
	int ret=0;
	if(snt8100fsr_g->grip_fw_loading_status == false){
		seq_printf(buf, "0x0\n");
		return -1;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	if(snt8100fsr_g->grip_fw_loading_status == true){
		PRINT_INFO("send firmware pass infor by special event format");
		grip_input_event_report(-1, -1, -1, -1, -1, -1, -1);
		grip_input_event_report(8888, 0, 0, 0, 0, 0, 0);
		Wait_Wake_For_RegW();
		//Grip_Chip_IRQ_EN(1);

		product_string = memory_allocate(PRODUCT_CONFIG_MAX_LEN, 0);
		if (product_string == NULL) {
			PRINT_CRIT("memory_allocate(PRODUCT_CONFIG_MAX_LEN) failed");
			mutex_unlock(&snt8100fsr_g->ap_lock);
			return -1;
		}

		ret = read_product_config(snt8100fsr_g, product_string);
		seq_printf(buf, "%s\n",product_string);
		memory_free(product_string);
		if (ret) {
			PRINT_WARN("Unable to read product config");
		}
		Into_DeepSleep_fun();
	}else{
		seq_printf(buf, "0x0\n");
	}
	mutex_unlock(&snt8100fsr_g->ap_lock);
	return 0;
}

static ssize_t Grip_FW_VER_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	int val;
	char messages[256];

	memset(messages, 0, sizeof(messages));
	if (len > 256) {
		len = 256;
	}
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	val = (int)simple_strtol(messages, NULL, 10);
	fw_version = val;
	return len;
}

static int Grip_FW_VER_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_FW_VER_proc_read, NULL);
}

void create_Grip_FW_VER_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_FW_VER_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_FW_VER_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_FW_VER_proc_open,
		.write = Grip_FW_VER_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file =
	proc_create("driver/grip_fw_ver", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

static int Grip_set_power_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "Grip 1V2_2V8 status: %d\n", g_snt_power_state);
	return 0;
}

static ssize_t Grip_set_power_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	int val;
	char messages[256];

	memset(messages, 0, sizeof(messages));
	if (len > 256) {
		len = 256;
	}
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	val = (int)simple_strtol(messages, NULL, 10);
	if(val == 1){
		snt8100fsr_g->mgrip_2v8_asus_func->grip_regulator_disable();
	}else{
		snt8100fsr_g->mgrip_2v8_asus_func->grip_regulator_enable();

	}
	PRINT_INFO("set power = %d", val);
	return len;
}

static int Grip_set_power_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_set_power_proc_read, NULL);
}

void create_Grip_set_power_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_set_power_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_write	= Grip_set_power_proc_write,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_set_power_proc_open,
		.write = Grip_set_power_proc_write,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file =
	proc_create("driver/grip_set_power", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

static void Grip_Read_Calibration_Data(struct seq_file *buf, int sleep_time){
	uint16_t cfg_bank_sync1[3] = {0x0000,0x0004,0x0e02};
	uint16_t cfg_bank_dataA[2] = {0x011a,0x0000};
	uint16_t cfg_bank_dataB[2] = {0x011b,0x0000};
	uint16_t cfg_bank_dataC[2] = {0x011c,0x0000};
	uint16_t cfg_bank_sync2[3] = {0x0000,0x0000,0x0e04};
	uint16_t cfg_bank_read[3] = {0x0000,0x0000,0x0e01};
	int block_len = 3;
	int partition_len = 2;
	int read_dataA_len = 16, read_dataB_len = 9, read_dataC_len = 9;
	int i = 0;
	uint16_t string[20];
	uint16_t string1[20];
	uint16_t string2[20];
	memset(string, 0, sizeof(string));
	cust_write_registers(snt8100fsr_g,0x2c, block_len, cfg_bank_sync1);
	msleep(sleep_time);
	cust_write_registers(snt8100fsr_g,0x200, partition_len, cfg_bank_dataA);
	msleep(sleep_time);
	cust_write_registers(snt8100fsr_g,0x2c, block_len, cfg_bank_sync2);
	msleep(sleep_time);

	cust_write_registers(snt8100fsr_g,0x2c, block_len, cfg_bank_read);
	msleep(sleep_time);
	cust_read_registers(snt8100fsr_g,0x200, 1, string);
	msleep(sleep_time);
	cust_read_registers(snt8100fsr_g,0x200, read_dataA_len, string);
	msleep(sleep_time);

	for(i = 0; i < read_dataA_len; i++){
		seq_printf(buf, "0x%x,",string[i]);
		if(i == (read_dataA_len/2)){
			seq_printf(buf, "&");
		}
	}
	seq_printf(buf, "&");
	memset(string1, 0, sizeof(string1));
	cust_write_registers(snt8100fsr_g,0x2c, block_len, cfg_bank_sync1);
	msleep(sleep_time);
	cust_write_registers(snt8100fsr_g,0x200, partition_len, cfg_bank_dataC);
	msleep(sleep_time);
	cust_write_registers(snt8100fsr_g,0x2c, block_len, cfg_bank_sync2);
	msleep(sleep_time);

	cust_write_registers(snt8100fsr_g,0x2c, block_len, cfg_bank_read);
	msleep(sleep_time);
	cust_read_registers(snt8100fsr_g,0x200, 1, string1);
	msleep(sleep_time);
	cust_read_registers(snt8100fsr_g,0x200, read_dataC_len, string1);
	msleep(sleep_time);

	for(i = 0; i < read_dataB_len; i++){
		seq_printf(buf, "0x%x,",string1[i]);
	}
	seq_printf(buf, "&");

	memset(string2, 0, sizeof(string2));
	cust_write_registers(snt8100fsr_g,0x2c, block_len, cfg_bank_sync1);
	msleep(sleep_time);
	cust_write_registers(snt8100fsr_g,0x200, partition_len, cfg_bank_dataB);
	msleep(sleep_time);
	cust_write_registers(snt8100fsr_g,0x2c, block_len, cfg_bank_sync2);
	msleep(sleep_time);

	cust_write_registers(snt8100fsr_g,0x2c, block_len, cfg_bank_read);
	msleep(sleep_time);
	cust_read_registers(snt8100fsr_g,0x200, 1, string2);
	msleep(sleep_time);
	cust_read_registers(snt8100fsr_g,0x200, read_dataB_len, string2);
	msleep(sleep_time);

	for(i = 0; i < read_dataC_len; i++){
		seq_printf(buf, "0x%x,",string2[i]);
	}
	PRINT_INFO("%s", buf->buf);
}

static int Grip_Cal_Read_proc_read(struct seq_file *buf, void *v)
{
	if(snt8100fsr_g->grip_fw_loading_status == false){
		seq_printf(buf, "0x0\n");
		return -1;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	if(snt8100fsr_g->grip_fw_loading_status == true){
		Wait_Wake_For_RegW();
		Grip_Read_Calibration_Data(buf,1);
		Into_DeepSleep_fun();
	}else{
		seq_printf(buf, "fw_loading fail\n");
	}
	mutex_unlock(&snt8100fsr_g->ap_lock);
	return 0;
}

static int Grip_Cal_Read_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Cal_Read_proc_read, NULL);
}

void create_Grip_Cal_Read_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_Cal_Read_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Cal_Read_proc_open,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file =
	proc_create("driver/grip_cal_read", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

static int Grip_FW_Fail_Count_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "fw failed count=%d\n", snt8100fsr_g->fw_failed_count);
	return 0;
}

static int Grip_FW_Fail_Count_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_FW_Fail_Count_proc_read, NULL);
}

void create_Grip_FW_Fail_Count_proc_file(void)
{
	static const struct proc_ops proc_fops = {
		.proc_open	= Grip_FW_Fail_Count_proc_open,
		.proc_read	= seq_read,
		.proc_lseek	= seq_lseek,
		.proc_release	= single_release,
	};
	/*
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_FW_Fail_Count_proc_open,
		.read = seq_read,
		.release = single_release,
	};*/
	struct proc_dir_entry *proc_file =
	proc_create("driver/grip_fw_fail_count", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
/*************** ASUS BSP Clay: proc file --- *******************/
