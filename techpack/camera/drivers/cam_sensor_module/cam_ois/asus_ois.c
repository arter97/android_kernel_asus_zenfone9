#include <linux/proc_fs.h>
#include <linux/time.h>
#include <linux/fs.h>
#include "asus_ois.h"
//#include <linux/miscdevice.h>

//////////#include "onsemi_interface.h"
//#include "onsemi_i2c.h"
//////////#include "utils.h"
#include "cam_eeprom_dev.h"
//#include "asus_cam_sensor.h"
#include <linux/delay.h>
//////#include "asus_actuator.h"
#undef  pr_fmt
#define pr_fmt(fmt) "OIS-ATD %s(): " fmt, __func__

//OIS PROC DRIVER NODE +++
//ASUS_BSP ASUS Factory use +++
#define	PROC_POWER	"driver/ois_power"
#define	PROC_I2C_RW	"driver/ois_i2c_rw"
#define	PROC_MODE	"driver/ois_mode"
#define	PROC_CALI	"driver/ois_cali"
#define PROC_RDATA  "driver/ois_rdata"
#define	PROC_ATD_STATUS	"driver/ois_atd_status"
#define PROC_VCM_ENABLE "driver/ois_vcm_enable" //for debug
#define	PROC_SMA_EEPROM_DUMP	"driver/ois_sma_eeprom_dump"
//ASUS_BSP ASUS Factory use ---

#define PROC_ON     "driver/ois_on"
#define PROC_STATE "driver/ois_state"
#define	PROC_PROBE_STATUS "driver/ois_status"
#define	PROC_FW_STATUS "driver/ois_fw_status"
#define	PROC_DEVICE	"driver/ois_device"
#define PROC_FW_UPDATE "driver/ois_fw_update"
#define	PROC_MODULE	"driver/ois_module" //ASUS_BSP Lucien +++: Add OIS SEMCO module
#define PROC_AF_ATATE  "driver/ois_af_state" //notify AF state
#define PROC_OIS_GET_LENS_INFO "driver/ois_i2c_rw_lens_info"
//OIS PROC DRIVER NODE ---
#define RDATA_OUTPUT_FILE "/sdcard/gyro.csv"
#define SMA_OFFSET_DUMP "/sdcard/sma_eeprom_dump"

#define FACTORYDIR "/vendor/factory/"

#define OIS_GYRO_K_OUTPUT_FILE_OLD ""FACTORYDIR"OIS_calibration_old"
#define OIS_GYRO_K_OUTPUT_FILE_NEW ""FACTORYDIR"OIS_calibration"

#define OIS_VCM_BACKUP ""FACTORYDIR"OIS_VCM_VERSION"
#define OIS_MODULE_SN  ""FACTORYDIR"OIS_MODULE_SN"
#define OIS_FW_UPDATE_TIMES ""FACTORYDIR"OIS_FW_UPDATE_TIMES"
#define OIS_FW_UPDATE_TIME_LIMIT 500

#define BATTERY_CAPACITY "/sys/class/power_supply/bms/capacity"
#define BATTERY_THRESHOLD 3


#define OIS_I2C_MODE 2
#define Wait(A) msleep(A)
//#define DEBUG_S4W_OIS


int32_t IIC_DataRead(uint32_t addr, int size, uint8_t *data);
int32_t IIC_DataSend(unsigned short addr, uint32_t size, unsigned char* SendData);


//#ifdef ASUS_SAKE_PROJECT
#define UPDATE_FW_AT_PROBE 0
//#else
//#define UPDATE_FW_AT_PROBE 1
//#endif
typedef enum {
	CHECK_BAD_IO = -1,
	CHECK_PASS = 0,
	CHECK_BAD_ID,
	CHECK_BAD_FW,
	CHECK_BAD_FUNCTION,
	CHECK_VENDOR_MISMATCH,
	CHECK_VENDOR_INVALID,
	CHECK_CUSTOMER_INVALID,
	CHECK_VCM_INVALID,
	CHECK_FW_VERSION_INVALID,
}chip_check_result_t;

typedef enum{
	UPDATE_FAIL = -1,
	UPDATE_OK = 0,
	NO_NEED = 1,
	BATTERY_LOW,
	NO_MATCH_FW,
	VENDOR_INVALID,
	NO_VCM_BACK,
	EXCEED_LIMIT,
	BAD_IO,
	BAD_FW,
}fw_trigger_result_t;
#ifdef ZS670KS
extern uint8_t eeprom_camera_specs; //ASUS_BSP Byron take this tag for distinguish device level
#endif
static struct cam_ois_ctrl_t * ois_ctrl[OIS_CLIENT_MAX] = {0};

uint8_t g_ois_status[OIS_CLIENT_MAX] = {0};
char g_module_vendor[8] = "UNKNOWN";
uint32_t g_fw_version = 0;
uint8_t g_ois_mode = 255;//only after set mode, mode value is valid

uint8_t g_ois_power_state[OIS_CLIENT_MAX] = {0};
uint8_t g_ois_camera_open[OIS_CLIENT_MAX] = {0};

static uint8_t g_atd_status = 0;//fail

static uint16_t g_reg_addr = 0xF012;
//static uint32_t g_reg_val = 0;
//static uint16_t g_slave_id = 0x0024;
static enum camera_sensor_i2c_type g_data_size = 1; //dword
static uint8_t g_write_operation = 0;//read

static char ois_subdev_string[32] = "";

////////static stReCalib g_calInfo;

//static struct mutex g_busy_job_mutex;

static uint32_t g_vendor_id = 0;
static uint32_t g_module_sn = 0;
static struct cam_eeprom_ctrl_t * g_ectrl = NULL;

static uint8_t g_vcm_enabled = 1;

static uint32_t g_dac_macro[OIS_CLIENT_MAX] = {0};
static uint32_t g_dac_infinity[OIS_CLIENT_MAX] = {0};
static struct timespec64 g_ssc_config_time_prev;
static uint32_t g_dac_macro_dit[OIS_CLIENT_MAX] = {0};
static uint32_t g_dac_infinity_dit[OIS_CLIENT_MAX] = {0};
static uint32_t g_dac_macro_base[OIS_CLIENT_MAX] = {0};
static uint32_t g_dac_infinity_base[OIS_CLIENT_MAX] = {0};
//static uint32_t g_lens_shift_10cm_to_50cm = 0;
//static uint32_t g_lens_shift_10cm = 0;
static uint8_t  g_verbose_log = 0;
static uint32_t g_lens_position_reg[2] = {0};
#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT || defined ASUS_SAKE_PROJECT || defined ASUS_VODKA_PROJECT
static void dump_sma_eeprom(struct cam_ois_ctrl_t *oisCtrl,uint32_t addr,uint8_t length);
#endif
#if defined ASUS_AI2202_PROJECT
static void set_ssc_gain_if_need(uint32_t ois_index,uint32_t val,uint32_t distance_cm);
#endif
#if 0
static void onsemi_read_check(struct cam_ois_ctrl_t *o_ctrl)
{
	char buf[512];

	onsemi_dump_state(o_ctrl,buf,sizeof(buf));
	pr_info("dump state is\n%s\n",buf);

	onsemi_ois_go_on(o_ctrl);
	onsemi_ssc_go_on(o_ctrl);
	onsemi_dump_state(o_ctrl,buf,sizeof(buf));
	pr_info("dump state is\n%s\n",buf);

	onsemi_check_sequence_read(o_ctrl);
}
#endif
#if 0
static int read_vendor_id_from_eeprom(uint32_t * vendor_id)
{
	int rc;
	uint32_t reg_addr = 0x09;

	if(g_ectrl == NULL)
	{
		pr_err("eeprom ctrl is NULL!\n");
		return -1;
	}
	camera_io_init(&(g_ectrl->io_master_info));
	rc = camera_io_dev_read(&(g_ectrl->io_master_info),
	                          reg_addr,
	                          vendor_id,
	                          CAMERA_SENSOR_I2C_TYPE_WORD,//addr_type
	                          CAMERA_SENSOR_I2C_TYPE_BYTE);//data_type
	camera_io_release(&(g_ectrl->io_master_info));
	if(rc < 0)
	{
		pr_err("EEPROM read reg 0x%x failed! rc = %d\n",reg_addr,rc);
		return -2;
	}
	else
	{
		pr_info("EEPROM read reg 0x%x get val 0x%x\n",reg_addr,*vendor_id);
		if(*vendor_id == 0x01)
			*vendor_id = VENDOR_ID_LITEON;
		else if(*vendor_id == 0x06)
			*vendor_id = VENDOR_ID_PRIMAX;
	}
	return 0;
}

static int read_module_sn_from_eeprom(uint32_t * module_sn)
{
	int rc;
	uint32_t reg_addr = 0x0E;
	uint8_t  sn[4];

	if(g_ectrl == NULL)
	{
		pr_err("eeprom ctrl is NULL!\n");
		return -1;
	}
	camera_io_init(&(g_ectrl->io_master_info));
	rc = camera_io_dev_read_seq(&(g_ectrl->io_master_info),
	                              reg_addr,
	                              sn,
	                              CAMERA_SENSOR_I2C_TYPE_WORD,
	                              CAMERA_SENSOR_I2C_TYPE_BYTE,
	                              4);
	camera_io_release(&(g_ectrl->io_master_info));
	if(rc < 0)
	{
		pr_err("EEPROM read reg 0x%x failed! rc = %d\n",reg_addr,rc);
		return -2;
	}
	else
	{
		*module_sn = (sn[0]<<24 | sn[1]<<16 | sn[2]<<8 | sn[3]);
		pr_info("EEPROM seq read reg 0x%x get SN 0x%08x\n",reg_addr,*module_sn);
	}
	return 0;
}
#endif
int get_ois_status(uint32_t index) {
	if(index < OIS_CLIENT_MAX) return g_ois_status[index];
	else {
		pr_err("un-predict index(%u)\n",index);
		return -1;
	}
}
int get_ois_power_state(uint32_t index) {
	if(index < OIS_CLIENT_MAX) return g_ois_power_state[index];
	else {
		pr_err("un-predict index(%u)\n",index);
		return -1;
	}
}
static int32_t get_ois_ctrl(struct cam_ois_ctrl_t **o_ctrl) {
		if ( ois_ctrl[0]!=NULL) {

		*o_ctrl = ois_ctrl[0];
		return 0;
	}
	#if 0
	uint8_t count = 0;
	while(*o_ctrl == NULL && count < 100) {
		if(ois_ctrl[OIS_CLIENT_IMX686]->ois_on == 1) {
			if(g_verbose_log)
				pr_info("Select cleint IMX686\n");
			*o_ctrl = ois_ctrl[OIS_CLIENT_IMX686];
			return OIS_CLIENT_IMX686;
		}else if(ois_ctrl[OIS_CLIENT_OV08A10]->ois_on == 1) {
			if(g_verbose_log)
				pr_info("Select cleint OV08A10\n");
			*o_ctrl = ois_ctrl[OIS_CLIENT_OV08A10];
			return OIS_CLIENT_OV08A10;
		}
		count++;
		msleep(20);
		pr_info("wait %u sec for ois ctrl ready \n",count*20);
	}
#endif


	pr_err("cannot find mapping client ois ctrl\n");
	*o_ctrl = NULL;
	return -1;
}
#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT || defined ASUS_SAKE_PROJECT || defined ASUS_VODKA_PROJECT
static fw_trigger_result_t trigger_fw_update(struct cam_ois_ctrl_t *ctrl, uint8_t update_mode, uint8_t force_update, uint32_t* updated_version)
{
	uint32_t fw_version;
	uint32_t actuator_version;

	uint8_t  module_vendor, vcm_version;
	uint8_t  backup_vcm = 0;
	uint32_t module_sn = 0;
	uint16_t fw_update_times = 0;

	uint8_t  battery_capacity;
	fw_trigger_result_t ret;

	int rc = 0;

	rc = onsemi_read_dword(ctrl,0x8000,&fw_version);
	if(rc < 0)
	{
		pr_err("read fw version failed!\n");
		return BAD_IO;
	}

	rc = onsemi_read_dword(ctrl,0x8004,&actuator_version);
	if(rc < 0)
	{
		pr_err("read actuator version failed!\n");
		return BAD_IO;
	}

	if(!ZF7_need_update_fw(fw_version,actuator_version,force_update))
	{
		return NO_NEED;//NO NEED UPDATE FW
	}

	if(sysfs_read_dword_seq(OIS_MODULE_SN,&module_sn,1) == 0 && g_module_sn == module_sn)
	{
		if(sysfs_read_word_seq(OIS_FW_UPDATE_TIMES,&fw_update_times,1) == 0 &&
			fw_update_times >= OIS_FW_UPDATE_TIME_LIMIT)
		{
			if(!force_update && fw_version != 0x0) //not force update nor bad FW
			{
				pr_err("fw has updated %d times, can not update anymore for safety\n",fw_update_times);
				return EXCEED_LIMIT;
			}
		}
	}
	else
	{
		fw_update_times = 0;
		if(fw_version == 0x0)
		{
			pr_err("Bad FW at First, not save it...\n");
			return BAD_FW;
		}
	}

	if(sysfs_read_uint8(BATTERY_CAPACITY,&battery_capacity) == 0)
	{
		pr_info("get battery capacity is %d%%\n",battery_capacity);
		if(battery_capacity<=BATTERY_THRESHOLD)
		{
			pr_err("battery is too low, not update FW\n");
			return BATTERY_LOW;
		}
	}

	if(fw_version == 0x0)
	{
		if(g_vendor_id == VENDOR_ID_LITEON || g_vendor_id == VENDOR_ID_PRIMAX)
		{
			pr_info("Saving failed module ... Vendor ID from EEPROM is 0x%x\n",g_vendor_id);
			module_vendor = g_vendor_id;
			//read backup vcm version
			if(sysfs_read_byte_seq(OIS_VCM_BACKUP,&backup_vcm,1) == 0)
			{
				pr_info("Got backup vcm %d from factory partition\n",backup_vcm);
				vcm_version = backup_vcm;
			}
			else
			{
				pr_err("Can not get backup vcm! Can not save failed module...\n");
				return NO_VCM_BACK;
			}
		}
		else
		{
			pr_err("Vendor ID 0x%x from EEPROM invalid, Can not save failed module...\n",g_vendor_id);
			return VENDOR_INVALID;
		}
	}
	else
	{
		module_vendor = fw_version >> 24;
		vcm_version = (actuator_version & 0x0000FF00)>>8;
	}

	if(update_mode == 1)
	{
		pr_info("warning: mode is 1, force change to 0\n");//0 don't erase user reserve area
		update_mode = 0;
	}

	if(fw_update_times == 0)//backup module sn & vcm for the first time
	{
		pr_info("Update FW FIRST time in this module!\n");
		if(sysfs_write_dword_seq_change_line(OIS_MODULE_SN,&g_module_sn,1,1,0) == 0)
		{
			pr_info("back up module SN 0x%08x done!\n",g_module_sn);
			if(sysfs_write_byte_seq(OIS_VCM_BACKUP,&vcm_version,1) == 0)
				pr_info("back up vcm version 0x%x done!\n",vcm_version);
		}
	}

	rc = ZF7_update_fw(ctrl, update_mode, module_vendor, vcm_version, updated_version);
	if(rc != 0xF0)//execute updating process
	{
		fw_update_times++;
		if(sysfs_write_word_seq(OIS_FW_UPDATE_TIMES,&fw_update_times,1) == 0)
		{
			pr_info("Save FW update times %d done\n",fw_update_times);
		}

		if(rc == 0)
		{
			ret = UPDATE_OK;
		}
		else
		{
			pr_err("update FW failed!\n");
			ret = UPDATE_FAIL;
		}
		get_module_name_from_fw_id(g_fw_version,g_module_vendor);
	}
	else
	{
		pr_err("Can not find correct FW to update for module 0x%x, vcm 0x%x...\n",module_vendor,vcm_version);
		ret = NO_MATCH_FW;//Not find FW to update
	}
	return ret;
}
#endif

int cam_ois_write_byte(struct cam_ois_ctrl_t *ctrl, uint32_t reg_addr, uint32_t reg_data)
{
	int rc = 0;

	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array data;

	data.reg_addr = reg_addr;
	data.reg_data = reg_data;
	data.delay = 0;
	data.data_mask = 0;//how to use? not used so far

	write_setting.reg_setting = &data;
	write_setting.size = 1;
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.delay = 0;

	rc = camera_io_dev_write(&(ctrl->io_master_info),&write_setting);
	if(rc < 0)
	{
		pr_err("write 0x%x to reg 0x%x failed! rc = %d",
						data.reg_data,data.reg_addr,rc);
	}
	return rc;
}

void OIS_S4W_FW_Update(void);
unsigned char Current_OIS_Status=0;
unsigned char Target_OIS_Status=0;
static void XYCoeffecBoundaryLimit(uint32_t addr);
static chip_check_result_t check_chip_info(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc=0;
//	struct cam_ois_soc_private     *soc_private =
//		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
//	struct cam_ois_i2c_info_t      *i2c_info = &soc_private->i2c_info;

	uint32_t chip_id=0;
//	uint32_t fw_id=0;
//	uint32_t actuator_id=0;
//	uint8_t  vendor,customer,vcm;
//	uint32_t servo_state;
	chip_check_result_t result;
//	uint32_t reg_addr=0x00f8;
	int iTry=0;
	//unsigned char SendData[256]="";
	//check i2c
////////	rc = ZF7_IORead32A(o_ctrl,i2c_info->id_register,&chip_id);
	pr_err("Randy debug try probe... stop here");

		/*
		rc = camera_io_dev_read(&(o_ctrl->io_master_info),
				reg_addr,
				&chip_id,
				CAMERA_SENSOR_I2C_TYPE_WORD,//addr_type
				CAMERA_SENSOR_I2C_TYPE_DWORD);//data_type
		pr_err("Randy debug try probe1.. . read[0x%X]=0x%X", reg_addr,chip_id);

		msleep(1000);
			reg_addr=0xfc;

	rc = camera_io_dev_read(&(o_ctrl->io_master_info),
			reg_addr,
			&chip_id,
			CAMERA_SENSOR_I2C_TYPE_WORD,//addr_type
			CAMERA_SENSOR_I2C_TYPE_DWORD);//data_type
	pr_err("Randy debug try probe1.. . read[0x%X]=0x%X", reg_addr,chip_id);

	//	cam_ois_write_byte(o_ctrl, 0xF0F0, 0xFFFF);
	*/
	//	msleep(1000);


		/* Check RUMBA device */

#if 1
	iTry=0;
	for (iTry=0;iTry<5;iTry++) {
		msleep(10);
		rc=IIC_DataRead(0xFC, 4, (uint8_t *)&chip_id);
		pr_err("Randy debug try probe1..%d . version=%d", iTry, chip_id);
		if (chip_id!=0) break;
	}
#else
	chip_id=0;
		for (iTry=0;iTry<256;iTry++) {
				SendData[iTry]=(iTry+1)&0xff;
		}
		for (iTry=0;iTry<100;iTry++) {
			msleep(1000);
			IIC_DataSend(0x100, 256, SendData);
			msleep(3000);
			pr_err("Randy debug try send data done");
		}
#endif


	if(rc < 0)
	{
		pr_err("read chip id failed! rc = %d\n",rc);
		Current_OIS_Status=99;
		return CHECK_BAD_IO;
	}

#if 0
	if(chip_id != i2c_info->chip_id)
	{
		pr_err("read chip id 0x%x not expected\n",chip_id);
		return CHECK_BAD_ID;
	}

	g_fw_version = fw_id;
////////	get_module_name_from_fw_id(fw_id,g_module_vendor);//ATD

	vendor = fw_id >> 24;
	customer = (fw_id & 0x00ff0000) >> 16;
	vcm = (actuator_id & 0xff00) >> 8;

	pr_info("read fw id 0x%x, vendor 0x%x, customer 0x%x, vcm 0x%x\n",
			fw_id,vendor,customer,vcm);

	if(fw_id == 0x0)
	{
		pr_err("FW is bad!\n");
		result = CHECK_BAD_FW;
	}
#endif

Current_OIS_Status=0;

#define OIS_AUTO_FW_UPDATE

#ifdef OIS_AUTO_FW_UPDATE
	OIS_S4W_FW_Update();
#endif
    result = CHECK_PASS;

	if (Current_OIS_Status==0) {
		IIC_DataRead(0x027c, 4,  (uint8_t *)&chip_id);
		if ( chip_id<100000 || chip_id>999999) Current_OIS_Status=1;
		IIC_DataRead(0x0294, 4,  (uint8_t *)&chip_id);
		if ( chip_id<100000 || chip_id>999999) Current_OIS_Status=1;
	}

	XYCoeffecBoundaryLimit(0x27c);
	XYCoeffecBoundaryLimit(0x294);
	return result;
}
static int ois_power_up(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc=0;
	struct cam_hw_soc_info         *soc_info = &o_ctrl->soc_info;
	struct cam_ois_soc_private     *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;
	pr_err("Randy debug, try ois power up");
#if defined ASUS_AI2202_PROJECT
		power_info=NULL;
		soc_info=NULL;
		pr_err("Randy debug OIS for AI2202, power up is controlled by sensor");
#else
	rc = cam_sensor_core_power_up(power_info, soc_info);
	if (rc) {
		pr_err("ois power up failed, rc %d\n", rc);
		return -1;
	}
#endif
	if (o_ctrl->io_master_info.master_type == CCI_MASTER)
	{
		rc = camera_io_init(&(o_ctrl->io_master_info));
		if (rc < 0) {
			pr_err("cci init failed!\n");
#if defined ASUS_AI2202_PROJECT
					pr_err("Randy debug OIS for AI2202, power down is controlled by sensor");
#else
			rc = cam_sensor_util_power_down(power_info, soc_info);
			if (rc) {
				pr_err("ois power down failed, rc %d\n", rc);
			}
#endif
			return -2;
		}
	}
	return rc;
}

static int ois_power_down(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc;
	struct cam_hw_soc_info         *soc_info = &o_ctrl->soc_info;
	struct cam_ois_soc_private     *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;
	pr_err("Randy debug, try ois power down");

	if(o_ctrl->io_master_info.master_type == CCI_MASTER)
	{
		rc = camera_io_release(&(o_ctrl->io_master_info));
		if (rc < 0)
			pr_err("cci release failed!\n");
	}

#if defined ASUS_AI2202_PROJECT
			soc_info=NULL;
			power_info=NULL;
			pr_err("Randy debug OIS for AI2202, power is controlled by sensor");
			return 0;
#else
		rc = cam_sensor_util_power_down(power_info, soc_info);
		if (rc) {
			pr_err("ois power down failed, rc %d\n", rc);
		}
#endif
	return rc;
}

static int ois_probe_status_proc_read(struct seq_file *buf, void *v)
{
	uint32_t k;
	for(k=0;k<OIS_CLIENT_MAX;k++) {
		if(ois_ctrl[k] == NULL) {
			pr_err("ois_ctrl[%u] is null\n",k);
		}else {
//			mutex_lock(&ois_ctrl[k]->ois_mutex);
			seq_printf(buf, "%d\n", g_ois_status[k]);
//			mutex_unlock(&ois_ctrl[k]->ois_mutex);
		}
	}

	return 0;
}

#define OIS_PIC_X_ORG_DATA 254
#define OIS_PIC_Y_ORG_DATA 253
static uint32_t ois_pic_x_org_data = 0;
static uint32_t ois_pic_y_org_data = 0;

static int ois_fw_status_proc_read(struct seq_file *buf, void *v)
{

	if (Target_OIS_Status==OIS_PIC_X_ORG_DATA) {
		pr_info("[OIS][PIC] Get PIC org X=%d\n", ois_pic_x_org_data);
		seq_printf(buf, "%d\n", ois_pic_x_org_data);
		return 0;
	}

	if (Target_OIS_Status==OIS_PIC_Y_ORG_DATA) {
		pr_info("[OIS][PIC] Get PIC org Y=%d\n", ois_pic_y_org_data);
		seq_printf(buf, "%d\n", ois_pic_y_org_data);
		return 0;
	}

	pr_info("[OIS] FW read Current_OIS_Status=%d\n", Current_OIS_Status);
	seq_printf(buf, "%d\n", Current_OIS_Status);
	return 0;
}

static int ois_probe_status_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, ois_probe_status_proc_read, NULL);
}

static int ois_fw_status_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, ois_fw_status_proc_read, NULL);
}

static const struct proc_ops ois_probe_status_fops = {
//	//.proc_owner = THIS_MODULE,
	.proc_open = ois_probe_status_proc_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

//#define UPDATE_FW_BY_PROC_ONLY
uint8_t Loop_Gain_Cali(void);
static ssize_t ois_fw_status_write(struct file *dev, const char *buf, size_t len, loff_t *ppos)
{
	ssize_t ret_len;
	uint8_t ret=0;
	char messages[64]="";

	ret_len = len;
	if (len > 64) {
		len = 64;
	}
	if (copy_from_user(messages, buf, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages, "%d", &Target_OIS_Status);
	if (Target_OIS_Status==OIS_PIC_X_ORG_DATA) {
		pr_info("[OIS][PIC] Get PIC org X=%d\n", Current_OIS_Status);
	}

	if (Target_OIS_Status==OIS_PIC_Y_ORG_DATA) {
		pr_info("[OIS][PIC] Get PIC org Y=%d\n", Current_OIS_Status);
	}


	//Target_OIS_Status=1, update fw
	if (Target_OIS_Status==1) {
		pr_info("[OIS] FW do OIS_S4W_FW_Update(), Current_OIS_Status=%d\n", Current_OIS_Status);
		OIS_S4W_FW_Update();
	}
	//Target_OIS_Status=2, loop gain
	if (Target_OIS_Status==2) {
		ret=Loop_Gain_Cali();
		if (ret==0) {
			Current_OIS_Status=0;
			pr_info("[OIS] Loop_Gain_Cali OK, Current_OIS_Status=%d\n", Current_OIS_Status);
		 } else {
			Current_OIS_Status=2;
			pr_info("[OIS] Loop_Gain_Cali Fail, ret=%d\n", ret);

		}
	}

	if (Target_OIS_Status==3) {
		Current_OIS_Status=1;
		pr_info("[OIS] Enable ois status OK, Current_OIS_Status=%d\n", Current_OIS_Status);

	}

	pr_info("[OIS] FW write fw status=%d\n", Target_OIS_Status);
	return ret_len;
}

static const struct proc_ops ois_fw_status_fops = {
//	//.proc_owner = THIS_MODULE,
	.proc_open = ois_fw_status_proc_open,
	.proc_read = seq_read,
	.proc_write = ois_fw_status_write,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static int ois_atd_status_proc_read(struct seq_file *buf, void *v)
{

	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;

//	mutex_lock(&oisCtrl->ois_mutex);

	seq_printf(buf, "%d\n", g_atd_status);
	g_atd_status = 0;//default is failure

//	mutex_unlock(&oisCtrl->ois_mutex);
	return 0;
}

static int ois_atd_status_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, ois_atd_status_proc_read, NULL);
}

static ssize_t ois_atd_status_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t rc;
	char messages[16]="";
	uint32_t val;
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;
	rc = len;

	if (len > 16) {
		len = 16;
	}

	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}
	sscanf(messages,"%d",&val);
//	mutex_lock(&oisCtrl->ois_mutex);

	switch(val)
	{
		case 0:
			g_atd_status = 0;
			g_verbose_log = 0;
			break;
		case 1:
			g_atd_status = 1;
			break;
		default:
			g_atd_status = 1;
			g_verbose_log = 1;
	}
//	mutex_unlock(&oisCtrl->ois_mutex);

	pr_info("ATD status changed to %d\n",g_atd_status);

	return rc;
}

static const struct proc_ops ois_atd_status_fops = {
	//.proc_owner = THIS_MODULE,
	.proc_open = ois_atd_status_proc_open,
	.proc_read = seq_read,
	.proc_write = ois_atd_status_proc_write,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static uint8_t* rDataRawText=NULL;
static uint16_t rDataAllocSize=0;
static uint8_t*	rpDataRaw=NULL;
#define OneRawInBytes 60

static uint16_t oneMaxBlockSize=50*OneRawInBytes;
uint8_t* oneBlock=NULL;

static int ois_cali_proc_read(struct seq_file *buf, void *v)
{
////	uint32_t x,y;
	struct cam_ois_ctrl_t *oisCtrl = NULL;
	int32_t ois_index = get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;

//	mutex_lock(&oisCtrl->ois_mutex);

	if(g_ois_power_state[ois_index])
	{
		seq_printf(buf,"%s\n",rDataRawText);//ASCII
		vfree(rDataRawText);rDataRawText=rpDataRaw=NULL;rDataAllocSize=0;
		pr_err("rdata cali  free rdataRawText, sizeof(rDataRawText)=%d", sizeof(rDataRawText));
	}
	else
	{
		seq_printf(buf,"POWER DOWN\n");
	}

//	mutex_unlock(&oisCtrl->ois_mutex);

	return 0;
}

static int ois_cali_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, ois_cali_proc_read, NULL);
}
#if defined ASUS_AI2202_PROJECT
void update_OIS_gyro_gain_XY(uint32_t, uint32_t ois_gryo_cal_data_x, uint32_t ois_gryo_cal_data_y);

int64_t diff_time_us(struct timespec64 *t1, struct timespec64 *t2 )
{
	return ((((t1->tv_sec*1000000000)+t1->tv_nsec)-((t2->tv_sec*1000000000)+t2->tv_nsec))/1000);
}

#endif

#if defined ASUS_AI2202_PROJECT
static int asus_ois_read_fw(const char* fileName, char * buf, uint32_t * size);
#endif

uint8_t GyrosensorCalibration (void);
static ssize_t ois_cali_proc_write(struct file *dev, const char *buf, size_t count, loff_t *ppos)
{
	uint8_t rc=0;
	char messages[16]="";
	uint32_t val=0;
#if defined ASUS_AI2202_PROJECT
	struct timespec64 t1,t2;
	uint32_t param[2]={0,0};
	unsigned char SendData[80]="";
#endif
	#define GYRO_K_REG_COUNT 2
//	uint32_t gyro_data[GYRO_K_REG_COUNT];
//	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	pr_info("E\n");
//	if(oisCtrl == NULL) return -1;

	if (copy_from_user(messages, buf, count)) {
		pr_err("%s ois cali no parameter!\n", __func__);
//		return -EFAULT;
	 } else {

		sscanf(messages,"%d",&val);

#if defined ASUS_AI2202_PROJECT
		 if ( val==98 ) {
			 pr_info("[OIS] DataRead dump 0x110 +++\n");
			 ktime_get_boottime_ts64(&t1);
			 rc=IIC_DataRead(0x110, 80, SendData);
			 ktime_get_boottime_ts64(&t2);
			 pr_info("[OIS] DataRead dump 0x110 ---, cost %lld us\n", diff_time_us(&t2,&t1));
			 return count;
		 }

		 if ( val==99 ) {
			 sscanf(messages,"%d %x %x",&val, param[0], param[1]);
			 update_OIS_gyro_gain_XY(0, param[0], param[1]);
			 return count;
		 }
#endif

	 }






//	mutex_lock(&g_busy_job_mutex);

//	mutex_lock(&oisCtrl->ois_mutex);

	pr_info("[OIS] start gyro calibration\n");
////	rc = onsemi_gyro_calibration(oisCtrl); //TODO
	rc=GyrosensorCalibration();

	if(rc != 0)
	{
		pr_err("[OIS] gyro_calibration failed! rc = 0x%02x\n", rc);
		g_atd_status = 0;
	}
	else
	{
		pr_info("[OIS] gyro_calibration success!\n");
		g_atd_status = 1;
	}
    //ASUS_BSP Lucien +++: Save one Gyro data after doing OIS calibration
////	rc = onsemi_gyro_read_xy(oisCtrl, &gyro_data[0], &gyro_data[1]);
/*
	if(rc < 0) pr_err("onsemi_gyro_read_xy get fail! rc = %d\n", rc);
#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT || defined ASUS_SAKE_PROJECT || defined ASUS_VODKA_PROJECT
	rc = sysfs_write_dword_seq_change_line(OIS_GYRO_K_OUTPUT_FILE_NEW,gyro_data,GYRO_K_REG_COUNT,2,0);
#endif
	if(rc != 0) pr_err("sysfs_write_dword_seq_change_line fail! rc = %d\n", rc);
	*/
	//ASUS_BSP Lucien ---: Save one Gyro data after doing OIS calibration

//	mutex_unlock(&oisCtrl->ois_mutex);
//		seq_printf(buf, "%d\n", g_atd_status);

//	mutex_unlock(&g_busy_job_mutex);
	pr_info("X\n" );

	return count;
}

static const struct proc_ops ois_cali_fops = {
	//.proc_owner = THIS_MODULE,
	.proc_open = ois_cali_proc_open,
	.proc_read = seq_read,
	.proc_write = ois_cali_proc_write,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static int ois_mode_proc_read(struct seq_file *buf, void *v)
{
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;

//        mutex_lock(&oisCtrl->ois_mutex);
	seq_printf(buf, "%d\n", g_ois_mode);
//	mutex_unlock(&oisCtrl->ois_mutex);
	return 0;
}

static int ois_mode_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, ois_mode_proc_read, NULL);
}


void s4sw_set_mode(uint8_t mode) {
uint8_t ois_modes[] = { 0x5, 0x11, 0x10 };
uint8_t SendData=1;

	pr_err("[OIS] set s4sw mode to 0x%x+++", ois_modes[mode]);
	IIC_DataSend(0x02, 1, &ois_modes[mode]);
	IIC_DataSend(0x00, 1, &SendData);
	pr_err("[OIS] set s4sw mode to 0x%x---", ois_modes[mode]);
	/* OISMODE = OIS Mode 0 (Still Mode) set */
	/* OISMODE register (0x0002) 1Byte Send */
	/* OISCTRL OISEN set */
	/* OISCTRL register (0x0000) 1Byte Send */
}



static ssize_t ois_mode_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t ret_len;
	char messages[16]="";
	uint32_t val;
	int rc=0;
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;

	ret_len = len;
	if (len > 16) {
		len = 16;
	}

	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages,"%d",&val);

//	mutex_lock(&oisCtrl->ois_mutex);

	pr_info("start change ois mode\n");

////	rc = onsemi_switch_mode(oisCtrl,val);

	if ( val==99 ) {
		OIS_S4W_FW_Update();
		return ret_len;
	}


	if(rc == 0)
	{
		g_atd_status = 1;
		g_ois_mode = val;
		pr_info("OIS mode changed to %d by ATD\n",g_ois_mode);

	s4sw_set_mode(g_ois_mode);




	}
	else
	{
		g_atd_status = 0;
		pr_err("switch to mode %d failed! rc = %d\n",val,rc);
	}

//	mutex_unlock(&oisCtrl->ois_mutex);

	return ret_len;
}

static const struct proc_ops ois_mode_fops = {
	//.proc_owner = THIS_MODULE,
	.proc_open = ois_mode_proc_open,
	.proc_read = seq_read,
	.proc_write = ois_mode_proc_write,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};


static int ois_device_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%s\n",ois_subdev_string);
	return 0;
}

static int ois_device_open(struct inode *inode, struct file *file)
{
	return single_open(file, ois_device_read, NULL);
}

static ssize_t ois_device_write(struct file *dev, const char *buf, size_t len, loff_t *ppos)
{
	ssize_t ret_len;
	char messages[64]="";

	ret_len = len;
	if (len > 64) {
		len = 64;
	}
	if (copy_from_user(messages, buf, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages, "%s", ois_subdev_string);

	pr_info("write subdev=%s\n", ois_subdev_string);
	return ret_len;
}

static const struct proc_ops ois_device_proc_fops = {
	//.proc_owner		= THIS_MODULE,
	.proc_open		= ois_device_open,
	.proc_read		= seq_read,
	.proc_lseek		= seq_lseek,
	.proc_release	= single_release,
	.proc_write		= ois_device_write,
};

//ASUS_BSP Lucien +++: Add OIS SEMCO module
static int ois_module_read(struct seq_file *buf, void *v)
{
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;

//	mutex_lock(&oisCtrl->ois_mutex);
	seq_printf(buf, "%s\n",g_module_vendor);
//	mutex_unlock(&oisCtrl->ois_mutex);
	return 0;
}

static int ois_module_open(struct inode *inode, struct file *file)
{
	return single_open(file, ois_module_read, NULL);
}

static ssize_t ois_module_write(struct file *dev, const char *buf, size_t len, loff_t *ppos)
{
	return 0;
}

static const struct proc_ops ois_module_proc_fops = {
	//.proc_owner		= THIS_MODULE,
	.proc_open		= ois_module_open,
	.proc_read		= seq_read,
	.proc_lseek		= seq_lseek,
	.proc_release	= single_release,
	.proc_write		= ois_module_write,
};
//ASUS_BSP Lucien ---: Add OIS SEMCO module

static int ois_i2c_debug_read(struct seq_file *buf, void *v)
{
//	uint32_t reg_val=0;
	int rc=0;
	struct cam_ois_ctrl_t *oisCtrl = NULL;

#ifdef DEBUG_S4W_OIS
	char oneChar[100]="";
	char oneLine[100]="";
	uint8_t val[4]="", iLoop=0;
#endif

	uint32_t value;
	int32_t ois_index = get_ois_ctrl(&oisCtrl);

//	pr_err("[ois] %s ++", __func__);

	if(oisCtrl == NULL) return -1;

	if(g_ois_power_state[ois_index])
	{
		rc=IIC_DataRead(g_reg_addr, g_data_size, (char*)&value);
		if(rc == 0){
			g_atd_status = 1;
#ifdef DEBUG_S4W_OIS
			memcpy(val,(char*)&value, g_data_size);
			for (iLoop=0;iLoop<g_data_size;iLoop++) {
					sprintf(oneChar, " 0x%X", val[iLoop]);
					strcat(oneLine, oneChar);

				}
			pr_err("%s OK,read value=0x%X(%d) [%s]\n", __func__, value,value, oneLine );
#endif
			seq_printf(buf,"%x\n",value);
		}else{
			g_atd_status = 0;
			pr_err("[OIS] read from reg 0x%x failed! rc = %d\n",g_reg_addr,rc);
			seq_printf(buf,"		read from reg 0x%x failed! rc = %d\n",g_reg_addr,rc);
		}
	}else{
		seq_printf(buf,"POWER DOWN\n");
	}

//	pr_err("[ois] %s --", __func__);
	return 0;
}

static int ois_i2c_debug_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ois_i2c_debug_read, NULL);
}

static int ois_i2c_DIT_read(struct seq_file *buf, void *v)
{
	seq_printf(buf,"%x\n",0);
	return 0;
}

static int ois_i2c_DIT_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ois_i2c_DIT_read, NULL);
}

static ssize_t ois_i2c_DIT_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t ret_len;
	int n;
	char messages[100]="";
	uint16_t g_reg_addr = 0x0; //VCM read focus data addr
	enum camera_sensor_i2c_type g_data_size = 1; //dword
	uint32_t value;
	int rc=0;
	struct cam_ois_ctrl_t *oisCtrl = NULL;
	int32_t ois_index = get_ois_ctrl(&oisCtrl);

	if(oisCtrl == NULL) return -1;

	ret_len = len;
	if (len > 32) {
		len = 32;
	}
	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	n = sscanf(messages,"%x %x %x",&g_reg_addr,&g_data_size,&value);

	switch (n) {
		case 3:
			break;
		default:
			pr_err("%s command format fail !!n=%d\n", __func__, n);
			return -EFAULT;
	}

	if (g_data_size<1 || g_data_size>4) {
		pr_err("%s command data size fail !!g_data_size=%d\n", __func__, g_data_size);
		return -EFAULT;
	}

	if(g_ois_power_state[ois_index]) {

		rc=IIC_DataSend(g_reg_addr, g_data_size, (char*)&value);
		if(rc < 0){
			pr_err("[OIS] write %d bytes to reg 0x%04x FAIL\n",g_data_size,g_reg_addr);
		}else{
			pr_info("[OIS] write%d bytes to reg 0x%04x OK\n",g_data_size,g_reg_addr);
		}
	} else {
		pr_err("[OIS] POWER DOWN\n");
	}

	return ret_len;
}

static ssize_t ois_i2c_debug_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t ret_len;
	int n;
	char messages[100]="";
#ifdef DEBUG_S4W_OIS
	char oneChar[100]="";
	char oneLine[100]="";
	uint8_t val[4]="", iLoop=0;
#endif
	uint32_t value;
	int rc=0;
	struct cam_ois_ctrl_t *oisCtrl = NULL;
	int32_t ois_index = get_ois_ctrl(&oisCtrl);

//	pr_err("[ois] %s ++", __func__);
	if(oisCtrl == NULL) return -1;

	g_write_operation = 0;
	ret_len = len;
	if (len > 32) {
		len = 32;
	}
	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	n = sscanf(messages,"%x %x %x",&g_reg_addr,&g_data_size,&value);

	switch (n) {
		case 1:
			g_data_size = 1;//default byte
		case 2:
			value=0;
			break;
		case 3:
			g_write_operation = 1;
			break;
		default:
			pr_err("%s command format fail !!n=%d\n", __func__, n);
			return -EFAULT;
	}

	if (g_data_size<1 || g_data_size>4) {
		pr_err("%s command data size fail !!g_data_size=%d\n", __func__, g_data_size);
		return -EFAULT;
	}

	if(g_write_operation == 1) {
		if(g_ois_power_state[ois_index]) {

#ifdef DEBUG_S4W_OIS
			memcpy(val,(char*)&value, g_data_size);
			for (iLoop=0;iLoop<g_data_size;iLoop++) {
					sprintf(oneChar, " 0x%X", val[iLoop]);
					strcat(oneLine, oneChar);
			}
			pr_err("[OIS] write SLAVE 0x%X [0x%04x][%d]=0x%X(%d), %s\n",
			oisCtrl->io_master_info.cci_client->sid,
			g_reg_addr,g_data_size,value, value, oneLine);
#endif
			rc=IIC_DataSend(g_reg_addr, g_data_size, (char*)&value);
			if(rc < 0){
				pr_err("[OIS] write %d bytes to reg 0x%04x FAIL\n",g_data_size,g_reg_addr);
				g_atd_status = 0;
			}else{
				pr_info("[OIS] write%d bytes to reg 0x%04x OK\n",g_data_size,g_reg_addr);
				g_atd_status = 1;
			}
		} else {
			pr_err("[OIS] POWER DOWN\n");
		}
	}

//	pr_err("[ois] %s --", __func__);
	return ret_len;
}
static const struct proc_ops ois_i2c_debug_fops = {
	//.proc_owner = THIS_MODULE,
	.proc_open = ois_i2c_debug_open,
	.proc_read = seq_read,
	.proc_write = ois_i2c_debug_write,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static int ois_solo_power_read(struct seq_file *buf, void *v)
{
	struct cam_ois_ctrl_t *oisCtrl = NULL;
	int32_t ois_index = get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;

//        mutex_lock(&oisCtrl->ois_mutex);
        pr_info("g_ois_power_state = %d\n", g_ois_power_state[ois_index]);
	seq_printf(buf,"%d\n",g_ois_power_state[ois_index]);
//	mutex_unlock(&oisCtrl->ois_mutex);
	return 0;
}

static int ois_solo_power_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ois_solo_power_read, NULL);
}

//just for ATD test
static ssize_t ois_solo_power_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t ret_len;
	char messages[16]="";
	int val;
//	int rc;
//	int i;
	struct cam_ois_ctrl_t *oisCtrl = NULL;
	struct cam_hw_soc_info         *soc_info = NULL;
	struct cam_ois_soc_private     *soc_private = NULL;
	struct cam_sensor_power_ctrl_t *power_info = NULL;
	int32_t ois_index  = get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;

#if defined ASUS_AI2202_PROJECT
  pr_err("[OIS] for AI2202, ois power should be controlled by sensor");
//	return 0;
#endif

	soc_info = &oisCtrl->soc_info;
	soc_private = (struct cam_ois_soc_private *)oisCtrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	ret_len = len;
	if (len > 16) {
		len = 16;
	}
	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages,"%d",&val);

	pr_err("OIS get power value=%d\n",val);
//	mutex_lock(&oisCtrl->ois_mutex);
	pr_err("OIS get power value=%d after mutex \n",val);
	if(g_ois_camera_open[ois_index] == 0)
	{
		#if 1
		//ASUS_BSP Byron not support test ois on if camera close
		if(val == 0)
		{
			if(g_ois_power_state[ois_index] == 1)
			{
//				camera_io_release(&(oisCtrl->io_master_info));
//				rc = cam_sensor_util_power_down(power_info, soc_info);
				if(ois_power_down(oisCtrl) != 0)
				{
					pr_err("ois power up failed\n");
				}
				else
				{
					g_ois_power_state[ois_index] = 0;
					g_ois_mode = 255;
					pr_info("OIS POWER DOWN\n");
				}
			}
			else
			{
				pr_info("OIS already power off, do nothing\n");
			}
		}
		else
		{
			if(g_ois_power_state[ois_index] == 0)
			{
				if(ois_power_up(oisCtrl) != 0)
				{
					pr_err("ois power up failed\n");
				}
				else
				{
					g_ois_power_state[ois_index] = 1;
					g_ois_mode = 255;
					if(g_ois_status[ois_index] == 1)
					{
//						ZF7_WaitProcess(oisCtrl,0,__func__);
//						onsemi_config_ssc_gain(ois_ctrl,100); //TODO
					}
					else
					{
						pr_err("OIS probe failed, not config ssc gain\n");
					}
					pr_info("OIS POWER UP\n");
				}
			}
			else
			{
				pr_info("OIS already power up, do nothing\n");
			}
		}
		#endif
		return -1;
	}
	else
	{
		pr_err("camera has been opened, can't control ois power\n");
	}
//	mutex_unlock(&oisCtrl->ois_mutex);
	return ret_len;
}
static const struct proc_ops ois_solo_power_fops = {
	//.proc_owner = THIS_MODULE,
	.proc_open = ois_solo_power_open,
	.proc_read = seq_read,
	.proc_write = ois_solo_power_write,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static int ois_state_proc_read(struct seq_file *buf, void *v)
{
	char dump_buf[512];//must be large enough
#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT || defined ASUS_SAKE_PROJECT || defined ASUS_VODKA_PROJECT
	uint8_t battery_capacity;
#endif
	struct cam_ois_ctrl_t *oisCtrl = NULL;
	int32_t ois_index = get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;

//	mutex_lock(&oisCtrl->ois_mutex);
	if(g_ois_power_state[ois_index] == 1)
	{
//		ZF7_WaitProcess(oisCtrl,0,__func__);
//		onsemi_dump_state(oisCtrl,dump_buf,sizeof(dump_buf));
		seq_printf(buf, "%s\n", dump_buf);
#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT || defined ASUS_SAKE_PROJECT || defined ASUS_VODKA_PROJECT
		if(sysfs_read_uint8(BATTERY_CAPACITY,&battery_capacity) == 0)
		{
			pr_info("get battery capacity is %d%%\n",battery_capacity);
			seq_printf(buf, "battery: %d%%\n", battery_capacity);
		}
#endif
	}
	else
	{
		seq_printf(buf, "POWER DOWN\n");
	}
//	mutex_unlock(&oisCtrl->ois_mutex);
	return 0;
}

static int ois_state_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, ois_state_proc_read, NULL);
}

static const struct proc_ops ois_state_fops = {
	//.proc_owner = THIS_MODULE,
	.proc_open = ois_state_proc_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static int ois_on_proc_read(struct seq_file *buf, void *v)
{
	int state=0;
	struct cam_ois_ctrl_t *oisCtrl = NULL;
	int32_t ois_index = get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;

//	mutex_lock(&oisCtrl->ois_mutex);
	if(g_ois_power_state[ois_index])
	{
		seq_printf(buf, "%d\n", state);
	}
	else
		seq_printf(buf, "%s\n", "POWER DOWN\n");
//	mutex_unlock(&oisCtrl->ois_mutex);
	return 0;
}

static int ois_on_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ois_on_proc_read, NULL);
}

static ssize_t ois_on_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t ret_len;
	char messages[16]="";
	uint32_t val;
	int rc=0;
	struct cam_ois_ctrl_t *oisCtrl = NULL;
	int32_t ois_index = get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;
	ret_len = len;


	if (len > 16) {
		len = 16;
	}

	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages,"%d",&val);
	pr_info("OIS on mode %d\n",val);

//	mutex_lock(&oisCtrl->ois_mutex);
	if(g_ois_power_state[ois_index])
	{
		switch(val)
		{
			case 0:
//				rc = onsemi_ois_go_off(oisCtrl);
				break;

			case 1:
//				rc = onsemi_ois_go_on(oisCtrl);
				break;
			default:
				pr_err("Not supported command %d\n",val);
				rc = -1;
		}

		if(rc == 0)
		{
			g_atd_status = 1;
		}
		else
		{
			g_atd_status = 0;
			pr_err("OIS on/off failed! rc = %d\n",rc);
		}
	}
	else
	{
		pr_err("OIS POWER DOWN, power it up first!\n");
		g_atd_status = 0;
	}
//	mutex_unlock(&oisCtrl->ois_mutex);

	return ret_len;
}

static const struct proc_ops ois_on_fops = {
	//.proc_owner = THIS_MODULE,
	.proc_open = ois_on_proc_open,
	.proc_read = seq_read,
	.proc_write = ois_on_proc_write,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

int read_file_into_buffer(const char *filename, uint8_t* data, uint32_t size)
{
	return -1;

#if 0
    struct file *fp;
    mm_segment_t fs;
    loff_t pos;
	int rc;

    fp =filp_open(filename,O_RDONLY,S_IRWXU | S_IRWXG | S_IRWXO);
    if (IS_ERR(fp)){
		pr_err("open(%s) failed\n",filename);
        return -1;
    }

    fs =get_fs();
    set_fs(KERNEL_DS);

    pos =0;
    rc = kernel_read(fp,data, size, &pos);

    set_fs(fs);
    filp_close(fp,NULL);

    return rc;
#endif
}


//const static uint64_t rDataRawSize=10000;


int sysfs_write_word_seq_change_line(char *filename, uint16_t *value, uint32_t size,uint32_t column, char SEP)
{
	int i = 0;
	char buf[5];

	if (rDataRawText)
	{
		pr_err("[OIS] ATD rdata have data... memory!\n");
		vfree(rDataRawText);
	////			mutex_unlock(&oisCtrl->ois_mutex);
	}
	//size=600x12
	rDataAllocSize=sizeof(uint8_t)*size*5;
	rDataRawText = vmalloc(rDataAllocSize);
	if (!rDataRawText)
	{
		pr_err("[OIS] ATD rdata no memory!\n");
	////			mutex_unlock(&oisCtrl->ois_mutex);
		rpDataRaw=NULL;
		return 1;
	}
//	rDataRawText[size*OneRawInBytes]=0;
//    rDataRawText[size*5]=0;
	rpDataRaw=rDataRawText;
//	pr_err("rdata init rdataRawText, iDataRawTextCounter=%d", iDataRawTextCounter);

	for(i = 0; i < size; i++, rpDataRaw+=5){
		sprintf(buf, "%04x", value[i]);
		if((i + 1) % column == 0)
			buf[4] = '\n';
		else
			buf[4] =SEP;// ',';
		memcpy(rpDataRaw, buf, 5);

//		if (i==100 || i==300 || i==500) {
//			pr_err("[OIS] ATD rdata[%d](0x%X) copy data to rpDataRaw, buf[0][1][2][3][4]=%c%c%c%c  !\n", i, value[i], buf[0], buf[1], buf[2], buf[3], buf[4]);
//			pr_err("[OIS] ATD rdata[%d](0x%X) copy data to rpDataRaw, rpDataRaw[0][1][2][3][4]=%c%c%c%c  !\n", i, value[i], rpDataRaw[0], rpDataRaw[1], rpDataRaw[2], rpDataRaw[3], rpDataRaw[4]);
//		}

	}
	rpDataRaw=rDataRawText;
//	pr_err("[OIS] ATD rdata rpDataRaw(%p)[500][501][502][503][504]=%c%c%c%c%c !\n", rpDataRaw, rpDataRaw[2500], rpDataRaw[2501], rpDataRaw[2502], rpDataRaw[2503], rpDataRaw[2504]);

	return 0;
}


static int process_rdata(int count,int32_t ois_index)
{
#define PAIR 3
#define regSize (PAIR<<1)
#define oneRowSize (regSize<<1)
	int rc;
	int i;
//	uint32_t old_state;

	uint16_t *pData = NULL, *orgP=NULL;
	uint32_t reg_addr[regSize] = {  0x0082,0x0084,//gyro raw
							     0x05A0,0x05A2, //acc raw
							     0x008E,0x0090 //hall raw
						        };
	int j;
	uint8_t SendData=1;
//	uint16_t RevData=0;
	pData = vmalloc(sizeof(uint16_t)*count*oneRowSize);
	if (!pData)
	{
		pr_err("no memory!\n");
		return -1;
	}
	orgP=pData;

	IIC_DataSend(0x0000, 1, &SendData); /* FWUPCTRL REG(0x000C) 1Byte Send */
	IIC_DataSend(0x0080, 1, &SendData); /* FWUPCTRL REG(0x000C) 1Byte Send */
	msleep(50);

	pr_err("[OIS] Randy deubg time, countx=%d", 1);
	for(i=0;i<count;i++)	{
		for(j=0;j<regSize;j+=2, pData+=4)
		{
			struct timespec64 ns_time;
			uint64_t time = 0x0;
			//pData[0,12,24,36]=gyroX=reg_addr[0]
			//pData[j+0]=gyroX
			//pData[1,13,25,37]=gyroX=reg_addr[1]
			//pData[j+1]=gyroY
			IIC_DataRead(reg_addr[j], 2, (uint8_t*)&(pData[0]));
			IIC_DataRead(reg_addr[j+1], 2, (uint8_t*)&(pData[1]));

			ktime_get_real_ts64(&ns_time);
/*
			time = ((ns_time.tv_sec*1000000000)+ns_time.tv_nsec);
			//pData[j+2]=gyro TH
			pData[2] = (time & 0x0000FFFF00000000) >> 32;
			//pData[j+3]=gyro TL
			pData[3]  = (time & 0x00000000FFFF0000)>>16;
			//timestamp[3] = (time & 0x000000000000FFFF) << 16;
*/
			time = ((ns_time.tv_sec*1000000000)+ns_time.tv_nsec)/1000000;

			pData[2] = (time & 0x0000FFFF0000) >> 16;
			pData[3] = (time & 0x00000000FFFF) ;

			if(rc < 0)
			{
				pr_err("read %d times x,y failed! rc = %d\n",i,rc);
				rc = -2;
				break;
			}
		}
		if(rc<0)
		{
			break;
		}

	}
	pr_err("[OIS] Randy deubg time, countx=%d", count);
//	pr_info("read %d times x,y values, cost %lld ms, each cost %lld us\n",
//		i,
//		diff_time_us(&t2,&t1)/1000,
//		diff_time_us(&t2,&t1)/(i)
//	);

//	onsemi_restore_ois_state(ois_ctrl[ois_index],old_state);
	if(i == count)
	{
		//all pass, store to /sdcard/gyro.csv
		//remove file....do it in script
		if(sysfs_write_word_seq_change_line(RDATA_OUTPUT_FILE,orgP,count*oneRowSize,oneRowSize, ',') == 0)
		{
			pr_info("store gyro data to %s succeeded!\n",RDATA_OUTPUT_FILE);
			rc = 0;
		}
		else
		{
			pr_err("store gyro data to %s failed!\n",RDATA_OUTPUT_FILE);
			rc = -3;
		}
	}
	else
	{
		pr_err("read data failed, read %d data\n",i);
	}

	vfree(orgP);

	return rc;
}

static int ois_rdata_proc_read(struct seq_file *buf, void *v)
{
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;

	if (oneBlock==NULL) {
			oneBlock=vmalloc(oneMaxBlockSize+1);
	}
	pr_err("rdata rpDataRaw(%p), sizeof(rDataRawText)=%d", rpDataRaw, sizeof(rDataRawText));

	if (rpDataRaw != NULL && oneBlock!=NULL) {
		memcpy(oneBlock, rpDataRaw, oneMaxBlockSize);
		oneBlock[oneMaxBlockSize]=0;
		seq_printf(buf,"%s\n",oneBlock);//ASCII

		rpDataRaw+=oneMaxBlockSize; //one raw 3000bytes
//		pr_err("[OIS] ATD rdata rDataRawText=\n");
//		pr_err("[OIS] ATD rdata rDataRawText=%s\n", rDataRawText);
		if (rpDataRaw>=rDataRawText+rDataAllocSize) {
			vfree(rDataRawText);rDataRawText=rpDataRaw=NULL;rDataAllocSize=0;
			pr_err("rdata free rdataRawText, sizeof(rDataRawText)=%d", sizeof(rDataRawText));
		}
	}
	else
	{
		pr_err("[OIS] ATD rdata rDataRawText is NULL !\n", rDataRawText);
		seq_printf(buf,"file is empty!\n");
	}
//	mutex_unlock(&oisCtrl->ois_mutex);

	return 0;
}

static int ois_rdata_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ois_rdata_proc_read, NULL);
}

static ssize_t ois_rdata_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t ret_len;
	char messages[16]="";
	uint32_t val;
	int rc;
	struct cam_ois_ctrl_t *oisCtrl = NULL;
	int32_t ois_index = get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;

	ret_len = len;

	if (len > 16) {
		len = 16;
	}

	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages,"%d",&val);

//	mutex_lock(&g_busy_job_mutex);

//	mutex_lock(&oisCtrl->ois_mutex);

	if(g_ois_power_state[ois_index])
	{
		rc = process_rdata(val,ois_index); //ASUS_BSP Lucien +++: Save val numbers of Gyro X/Y and ACC X/Y data
		if(rc == 0)
		{
			g_atd_status = 1;
		}
		else
		{
			g_atd_status = 0;
			pr_err("OIS Rdata failed! rc = %d\n",rc);
		}
	}
	else
	{
		pr_err("OIS POWER DOWN, power it up first!\n");
		g_atd_status = 0;
	}

//	mutex_unlock(&oisCtrl->ois_mutex);

//	mutex_unlock(&g_busy_job_mutex);

	return ret_len;
}

static const struct proc_ops ois_rdata_fops = {
	//.proc_owner = THIS_MODULE,
	.proc_open = ois_rdata_proc_open,
	.proc_read = seq_read,
	.proc_write = ois_rdata_proc_write,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static int ois_update_fw_read(struct seq_file *buf, void *v)
{
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;

//        mutex_lock(&oisCtrl->ois_mutex);
	seq_printf(buf, "%x\n", g_fw_version);
//	mutex_unlock(&oisCtrl->ois_mutex);
	return 0;
}

static int ois_update_fw_open(struct inode *inode, struct file *file)
{
	return single_open(file, ois_update_fw_read, NULL);
}

static ssize_t ois_update_fw_write(struct file *dev, const char *buf, size_t len, loff_t *ppos)
{
	ssize_t ret_len;
	char messages[64]="";
	int n;
	uint32_t val[2];
	uint32_t force_update, update_mode;
	struct cam_ois_ctrl_t *oisCtrl = ois_ctrl[OIS_CLIENT_OV08A10];
	int32_t ois_index = OIS_CLIENT_OV08A10;
	if(oisCtrl == NULL) return -1;

	ret_len = len;

	if (len > 64) {
		len = 64;
	}
	if (copy_from_user(messages, buf, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	n = sscanf(messages, "%x %x", &val[0],&val[1]);
	if(n == 1)
	{
		force_update = val[0];
		update_mode = 0;
	}
	else if(n == 2)
	{
		force_update = val[0];
		update_mode = val[1];
	}
	else
	{
		pr_err("Invalid argument count %d!\n",n);
		return ret_len;
	}

	mutex_lock(&g_busy_job_mutex);

//	mutex_lock(&oisCtrl->ois_mutex);

	if(g_ois_power_state[ois_index] == 0)
	{
		pr_err("OIS POWER DOWN, power it up first!\n");
		goto END;
	}

	pr_info("trigger fw update, force update %d, update mode %d\n",force_update,update_mode);
#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT || defined ASUS_SAKE_PROJECT || defined ASUS_VODKA_PROJECT
	trigger_fw_update(oisCtrl, update_mode, force_update, &g_fw_version);
#endif

END:
//	mutex_unlock(&oisCtrl->ois_mutex);
	mutex_unlock(&g_busy_job_mutex);
	return ret_len;
}

static const struct proc_ops ois_update_fw_proc_fops = {
	//.proc_owner		= THIS_MODULE,
	.proc_open		= ois_update_fw_open,
	.proc_read		= seq_read,
	.proc_lseek		= seq_lseek,
	.proc_release	= single_release,
	.proc_write		= ois_update_fw_write,
};


static int ois_vcm_enable_read(struct seq_file *buf, void *v)
{
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;

//        mutex_lock(&oisCtrl->ois_mutex);
	seq_printf(buf, "%d\n", g_vcm_enabled);
//	mutex_unlock(&oisCtrl->ois_mutex);
	return 0;
}

static int ois_vcm_enable_open(struct inode *inode, struct file *file)
{
	return single_open(file, ois_vcm_enable_read, NULL);
}

static ssize_t ois_vcm_enable_write(struct file *dev, const char *buf, size_t len, loff_t *ppos)
{
	ssize_t ret_len;
	char messages[8]="";

	uint32_t val;
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;

	ret_len = len;
	if (len > 8) {
		len = 8;
	}
	if (copy_from_user(messages, buf, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages, "%d", &val);

//	mutex_lock(&oisCtrl->ois_mutex);

	if(val == 0)
		g_vcm_enabled = 0;
	else
		g_vcm_enabled = 1;

	pr_info("ois vcm enabled set to %d\n",g_vcm_enabled);

//	mutex_unlock(&oisCtrl->ois_mutex);

	return ret_len;
}

static const struct proc_ops ois_vcm_enable_proc_fops = {
	//.proc_owner		= THIS_MODULE,
	.proc_open		= ois_vcm_enable_open,
	.proc_read		= seq_read,
	.proc_lseek		= seq_lseek,
	.proc_release	= single_release,
	.proc_write		= ois_vcm_enable_write,
};
static int ois_sma_eeprom_dump_read(struct seq_file *buf, void *v)
{
	return 0;
}
static int ois_sma_eeprom_dump_open(struct inode *inode, struct file *file)
{
	return single_open(file, ois_sma_eeprom_dump_read, NULL);
}

static ssize_t ois_sma_eeprom_dump_write(struct file *dev, const char *buf, size_t len, loff_t *ppos)
{
#if 1
	ssize_t ret_len;
	char messages[8]="";

	uint32_t addr;
	uint8_t length;
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) {
		pr_err("no ois ctrl to use\n");
		return -1;
	}

	ret_len = len;
	if (len > 8) {
		len = 8;
	}
	if (copy_from_user(messages, buf, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages, "%x %u", &addr,&length);

	pr_info("check addr(0x%x),length(%u)\n",addr,length);
#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT || defined ASUS_SAKE_PROJECT || defined ASUS_VODKA_PROJECT
	dump_sma_eeprom(oisCtrl,addr,length);
#endif
	return ret_len;
	#endif
}

static const struct proc_ops ois_sma_eerpom_dump_proc_fops = {
	//.proc_owner		= THIS_MODULE,
	.proc_open		= ois_sma_eeprom_dump_open,
	.proc_read		= seq_read,
	.proc_lseek		= seq_lseek,
	.proc_release	= single_release,
	.proc_write		= ois_sma_eeprom_dump_write,
};

static int ois_af_state_read(struct seq_file *buf, void *v)
{
	struct cam_ois_ctrl_t *oisCtrl = NULL;
	int32_t ois_index __attribute__((unused))= get_ois_ctrl(&oisCtrl);
	int k;
	if(oisCtrl == NULL) return -1;

//	mutex_lock(&oisCtrl->ois_mutex);
	for(k=0;k<OIS_CLIENT_MAX;k++) {
		seq_printf(buf, "ois Index(%u) DAC_macro: %d, DAC_infinity: %d\n",k,
					     g_dac_macro[k], g_dac_infinity[k]);
		seq_printf(buf, "ois Index(%u) DAC_macro_DIT: %d, DAC_infinity_DIT: %d\n",k,
					     g_dac_macro_dit[k], g_dac_infinity_dit[k]);
	}
//	mutex_unlock(&oisCtrl->ois_mutex);
	return 0;
}

static int ois_af_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, ois_af_state_read, NULL);
}

static ssize_t ois_af_state_write(struct file *dev, const char *buf, size_t len, loff_t *ppos)
{
	ssize_t ret_len;
#if defined ASUS_AI2202_PROJECT
	char messages[100]="";
	uint8_t cmd_type;
	uint32_t af_state;
	uint32_t distance_mm;

	ret_len = len;
	if (len > 100) {
		len = 99;
	}
	if (copy_from_user(messages, buf, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}


	sscanf(messages, "%d %x %x", &cmd_type, &af_state,&distance_mm);
	if (cmd_type==0) {
		set_ssc_gain_if_need(0,af_state,distance_mm);
	}

	if (cmd_type==1) {
		update_OIS_gyro_gain_XY(0,af_state,distance_mm);
	}

//	mutex_unlock(&oisCtrl->ois_mutex);
#endif

	return ret_len;
}

static const struct proc_ops ois_af_state_proc_fops = {
	//.proc_owner		= THIS_MODULE,
	.proc_open		= ois_af_state_open,
	.proc_read		= seq_read,
	.proc_lseek		= seq_lseek,
	.proc_release	= single_release,
	.proc_write		= ois_af_state_write,
};


static int ois_get_lens_info_read(struct seq_file *buf, void *v)
{
//	uint32_t lens_position_val[2] = {0};
	int rc=0;
	struct timespec64 t1,t2, t3;
	unsigned char SendData[256]="";
#ifdef OIS_FRAME_SYNC_DEBUG_MODE
	uint16_t* ts1; uint16_t* ts2;
#endif
	uint64_t recordTimeStart, recordTimeEnd, recordTimeTotal = 0;
	//const uint64_t NanoSecondsPerSecond = 1000000000ULL;
	struct cam_ois_ctrl_t *oisCtrl = NULL;
	int32_t ois_index = get_ois_ctrl(&oisCtrl);
		uint16_t loop=0;
	if(oisCtrl == NULL) return -1;

//	mutex_lock(&oisCtrl->ois_mutex);

	if(g_ois_power_state[ois_index])
	{


		//pr_err("Byron check LensPostion value (0x%08x 0x%08x) (%llu)\n",lens_position_val[0],lens_position_val[1],recordTimeStamp);
//		pr_err("[OIS] DataRead dump 0x%X, size=%d +++\n", g_lens_position_reg[0], g_lens_position_reg[1]);
		ktime_get_boottime_ts64(&t1);
		recordTimeStart = (t1.tv_sec * 1000000000) + (t1.tv_nsec);
		rc=IIC_DataRead(g_lens_position_reg[0], g_lens_position_reg[1], SendData);
		ktime_get_boottime_ts64(&t2);
		recordTimeEnd = (t2.tv_sec * 1000000000) + (t2.tv_nsec);
//		pr_err("[OIS] DataRead dump 0x110 ---, I2C read cost %lld us\n", diff_time_us(&t2,&t1));
		while ( loop < g_lens_position_reg[1] ) {
			seq_printf(buf,"%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x \n",SendData[loop+0],SendData[loop+1],SendData[loop+2],SendData[loop+3],SendData[loop+4],SendData[loop+5],SendData[loop+6],SendData[loop+7],SendData[loop+8],SendData[loop+9],SendData[loop+10],SendData[loop+11],SendData[loop+12],SendData[loop+13],SendData[loop+14],SendData[loop+15]);
			loop+=16;
		}
#ifdef OIS_FRAME_SYNC_DEBUG_MODE
		for (loop=3, ts1=((uint16_t*) SendData)+loop, ts2=((uint16_t*) SendData+(loop<<1));loop<(g_lens_position_reg[1]>>1)-3;loop+=3, ts1+=3, ts2+=3) {
			if (*(ts1)+1 !=*ts2 ) {
					pr_err("[OIS] Randy debug ois_get_lens_info_read error: timestamp of SendData[%d]=%hu, SendData[%d]=%hu is not adjacent",loop, *ts1 , loop+3, *ts2 );
			}
			else {
//					if (loop==9)
//				pr_err("[OIS] Randy debug ois_get_lens_info_read debug: timestamp of SendData[%d]=%hu, SendData[%d]=%hu",loop, *ts1, loop+3, *ts2);


			}

		}
#endif
		seq_printf(buf,"%lx %lx %x \n",recordTimeStart, recordTimeEnd, diff_time_us(&t2,&t1));
		ktime_get_boottime_ts64(&t3);
		recordTimeTotal = (t3.tv_sec * 1000000000) + (t3.tv_nsec);
//		pr_err("[OIS] DataRead dump 0x110 ---, Total cost %lld us\n", diff_time_us(&t3,&t1));
	}
	else
	{
		seq_printf(buf,"POWER DOWN\n");
	}

//	mutex_unlock(&oisCtrl->ois_mutex);

	return 0;
}

static int ois_get_lens_info_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ois_get_lens_info_read, NULL);
}
static ssize_t ois_get_lens_info_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t ret_len;
	int n;
	char messages[32]="";
	uint32_t val[2];
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;

	ret_len = len;

	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	n = sscanf(messages,"%x %d",&val[0],&val[1]);

//	mutex_lock(&oisCtrl->ois_mutex);
	g_lens_position_reg[0] = val[0];
	g_lens_position_reg[1] = val[1];
	//pr_err("Byron check LensPostion (0x%04x,0x%04x)\n",g_lens_position_reg[0],g_lens_position_reg[1]);
//	mutex_unlock(&oisCtrl->ois_mutex);

	return ret_len;
}
static const struct proc_ops ois_get_lens_info_fops = {
	//.proc_owner = THIS_MODULE,
	.proc_open = ois_get_lens_info_open,
	.proc_read = seq_read,
	.proc_write = ois_get_lens_info_write,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static void create_proc_file(const char *PATH,const struct proc_ops* f_ops)
{
//dump function body, needs to implement

	struct proc_dir_entry *pde;

	pde = proc_create(PATH, 0666, NULL, f_ops);
	if(pde)
	{
		pr_info("create(%s) done\n",PATH);
	}
	else
	{
		pr_err("create(%s) failed!\n",PATH);
	}

}

static void create_ois_proc_files_shipping(void)
{
	static uint8_t has_created = 0;
	if(!has_created)
	{
		create_proc_file(PROC_DEVICE, &ois_device_proc_fops);
		create_proc_file(PROC_AF_ATATE, &ois_af_state_proc_fops);
		has_created = 1;
	}
	else
	{
		pr_err("OIS shipping proc files have already created!\n");
	}
}


static const struct proc_ops ois_i2c_DIT_fops = {
	//.proc_owner = THIS_MODULE,
	.proc_open = ois_i2c_DIT_open,
	.proc_read = seq_read,
	.proc_write = ois_i2c_DIT_write,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};


static void create_ois_proc_files_factory(void)
{
	static uint8_t has_created = 0;

	if(!has_created)
	{
		create_proc_file(PROC_POWER, &ois_solo_power_fops);
		create_proc_file(PROC_I2C_RW,&ois_i2c_debug_fops);//ATD
		create_proc_file(PROC_ON, &ois_on_fops);
		create_proc_file(PROC_STATE, &ois_state_fops);
		create_proc_file(PROC_MODE, &ois_mode_fops);//ATD
		create_proc_file(PROC_CALI, &ois_cali_fops);//ATD
		create_proc_file(PROC_RDATA, &ois_rdata_fops);//ATD
		create_proc_file(PROC_ATD_STATUS, &ois_atd_status_fops);//ATD
		create_proc_file(PROC_FW_STATUS, &ois_fw_status_fops);//For OIS fw update and loop gain
		create_proc_file(PROC_PROBE_STATUS, &ois_probe_status_fops);//ATD
		create_proc_file(PROC_FW_UPDATE, &ois_update_fw_proc_fops);
		create_proc_file(PROC_MODULE, &ois_module_proc_fops);//ATD ASUS_BSP Lucien +++: Add OIS SEMCO module
		create_proc_file(PROC_VCM_ENABLE, &ois_vcm_enable_proc_fops);
		create_proc_file(PROC_SMA_EEPROM_DUMP, &ois_sma_eerpom_dump_proc_fops);
		create_proc_file(PROC_OIS_GET_LENS_INFO,&ois_get_lens_info_fops);
		create_proc_file( "driver/ois_DIT_WRITE", &ois_i2c_DIT_fops);//For OIS fw update and loop gain
		has_created = 1;
	}
	else
	{
		pr_err("OIS factory proc files have already created!\n");
	}
}
static void check_chip_and_update_status(struct cam_ois_ctrl_t *o_ctrl,int32_t ois_index)
{
	chip_check_result_t check_result;
	check_result = check_chip_info(o_ctrl);
	switch(check_result)
	{
		case CHECK_BAD_IO:
			 g_ois_status[ois_index] = 0;
			 break;
		case CHECK_PASS:
			 g_ois_status[ois_index] = 1;
			 break;
		case CHECK_BAD_ID:
			 g_ois_status[ois_index] = 2;//ID not match
			 break;
		case CHECK_BAD_FW:
			 g_ois_status[ois_index] = 3;//FW bad
			 break;
		case CHECK_BAD_FUNCTION:
		     g_ois_status[ois_index] = 4;//Function bad
			 break;
		case CHECK_VENDOR_MISMATCH:
		     g_ois_status[ois_index] = 5;//vendor in eeprom mismatch fw
		     break;
		case CHECK_VENDOR_INVALID:
		case CHECK_CUSTOMER_INVALID:
		case CHECK_VCM_INVALID:
		case CHECK_FW_VERSION_INVALID:
			 g_ois_status[ois_index] = 6;//module invalid
			 break;
	}
}
void ois_probe_check(uint16_t sensor_id)
{
	//int rc;
#if UPDATE_FW_AT_PROBE
	fw_trigger_result_t trigger_result;
#endif
#ifdef ZS670KS
	const uint8_t device_high_level_check = 0x6b;
#endif
	struct cam_ois_ctrl_t *oisCtrl;
	int32_t ois_index=0;

	if (sensor_id>0) {
			pr_err("[OIS] Randy debug sensor_id=%d, skip ois probe", sensor_id );
			return;
	}


	oisCtrl=ois_ctrl[0];

	#ifdef ZS670KS
	pr_info("eeprom_camera_specs = 0x%x\n",eeprom_camera_specs);
	if(eeprom_camera_specs != device_high_level_check) {
		pr_info("with low level device, no ois to probe\n");
		return;
	}
	#endif
	if(oisCtrl == NULL)
	{
		#if defined(ASUS_DXO) ||defined(ZS670KS)
		pr_err("oisCtrl is NULL!!! in sensor_id(0x%x)\n",sensor_id);
		#else
		pr_info("oisCtrl is NULL!!! in sensor_id(0x%x)\n",sensor_id);
		#endif
		return;
	}
	if(ois_power_up(oisCtrl) != 0)
	{
		pr_err("ois power up failed\n");
		return;
	}
//	ZF7_WaitProcess(oisCtrl,0,__func__);//waiting after power on
#if 0
	rc = read_vendor_id_from_eeprom(&g_vendor_id);
	if(rc == 0)
	{
		pr_info("Got Vendor ID 0x%x from EEPROM Read\n",g_vendor_id);
	}
	else
		g_vendor_id = 0;

	rc = read_module_sn_from_eeprom(&g_module_sn);
	if(rc == 0)
	{
		pr_info("Got Module SN 0x%08x from EEPROM Read\n",g_module_sn);
	}
	else
		g_module_sn = 0;
#endif
	check_chip_and_update_status(oisCtrl,ois_index);
	if(g_ois_status[ois_index] == 1 || g_ois_status[ois_index] == 3)
	{
	#if UPDATE_FW_AT_PROBE
		pr_info("trigger fw update...ois status 0x%x\n",g_ois_status[ois_index]);
		trigger_result = trigger_fw_update(oisCtrl,0,0,&g_fw_version);
		if(trigger_result == UPDATE_OK || trigger_result == UPDATE_FAIL)//execute FW updating process
		{
			check_chip_and_update_status(oisCtrl,ois_index);
			pr_info("after FW update, ois status is 0x%x\n",g_ois_status[ois_index]);
		}
		else
		{
			pr_info("FW not update\n");
		}
	#endif
	}
	else
	{
		pr_err("check chip failed! ois status 0x%x\n",g_ois_status[ois_index]);
	}
	if(g_ois_status[ois_index] == 1)
		create_ois_proc_files_shipping();
	ois_power_down(oisCtrl);
}

uint8_t get_ois_probe_status(uint32_t index)
{
	#if 1
	//ASUS_BSP Byron no work
	struct cam_ois_ctrl_t *oisCtrl = ois_ctrl[OIS_CLIENT_OV08A10];
	if(oisCtrl == NULL)
	{
		pr_err("OIS not init!!!\n");
		return 0;
	}
	return g_ois_status[OIS_CLIENT_OV08A10];
	#endif
	return 0;
}

void ois_lock(void)
{
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL)
	{
		pr_err("OIS not init!!!\n");
		return;
	}
//	mutex_lock(&oisCtrl->ois_mutex);
}

void ois_unlock(void)
{
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL)
	{
		pr_err("OIS not init!!!\n");
		return;
	}
//	mutex_unlock(&oisCtrl->ois_mutex);
}

void ois_wait_process(void)
{
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL)
	{
		pr_err("OIS not init!!!\n");
		return;
	}
//	ZF7_WaitProcess(oisCtrl,0,__func__);
}

int ois_busy_job_trylock(void)
{
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL)
	{
		pr_err("OIS not init!!!\n");
		return 0;
	}
	return mutex_trylock(&g_busy_job_mutex);
}

void ois_busy_job_unlock(void)
{
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL)
	{
		pr_err("OIS not init!!!\n");
		return;
	}
	mutex_unlock(&g_busy_job_mutex);
}

void set_ois_module_vendor_from_eeprom(uint8_t vendor_id)
{
//	if(vendor_id == 0x01)
//		g_vendor_id = VENDOR_ID_LITEON;
//	else if(vendor_id == 0x06)
//		g_vendor_id = VENDOR_ID_PRIMAX;
	pr_info("Got Vendor ID 0x%x from EEPROM DUMP Info\n",g_vendor_id);
}

void set_ois_module_sn_from_eeprom(uint32_t sn)
{
	g_module_sn = sn;
	pr_info("Got Module SN 0x%08x from EEPROM DUMP Info\n",g_module_sn);
}

void set_rear_eeprom_ctrl(struct cam_eeprom_ctrl_t * e_ctrl)
{
	struct cam_sensor_cci_client *cci_client = NULL;

	if(e_ctrl != NULL)
	{
		if (e_ctrl->io_master_info.master_type == CCI_MASTER)
		{
			cci_client = e_ctrl->io_master_info.cci_client;
			if (!cci_client) {
				pr_err("failed: cci_client %pK",cci_client);
				return;
			}
			cci_client->cci_i2c_master = e_ctrl->cci_i2c_master;
			cci_client->sid = (0xA0) >> 1;
			cci_client->retries = 3;
			cci_client->id_map = 0;
			cci_client->i2c_freq_mode = OIS_I2C_MODE;

			g_ectrl = e_ctrl;
			pr_info("config ectrl done!\n");
		}
	}
	else
	{
		pr_err("e_ctrl is NULL!\n");
	}
}

uint8_t ois_allow_vcm_move(void)
{
	return g_vcm_enabled;
}

void set_ois_afc_data_from_eeprom(uint32_t dac_macro, uint32_t dac_infinity,char* physicalSnesorModuleName)
{
	int k;
	if(physicalSnesorModuleName == NULL) {
		pr_err("get physical sensor module name is null\n");
		return;
	}
	if(strncmp(physicalSnesorModuleName,"IMX686_H",8) == 0 && g_ois_status[OIS_CLIENT_IMX686] == 1) {
		g_dac_macro[OIS_CLIENT_IMX686] = dac_macro;
		g_dac_infinity[OIS_CLIENT_IMX686] = dac_infinity;
	}else if(strncmp(physicalSnesorModuleName,"OV08A10_H",9) == 0 && g_ois_status[OIS_CLIENT_OV08A10] == 1) {
		g_dac_macro[OIS_CLIENT_OV08A10] = dac_macro;
		g_dac_infinity[OIS_CLIENT_OV08A10] = dac_infinity;
	}else {
		return;
	}

	for(k=0;k<OIS_CLIENT_MAX;k++) {
		pr_info("Get AFC data[%u], [marco - %u], [infinity - %u]\n",k,g_dac_macro[k], g_dac_infinity[k]);
	}
}

void set_ois_dit_afc_data_from_eeprom(uint32_t dac_macro, uint32_t dac_infinity,char* physicalSnesorModuleName)
{
	int k;
	if(physicalSnesorModuleName == NULL) {
		pr_err("get physical sensor module name is null\n");
		return;
	}
	if(strncmp(physicalSnesorModuleName,"IMX686_H",8) == 0 && g_ois_status[OIS_CLIENT_IMX686] == 1) {
		g_dac_macro_dit[OIS_CLIENT_IMX686] = dac_macro;
		g_dac_infinity_dit[OIS_CLIENT_IMX686] = dac_infinity;
	}else if(strncmp(physicalSnesorModuleName,"OV08A10_H",9) == 0 && g_ois_status[OIS_CLIENT_OV08A10] == 1) {
		g_dac_macro_dit[OIS_CLIENT_OV08A10] = dac_macro;
		g_dac_infinity_dit[OIS_CLIENT_OV08A10] = dac_infinity;
	}else {
		return;
	}
	for(k=0;k<OIS_CLIENT_MAX;k++) {
		pr_info("Get DIT AFC data[%u], [marco - %u], [infinity - %u]\n",k,g_dac_macro_dit[k],g_dac_infinity_dit[k]);
	}
}

#if defined ASUS_AI2202_PROJECT

static void set_ssc_gain_if_need(uint32_t ois_index,uint32_t af_state,uint32_t distance_mm)
{
	struct timespec64 t1,t2;
	uint8_t SendData=0x01;
	int rc = 0;
//	struct cam_ois_ctrl_t *oisCtrl = ois_ctrl[ois_index];
	static uint32_t prev_distance = 0;

	if( g_ois_power_state[ois_index])
	{
		if(rc == 0) {
			if(distance_mm != prev_distance)
			{
					/* To enable the shift correction. */
					SendData = 0x01;
					IIC_DataSend(0x0558, 1, &SendData); /* ACC_CTRL ACCEN set */
					/* ACC_CTRL register(0x0558) 1byte send */

					ktime_get_boottime_ts64(&t1);
					IIC_DataSend(0x588, 4, (uint8_t*)&distance_mm);
					ktime_get_boottime_ts64(&t2);
					pr_info("[OIS] AF state(%u), CONFIG ssc distance(%x) mm, cost %lld us\n",af_state,distance_mm,diff_time_us(&t2,&t1));
					g_ssc_config_time_prev = t1;
					prev_distance = distance_mm;
			}else {
				pr_info("[OIS] distance_cm is same as previous distance_mm(%x) g_distance_prev(%x)\n",distance_mm,prev_distance);
			}
		}
	}
}
#endif

#define DEFAULT_ONE_LINE_MAX_SIZE 100000

#if defined ASUS_AI2202_PROJECT
bool checkForNewBoard(uint8_t id) { return false;}
#define Sake_IMX686_Camera_ID 0x76
#define OIS_GYRO_GAIN_RECAL "ois_gyro_gain_cali.txt"
#define OIS_STATUS_INITIALIZE 255
void update_OIS_gyro_gain_XY(uint32_t ois_index, uint32_t ois_gryo_cal_data_x, uint32_t ois_gryo_cal_data_y ) {

	if(g_ois_power_state[ois_index])
	{
		unsigned char SendData;
		pr_err("[OIS] gyro gain update x=0x%X, y=0x%X\n", ois_gryo_cal_data_x, ois_gryo_cal_data_y);
		SendData = 0x10;
		/* GGACTRL XGGEN=1 */
		IIC_DataSend(0x0254, 4, (unsigned char*) &( ois_gryo_cal_data_x ));
		/* XGG register (0x0254) 4Byte Send */
		IIC_DataSend(0x15, 1, & SendData);
		/* GGACTRL register (0x0015) 1Byte Send */
		Wait(10);

		SendData = 0x20;
		/* GGACTRL YGGEN=1 */
		IIC_DataSend(0x0258, 4, (unsigned char*) &( ois_gryo_cal_data_y)); /* YGG register (0x0258) 4Byte Send */
		IIC_DataSend(0x15, 1, &SendData);
		/* GGACTRL register (0x0015) 1Byte Send */
		Wait(10);
		/* wait X msec */

	}
	else
		pr_err("[OIS] gyro gain OIS[%d] POWER DOWN\n", ois_index);

}
#endif
void asus_ois_init_config(uint32_t ois_index)
{
//jj
	pr_info("ois_index = %d\n",ois_index);
	if(ois_ctrl[ois_index] == NULL)
	{
		pr_err("OIS not init!!!\n");
		return;
	}

	if(g_ois_status[ois_index] == 1)
	{
//		ZF7_WaitProcess(ois_ctrl[ois_index],0,__func__);
		//onsemi_config_ssc_gain(ois_ctrl,100);//TODO
	}
	else
	{
		pr_err("OIS probe failed, not config ssc gain\n");
	}
	g_ois_power_state[ois_index] = 1;
	g_ois_camera_open[ois_index] = 1;
	g_ois_mode = 255;
	ois_ctrl[ois_index]->ois_on = true;
	memset(&g_ssc_config_time_prev,0,sizeof(struct timespec64));

	if(g_dac_macro_dit[ois_index] && g_dac_infinity_dit[ois_index])
	{
		g_dac_macro_base[ois_index] = g_dac_macro_dit[ois_index];
		g_dac_infinity_base[ois_index] = g_dac_infinity_dit[ois_index];
	}
	else
	{
		g_dac_macro_base[ois_index] = g_dac_macro[ois_index];
		g_dac_infinity_base[ois_index] = g_dac_infinity[ois_index];
	}
}

void asus_ois_deinit_config(uint32_t ois_index)
{
	pr_info("ois_index = %d\n",ois_index);
	g_ois_power_state[ois_index] = 0;
	g_ois_camera_open[ois_index] = 0;
	g_ois_mode = 255;
	ois_ctrl[ois_index]->ois_on = 0;
	memset(&g_ssc_config_time_prev,0,sizeof(struct timespec64));

	g_dac_macro_base[ois_index] = 0;
	g_dac_infinity_base[ois_index] = 0;
}

void asus_ois_init(struct cam_ois_ctrl_t * ctrl)
{
	if(ctrl && ctrl->soc_info.index < OIS_CLIENT_MAX) {
		ois_ctrl[0] = ctrl;
		pr_info("E index = %d\n",ctrl->soc_info.index);
	}else
	{
		pr_err("ois_ctrl_t NULL?(%d), index = %d\n",ctrl == NULL?1:0,ctrl->soc_info.index);
		return;
	}
	create_ois_proc_files_factory();

	mutex_init(&g_busy_job_mutex);
	//onsemi_get_50cm_to_10cm_lens_shift(&g_lens_shift_10cm_to_50cm);//TODO
	//onsemi_get_10cm_lens_shift(&g_lens_shift_10cm);//TODO
}
//ASUS_BSP Byron add for sma eeprom dump +++
#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT || defined ASUS_SAKE_PROJECT || defined ASUS_VODKA_PROJECT
static void dump_sma_eeprom(struct cam_ois_ctrl_t *oisCtrl,uint32_t addr,uint8_t length) {
	const uint32_t addr_offset = 0x01320300;
	uint32_t check_value;
	uint8_t i=0;
	uint32_t *data;
	data	= (uint32_t *)vmalloc(sizeof(uint32_t)*length);
	if(data == NULL) {
		pr_err("no memory to alloc\n");
	}

	for(i=0;i<length;i++) {
		addr += i;
		onsemi_write_dword(oisCtrl,0xF01E, 0x01320200);
		ZF7_WaitProcess(oisCtrl,0,__func__);
		onsemi_write_dword(oisCtrl,0xF01E, (addr_offset+addr));
		ZF7_WaitProcess(oisCtrl,0,__func__);
		onsemi_write_dword(oisCtrl,0xF01E, 0x01320401);
		ZF7_WaitProcess(oisCtrl,0,__func__);
		onsemi_write_dword(oisCtrl,0xF01D, 0x01320600);
		ZF7_WaitProcess(oisCtrl,0,__func__);
		onsemi_read_dword(oisCtrl,0xF01D, &check_value);
		ZF7_WaitProcess(oisCtrl,0,__func__);
		if(check_value != 0x40) {
			pr_err("get addr(0x%02x) failed, check value is = 0x%x\n",addr,check_value);
			break;
		}
		onsemi_write_dword(oisCtrl,0xF01D, 0x01320000);
		ZF7_WaitProcess(oisCtrl,0,__func__);
		onsemi_read_dword(oisCtrl,0xF01D, (data+i));
		ZF7_WaitProcess(oisCtrl,0,__func__);
		pr_info("get data from addr(0x%02x)= 0x%02x\n",addr,*(data+i));
	}
	if(sysfs_write_byte_from_dword_seq(SMA_OFFSET_DUMP,data,length)) {
		pr_err("write file failed\n");
	}
	vfree(data);
	return;

}
#endif
//ASUS_BSP Byron add for sma eeprom dump ---

bool read_binary_fw_file(const char* fileName, void (*process)( char*, ssize_t)) {
	uint8_t* mybuf=NULL;
	uint32_t size=0;
	pr_info("[OIS] FW read_kernel_file++");
	mybuf = vmalloc(sizeof(uint8_t)*DEFAULT_ONE_LINE_MAX_SIZE);
	if ( !mybuf ) {
		pr_err("[OIS] FW update ois fw, malloc mybuf fail!!");
		return 0;
	}

	asus_ois_read_fw(fileName, mybuf, &size);
	if (size) {
		(*process)(mybuf, size);
	}

	vfree(mybuf);
	pr_info("[OIS] FW read_kernel_file--");
	return size>0;
}

unsigned short CalcCheckSum(unsigned short *Addr, unsigned short Size)
{
	unsigned short CheckSum;
	unsigned short i;
	CheckSum = 0;
	for( i = 0; i < (Size / 2); i++ ) {
		CheckSum += Addr[i];
	}


	pr_info("[OIS] FW check sum=%d...", CheckSum);
	return CheckSum;
}


int32_t IIC_DataSend(unsigned short addr, uint32_t size, unsigned char* SendData) {
	int32_t rc=0;
	uint16_t loop=1;
	uint16_t iLoop=0;
	struct cam_ois_ctrl_t *oisCtrl = NULL;
	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array data;
	uint8_t *ptr = NULL;
	int32_t  cnt=0;
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	void *vaddr = NULL;
	#ifdef DEBUG_S4W_OIS
		char oneLine[1024]="";
		char oneChar[100]="";
	#endif

	bool bWriteByteVersion=1;
	struct cci_device *cci_dev=NULL;

	get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;

#ifdef DEBUG_S4W_OIS
	if (size<100) {
		for (cnt=0;cnt<size;cnt++) {
					sprintf(oneChar, " 0x%X", SendData[cnt]);
					strcat(oneLine, oneChar);
		}
		pr_err("[OIS] DataSend[0x%X]=%s\n",  addr, oneLine );
	} else {
		pr_err("[OIS] DataSend, size=%d > 100, skip", size);
	}
#endif


	oisCtrl->io_master_info.cci_client->i2c_freq_mode=OIS_I2C_MODE;
	cci_dev = v4l2_get_subdevdata(oisCtrl->io_master_info.cci_client->cci_subdev);
	if (cci_dev->cci_clk_params[oisCtrl->io_master_info.cci_client->i2c_freq_mode].hw_scl_stretch_en==0 && size>10) {
		pr_err("[OIS] Randy debug I2C channel stretching mode is off and size>10. Skip write");
		return -1;
	}
	if ( bWriteByteVersion) {
		i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
		i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
		i2c_reg_setting.size = size;
		i2c_reg_setting.delay = 0;
		vaddr = vmalloc(sizeof(struct cam_sensor_i2c_reg_array) * size);
		if (!vaddr) {
			CAM_ERR(CAM_OIS,
				"Failed in allocating i2c_array: size: %u", size);
			return -ENOMEM;
		}

		i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (vaddr);

		for (cnt = 0, ptr = (uint8_t *)SendData; cnt < size;
			cnt++, ptr++, addr++) {
			i2c_reg_setting.reg_setting[cnt].reg_addr = addr;
			i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
			i2c_reg_setting.reg_setting[cnt].delay = 0;
			i2c_reg_setting.reg_setting[cnt].data_mask = 0;
		}

		rc = camera_io_dev_write_continuous(&(oisCtrl->io_master_info),
			&i2c_reg_setting, CAM_SENSOR_I2C_WRITE_BURST);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "IIC_DataSend size(%d)  failed. %d",size, rc);
		}
		vfree(vaddr);
		vaddr = NULL;

	} else {

		data.reg_addr = addr;
		data.reg_data = *SendData;
		data.delay = 0;
		data.data_mask = 0;//how to use? not used so far

		write_setting.reg_setting = &data;
		write_setting.size = 1;
		write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
		write_setting.data_type = size;
		write_setting.delay = 0;
		if (size>4) {
			loop=size;
			write_setting.data_type=1;
		}

		for (iLoop=0;iLoop<loop;iLoop++,data.reg_addr++,++SendData) {
			rc = camera_io_dev_write(&(oisCtrl->io_master_info),&write_setting);
			if(rc < 0){
				pr_err("write 0x%x to reg 0x%x failed! rc = %d", data.reg_data,data.reg_addr,rc);
			}
			data.reg_data = *(SendData);
		}

}
	return rc;
}


unsigned char IIC_WaitSendByte(unsigned short addr, unsigned char SendData) {
	unsigned char RcvData;
	unsigned char retryTimes=0;
	IIC_DataSend(addr, 1, & SendData);
	Wait (190); /* Wait for Flash ROM Write + 20msec is margin in low frequency clock */
	do
	{
		IIC_DataRead(addr, 1, & RcvData);
		Wait(1);
		if (++retryTimes>100) {
			pr_err("[OIS] IIC_WaitSendByte, [%X]=0x%X. Fail!!", addr, RcvData);
			return RcvData;
		}
	}while (RcvData != 0);
	pr_err("[OIS] IIC_WaitSendByte, [%X]=0x%X. OK", addr, SendData);
	return 0;
}
#define X_PID_Filter_Gain_Address 0x27c
#define Y_PID_Filter_Gain_Address 0x294

static bool FlashWriteResultCheckForCaliOnly(void);

static void XYCoeffecBoundaryLimit(uint32_t addr) {
#define XY_COF_BOUNDARY 0x30D40
//#define XY_COF_BOUNDARY 0x249F1
#define XY_COF_LIMIT 0x249F0
	uint32_t data;
	IIC_DataRead(addr, 4,  (uint8_t *)&data);
	if ( data>XY_COF_BOUNDARY) {
		pr_err("[OIS][XYCoff] reg 0x%x=0x%x is larger then 0x%x, limit to 0x%x", addr, data, XY_COF_BOUNDARY, XY_COF_LIMIT);
		if ( addr == Y_PID_Filter_Gain_Address ) {
			ois_pic_y_org_data=data;
		} else {
			if ( addr == X_PID_Filter_Gain_Address ) {
				ois_pic_x_org_data=data;
			}
		}
		data=XY_COF_LIMIT;
		IIC_DataSend(addr, 4,  (uint8_t *)&data);
		FlashWriteResultCheckForCaliOnly();
	} else {
//remove below debug info
		pr_err("[OIS][XYCoff] reg 0x%x=0x%x is smaller then 0x%x, skip apply limit", addr, data, XY_COF_BOUNDARY);
	}

}


int32_t IIC_DataRead(uint32_t addr, int size, uint8_t *data){
#ifdef DEBUG_S4W_OIS
	int32_t  cnt=0;
		char oneLine[1024]="";
		char oneChar[100]="";
#endif
	struct cam_ois_ctrl_t *oisCtrl = NULL;

	get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) {
		pr_err("Randy debug OIS get OIS control fail");
		return -1;
	}
	oisCtrl->io_master_info.cci_client->i2c_freq_mode=OIS_I2C_MODE;
#ifdef DEBUG_S4W_OIS
		cnt= camera_io_dev_read_seq(&(oisCtrl->io_master_info),addr,data,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_BYTE, size);
		if (size<100 && cnt >=0) {
			for (cnt=0;cnt<size;cnt++) {
						sprintf(oneChar, " 0x%X", data[cnt]);
						strcat(oneLine, oneChar);
			}
			pr_err("[OIS] DataRead[0x%X]=%s\n",  addr, oneLine );
		} else {
			pr_err("[OIS] DataRead, size=%d rc=%d, skip", size, cnt);
		}
		return 0;
		#else
		return camera_io_dev_read_seq(&(oisCtrl->io_master_info),addr,data,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_BYTE, size);

#endif
}

#include <linux/kernel.h>
#include <linux/of.h>
int checkHWBoard(void) {
    u32 board_id[2] = { 0 };
    struct device_node *root;
   root = of_find_node_by_path("/");

    if (!of_property_read_u32_array(root, "qcom,board-id", board_id, 2)) {
        return board_id[0];
    }
	return -1;

}

#define WAIT_LOOP_DELAY 10
bool Loop_Check_reg(uint32_t addr, int size, uint8_t data, uint16_t delay) {
	unsigned char iRetryTime=0;
	unsigned char RcvData=0;
	uint16_t orgDelay=delay;
	for (;iRetryTime<5;iRetryTime++) {
		for (delay=orgDelay;(delay-=WAIT_LOOP_DELAY)>0;Wait(WAIT_LOOP_DELAY)) {
			IIC_DataRead(addr, size, &RcvData);
			if (RcvData==data) {
				pr_err("[OIS] LoopGain status [0x%X]=0x%X=0x%X OK", addr, RcvData, data);
				return 1;
			}
			if (Target_OIS_Status) return 0;
		}
	}
	pr_err("[OIS] LoopGain status error [0x%X]=0x%X != 0x%X", addr, RcvData, data);
	return 0;
}

bool FlashWriteResultCheck(void) {
//	unsigned char SendData;
	unsigned char RcvData;
	#if 0
	SendData = 0xAA;
	IIC_DataSend(0x0027, 1, & SendData);
	/* Set Result Check Data */
	/* FLSWRTRESULT register (0x0027) 1byte send */
	/* Exec OIS DATA AREA Write */
	SendData = 0x01;
	/* Set OIS_W=1 */
	IIC_DataSend(0x0003, 1, & SendData);
	/* OISDATAWRITE register (0x0003) 1byte send */
	Wait(190); /* Wait for Flash ROM Write + 20msec is margin in low frequency clock */
//	do
//	{
//		IIC_DataRead(0x0003, 1, & RcvData);
//		/* OIS_W read */
//		retryTime++
//		if (retryTime>5) return 0;
//		Wait(1);
//	}while (RcvData != 0);
	if (!Loop_Check_reg(0x0003, 1, 0, 1)) return 0;
	/* Exec PID parameter Init */
	SendData = 0x21;
	/* PIDPARAMINIT PART2 set */
	IIC_DataSend(0x0036, 1, & SendData);
	/* PIDPARAMINIT register (0x0036) 1Byte Send */
	Wait (190); /* Wait for Flash ROM Write + 20msec is margin in low frequency clock */
//	do
//	{
//		IIC_DataRead(0x0036, 1, & RcvData);
//		/* PIDPARAMINIT INIT_EN read */
//		retryTime++
//		if (retryTime>5) return 0;
//		Wait(1);
//	}while (RcvData != 0);
	if (!Loop_Check_reg(0x0036, 1, 0, 1)) return 0;
	/* Exec User Data Area Write */
	SendData = 0x07;
	/* Write to Flash Command Set */
	IIC_DataSend(0x000E, 1, &SendData);
	/* DFLSCMD register (0x000E) 1byte send */
	Wait (190); /* Wait for Flash ROM Write + 20msec is margin in low frequency clock */
//	do
//	{
//		IIC_DataRead(0x000E, 1, &RcvData);
//		/* DFLSCMD register (0x000E) Read */
//		retryTime++
//		if (retryTime>5) return 0;
//		Wait(1);
//	}while (RcvData != 0x17);
	if (!Loop_Check_reg(0x000E, 1, 0x17, 1)) return 0;
	IIC_DataRead(0x0027, 1, &RcvData);
	/* FLSWRTRESULT register (0x0027) 1byte read */
	if(RcvData != 0x00AA )
	{
	/* Flash Write Error */
		pr_err("[OIS] S4W flash ROM write, status=%d. Fail!!", RcvData);
		return 0;
	}
	else
	{
	/* Flash Write Success */
		pr_err("[OIS] S4W flash ROM write OK");
		return 1;
	}
#else //for EVB&SR
	unsigned long PCLK=19200000;
	unsigned char PLL=5;
	unsigned long PCLKfromFW=0;
	/* Exec PID parameter Init */
	IIC_WaitSendByte(0x0036, 0x21);


//read and check if PCLK is correct
	IIC_DataRead(0x03F0, 4, (unsigned char*)&PCLKfromFW);

//After SR
//PCLK = 19200000
//PLL = 5

//EVB
if (checkHWBoard()==AI2202_EVB) {
	PCLK = 24000000;
	PLL = 4;
}

	if( PCLKfromFW!=PCLK) {
		pr_err("OIS s4sw PCLK=%d change to %d", PCLKfromFW, PCLK);
		IIC_DataSend(0x03F0, 4, (unsigned char*)&PCLK);
		IIC_DataSend(0x03F4, 1, (unsigned char*)&PLL);
	} else {
		pr_err("OIS s4sw PCLK=%d is same to %d, skip update PCLK", PCLKfromFW, PCLK);
	}

	/* OIS Status Check */
	IIC_DataRead(0x01, 1, &RcvData);
	if( RcvData != 1 )
	{
		pr_err("[OIS] S4W is not ready to update2, status=%d. Fail!!", RcvData);
		return 0;
	}

	if( IIC_WaitSendByte(0x0003, 0x01) ) {
		pr_err("[OIS] S4W flash ROM write Fail!!");
		return 0;
	} else {
		pr_err("[OIS] S4W flash ROM write OK");
		return 1;
	}


	#endif
}


#include <linux/firmware.h>

static int asus_ois_read_fw(const char* fileName, char * buf, uint32_t* readSize)
{
	int32_t                            rc = 0;
	const struct firmware             *fw = NULL;

	struct cam_ois_ctrl_t *o_ctrl = NULL;
	get_ois_ctrl(&o_ctrl);
	*readSize=0;
	pr_info("[OIS] FW asus_ois_read_fw++");

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	rc = request_firmware(&fw, fileName,  &(o_ctrl->pdev->dev));
	if (rc) {
		pr_err( "[OIS] FW Failed to locate %s", fileName);
		return rc;
	}
	pr_info("[OIS] FW prog size:%d.", fw->size);
	memcpy(buf, fw->data, fw->size);
	release_firmware(fw);
	*readSize=fw->size;
	pr_info("[OIS] FW asus_ois_read_fw--, rc=%d", rc);
	return rc;
}



static bool FlashWriteResultCheckForCaliOnly(void) {
	if( IIC_WaitSendByte(0x0003, 0x01))
	{
		pr_err("[OIS] S4W calibration flash write Fail!!");
		return 0;
	} else {
		pr_err("[OIS] S4W calibration flash write OK");
		return 1;
	}

}

void S4W_FW_Update( char* Firmware_Data_Buffer, ssize_t FirmwareSize)
{
#define S4W_FULL_Size 28672
	unsigned char SendData[256], RcvData;
	unsigned char*progCode=NULL;
	unsigned short RcvDataShort;
	unsigned long vFirmwareVersion;
	unsigned long vFileVersion;
	int WriteCnt;
	unsigned short checkSum;
	int iRetry=0, iBlockRetry=0;

	if ( FirmwareSize != S4W_FULL_Size ) {
		pr_err("[OIS] FW read firmware is not complete, only %d", FirmwareSize);
		return;
	}

	/* OIS Status Check */
	IIC_DataRead(0x01, 1, &RcvData);
	if( RcvData != 1 )
	{
		pr_err("[OIS] FW S4W is not ready to update1, status=%d. Fail!!", RcvData);
		return;
	}
	/* OISSTS Read */
	/* OISSTS != IDLE */
	/* get New F/W Revision */
//	NewFWRev = Read target Binary File address 0x6FF4-0x6FF7; (*1)
	vFileVersion = *(long*)&Firmware_Data_Buffer[0x6FF4];

	/* Check RUMBA device */
	IIC_DataRead(0xFC, 4, SendData);
	vFirmwareVersion = *(long*)&SendData[0];

	if (checkHWBoard()==AI2202_EVB) {
		pr_err("[OIS] FW This is EVB board");
	}


	if ((int)vFirmwareVersion>=(int)vFileVersion) {
		pr_err("[OIS] FW  X binary file version=%d, module firmware version=%d, SKIP! Current_OIS_status=%d", vFileVersion, vFirmwareVersion, Current_OIS_Status);
		return;
	}

#ifdef UPDATE_FW_BY_PROC_ONLY
	if (Target_OIS_Status!=1) {
		pr_err("[OIS] FW X binary file version=%d, firmware version=%d, PAUSE update firmware, Current_OIS_status=%d", vFileVersion, vFirmwareVersion, Current_OIS_Status);
		return;
	}
#endif
	pr_err("[OIS] FW X binary file version=%d, firmware version=%d. Start to update OIS firmware", vFileVersion, vFirmwareVersion);
#if 1
	/* Update a User Program */
	/* SET FWUP_CTRL REG */
	SendData [0] = 0x75;
	/* Update Block Size =7, FWUP_DSIZE=256Byte, FWUPEN=Start */
	IIC_DataSend(0x0C, 1, SendData); /* FWUPCTRL REG(0x000C) 1Byte Send */
	pr_info("[OIS] FW set update ready set[0x75]=0x%X", SendData[0]);
	Wait(65); /* Wait for FlashROM(7Blocks) Erase Process + 10msec is margin in low frequency clock */
	/* Read BOOT F/W revision */
#else
	pr_err("[OIS] for test purpose, skip update");
#endif

	progCode = &Firmware_Data_Buffer[0];
#define I2C_BLOCK_RETRY 3
for (iRetry=0;iRetry<I2C_BLOCK_RETRY;iRetry++){
	pr_err("[OIS] FW update retry=%d...", iRetry);

	for( WriteCnt=0; WriteCnt<28672/256; WriteCnt++)
	{
		memcpy(SendData, progCode, 256);
		for(iBlockRetry=0;iBlockRetry<I2C_BLOCK_RETRY;iBlockRetry++) {
			if (IIC_DataSend(0x100, 256, SendData)<0) { /* FLS_DATA REG(0x0100) 256Byte Send */
			pr_err("[OIS] FW update error, WriteCnt=%d, retry=%d", WriteCnt, iBlockRetry);
			continue;
			} else {
				break;
			}
		}
		if (iBlockRetry>=I2C_BLOCK_RETRY){
			pr_err("[OIS] FW update error, WriteCnt=%d, retry=%d, reach maximun retry, abort", WriteCnt, iBlockRetry);
			checkSum = CalcCheckSum((unsigned short *)progCode, 28672);
			return	;
		}
		progCode += 256;
		Wait(13); /* Wait for Flash ROM Writing(64bytes*4) + 3msec is margin in low frequency clock. */
	}
	/* Write Error Status Check */
	IIC_DataRead(0x6, 2, (unsigned char*)& RcvDataShort); /* Error Status read */
	if(RcvDataShort == 0x0000)
	/* F/W Update Error check */
	{
		/* CHECKSUM */
		progCode = &Firmware_Data_Buffer[0];
		checkSum = CalcCheckSum((unsigned short *)progCode, 28672);
		SendData [0] = (checkSum & 0x00FF);
		SendData [1] = (checkSum & 0xFF00) >> 8;
		SendData [2] = 0;
		/* Don't Care */
		SendData [3] = 0x80;
		/* Self Reset Request */
		IIC_DataSend(0x8, 4, SendData); /* FWUP_CHKSUM REG(0x0008) 4Byte Send */
		Wait(20);
		Wait(190);
		/* Wait RUMBA Self Reset */
		/* OIS Data Section initialize + 20msec is margin in low frequency clock */
		IIC_DataRead(0x6, 2, (unsigned char*)& RcvDataShort);
		/* Error Status read */
		if(RcvDataShort == 0x0000)
		/* F/W Update Error check */
		{
			IIC_DataRead(0xFC, 4, (unsigned char*)&vFirmwareVersion); /* Read revision of writing F/W */
			if((int)vFirmwareVersion==(int)vFileVersion)
			/* Check Writing F/W Rev. */
			{
			/* F/W Update Success Process */
				pr_err("[OIS] FW update OK, new version=%d", vFirmwareVersion);
				FlashWriteResultCheck();
#ifdef ENABLE_OIS_CALIBRATION
				Current_OIS_Status=1;
#endif
				break;
			}
			else {
			/* F/W Update Error Process */
				pr_err("[OIS] FW update Fail, version dismatch, current version=%d, expect to %d", vFirmwareVersion, vFileVersion);
			}
		} else {
			/* F/W Update Error Process */
			pr_err("[OIS] FW update complete, but checksum error=0x%X", RcvDataShort);
		}
	} else {
		/*F/W update Error*/
		pr_err("[OIS] FW update error=0x%X", RcvDataShort);

}}
}



uint8_t Loop_Gain_Cali(void) {
	unsigned char SendData=0;

	uint32_t old_0x27c=0;
	uint32_t old_0x294=0;
	uint32_t new_0x27c=0;
	uint32_t new_0x294=0;
	Target_OIS_Status=0;
	if (Current_OIS_Status==99) {
		pr_err("[OIS] LoopGain status error=%d, FW is upgrading, can not do loop gain", Current_OIS_Status);
		return 1;
	}

	Current_OIS_Status=3;

	IIC_DataRead(0x027c, 4, (uint8_t *)&old_0x27c);
	IIC_DataRead(0x0294, 4, (uint8_t *)&old_0x294);

	IIC_DataSend(0x0, 1, &SendData);
	if ( !Loop_Check_reg(0x1, 1, 1, 100) ) return 2;
	SendData=0x03;
	IIC_DataSend(0xD0, 1, &SendData);
	if ( !Loop_Check_reg(0xD0, 1, 0x2, 1000) ) return 3;
	SendData=0x13;
	IIC_DataSend(0xD0, 1, &SendData);
	if ( !Loop_Check_reg(0xD0, 1, 0x12, 1000) ) return 4;

	IIC_DataRead(0x027c, 4, (uint8_t *)&new_0x27c);
	if (new_0x27c==old_0x27c || new_0x27c<100000 || new_0x27c>999999) return 5;
	IIC_DataRead(0x0294, 4, (uint8_t *)&new_0x294);
	if (new_0x294==old_0x294 || new_0x294<100000 || new_0x294>999999) return 6;
//temp to disable save	if (FlashWriteResultCheckForCaliOnly()==0) return 7;

	if (Current_OIS_Status==3) Current_OIS_Status=30;

	if (Current_OIS_Status==30) {
		pr_err("[OIS] LoopGain status OK, Current_OIS_Status=%d, 0x27c=[%X] to [%X], 0x294=[%X] to [%X]", Current_OIS_Status, old_0x27c, new_0x27c, old_0x294, new_0x294);
		return 0;
	} else {
		pr_err("[OIS] LoopGain status Fail, Current_OIS_Status=%d, 0x27c=[%X] to [%X], 0x294=[%X] to [%X]", Current_OIS_Status, old_0x27c, new_0x27c, old_0x294, new_0x294);
		return Current_OIS_Status;
	}
}

void OIS_S4W_FW_Update(void) {
#define S4W_FW_File "s4w.bin"
	read_binary_fw_file(S4W_FW_File, S4W_FW_Update);
}


uint8_t GyrosensorCalibration (void) {
	unsigned char SendData [11];
	unsigned char RcvData=0;
	short gyro_data[GYRO_K_REG_COUNT], caliration_sts;
	int rc=0;

	/* OIS Status Check */
	IIC_DataRead(0x01, 1, &RcvData);
	if ( RcvData != 1 ) {
			pr_err("[OIS] OIS status(%d) is not idle, skip gyro calibration", RcvData);
		return 1;
	}
	/* Gyro Calibration Start */
	SendData [0] = 0x01;

	if( IIC_WaitSendByte(0x0014, 0x01)) return -1;

	/* Result check */
	IIC_DataRead(0x04, 1, &RcvData);
	/* OISERR Read */
	if ((RcvData & 0x23) == 0x0)
	/* OISERR register GXZEROERR & GYZEROERR & GCOMERR Bit = 0 (No Error) */
	{
		/* Write Gyro Calibration result to OIS DATA SECTION */
		if (FlashWriteResultCheckForCaliOnly()==0) return -1;
		/* refer to 4.17Flash ROM Write Result Check Sample Source */
	}
	Wait (150);
	/* After the Gyro Sensor Calibration is finished, wait time of 150msec is necessary because of the specification of ICM-40608.
	During the 150msec, please do not set "OISEN=1" and do set not "GCEN=1".
	Detail is described in "AN-000166" from TDK. */


	IIC_DataRead(0x0248, 2, (uint8_t*)&gyro_data[0]);
	IIC_DataRead(0x024A, 2, (uint8_t*)&gyro_data[1]);
	pr_err("[OIS] GyrosensorCalibration OK, XGZERO[0x0248]=%d, XGZERO[0x024A]=%d", gyro_data[0], gyro_data[1]);

	IIC_DataRead(0x0004, 2, (uint8_t*)&caliration_sts);
	pr_err("[OIS] GyrosensorCalibration OK, OISERR[0x0004]=0x%X", caliration_sts);

/*
	SendData [0] = 0x01;
	IIC_DataSend(0x0, 1, SendData); //OIS ON
	SendData [0] = 0x01;
	IIC_DataSend(0x80, 1, SendData); //info ON
	Wait (20);


	IIC_DataRead(0x0082, 2, (uint8_t*)&gyro_data[0]);
	IIC_DataRead(0x0084, 2, (uint8_t*)&gyro_data[1]);

	pr_err("[OIS] GyrosensorCalibration OK, GETGRX=0x%X, GETGRY=0x%X", gyro_data[0], gyro_data[1]);
*/

	rc=sysfs_write_word_seq_change_line(OIS_GYRO_K_OUTPUT_FILE_NEW,gyro_data,GYRO_K_REG_COUNT,2, ' ');

	if ( gyro_data[0]>2096 || gyro_data[0]<-2096 ||
		 gyro_data[1]>2096 || gyro_data[1]<-2096 || rc !=0 ) {
			return -1;
	}
	return 0;
}
