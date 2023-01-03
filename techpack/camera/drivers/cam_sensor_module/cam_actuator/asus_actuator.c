#include <linux/proc_fs.h>
//#include "asus_cam_sensor.h"
#include "asus_actuator.h"
#include "actuator_i2c.h"
#include "cam_actuator_core.h"

#undef  pr_fmt
#define pr_fmt(fmt) "ACTUATOR-ATD %s(): " fmt, __func__

#define	PROC_POWER	"driver/actuator_power"
#define	PROC_I2C_RW	"driver/actuator_i2c_rw"
#define	PROC_VCM_ENABLE	"driver/vcm_enable"
#define	PROC_VCM_VALUE	"driver/vcm"
#define	PROC_VCM_EEPROM	"driver/vcm_eeprom"
#define	PROC_VCM_ACC_OFFSET	"driver/vcm_acc_offset"
#define	PROC_VCM_ACC_DATA	"driver/vcm_acc_data"
#define	PROC_VCM_IC_RW	"driver/vcm_ic_rw"
#define	PROC_VCM_GYRO_INIT	"driver/vcm_gyro_init"



static struct cam_actuator_ctrl_t * actuator_ctrl = NULL;



static struct mutex g_busy_job_mutex;

uint8_t g_actuator_power_state = 0;
uint8_t g_actuator_camera_open = 0;

#if defined ASUS_AI2202_PROJECT
#define VCM_NUMBER 4 //AI2202 project has 4 physical cam
static struct cam_actuator_ctrl_t * g_actuator_ctrl[VCM_NUMBER];
static uint16_t vcm_dac_reg_addr[VCM_NUMBER] = {0x03,0x03,0x84,0x03};//{imx766_12m,imx663,imx363,imx766}
//static uint16_t vcm_slave_id[VCM_NUMBER] = {0x72,0x0c,0x72,0x72};//{imx766_12m,imx663,imx363,imx766}
static uint8_t g_atd_status = 0;//fail
static uint16_t g_reg_addr = 0x03; //VCM read focus data addr  //default set  cam0 imx766's reg_addr
static uint32_t g_reg_val = 0;
static uint16_t g_slave_id = 0x72; //E4(8bit) 72(7bit)  //default set  cam0 imx766's slave_id

static uint16_t g_slave_id_eeprom = 0x73;
#else
static uint8_t g_atd_status = 0;//fail
static uint16_t g_reg_addr = 0x03; //VCM read focus data addr
static uint32_t g_reg_val = 0;
static uint16_t g_slave_id = 0x72; //E4(8bit) 72(7bit)
static uint16_t g_slave_id_eeprom = 0x73;
#endif

static uint32_t vcm_acc_data[16] = {0};

static enum camera_sensor_i2c_type g_data_type = CAMERA_SENSOR_I2C_TYPE_WORD; //word
static enum camera_sensor_i2c_type g_addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;

static uint8_t g_operation = 0;//read



void actuator_lock(void)
{
	if(actuator_ctrl == NULL)
	{
		pr_err("ACTUATOR not init!!!\n");
		return;
	}
	mutex_lock(&actuator_ctrl->actuator_mutex);
}

void actuator_unlock(void)
{
	if(actuator_ctrl == NULL)
	{
		pr_err("ACTUATOR not init!!!\n");
		return;
	}
	mutex_unlock(&actuator_ctrl->actuator_mutex);
}

int actuator_busy_job_trylock(void)
{
	if(actuator_ctrl == NULL)
	{
		pr_err("OIS not init!!!\n");
		return 0;
	}
	return mutex_trylock(&g_busy_job_mutex);
}

void actuator_busy_job_unlock(void)
{
	if(actuator_ctrl == NULL)
	{
		pr_err("OIS not init!!!\n");
		return;
	}
	mutex_unlock(&g_busy_job_mutex);
}

static void create_proc_file(const char *PATH, const struct proc_ops* f_ops)
{
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


#if 0

int32_t trans_dac_value(uint32_t input_dac)
{
	int32_t HallMax = 0x6000;
	int32_t HallMin = 0xEA80;
	int TotalRange;
	int32_t DAC;

	if(input_dac < 1 || input_dac > 4095)
	{
		CAM_INFO(CAM_ACTUATOR,"Input DAC value is wrong");
		return -1;
	}
	if(input_dac >0) input_dac -=1;

	TotalRange = (HallMax - 0x0000) + (0xFFFF - HallMin)+1;

	DAC = HallMin + ((TotalRange * input_dac)/4094);

	if(DAC > 0xFFFF ) DAC -= 0xFFFF;

	return DAC;
}
/*
int onsemi_actuator_init_setting(struct cam_actuator_ctrl_t *a_ctrl)
{
	int rc;
	uint32_t reg_addr[2] = {0xf0, 0xe0};
	uint32_t reg_data;
	uint32_t reg_input_data = 0x1;

	CAM_INFO(CAM_ACTUATOR,"onsemi_actuator_init_setting E\n");

	rc = actuator_read_byte(a_ctrl, reg_addr[0], &reg_data);
	if( rc < 0)
		CAM_INFO(CAM_ACTUATOR,"Read 0x%x failed data 0x%x\n", reg_addr[0], reg_data);

	if( reg_data == 0x42)
	{
		CAM_INFO(CAM_ACTUATOR,"0xf0 data is 0x%x! Right, do next cmd\n", reg_data);

		rc = actuator_write_byte(a_ctrl,reg_addr[1],reg_input_data);
		if(rc == 0)
		{
			CAM_INFO(CAM_ACTUATOR,"Write addr 0x%x data 0x%x success! Start delay\n",
				reg_addr[1], reg_input_data);
			mdelay(5);
			CAM_INFO(CAM_ACTUATOR,"Write addr 0x%x data 0x%x success! End delay\n",
				reg_addr[1], reg_input_data);
		}
		if( rc < 0 )
			CAM_INFO(CAM_ACTUATOR,"Write 0x%x failed\n", reg_addr[1]);
		else
		{
			reg_data = 0x5;
			rc = actuator_read_byte(a_ctrl, reg_addr[1], &reg_data);
			CAM_INFO(CAM_ACTUATOR,"Read 0x%x data 0x%x\n", reg_addr[1], reg_data);
			if(reg_data == 0x0)
				g_actuator_init_setting = 0;
		}
	}
	else
		CAM_INFO(CAM_ACTUATOR,"0xf0 data is not 0x42: 0x%x\n", reg_data);

	CAM_INFO(CAM_ACTUATOR,"onsemi_actuator_init_setting X\n");
	return rc;
}*/

#endif
static int actuator_i2c_debug_read(struct seq_file *buf, void *v)
{
	uint32_t reg_val;
	int rc;

	mutex_lock(&actuator_ctrl->actuator_mutex);
	g_actuator_power_state = 1;
	if(g_actuator_power_state)
	{
		//F40_WaitProcess(ois_ctrl,0,__func__);

		//pr_err("Actuator sid = 0x%x ", actuator_ctrl->io_master_info.cci_client->sid);
		//actuator_ctrl->io_master_info.cci_client->sid = g_slave_id;
		switch(g_data_type)
		{
			case CAMERA_SENSOR_I2C_TYPE_BYTE:
				rc = actuator_read_byte(actuator_ctrl, g_reg_addr, &reg_val, g_addr_type);
				break;
			case CAMERA_SENSOR_I2C_TYPE_WORD:
				rc = actuator_read_word(actuator_ctrl, g_reg_addr, &reg_val, g_addr_type);
				break;
			case CAMERA_SENSOR_I2C_TYPE_DWORD:
				rc = actuator_read_dword(actuator_ctrl, g_reg_addr, &reg_val, g_addr_type);
				break;
			default:
				rc = actuator_read_dword(actuator_ctrl, g_reg_addr, &reg_val, g_addr_type);
		}

		if(g_operation == 1)//write
		{
			if(reg_val == g_reg_val)
			{
				pr_info("read back the same value as write!\n");
			}
			else
			{
				pr_err("write value 0x%x and read back value 0x%x not same!\n",
						g_reg_val, reg_val
				);
			}
		}

		if(rc == 0)
		{
			g_atd_status = 1;
		}
		else
		{
			g_atd_status = 0;
			pr_err("read from reg 0x%x failed! rc = %d\n",g_reg_addr,rc);
		}

		seq_printf(buf,"0x%x\n",reg_val);
	}
	else
	{
		seq_printf(buf,"POWER DOWN\n");
	}

	mutex_unlock(&actuator_ctrl->actuator_mutex);

	return 0;
}

static int actuator_i2c_debug_open(struct inode *inode, struct  file *file)
{
	return single_open(file, actuator_i2c_debug_read, NULL);
}

/*
Normorl write:

echo   addr 2 value > actuator_i2c

		g_reg_addr = val[0];
		g_data_type = val[1];
		g_reg_val = val[2];
		g_operation = 1;






*/

static ssize_t actuator_i2c_debug_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t ret_len;
	int n;
	char messages[32]="";
	uint32_t val[5];
	int rc;
	int32_t vcm_register_value = 0;

	ret_len = len;
	if (len > 32) {
		len = 32;
	}
	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	n = sscanf(messages,"%x %x %x %x %x", &val[0], &val[1], &val[2], &val[3], &val[4]);

	mutex_lock(&actuator_ctrl->actuator_mutex);

	if(n == 1)
	{
		g_reg_addr = val[0];
		g_operation = 0;
		g_data_type = 2;//default word
	}
	else if(n == 2)
	{
		//data type
		// 1 byte 2 word 4 dword
		g_reg_addr = val[0];
		g_data_type = val[1];
		g_operation = 0;
	}
	else if(n == 3)
	{
		g_reg_addr = val[0];
		g_data_type = val[1];
		g_reg_val = val[2];
		g_operation = 1;
	}
	else if(n == 4)
	{
		g_reg_addr = val[0];
		g_data_type = val[1];
		g_reg_val = val[2];
	#if defined ASUS_AI2202_PROJECT
		if(val[3]<VCM_NUMBER )
		{
			actuator_ctrl=g_actuator_ctrl[val[3]];
			g_reg_addr=vcm_dac_reg_addr[val[3]];
			pr_info(" actuator_ctrl =%x addr = 0x%x",actuator_ctrl,g_reg_addr);
			//g_slave_id=vcm_slave_id[val[3]];
		}
		else
		{
			pr_err("camera_dir=%d not define FAIL\n",val[3]);
			//mutex_unlock(&actuator_ctrl->actuator_mutex); //ASUS_BSP Jason fix multi actuator write
			return ret_len;
		}
	#else
		g_slave_id = val[3];//slave id
	#endif
		g_operation = 1;
	}
	else if(n == 5)
	{
		g_reg_addr = val[0];
		g_data_type = val[1];
		g_reg_val = val[2];
	#if defined ASUS_AI2202_PROJECT
		if(val[3]<VCM_NUMBER )
		{
			actuator_ctrl=g_actuator_ctrl[val[3]];
			g_reg_addr=vcm_dac_reg_addr[val[3]];
			//g_slave_id=vcm_slave_id[val[3]];
		}
		else
		{
			pr_err("camera_dir=%d not define FAIL\n",val[3]);
			//mutex_unlock(&actuator_ctrl->actuator_mutex); //ASUS_BSP Jason fix multi actuator write
			return ret_len;
		}
	#else
		g_slave_id = val[3];//slave id
	#endif
		g_operation = val[4]; //0 read 1 write

		//actuator_ctrl->io_master_info.cci_client->sid = g_slave_id;
	}

	if(g_data_type != 1 && g_data_type != 2 && g_data_type != 4 )
		g_data_type = 4;//default dword


	pr_err("Actuator debug  %s SLAVE 0x%x reg 0x%x, data type %s\n",
			g_operation ? "WRITE":"READ",
			g_slave_id,
			g_reg_addr,
			g_data_type == 1?"BYTE":(g_data_type == 2?"WORD":"DWORD")
			);

	switch(g_data_type)
	{
		case 1:
			g_data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
			break;
		case 2:
			g_data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
			break;
		case 4:
			g_data_type = CAMERA_SENSOR_I2C_TYPE_DWORD;
			break;
		default:
			g_data_type = CAMERA_SENSOR_I2C_TYPE_DWORD;
	}

	if(g_operation == 1)
	{
		switch(g_data_type)
		{
			case CAMERA_SENSOR_I2C_TYPE_BYTE:

				rc = actuator_write_byte(actuator_ctrl,g_reg_addr,g_reg_val,g_addr_type);
				break;
			case CAMERA_SENSOR_I2C_TYPE_WORD:
				if(g_reg_addr == 0xFFFE)
				{
					g_reg_addr = 0xA0;
					//vcm_register_value = trans_dac_value(g_reg_val);
					rc = actuator_write_word(actuator_ctrl, g_reg_addr, vcm_register_value, g_addr_type);
					//pr_info("Dac value 0x%x register value 0x%04x\n",g_reg_val,vcm_register_value);
				}
				else
					rc = actuator_write_word(actuator_ctrl, g_reg_addr, g_reg_val, g_addr_type);
				break;
			case CAMERA_SENSOR_I2C_TYPE_DWORD:
				rc = actuator_write_dword(actuator_ctrl, g_reg_addr, g_reg_val, g_addr_type);
				break;
			default:
				rc = actuator_write_byte(actuator_ctrl, g_reg_addr, g_reg_val, g_addr_type);
		}
		if(rc < 0)
		{
			pr_err("write 0x%x to reg 0x%04x FAIL\n", g_reg_val, g_reg_addr);
			g_atd_status = 0;
		}
		else
		{
			pr_info("write 0x%x to reg 0x%04x OK\n", g_reg_val, g_reg_addr);
			g_atd_status = 1;
		}
	}

	mutex_unlock(&actuator_ctrl->actuator_mutex);

	return ret_len;
}
static const struct proc_ops actuator_i2c_debug_fops = {
	//.owner = THIS_MODULE,
	.proc_open = actuator_i2c_debug_open,
	.proc_read = seq_read,
	.proc_write = actuator_i2c_debug_write,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};





static int actuator_allow_vcm_move_read(struct seq_file *buf, void *v)
{
	mutex_lock(&actuator_ctrl->actuator_mutex);
	seq_printf(buf, "%d\n", g_vcm_enabled);
	mutex_unlock(&actuator_ctrl->actuator_mutex);
	return 0;
}

static int actuator_allow_vcm_move_open(struct inode *inode, struct  file *file)
{
	return single_open(file, actuator_allow_vcm_move_read, NULL);
}
static ssize_t actuator_allow_vcm_move_write(struct file *dev, const char *buf, size_t len, loff_t *ppos)
{
	ssize_t ret_len;
	char messages[8]="";

	uint32_t val;

	ret_len = len;
	if (len > 8) {
		len = 8;
	}
	if (copy_from_user(messages, buf, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages, "%d", &val);

	mutex_lock(&actuator_ctrl->actuator_mutex);

	if(val == 0)
		g_vcm_enabled = 0;
	else
		g_vcm_enabled = 1;

	pr_info("vcm enabled set to %d\n", g_vcm_enabled);

	mutex_unlock(&actuator_ctrl->actuator_mutex);

	return ret_len;
}

static const struct proc_ops actuator_allow_vcm_move_fops = {
	//.owner = THIS_MODULE,
	.proc_open = actuator_allow_vcm_move_open,
	.proc_read = seq_read,
	.proc_write = actuator_allow_vcm_move_write,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};
static int actuator_read_vcm_dac_read(struct seq_file *buf, void *v)
{
	uint32_t reg_val = 0;
	int rc;

	mutex_lock(&actuator_ctrl->actuator_mutex);
	g_actuator_power_state = 1;
	if(g_actuator_power_state)
	{
		//actuator_ctrl->io_master_info.cci_client->sid = g_slave_id;
		pr_info(" actuator_ctrl=%x g_reg_addr=0x%x",actuator_ctrl,g_reg_addr);
		rc = actuator_read_word(actuator_ctrl, g_reg_addr, &reg_val, CAMERA_SENSOR_I2C_TYPE_BYTE);
		if(rc == 0)
		{
			g_atd_status = 1;
		}
		else
		{
			g_atd_status = 0;
		}

		seq_printf(buf,"%d\n",reg_val);
	}
	else
	{
		seq_printf(buf,"POWER DOWN\n");
	}

	mutex_unlock(&actuator_ctrl->actuator_mutex);

	return 0;
}

static int actuator_read_vcm_dac_open(struct inode *inode, struct  file *file)
{
#if defined ASUS_AI2202_PROJECT
//ASUS_BSP for jason_yeh +++ support multi camera vcm
	char path[256];
	char * vcm_file_name= NULL;
	char temp[2];
	long vcm_open_number=0;
	int ret=0;
	vcm_file_name=d_path(&file->f_path, path, 256);
	temp[0]=vcm_file_name[strlen(vcm_file_name)-1];
	temp[1]='\0';
	ret=kstrtol(temp, 10, &vcm_open_number) ;

	pr_info(" actuator_read_vcm_dac_open descriptor for file %s strlen=%d result=%d test=%d \n",temp,strlen(temp),vcm_open_number,ret);

	actuator_ctrl= g_actuator_ctrl[vcm_open_number];
	g_reg_addr=vcm_dac_reg_addr[vcm_open_number];
	//g_slave_id=vcm_slave_id[vcm_open_number];
	pr_info(" actuator_ctrl = %x vcm_open_number = %d g_reg_addr=0x%x  g_slave_id=0x%x",actuator_ctrl,vcm_open_number,g_reg_addr,g_slave_id);
//ASUS_BSP for jason_yeh ---support multi camera vcm
#endif
	return single_open(file, actuator_read_vcm_dac_read, NULL);
}

static const struct proc_ops actuator_read_vcm_dac_fops = {
		//.owner = THIS_MODULE,
		.proc_open = actuator_read_vcm_dac_open,
		.proc_read = seq_read,
		.proc_lseek = seq_lseek,
		.proc_release = single_release,
};




static int actuator_read_vcm_eeprom_read(struct seq_file *buf, void *v)
{
	uint32_t reg_val = 0, tmp_slave_addr;
	int rc,i;
	#if defined ASUS_AI2202_PROJECT
	actuator_ctrl = g_actuator_ctrl[0];
	#endif
	mutex_lock(&actuator_ctrl->actuator_mutex);

	tmp_slave_addr = actuator_ctrl->io_master_info.cci_client->sid;
	actuator_ctrl->io_master_info.cci_client->sid = g_slave_id_eeprom;


	for (i = 0x00; i <= 0x13F; i++)
	{

		rc = actuator_read_byte(actuator_ctrl, i, &reg_val, CAMERA_SENSOR_I2C_TYPE_BYTE);
		if(rc != 0)
		{
			pr_err("Actuator read from reg 0x%x failed! rc = %d\n", i, rc);
		}else
		{
			seq_printf(buf,"0x%04X  0x%02X\n", i, reg_val);
		}
	}





	actuator_ctrl->io_master_info.cci_client->sid = tmp_slave_addr;

	mutex_unlock(&actuator_ctrl->actuator_mutex);
	return 0;

}



static int actuator_read_vcm_eeprom_open(struct inode *inode, struct  file *file)
{
	return single_open(file, actuator_read_vcm_eeprom_read, NULL);
}

static const struct proc_ops actuator_read_vcm_eeprom_fops = {
		//.owner = THIS_MODULE,
		.proc_open = actuator_read_vcm_eeprom_open,
		.proc_read = seq_read,
		.proc_lseek = seq_lseek,
		.proc_release = single_release,
};


static int actuator_read_vcm_acc_offset_read(struct seq_file *buf, void *v)
{
	uint32_t reg_val = 0;
	int rc, i, check_data_fail=0;

	uint32_t reg_val_L, reg_val_U;

	int32_t FC_ave = 0, reg_val_FC = 0;

	#if defined ASUS_AI2202_PROJECT
        actuator_ctrl = g_actuator_ctrl[0];
        #endif

	mutex_lock(&actuator_ctrl->actuator_mutex);

	//actuator_ctrl->io_master_info.cci_client->sid = g_slave_id;


	rc = actuator_read_byte(actuator_ctrl, 0x00, &reg_val, CAMERA_SENSOR_I2C_TYPE_BYTE);
	pr_err("Read VCM actuator data addr:0x00 data:0x%x ", reg_val);
	rc = actuator_read_byte(actuator_ctrl, 0x01, &reg_val, CAMERA_SENSOR_I2C_TYPE_BYTE);
	pr_err("Read VCM actuator data addr:0x01 data:0x%x ", reg_val);


	//Slave address(E4h) check; Read 0Fh bit3-0 = 0001
    rc = actuator_read_byte(actuator_ctrl, 0x0F, &reg_val, CAMERA_SENSOR_I2C_TYPE_BYTE);
    if((reg_val&0x01)!=1)
    {
		pr_err("VCM NG fail");
		seq_printf(buf,"VCM NG fail\n");
		mutex_unlock(&actuator_ctrl->actuator_mutex);
		return 0;
	}

	i=0;
	do{
		msleep(1);
		//LSI wake up or not?  Read    0Fh bit4 = 1
		rc = actuator_read_byte(actuator_ctrl, 0x0F, &reg_val, CAMERA_SENSOR_I2C_TYPE_BYTE);
		pr_err("VCM LSI wake up or not;   Read 0Fh bit4 = 1  data:0x%x", reg_val);
		i++;
	}while((reg_val & 0x0010) != 0x0010 && (i<10));


	//Current set1; Write   03h = 0x0220
	rc = actuator_write_word(actuator_ctrl, 0x03, 0x0220, CAMERA_SENSOR_I2C_TYPE_WORD);
	msleep(30);


	//Read Acceleration-Z data; Read   FCh = 0xXXXX; Repeated 10 times?
	for(i=0 ; i<16 ; i++)
	{
		rc = actuator_read_byte(actuator_ctrl, 0xFC, &reg_val_U, CAMERA_SENSOR_I2C_TYPE_BYTE);
		rc = actuator_read_byte(actuator_ctrl, 0xFD, &reg_val_L, CAMERA_SENSOR_I2C_TYPE_BYTE);
		reg_val_L = reg_val_L & 0x00FF;
		//pr_err(" reg_val_L 0x%x ", reg_val_L);

		reg_val_FC = (reg_val_U<<8|reg_val_L) & 0xFFFF;

		if (reg_val_FC & 0x8000)
		{
			FC_ave = FC_ave - (((~reg_val_FC) & 0xFFFF) + 1);
		}else
		{
				FC_ave = FC_ave + reg_val_FC;
		}

		//pr_err(" Read Z data_16:0x%x data_10:%d  reg_val_U 0x%x   reg_val_L 0x%x    FC_ave 0x%x ", reg_val_FC, reg_val_FC, reg_val_U, reg_val_L, FC_ave);
		vcm_acc_data[i]=reg_val_FC;
	}

	FC_ave = (FC_ave >> 4) & 0xFFFF;// FC_ave/16

	reg_val_U = (FC_ave >> 8) & 0xFF;
	reg_val_L = FC_ave & 0xFF;

	/* Calculate Average of Accel-Z
		Write average data to EEPROM
		(Refer to EEPROM Writing Flowchart)

		Write EEPROM 30h  <= Upper 8bit
		Write EEPROM 31h  <= Lower 8bit */


	//pr_err(" FC_ave 0x%x  reg_val_U:0x%x  reg_val_L 0x%x   ", FC_ave, reg_val_U, reg_val_L);

	i=0;
	do
	{
		msleep(1);
		rc = actuator_read_byte(actuator_ctrl, 0x60, &reg_val, CAMERA_SENSOR_I2C_TYPE_BYTE);
		//pr_err(" addr:0x60  data 0x%x   slave_addr 0x%x ", reg_val, actuator_ctrl->io_master_info.cci_client->sid);
		i++;
	}while(reg_val != 0x00 && (i < 10));
	if(i>=10)
	{
		pr_err("VCM addr:0x60  data 0x%x", reg_val);
		pr_err("VCM Gyro ACC setting fail");
		seq_printf(buf,"VCM Gyro ACC setting fail\n");
		mutex_unlock(&actuator_ctrl->actuator_mutex);
		return 0;
	}

	// Function stop
	rc = actuator_write_byte(actuator_ctrl, 0x02, 0x00, CAMERA_SENSOR_I2C_TYPE_BYTE);

	//Current Stop
	rc = actuator_write_word(actuator_ctrl, 0x03, 0x0000, CAMERA_SENSOR_I2C_TYPE_WORD);
	msleep(100);

	//IC write mode enable
	rc = actuator_write_byte(actuator_ctrl, 0x16, 0xE2, CAMERA_SENSOR_I2C_TYPE_BYTE);
	rc = actuator_write_byte(actuator_ctrl, 0x17, 0xA6, CAMERA_SENSOR_I2C_TYPE_BYTE);

	msleep(1);

	actuator_ctrl->io_master_info.cci_client->sid = g_slave_id_eeprom;

	rc = actuator_write_byte(actuator_ctrl, 0x30, reg_val_U, CAMERA_SENSOR_I2C_TYPE_BYTE);
	msleep(5);
	rc = actuator_write_byte(actuator_ctrl, 0x31, reg_val_L, CAMERA_SENSOR_I2C_TYPE_BYTE);
	msleep(5);

	rc = actuator_write_byte(actuator_ctrl, 0x109, 0x32, CAMERA_SENSOR_I2C_TYPE_BYTE);
	msleep(2);


	actuator_ctrl->io_master_info.cci_client->sid = g_slave_id;

	i=0;
	do
	{
	    msleep(1);
	    rc = actuator_read_byte(actuator_ctrl, 0x61, &reg_val, CAMERA_SENSOR_I2C_TYPE_BYTE);
	    //pr_err("  addr:0x61 data:0x%x ", reg_val);
	    i++;
	}while(!((reg_val&0x01)==0) && i<20);




	//IC write mode disable
	rc = actuator_write_byte(actuator_ctrl, 0x16, 0x00, CAMERA_SENSOR_I2C_TYPE_BYTE);
	rc = actuator_write_byte(actuator_ctrl, 0x17, 0x00, CAMERA_SENSOR_I2C_TYPE_BYTE);



	//check eeprom data
	actuator_ctrl->io_master_info.cci_client->sid = g_slave_id_eeprom;
	rc = actuator_read_byte(actuator_ctrl, 0x30, &reg_val, CAMERA_SENSOR_I2C_TYPE_BYTE);
	pr_err("VCM IC eeprom data addr:0x30 data:0x%x", reg_val);
	if (reg_val != reg_val_U)
	{
		pr_err("VCM check_data_fail reg_val:0x%x reg_val_U:0x%x", reg_val, reg_val_U);
		check_data_fail=1;
	}

	rc = actuator_read_byte(actuator_ctrl, 0x31, &reg_val, CAMERA_SENSOR_I2C_TYPE_BYTE);
	pr_err("VCM eeprom data addr:0x31 data:0x%x", reg_val);
	if (reg_val != reg_val_L)
	{
		pr_err("VCM check_data_fail reg_val:0x%x reg_val_L:0x%x", reg_val, reg_val_L);
		check_data_fail=2;
	}

	rc = actuator_read_byte(actuator_ctrl, 0x109, &reg_val, CAMERA_SENSOR_I2C_TYPE_BYTE);
	pr_err("VCM eeprom data addr:0x109 data:0x%x", reg_val);
	if (reg_val != 0x32)
	{
		pr_err("VCM check_data_fail reg_val:0x%x reg_val_L:0x%x", reg_val, reg_val_L);
		check_data_fail=3;
	}



	actuator_ctrl->io_master_info.cci_client->sid = g_slave_id;

	if (check_data_fail > 0)
	{
		pr_err("VCM Gyro ACC setting fail check_data_fail %d", check_data_fail);
		seq_printf(buf,"VCM Gyro ACC setting fail\n");
		mutex_unlock(&actuator_ctrl->actuator_mutex);
		return 0;
	}

	pr_err("VCM Gyro ACC setting OK!\n");
	seq_printf(buf,"success VCM Gyro ACC setting OK!\n");

	mutex_unlock(&actuator_ctrl->actuator_mutex);
	return 0;

}



static int actuator_read_vcm_acc_offset_open(struct inode *inode, struct  file *file)
{
	return single_open(file, actuator_read_vcm_acc_offset_read, NULL);
}

static const struct proc_ops actuator_read_vcm_acc_offset_fops = {
		//.owner = THIS_MODULE,
		.proc_open = actuator_read_vcm_acc_offset_open,
		.proc_read = seq_read,
		.proc_lseek = seq_lseek,
		.proc_release = single_release,
};




static int actuator_acc_data_read(struct seq_file *buf, void *v)
{

	seq_printf(buf,"0x%04X  0x%04X  0x%04X  0x%04X  0x%04X  0x%04X  0x%04X  0x%04X\n",
		vcm_acc_data[0], vcm_acc_data[1], vcm_acc_data[2], vcm_acc_data[3],
		vcm_acc_data[4], vcm_acc_data[5], vcm_acc_data[6], vcm_acc_data[7]);
	seq_printf(buf,"0x%04X  0x%04X  0x%04X  0x%04X  0x%04X  0x%04X  0x%04X  0x%04X\n",
		vcm_acc_data[8], vcm_acc_data[9], vcm_acc_data[10], vcm_acc_data[11],
		vcm_acc_data[12], vcm_acc_data[13], vcm_acc_data[14], vcm_acc_data[15]);

	return 0;

}


static int actuator_acc_data_open(struct inode *inode, struct  file *file)
{
	return single_open(file, actuator_acc_data_read, NULL);
}

static const struct proc_ops actuator_acc_data_fops = {
		//.owner = THIS_MODULE,
		.proc_open = actuator_acc_data_open,
		.proc_read = seq_read,
		.proc_lseek = seq_lseek,
		.proc_release = single_release,
};



static int actuator_IC_read(struct seq_file *buf, void *v)
{

	uint32_t reg_data;
	int rc;

	mutex_lock(&actuator_ctrl->actuator_mutex);
	actuator_ctrl->io_master_info.cci_client->sid = g_slave_id_eeprom;

	rc = actuator_read_byte(actuator_ctrl, g_reg_addr, &reg_data, CAMERA_SENSOR_I2C_TYPE_BYTE);
	pr_err("VCM eeprom data addr:0x%x data:0x%x", g_reg_addr, reg_data);

	actuator_ctrl->io_master_info.cci_client->sid = g_slave_id;

	mutex_unlock(&actuator_ctrl->actuator_mutex);

	seq_printf(buf,"addr:0x%x data:0x%x\n", g_reg_addr, reg_data);

	return 0;

}



static ssize_t actuator_IC_write(struct file *dev, const char *buf, size_t len, loff_t *ppos)
{
	ssize_t ret_len;
	int n;
	char messages[32]="";
	uint32_t val[5];
	int rc;
	uint32_t reg_addr, reg_data, reg_val;


	ret_len = len;
	if (len > 32) {
		len = 32;
	}
	if (copy_from_user(messages, buf, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	n = sscanf(messages,"%x %x", &val[0], &val[1]);

	pr_err("VCM eeprom data %d\n", n);

	mutex_lock(&actuator_ctrl->actuator_mutex);

	if (n==2)
	{

		// Function stop
		rc = actuator_write_byte(actuator_ctrl, 0x02, 0x00, CAMERA_SENSOR_I2C_TYPE_BYTE);

		//Current Stop
		rc = actuator_write_word(actuator_ctrl, 0x03, 0x0000, CAMERA_SENSOR_I2C_TYPE_WORD);
		msleep(100);

		//IC write mode enable
		rc = actuator_write_byte(actuator_ctrl, 0x16, 0xE2, CAMERA_SENSOR_I2C_TYPE_BYTE);
		rc = actuator_write_byte(actuator_ctrl, 0x17, 0xA7, CAMERA_SENSOR_I2C_TYPE_BYTE);

		msleep(1);

		actuator_ctrl->io_master_info.cci_client->sid = g_slave_id_eeprom;
		reg_addr = val[0];
		reg_data = val[1];

		rc = actuator_write_byte(actuator_ctrl, reg_addr, reg_data, CAMERA_SENSOR_I2C_TYPE_BYTE);
		msleep(20);

		rc = actuator_read_byte(actuator_ctrl, reg_addr, &reg_val, CAMERA_SENSOR_I2C_TYPE_BYTE);
		pr_err("VCM eeprom data addr:0x%x data:0x%x",reg_addr , reg_val);

		if (reg_data != reg_val)
			pr_err("VCM eeprom data write check FAIL\n");
		else
			pr_err("VCM eeprom data write check PASS\n");


		actuator_ctrl->io_master_info.cci_client->sid = g_slave_id;

	}
	else if(n==1)
	{
		g_reg_addr = val[0];
	}

	mutex_unlock(&actuator_ctrl->actuator_mutex);

	return ret_len;

}


static int actuator_IC_open(struct inode *inode, struct  file *file)
{
	return single_open(file, actuator_IC_read, NULL);
}


static const struct proc_ops actuator_IC_RW_fops = {
		//.owner = THIS_MODULE,
		.proc_open = actuator_IC_open,
		.proc_read = seq_read,
		.proc_write = actuator_IC_write,
		.proc_lseek = seq_lseek,
		.proc_release = single_release,
};

static int ois_actuator_power_read(struct seq_file *buf, void *v)
{
    mutex_lock(&actuator_ctrl->actuator_mutex);
    pr_info("g_actuator_power_state = %d\n", g_actuator_power_state);
	seq_printf(buf,"%d\n",g_actuator_power_state);
	mutex_unlock(&actuator_ctrl->actuator_mutex);
	return 0;
}

static int actuator_solo_power_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ois_actuator_power_read, NULL);
}

//just for ATD test
static ssize_t actuator_solo_power_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t ret_len;
	char messages[16]="";
	int val;
	int rc;
	int i;
	struct cam_hw_soc_info         *soc_info = &actuator_ctrl->soc_info;
	struct cam_actuator_soc_private     *soc_private =
		(struct cam_actuator_soc_private *)actuator_ctrl->soc_info.soc_private;

	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;

	ret_len = len;
	if (len > 16) {
		len = 16;
	}
	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages,"%d",&val);
	mutex_lock(&actuator_ctrl->actuator_mutex);
	if(g_actuator_camera_open == 0)
	{
		if(val == 0)
		{
			if(g_actuator_power_state == 1)
			{
				camera_io_release(&(actuator_ctrl->io_master_info));
				rc = cam_sensor_util_power_down(power_info, soc_info);
				if (rc) {
					pr_err("%s: msm_camera_power_down fail rc = %d\n", __func__, rc);
				}
				else
				{
					g_actuator_power_state = 0;
					pr_info("ACTUATOR POWER DOWN\n");
				}
			}
			else
			{
				pr_info("ACTUATOR already power off, do nothing\n");
			}
		}
		else
		{
			if(g_actuator_power_state == 0)
			{
				/* Get Clock */
				for (i = 0; i < soc_info->num_clk; i++) {
					soc_info->clk[i] = clk_get(soc_info->dev,
						soc_info->clk_name[i]);
					if (!soc_info->clk[i]) {
						CAM_ERR(CAM_UTIL, "get failed for %s",
							soc_info->clk_name[i]);
					}
				}
				rc = cam_sensor_core_power_up(power_info, soc_info);
				if (rc) {
					pr_err("%s: msm_camera_power_up fail rc = %d\n", __func__, rc);
				}
				else
				{
					g_actuator_power_state = 1;
					camera_io_init(&(actuator_ctrl->io_master_info));
					//rc = onsemi_actuator_init_setting(actuator_ctrl); //tmp disable
					if(rc < 0)
					{
					  pr_err("%s: onsemi_actuator_init_setting failed rc = %d", __func__, rc);
					}
					pr_info("ACTUATOR POWER UP\n");
				}
			}
			else
			{
				pr_info("ACTUATOR already power up, do nothing\n");
			}
		}
	}
	else
	{
		pr_err("camera has been opened, can't control actuator power\n");
	}
	mutex_unlock(&actuator_ctrl->actuator_mutex);
	return ret_len;
}
static const struct proc_ops actuator_solo_power_fops = {
	//.owner = THIS_MODULE,
	.proc_open = actuator_solo_power_open,
	.proc_read = seq_read,
	.proc_write = actuator_solo_power_write,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};


static int gyro_actuator_init_read(struct seq_file *buf, void *v)
{
    mutex_lock(&actuator_ctrl->actuator_mutex);
    pr_info("g_actuator_gyro_init_state = %d\n", g_actuator_gyro_init_state);
	seq_printf(buf,"%d\n",g_actuator_gyro_init_state);
	mutex_unlock(&actuator_ctrl->actuator_mutex);
	return 0;
}

static int actuator_gyro_init_open(struct inode *inode, struct  file *file)
{
	return single_open(file, gyro_actuator_init_read, NULL);
}

static const struct proc_ops actuator_gyro_init_fops = {
	//.owner = THIS_MODULE,
	.proc_open = actuator_gyro_init_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};




#if 0
int actuator_power_up(struct cam_actuator_ctrl_t *actuator_ctrl)
{

	int rc;
	struct cam_hw_soc_info         *soc_info = &actuator_ctrl->soc_info;
	struct cam_actuator_soc_private     *soc_private =
		(struct cam_actuator_soc_private *)actuator_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;


	rc = cam_sensor_core_power_up(power_info, soc_info);
	if (rc) {
		pr_err("actuator power up failed, rc %d\n", rc);
		return -1;
	}
	if (actuator_ctrl->io_master_info.master_type == CCI_MASTER)
	{
		rc = camera_io_init(&(actuator_ctrl->io_master_info));
		if (rc < 0) {
			pr_err("cci init failed!\n");
			rc = cam_sensor_util_power_down(power_info, soc_info);
			if (rc) {
				pr_err("actuator power down failed, rc %d\n", rc);
			}
			return -2;
		}
	}
	pr_err("Actuator probe power up!");
	return rc;
}

int actuator_power_down(struct cam_actuator_ctrl_t *actuator_ctrl)
{

	int rc;
	struct cam_hw_soc_info         *soc_info = &actuator_ctrl->soc_info;
	struct cam_actuator_soc_private     *soc_private =
		(struct cam_actuator_soc_private *)actuator_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;


    pr_err("actuator_power_down :E");
	if(actuator_ctrl->io_master_info.master_type == CCI_MASTER)
	{
		rc = camera_io_release(&(actuator_ctrl->io_master_info));
		if (rc < 0)
			pr_err("cci release failed!\n");
	}

	rc = cam_sensor_util_power_down(power_info, soc_info);
	if (rc) {
		pr_err("actuator power down failed, rc %d\n", rc);
	}
	pr_err("Actuator probe power down!");
	return rc;
}

void actuator_probe_check(void)
{
	if(actuator_ctrl == NULL)
	{
		pr_err("actuator_ctrl is NULL!!!\n");
		return;
	}
	if(actuator_power_up(actuator_ctrl) != 0)
	{
		pr_err("actuator power up failed\n");
		return;
	}

	mdelay(1000);

	actuator_power_down(actuator_ctrl);
}

uint8_t asus_allow_vcm_move(void)
{
	return g_vcm_enabled;
}

#endif

static void create_actuator_proc_files_factory(int index)
{
	static uint8_t has_created = 0;
	char actuator_proc_file[32]="";

	if(!has_created)
	{
		create_proc_file(PROC_POWER, &actuator_solo_power_fops);
		create_proc_file(PROC_I2C_RW,&actuator_i2c_debug_fops);//ATD

		create_proc_file(PROC_VCM_ENABLE, &actuator_allow_vcm_move_fops);
		#if defined ASUS_AI2201_PROJECT
		sprintf(actuator_proc_file,"%s%d",PROC_VCM_VALUE,index);
		create_proc_file(actuator_proc_file, &actuator_read_vcm_dac_fops);
		#endif
		create_proc_file(PROC_VCM_EEPROM, &actuator_read_vcm_eeprom_fops);
		create_proc_file(PROC_VCM_ACC_OFFSET, &actuator_read_vcm_acc_offset_fops);
		create_proc_file(PROC_VCM_IC_RW, &actuator_IC_RW_fops);
		create_proc_file(PROC_VCM_GYRO_INIT, &actuator_gyro_init_fops);
		create_proc_file(PROC_VCM_ACC_DATA, &actuator_acc_data_fops);

	#if 0
		//create_proc_file(PROC_DSI_CHECK, &cam_csi_check_fops);  //ASUS_BSP Bryant "Add for camera csi debug"
	#endif
		has_created = 1;
	}
	else
	{
		pr_err("ACTUATOR factory proc files have already created!\n");
	}
#if defined ASUS_AI2202_PROJECT
	//ASUS_BSP for jason_yeh +++ support multi camera vcm

	sprintf(actuator_proc_file,"%s%d",PROC_VCM_VALUE,index);
	pr_info("ACTUATOR factory proc=%s\n",actuator_proc_file);
	create_proc_file(actuator_proc_file, &actuator_read_vcm_dac_fops);
	//ASUS_BSP for jason_yeh --- support multi camera vcm
#endif
}

void asus_actuator_init(struct cam_actuator_ctrl_t * ctrl)
{
	if(ctrl)
	{
		actuator_ctrl = ctrl;
		#if defined ASUS_AI2202_PROJECT
		g_actuator_ctrl[ctrl->soc_info.index]= ctrl;
		pr_info("%s g_actuator_ctrl=%x index=%d \n",__func__,g_actuator_ctrl[ctrl->soc_info.index],ctrl->soc_info.index);
		#endif
	}
	else
	{
		pr_err("actuator_ctrl_t passed in is NULL!\n");
		return;
	}
	pr_err("asus_actuator_init :E create_actuator_proc_files_factory %d\n",ctrl->soc_info.index);
	create_actuator_proc_files_factory(ctrl->soc_info.index);
	mutex_init(&g_busy_job_mutex);
}
