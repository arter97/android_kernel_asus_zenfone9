#include "asus_flash.h"
#include "cam_flash_core.h"
#include <linux/proc_fs.h>

#undef  pr_fmt
#define pr_fmt(fmt) "FLASH-ATD %s() " fmt, __func__

#define PROC_ATD_FLASH1 "driver/asus_flash"
#define PROC_ATD_FLASH2 "driver/asus_flash2"

#define PROC_ATD_STATUS "driver/flash_status"

#define PROC_CTRL1_LEDS "driver/ctrl1_leds"

#define BATTERY_CAPACITY "/sys/class/power_supply/bms/capacity"
#define LOW_BATTERY_THRESHOLD 20

#define MAX_CTRL 4

#define ATD_TORCH_CURRENT 200
#define ATD_FLASH_CURRENT 1000

typedef struct
{
	int (*led_init)(uint32_t id);
	int (*led_release)(uint32_t id);
	int (*led_off)(uint32_t id);
	int (*led_high)(uint32_t id, uint32_t current1, uint32_t current2);
	int (*led_low)(uint32_t id, uint32_t current1, uint32_t current2);
}asus_flash_function_table_t;

static asus_flash_function_table_t g_func_tbl;
static struct cam_flash_ctrl *g_fctrl[MAX_CTRL];//ctrl for flash
static uint32_t g_flash_index[MAX_CTRL] = {0,1,2,3}; //fctrl's soc_info.index global table(mapping to dtsi's cell-index)
static uint32_t g_flash_mapping[MAX_CTRL] = {0,MAX_CTRL,MAX_CTRL,MAX_CTRL};// for mapping fctrl MAX_CTRL:not support 0:led1 1:led2
static uint8_t g_ATD_status = 0;

static int g_error_value = 0;
static uint8_t g_camera_in_use = 0;//used to avoid control led when camera open

static const char * get_state_string(enum cam_flash_state state)
{
	switch(state)
	{
		case CAM_FLASH_STATE_INIT:
			 return "INIT";
		case CAM_FLASH_STATE_ACQUIRE:
			 return "ACQUIRE";
		case CAM_FLASH_STATE_CONFIG:
			 return "CONFIG";
		case CAM_FLASH_STATE_START:
			 return "START";
		default:
			 return "UNKNOWN";
	}
}

static int asus_led_init(uint32_t id)
{
	int rc = 0;
	struct cam_flash_ctrl *fctrl;

	if(g_flash_index[id] >= MAX_CTRL)
	{
		pr_err("invalid flash id %d!\n",id);
		return -EINVAL;
	}

	fctrl = g_fctrl[id];
	if(fctrl == NULL)
	{
		pr_err("fctrl for id %d is NULL!\n",id);
		return -EINVAL;
	}

	mutex_lock(&(fctrl->flash_mutex));
	if(!fctrl->is_regulator_enabled)
	{
		if (fctrl->func_tbl.power_ops) {
			rc = fctrl->func_tbl.power_ops(fctrl, true);
			if (rc)
				CAM_ERR(CAM_FLASH, "Power Down Failed rc: %d", rc);
		}
	}
	else
	{
		pr_info("regulator already enabled, do nothing\n");
	}
	mutex_unlock(&(fctrl->flash_mutex));
	return rc;
}

static int asus_led_release(uint32_t id)
{
	int rc = 0;
	struct cam_flash_ctrl *fctrl;

	if(g_flash_index[id] >= MAX_CTRL)
	{
		pr_err("invalid flash id %d!\n",id);
		return -EINVAL;
	}

	fctrl = g_fctrl[id];
	if(fctrl == NULL)
	{
		pr_err("fctrl for id %d is NULL!\n",id);
		return -EINVAL;
	}

	mutex_lock(&(fctrl->flash_mutex));
	if(fctrl->is_regulator_enabled)
	{
		if (fctrl->func_tbl.power_ops) {
			rc = fctrl->func_tbl.power_ops(fctrl, false);
			if (rc)
				CAM_ERR(CAM_FLASH, "Power Down Failed rc: %d", rc);
		}
	}
	else
	{
		pr_info("regulator already disabled, do nothing\n");
	}
	mutex_unlock(&(fctrl->flash_mutex));
	return rc;
}

static int asus_led_off(uint32_t id)
{
	int rc = 0;
	struct cam_flash_ctrl *fctrl;

	if(g_flash_index[id] >= MAX_CTRL)
	{
		pr_err("invalid flash id %d!\n",id);
		return -EINVAL;
	}

	fctrl = g_fctrl[id];
	if(fctrl == NULL)
	{
		pr_err("fctrl for id %d is NULL!\n",id);
		return -EINVAL;
	}

	mutex_lock(&(fctrl->flash_mutex));


	if(fctrl->is_regulator_enabled || fctrl->func_tbl.power_ops == NULL)
	{
		rc = cam_flash_off(fctrl);
	}
	else
	{
		pr_info("flash state is %d -> %s, regulator disabled, not call off\n",
				fctrl->flash_state,get_state_string(fctrl->flash_state));
		rc = -1;
	}

	mutex_unlock(&(fctrl->flash_mutex));
	return rc;
}

static int asus_led_high(uint32_t id, uint32_t current1, uint32_t current2)
{
	int rc = 0;
	struct cam_flash_ctrl *fctrl;
	struct cam_flash_frame_setting setting;

	if(g_flash_index[id] >= MAX_CTRL)
	{
		pr_err("invalid flash id %d!\n",id);
		return -EINVAL;
	}

	fctrl = g_fctrl[id];
	if(fctrl == NULL)
	{
		pr_err("fctrl for id %d is NULL!\n",id);
		return -EINVAL;
	}

	pr_info("fctrl for id %d current1 %d current2 %d",id,current1,current2);

	setting.led_current_ma[0] = current1;
	setting.led_current_ma[1] = current2;


	mutex_lock(&(fctrl->flash_mutex));

	if(fctrl->is_regulator_enabled)
	{
		rc = cam_flash_high(fctrl,&setting);
	}
	else if (fctrl->func_tbl.power_ops == NULL)
	{
		pr_info("fctrl->func_tbl.power_ops is NULL!\n");
		rc = cam_flash_high(fctrl,&setting);
	}
	else
	{
		pr_err("flash state is %d -> %s, regulator disabled, not call high asus_led_high fail\n",
				fctrl->flash_state,get_state_string(fctrl->flash_state));
		rc = -1;
	}

	mutex_unlock(&(fctrl->flash_mutex));
	return rc;
}

static int asus_led_low(uint32_t id, uint32_t current1, uint32_t current2)
{
	int rc = 0;
	struct cam_flash_ctrl *fctrl;
	struct cam_flash_frame_setting setting;

	if(g_flash_index[id] >= MAX_CTRL)
	{
		pr_err("invalid flash id %d!\n",id);
		return -EINVAL;
	}

	fctrl = g_fctrl[id];
	if(fctrl == NULL)
	{
		pr_err("fctrl for id %d is NULL!\n",id);
		return -EINVAL;
	}

	pr_info("fctrl for id %d current1 %d current2 %d",id,current1,current2);

	setting.led_current_ma[0] = current1;
	setting.led_current_ma[1] = current2;


	mutex_lock(&(fctrl->flash_mutex));


	if(!fctrl->is_regulator_enabled)
	{
		rc = cam_flash_low(fctrl,&setting);
	}
	else if (fctrl->func_tbl.power_ops == NULL)
	{
		pr_info("fctrl->func_tbl.power_ops is NULL!\n");
		rc = cam_flash_low(fctrl,&setting);
	}
	else
	{
		pr_err("flash state is %d -> %s, regulator disabled, not call high asus_led_low fail\n",
				fctrl->flash_state,get_state_string(fctrl->flash_state));
		rc = -1;
	}


	mutex_unlock(&(fctrl->flash_mutex));
	return rc;
}

void asus_flash_set_camera_state(uint8_t in_use)
{
	g_camera_in_use = in_use;
	pr_info("Camera use flash %d\n",g_camera_in_use);
}

void asus_flash_set_led_fault(int error_value)
{
	g_error_value = error_value;
	pr_err("ERROR occured! value is 0x%x -> %d\n",g_error_value,g_error_value);
}

static ssize_t status_show(struct file *dev, char *buffer, size_t count, loff_t *ppos)
{
	int n;
	char kbuf[8];

	if(*ppos == 0)
	{
		n = snprintf(kbuf, sizeof(kbuf),"%d\n%c", g_ATD_status,'\0');
		if(copy_to_user(buffer,kbuf,n))
		{
			pr_err("%s(): copy_to_user fail !\n",__func__);
			return -EFAULT;
		}
		*ppos += n;
		g_ATD_status = 0;
		return n;
	}
	return 0;
}

static ssize_t flash_led_show(struct file *dev, char *buffer, size_t count, loff_t *ppos)
{
	ssize_t ret=0;
	char buff[1024];
	ssize_t desc = 0;

	struct cam_flash_ctrl *fctrl;
	struct cam_flash_private_soc *soc_private;

	uint32_t id = *((uint32_t*)PDE_DATA(file_inode(dev)));

	if(g_flash_mapping[id] == 0 || g_flash_mapping[id] == 1)
	{
		fctrl = g_fctrl[id];
	}
	else
		fctrl = NULL;

	if(fctrl == NULL)
	{
		pr_err("fctrl for led index %d is NULL!\n",id);
		return 0;
	}

	soc_private = (struct cam_flash_private_soc *)fctrl->soc_info.soc_private;
	desc+=sprintf(buff+desc, "flash_soc_info.index: %d\n",fctrl->soc_info.index);
	desc+=sprintf(buff+desc, "flash_num_sources: %d\n",fctrl->flash_num_sources);
	desc+=sprintf(buff+desc, "torch_num_sources: %d\n",fctrl->torch_num_sources);
	desc+=sprintf(buff+desc, "flash_state: %d %s\n",fctrl->flash_state, get_state_string(fctrl->flash_state));

	desc+=sprintf(buff+desc, "switch_trigger_name: %s\n", soc_private->switch_trigger_name);
	desc+=sprintf(buff+desc, "flash_trigger_name: %s\n", soc_private->flash_trigger_name[id]);
	desc+=sprintf(buff+desc, "flash_op_current: %d\n",soc_private->flash_op_current[id]);
	desc+=sprintf(buff+desc, "flash_max_current: %d\n",soc_private->flash_max_current[id]);
	desc+=sprintf(buff+desc, "flash_max_duration: %d\n",soc_private->flash_max_duration[id]);
	desc+=sprintf(buff+desc, "torch_trigger_name: %s\n", soc_private->torch_trigger_name[id]);
	desc+=sprintf(buff+desc, "torch_op_current: %d\n",soc_private->torch_op_current[id]);
	desc+=sprintf(buff+desc, "torch_max_current: %d\n",soc_private->torch_max_current[id]);

	ret = simple_read_from_buffer(buffer,count,ppos,buff,desc);
	return ret;
}

static ssize_t flash_led_store(struct file *dev, const char *buf, size_t count, loff_t *loff)
{
	char kbuf[8];
	int  mode = -1;
	int  on = -1;
	int  rc = 0;
	int  val = 0;

	ssize_t ret_len = count;

	uint32_t id = *((uint32_t*)PDE_DATA(file_inode(dev)));

	if(count > 8)
		count = 8;

	if(copy_from_user(kbuf, buf, count))
	{
		pr_err("%s(): copy_from_user fail !\n",__func__);
		return -EFAULT;
	}

	rc=sscanf(kbuf, "%d %d %d", &mode, &on, &val);

	pr_info("id %d, command %d %d %d rc=%d\n",id, mode, on, val, rc);

	g_error_value = 0;
	g_ATD_status = 0;

	g_func_tbl.led_init(id);

	if(mode == 0)//torch
	{
		if(on == 1)
		{
			val = val==0 ? ATD_TORCH_CURRENT : val;
			val = val > ATD_TORCH_CURRENT ? ATD_TORCH_CURRENT : val;
			g_func_tbl.led_off(id);//prevent improper command seqence

			if(g_flash_mapping[id] == 0)
				rc = g_func_tbl.led_low(id,val,0);
			else if(g_flash_mapping[id] == 1)
				rc = g_func_tbl.led_low(id,0,val);
			else
				rc = g_func_tbl.led_low(id,val,val);
		}
		else if(on == 0)
		{
			rc = g_func_tbl.led_off(id);
		}
		else
		{
			pr_err("invalid command value %d, should be 0-OFF, 1-ON\n",on);
			rc = -1;
		}
	}
	else if(mode == 1)//flash
	{
		if(on == 1)
		{
			val = val==0 ? ATD_FLASH_CURRENT : val;
			val = val > ATD_FLASH_CURRENT ? ATD_FLASH_CURRENT : val;
			g_func_tbl.led_off(id);//prevent improper command seqence
			if(g_flash_mapping[id] == 0)
				rc = g_func_tbl.led_high(id,val,0);
			else if(g_flash_mapping[id] == 1)
				rc = g_func_tbl.led_high(id,0,val);
			else
				rc = g_func_tbl.led_high(id,val,val);
		}
		else if(on == 0)
		{
			rc = g_func_tbl.led_off(id);
		}
		else
		{
			pr_err("invalid command value %d, should be 0-OFF, 1-ON\n",on);
			rc = -1;
		}
	}
	else if(mode == 2)//low battery control
	{
		cam_flash_battery_low(on);
	}
	else
	{
		pr_err("invalid command type %d, should be 0 - Torch, 1 - Flash\n",mode);
		rc = -1;
	}

	if(rc < 0)
	{
		g_func_tbl.led_off(id);
		g_func_tbl.led_release(id);
		g_ATD_status = 0;//failure
	}
	else
	{
		if(on == 0 )//torch off or flash off
		{
			g_func_tbl.led_release(id);
			g_ATD_status = 1;
		}
		else
		{
			usleep_range(5*1000,6*1000);
			g_ATD_status = (g_error_value == 0);
			if(g_ATD_status == 0)
			{
				pr_err("Fault detected! Turn off and Release\n");
				g_func_tbl.led_off(id);
				g_func_tbl.led_release(id);
			}
		}
	}
	pr_info("id %d, command %d %d, rc %d, ATD status %d\n",id, mode, on, rc, g_ATD_status);

	return ret_len;
}

static ssize_t flash_leds_store(struct file *dev, const char *buf, size_t count, loff_t *loff)
{
	char kbuf[16];
	int  rc = 0;

	int  mode;
	int  mA[2] = {0,0};
	int  val[3];
	int  n, i;

	ssize_t ret_len = count;
	uint32_t id = MAX_CTRL;
	for(i = 0;i < MAX_CTRL; i++)
	{
		if(g_flash_mapping[i] == 2)
		{
			id = i;
			break;
		}
		else if(g_flash_mapping[i] == 0)
		{
			id = i;
		}
	}

	if(id == MAX_CTRL)
	{
		pr_err("invalid flash id %d!\n",id);
	}
	if(count > 16)
		count = 16;

	if(copy_from_user(kbuf, buf, count))
	{
		pr_err("%s(): copy_from_user fail !\n",__func__);
		return -EFAULT;
	}

	n = sscanf(kbuf, "%d %d %d", &val[0], &val[1], &val[2]);
	if(n < 1)
	{
		pr_err("need at least one argument!\n");
		return ret_len;
	}
	else if(n == 1)
	{
		mode = val[0];
		if(mode == 0)//torch
		{
			mA[0] = ATD_TORCH_CURRENT;//default current
			mA[1] = ATD_TORCH_CURRENT;//default current
		}
		else if(mode == 1)
		{
			mA[0] = ATD_FLASH_CURRENT;//default current
			mA[1] = ATD_FLASH_CURRENT;//default current
		}
	}
	else if(n == 2)
	{
		mode = val[0];
		mA[0] = (val[1] > 0) ? val[1] : 0;
		mA[1] = mA[0];
	}
	else
	{
		mode = val[0];
		mA[0] = (val[1] > 0) ? val[1] : 0;
		mA[1] = (val[2] > 0) ? val[2] : 0;
	}

	if(mode != 0 && mode != 1)
	{
		pr_err("mode %d not valid! should be 0-Torch, 1-Flash\n",mode);
		return ret_len;
	}

	pr_info("node %d, command mode %d, current[%d %d]\n",id, mode, mA[0], mA[1]);

	g_error_value = 0;
	g_ATD_status = 0;

	g_func_tbl.led_init(id);

	if((mA[0] == 0 && mA[1] == 0) || (g_flash_mapping[id] == 0 && mA[0] == 0) || (g_flash_mapping[id] == 1 && mA[1] == 0))
	{
		rc = g_func_tbl.led_off(id);
	}
	else
	{
		if(mode == 0)//torch
		{
			g_func_tbl.led_off(id);//turn off to show effect
			rc = g_func_tbl.led_low(id,mA[0],mA[1]);
		}
		else if(mode == 1)//flash
		{
			g_func_tbl.led_off(id);//turn off to show effect
			rc = g_func_tbl.led_high(id,mA[0],mA[1]);
		}
	}

	if(rc < 0)
	{
		g_func_tbl.led_off(id);
		g_func_tbl.led_release(id);
		g_ATD_status = 0;//failure
	}
	else
	{
		if((mA[0] == 0 && mA[1] == 0) || (g_flash_mapping[id] == 0 && mA[0] == 0) || (g_flash_mapping[id] == 1 && mA[1] == 0))
		{
			g_func_tbl.led_release(id);
			g_ATD_status = 1;
		}
		else
		{
			usleep_range(5*1000,6*1000);
			g_ATD_status = (g_error_value == 0);
			if(g_ATD_status == 0)
			{
				pr_err("Fault detected! Turn off and Release\n");
				g_func_tbl.led_off(id);
				g_func_tbl.led_release(id);
			}
		}
	}
	pr_info("node %d, command mode %d, current [%d %d], rc %d\n",id, mode, mA[0], mA[1], rc);

	return ret_len;
}

static const struct proc_ops flash_leds_proc_fops = {
       .proc_read = NULL,
       .proc_write = flash_leds_store,
};

static const struct proc_ops flash_led_proc_fops = {
       .proc_read = flash_led_show,
       .proc_write = flash_led_store,
};

static const struct proc_ops status_proc_fops = {
       .proc_read = status_show,
       .proc_write = NULL,
};

static void create_proc_file(const char *PATH,const struct proc_ops* f_ops, void *data)
{
	struct proc_dir_entry *pde;

	pde = proc_create_data(PATH, 0666, NULL, f_ops, data);
	if(pde)
	{
		pr_info("create(%s) done\n",PATH);
	}
	else
	{
		pr_err("create(%s) failed!\n",PATH);
	}
}

static void create_flash_proc_files(uint32_t id)
{
	if(g_flash_mapping[id] == 0)
	{
		create_proc_file(PROC_ATD_STATUS, &status_proc_fops, NULL);
		create_proc_file(PROC_ATD_FLASH1, &flash_led_proc_fops, &g_flash_index[id]);
		create_proc_file(PROC_CTRL1_LEDS, &flash_leds_proc_fops, NULL);
	}
	else if(g_flash_mapping[id] == 1)
	{
		create_proc_file(PROC_ATD_FLASH2, &flash_led_proc_fops, &g_flash_index[id]);
	}
	else
		pr_info("Unsupported flash id %d! not create any node\n",id);
}

void asus_flash_init(struct cam_flash_ctrl * fctrl)
{
	uint32_t id;

	if(fctrl)
	{
		id = fctrl->soc_info.index;
		if(g_flash_index[id] >= MAX_CTRL)
		{
			pr_err("invalid flash id %d\n",id);
			return;
		}

		if(g_fctrl[id] == NULL)
		{
			g_fctrl[id] = fctrl;
			g_func_tbl.led_init = asus_led_init;
			g_func_tbl.led_release = asus_led_release;
			g_func_tbl.led_off = asus_led_off;
			g_func_tbl.led_high = asus_led_high;
			g_func_tbl.led_low = asus_led_low;

			create_flash_proc_files(id);
		}
		else
		{
			pr_err("flash ctrl for id %d already inited!\n",id);
		}
	}
	else
	{
		pr_err("flash ctrl passed in is NULL!\n");
	}
}
