/*
 * Copyright (c) 2021, ASUS. All rights reserved.
 */

#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/syscalls.h>

#include "dsi_ai2202.h"

#if defined ASUS_AI2202_PROJECT
#include <sde_encoder_phys.h>

static char dsi_display_unique_id[MAX_CMDLINE_PARAM_LEN];
struct dsi_display *g_display;
char g_reg_buffer[REG_BUF_SIZE];
int display_commit_cnt = COMMIT_FRAMES_COUNT;
static int g_hdr = 0;
// DC mode
bool dc_fixed_bl;

/**
 * set_tcon_cmd()
 * @cmd: Panel command to write
 */
static void set_tcon_cmd(char *cmd, short len)
{
	int i = 0;
	ssize_t rc;
	struct dsi_cmd_desc cmds;
	struct mipi_dsi_msg tcon_cmd = {0, 0x15, 0, 0, 0, len, cmd, 0, NULL};
	const struct mipi_dsi_host_ops *ops = g_display->panel->host->ops;

	if (!g_display || !g_display->panel) {
		DSI_LOG("Invalid params\n");
		return;
	}

	for(i=0; i<1; i++)
		DSI_LOG("cmd[%d] = 0x%02x\n", i, cmd[i]);

	if(len > 2)
		tcon_cmd.type = 0x39;

	mutex_lock(&g_display->display_lock);
	dsi_panel_acquire_panel_lock(g_display->panel);

	if (g_display->ctrl[0].ctrl->current_state.controller_state == DSI_CTRL_ENGINE_ON) {
		// cmds assigned
		cmds.msg = tcon_cmd;
		cmds.last_command = 1;
		cmds.post_wait_ms = 1;
		if (cmds.last_command) {
			cmds.msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;
		}

		rc = ops->transfer(g_display->panel->host, &cmds.msg);
		if (rc < 0) {
			DSI_LOG("tx cmd transfer failed rc=%d\n", rc);
		}
	} else {
		DSI_LOG("DSI_CTRL_ENGINE is off\n");
	}

	dsi_panel_release_panel_lock(g_display->panel);
	mutex_unlock(&g_display->display_lock);
}

/**
 * get_tcon_cmd()
 * @cmd: Panel command to read
 * @g_reg_buffer: To store return buffer
 */
static void get_tcon_cmd(char cmd, int rlen)
{
	char tmp[256];
	int i = 0, rc = 0, start = 0;
	u8 *tx_buf, *return_buf, *status_buf;

	struct dsi_cmd_desc cmds;
	struct mipi_dsi_msg tcon_cmd = {0, 0x06, 0, 0, 0, sizeof(cmd), NULL, rlen, NULL};
	const struct mipi_dsi_host_ops *ops = g_display->panel->host->ops;

	if (!g_display || !g_display->panel) {
		DSI_LOG("Invalid params\n");
		return;
	}

	mutex_lock(&g_display->display_lock);
	dsi_panel_acquire_panel_lock(g_display->panel);

	if (g_display->ctrl[0].ctrl->current_state.controller_state == DSI_CTRL_ENGINE_ON) {
		// buffer assigned
		tx_buf = &cmd;
		return_buf = kcalloc(rlen, sizeof(unsigned char), GFP_KERNEL);
		status_buf = kzalloc(SZ_4K, GFP_KERNEL);
		memset(status_buf, 0x0, SZ_4K);

		// cmds assigned
		cmds.msg = tcon_cmd;
		cmds.last_command = 1;
		cmds.post_wait_ms = 0;
		if (cmds.last_command) {
			cmds.msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;
		}
		cmds.msg.flags |= MIPI_DSI_MSG_CMD_READ;

		cmds.msg.tx_buf = tx_buf;
		cmds.msg.rx_buf = status_buf;
		cmds.msg.rx_len = rlen;
		memset(g_reg_buffer, 0, REG_BUF_SIZE*sizeof(char));

		rc = ops->transfer(g_display->panel->host, &cmds.msg);
		if (rc <= 0) {
			DSI_LOG("rx cmd transfer failed rc=%d\n", rc);
		} else {
			memcpy(return_buf + start, status_buf, rlen);
			start += rlen;

			for(i=0; i<rlen; i++) {
				memset(tmp, 0, 256*sizeof(char));
				snprintf(tmp, sizeof(tmp), "0x%02x = 0x%02x\n", cmd, return_buf[i]);
				strcat(g_reg_buffer,tmp);
			}
		}
	} else {
		DSI_LOG("DSI_CTRL_ENGINE is off\n");
	}

	dsi_panel_release_panel_lock(g_display->panel);
	mutex_unlock(&g_display->display_lock);
}


// panel_vendor_id_ops() - show panel vendor id
static ssize_t panel_vendor_id_show(struct file *file, char __user *buf,
				 size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	buff = kzalloc(SZ_4K, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;
	len += sprintf(buff, "%s\n", g_display->panel->panel_vendor_id);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static struct proc_ops panel_vendor_id_ops = {
	.proc_read = panel_vendor_id_show,
};

// panel uid read
static ssize_t lcd_unique_id_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	buff = kzalloc(256, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	len += sprintf(buff, "%s\n", dsi_display_unique_id);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static struct  proc_ops lcd_unique_id_ops = {
	.proc_read = lcd_unique_id_read,
};

// panel fps read
static ssize_t panel_fps_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	DSI_LOG("refreshrate  %d\n",g_display->panel->cur_mode->timing.refresh_rate);
	buff = kzalloc(256, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	len += sprintf(buff, "%d\n", g_display->panel->cur_mode->timing.refresh_rate);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static struct proc_ops panel_fps_ops = {
	.proc_read = panel_fps_read,
};


static int dsi_ai2202_tx_cmd_set(struct dsi_panel *panel,
				enum dsi_cmd_set_type type)
{
	int rc = 0, i = 0;
	ssize_t len;
	struct dsi_cmd_desc *cmds;
	u32 count;
	enum dsi_cmd_set_state state;
	struct dsi_display_mode *mode;
	const struct mipi_dsi_host_ops *ops = panel->host->ops;

	if (!panel || !panel->cur_mode)
		return -EINVAL;

	mode = panel->cur_mode;

	cmds = mode->priv_info->cmd_sets[type].cmds;
	count = mode->priv_info->cmd_sets[type].count;
	state = mode->priv_info->cmd_sets[type].state;

	if (count == 0) {
		DSI_LOG("[%s] No commands to be sent for state(%d)\n",
			 panel->name, type);
		goto error;
	}

	for (i = 0; i < count; i++) {
		if (state == DSI_CMD_SET_STATE_LP)
			cmds->msg.flags |= MIPI_DSI_MSG_USE_LPM;

		if (cmds->last_command)
			cmds->msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;

		len = ops->transfer(panel->host, &cmds->msg);
		if (len < 0) {
			rc = len;
			DSI_LOG("failed to set cmds(%d), rc=%d\n", type, rc);
			goto error;
		}
		if (cmds->post_wait_ms)
			usleep_range(cmds->post_wait_ms*1000,
					((cmds->post_wait_ms*1000)+10));
		cmds++;
	}
error:
	return rc;
}

// send Dimming Smooth command to panel
void dsi_ai2202_set_dimming_smooth(struct dsi_panel *panel, u32 backlight)
{
	int rc = 0;

	// set to 1 if set bl from FOD or DC process in kernel
	if (atomic_read(&panel->allow_bl_change)) {
		panel->panel_bl_count = 1;
		return;
	}

	if (panel->aod_state || backlight == 0) {
		panel->panel_bl_count = 0;
		return;
	}

	if (panel->panel_bl_count == 1) {
		DSI_LOG("restore dimming smooth\n");
		rc = dsi_ai2202_tx_cmd_set(panel, DSI_CMD_SET_DIMMING_SMOOTH);
		if (rc)
			DSI_LOG("[%s] failed to send DSI_CMD_SET_DIMMING_SMOOTH cmd, rc=%d\n",
				   panel->name, rc);
	}

	panel->panel_bl_count++;
}

// panel_reg_rw_ops() - read/write/show panel register
static ssize_t panel_reg_rw(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char *messages, *tmp, *cur;
	char *token, *token_par;
	char *put_cmd;
	bool flag = 0; /* w/r type : w=1, r=0 */
	int *store;
	int i = 0, cnt = 0, cmd_cnt = 0;
	int ret = 0;
	uint8_t str_len = 0;

	pr_err("[Display] : panel_reg_rw \n");
	messages = (char*) kmalloc(len*sizeof(char), GFP_KERNEL);
	if(!messages)
		return -EFAULT;

	tmp = (char*) kmalloc(len*sizeof(char), GFP_KERNEL);
	memset(tmp, 0, len*sizeof(char));
	store =  (int*) kmalloc((len/MIN_LEN)*sizeof(int), GFP_KERNEL);
	put_cmd = (char*) kmalloc((len/MIN_LEN)*sizeof(char), GFP_KERNEL);
	memset(g_reg_buffer, 0, REG_BUF_SIZE*sizeof(char));

	/* add '\0' to end of string */
	if (copy_from_user(messages, buff, len)) {
		ret = -1;
		goto error;
	}

	cur = messages;
	*(cur+len-1) = '\0';
	pr_err("[Display] : %s +++\n", cur);

	if (strncmp(cur, "w", 1) == 0)
		flag = true;
	else if(strncmp(cur, "r", 1) == 0)
		flag = false;
	else {
		ret = -1;
		goto error;
	}

	while ((token = strsep(&cur, "wr")) != NULL) {
		str_len = strlen(token);

		if(str_len > 0) { /* filter zero length */
			if(!(strncmp(token, ",", 1) == 0) || (str_len < MAX_LEN)) {
				ret = -1;
				goto error;
			}

			memset(store, 0, (len/MIN_LEN)*sizeof(int));
			memset(put_cmd, 0, (len/MIN_LEN)*sizeof(char));
			cmd_cnt++;

			/* register parameter */
			while ((token_par = strsep(&token, ",")) != NULL) {
				if(strlen(token_par) > MIN_LEN) {
					ret = -1;
					goto error;
				}
				if(strlen(token_par)) {
					sscanf(token_par, "%x", &(store[cnt]));
					cnt++;
				}
			}

			for(i=0; i<cnt; i++)
				put_cmd[i] = store[i]&0xff;

			if(flag) {
				pr_err("[Display] : write panel command\n");
				set_tcon_cmd(put_cmd, cnt);
			}
			else {
				pr_err("[Display] : read panel command\n");
				get_tcon_cmd(put_cmd[0], store[1]);
			}

			if(cur != NULL) {
				if (*(tmp+str_len) == 'w')
					flag = true;
				else if (*(tmp+str_len) == 'r')
					flag = false;
			}
			cnt = 0;
		}

		memset(tmp, 0, len*sizeof(char));

		if(cur != NULL)
			strcpy(tmp, cur);
	}

	if(cmd_cnt == 0) {
		ret = -1;
		goto error;
	}

	ret = len;

error:
	pr_err("[Display] : len = %d ---\n", ret);
	kfree(messages);
	kfree(tmp);
	kfree(store);
	kfree(put_cmd);
	return ret;
}

static ssize_t panel_reg_show(struct file *file, char __user *buf,
                 size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	buff = kzalloc(SZ_4K, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	len += sprintf(buff, "%s\n", g_reg_buffer);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static struct proc_ops panel_reg_rw_ops = {
	.proc_write = panel_reg_rw,
	.proc_read = panel_reg_show,
};

// to support DSI_CTRL_CMD_READ if MIPI_DSI_MSG_CMD_READ is enabled
u32 dsi_ai2202_support_cmd_read_flags(u32 flags)
{
	u32 ret_flags = 0;
	if (flags & MIPI_DSI_MSG_CMD_READ) {
		ret_flags |= (DSI_CTRL_CMD_LAST_COMMAND | DSI_CTRL_CMD_FETCH_MEMORY | DSI_CTRL_CMD_READ);
		DSI_LOG("DSI_CTRL_CMD is 0x%x\n", ret_flags);
	}
	return ret_flags;
}

static void dsi_ai2202_restore_backlight(void)
{
	int rc = 0;

	DSI_LOG("restore bl=%d\n", g_display->panel->panel_last_backlight);
	rc = dsi_panel_set_backlight(g_display->panel, g_display->panel->panel_last_backlight);
	if (rc)
		DSI_LOG("unable to set backlight\n");
}

// send HBM command to panel
static int dsi_ai2202_set_hbm(struct dsi_panel *panel, int enable)
{
	int rc = 0;

	if (!panel) {
		pr_err("invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);
	if (!panel->panel_initialized)
		goto exit;

	if (enable > 0) {
		if (enable == 3) {
			DSI_LOG("[%s] send DSI_CMD_SET_HDR_HBM_ON cmd \n",panel->name);
			rc = dsi_ai2202_tx_cmd_set(panel, DSI_CMD_SET_HDR_HBM_ON);
			if (rc)
				DSI_LOG("[%s] failed to send DSI_CMD_SET_HDR_HBM_ON cmd, rc=%d\n",
					panel->name, rc);
		} else if (enable == 2) {
			DSI_LOG("[%s] send DSI_CMD_SET_CAM_HBM_ON cmd \n",panel->name);
			rc = dsi_ai2202_tx_cmd_set(panel, DSI_CMD_SET_CAM_HBM_ON);
			if (rc)
				DSI_LOG("[%s] failed to send DSI_CMD_SET_CAM_HBM_ON cmd, rc=%d\n",
					panel->name, rc);
		} else {
			DSI_LOG("[%s] send DSI_CMD_SET_HBM_ON cmd \n",panel->name);
			rc = dsi_ai2202_tx_cmd_set(panel, DSI_CMD_SET_HBM_ON);
			if (rc)
				DSI_LOG("[%s] failed to send DSI_CMD_SET_HBM_ON cmd, rc=%d\n",
					panel->name, rc);
		}
	} else {
		if (g_display->panel->panel_hbm_mode == 3) {
			DSI_LOG("[%s] send DSI_CMD_SET_HDR_HBM_OFF cmd \n",panel->name);
			rc = dsi_ai2202_tx_cmd_set(panel, DSI_CMD_SET_HDR_HBM_OFF);
			if (rc)
				DSI_LOG("[%s] failed to send DSI_CMD_SET_HDR_HBM_OFF cmd, rc=%d\n",
					panel->name, rc);
			g_display->panel->panel_hbm_mode = 0;
			dsi_ai2202_restore_backlight();
		} else {
			DSI_LOG("[%s] send DSI_CMD_SET_HBM_OFF cmd \n",panel->name);
			rc = dsi_ai2202_tx_cmd_set(panel, DSI_CMD_SET_HBM_OFF);
			if (rc)
				DSI_LOG("[%s] failed to send DSI_CMD_SET_HBM_OFF cmd, rc=%d\n",
					panel->name, rc);
		}
	}
exit:
	mutex_unlock(&panel->panel_lock);
	return rc;
}

// called for /proc/hbm_mode
static void display_set_hbm_mode(int mode)
{
	if (mode == g_display->panel->panel_hbm_mode) {
		DSI_LOG("hbm mode same.\n");
		return;
	}

	DSI_LOG("hbm mode set from (%d) to (%d)\n", g_display->panel->panel_hbm_mode, mode);
	dsi_ai2202_set_hbm(g_display->panel, mode);
	g_display->panel->panel_hbm_mode = mode;
}

// check display or panal valid
static inline bool display_panel_valid(void)
{
	if (!g_display || !g_display->panel) {
		DSI_LOG("[%pS] display or panel is not valid.\n", __builtin_return_address(0));
		return false;
	}

	return true;
}

// hbm_mode_ops() - set HBM on/off & read HBM status
static ssize_t hbm_mode_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	memset(messages, 0, sizeof(messages));

	if (len > 256)
		len = 256;
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if (!display_panel_valid())
		return -EINVAL;

	if (g_display->panel->panel_is_on) {
		if (strncmp(messages, "0", 1) == 0) {
			display_set_hbm_mode(0);
		} else {
			display_set_hbm_mode(messages[0]-'0');
		}
	} else {
		DSI_LOG("unable to set in display off\n");
		g_display->panel->panel_hbm_mode = 0;
	}

	return len;
}

static ssize_t hbm_mode_read(struct file *file, char __user *buf,
							 size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	if (*ppos)
		return 0;

	DSI_LOG("hbm mode is %d\n", g_display->panel->panel_hbm_mode);

	len += sprintf(buff, "%d\n", g_display->panel->panel_hbm_mode);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static struct proc_ops hbm_mode_ops = {
	.proc_write = hbm_mode_write,
	.proc_read = hbm_mode_read,
};

// clear dc parameters
static inline void display_panel_clear_dc(void)
{
	if (!display_panel_valid())
		return;

	atomic_set(&g_display->panel->is_dc_change, 0);
	atomic_set(&g_display->panel->is_bl_ready, 0);
	atomic_set(&g_display->panel->allow_bl_change, 0);
	atomic_set(&g_display->panel->is_i6_change, 0);
}

static ssize_t lcd_brightness_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	u32 DC_mode = 0;
	static u32 old_mode = 0;

	memset(messages, 0, sizeof(messages));

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if (!display_panel_valid())
		return -EFAULT;

	mutex_lock(&g_display->panel->panel_lock);
	if (!g_display->panel->panel_initialized)
		goto exit;
	sscanf(messages, "%u", &DC_mode);

	DSI_LOG("dc lcd brightess write (%d) +++ \n",DC_mode);
	if (old_mode == DC_mode) {
		//DSI_LOG("dc same (%d)\n", DC_mode);
		goto exit;
	}

	old_mode = DC_mode;
	g_display->panel->dc_mode = DC_mode;

	display_panel_clear_dc();

	// only this criteria need to validate igc
	if (g_display->panel->panel_last_backlight <= 248) {
		DSI_LOG("dc=%d\n", DC_mode);
		atomic_set(&g_display->panel->is_dc_change, 1);
	}
exit:
	mutex_unlock(&g_display->panel->panel_lock);
	return len;
}

static struct proc_ops lcd_brightness_ops = {
	.proc_write = lcd_brightness_write,
};

// send dimming command to panel
static int dsi_ai2202_set_dimming(struct dsi_panel *panel, bool enable)
{
	int rc = 0;

	if (!panel) {
		DSI_LOG("invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);
	if (!panel->panel_initialized)
		goto exit;

	if (enable)
		rc = dsi_ai2202_tx_cmd_set(panel, DSI_CMD_SET_DIMMING_SPEED_1);
	else
		rc = dsi_ai2202_tx_cmd_set(panel, DSI_CMD_SET_DIMMING_SPEED_20);

exit:
	mutex_unlock(&panel->panel_lock);
	return rc;
}

static ssize_t dimming_speed_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	int rc = 0;
	memset(messages, 0, sizeof(messages));

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if (!display_panel_valid())
		return -EFAULT;

	DSI_LOG("dimming speed write +++ val:%s \n",messages);

	if (g_display->panel->panel_is_on) {
		if (strncmp(messages, "1", 1) == 0) {
			rc = dsi_ai2202_set_dimming(g_display->panel, true);
		} else if (strncmp(messages, "20", 20) == 0) {
		} else {
			DSI_LOG("doesn't match any dimming speed .\n");
		}
	} else {
		DSI_LOG("unable to set in display off\n");
	}

	if(rc) {
		DSI_LOG("panel set dimming speed failed\n");
	}

	return len;
}

static struct proc_ops dimming_speed_ops = {
	.proc_write = dimming_speed_write,
};

// add for set min backlight to 2
u32 dsi_ai2202_backlightupdate(u32 bl_lvl)
{
	// for dc dimming
	if (bl_lvl <= 248 && bl_lvl != 0 &&
		(g_display->panel->dc_mode || g_display->panel->csc_mode)) {
		//DSI_LOG("dc bl 248");
		return 248;
	}

	if (bl_lvl >= 1 && bl_lvl < 4) {
		return 4;
	} else {
		return bl_lvl;
	}
}

void dsi_ai2202_need_aod_reset(struct dsi_panel *panel)
{
	int rc = 0;

	mutex_lock(&panel->panel_lock);
	if (!panel->panel_initialized)
		goto exit;

	if(panel->panel_last_backlight == 1 || panel->aod_mode == 1) {
		DSI_LOG("notify set AOD LOW command\n");
		rc = dsi_ai2202_tx_cmd_set(panel, DSI_CMD_SET_AOD_LOW);
	} else if(panel->panel_last_backlight == 61 || panel->aod_mode == 2) {
		DSI_LOG("notify set AOD HIGH command\n");
		rc = dsi_ai2202_tx_cmd_set(panel, DSI_CMD_SET_AOD_HIGH);
	}

	if(rc) {
		DSI_LOG("failed to set notify AOD command");
	}

	panel->panel_aod_last_bl = panel->panel_last_backlight;
	panel->aod_delay = false;
exit:
	mutex_unlock(&panel->panel_lock);
	return;
}

static void dsi_ai2202_aod_backlight(struct dsi_panel *panel)
{
	int rc = 0;
	if (!panel) {
		DSI_LOG("invalid params\n");
		return;
	}

	if (panel->aod_state ) {
		if (panel->panel_last_backlight == 61 || panel->panel_last_backlight == 1) {
			if (panel->panel_last_backlight == 61 && panel->panel_last_backlight != panel->panel_aod_last_bl) {
				DSI_LOG("set AOD HIGH command\n");
				panel->panel_aod_last_bl = panel->panel_last_backlight;
				panel->aod_mode = 2;
				rc = dsi_ai2202_tx_cmd_set(panel, DSI_CMD_SET_AOD_HIGH);
			} else if (panel->panel_last_backlight == 1 && panel->panel_last_backlight != panel->panel_aod_last_bl) {
				DSI_LOG("set AOD LOW command\n");
				panel->panel_aod_last_bl = panel->panel_last_backlight;
				panel->aod_mode = 1;
				rc = dsi_ai2202_tx_cmd_set(panel, DSI_CMD_SET_AOD_LOW);
			}
		} else { // to prevent display off
			DSI_LOG("set AOD Other command\n");
			rc = dsi_ai2202_tx_cmd_set(panel, DSI_CMD_SET_AOD_OTHER);
		}

		if (rc) {
			DSI_LOG("unable to set AOD command\n");
			g_display->panel->aod_mode = 0;
		}
	}
}

static ssize_t hbm_mode_show(struct class *class,
					struct class_attribute *attr,
					char *buf)
{
	return sprintf(buf, "%d\n", g_display->panel->panel_hbm_mode);
}

static ssize_t hbm_mode_store(struct class *class,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	if (!count)
		return -EINVAL;

	if (g_display->panel->panel_is_on) {
		if (strncmp(buf, "0", 1) == 0) {
			display_set_hbm_mode(0);
		} else {
			display_set_hbm_mode(buf[0]-'0');
		}
	} else {
		DSI_LOG("unable to set in display off\n");
		g_display->panel->panel_hbm_mode = 0;
	}

	return count;
}

static ssize_t hdr_mode_show(struct class *class,
					struct class_attribute *attr,
					char *buf)
{
	return sprintf(buf, "%d\n", g_hdr);
}

static ssize_t hdr_mode_store(struct class *class,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	if (!count)
		return -EINVAL;

	sscanf(buf, "%d", &g_hdr);

	return count;
}

static CLASS_ATTR_RW(hbm_mode);
static CLASS_ATTR_RW(hdr_mode);


static void drm_class_device_release(struct device *dev)
{
	pr_err("[Display] release asus drm\n");
}

static struct device_type drm_class_type = {
	.name = "asus_fod",
};

static struct device drm_dev = {
	.type = &drm_class_type,
	.release = &drm_class_device_release
};


static void dsi_create_drm_class_obj(void)
{
	int err;
	err = dev_set_name(&drm_dev, "asus_fod");
	if (err != 0) {
		pr_err("[Display] Fail to create asus fod\n");
		return;
	}

	drm_dev.class = class_create(THIS_MODULE, "asus_fod");

	err = class_create_file(drm_dev.class, &class_attr_hbm_mode);
	if (err) {
		pr_err("[Display] Fail to create hbm_mode file node\n");
	}

	err = class_create_file(drm_dev.class, &class_attr_hdr_mode);
	if (err) {
		pr_err("[Display] Fail to create hdr_mode file node\n");
	}
}

// to record & restore user's last backlight
void dsi_ai2202_record_backlight(u32 bl_lvl)
{
	if (bl_lvl == 0)
		return;

	if (g_display->panel->panel_last_backlight == bl_lvl) {
		g_display->panel->panel_bl_count = 0;
	}
	g_display->panel->panel_last_backlight = bl_lvl;
	//#define SDE_MODE_DPMS_LP1	1      sde_drm.h
	//#define SDE_MODE_DPMS_LP2	2

	if ((g_display->panel->power_mode == 1) ||
		(g_display->panel->power_mode == 2)) {
		dsi_ai2202_aod_backlight(g_display->panel);
	}
}

void dsi_ai2202_frame_commit_cnt(struct drm_crtc *crtc)
{
	static int aod_delay_frames = 0;

	if (display_commit_cnt > 0 && !strcmp(crtc->name, "crtc-0")) {
		DSI_LOG("fbc%d\n", display_commit_cnt);
		display_commit_cnt--;
	}

	if(g_display->panel->aod_delay && g_display->panel->aod_state) {
		aod_delay_frames++;
		// for receive msg case only
		if(aod_delay_frames >= 5) {
			dsi_ai2202_need_aod_reset(g_display->panel);
			aod_delay_frames = 0;
		}
	}
}

// to show & clear frame commit count
void dsi_ai2202_clear_commit_cnt(void)
{
	display_commit_cnt = COMMIT_FRAMES_COUNT;
}

void dsi_ai2202_display_init(struct dsi_display *display)
{
	DSI_LOG("dsi_ai2202_display_init  !\n ");
	dsi_create_drm_class_obj();

	g_display = display;
	dsi_ai2202_parse_panel_vendor_id(g_display->panel);

	g_display->panel->panel_hbm_mode = 0;
	g_display->panel->panel_is_on = false;
	g_display->panel->panel_bl_count = 0;
	g_display->panel->dc_bl_delay = false;
	g_display->panel->csc_mode = 0;
	g_display->panel->aod_state = false;
	g_display->panel->aod_delay = false;

	proc_create(LCD_UNIQUE_ID, 0444, NULL, &lcd_unique_id_ops);
	proc_create(PANEL_VENDOR_ID, 0640, NULL, &panel_vendor_id_ops);
	proc_create(PANEL_FPS, 0660, NULL, &panel_fps_ops);
	proc_create(PANEL_REGISTER_RW, 0640, NULL, &panel_reg_rw_ops);
	proc_create(HBM_MODE, 0666, NULL, &hbm_mode_ops);
	proc_create(DIMMING_SPEED, 0666, NULL, &dimming_speed_ops);
	proc_create(LCD_BACKLIGNTNESS, 0666, NULL, &lcd_brightness_ops);
}

// to parse panel_vendor_id from panel dtsi
void dsi_ai2202_parse_panel_vendor_id(struct dsi_panel *panel)
{
	struct dsi_parser_utils *utils = &panel->utils;

	panel->panel_vendor_id = utils->get_property(utils->data,
			"qcom,mdss-dsi-panel-vendor-id", NULL);
	DSI_LOG("panel vendor id = %s", panel->panel_vendor_id);
}

// to store panel status & reset display parameter
void dsi_ai2202_set_panel_is_on(bool on)
{
     // DSI_LOG("dsi_zf8_set_panel_is_on  !\n ");
	g_display->panel->panel_is_on = on;

	if (on == false) {
		DSI_LOG("dsi_ai2202_set_panel_is_on  false !\n ");
		g_display->panel->panel_hbm_mode = 0;
		g_display->panel->panel_last_backlight = 0;
		g_display->panel->panel_aod_last_bl = 0;
		g_display->panel->aod_state = false;
		g_display->panel->aod_delay = false;
		g_display->panel->aod_mode = 0;
		g_display->panel->panel_bl_count = 0;
		display_panel_clear_dc();
		g_display->panel->csc_mode = 0;
	}
}

// to validate skip igc or not
// to allow set bl if is_bl_ready count to 2
static bool dsi_ai2202_validate_c2_last(u32 c2_last)
{
	if (g_display->panel->dc_mode) {
		// case#off2on : 4095 -> (<4095)
		// must same c2_last value set twice from framework (514 command)
		// the first c2_last should be different from last c2_last.
		if (c2_last != g_display->panel->c2_last) {
			atomic_set(&g_display->panel->is_bl_ready, 1);
			return (c2_last < 4095);
		}

		// expect that is_bl_ready equal to 2 after atomic_add
		atomic_add(1, &g_display->panel->is_bl_ready);
		return false;

	} else {
		// case#on2off : (<4095) -> 4095
		// skip igc data if lower than 4095
		atomic_set(&g_display->panel->is_bl_ready, 2);
		return (c2_last < 4095);
	}
}

// called from sde_hw_reg_dma_v1_color_proc.c
bool ai2202_need_skip_data(u32 c2_last)
{
	bool rc = false;

	// don't validate if no dc change
	if (!atomic_read(&g_display->panel->is_dc_change))
		return rc;

	rc = dsi_ai2202_validate_c2_last(c2_last);
	DSI_LOG("igc=%s\n", rc?"skip":"apply");
	return rc;
}

void ai2202_store_c2_last(u32 c2_last)
{
	g_display->panel->c2_last = c2_last;
}

static void dsi_ai2202_dc_bl_set(void)
{
	atomic_set(&g_display->panel->allow_bl_change, 1);
	dsi_ai2202_restore_backlight();
	display_panel_clear_dc();
}

// called every frame commit
void ai2202_set_dc_bl_process(struct drm_encoder *encoder, struct drm_crtc *crtc)
{
	int rc = 0;
	static int csc_cnt = 0;
	if (!display_panel_valid())
		return;

	if (strcmp(crtc->name, "crtc-0"))
		return;

	// first priority for i6 csc change
	if (atomic_read(&g_display->panel->is_i6_change) == 1) {
		if (g_display->panel->csc_mode) {
			if (csc_cnt == 3)
				goto start;
		} else {
			goto start;
		}
		csc_cnt++;
	}

	// don't validate if no dc change
	if (!atomic_read(&g_display->panel->is_dc_change))
		return;

	// only set bl after is_bl_ready count to 2
	if (!(atomic_read(&g_display->panel->is_bl_ready) == 2))
		return;

start:
	mutex_lock(&g_display->panel->panel_lock);
	if (!g_display->panel->panel_initialized)
		goto exit;

	sde_encoder_wait_for_event(encoder, MSM_ENC_VBLANK);
	dsi_ai2202_dc_bl_set();

exit:
	csc_cnt = 0;

	mutex_unlock(&g_display->panel->panel_lock);
	// fps switch pending
	if (atomic_read(&g_display->panel->is_fps_pending)) {
		rc = dsi_panel_switch(g_display->panel);
		if (rc)
			DSI_ERR("[%s] failed to switch DSI panel mode, rc=%d\n",
				   g_display->name, rc);
		atomic_set(&g_display->panel->is_fps_pending, 0);
	}
}

module_param_string(LCDUID, dsi_display_unique_id, MAX_CMDLINE_PARAM_LEN,
								0600);
#endif
