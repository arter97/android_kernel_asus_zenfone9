/*
 * Copyright (c) 2020, ASUS. All rights reserved.
 */

#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/syscalls.h>
#include <linux/delay.h>
#include <linux/device.h>

#include "dsi_ai2201.h"
#include "dsi_panel.h"

#ifdef ASUS_AI2201_PROJECT
#include <sde_encoder_phys.h>

struct dsi_display *g_display;
char g_reg_buffer[REG_BUF_SIZE];
char g_lcd_unique_id[10];
int display_commit_cnt = COMMIT_FRAMES_COUNT;
// FOD feature
bool has_fod_masker;
bool old_has_fod_masker;
bool has_fod_spot;
bool old_has_fod_spot;
bool allow_fod_hbm_pending = false;  // ready to access FOD HBM from framework
extern bool has_fod_spot;   //flag indicate the fod spot layer exist
extern bool g_Charger_mode;
bool err_fg_pending;

static int g_hdr = 0;
int fod_spot_ui_ready;
int ghbm_on_requested;
int ghbm_on_achieved;
int fod_gesture_touched;

// DC mode
bool dc_fixed_bl;

// check display or panal valid
static inline bool display_panel_valid(void)
{
	if (!g_display || !g_display->panel) {
		DSI_LOG("[%pS] display or panel is not valid.\n", __builtin_return_address(0));
		return false;
	}

	return true;
}

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

// copy from dsi_panel_tx_cmd_set()
static int dsi_ai2201_tx_cmd_set(struct dsi_panel *panel,
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
/*
		if (type == DSI_CMD_SET_VID_TO_CMD_SWITCH)
			cmds->msg.flags |= MIPI_DSI_MSG_ASYNC_OVERRIDE;
*/
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
	// only for FOD HBM ON
	if (type == DSI_CMD_SET_FOD_HBM_ON || type == DSI_CMD_SET_POST_FOD_HBM_ON) {
		panel->ktime0 = ktime_get();
		//DSI_LOG("hbm on start on %d (us). +++", (u32)ktime_to_us(g_display->panel->ktime0));
	}
	return rc;
}

// send HBM command to panel
static int dsi_ai2201_set_hbm(struct dsi_panel *panel, bool enable)
{
	int rc = 0;

	if (!panel) {
		pr_err("invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);
	if (!panel->panel_initialized)
		goto exit;

	if (enable) {
		rc = dsi_ai2201_tx_cmd_set(panel, DSI_CMD_SET_HBM_ON);
		if (rc)
			DSI_LOG("[%s] failed to send DSI_CMD_SET_HBM_ON cmd, rc=%d\n",
				   panel->name, rc);
	} else {
		rc = dsi_ai2201_tx_cmd_set(panel, DSI_CMD_SET_HBM_OFF);
		if (rc)
			DSI_LOG("[%s] failed to send DSI_CMD_SET_HBM_ON cmd, rc=%d\n",
				   panel->name, rc);
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

	g_display->panel->panel_hbm_mode = mode;
	DSI_LOG("hbm mode is (%d)\n", g_display->panel->panel_hbm_mode);
	dsi_ai2201_set_hbm(g_display->panel, mode);
}

// exit idle mode
static void display_exit_idle_mode()
{
	int rc = 0;

	DSI_LOG("power_mode/aod_state are (%d, %d)\n",
			g_display->panel->power_mode, g_display->panel->aod_state);
	// SDE_MODE_DPMS_LP1    1
	// SDE_MODE_DPMS_LP2    2
	// checking aod_state
	if ((g_display->panel->power_mode != 1 &&
			g_display->panel->power_mode != 2) && !g_display->panel->aod_state)
		return;

	rc = dsi_ai2201_tx_cmd_set(g_display->panel, DSI_CMD_SET_NOLP);
	if (rc)
		DSI_LOG("[%s] failed to send DSI_CMD_SET_NOLP cmd, rc=%d\n",
			   g_display->panel->name, rc);
	else {
		g_display->panel->fod_in_doze = true;
		g_display->panel->aod_state = false;
		g_display->panel->aod_mode = -1;
		g_display->panel->panel_aod_last_bl = 0;
	}
}

// send Fod HBM command to panel
static int dsi_ai2201_set_fod_hbm(struct dsi_panel *panel, bool enable)
{
	int rc = 0;

	if (!panel) {
		DSI_LOG("invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);
	if (!panel->panel_initialized)
		goto exit;

	if (enable) {
		// exit idle mode if enter doze before
		display_exit_idle_mode();

		// to aviod ghbm without mask
		if (panel->fod_in_doze) {
			DSI_LOG("set display off first\n");
			rc = dsi_ai2201_tx_cmd_set(panel, DSI_CMD_SET_LP1);
			if (rc)
				DSI_LOG("[%s] failed to send DSI_CMD_SET_LP1 cmd, rc=%d\n",
					   panel->name, rc);
			else if (panel->cur_mode->timing.refresh_rate > 120) {
				DSI_LOG("DSI_CMD_SET_TIMING_SWITCH %d", panel->cur_mode->timing.refresh_rate);
				rc = dsi_ai2201_tx_cmd_set(panel, DSI_CMD_SET_TIMING_SWITCH);
				if (rc)
					DSI_LOG("[%s] failed to send DSI_CMD_SET_TIMING_SWITCH cmd, rc=%d\n",
					   panel->name, rc);
			}
		} else {
			DSI_LOG("set display on directly\n");
			rc = dsi_ai2201_tx_cmd_set(panel, DSI_CMD_SET_FOD_HBM_ON);
			if (rc)
				DSI_LOG("[%s] failed to send DSI_CMD_SET_FOD_HBM_ON cmd, rc=%d\n",
					   panel->name, rc);
		}
	} else {
		dsi_ai2201_restore_backlight();

		rc = dsi_ai2201_tx_cmd_set(panel, DSI_CMD_SET_FOD_HBM_OFF);
		if (rc)
			DSI_LOG("[%s] failed to send DSI_CMD_SET_FOD_HBM_OFF cmd, rc=%d\n",
				   panel->name, rc);
	}

exit:
	mutex_unlock(&panel->panel_lock);
	return rc;
}

// called for /proc/globalHbm
static void display_set_fod_hbm_mode(int mode)
{
	if (mode == g_display->panel->panel_fod_hbm_mode) {
		DSI_LOG("fod hbm mode same.\n");
		return;
	}

	g_display->panel->panel_fod_hbm_mode = mode;
	DSI_LOG("fod hbm mode is (%d)\n", g_display->panel->panel_fod_hbm_mode);
	dsi_ai2201_set_fod_hbm(g_display->panel, mode);
}

// called for FOD flow, as /proc/globalHbm
static void display_set_fod_hbm(void)
{
	if (!display_panel_valid())
		return;

	if (!g_display->panel->panel_is_on) {
		DSI_LOG("display is off.\n");
		return;
	}

	DSI_LOG("FOD: global hbm <fod> (%d) +++ \n", g_display->panel->allow_panel_fod_hbm);
	dsi_ai2201_set_fod_hbm(g_display->panel, g_display->panel->allow_panel_fod_hbm);
	ai2201_drm_notify(ASUS_NOTIFY_GHBM_ON_REQ, g_display->panel->allow_panel_fod_hbm);
	DSI_LOG("FOD: global hbm <fod> (%d) --- \n", g_display->panel->allow_panel_fod_hbm);

	// reset fod hbm pending immediately, need be guard by lock
	g_display->panel->allow_fod_hbm_process = false;

	if (g_display->panel->allow_panel_fod_hbm == 0) {
		DSI_LOG("global hbm off, reset hbm mode %d", g_display->panel->panel_hbm_mode);
		if (g_display->panel->panel_hbm_mode) dsi_ai2201_set_hbm(g_display->panel, 1);
	}
}

// send Dimming Smooth command to panel
void dsi_ai2201_set_dimming_smooth(struct dsi_panel *panel, u32 backlight)
{
	int rc = 0;

	// set to 1 if set bl from FOD or DC process in kernel
	if (atomic_read(&panel->allow_bl_change) || panel->allow_fod_hbm_process || atomic_read(&panel->is_spot_ready)) {
		panel->panel_bl_count = 1;
		return;
	}

	if (panel->aod_state || backlight == 0) {
		panel->panel_bl_count = 0;
		return;
	}

	if (panel->panel_bl_count == 1) {
		DSI_LOG("restore dimming smooth\n");
		rc = dsi_ai2201_tx_cmd_set(panel, DSI_CMD_SET_DIMMING_SMOOTH);
		if (rc)
			DSI_LOG("[%s] failed to send DSI_CMD_SET_DIMMING_SMOOTH cmd, rc=%d\n",
				   panel->name, rc);
	}

	panel->panel_bl_count++;
}

/**
 * panel_reg_rw_ops()
 * @panel_reg_rw: Read/write panel register
 * @panel_reg_show: Show panel register
 */
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
	DSI_LOG("%s +++\n", cur);

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
				DSI_LOG("write panel command\n");
				set_tcon_cmd(put_cmd, cnt);
			}
			else {
				DSI_LOG("read panel command\n");
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
	DSI_LOG("len = %d ---\n", ret);
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
u32 dsi_ai2201_support_cmd_read_flags(u32 flags)
{
	u32 ret_flags = 0;
	if (flags & MIPI_DSI_MSG_CMD_READ) {
		ret_flags |= (DSI_CTRL_CMD_LAST_COMMAND | DSI_CTRL_CMD_FETCH_MEMORY | DSI_CTRL_CMD_READ);
		//DSI_LOG("DSI_CTRL_CMD is 0x%x\n", ret_flags);
	}

	return ret_flags;
}

/**
 * dsi_ai2201_clear_commit_cnt()
 * @display_commit_cnt: set to COMMIT_FRAMES_COUNT
 */
void dsi_ai2201_clear_commit_cnt(void)
{
	display_commit_cnt = COMMIT_FRAMES_COUNT;
};

/**
* dsi_ai2201_frame_commit_cnt()
* @crtc : Central CRTC control structure
*/
void dsi_ai2201_frame_commit_cnt(struct drm_crtc *crtc)
{
	static int dc_delay_frames = 0;
	static int aod_delay_frames = 0;
	if (display_commit_cnt > 0 && !strcmp(crtc->name, "crtc-0")) {
		DSI_LOG("fbc%d\n", display_commit_cnt);
		display_commit_cnt--;
	}

	if(g_display->panel->aod_delay && g_display->panel->aod_state) {
		if(g_display->panel->dc_bl_delay) {
			dc_delay_frames = 0;
			g_display->panel->dc_bl_delay = false;
		}
		aod_delay_frames++;
		// for receive msg case only
		if(aod_delay_frames >= 5 && !(g_display->panel->allow_panel_fod_hbm || g_display->panel->allow_fod_hbm_process)) {
			dsi_ai2201_need_aod_reset(g_display->panel);
			aod_delay_frames = 0;
		}
	}

	// Set default backlight = 9, if nobody coming to set AOD HIGH/LOW.
	if (display_commit_cnt == 0 && g_display->panel->aod_state) {
		if (g_display->panel->aod_mode == -1) {
			mutex_lock(&g_display->panel->panel_lock);
			if (g_display->panel->panel_last_backlight > 15) {
				DSI_LOG("DEFAULT AOD HIGH BACKLIGHT, panel_last_backlight = %d", g_display->panel->panel_last_backlight);
				dsi_panel_set_backlight(g_display->panel, 61);
			} else {
				DSI_LOG("DEFAULT AOD LOW BACKLIGHT,  panel_last_backlight = %d", g_display->panel->panel_last_backlight);
				dsi_panel_set_backlight(g_display->panel,  9);
			}
			mutex_unlock(&g_display->panel->panel_lock);
		}
		display_commit_cnt--;
	}
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

/**
* lcd_unique_id_read() - read panel id*
* @g_lcd_unique_id : Panel ID*
*/
static ssize_t lcd_unique_id_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	buff = kzalloc(256, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	len += sprintf(buff, "%s\n", g_lcd_unique_id);
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

	//DSI_LOG("refreshrate  %d\n",g_display->panel->cur_mode->timing.refresh_rate);
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

	//DSI_LOG("dc lcd brightess write (%d) +++ \n",DC_mode);
	if (old_mode == DC_mode) {
		//DSI_LOG("dc same (%d)\n", DC_mode);
		goto exit;
	}

	old_mode = DC_mode;
	g_display->panel->dc_mode = DC_mode;

	display_panel_clear_dc();

	// only this criteria need to validate igc
	if (g_display->panel->panel_last_backlight <= 224) {
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

// global_hbm_mode_ops() - set Fod HBM on/off & read Fod HBM status
static ssize_t global_hbm_mode_write(struct file *filp, const char *buff, size_t len, loff_t *off)
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
			display_set_fod_hbm_mode(0);
		} else if (strncmp(messages, "1", 1) == 0) {
			display_set_fod_hbm_mode(1);
		} else {
			DSI_LOG("don't match any hbm mode.\n");
		}
	} else {
		DSI_LOG("unable to set in display off\n");
		g_display->panel->panel_fod_hbm_mode = 0;
	}

	return len;
}

static ssize_t global_hbm_mode_read(struct file *file, char __user *buf,
							 size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	if (*ppos)
		return 0;

	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	DSI_LOG("fod hbm mode is %d\n", g_display->panel->panel_fod_hbm_mode);

	len += sprintf(buff, "%d\n", g_display->panel->panel_fod_hbm_mode);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static struct proc_ops global_hbm_mode_ops = {
	.proc_write = global_hbm_mode_write,
	.proc_read  = global_hbm_mode_read,
};

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

	DSI_LOG("hbm mode is %d\n", g_display->panel->panel_hbm_mode);

	len += sprintf(buff, "%d\n", g_display->panel->panel_hbm_mode);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static struct proc_ops hbm_mode_ops = {
	.proc_write = hbm_mode_write,
	.proc_read  = hbm_mode_read,
};

// send dimming command to panel
static int dsi_ai2201_set_dimming(struct dsi_panel *panel, bool enable)
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
		rc = dsi_ai2201_tx_cmd_set(panel, DSI_CMD_SET_DIMMING_SPEED_1);
	else
		rc = dsi_ai2201_tx_cmd_set(panel, DSI_CMD_SET_DIMMING_SPEED_20);

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

	if (g_display->panel->panel_is_on
		&& !(g_display->panel->allow_panel_fod_hbm || g_display->panel->allow_fod_hbm_process)) {
		if (strncmp(messages, "1", 1) == 0) {
			rc = dsi_ai2201_set_dimming(g_display->panel, true);
		} else if (strncmp(messages, "20", 20) == 0) {
			// restore by dsi_ai2201_set_dimming_smooth
			//rc = dsi_ai2201_set_dimming(g_display->panel, false);
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
u32 dsi_ai2201_backlightupdate(u32 bl_lvl)
{
	// for dc dimming
	if (bl_lvl <= 224 && bl_lvl != 0 &&
		(g_display->panel->dc_mode || g_display->panel->csc_mode)) {
		// restore bl from FOD if dc change still processing
		if (g_display->panel->allow_fod_hbm_process &&
				atomic_read(&g_display->panel->is_dc_change))
			return bl_lvl;
		//DSI_LOG("dc bl 224");
		return 224;
	}

	if (bl_lvl == 1) {
		return 2;
	}
	else if (bl_lvl == 9) {
		return 10;
	}
	else {
		return bl_lvl;
	}
}

void ai2201_set_err_fg_irq_state(bool state) {
	if(g_display->panel->panel_is_on || !state) {
		g_display->panel->err_fg_irq_is_on = state;
		if(state) {
			DSI_LOG("set err fg irq as on");
			g_display->panel->esd_fail = true;
			g_display->panel->power_on = false;
			ai2201_set_err_fg_pending(true);
		}
	}
}

bool ai2201_get_err_fg_irq_state(void) {
	if(g_display->panel->panel_is_on) {
		return g_display->panel->err_fg_irq_is_on;
	}
	else {
		return false;
	}
}

void ai2201_set_err_fg_pending(bool state) {
    err_fg_pending = state;
    if(state) {
        DSI_LOG("err fg pending backlight update");
    } else {
        DSI_LOG("err fg backlight updated");
    }
}

bool ai2201_get_err_fg_pending(void) {
    return err_fg_pending;
}

// FOD Node
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

static ssize_t ghbm_on_requested_show(struct class *class,
					struct class_attribute *attr,
					char *buf)
{
	return sprintf(buf, "%d\n", ghbm_on_requested);
}

static ssize_t ghbm_on_achieved_show(struct class *class,
					struct class_attribute *attr,
					char *buf)
{
	return sprintf(buf, "%d\n", ghbm_on_achieved);
}

static ssize_t spot_on_achieved_show(struct class *class,
					struct class_attribute *attr,
					char *buf)
{
	return sprintf(buf, "%d\n", fod_spot_ui_ready);
}

static ssize_t fod_touched_show(struct class *class,
					struct class_attribute *attr,
					char *buf)
{
	return sprintf(buf, "%d\n", fod_gesture_touched);
}

static ssize_t fod_touched_store(struct class *class,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	if (!count)
		return -EINVAL;

	sscanf(buf, "%d", &fod_gesture_touched);

	return count;
}

static CLASS_ATTR_RW(hbm_mode);
static CLASS_ATTR_RW(hdr_mode);
static CLASS_ATTR_RO(ghbm_on_requested);
static CLASS_ATTR_RO(ghbm_on_achieved);
static CLASS_ATTR_RO(spot_on_achieved);
static CLASS_ATTR_RW(fod_touched);

static void drm_class_device_release(struct device *dev)
{
	pr_err("[Display] release asus drm\n");
}

static struct device_type drm_class_type = {
	.name = "asus_drm",
};

static struct device drm_dev = {
	.type = &drm_class_type,
	.release = &drm_class_device_release
};

static void dsi_create_drm_class_obj(void)
{
	int err;
	err = dev_set_name(&drm_dev, "asus_drm");
	if (err != 0) {
		pr_err("[Display] Fail to create asus drm\n");
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

	err = class_create_file(drm_dev.class, &class_attr_ghbm_on_requested);
	if (err) {
		pr_err("[Display] Fail to create ghbm_on_requested file node\n");
	}

	err = class_create_file(drm_dev.class, &class_attr_ghbm_on_achieved);
	if (err) {
		pr_err("[Display] Fail to create ghbm_on_achieved file node\n");
	}

	err = class_create_file(drm_dev.class, &class_attr_spot_on_achieved);
	if (err) {
		pr_err("[Display] Fail to create fod_ui_ready file node\n");
	}

	err = class_create_file(drm_dev.class, &class_attr_fod_touched);
	if (err) {
		pr_err("[Display] Fail to create fod_touched file node\n");
	}
}

// FOD feature
struct drm_subsys_private {
	struct kset subsys;
	struct kset *devices_kset;
	struct list_head interfaces;
	struct mutex mutex;

	struct kset *drivers_kset;
	struct klist klist_devices;
	struct klist klist_drivers;
	struct blocking_notifier_head bus_notifier;
	unsigned int drivers_autoprobe:1;
	struct bus_type *bus;

	struct kset glue_dirs;
	struct class *class;
};

struct kobject* asus_class_get_kobj(struct class *cls)
{
	if (cls) {
		struct drm_subsys_private* private = (struct drm_subsys_private*)cls->p;
		return &private->subsys.kobj;
	}
	return NULL;
}

void ai2201_drm_notify(int var, int value)
{
	int *selected_var = NULL;
	char *selected_var_name;

	switch (var) {
	case ASUS_NOTIFY_GHBM_ON_REQ:
		selected_var = &ghbm_on_requested;
		selected_var_name = "ghbm_on_requested";
		break;
	case ASUS_NOTIFY_GHBM_ON_READY:
		selected_var = &ghbm_on_achieved;
		selected_var_name = "ghbm_on_achieved";
		break;
	case ASUS_NOTIFY_SPOT_READY:
		selected_var = &fod_spot_ui_ready;
		selected_var_name = "spot_on_achieved";
		break;
	case ASUS_NOTIFY_FOD_TOUCHED:
		selected_var = &fod_gesture_touched;
		selected_var_name = "fod_touched";
		break;
	default:
		pr_err("[Display] unsupported drm notify variable type %d\n", var);
		return;
	}

	if (!selected_var) {
		pr_err("[Display] var is null\n");
		return;
	}

	if (*selected_var == value) {
		pr_err("[Display] value same, variable type %d, value %d\n", var, *selected_var);
	} else {
		pr_err("[Display] update variable type %d from %d to %d\n", var, *selected_var, value);
		*selected_var = value;
		sysfs_notify(asus_class_get_kobj(drm_dev.class), NULL, selected_var_name);
	}
}

bool is_DSI_mode(int vdisplay, int vtotal)
{
	if( (2448 == vdisplay) && (2463 == vtotal) ) {
		return true;
	}
	return false;
}

bool refreshrate_match(int refresh1, int refresh2)
{
	if( refresh1 == refresh2 ) {
		return true;
	}
	return false;
}

/**
 * dsi_ai2201_display_init() - to initial display parameters
 * @g_display: Pointer to get dsi_display information.
 * @proc_create: Panel read/write node
 */
void dsi_ai2201_display_init(struct dsi_display *display)
{
	DSI_LOG("AI2201 Display Init");

	dsi_create_drm_class_obj();

	g_display = display;
	dsi_ai2201_parse_panel_vendor_id(g_display->panel);

	g_display->panel->panel_hbm_mode = 0;
	g_display->panel->panel_fod_hbm_mode = 0;
	g_display->panel->allow_panel_fod_hbm = 0;
	g_display->panel->allow_fod_hbm_process = false;
	g_display->panel->panel_is_on = false;
	g_display->panel->panel_last_backlight = 0;
	g_display->panel->panel_aod_last_bl = 0;
	g_display->panel->panel_bl_count = 0;
	g_display->panel->aod_state = false;
	g_display->panel->aod_mode = -1;
	g_display->panel->dc_bl_delay = false;
	g_display->panel->fod_in_doze = false;
	g_display->panel->err_fg_irq_is_on = false;
	g_display->panel->is_gamma_get = false;
	g_display->panel->is_gamma_change = false;
	g_display->panel->is_first_gamma_set = false;
	g_display->panel->aod_delay = false;
	g_display->panel->csc_mode = 0;
	atomic_set(&g_display->panel->is_spot_ready, 0);
	err_fg_pending = false;

	proc_create(PANEL_REGISTER_RW, 0640, NULL, &panel_reg_rw_ops);
	proc_create(PANEL_VENDOR_ID, 0640, NULL, &panel_vendor_id_ops);
	proc_create(PANEL_FPS, 0666, NULL, &panel_fps_ops);
	proc_create(LCD_UNIQUE_ID, 0444, NULL, &lcd_unique_id_ops);
	proc_create(HBM_MODE, 0666, NULL, &hbm_mode_ops);
	proc_create(GLOBAL_HBM_MODE, 0666, NULL, &global_hbm_mode_ops);
	proc_create(DIMMING_SPEED, 0666, NULL, &dimming_speed_ops);
	proc_create(LCD_BACKLIGNTNESS, 0666, NULL, &lcd_brightness_ops);
}

// to parse panel_vendor_id from panel dtsi
void dsi_ai2201_parse_panel_vendor_id(struct dsi_panel *panel)
{
	struct dsi_parser_utils *utils = &panel->utils;

	panel->panel_vendor_id = utils->get_property(utils->data,
			"qcom,mdss-dsi-panel-vendor-id", NULL);
	DSI_LOG("panel vendor id = %s", panel->panel_vendor_id);
}

// to store panel status & reset display parameter
void dsi_ai2201_set_panel_is_on(bool on)
{
	g_display->panel->panel_is_on = on;

	if (on == false) {
		g_display->panel->panel_hbm_mode = 0;
		g_display->panel->panel_fod_hbm_mode = 0;
		g_display->panel->allow_panel_fod_hbm = 0;
		g_display->panel->allow_fod_hbm_process = false;
		g_display->panel->aod_state = false;
		g_display->panel->aod_mode = -1;
		g_display->panel->panel_aod_last_bl = 0;
		g_display->panel->panel_bl_count = 0;
		g_display->panel->fod_in_doze = false;
		g_display->panel->err_fg_irq_is_on = false;
		has_fod_masker = false;
		old_has_fod_masker = false;
		has_fod_spot = false;
		old_has_fod_spot = false;
		ai2201_drm_notify(ASUS_NOTIFY_SPOT_READY, 0);
		ai2201_drm_notify(ASUS_NOTIFY_GHBM_ON_READY, 0);
		ai2201_drm_notify(ASUS_NOTIFY_GHBM_ON_REQ, 0);
		g_display->panel->is_gamma_change = false;
		g_display->panel->aod_delay = false;
		display_panel_clear_dc();
		g_display->panel->csc_mode = 0;
		atomic_set(&g_display->panel->is_spot_ready, 0);
	}
}

static void dsi_ai2201_aod_backlight(struct dsi_panel *panel)
{
	int rc = 0;
	if (!panel) {
		DSI_LOG("invalid params\n");
		return;
	}

	// skip if fod hbm is processing
	if (panel->aod_state && !(panel->allow_panel_fod_hbm || panel->allow_fod_hbm_process)) {
		// for bl=61 & bl=1
		if (panel->panel_last_backlight == 61 || panel->panel_last_backlight == 9) {
			if (panel->panel_last_backlight == 61 && panel->panel_last_backlight != panel->panel_aod_last_bl) {
				DSI_LOG("set AOD HIGH command\n");
				rc = dsi_ai2201_tx_cmd_set(panel, DSI_CMD_SET_AOD_HIGH);
				panel->panel_aod_last_bl = panel->panel_last_backlight;
				panel->aod_mode = 1;
			} else if (panel->panel_last_backlight == 9 && panel->panel_last_backlight != panel->panel_aod_last_bl) {
				DSI_LOG("set AOD LOW command\n");
				rc = dsi_ai2201_tx_cmd_set(panel, DSI_CMD_SET_AOD_LOW);
				panel->panel_aod_last_bl = panel->panel_last_backlight;
				panel->aod_mode = 0;
			}
		} else { // to prevent display off
			DSI_LOG("set AOD Other command\n");
			rc = dsi_ai2201_tx_cmd_set(panel, DSI_CMD_SET_AOD_OTHER);
		}

		if (rc)
			DSI_LOG("unable to set AOD command\n");
	}
}

// to record & restore user's last backlight
void dsi_ai2201_record_backlight(u32 bl_lvl)
{
	if (bl_lvl == 0)
		return;

	g_display->panel->panel_last_backlight = bl_lvl;
	//#define SDE_MODE_DPMS_LP1	1      sde_drm.h
	//#define SDE_MODE_DPMS_LP2	2

	if ((g_display->panel->power_mode == 1) ||
		(g_display->panel->power_mode == 2)) {
		dsi_ai2201_aod_backlight(g_display->panel);
	}
}

void dsi_ai2201_restore_backlight(void)
{
	int rc = 0;

	DSI_LOG("restore bl=%d\n", g_display->panel->panel_last_backlight);
	rc = dsi_panel_set_backlight(g_display->panel, g_display->panel->panel_last_backlight);
	if (rc)
		DSI_LOG("unable to set backlight\n");
}

// called from sde_crtc_atomic_set_property, ready for FOD ON/OFF
void ai2201_crtc_fod_masker_spot(struct drm_crtc *crtc, int idx, uint64_t val)
{
	if (!strcmp(crtc->name, "crtc-0")) {
		switch (idx) {
		case CRTC_PROP_FOD_MASKER:
			has_fod_masker = val;
			if (old_has_fod_masker == false && has_fod_masker == true) {
				g_display->panel->allow_panel_fod_hbm = 1;
				g_display->panel->allow_fod_hbm_process = true;
				DSI_LOG("FOD:OFF->ON");
			} else if (old_has_fod_masker == true && has_fod_masker == false) {
				g_display->panel->allow_panel_fod_hbm = 0;
				g_display->panel->allow_fod_hbm_process = true;
				DSI_LOG("FOD:ON->OFF");
			}
			old_has_fod_masker = has_fod_masker;
			break;
		case CRTC_PROP_FOD_SPOT:
			has_fod_spot = val;
			if (old_has_fod_spot == false && has_fod_spot == true) {
				DSI_LOG("FOD SPOT:OFF->ON");
			} else if (old_has_fod_spot == true && has_fod_spot == false) {
				DSI_LOG("FOD SPOT:ON->OFF");
			}
			old_has_fod_spot = has_fod_spot;
			break;
		default:
			break;
		}
	}
}

// called from sde_crtc_commit_kickoff, to enable panel global HBM
void ai2201_crtc_display_commit(struct drm_encoder *encoder, struct drm_crtc *crtc)
{
	if (g_display->panel->allow_fod_hbm_process && !strcmp(crtc->name, "crtc-0")) {
		DSI_LOG("FOD HBM setting +++\n");
		sde_encoder_wait_for_event(encoder, MSM_ENC_VBLANK);
		display_set_fod_hbm();

		if (g_display->panel->allow_panel_fod_hbm == 1) {
			// need delay time, waiting for fine tune
			ai2201_drm_notify(ASUS_NOTIFY_GHBM_ON_READY, 1);
			g_display->panel->panel_fod_hbm_mode = 1;
			DSI_LOG("panel_fod_hbm_mode set to 1");
		} else if (g_display->panel->allow_panel_fod_hbm == 0) {
			// need delay time, waiting for fine tune
			ai2201_drm_notify(ASUS_NOTIFY_GHBM_ON_READY, 0);
			g_display->panel->panel_fod_hbm_mode = 0;
			DSI_LOG("panel_fod_hbm_mode set to 0");
		}
		DSI_LOG("FOD HBM setting ---\n");
	}
}

// add for spot ready after hbm ramping to the highest
// type 1:from first spot; 2:from crtc
static void dsi_ai2201_set_notify_spot_ready(int type)
{
	uint32_t delta_us = 0;
	uint32_t frame_us = 0;
	uint32_t delay = 0;
	int frame_delay = 2;
	ktime_t ktime1;
	bool is_ready = false;

	if (!display_panel_valid())
		return;

	// for spot layer is ready
	if (!atomic_read(&g_display->panel->is_spot_ready))
		return;

	ktime1 = ktime_get(); //current frame time
	delta_us = (u32)ktime_to_us(ktime1) - (u32)ktime_to_us(g_display->panel->ktime0);
	frame_us = 1000000 / g_display->panel->cur_mode->timing.refresh_rate;
	is_ready = (delta_us >= frame_us) ? true : false;

	// fod_spot_ui_ready represent to SYS NODE
	if (is_ready && !fod_spot_ui_ready) {
		// spec is 1 frame, but fine tune is 2 frames
		if (delta_us < (frame_delay*frame_us) && type == 2) {
			delay = ((frame_delay*frame_us) - delta_us);
			usleep_range(delay, delay);
		}
		DSI_LOG("tooks %d (us) ---", (delta_us+delay));
		ai2201_drm_notify(ASUS_NOTIFY_SPOT_READY, 1);
	}
}

// called from sde_crtc.c
void ai2201_set_notify_spot_ready(struct drm_crtc *crtc) {

	if (strcmp(crtc->name, "crtc-0"))
		return;

	dsi_ai2201_set_notify_spot_ready(2);
}

// called from _msm_drm_commit_work_cb, to notify spot ready
// type 0: commit_for_fod_spot
//      1: report_fod_spot_disappear
bool ai2201_atomic_get_spot_status(int type)
{
	if (type == 0) {
		if (has_fod_spot && !atomic_read(&g_display->panel->is_spot_ready)) {
			DSI_LOG("commit FOD spot to panel +++ \n");
			return true;
		}
	} else if (type == 1) {
		if (!has_fod_spot && atomic_read(&g_display->panel->is_spot_ready))
			return true;
	}

	return false;
}

void ai2201_atomic_set_spot_status(int type)
{
	int rc = 0;

	if (type == 0) {
		int period_ms = 1000000 / g_display->panel->cur_mode->timing.refresh_rate;
		DSI_LOG("commit FOD spot to panel (%d) --- \n", period_ms);

		// display on after ASUS_NOTIFY_SPOT_READY
		if (g_display->panel->fod_in_doze) {
			rc = dsi_ai2201_tx_cmd_set(g_display->panel, DSI_CMD_SET_POST_FOD_HBM_ON);
			if (rc) {
				DSI_LOG("[%s] failed to send DSI_CMD_SET_POST_FOD_HBM_ON cmd, rc=%d\n",
					   g_display->panel->name, rc);
			} else {
				g_display->panel->fod_in_doze = false;
				atomic_set(&g_display->panel->is_spot_ready, 1);
				//ai2201_drm_notify(ASUS_NOTIFY_SPOT_READY, 1);
			}
		} else {
			atomic_set(&g_display->panel->is_spot_ready, 1);
			//ai2201_drm_notify(ASUS_NOTIFY_SPOT_READY, 1);
		}
	} else if (type == 1) {
		ai2201_drm_notify(ASUS_NOTIFY_SPOT_READY, 0);
		atomic_set(&g_display->panel->is_spot_ready, 0);
		DSI_LOG("removed fod spot \n");
	}

	dsi_ai2201_set_notify_spot_ready(1);
}

void dsi_ai2201_need_aod_reset(struct dsi_panel *panel)
{
	int rc = 0;

	mutex_lock(&panel->panel_lock);
	if (!panel->panel_initialized)
		goto exit;

	if(panel->panel_last_backlight == 9 || !panel->aod_mode) {
		DSI_LOG("notify set AOD LOW command\n");
		rc = dsi_ai2201_tx_cmd_set(panel, DSI_CMD_SET_AOD_LOW);
		panel->aod_mode = 0;
	} else if(panel->panel_last_backlight == 61 || panel->aod_mode) {
		DSI_LOG("notify set AOD HIGH command\n");
		rc = dsi_ai2201_tx_cmd_set(panel, DSI_CMD_SET_AOD_HIGH);
		panel->aod_mode = 1;
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

// to validate skip igc or not
// to allow set bl if is_bl_ready count to 2
static bool dsi_ai2201_validate_c2_last(u32 c2_last)
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
bool ai2201_need_skip_data(u32 c2_last)
{
	bool rc = false;

	// need to update igc when entering doze mode
	if (g_display->panel->aod_state)
		return rc;

	// don't validate if no dc change
	if (!atomic_read(&g_display->panel->is_dc_change))
		return rc;

	rc = dsi_ai2201_validate_c2_last(c2_last);
	DSI_LOG("igc=%s\n", rc?"skip":"apply");
	return rc;
}

void ai2201_store_c2_last(u32 c2_last)
{
	g_display->panel->c2_last = c2_last;
}

static void dsi_ai2201_dc_bl_set(void)
{
	atomic_set(&g_display->panel->allow_bl_change, 1);
	dsi_ai2201_restore_backlight();
	display_panel_clear_dc();
}

// called every frame commit
void ai2201_set_dc_bl_process(struct drm_encoder *encoder, struct drm_crtc *crtc)
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
	dsi_ai2201_dc_bl_set();

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

bool get_charger_mode(void) {
	return g_Charger_mode;
}

module_param_string(LCDUID, g_lcd_unique_id, 10,
								0600);
#endif
