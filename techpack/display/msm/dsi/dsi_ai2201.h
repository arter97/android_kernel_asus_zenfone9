/*
 * Copyright (c) 2020, ASUS. All rights reserved.
 */

#ifndef _DSI_AI2201_H_
#define _DSI_AI2201_H_

#include "dsi_display.h"

#ifdef ASUS_AI2201_PROJECT

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/workqueue.h>

/* for panel_reg_rw parameters */
#define ASUS_NOTIFY_GHBM_ON_REQ        0
#define ASUS_NOTIFY_GHBM_ON_READY      1
#define ASUS_NOTIFY_SPOT_READY         2
#define ASUS_NOTIFY_FOD_TOUCHED        3
#define MIN_LEN 2
#define MAX_LEN 4
#define REG_BUF_SIZE 4096
#define PANEL_REGISTER_RW        "driver/panel_reg_rw"
#define PANEL_VENDOR_ID          "driver/panel_vendor_id"
#define PANEL_FPS                "driver/panel_fps"
#define LCD_UNIQUE_ID            "lcd_unique_id"
#define HBM_MODE                 "hbm_mode"
#define GLOBAL_HBM_MODE          "globalHbm"
#define DIMMING_SPEED            "lcd_dimming_speed"
#define LCD_BACKLIGNTNESS        "lcd_brightness"
#define MIPI_DSI_MSG_CMD_READ BIT(8)

/* for frame commit count */
#define COMMIT_FRAMES_COUNT 5

u32 dsi_ai2201_support_cmd_read_flags(u32 flags);
void dsi_ai2201_set_dimming_smooth(struct dsi_panel *panel, u32 backlight);
void dsi_ai2201_clear_commit_cnt(void);
void dsi_ai2201_frame_commit_cnt(struct drm_crtc *crtc);
void dsi_ai2201_display_init(struct dsi_display *display);
void dsi_ai2201_parse_panel_vendor_id(struct dsi_panel *panel);
void dsi_ai2201_set_panel_is_on(bool on);
void ai2201_set_err_fg_irq_state(bool state);
bool ai2201_get_err_fg_irq_state(void);
void ai2201_set_err_fg_pending(bool state);
bool ai2201_get_err_fg_pending(void);
void dsi_ai2201_record_backlight(u32 bl_lvl);
u32 dsi_ai2201_backlightupdate(u32 bl_lvl);
void dsi_ai2201_restore_backlight(void);
void dsi_ai2201_need_aod_reset(struct dsi_panel *panel);
void ai2201_crtc_fod_masker_spot(struct drm_crtc *crtc, int idx, uint64_t val);
void ai2201_crtc_display_commit(struct drm_encoder *encoder, struct drm_crtc *crtc);
void ai2201_set_notify_spot_ready(struct drm_crtc *crtc);
bool ai2201_atomic_get_spot_status(int type);
void ai2201_atomic_set_spot_status(int type);
bool ai2201_need_skip_data(u32 c2_last);
void ai2201_store_c2_last(u32 c2_last);
void ai2201_set_dc_bl_process(struct drm_encoder *encoder, struct drm_crtc *crtc);

void ai2201_drm_notify(int var, int value);
bool is_DSI_mode(int,int);
bool refreshrate_match(int,int);
bool get_charger_mode(void);

#else

static inline u32 dsi_ai2201_support_cmd_read_flags(u32 flags){ return 0; }
static inline void dsi_ai2201_set_dimming_smooth(struct dsi_panel *panel, u32 backlight) {}
static inline void dsi_ai2201_clear_commit_cnt(void) {}
static inline void dsi_ai2201_frame_commit_cnt(struct drm_crtc *crtc) {}
static inline void dsi_ai2201_display_init(struct dsi_display *display) {}
static inline void dsi_ai2201_parse_panel_vendor_id(struct dsi_panel *panel) {}
static inline void dsi_ai2201_set_panel_is_on(bool on) {}
static inline void ai2201_set_err_fg_irq_state(bool state){}
static inline bool ai2201_get_err_fg_irq_state(void){return false;}
static inline void ai2201_set_err_fg_pending(bool state){}
static inline bool ai2201_get_err_fg_pending(void){return false;}
static inline void dsi_ai2201_record_backlight(u32 bl_lvl) {}
static inline u32 dsi_ai2201_backlightupdate(u32 bl_lvl) {return bl_lvl;}
static inline void dsi_ai2201_restore_backlight(void) {}
static inline void dsi_ai2201_need_aod_reset(void) {}
static inline void ai2201_crtc_fod_masker_spot(struct drm_crtc *crtc, int idx, uint64_t val) {}
static inline void ai2201_crtc_display_commit(struct drm_encoder *encoder, struct drm_crtc *crtc) {}
static inline void ai2201_set_notify_spot_ready(struct drm_crtc *crtc) {}
static inline bool ai2201_atomic_get_spot_status(int type) { return false; }
static inline void ai2201_atomic_set_spot_status(int type) {}
static inline bool ai2201_need_skip_data(u32 c2_last) { return false; }
static inline void ai2201_store_c2_last(u32 c2_last) {}
static inline void ai2201_set_dc_bl_process(struct drm_encoder *encoder, struct drm_crtc *crtc) {}

static inline void ai2201_drm_notify(int var, int value) {}
static inline bool is_DSI_mode(int vdisplay, int vtotal) { return false; }
static inline bool refreshrate_match(int refresh1, int refresh2) { return true; }
static inline bool get_charger_mode(void) { return false; }

#endif
#endif /* _DSI_AI2201_H_ */
