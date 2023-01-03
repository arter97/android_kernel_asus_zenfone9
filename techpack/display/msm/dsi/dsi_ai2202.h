/*
 * Copyright (c) 2021, ASUS. All rights reserved.
 */

#ifndef _DSI_AI2202_H_
#define _DSI_AI2202_H_

#include "dsi_display.h"

#if defined ASUS_AI2202_PROJECT

#define LCD_UNIQUE_ID            "lcd_unique_id"
#define PANEL_FPS                "driver/panel_fps"
#define MIN_LEN 2
#define MAX_LEN 4
#define REG_BUF_SIZE 4096
#define PANEL_REGISTER_RW        "driver/panel_reg_rw"
#define PANEL_VENDOR_ID          "driver/panel_vendor_id"
#define MIPI_DSI_MSG_CMD_READ BIT(8)
#define HBM_MODE                 "hbm_mode"
#define COMMIT_FRAMES_COUNT 5
#define DIMMING_SPEED            "lcd_dimming_speed"
#define LCD_BACKLIGNTNESS        "lcd_brightness"

u32 dsi_ai2202_support_cmd_read_flags(u32 flags);
void dsi_ai2202_set_dimming_smooth(struct dsi_panel *panel, u32 backlight);
void dsi_ai2202_display_init(struct dsi_display *display);
void dsi_ai2202_parse_panel_vendor_id(struct dsi_panel *panel);
void dsi_ai2202_set_panel_is_on(bool on);
void dsi_ai2202_record_backlight(u32 bl_lvl);
u32 dsi_ai2202_backlightupdate(u32 bl_lvl);
void dsi_ai2202_frame_commit_cnt(struct drm_crtc *crtc);
void dsi_ai2202_clear_commit_cnt(void);
bool ai2202_need_skip_data(u32 c2_last);
void ai2202_store_c2_last(u32 c2_last);
void ai2202_set_dc_bl_process(struct drm_encoder *encoder, struct drm_crtc *crtc);
void dsi_ai2202_need_aod_reset(struct dsi_panel *panel);

#else

static inline u32 dsi_ai2202_support_cmd_read_flags(u32 flags){ return 0; }
static inline void dsi_ai2202_set_dimming_smooth(struct dsi_panel *panel, u32 backlight) {}
static inline void dsi_ai2202_display_init(struct dsi_display *display) {}
static inline void dsi_ai2202_parse_panel_vendor_id(struct dsi_panel *panel) {}
static inline void dsi_ai2202_set_panel_is_on(bool on) {};
static inline u32 dsi_ai2202_backlightupdate(u32 bl_lvl) {return bl_lvl;}
static inline void dsi_ai2202_record_backlight(u32 bl_lvl) {}
static inline void dsi_ai2202_frame_commit_cnt(struct drm_crtc *crtc) {}
static inline void dsi_ai2202_clear_commit_cnt(void) {}
static inline bool ai2202_need_skip_data(u32 c2_last) { return false; }
static inline void ai2202_store_c2_last(u32 c2_last) {}
static inline void ai2202_set_dc_bl_process(struct drm_encoder *encoder, struct drm_crtc *crtc) {}
static inline void dsi_ai2202_need_aod_reset(struct dsi_panel *panel) {}

#endif

#endif /* _DSI_ZF9_H_ */
