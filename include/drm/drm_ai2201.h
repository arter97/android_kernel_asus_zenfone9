/*
 * Copyright (c) 2020, ASUS. All rights reserved.
 */
#ifndef _DRM_AI2201_H_
#define _DRM_AI2201_H_

#ifdef ASUS_AI2201_PROJECT
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/workqueue.h>
#define ASUS_NOTIFY_GHBM_ON_REQ        0
#define ASUS_NOTIFY_GHBM_ON_READY      1
#define ASUS_NOTIFY_SPOT_READY         2
#define ASUS_NOTIFY_FOD_TOUCHED        3

#ifndef ASUS_GKI_BUILD
void ai2201_drm_notify(int var, int value);
#else
static inline void ai2201_drm_notify(int var, int value) {}
#endif
void drm_ai2201_sysfs_destroy(void);
int drm_ai2201_sysfs_init(void);
bool is_DSI_mode(int,int);
bool refreshrate_match(int,int);

#else
static inline void ai2201_drm_notify(int var, int value) {}
static inline void drm_ai2201_sysfs_destroy(void) {}
static inline int drm_ai2201_sysfs_init(void) { return 0; }
static inline bool is_DSI_mode(int vdisplay, int vtotal) { return false; }
static inline bool refreshrate_match(int refresh1, int refresh2) { return true; }

#endif
#endif /* _DRM_AI2201_H_ */
