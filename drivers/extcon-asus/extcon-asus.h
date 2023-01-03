/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_EXTCON_INTERNAL_H__
#define __LINUX_EXTCON_INTERNAL_H__

#include <linux/extcon-provider.h>

/**
 * struct extcon_asus_dev - An extcon device represents one external connector.
 * @name:		The name of this extcon device. Parent device name is
 *			used if NULL.
 * @dev:		Device of this extcon.
 * @state:		Attach/detach state of this extcon. Do not provide at
 *			register-time.
 * @lock:
 */
struct extcon_asus_dev {
	/* Optional user initializing data */
	const char *name;

	/* Internal data. Please do not set. */
	struct device dev;
	spinlock_t lock;	/* could be called by irq handler */
	u32 state;
};

#endif /* __LINUX_EXTCON_INTERNAL_H__ */
