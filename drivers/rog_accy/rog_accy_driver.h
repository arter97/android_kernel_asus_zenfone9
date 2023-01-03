#include <linux/fcntl.h>
//#include <stdio.h>
//#include <stdlib.h>
#include <linux/unistd.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>
#include <linux/string.h>
#include <linux/syscalls.h>
#include <linux/pinctrl/consumer.h>
#include <linux/mutex.h>
#include <linux/hid.h>
#include <linux/semaphore.h>

//For HID wait for completion
#include <linux/completion.h>

// USB suspend function
//extern void usb_enable_autosuspend(struct usb_device *);
//extern void usb_disable_autosuspend(struct usb_device *);

#define	CLASS_NAME		    "ec_hid"
//#define HID_PATCH			"/dev/hidraw0"

//Include extcon register
#include <linux/extcon.h>
#include <../extcon-asus/extcon-asus.h>
static struct extcon_asus_dev	*accy_extcon;

extern int asus_extcon_set_state_sync(struct extcon_asus_dev *edev, int cable_state);
extern struct extcon_asus_dev *extcon_asus_dev_allocate(void);
extern int extcon_asus_dev_register(struct extcon_asus_dev *edev);

#ifdef ASUS_AI2201_PROJECT
extern void Inbox_role_switch(int enable);
bool FANDG_USBID_detect = false;
EXPORT_SYMBOL(FANDG_USBID_detect);
bool FANDG_PDID_detect = false;
EXPORT_SYMBOL(FANDG_PDID_detect);
#endif

enum asus_dongle_type
{
	Dongle_NO_INSERT = 0,
	Dongle_INBOX5,
	Dongle_Station2,
	Dongle_DT1,
	Dongle_PCIE,
	Dongle_ERROR,
	Dongle_Others,
	Dongle_BackCover,
	Dongle_FANDG6,
	Dongle_default_status = 255,
};

enum asus_dock_event
{
	EXTRA_DOCK_STATE_UNDOCKED = 0,
	EXTRA_DOCK_STATE_ASUS_INBOX = 6,
	EXTRA_DOCK_STATE_ASUS_STATION =7,
	EXTRA_DOCK_STATE_ASUS_DT = 8,
	EXTRA_DOCK_STATE_ASUS_OTHER = 9,
/*
	EXTRA_DOCK_STATE_KEYBOARD = 10,
	EXTRA_DOCK_STATE_ASUS_PRODONGLE = 13,
	EXTRA_DOCK_STATE_ASUS_GAMEVICE = 14,
	EXTRA_DOCK_STATE_PD_GV = 15,
	EXTRA_DOCK_STATE_STATION_LOCK = 16,
	EXTRA_DOCK_STATE_STATION_LOW_BATTERY = 17,
	EXTRA_DOCK_STATE_STATION_UNLOCK = 18,
	EXTRA_DOCK_STATE_INBOX_INSERT_ERROR = 19,
	EXTRA_DOCK_STATE_STATION_INSERT_ERROR = 20,
	EXTRA_DOCK_STATE_GAMEVICE_INSERT_ERROR = 21,
	EXTRA_DOCK_STATE_STATION_INSERT_ERROR = 20,
	EXTRA_DOCK_STATE_GAMEVICE_INSERT_ERROR = 21,
	EXTRA_DOCK_STATE_PD_GV_INSERT_ERROR = 22,
*/
	EXTRA_DOCK_STATE_BACKGROUND_PAD = 23,
	EXTRA_DOCK_STATE_ASUS_2nd_INBOX = 26,
};

static struct class *rog_accy_class;

static struct rog_accy_data *g_rog_accy_data;
//EXPORT_SYMBOL(g_rog_accy_data);

struct rog_accy_data {
	dev_t devt;
	struct device *dev;

	int vph_ctrl;

	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	//struct pinctrl_state *pins_suspend;

	bool lock;
	struct mutex report_mutex;
	struct semaphore pogo_sema;
};
