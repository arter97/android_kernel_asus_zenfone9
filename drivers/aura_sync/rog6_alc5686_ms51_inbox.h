#include <linux/leds.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/syscalls.h>
#include <linux/pinctrl/consumer.h>
#include <linux/mutex.h>
#include <linux/kernel.h>

#define RGB_MAX 21   //for rainbow color setting
#define FEATURE_WRITE_COMMAND_SIZE 6
#define FEATURE_READ_COMMAND_SIZE 3
#define FEATURE_WRITE_LONG_COMMAND_SIZE 52
#define FEATURE_READ_LONG_COMMAND_SIZE 14

struct inbox_drvdata {
	struct led_classdev led;
};

//extern bool g_Charger_mode;
static bool g_Charger_mode=false;

struct mutex ms51_mutex;
struct mutex update_lock;

struct hidraw *rog6_inbox_hidraw;
EXPORT_SYMBOL_GPL(rog6_inbox_hidraw);

extern int usbhid_set_raw_report(struct hid_device *hid, unsigned int reportnum,
                                __u8 *buf, size_t count, unsigned char rtype);
extern int usbhid_get_raw_report(struct hid_device *hid, unsigned char report_number, __u8 *buf, size_t count,
               unsigned char report_type);

extern void FANDG_connect(int val);
extern bool FANDG_USBID_detect;

// Choose I2C addr
static u8 IC_switch;

enum i2c_address_list
{
	addr_0x16 = 1,
	addr_0x18 = 2,
	addr_0x75 = 3,
	addr_0x40 = 4,
	addr_ohter = 255,
};

static u8 g_led_on = 0;
static u8 g_door_on = 0;
static u8 g_logo_on = 0;
static u8 g_cooling_en = 0;
static u8 g_cooling_stage = 0;
static u8 g_TH_meaure = 0;

//  For 2Leds MS51
static u8 g_2led_mode = 0;
static u8 g_2led_mode2 = 0;
static u8 g_2led_apply = 0;
static u8 key_state = 0;

static u32 g_2led_red_max;
static u32 g_2led_green_max;
static u32 g_2led_blue_max;
static u32 g_2led_red;
static u32 g_2led_green;
static u32 g_2led_blue;
static u32 g_2led_speed;

//  For 3Leds MS51
static u8 g_3led_mode = 0;
static u8 g_3led_mode2 = 0;
static u8 g_3led_apply = 0;

static u32 g_3led_red_max;
static u32 g_3led_green_max;
static u32 g_3led_blue_max;
static u32 g_3led_red;
static u32 g_3led_green;
static u32 g_3led_blue;
static u32 g_3led_speed;
