#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#define CONFIG_I2C_MODULE
#include <linux/i2c.h>
#include "device.h"
#include "memory.h"
#include "serial_bus.h"
#include "main.h"
#include "event.h"
#include "hardware.h"
#include "sysfs.h"
#include "utils.h"
#include "config.h"
#include "debug.h"
#include "sonacomm.h"
#include "workqueue.h"
#include "irq.h"
#include "locking.h"
#include "customize.h"
#include "file_control.h"

void grip_enable_func_noLock(int val);

void Wait_Wake_For_RegW(void);
void Into_DeepSleep_fun(void);
void grip_dump_status_func(struct work_struct *work);
void Reset_Func(struct work_struct *work);
static int Health_Check_Enable_No_Delay(int en);
//static void Enable_tap_sensitive(const char *buf, size_t count);
//Check grip all gesture status, 1: enable, 0:disable
static int grip_all_gesture_status(void);

//Check grip game gesture (tap/slide/swipe) status, 1: enable, 0:disable
int grip_game_gesture_status(void);
static int grip_tap_gesture_status(void);
static int grip_swipe_gesture_status(void);
static int grip_slide_gesture_status(void);

//report event to sensor hal when chip reset occurs
extern void grip_input_event_report(int g_id, int len, int trk_id, int bar_id, int force, int fr_nr, int center);

/* ASUS BSP Clay: setting vibrator trig1 & trig2 register for different tap
modes +++ */
extern int G_grip_tap_vib1_reg;
extern int G_grip_tap_vib2_reg;
static void gripVibratorSetting(int val, int tap_id);
/* ASUS BSP Clay--- */

#ifdef DYNAMIC_PWR_CTL
extern int snt_activity_request(void);
#endif

extern void snt_set_pinctrl(struct device *dev, char *str);
struct delayed_work check_stuck_wake;
struct workqueue_struct *asus_wq;
extern struct delayed_work event_wq;
/*
	param. fw_loading_status:
	0: when charger/recorver or the other mode, grip fw will load fail
	1: load fw success
int fw_loading_status = 0;
*/

/* init 1V2_2V8 power status */
static int g_snt_power_state = 1;

static uint32_t ms_start=0, ms_end=0;

/* init record helath check result */
uint16_t FPC_value = 0;

static uint16_t Tap_sense_data1[3] = {
		0x0003,
		0x0000,
		0x8062
};
static uint16_t Tap_sense_data2[3] = {
		0x0014,
		0x0000,
		0x804b
};

static uint16_t Tap_sense_data3[3] = {
		0x0028,
		0x0000,
		0x8053
};

static uint16_t Tap_sense_reset_data1[3] = {
		0x0032,
		0x0000,
		0x804b
};
static uint16_t Tap_sense_reset_data2[3] = {
		0x003c,
		0x0000,
		0x8053
};

static uint16_t Slide_Swipe_sense_data[3] = {
		0x0014,
		0x0000,
		0x804b
};

static uint16_t Slide_Swipe_sense_reset_data[3] = {
		0x0032,
		0x0000,
		0x804b
};

static uint16_t Swipe_sense_data2[3] = {
		0x8000,
		0x0000,
		0x8057
};

static uint16_t Swipe_sense_reset_data2[3] = {
		0x0400,
		0x0000,
		0x8057
};

static uint16_t sys_param_addr[3] = {
		0x42,
		0x43,
		0x44
};
extern int track_report_count;


/* after chip reset, recovery according to status which records from
property */
struct delayed_work rst_recovery_wk;

/* reset chip when i2c error or chip can't be waked up */
struct delayed_work rst_gpio_wk;

static int Grip_Check_FW_Status(void){
	if(snt8100fsr_g->chip_reset_flag != GRIP_FW_DL_END){
		mutex_unlock(&snt8100fsr_g->ap_lock);
		snt8100fsr_g->Recovery_Flag = 1;
		return 1;
	}else{
		return 0;
	}
}

void Grip_Chip_IRQ_EN(bool flag){
	uint16_t irq_val = 0;
	if(flag){
		irq_val = 0x1;
		PRINT_INFO("Enable Chip iRQ, Reg 0x1 = 0x%x", irq_val);
		write_register(snt8100fsr_g,
				REGISTER_ENABLE,
				&irq_val);
	}else{
		irq_val = 0x0;
		PRINT_INFO("Disable Chip IRQ, Reg 0x1 = 0x%x", irq_val);
		write_register(snt8100fsr_g,
				REGISTER_ENABLE,
				&irq_val);
	}
}
void Grip_Driver_IRQ_EN(bool flag){
	static bool irq_en = false;
	if(irq_en != flag){
		irq_en = flag;
		if(irq_en == true){
			PRINT_INFO("Enable Driver IRQ, %d", snt8100fsr_g->hostirq_gpio);
			enable_irq(gpio_to_irq(snt8100fsr_g->hostirq_gpio));
		}else{
			PRINT_INFO("Disable Driver IRQ");
			disable_irq_nosync(gpio_to_irq(snt8100fsr_g->hostirq_gpio));
		}
	}
}

/* for squeeze/tap cancel missing when grip reset */
extern int grip_write_fail_count;
void Reset_Func(struct work_struct *work){
	PRINT_INFO("status: %d, failed_count=%d",
	snt8100fsr_g->grip_fw_loading_status, snt8100fsr_g->fw_failed_count);
	if(snt8100fsr_g->grip_fw_loading_status == false && FW_FAIL_REDOWNLOAD_LIMIT < snt8100fsr_g->fw_failed_count){
		PRINT_INFO("1. load continue or 2. fw load fail at init, chip broken case, don't retry, count=%d", snt8100fsr_g->fw_failed_count);
		Power_Control(0);
		return;
	}
	if(snt8100fsr_g->chip_reset_flag == GRIP_RST_FW_DL){
		PRINT_INFO("Chip reset is ongoing, skip request");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	if(snt8100fsr_g->chip_reset_flag == GRIP_FW_DL_END){
		PRINT_INFO("Starting chip reset");
#ifdef GRIP_APPLY_ASUSEVTLOG
		ASUSEvtlog("[Grip] Workaround : reset chip\n");
#endif
		snt8100fsr_g->chip_reset_flag = GRIP_RST_FW_DL;
		snt8100fsr_g->grip_fw_loading_status = false; /* reset fw_loading status */
		grip_write_fail_count = 0; /* reset i2c write failed count */
		snt8100fsr_g->snt_state = GRIP_WAKEUP;
		Grip_Driver_IRQ_EN(1);
		Power_Control(1);
	}
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void check_stuck_semaphore(struct work_struct *work){
	int ret;
	/* prevent chip reset fail */
	if(snt8100fsr_g->chip_reset_flag != GRIP_FW_DL_END){
		workqueue_cancel_work(&check_stuck_wake);
		PRINT_INFO("Don't wake chip during chip reset & long fw");
		snt8100fsr_g->stuck_retry_count = 0;
		return;
	}
	if(snt8100fsr_g->stuck_flag == true){
		PRINT_INFO("Used to solve wailting semaphore!!! retry times = %d",
		snt8100fsr_g->stuck_retry_count);
		/* when retry occurs, check listened gpio status */
		PRINT_INFO("GPIO: %d=%d", IRQ_GPIO, gpio_get_value(IRQ_GPIO));
		MUTEX_LOCK(&snt8100fsr_g->sb_lock);
		ret = sb_wake_device(snt8100fsr_g);
		mutex_unlock(&snt8100fsr_g->sb_lock);
		if (ret) {
			PRINT_CRIT("sb_wake_device() failed");
		}
		//retry check
		if(snt8100fsr_g->stuck_retry_count < snt8100fsr_g->stuck_retry_limit){
			workqueue_cancel_work(&check_stuck_wake);
			workqueue_queue_work(&check_stuck_wake, 200);
			snt8100fsr_g->stuck_retry_count++;
		}else{
#ifdef GRIP_APPLY_ASUSEVTLOG
			ASUSEvtlog("[Grip] driver is failed to wait semaphore due to non-wakeable chip\n");
#endif
			up(&snt8100fsr_g->wake_rsp);
			if (down_trylock(&snt8100fsr_g->wake_req)){
				PRINT_INFO("Wake Req alread consumed");
				if(down_trylock(&snt8100fsr_g->wake_rsp)){
					PRINT_INFO("Wake Rsq alread consumed");
				}
			}
			//queue_delayed_work(asus_wq, &rst_gpio_wk, msecs_to_jiffies(0));
		}
	}else{
		PRINT_INFO("None used");
		snt8100fsr_g->stuck_retry_count = 0;
		workqueue_cancel_work(&check_stuck_wake);
	}
}
static int Health_Check_Enable_No_Delay(int en){
	uint16_t En_fpc;
	int ret;
	ret = read_register(snt8100fsr_g, 0x003d, &En_fpc);
	if(ret < 0) {
		PRINT_ERR("Read Reg 3d Failed");
		return -1;
	}
	//PRINT_INFO("0x003d:%x ", En_fpc);
	if(en == 0){
		En_fpc = En_fpc | 0x0004;
	}else if(en ==1){
		En_fpc = En_fpc & 0xfffb;
	}else{
		PRINT_ERR("en=%d, out of 1 or 2 ", en);
		return -1;
	}

	ret = write_register(snt8100fsr_g, 0x003d, &En_fpc);
	if(ret < 0) {
		PRINT_ERR("Read Reg 3d Failed");
		return -1;
	}
	PRINT_INFO("Health Check EN=%d", en);
	return 0;
}

int Health_Check_Enable(int en){
	uint16_t En_fpc;
	int ret;
	ret = read_register(snt8100fsr_g, 0x003d, &En_fpc);
	if(ret < 0) {
		PRINT_ERR("Read Reg 3d Failed");
		return -1;
	}
	//PRINT_INFO("0x003d:%x ", En_fpc);
	if(en == 0){
		En_fpc = En_fpc | 0x0004;
	}else if(en ==1){
		En_fpc = En_fpc & 0xfffb;
	}else{
		PRINT_ERR("en=%d, out of 1 or 2 ", en);
		return -1;
	}

	ret = write_register(snt8100fsr_g, 0x003d, &En_fpc);
	if(ret < 0) {
		PRINT_ERR("Read Reg 3d Failed");
		return -1;
	}
	msleep(100);
	ret = read_register(snt8100fsr_g, 0x003d, &En_fpc);
	if(ret < 0) {
		PRINT_ERR("Read Reg 3d Failed");
		return -1;
	}
	PRINT_INFO("Health Check EN=%d", en);
	return 0;
}
int Health_Check(uint16_t val){
	int ret;
	uint16_t FPC_status;
	//Enable Health Check
	ret = Health_Check_Enable(0);
	ret = Health_Check_Enable(1);

	ret = read_register(snt8100fsr_g, REGISTER_PHY_STAT_LSB, &FPC_status);
	if(ret < 0) {
		PRINT_ERR("Read 0x03 Fail");
		Health_Check_Enable(0);
		return -1;
	}
	PRINT_INFO("0x03: 0x%x, expect: 0x%x", FPC_status, FPC_status |val);
	FPC_value = FPC_status;
	if (FPC_status != (FPC_status | val)) {
		PRINT_INFO("Health Check Fail!!!");
		Health_Check_Enable(0);
		return -1;
	}
	ret = Health_Check_Enable(0);
	return 0;
}
/*---BSP Clay proc asusGripDebug Interface---*/

void Into_DeepSleep_fun(void){
	int ret = 0;
	int frame_rate=65535;
	//int frame_rate=20;
	if(snt8100fsr_g->chip_reset_flag != GRIP_FW_DL_END){
		PRINT_INFO("skip during fw download");
	}else if(grip_status_g->G_EN==0 || grip_status_g->G_EN==-1){
		//Disable irq when driver requests chip into deep sleep mode
		//Grip_Chip_IRQ_EN(0);
		MUTEX_LOCK(&snt8100fsr_g->IRQ_WAKE_SLEEP_LOCK);
		snt8100fsr_g->snt_state = GRIP_DEEPSLEEP;
		workqueue_cancel_work(&event_wq);
		ret = write_register(snt8100fsr_g,
				REGISTER_FRAME_RATE,
				&frame_rate);
		if(ret < 0) {
			PRINT_ERR("Grip register_enable write fail");
		}else{
			PRINT_INFO("Grip_EN = %d, Grip_Frame = %d", grip_status_g->G_EN, frame_rate);
		}
		//msleep(10);
		Grip_Driver_IRQ_EN(0);
		ms_start = get_time_in_ms();
		Power_Control(2);
		mutex_unlock(&snt8100fsr_g->IRQ_WAKE_SLEEP_LOCK);
	}
}

/*
void Wake_device_func(void){
	int ret;
	MUTEX_LOCK(&snt8100fsr_g->sb_lock);
	ret = sb_wake_device(snt8100fsr_g);
	if (ret) {
		PRINT_CRIT("sb_wake_device() failed");
		mutex_unlock(&snt8100fsr_g->sb_lock);
		return;
	}
	mutex_unlock(&snt8100fsr_g->sb_lock);
	msleep(100);
}
*/
void Wait_Wake_For_RegW(void){
#ifdef DYNAMIC_PWR_CTL
	//sb_wake_device(snt8100fsr_g);
	if(snt8100fsr_g->grip_fw_loading_status == true){
		if (snt_activity_request() != 0) {
			PRINT_CRIT("snt_activity_request() failed");
		}
	}else{
		PRINT_INFO("Load FW Fail, skip wakeup request");
	}
#endif
}
/* write DPC function */
void DPC_write_func(int flag){
#ifdef DYNAMIC_PWR_CTL
	int ret;
	grip_status_g->G_DPC_STATUS = flag;
	if(flag == 1){
		if(grip_game_gesture_status()){
			PRINT_INFO("Don't Enable DPC since tap/slide/swipe enable");
		}else{
			PRINT_INFO("Enable DPC, write 0x%x, 0x%x, 0x%x",
				Grip_DPC_status_g->High, Grip_DPC_status_g->Low, Grip_DPC_status_g->Condition);
			ret = write_register(snt8100fsr_g, REGISTER_DPC_HIGH_FRAME, &Grip_DPC_status_g->High);
			if (ret) {
				PRINT_CRIT("set DPC 0x%x failed", REGISTER_DPC_HIGH_FRAME);
			}
			ret = write_register(snt8100fsr_g, REGISTER_DPC_LOW_FRAME, &Grip_DPC_status_g->Low);
			if (ret) {
				PRINT_CRIT("set DPC 0x%x failed", REGISTER_DPC_LOW_FRAME);
			}
			ret = write_register(snt8100fsr_g, REGISTER_DPC_CONDITION, &Grip_DPC_status_g->Condition);
			if (ret) {
				PRINT_CRIT("set DPC 0x%x failed", REGISTER_DPC_CONDITION);
			}
		}
	}else{
		PRINT_INFO("Disable DPC");
		ret = write_register(snt8100fsr_g, REGISTER_DPC_CONDITION, &flag);
		if (ret) {
			PRINT_CRIT("set DPC 0x%x failed", REGISTER_DPC_CONDITION);
		}
		ret = write_register(snt8100fsr_g, REGISTER_DPC_HIGH_FRAME, &flag);
		if (ret) {
			PRINT_CRIT("set DPC 0x%x failed", REGISTER_DPC_HIGH_FRAME);
		}
		ret = write_register(snt8100fsr_g, REGISTER_DPC_LOW_FRAME, &flag);
		if (ret) {
			PRINT_CRIT("set DPC 0x%x failed", REGISTER_DPC_LOW_FRAME);
		}
	}
#endif
}
void Check_Tap_sense_val(void){
	int i=0;
	uint32_t RegRead_t;
	msleep(50);
	for(i = 0; i < 3; i++){
		read_register(snt8100fsr_g, sys_param_addr[i], &RegRead_t);
		PRINT_INFO("Reg: 0x%x, Val: 0x%x", sys_param_addr[i], RegRead_t);
	}
}
void Tap_sense_write_func(int flag){
	int i = 0;
	if(grip_status_g->G_TAP_SENSE_EN != flag){
		grip_status_g->G_TAP_SENSE_EN = flag;
	}else{
		PRINT_INFO("TAP SENSE=%d, Already set before=======", flag);
		return;
	}
	if(flag == 1){
		PRINT_INFO("[Enable] Tap Sense data");
		for(i = 0; i < 3; i++){
			write_register(snt8100fsr_g, sys_param_addr[i], &Tap_sense_data1[i]);
		}
		Check_Tap_sense_val();
		for(i = 0; i < 3; i++){
			write_register(snt8100fsr_g, sys_param_addr[i], &Tap_sense_data2[i]);
		}
		Check_Tap_sense_val();

		for(i = 0; i < 3; i++){
			write_register(snt8100fsr_g, sys_param_addr[i], &Tap_sense_data3[i]);
		}
		Check_Tap_sense_val();

	}else{
		PRINT_INFO("[Disable] Tap Sense data");
		for(i = 0; i < 3; i++){
			write_register(snt8100fsr_g, sys_param_addr[i], &Tap_sense_reset_data1[i]);
		}
		Check_Tap_sense_val();
		for(i = 0; i < 3; i++){
			write_register(snt8100fsr_g, sys_param_addr[i], &Tap_sense_reset_data2[i]);
		}
		Check_Tap_sense_val();
	}
}

//Used in Tap function
static void grip_check_DPC_and_sensitivity_func(void){
	int ret = 0;

	/********* DPC part **********/
	if(grip_game_gesture_status()){
		//there exist tap which is on
		//Disable DPC
		if(grip_status_g->G_DPC_STATUS==1){
			snt8100fsr_g->frame_rate = 100;

			PRINT_INFO("game gesture enable, DPC turn off from on state and set frame_rate = %d",
			snt8100fsr_g->frame_rate);
			DPC_write_func(0);
			ret = write_register(snt8100fsr_g, REGISTER_FRAME_RATE, &snt8100fsr_g->frame_rate);
			if (ret) {
				PRINT_CRIT("write_register(REGISTER_FRAME_RATE) failed");
			}
		}else{
			PRINT_INFO("DPC already off");
		}
	}else{
		if(grip_status_g->G_DPC_STATUS==0){
			PRINT_INFO("Enable DPC when all taps disable");
			DPC_write_func(1);
		}
	}

	/********* sensitivity part **********/
	if(grip_status_g->G_TAP_EN[0] <= 0 && grip_status_g->G_TAP_EN[1] <= 0){
		//Disable tap sense
		if(grip_status_g->G_TAP_SENSE_SET == 1 && grip_status_g->G_TAP_SENSE_EN == 1){
			Tap_sense_write_func(0);
		}else{
			//Do nothing
		}
	}else if(grip_status_g->G_TAP_EN[0] == 1 && grip_status_g->G_TAP_EN[1] == 1){
		if(grip_status_g->G_TAP_SENSE_SET == 1 && grip_status_g->G_TAP_SENSE_EN <= 0){
			Tap_sense_write_func(1);
		}else{
			Tap_sense_write_func(0);
		}
	}else{
		//Do nothing
	}
}

static void grip_set_sys_param(const char *buf){
	int l = (int)strlen(buf) + 1;
	uint32_t val;
	uint32_t id;
	int status;
	int ret;
	PRINT_INFO("buf=%s, size=%d", buf, l);

	status  = ReadNumFromBuf((const uint8_t**)&buf, &l, &id);
	if (status != 0) {
		PRINT_CRIT("Could not parse param_id %d", status);
		goto errexit;
	}

	status = ReadNumFromBuf((const uint8_t**)&buf, &l, &val);
	if (status != 0) {
		PRINT_CRIT("Could not parse param_val %d", status);
		goto errexit;
	}
	if (enable_set_sys_param(snt8100fsr_g, id, val) != 0) {
		PRINT_DEBUG("send of set sys param failed");
		goto errexit;
	}

	// wait for response from driver irpt thread
	PRINT_DEBUG("SetSysParam Rsp -- wait");
	do {
		//ret= down_timeout(&snt8100fsr_g->sc_wf_rsp, msecs_to_jiffies(3*MSEC_PER_SEC);
		ret = down_interruptible(&snt8100fsr_g->sc_wf_rsp);
		PRINT_DEBUG("SetSysParam Rsp -- acquired %d", ret);
	} while (ret == -EINTR);

errexit:
    PRINT_DEBUG("done.");
    return;
}

/* ASUS BSP Clay: +++ swipe/slide used sense setting*/
static void grip_sense1_setting(bool flag){
	int i = 0;
	static bool status = false;
	if(status == flag){
		/* do nothing */
		return;
	}else{
		status = flag;
	}

	if(true == flag){
		for(i = 0; i < 3; i++){
			write_register(snt8100fsr_g, sys_param_addr[i], &Slide_Swipe_sense_data[i]);
		}
		PRINT_INFO("write 0x%x=0x%x, 0x%x, 0x%x",
			sys_param_addr[0], Slide_Swipe_sense_data[0],
			Slide_Swipe_sense_data[1], Slide_Swipe_sense_data[2]);
	}else{
		for(i = 0; i < 3; i++){
			write_register(snt8100fsr_g, sys_param_addr[i], &Slide_Swipe_sense_reset_data[i]);
		}
		PRINT_INFO("write 0x%x=0x%x, 0x%x, 0x%x",
			sys_param_addr[0], Slide_Swipe_sense_reset_data[0],
			Slide_Swipe_sense_reset_data[1], Slide_Swipe_sense_reset_data[2]);
	}
}
/* ASUS BSP Clay: ---*/
/* ASUS BSP Clay: +++ swipe used sense setting*/
static void grip_sense2_setting(bool flag){
	int i = 0;
	static bool status = false;
	if(status == flag){
		/* do nothing */
		return;
	}else{
		status = flag;
	}

	if(true == flag){
		for(i = 0; i < 3; i++){
			write_register(snt8100fsr_g, sys_param_addr[i], &Swipe_sense_data2[i]);
		}
		PRINT_INFO("write 0x%x=0x%x, 0x%x, 0x%x",
			sys_param_addr[0], Swipe_sense_data2[0],
			Swipe_sense_data2[1], Swipe_sense_data2[2]);
	}else{
		for(i = 0; i < 3; i++){
			write_register(snt8100fsr_g, sys_param_addr[i], &Swipe_sense_reset_data2[i]);
		}
		PRINT_INFO("write 0x%x=0x%x, 0x%x, 0x%x",
			sys_param_addr[0], Swipe_sense_reset_data2[0],
			Swipe_sense_reset_data2[1], Swipe_sense_reset_data2[2]);
	}
}
/* ASUS BSP Clay: ---*/

static void grip_set_game_gesture_sysp(bool flag){
	const char *buf_on = "110 5\n";
	const char *buf_off = "110 0\n";
	const char *buf_swipe_on = "110 4\n";
	static bool status = false;
	if(status == false || status != flag){
		if(flag == false){
			if(!grip_tap_gesture_status()){
				status = flag;
			}

			if(!grip_game_gesture_status()){ //all off
				msleep(50);
				grip_set_sys_param(buf_off);
				status = flag;
			}else if(grip_slide_gesture_status() == 0 &&
			grip_swipe_gesture_status()){ //swipe on, slide off
				msleep(50);
				grip_set_sys_param(buf_swipe_on);
				status = flag;
			}
		}else if(grip_game_gesture_status() == 1 && flag == true){
			msleep(50);
			grip_set_sys_param(buf_on);
			status = flag;
		}
	}
}

static void grip_slide_swipe_status_check(void){
	uint16_t enable_sensitive_boost = 0x84;
	uint16_t disable_sensitive_boost = 0x4;
	uint16_t boost_addr = 0x3d;
	static int swipe_status = 0, gesture_status = 0, slide_status = 0;
	const char *swipe_buf_on = "110 4\n";
	const char *slide_buf_on = "110 6\n";
	const char *buf_off = "110 0\n";

	if(grip_status_g->G_SLIDE_EN[0] == 1 || grip_status_g->G_SLIDE_EN[1] == 1
		|| grip_status_g->G_SWIPE_EN[0] == 1 || grip_status_g->G_SWIPE_EN[1] == 1){
		if(gesture_status != 1){
			gesture_status = 1;
		}
		grip_sense1_setting(true);
		if(grip_status_g->G_SLIDE_EN[0] == 1 || grip_status_g->G_SLIDE_EN[1] == 1){
			if(0 == slide_status){ //slide off => on
				slide_status = 1;
				/* set swipe buf off when slide has higher priority */
				write_register(snt8100fsr_g, boost_addr, &disable_sensitive_boost);
				msleep(50);
				grip_set_sys_param(slide_buf_on);
			}
			if(grip_status_g->G_SWIPE_EN[0] == 1 || grip_status_g->G_SWIPE_EN[1] == 1){
				if(0 == swipe_status){ //slide on, swipe off => on
					swipe_status = 1;
					grip_sense2_setting(true);
				}
			}else{
				if(1 == swipe_status){ //slide on, swipe on => off
					swipe_status = 0;
					grip_sense2_setting(false);
				}
			}
		}else{ //swipe enable, slide off
			if(1 == slide_status){ //swipe on, slide on => off
				slide_status = 0;
				write_register(snt8100fsr_g, boost_addr, &enable_sensitive_boost);
				msleep(50);
				if(!grip_tap_gesture_status()){
					grip_set_sys_param(swipe_buf_on);
				}
				grip_sense2_setting(true);
			}
			if(0 == swipe_status){ //slide off, swipe off=> on
				swipe_status = 1;
				write_register(snt8100fsr_g, boost_addr, &enable_sensitive_boost);
				msleep(50);
				if(!grip_tap_gesture_status()){
					grip_set_sys_param(swipe_buf_on);
				}
				grip_sense2_setting(true);
			}
		}
	}else{
		if(gesture_status != 0){
			gesture_status = 0;
			grip_sense1_setting(false);
			swipe_status = 0;
			slide_status = 0;
			write_register(snt8100fsr_g, boost_addr, &disable_sensitive_boost);
			msleep(50);
			if(grip_game_gesture_status() == 0){
				grip_set_sys_param(buf_off);
			}
			grip_sense2_setting(false);
		}
	}
}
/**************** ---wrtie DPC & Frame function --- **************/

/**************** +++ wrtie gesture function +++ **************/
void grip_enable_func_noLock(int val){

	if(g_snt_power_state == 0){
		grip_status_g->G_EN= 0;
		PRINT_INFO("Grip Sensor Power off, skip grip enable function");
		return;
	}
//	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	if(snt8100fsr_g->chip_reset_flag != GRIP_FW_DL_END){ //reseting
		//mutex_unlock(&snt8100fsr_g->ap_lock);
		grip_status_g->G_EN= val;
		PRINT_INFO("chip_reset_flag = %d", snt8100fsr_g->chip_reset_flag);
		return;
	}

	if(snt8100fsr_g->grip_fw_loading_status == false){
		grip_status_g->G_EN= 0;
		PRINT_INFO("Load fw fail, skip grip enable function");
		return;
	}
	if(val == 1){ //turn on
		if(grip_status_g->G_EN <= 0){ //Need turn on
			Power_Control(3);
			// We mutex lock here since we're calling sb_wake_device which never locks
			Grip_Driver_IRQ_EN(1);

			/* Check Time before wake up */
			ms_end = get_time_in_ms();
			if((ms_end-ms_start)< snt8100fsr_g->sleep_ms_time){
				PRINT_INFO("ms_start=%u, ms_end=%u", ms_start, ms_end);
				msleep(snt8100fsr_g->sleep_ms_time-(ms_end-ms_start));
			}


			Wait_Wake_For_RegW();
			//Grip_Chip_IRQ_EN(1);
			Grip_DPC_status_g->Low = 5;
			DPC_write_func(1);
			write_register(snt8100fsr_g, REGISTER_FRAME_RATE, &snt8100fsr_g->frame_rate);
			PRINT_INFO("Grip_EN = %d , Grip_Frame = %d", grip_status_g->G_EN, snt8100fsr_g->frame_rate);
		}
	}else{
		if(grip_status_g->G_EN == 1){
			grip_status_g->G_EN= val;
			Wait_Wake_For_RegW();
			DPC_write_func(0);
			Into_DeepSleep_fun();
		}
	}
	grip_status_g->G_EN= val;
	//mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_raw_enable_func(int val){
	int ret;
	uint16_t RegRead_t = 0;
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}

	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	if(grip_status_g->G_RAW_EN == val){
		mutex_unlock(&snt8100fsr_g->ap_lock);
		return;
	}
	grip_status_g->G_RAW_EN = val;
	if(Grip_Check_FW_Status()){ return; }

	Wait_Wake_For_RegW();
	if(val == 0){
		val = 1;
	}else{
		val = 0;
	}
	RegRead_t = val << 3;
	ret = write_register(snt8100fsr_g, REGISTR_RAW_DATA, &RegRead_t);
	if(ret < 0) {
		PRINT_ERR("Write reg 0x%X faill", REGISTR_RAW_DATA);
	}
	track_report_count = 0;
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

static int grip_all_gesture_status(){
	if(grip_status_g->G_SQUEEZE_EN[0] == 1 || grip_status_g->G_SQUEEZE_EN[1] == 1
		|| grip_status_g->G_TAP_EN[0] == 1 || grip_status_g->G_TAP_EN[1] == 1
		|| grip_status_g->G_TAP_EN[2] == 1 || grip_status_g->G_TAP_EN[3] == 1
		|| grip_status_g->G_SWIPE_EN[0] == 1 || grip_status_g->G_SWIPE_EN[1] == 1
		|| grip_status_g->G_SLIDE_EN[0] == 1 || grip_status_g->G_SLIDE_EN[1] == 1){
		return 1;
	}else{ /* No gesture or raw enable */
		return 0;
	}
}

int grip_game_gesture_status(){
	if(grip_tap_gesture_status() || grip_slide_gesture_status() ||
	grip_swipe_gesture_status()){
		return 1;
	}else{ /* No gesture or raw enable */
		return 0;
	}
}

static int grip_tap_gesture_status(){
	if(grip_status_g->G_TAP_EN[0] == 1 || grip_status_g->G_TAP_EN[1] == 1
		|| grip_status_g->G_TAP_EN[2] == 1 || grip_status_g->G_TAP_EN[3] == 1){
		return 1;
	}else{ /* No gesture or raw enable */
		return 0;
	}
}

static int grip_swipe_gesture_status(){
	if(grip_status_g->G_SWIPE_EN[0] == 1 || grip_status_g->G_SWIPE_EN[1] == 1){
		return 1;
	}else{ /* No gesture or raw enable */
		return 0;
	}
}

static int grip_slide_gesture_status(){
	if(grip_status_g->G_SLIDE_EN[0] == 1 || grip_status_g->G_SLIDE_EN[1] == 1){
		return 1;
	}else{ /* No gesture or raw enable */
		return 0;
	}
}

static void grip_checkToLowPower_noLock(){
	if(grip_all_gesture_status()){
		/* Do nothing */
	}else{ /* No gesture or raw enable */
		grip_enable_func_noLock(0);
	}
}

int write_registers_fifo(int reg, int num, void *value) {
	int ret=0;
	mutex_lock(&snt8100fsr_g->sb_lock);
	ret = sb_write_fifo(snt8100fsr_g, reg, num*2, value);
	if (ret) {
		PRINT_CRIT("write_registers_fifo() failed (%d)", ret);
	}
	mutex_unlock(&snt8100fsr_g->sb_lock);
	return ret;
}

int read_registers_fifo(int reg, int num, void *value) {
	int ret=0;
	mutex_lock(&snt8100fsr_g->sb_lock);
	ret = sb_read_fifo(snt8100fsr_g, reg, num*2, value);
	if (ret) {
		PRINT_CRIT("cust_read_registers() failed (%d)", ret);
	} else {
	}
	mutex_unlock(&snt8100fsr_g->sb_lock);
	return ret;
}

int tap_reg_num = 16;
int squeeze_reg_num = 26;
int slide_reg_num = 12;
int swipe_reg_num = 10;

//Tap part start===========================================
void get_tap_gesture(uint16_t tap_id, uint16_t reg_val, int index, int len){
	//int ret=0;
	//int bytes=0;
	int i=0;
	uint16_t cfg_bank_write[3] = { 0, 0, 0x0801};
	uint16_t buf[255];
	write_registers_fifo(0x2c, 3, cfg_bank_write);
	read_registers_fifo(0x200, len, buf);

	for(i = 0; i < len; i++){
		//if(i%8==1 || i%8==2){
		PRINT_INFO("reg_val=0x%x", buf[i]);
		//}
	}
}


void set_tap_gesture(uint16_t tap_id, uint16_t reg_val, int index){
	uint16_t cfg_bank_write[3] = { tap_id*tap_reg_num + index*2, 2, 0x0802};
	uint16_t cfg_bank_commit[3] = { 0, 0, 0x0803};
	write_registers_fifo(0x2c, 3, cfg_bank_write);
	write_registers_fifo(0x200, 1, &reg_val);
	write_registers_fifo(0x2c, 3, cfg_bank_commit);
}

void grip_tap_enable_func(int tap_id, int val, uint16_t* reg ) {
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);

	if( val==1 ) {
		grip_enable_func_noLock(1);
	}
	grip_status_g->G_TAP_EN[tap_id] = val;
	if(grip_status_g->G_EN == -1)
		grip_status_g->G_EN = 0;

	/* when G_EN=0 which means that all gestures close, don't wakeup chip and do setting */
	if(grip_status_g->G_EN == 1){
		if(Grip_Check_FW_Status()){ return; }

		Wait_Wake_For_RegW();
		*reg = (*reg & 0xFFFE) | ( val & 0x0001);
		set_tap_gesture(tap_id, *reg, 0);

		/* ASUS BSP Clay: setting vibrator trig1 & trig2 register for different tap modes +++ */
		gripVibratorSetting(val, tap_id);
		/* ASUS BSP Clay:--- */

		//get_tap_gesture(0, *reg, 0, tap_id*8 +9);
		grip_check_DPC_and_sensitivity_func();
		if(val == 1){
			grip_set_game_gesture_sysp(true);
		}else{
			grip_set_game_gesture_sysp(false);
		}

		if( val==0) { //turn off, check if (all off) goto low power
			grip_checkToLowPower_noLock();
		}
	}else{
		PRINT_INFO("No gesture enable, skip it");
	}
	mutex_unlock(&snt8100fsr_g->ap_lock);
}
void grip_tapX_enable_func(int val, int id, uint16_t *reg_val ){

	//int id = 0;
	grip_tap_enable_func(id, val, reg_val);
	PRINT_INFO("Write tap id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_tap_force_func(int tap_id, int val, uint16_t* reg ) {
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_TAP_FORCE[tap_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	val = val << 8;
	*reg = (val & 0xFF00) | ( *reg & 0x00FF);
	set_tap_gesture(tap_id, *reg, 0);
	//get_tap_gesture(0, *reg, 0, tap_id*8 +9);
	/* check bar scan behavior */
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_tapX_force_func(int val, int id, uint16_t *reg_val ){

	//int id = 0;
	grip_tap_force_func(id, val, reg_val);
	PRINT_INFO("Write tap id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_tap_min_position_func(int tap_id, int val, uint16_t* reg ) {
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_TAP_MIN_POS[tap_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = val;
	set_tap_gesture(tap_id, *reg, 2);
	//get_tap_gesture(0, *reg, 0, tap_id*8 +9);
	/* check bar scan behavior */
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_tapX_min_position_func(int val, int id, uint16_t *reg_val ){

	//int id = 0;
	grip_tap_min_position_func(id, val, reg_val);
	PRINT_INFO("Write tap id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_tap_max_position_func(int tap_id, int val, uint16_t* reg ) {
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_TAP_MAX_POS[tap_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = val;
	set_tap_gesture(tap_id, *reg, 3);
	//get_tap_gesture(0, *reg, 0, tap_id*8 +9);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_tapX_max_position_func(int val, int id, uint16_t *reg_val ){

	//int id = 0;
	grip_tap_max_position_func(id, val, reg_val);
	PRINT_INFO("Write tap id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_tap_sense_enable_func(int val){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_TAP_SENSE_SET= val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	grip_check_DPC_and_sensitivity_func();
	mutex_unlock(&snt8100fsr_g->ap_lock);
}
void grip_tap_slope_window_func(int tap_id, int val, uint16_t* reg ) {
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_TAP_SLOPE_WINDOW[tap_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = val | (*reg & 0xFF00);
	set_tap_gesture(tap_id, *reg, 1);
	//get_tap_gesture(0, *reg, 0, tap_id*8 +9);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_tapX_slope_window_func(int val, int id, uint16_t *reg_val ){

	//int id = 0;
	grip_tap_slope_window_func(id, val, reg_val);
	PRINT_INFO("Write tap id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_tap_slope_release_force_func(int tap_id, int val, uint16_t* reg ) {
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_TAP_SLOPE_RELEASE_FORCE[tap_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = val<<8 | (*reg & 0x00FF);
	set_tap_gesture(tap_id, *reg, 5);
	//get_tap_gesture(0, *reg, 0, tap_id*8 +9);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_tapX_slope_release_force_func(int val, int id, uint16_t *reg_val ){

	//int id = 0;
	grip_tap_slope_release_force_func(id, val, reg_val);
	PRINT_INFO("Write tap id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_tap_slope_tap_force_func(int tap_id, int val, uint16_t* reg ) {
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_TAP_SLOPE_TAP_FORCE[tap_id] = val;


	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = val | (*reg & 0xFF00);
	set_tap_gesture(tap_id, *reg, 5);
	//get_tap_gesture(0, *reg, 0, tap_id*8 +9);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_tapX_slope_tap_force_func(int val, int id, uint16_t *reg_val ){

	//int id = 0;
	grip_tap_slope_tap_force_func(id, val, reg_val);
	PRINT_INFO("Write tap id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_tap_delta_tap_force_func(int tap_id, int val, uint16_t* reg ) {
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_TAP_DELTA_TAP_FORCE[tap_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = val | (*reg & 0xFF00);
	set_tap_gesture(tap_id, *reg, 4);
	//get_tap_gesture(0, *reg, 0, tap_id*8 +9);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_tapX_delta_tap_force_func(int val, int id, uint16_t *reg_val ){

	//int id = 0;
	grip_tap_delta_tap_force_func(id, val, reg_val);
	PRINT_INFO("Write tap id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_tap_delta_release_force_func(int tap_id, int val, uint16_t* reg ) {
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_TAP_DELTA_RELEASE_FORCE[tap_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = val<<8 | (*reg & 0x00FF);
	set_tap_gesture(tap_id, *reg, 4);
	//get_tap_gesture(0, *reg, 0, tap_id*8 +9);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_tapX_delta_release_force_func(int val, int id, uint16_t *reg_val ){

	//int id = 0;
	grip_tap_delta_release_force_func(id, val, reg_val);
	 PRINT_INFO("Write tap id=%d, val=%d, reg: %x", id, val, *reg_val);
}

/* ASUS BSP Clay: setting vibrator trig1 & trig2 register for different tap modes +++ */
static void gripVibratorSetting(int val, int tap_id){
	static uint16_t reg_val[4] = { 0x1, 0x2, 0x8, 0x20 };
	PRINT_INFO("Tap_id=%d, EN=%d, VIB_EN=%d, 0x%x, 0x%x",
		tap_id,	grip_status_g->G_TAP_EN[tap_id],
		grip_status_g->G_TAP_VIB_EN[tap_id],
		G_grip_tap_vib1_reg, G_grip_tap_vib2_reg);

//	if(g_ASUS_hwID >= HW_REV_PR){
		if(val && grip_status_g->G_TAP_EN[tap_id]==1 && grip_status_g->G_TAP_VIB_EN[tap_id]==1){
			if(tap_id == 1){
				if(grip_status_g->G_TAP_EN[tap_id]==1 && grip_status_g->G_TAP_EN[tap_id+2]==1){
					PRINT_INFO("Double tap case");
					/* tap2 */
					G_grip_tap_vib1_reg = G_grip_tap_vib1_reg & (0xffff - reg_val[tap_id]);
					G_grip_tap_vib2_reg = G_grip_tap_vib2_reg | reg_val[tap_id];
				}else{
					G_grip_tap_vib1_reg = G_grip_tap_vib1_reg | reg_val[tap_id];
					G_grip_tap_vib2_reg = G_grip_tap_vib2_reg & (0xffff - reg_val[tap_id]);
				}
			}else if(tap_id == 3){
				if(grip_status_g->G_TAP_EN[tap_id-2]==1	&& grip_status_g->G_TAP_VIB_EN[tap_id-2]==1){
					PRINT_INFO("Double tap case");
					/* tap2 */
					G_grip_tap_vib1_reg = G_grip_tap_vib1_reg & (0xffff - reg_val[tap_id-2]);
					G_grip_tap_vib2_reg = G_grip_tap_vib2_reg | reg_val[tap_id-2];
				}
				G_grip_tap_vib1_reg = G_grip_tap_vib1_reg | reg_val[tap_id];
			}else if(tap_id == 2){
				G_grip_tap_vib2_reg = G_grip_tap_vib2_reg | reg_val[tap_id];
			}else{
				G_grip_tap_vib1_reg = G_grip_tap_vib1_reg | reg_val[tap_id];
			}
		}else{
			G_grip_tap_vib1_reg = G_grip_tap_vib1_reg & (0xffff - reg_val[tap_id]);
			G_grip_tap_vib2_reg = G_grip_tap_vib2_reg & (0xffff - reg_val[tap_id]);
			//tap3 off or tap3_vib off
			//recovery case: tap2 on and tap2_vib on
			if(tap_id==3 && grip_status_g->G_TAP_EN[tap_id-2]==1
				&& grip_status_g->G_TAP_VIB_EN[tap_id-2]==1){
				PRINT_INFO("Double tap to One tap");
				G_grip_tap_vib1_reg = G_grip_tap_vib1_reg | reg_val[tap_id-2];
				G_grip_tap_vib2_reg = G_grip_tap_vib2_reg & (0xffff - reg_val[tap_id-2]);
			}
		}
		/*
	}else{
		if(val && grip_status_g->G_TAP_EN[tap_id]==1 && grip_status_g->G_TAP_VIB_EN[tap_id]==1){
			G_grip_tap_vib1_reg = G_grip_tap_vib1_reg | reg_val[tap_id];
		}else{
			G_grip_tap_vib1_reg = G_grip_tap_vib1_reg & (0xffff - reg_val[tap_id]);
		}
	}*/
	Wait_Wake_For_RegW();
	write_register(snt8100fsr_g, REGISTER_TIRGGER_LINK1, &G_grip_tap_vib1_reg);
	write_register(snt8100fsr_g, REGISTER_TIRGGER_LINK2, &G_grip_tap_vib2_reg);
	PRINT_INFO("[0x%x] = 0x%x, [0x%x] = 0x%x",
		REGISTER_TIRGGER_LINK1, G_grip_tap_vib1_reg,
		REGISTER_TIRGGER_LINK2, G_grip_tap_vib2_reg);
}
/* ASUS BSP--- */

void grip_tapX_vibrator_enable_func(int val, int tap_id){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_INFO("val = %d, tap_id=%d", val, tap_id);
	grip_status_g->G_TAP_VIB_EN[tap_id] = val;

	if(Grip_Check_FW_Status()){return;}

	/* ASUS BSP Clay: setting vibrator trig1 & trig2 register for different tap modes +++ */
	gripVibratorSetting(val, tap_id);
	/* ASUS BSP--- */
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_tap_vibrator_repeat_enable_func(int tap_id, int val){
	static uint16_t trig_ctrl1 = 0, trig_ctrl2 = 0;
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_TAP_VIB_REPEAT_EN[tap_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	//if tap1 or tap4 vib repeat flag enable, enable trig3 vib repeat repeat
	read_register(snt8100fsr_g, REGISTER_TIRG_CTRL1, &trig_ctrl1);
	if(grip_status_g->G_TAP_VIB_REPEAT_EN[0] || grip_status_g->G_TAP_VIB_REPEAT_EN[3]){
		trig_ctrl1 |= TRIG3_VIB_REPEAT_FLAG;
	}else{
		trig_ctrl1 &= (0xFFFF - TRIG3_VIB_REPEAT_FLAG);
	}

	//if tap2 or tap3 vib repeat flag enable, enable trig3 vib repeat repeat
	read_register(snt8100fsr_g, REGISTER_TIRG_CTRL2, &trig_ctrl2);
	if(grip_status_g->G_TAP_VIB_REPEAT_EN[1] || grip_status_g->G_TAP_VIB_REPEAT_EN[2]){
		trig_ctrl2 |= TRIG5_VIB_REPEAT_FLAG;
	}else{
		trig_ctrl2 &= (0xFFFF - TRIG5_VIB_REPEAT_FLAG);
	}
	if(val == 1){
		switch(tap_id){
			case 0:
				trig_ctrl1 |= (TAP1_VIB_REPEAT_FLAG | TAP1_VIB_PULSE_FLAG);
				break;
			case 1:
				trig_ctrl1 |= (TAP2_VIB_REPEAT_FLAG | TAP2_VIB_PULSE_FLAG);
				break;
			case 2:
				trig_ctrl1 |= (TAP3_VIB_REPEAT_FLAG | TAP3_VIB_PULSE_FLAG);
				break;
			case 3:
				trig_ctrl2 |= (TAP4_VIB_REPEAT_FLAG | TAP4_VIB_PULSE_FLAG);
				break;
			default:
				break;
		}
	}else{
		switch(tap_id){
			case 0:
				trig_ctrl1 &= (0xFFFF - (TAP1_VIB_REPEAT_FLAG | TAP1_VIB_PULSE_FLAG));
				break;
			case 1:
				trig_ctrl1 &= (0xFFFF - (TAP2_VIB_REPEAT_FLAG | TAP2_VIB_PULSE_FLAG));
				break;
			case 2:
				trig_ctrl1 &= (0xFFFF - (TAP3_VIB_REPEAT_FLAG | TAP3_VIB_PULSE_FLAG));
				break;
			case 3:
				trig_ctrl2 &= (0xFFFF - (TAP4_VIB_REPEAT_FLAG | TAP4_VIB_PULSE_FLAG));
				break;
			default:
				break;
		}
	}
	PRINT_INFO("trig_ctrl1 = 0x%02x, trig_ctrl2 = 0x%02x", trig_ctrl1, trig_ctrl2);
	write_register(snt8100fsr_g, REGISTER_TIRG_CTRL1, &trig_ctrl1);
	write_register(snt8100fsr_g, REGISTER_TIRG_CTRL2, &trig_ctrl2);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}
void grip_tapX_vibrator_repeat_enable_func(int val, int tap_id){
	grip_tap_vibrator_repeat_enable_func(tap_id, val);
	 PRINT_INFO("Write tap id=%d, val=%d", tap_id, val);
}

void grip_tap_vibrator_repeat_dur_func(int tap_id, int val){
	uint16_t reg_addr = 0;
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_TAP_VIB_REPEAT_DUR[tap_id] = val;
	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	switch(tap_id){
		case 0:
			reg_addr = REGISTER_TIRG1_REPEAT_DUR;
			break;
		case 1:
			reg_addr = REGISTER_TIRG2_REPEAT_DUR;
			break;
		case 2:
			reg_addr = REGISTER_TIRG4_REPEAT_DUR;
			break;
		case 3:
			reg_addr = REGISTER_TIRG6_REPEAT_DUR;
			break;
		default:
			break;
	}
	write_register(snt8100fsr_g, reg_addr, &val);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}
void grip_tapX_vibrator_repeat_dur_func(int val, int tap_id){
	grip_tap_vibrator_repeat_dur_func(tap_id, val);
	 PRINT_INFO("Write tap id=%d, val=%d", tap_id, val);
}

//Tap part down===========================================

//Squeeze part start===========================================
void get_sq_gesture(uint16_t tap_id, uint16_t reg_val, int index, int len){
	//int ret=0;
	//int bytes=0;
	int i=0;
	uint16_t cfg_bank_read[3] = { 0, 0, 0x0a01};
	uint16_t buf[255];
	write_registers_fifo(0x2c, 3, cfg_bank_read);
	read_registers_fifo(0x200, len, buf);

	for(i = 0; i < len; i++){
		PRINT_INFO("reg_val=0x%x", buf[i]);
	}
}

void set_sq_gesture(uint16_t sq_id, uint16_t reg_val, int index){
	uint16_t cfg_bank_write[3] = { sq_id*squeeze_reg_num + index * 2, 2, 0x0a02};
	uint16_t cfg_bank_commit[3] = { 0, 0, 0x0a03};
	write_registers_fifo(0x2c, 3, cfg_bank_write);
	write_registers_fifo(0x200, 1, &reg_val);
	write_registers_fifo(0x2c, 3, cfg_bank_commit);
}
void grip_squeeze_enable_func(int sq_id, int val, uint16_t* reg){
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_INFO("id=%d, val = %d", sq_id, val);

	if( val==1) {
		grip_enable_func_noLock(1);
	}

	grip_status_g->G_SQUEEZE_EN[sq_id] = val;
	if(grip_status_g->G_EN == -1)
		grip_status_g->G_EN = 0;

	/* when G_EN=0 which means that all gestures close, don't wakeup chip and do setting */
	if(grip_status_g->G_EN == 1){

		if(Grip_Check_FW_Status()){return;}

		Wait_Wake_For_RegW();
		val = val << 15;
		* reg = (val & 0x8000) | (* reg & 0x7FFF);
		set_sq_gesture(sq_id, * reg, 0);
		//get_sq_gesture(0, * reg, 0, 2);

		if(val == 1){
			grip_set_game_gesture_sysp(false);
		}

		if( val==0) { //turn off, check if (all off) goto low power
			grip_checkToLowPower_noLock();
		}
	}else{
		PRINT_INFO("No gesture enable, skip it");
	}
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_squeezeX_enable_func(int val, int id, uint16_t *reg_val ){
	grip_squeeze_enable_func(id, val, reg_val);
	PRINT_INFO("Write sq id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_squeeze1_enable_func(int val){
	int id=0;
	grip_squeeze_enable_func(id, val, &SQ1_BIT0);
	PRINT_INFO("Write sq1 reg: %x", SQ1_BIT0);
}

void grip_squeeze2_enable_func(int val){
	int id=1;
	grip_squeeze_enable_func(id, val, &SQ2_BIT0);
	PRINT_INFO("Write sq2 reg: %x", SQ2_BIT0);
}

void grip_squeeze_force_func(int sq_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);

	grip_status_g->G_SQUEEZE_FORCE[sq_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = (val & 0x00FF) | (*reg & 0xFF00);
	set_sq_gesture(sq_id, * reg, 0);
	//get_sq_gesture(0, * reg, 0, 2);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_squeezeX_force_func(int val, int id, uint16_t *reg_val ){
	grip_squeeze_force_func(id, val, reg_val);
	PRINT_INFO("Write sq id=%d, val=%d, reg: %x", id, val, *reg_val);
}

bool G_Skip_Sq1_Long = 0;
bool G_Skip_Sq2_Long = 0;

void grip_squeeze_short_dur_func(int sq_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);

	grip_status_g->G_SQUEEZE_SHORT[sq_id]= val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = ((val/20) << 8) | (*reg & 0x00FF);
	set_sq_gesture(sq_id, *reg, 5);
	//get_sq_gesture(0, * reg, 0, 2);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_squeezeX_short_dur_func(int val, int id, uint16_t *reg_val ){
	grip_squeeze_short_dur_func(id, val, reg_val);
	PRINT_INFO("Write sq id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_squeeze_long_dur_func(int sq_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);

	if(sq_id == 0){
		grip_status_g->G_SQUEEZE_LONG[sq_id]= val;
		if(val == 0){
			G_Skip_Sq1_Long = 1;
			mutex_unlock(&snt8100fsr_g->ap_lock);
			return;
		}else
			G_Skip_Sq1_Long = 0;
	}else if(sq_id ==1){
		grip_status_g->G_SQUEEZE_LONG[sq_id] = val;
		if(val == 0){
			G_Skip_Sq2_Long = 1;
			mutex_unlock(&snt8100fsr_g->ap_lock);
			return;
		}else
			G_Skip_Sq2_Long = 0;
	}

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = (val/20) | (*reg & 0xFF00);
	set_sq_gesture(sq_id, *reg, 5);
	//get_sq_gesture(0, *reg, 0, 6);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_squeezeX_long_dur_func(int val, int id, uint16_t *reg_val ){
	grip_squeeze_long_dur_func(id, val, reg_val);
	PRINT_INFO("Write sq id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_squeeze_BarA_up_rate_func(int sq_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SQUEEZE_BARA_UP_RATE[sq_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = val<<8 | (*reg & 0x00FF);
	set_sq_gesture(sq_id, *reg, 6);
	PRINT_INFO("Write Squeeze_BarA_up_rate: 0x%x", * reg);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_squeezeX_BarA_up_rate_func(int val, int id, uint16_t *reg_val ){
	grip_squeeze_BarA_up_rate_func(id, val, reg_val);
	PRINT_INFO("Write sq id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_squeeze_BarA_up_rate_acc_func(int sq_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SQUEEZE_BARA_UP_RATE_ACC[sq_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = val | (*reg & 0xFF00);
	set_sq_gesture(sq_id, *reg, 6);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_squeezeX_BarA_up_rate_acc_func(int val, int id, uint16_t *reg_val ){
	grip_squeeze_BarA_up_rate_acc_func(id, val, reg_val);
	PRINT_INFO("Write sq id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_squeeze_BarA_drop_rate_func(int sq_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SQUEEZE_BARA_DROP_RATE[sq_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = val<<8 | (*reg & 0x00FF);
	set_sq_gesture(sq_id, *reg, 7);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_squeezeX_BarA_drop_rate_func(int val, int id, uint16_t *reg_val ){
	grip_squeeze_BarA_drop_rate_func(id, val, reg_val);
	PRINT_INFO("Write sq id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_squeeze_BarA_drop_rate_acc_func(int sq_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SQUEEZE_BARA_DROP_RATE_ACC[sq_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = val | (*reg & 0xFF00);
	set_sq_gesture(sq_id, *reg, 7);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_squeezeX_BarA_drop_rate_acc_func(int val, int id, uint16_t *reg_val ){
	grip_squeeze_BarA_drop_rate_acc_func(id, val, reg_val);
	PRINT_INFO("Write sq id=%d, val=%d, reg: %x", id, val, *reg_val);
}


void grip_squeeze_BarB_up_rate_func(int sq_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SQUEEZE_BARB_UP_RATE[sq_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = val<<8 | (*reg & 0x00FF);
	set_sq_gesture(sq_id, *reg, 9);
	PRINT_INFO("Write Squeeze_BarB_up_rate: 0x%x", * reg);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_squeezeX_BarB_up_rate_func(int val, int id, uint16_t *reg_val ){
	grip_squeeze_BarB_up_rate_func(id, val, reg_val);
	PRINT_INFO("Write sq id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_squeeze_BarB_up_rate_acc_func(int sq_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SQUEEZE_BARB_UP_RATE_ACC[sq_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = val | (*reg & 0xFF00);
	set_sq_gesture(sq_id, *reg, 9);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_squeezeX_BarB_up_rate_acc_func(int val, int id, uint16_t *reg_val ){
	grip_squeeze_BarB_up_rate_acc_func(id, val, reg_val);
	PRINT_INFO("Write sq id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_squeeze_BarB_drop_rate_func(int sq_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SQUEEZE_BARB_DROP_RATE[sq_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = val<<8 | (*reg & 0x00FF);
	set_sq_gesture(sq_id, *reg, 10);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_squeezeX_BarB_drop_rate_func(int val, int id, uint16_t *reg_val ){
	grip_squeeze_BarB_drop_rate_func(id, val, reg_val);
	PRINT_INFO("Write sq id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_squeeze_BarB_drop_rate_acc_func(int sq_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SQUEEZE_BARB_DROP_RATE_ACC[sq_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = val | (*reg & 0xFF00);
	set_sq_gesture(sq_id, *reg, 10);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_squeezeX_BarB_drop_rate_acc_func(int val, int id, uint16_t *reg_val ){
	grip_squeeze_BarB_drop_rate_acc_func(id, val, reg_val);
	PRINT_INFO("Write sq id=%d, val=%d, reg: %x", id, val, *reg_val);
}

//Squeeze part down===========================================

/****************** Slide ********************************/
void get_slide_gesture(uint16_t tap_id, uint16_t reg_val, int index, int len){
	//int ret=0;
	//int bytes=0;
	int i=0;
	uint16_t cfg_bank_read[3] = { 0, 0, 0x0b01};
	uint16_t buf[255];
	write_registers_fifo(0x2c, 3, cfg_bank_read);
	read_registers_fifo(0x200, len, buf);

	for(i = 0; i < len; i++){
		PRINT_INFO("reg_val=0x%x", buf[i]);
	}
}
void set_slide_gesture(uint16_t slide_id, uint16_t reg_val, int index){
	uint16_t cfg_bank_write[3] = { slide_id*slide_reg_num + index * 2, 2, 0x0b02};
	uint16_t cfg_bank_commit[3] = { 0, 0, 0x0b03};
	write_registers_fifo(0x2c, 3, cfg_bank_write);
	write_registers_fifo(0x200, 1, &reg_val);
	write_registers_fifo(0x2c, 3, cfg_bank_commit);
}

void grip_slide_enable_func(int slide_id, int val, uint16_t* reg){
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);

	if( val==1) {
		grip_enable_func_noLock(1);
	}

	grip_status_g->G_SLIDE_EN[slide_id] = val;
	if(grip_status_g->G_EN == -1)
		grip_status_g->G_EN = 0;

	/* when G_EN=0 which means that all gestures close, don't wakeup chip and do setting */
	if(grip_status_g->G_EN == 1){

		if(Grip_Check_FW_Status()){return;}

		Wait_Wake_For_RegW();
		val = val << 15;
		* reg = (val & 0x8000) | (* reg & 0x7FFF);
		set_slide_gesture(slide_id, *reg, 0);
		grip_check_DPC_and_sensitivity_func();
		grip_slide_swipe_status_check();

		if( val==0) { //turn off, check if (all off) goto low power
			grip_checkToLowPower_noLock();
		}
	}else{
		PRINT_INFO("No gesture enable, skip it");
	}
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_slideX_enable_func(int val, int id, uint16_t *reg_val){
	grip_slide_enable_func(id, val, reg_val);
	PRINT_INFO("Write slide id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_slide_dist_func(int slide_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SLIDE_DIST[slide_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	val = val << 8;
	*reg = (val & 0xFF00) | (*reg & 0x00FF);
	set_slide_gesture(slide_id, *reg, 3);
	PRINT_INFO("Write Slide1_Dist: 0x%x", *reg);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_slideX_dist_func(int val, int id, uint16_t *reg_val){
	grip_slide_dist_func(id, val, reg_val);
	PRINT_INFO("Write slide id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_slide_force_func(int slide_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SLIDE_FORCE[slide_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = (val & 0x00FF) | (*reg & 0xFF00);
	set_slide_gesture(slide_id, *reg, 3);
	PRINT_INFO("Write Slide1_Force: 0x%x", *reg);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_slideX_force_func(int val, int id, uint16_t *reg_val){
	grip_slide_force_func(id, val, reg_val);
	PRINT_INFO("Write slide id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_slide_2nd_dist_func(int slide_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SLIDE_2ND_DIST[slide_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = (val & 0x00FF) | (*reg & 0xFF00);
	set_slide_gesture(slide_id, *reg, 4);
	PRINT_INFO("Write Slide1_Force: 0x%x", *reg);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_slideX_2nd_dist_func(int val, int id, uint16_t *reg_val){
	grip_slide_2nd_dist_func(id, val, reg_val);
	PRINT_INFO("Write slide id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_slide_vibrator_enable_func(int slide_id, int val){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SLIDE_VIB_EN[slide_id] = val;

	if(Grip_Check_FW_Status()){return;}

	PRINT_INFO("SLIDE1 Do nothing");
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_slideX_vibrator_enable_func(int val, int id){
	grip_slide_vibrator_enable_func(id, val);
	PRINT_INFO("Write slide id=%d, val=%d", id, val);
}

void grip_slide_tap_priority_func(int slide_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SLIDE_TAP_PRIORITY[slide_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	val = val << 14;
	*reg = (val & 0x4000) | (*reg & 0xBFFF);
	set_slide_gesture(slide_id, *reg, 0);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_slideX_tap_priority_func(int val, int id, uint16_t* reg_val){
	grip_slide_tap_priority_func(id, val, reg_val);
	PRINT_INFO("Write slide id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_slide_min_position_func(int slide_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SLIDE_MIN_POS[slide_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = val;
	set_slide_gesture(slide_id, *reg, 1);
	PRINT_INFO("Write Slide1_Force: 0x%x", *reg);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_slideX_min_position_func(int val, int id, uint16_t *reg_val){
	grip_slide_min_position_func(id, val, reg_val);
	PRINT_INFO("Write slide id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_slide_max_position_func(int slide_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SLIDE_MAX_POS[slide_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = val;
	set_slide_gesture(slide_id, *reg, 2);
	PRINT_INFO("Write Slide1_Force: 0x%x", *reg);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_slideX_max_position_func(int val, int id, uint16_t *reg_val){
	grip_slide_max_position_func(id, val, reg_val);
	PRINT_INFO("Write slide id=%d, val=%d, reg: %x", id, val, *reg_val);
}

/****************** SWIPE ********************************/
void get_swipe_gesture(uint16_t swipe_id, uint16_t reg_val, int index, int len){
	//int ret=0;
	//int bytes=0;
	int i=0;
	uint16_t cfg_bank_read[3] = { 0, 0, 0x0901};
	uint16_t buf[255];
	write_registers_fifo(0x2c, 3, cfg_bank_read);
	read_registers_fifo(0x200, len, buf);

	for(i = 0; i < len; i++){
		PRINT_INFO("reg_val=0x%x", buf[i]);
	}
}

void set_swipe_gesture(uint16_t swipe_id, uint16_t reg_val, int index){
	uint16_t cfg_bank_write[3] = { swipe_id*swipe_reg_num + index * 2, 2, 0x0902};
	uint16_t cfg_bank_commit[3] = { 0, 0, 0x0903};
	write_registers_fifo(0x2c, 3, cfg_bank_write);
	write_registers_fifo(0x200, 1, &reg_val);
	write_registers_fifo(0x2c, 3, cfg_bank_commit);
}

void grip_swipe_enable_func(int swipe_id, int val, uint16_t* reg){
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);

	if( val==1) {
		grip_enable_func_noLock(1);
	}

	grip_status_g->G_SWIPE_EN[swipe_id] = val;
	if(grip_status_g->G_EN == -1)
		grip_status_g->G_EN = 0;

	/* when G_EN=0 which means that all gestures close, don't wakeup chip and do setting */
	if(grip_status_g->G_EN == 1){

		if(Grip_Check_FW_Status()){return;}

		Wait_Wake_For_RegW();
		val = val << 15;
		* reg = (val & 0x8000) | (* reg & 0x7FFF);
		set_swipe_gesture(swipe_id, *reg, 0);
		grip_check_DPC_and_sensitivity_func();
		grip_slide_swipe_status_check();

		if( val==0) { //turn off, check if (all off) goto low power
			grip_checkToLowPower_noLock();
		}
	}else{
		PRINT_INFO("No gesture enable, skip it");
	}
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_swipeX_enable_func(int val, int id, uint16_t *reg_val){
	grip_swipe_enable_func(id, val, reg_val);
	PRINT_INFO("Write slide id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_swipe_velocity_func(int swipe_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SWIPE_VELOCITY[swipe_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	* reg = (val & 0x00FF) | (* reg & 0xFF00);
	set_swipe_gesture(swipe_id, * reg, 0);
	 PRINT_INFO("Write Swipe_Velocity[%d]: 0x%x", swipe_id, *reg);
	 mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_swipeX_velocity_func(int val, int id, uint16_t *reg_val){
	grip_swipe_velocity_func(id, val, reg_val);
	PRINT_INFO("Write slide id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_swipe_len_func(int swipe_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SWIPE_LEN[swipe_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = val;
	set_swipe_gesture(swipe_id, *reg, 3);
	PRINT_INFO("Write Swipe_Len[%d]: 0x%x", swipe_id, *reg);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_swipeX_len_func(int val, int id, uint16_t *reg_val){
	grip_swipe_len_func(id, val, reg_val);
	PRINT_INFO("Write slide id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_swipe_min_position_func(int swipe_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SWIPE_MIN_POS[swipe_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = val;
	set_swipe_gesture(swipe_id, *reg, 1);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_swipeX_min_position_func(int val, int id, uint16_t *reg_val){
	grip_swipe_min_position_func(id, val, reg_val);
	PRINT_INFO("Write slide id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_swipe_max_position_func(int swipe_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SWIPE_MAX_POS[swipe_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = val;
	set_swipe_gesture(swipe_id, *reg, 2);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_swipeX_max_position_func(int val, int id, uint16_t *reg_val){
	grip_swipe_max_position_func(id, val, reg_val);
	PRINT_INFO("Write slide id=%d, val=%d, reg: %x", id, val, *reg_val);
}
/****************** SWIPE ********************************/
/* When K data is applied, squeeze should re-enable to make K data work */
void Grip_ReEnable_Squeeze_Check(void){
	int count = 0;
	int sq_num = 2;
	/* apply K data to squeeze algorithm */
	for(count=0;count < sq_num;count++){
		if(grip_status_g->G_SQUEEZE_EN[count] > 0){
			grip_squeezeX_enable_func(grip_status_g->G_SQUEEZE_EN[count], count, &SQ_BIT0[count]);
		}
	}
}

static void Grip_K_data_recovery(void){
	/* do nothing */
}

/* Recovery status after reset */
void grip_dump_status_func(struct work_struct *work){
	int count=0;
	int sq_num=2, tap_num=4, swipe_num=2, slide_num=2;
	int need_en=0;
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_INFO("framework setting recovery");
	PRINT_INFO("EN:%d, RAW_EN:%d, DPC_EN:%d",
		grip_status_g->G_EN, grip_status_g->G_RAW_EN,
		grip_status_g->G_DPC_STATUS);
	for(count=0;count < sq_num;count++){
		PRINT_INFO("SQ[%d], EN:%d, Force:%d, Short:%d, Long:%d",
			count, grip_status_g->G_SQUEEZE_EN[count], grip_status_g->G_SQUEEZE_FORCE[count],
			grip_status_g->G_SQUEEZE_SHORT[count], grip_status_g->G_SQUEEZE_LONG[count]);
	}
	for(count=0;count < tap_num;count++){
		PRINT_INFO("TAP[%d], EN:%d, Force:%d, min_pos:%d, max_pos:%d",
			count, grip_status_g->G_TAP_EN[count], grip_status_g->G_TAP_FORCE[count],
			grip_status_g->G_TAP_MIN_POS[count], grip_status_g->G_TAP_MAX_POS[count]);

		PRINT_INFO("slope_window:%d, slope_tap:%d, slope_release:%d, delta_tap:%d, delta_release:%d",
			grip_status_g->G_TAP_SLOPE_WINDOW[count],
			grip_status_g->G_TAP_SLOPE_TAP_FORCE[count], grip_status_g->G_TAP_SLOPE_RELEASE_FORCE[count],
			grip_status_g->G_TAP_DELTA_TAP_FORCE[count], grip_status_g->G_TAP_DELTA_RELEASE_FORCE[count]);
	}
	for(count=0;count < slide_num;count++){
		PRINT_INFO("SLIDE[%d], EN:%d, DIST:%d, Force:%d",
			count, grip_status_g->G_SLIDE_EN[count], grip_status_g->G_SLIDE_DIST[count],
			grip_status_g->G_SLIDE_FORCE[count]);
	}
	for(count=0;count < swipe_num;count++){
		PRINT_INFO("SWIPE[%d], EN:%d, Velocity:%d, LEN:%d",
			count, grip_status_g->G_SWIPE_EN[count], grip_status_g->G_SWIPE_VELOCITY[count],
			grip_status_g->G_SWIPE_LEN[count]);
	}
	mutex_unlock(&snt8100fsr_g->ap_lock);

	Wait_Wake_For_RegW();

	if(grip_status_g->G_EN > 0){
		grip_enable_func_noLock(grip_status_g->G_EN);
	}else{ //when en = 0, enable grip to prevent register recovery fail, and disable grip at final
		need_en = 1;
	}
	if(need_en==1){
		PRINT_INFO("Enable grip at first due to G_EN != 1");
		grip_enable_func_noLock(1);
	}
	if(grip_status_g->G_RAW_EN > 0){
		grip_raw_enable_func(grip_status_g->G_RAW_EN);
	}

	/* Squeeze Part */
	for(count=0;count < sq_num;count++){
		if(grip_status_g->G_SQUEEZE_EN[count] > 0){
			grip_squeezeX_enable_func(grip_status_g->G_SQUEEZE_EN[count], count, &SQ_BIT0[count]);
		}
		if(grip_status_g->G_SQUEEZE_SHORT[count] > 0){
			grip_squeezeX_short_dur_func(grip_status_g->G_SQUEEZE_SHORT[count], count, &SQ_BIT5[count]);
		}
		if(grip_status_g->G_SQUEEZE_LONG[count] > 0){
			grip_squeezeX_long_dur_func(grip_status_g->G_SQUEEZE_LONG[count], count, &SQ_BIT5[count]);
		}
		if(grip_status_g->G_SQUEEZE_FORCE[count] > 0){
			grip_squeezeX_force_func(grip_status_g->G_SQUEEZE_FORCE[count], count, &SQ_BIT0[count]);
		}

		if(grip_status_g->G_SQUEEZE_BARA_UP_RATE[count] > 0){
			grip_squeezeX_BarA_up_rate_func(grip_status_g->G_SQUEEZE_BARA_UP_RATE[count], count, &SQ_BIT6[count]);
		}
		if(grip_status_g->G_SQUEEZE_BARA_UP_RATE_ACC[count] > 0){
			grip_squeezeX_BarA_up_rate_acc_func(grip_status_g->G_SQUEEZE_BARA_UP_RATE_ACC[count], count, &SQ_BIT6[count]);
		}
		if(grip_status_g->G_SQUEEZE_BARA_DROP_RATE[count] > 0){
			grip_squeezeX_BarA_drop_rate_func(grip_status_g->G_SQUEEZE_BARA_DROP_RATE[count], count, &SQ_BIT7[count]);
		}
		if(grip_status_g->G_SQUEEZE_BARA_DROP_RATE_ACC[count] > 0){
			grip_squeezeX_BarA_drop_rate_acc_func(grip_status_g->G_SQUEEZE_BARA_DROP_RATE_ACC[count], count, &SQ_BIT7[count]);
		}

		if(grip_status_g->G_SQUEEZE_BARB_UP_RATE[count] > 0){
			grip_squeezeX_BarB_up_rate_func(grip_status_g->G_SQUEEZE_BARB_UP_RATE[count], count, &SQ_BIT6[count]);
		}
		if(grip_status_g->G_SQUEEZE_BARB_UP_RATE_ACC[count] > 0){
			grip_squeezeX_BarB_up_rate_acc_func(grip_status_g->G_SQUEEZE_BARB_UP_RATE_ACC[count], count, &SQ_BIT6[count]);
		}
		if(grip_status_g->G_SQUEEZE_BARB_DROP_RATE[count] > 0){
			grip_squeezeX_BarB_drop_rate_func(grip_status_g->G_SQUEEZE_BARB_DROP_RATE[count], count, &SQ_BIT7[count]);
		}
		if(grip_status_g->G_SQUEEZE_BARB_DROP_RATE_ACC[count] > 0){
			grip_squeezeX_BarB_drop_rate_acc_func(grip_status_g->G_SQUEEZE_BARB_DROP_RATE_ACC[count], count, &SQ_BIT7[count]);
		}
	}


	/* Tap Part */
	for(count=0;count < tap_num;count++){
		if(grip_status_g->G_TAP_EN[count] > 0){
			grip_tapX_enable_func(grip_status_g->G_TAP_EN[count], count, &TAP_BIT0[count]);
		}
		if(grip_status_g->G_TAP_FORCE[count] > 0){
			grip_tapX_force_func(grip_status_g->G_TAP_FORCE[count], count, &TAP_BIT0[count]);
		}
		if(grip_status_g->G_TAP_MIN_POS[count] > 0){
			grip_tapX_min_position_func(grip_status_g->G_TAP_MIN_POS[count], count, &TAP_BIT2[count]);
		}
		if(grip_status_g->G_TAP_MAX_POS[count] > 0){
			grip_tapX_max_position_func(grip_status_g->G_TAP_MAX_POS[count], count, &TAP_BIT3[count]);
		}
		if(grip_status_g->G_TAP_DELTA_RELEASE_FORCE[count] > 0)
			grip_tapX_delta_release_force_func(grip_status_g->G_TAP_DELTA_RELEASE_FORCE[count], count, &TAP_BIT4[count]);
		if(grip_status_g->G_TAP_DELTA_TAP_FORCE[count] > 0)
			grip_tapX_delta_tap_force_func(grip_status_g->G_TAP_DELTA_TAP_FORCE[count], count, &TAP_BIT4[count]);
		if(grip_status_g->G_TAP_SLOPE_RELEASE_FORCE[count] > 0)
			grip_tapX_slope_release_force_func(grip_status_g->G_TAP_SLOPE_RELEASE_FORCE[count], count, &TAP_BIT5[count]);
		if(grip_status_g->G_TAP_SLOPE_TAP_FORCE[count] > 0)
			grip_tapX_slope_tap_force_func(grip_status_g->G_TAP_SLOPE_TAP_FORCE[count], count, &TAP_BIT5[count]);
		if(grip_status_g->G_TAP_SLOPE_WINDOW[count] > 0)
			grip_tapX_slope_window_func(grip_status_g->G_TAP_SLOPE_WINDOW[count], count, &TAP_BIT1[count]);
	}

	if(grip_status_g->G_TAP_SENSE_SET > 0){
		grip_tap_sense_enable_func(0);
		grip_tap_sense_enable_func(1);
	}

	/* Slide Part */
	for(count=0;count < slide_num;count++){
		if(grip_status_g->G_SLIDE_EN[count] > 0)
			grip_slideX_enable_func(grip_status_g->G_SLIDE_EN[count], count, &SLIDE_BIT0[count]);
		if(grip_status_g->G_SLIDE_DIST[count] > 0)
			grip_slideX_dist_func(grip_status_g->G_SLIDE_DIST[count], count, &SLIDE_BIT3[count]);
		if(grip_status_g->G_SLIDE_FORCE[count] > 0)
			grip_slideX_force_func(grip_status_g->G_SLIDE_FORCE[count], count, &SLIDE_BIT3[count]);
	}

	/* Swipe Part */
	for(count=0;count < swipe_num;count++){
		if(grip_status_g->G_SWIPE_EN[count] > 0)
			grip_swipeX_enable_func(grip_status_g->G_SWIPE_EN[count], count, &SWIPE_BIT0[count]);
		if(grip_status_g->G_SWIPE_VELOCITY[count] > 0)
			grip_swipeX_velocity_func(grip_status_g->G_SWIPE_VELOCITY[count], count, &SWIPE_BIT0[count]);
		if(grip_status_g->G_SWIPE_LEN[count] > 0)
			grip_swipeX_len_func(grip_status_g->G_SWIPE_LEN[count], count, &SWIPE_BIT3[count]);
	}

	/* Bar control, Health check and tap status*/
	PRINT_INFO("Reset check: Bar control, Health check and tap status ");
	Health_Check_Enable_No_Delay(0);
	grip_check_DPC_and_sensitivity_func();
	Grip_K_data_recovery();
	Grip_ReEnable_Squeeze_Check();

	if(need_en==1){
		PRINT_INFO("Disable grip at final due to G_EN != 1");
		grip_status_g->G_EN=0;
	}
	Into_DeepSleep_fun();
	grip_input_event_report(65535, 0, 0, 0, 0, 0, 0);
}

void Power_Control(int en){
	int gpio_req;
	gpio_req = of_get_named_gpio(snt8100fsr_g->dev->of_node, SNT_RST_NAME, 0);
	if(en == 0){
		grip_input_event_report(119, 0, 0, 0, 0, 0, 0);
		g_snt_power_state = 0;
		PRINT_INFO("Set pinctl: 1V2 2V8 down");
		snt8100fsr_g->mgrip_2v8_asus_func->grip_regulator_disable();
		snt8100fsr_g->mgrip_1v2_asus_func->grip_regulator_disable();
		snt8100fsr_g->mgrip_1v8_asus_func->grip_regulator_disable();
		PRINT_INFO("Set pinctl: RST down");
		gpio_direction_output(gpio_req, 0); //output low
		msleep(50);
	}else if(en == 1){
		grip_input_event_report(119, 0, 0, 0, 0, 0, 0);
		PRINT_INFO("Set pinctl: RST down");
		gpio_request(gpio_req, "snt_rst_gpio");
		gpio_direction_output(gpio_req, 0); //output low
		msleep(5);
		if(0 == g_snt_power_state){
			snt8100fsr_g->mgrip_1v8_asus_func->grip_regulator_enable();
			snt8100fsr_g->mgrip_1v2_asus_func->grip_regulator_enable();
			snt8100fsr_g->mgrip_2v8_asus_func->grip_regulator_enable();
			msleep(5);
		}
		PRINT_INFO("Set pinctl: RST up");
		gpio_direction_output(gpio_req, 1); //output high
		g_snt_power_state = 1;
	}/*
	else if(en == 2){
		if(snt8100fsr_g->count_2v8 >= 1){
			snt8100fsr_g->mgrip_2v8_asus_func->grip_regulator_disable();
			msleep(5);
		}
	}else if(en ==3){
		if(snt8100fsr_g->count_2v8 <= 0){
			snt8100fsr_g->mgrip_2v8_asus_func->grip_regulator_enable();
			msleep(5);
		}
	}*/
}
