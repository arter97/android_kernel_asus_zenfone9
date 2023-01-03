#include <linux/proc_fs.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

extern int sntSensor_miscRegister(void);

//Function: DPC wake from low power mode
void Wait_Wake_For_RegW(void);
extern void DPC_write_func(int flag);

// Gesture enable func
void grip_raw_enable_func(int val);
void grip_enable_func_noLock(int val);
void grip_tap_sense_enable_func(int val);


#ifdef DYNAMIC_PWR_CTL
extern int snt_activity_request(void);
#endif

int Health_Check_Enable(int en);
int Health_Check(uint16_t val);
void Into_DeepSleep_fun(void);

/* used to record health check value */
extern uint16_t FPC_value;

extern bool G_Skip_Sq1_Long;
extern bool G_Skip_Sq2_Long;

/* Workaround for stucked semaphore */
extern struct delayed_work check_stuck_wake;
void check_stuck_semaphore(struct work_struct *work);


extern struct delayed_work rst_recovery_wk;
extern struct delayed_work rst_gpio_wk;
extern void Reset_Func(struct work_struct *work);
extern void grip_dump_status_func(struct work_struct *work);
extern struct workqueue_struct *asus_wq;


extern void set_sq_gesture(uint16_t slide_id, uint16_t reg_val, int index);
extern void set_tap_gesture(uint16_t slide_id, uint16_t reg_val, int index);
extern void set_slide_gesture(uint16_t slide_id, uint16_t reg_val, int index);
extern void set_swipe_gesture(uint16_t slide_id, uint16_t reg_val, int index);

extern void get_sq_gesture(uint16_t tap_id, uint16_t reg_val, int index, int len);
/* Enable/disable Grip Sensor Power 1V2_2V8 */
void Power_Control(int en);

extern void Grip_Driver_IRQ_EN(bool flag);
extern void Grip_Chip_IRQ_EN(bool flag);

extern int grip_game_gesture_status(void);
void Grip_Driver_IRQ_EN(bool flag);
void Grip_ReEnable_Squeeze_Check(void);

void grip_tapX_enable_func(int val, int id, uint16_t *reg_val);
void grip_raw_enable_func(int val);
void grip_tapX_force_func(int val, int id, uint16_t *reg_val);
void grip_tapX_min_position_func(int val, int id, uint16_t *reg_val);
void grip_tapX_max_position_func(int val, int id, uint16_t *reg_val);
void grip_tapX_slope_window_func(int val, int id, uint16_t *reg_val);
void grip_tapX_slope_tap_force_func(int val, int id, uint16_t *reg_val);
void grip_tapX_slope_release_force_func(int val, int id, uint16_t *reg_val);
void grip_tapX_delta_tap_force_func(int val, int id, uint16_t *reg_val);
void grip_tapX_delta_release_force_func(int val, int id, uint16_t *reg_val);
void grip_tapX_vibrator_enable_func(int val, int tap_id);
void grip_tapX_vibrator_repeat_enable_func(int val, int tap_id);
void grip_tapX_vibrator_repeat_dur_func(int val, int tap_id);
void grip_squeezeX_enable_func(int val, int id, uint16_t *reg_val);
void grip_squeezeX_force_func(int val, int id, uint16_t *reg_val);
void grip_squeezeX_short_dur_func(int val, int id, uint16_t *reg_val);
void grip_squeezeX_long_dur_func(int val, int id, uint16_t *reg_val);
void grip_squeezeX_BarA_drop_rate_func(int val, int id, uint16_t *reg_val);
void grip_squeezeX_BarA_drop_rate_acc_func(int val, int id, uint16_t *reg_val);
void grip_squeezeX_BarA_up_rate_func(int val, int id, uint16_t *reg_val);
void grip_squeezeX_BarA_up_rate_acc_func(int val, int id, uint16_t *reg_val);
void grip_squeezeX_BarB_drop_rate_func(int val, int id, uint16_t *reg_val);
void grip_squeezeX_BarB_drop_rate_acc_func(int val, int id, uint16_t *reg_val);
void grip_squeezeX_BarB_up_rate_func(int val, int id, uint16_t *reg_val);
void grip_squeezeX_BarB_up_rate_acc_func(int val, int id, uint16_t *reg_val);
void grip_slideX_enable_func(int val, int id, uint16_t *reg_val);
void grip_slideX_dist_func(int val, int id, uint16_t *reg_val);
void grip_slideX_2nd_dist_func(int val, int id, uint16_t *reg_val);
void grip_slideX_force_func(int val, int id, uint16_t *reg_val);
void grip_slideX_min_position_func(int val, int id, uint16_t *reg_val);
void grip_slideX_max_position_func(int val, int id, uint16_t *reg_val);
void grip_slideX_vibrator_enable_func(int val, int id);
void grip_slideX_tap_priority_func(int val, int id, uint16_t* reg_val);
void grip_swipeX_enable_func(int val, int id, uint16_t *reg_val);
void grip_swipeX_velocity_func(int val, int id, uint16_t *reg_val);
void grip_swipeX_len_func(int val, int id, uint16_t *reg_val);
void grip_swipeX_min_position_func(int val, int id, uint16_t *reg_val);
void grip_swipeX_max_position_func(int val, int id, uint16_t *reg_val);



//extern enum DEVICE_PROJID g_ASUS_prjID;
//extern enum DEVICE_HWID g_ASUS_hwID;
