#include <linux/kernel.h>
#include <linux/module.h>

extern unsigned char Current_OIS_Status;
uint8_t Loop_Gain_Cali(void);
uint8_t executeLoopGain(unsigned long arg) {
	return Loop_Gain_Cali();
}
uint8_t getStatus(unsigned long arg) {
	uint8_t ret=0;
	pr_err("[OIS] Loop_Gain_getStatus OK, Current_OIS_Status=%d\n", Current_OIS_Status);
	ret = copy_to_user((int __user*)arg, &Current_OIS_Status, sizeof(Current_OIS_Status));
	return ret;
}
uint8_t cancelLoopGain(unsigned long arg) {
	Current_OIS_Status=1;
	pr_err("[OIS] cancel loop gain\n");
	return 0;
}
typedef uint8_t (*ioctl_f)(unsigned long);
ioctl_f ioctl[] = { &executeLoopGain, &cancelLoopGain, &getStatus };
uint16_t ioctl_size=sizeof(ioctl)/sizeof(ioctl_f);
