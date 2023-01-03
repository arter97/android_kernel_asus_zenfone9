#define MODULENAME OIS
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
//gcc -E __FILE__
static struct cdev dev;
static dev_t dev_num;
#define MODULE_QUATE(M) # M
#define MODULE_DEV(M) MODULE_QUATE(M)
#define MODULETAG(M) "[" MODULE_QUATE(M) "]"
#define MODULD_CN(NAME, EXT) NAME ## _ ## EXT
#define DT(A,B) MODULD_CN(A,B)
#define DEF_IO(M) static long DT(M,ioctl)(struct file *file, unsigned int cmd, unsigned long arg)
#define DEF_INIT(M) int DT(M,dev_init)(void)
#define DEF_EXIT(M) void DT(M,dev_exit)(void)
#define FILE_OP(A, B, M) static struct DT(A,B) DT(M,fops) = { .owner = THIS_MODULE, .unlocked_ioctl = DT(M,ioctl) }
#define MODULENAME_OP(OP)  _IOW('G',OP,int)
static struct class *DT(MODULENAME,class) = NULL;
typedef uint8_t (*ioctl_f)(unsigned long);
extern ioctl_f ioctl[];
extern uint16_t ioctl_size;
DEF_IO(MODULENAME) {
	uint8_t i=0;
	pr_err(MODULETAG(MODULENAME)" %s cmd=%d", __func__, cmd);
	for (i=0;i<ioctl_size;i++) {
		if (cmd==MODULENAME_OP(i)) {
			return (*ioctl[i])(arg);
		}
	}
	pr_err(MODULETAG(MODULENAME)" not valid command\n");
	return -EINVAL;
}
FILE_OP(file,operations,MODULENAME);
DEF_INIT(MODULENAME) {
	alloc_chrdev_region(&dev_num,0,1,MODULE_DEV(MODULENAME));
	cdev_init(&dev, &DT(MODULENAME,fops));
	dev.owner = MODULD_CN(THIS,MODULE);
	cdev_add(&dev,dev_num,1);
	DT(MODULENAME,class) = class_create(THIS_MODULE,MODULE_DEV(MODULENAME));
	device_create(DT(MODULENAME,class),NULL,dev_num,NULL,"%s",MODULE_DEV(MODULENAME));
	pr_err(MODULETAG(MODULENAME)" init %s device is OK!\n", MODULE_DEV(MODULENAME));
	return 0;
}
DEF_EXIT(MODULENAME) {
	device_destroy(DT(MODULENAME,class),dev_num);
	class_destroy(DT(MODULENAME,class));
	cdev_del(&dev);
	unregister_chrdev_region(dev_num,1);
}
