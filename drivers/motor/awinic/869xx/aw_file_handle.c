#include <linux/fs.h>

#define AWINIC_CALI_FILE  "/mnt/vendor/persist/aw_cali.bin"
#define AWINIC_RTP_CALI_FILE  "/mnt/vendor/persist/aw_rtp_cali.bin"

#define FW_PATH_SETTING "/sys/module/firmware_class/parameters/path"
#define INTERNAL_BIN_PATH "/vendor/firmware/awinic/"


void add_bin_search_path(char *path)
{
	struct file *fp;
	char buf[50] = {0};

	loff_t pos = 0;

	fp = filp_open(FW_PATH_SETTING, O_RDWR | O_CREAT, 0644);

	if (IS_ERR(fp)) {
		pr_err("%s: open %s failed!\n",__func__, FW_PATH_SETTING);
		return;
	}

	snprintf(buf, sizeof(buf), "%s", path);

	kernel_write(fp, buf, strlen(buf), &pos);

	printk("aw869xx: %s: add bin search path=%s\n", __func__, buf);

	filp_close(fp, NULL);
}



static int write_cali_to_file(unsigned int cali_re, int channel, int type)
{
	struct file *fp;
	char buf[50] = {0};
	loff_t pos = 0;
//	mm_segment_t fs;

	if (type == 0)
	fp = filp_open(AWINIC_CALI_FILE, O_RDWR | O_CREAT, 0644);
	else if (type == 1)
		fp = filp_open(AWINIC_RTP_CALI_FILE, O_RDWR | O_CREAT, 0644);

	if (IS_ERR(fp)) {
		pr_err("aw869xx: %s:channel:%d open %s failed!\n",__func__, channel, type ? AWINIC_RTP_CALI_FILE:AWINIC_CALI_FILE);
		return -EINVAL;
	}

	//snprintf(buf, PAGE_SIZE, "%d", cali_re);
	snprintf(buf, 50, "%d", cali_re);

//	fs = get_fs();
//	set_fs(KERNEL_DS);

//	vfs_write(fp, buf, strlen(buf), &pos);
	kernel_write(fp, buf, strlen(buf), &pos);
//	set_fs(fs);

	pr_info("aw869xx: %s: channel:%d buf:%s type:%d cali:%d\n",
		__func__, channel, buf, type, cali_re);

	filp_close(fp, NULL);
	return 0;
}

static int get_cali_from_file(unsigned int *cali_re, int channel, int type)
{
	struct file *fp;
	/*struct inode *node;*/
	int f_size;
	char *buf;
	unsigned int int_cali_re = 0;

	loff_t pos = 0;
//	mm_segment_t fs;

	if (type == 0)
	fp = filp_open(AWINIC_CALI_FILE, O_RDWR, 0);
	else if (type == 1)
		fp = filp_open(AWINIC_RTP_CALI_FILE, O_RDWR, 0);

	if (IS_ERR(fp)) {
		pr_err("aw869xx: %s:channel:%d open %s failed!\n",__func__, channel, type ? AWINIC_RTP_CALI_FILE:AWINIC_CALI_FILE);
		return -EINVAL;
	}

	/*node = fp->f_dentry->d_inode;*/
	/*f_size = node->i_size;*/
	f_size = sizeof(unsigned int);

	buf = kzalloc(f_size + 1, GFP_ATOMIC);
	if (!buf) {
		pr_err("aw869xx: %s: channel:%d malloc mem %d failed!\n",
			__func__, channel, f_size);
		filp_close(fp, NULL);
		return -EINVAL;
	}

//	fs = get_fs();
//	set_fs(KERNEL_DS);

//	vfs_read(fp, buf, f_size, &pos);
	kernel_read(fp, buf, f_size, &pos);

//	set_fs(fs);

	if (sscanf(buf, "%d", &int_cali_re) == 1)
		*cali_re = int_cali_re;
	else
		*cali_re = 0x7777;

	pr_info("aw869xx: %s: channel:%d buf:%s type:%d cali:%d\n",
		__func__, channel, buf, type, int_cali_re);

	filp_close(fp, NULL);

	return  0;

}
