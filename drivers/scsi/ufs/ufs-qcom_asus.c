#include "ufs-qcom_asus.h"
#include <scsi/fc_frame.h>	//include to use ntohll API +++
#if IS_ENABLED(CONFIG_EXPANDMEM)
#include "../sd.h"
#endif

static  u8 pre_Pre_EOL;
static  u8 pre_life_time_A;
static  u8 pre_life_time_B;
static char* ufs_vendor;
static char* ufs_spec_ver;
static bool init_ufs_health_flag = false;
static char ufs_fw_rev[5]={0};

enum ufs_desc_def_size {
	QUERY_DESC_DEVICE_DEF_SIZE		= 0x59,
	QUERY_DESC_CONFIGURATION_DEF_SIZE	= 0x90,
	QUERY_DESC_UNIT_DEF_SIZE		= 0x2D,
	QUERY_DESC_INTERCONNECT_DEF_SIZE	= 0x06,
	QUERY_DESC_GEOMETRY_DEF_SIZE		= 0x48,
	QUERY_DESC_POWER_DEF_SIZE		= 0x62,
	QUERY_DESC_HEALTH_DEF_SIZE		= 0x25,
};

static bool init_flag_size = false;
static bool init_flag_fw_rev = false;

static struct {
	u32 manfid;
	char *band_type;
} ufs_vendor_tbl[] = {
	{ 0x12C, "MICRON" },
	{ 0x1AD, "HYNIX" },
	{ 0x1CE, "SAMSUNG" },
	{ 0x198, "TOSHIBA" },
};
#define UFS_VENDOR_TBL_MAX	(sizeof(ufs_vendor_tbl)/sizeof(ufs_vendor_tbl[0]))
//========================================================================================================================
static inline int ufshcd_read_desc(struct ufs_hba *hba,
				   enum desc_idn desc_id,
				   int desc_index,
				   void *buf,
				   u32 size)
{
	return ufshcd_read_desc_param(hba, desc_id, desc_index, 0, buf, size);
}

static int ufshcd_read_device_desc(struct ufs_hba *hba, u8 *buf, u32 size)
{
	return ufshcd_read_desc(hba, QUERY_DESC_IDN_DEVICE, 0, buf, size);
}

int ufshcd_read_geometry_desc(struct ufs_hba *hba, u8 *buf, u32 size)
{
	return ufshcd_read_desc(hba, QUERY_DESC_IDN_GEOMETRY, 0, buf, size);
}

int ufshcd_read_unit_desc(struct ufs_hba *hba, int desc_index, u8 *buf, u32 size)
{
	return ufshcd_read_desc(hba, QUERY_DESC_IDN_UNIT, desc_index, buf, size);
}

int ufshcd_read_health_desc(struct ufs_hba *hba, u8 *buf, u32 size)
{
	return ufshcd_read_desc(hba, QUERY_DESC_IDN_HEALTH, 0, buf, size);
}

int ufshcd_read_string_desc_asus(struct ufs_hba *hba, int desc_index, u8 *buf, u32 size)
{
	return ufshcd_read_desc(hba, QUERY_DESC_IDN_STRING, desc_index, buf, size);
}

static void get_ufs_size(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int err=0;
	int buff_len = QUERY_DESC_GEOMETRY_DEF_SIZE;
	u8 desc_buf[QUERY_DESC_GEOMETRY_DEF_SIZE];

	pm_runtime_get_sync(hba->dev);
	err = ufshcd_read_geometry_desc(hba, desc_buf, buff_len);
	pm_runtime_put_sync(hba->dev);

	host->ufs_info.ufs_size = ntohll(*(u64 *)&desc_buf[0x4]);

	if(host->ufs_info.ufs_size > 0x70000000)
		sprintf(host->ufs_info.ufs_total_size, "1024");
	else if(host->ufs_info.ufs_size > 0x30000000)
		sprintf(host->ufs_info.ufs_total_size, "512");
	else if(host->ufs_info.ufs_size > 0x10000000)
		sprintf(host->ufs_info.ufs_total_size, "256");
	else if(host->ufs_info.ufs_size > 0xe000000)
		sprintf(host->ufs_info.ufs_total_size, "128");
	else if(host->ufs_info.ufs_size > 0x7000000)
		sprintf(host->ufs_info.ufs_total_size, "64");
	else if(host->ufs_info.ufs_size > 0x3000000)
		sprintf(host->ufs_info.ufs_total_size, "32");
	else
		sprintf(host->ufs_info.ufs_total_size, "16");

	init_flag_size = true;

	return;
}

static int get_ufs_health(struct ufs_hba *hba, u8 index)
{
	int err=0;
	int buff_len = QUERY_DESC_HEALTH_DEF_SIZE;
	u8 desc_buf[QUERY_DESC_HEALTH_DEF_SIZE];

	printk("[UFS] get_ufs_health : 0x%x\n", index);
	pm_runtime_get_sync(hba->dev);
	err = ufshcd_read_health_desc(hba, desc_buf, buff_len);
	pm_runtime_put_sync(hba->dev);

	return (u8)desc_buf[index];
}

static void get_ufs_fw_version(struct ufs_hba *hba, char *rev_buf, bool realtime)
{
	int err=0;
	u8 model_index;
	int buff_len = QUERY_DESC_DEVICE_DEF_SIZE;
	u8 desc_buf[QUERY_DESC_DEVICE_DEF_SIZE];
	u8 str_desc_buf[QUERY_DESC_MAX_SIZE];
	u16 str_desc_h[4]={0};

	if(!realtime){
		if(init_flag_fw_rev){
			memcpy(rev_buf, ufs_fw_rev, 4);
			return;
		}
		printk("ufs_d: ufs_fw_rev is uninitialized. reading...\n");
	}
	//===get ufs fw version===
	pm_runtime_get_sync(hba->dev);
	err = ufshcd_read_device_desc(hba, desc_buf, buff_len);
	pm_runtime_put_sync(hba->dev);

	model_index=desc_buf[0x2A];	// REV

	pm_runtime_get_sync(hba->dev);
	err = ufshcd_read_string_desc_asus(hba, model_index, str_desc_buf, QUERY_DESC_MAX_SIZE);
	pm_runtime_put_sync(hba->dev);
	//printk("ufs_d: 	get_ufs_fw_version: (%X) (%x) (%x) (%x) (%x) (%x) (%x) (%x) (%x) (%x)\n",str_desc_buf[0], str_desc_buf[1], str_desc_buf[2],str_desc_buf[3],str_desc_buf[4],str_desc_buf[5],str_desc_buf[6],str_desc_buf[7],str_desc_buf[8],str_desc_buf[9]);
	str_desc_h[0]= (str_desc_buf[2]<<8) |  str_desc_buf[3];
	str_desc_h[1]= (str_desc_buf[4]<<8) |  str_desc_buf[5];
	str_desc_h[2]= (str_desc_buf[6]<<8) |  str_desc_buf[7];
	str_desc_h[3]= (str_desc_buf[8]<<8) |  str_desc_buf[9];

	snprintf( rev_buf, 255,"%c%c%c%c", str_desc_h[0],str_desc_h[1],str_desc_h[2],str_desc_h[3] );
	memcpy(ufs_fw_rev, rev_buf, 4);

	init_flag_fw_rev = true;

//	printk("ufs_d: rev_buf=%s ufs_fw_rev=(%s)(%04s)\n", rev_buf, ufs_fw_rev, ufs_fw_rev );
}

static void get_ufs_status(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int err=0, i;
	int buff_len = QUERY_DESC_DEVICE_DEF_SIZE;
	u8 desc_buf[QUERY_DESC_DEVICE_DEF_SIZE];
	int manfid, spec_ver;

	pm_runtime_get_sync(hba->dev);
	err = ufshcd_read_device_desc(hba, desc_buf, buff_len);
	pm_runtime_put_sync(hba->dev);

	manfid = ntohs(*(u16 *)&desc_buf[0x18]);
	printk("[UFS] manfid : 0x%x\n", manfid);

	spec_ver = ntohs(*(u16 *)&desc_buf[0x10]);
	printk("[UFS] spec_ver : 0x%x\n", spec_ver);

	memset(host->ufs_info.ufs_status, 0, sizeof(host->ufs_info.ufs_status));

	for (i = 0; i < UFS_VENDOR_TBL_MAX; i++) {
		if (manfid == ufs_vendor_tbl[i].manfid) {
			strcpy(host->ufs_info.ufs_status, ufs_vendor_tbl[i].band_type);
			if(!init_ufs_health_flag)
				ufs_vendor=ufs_vendor_tbl[i].band_type;

			if (spec_ver == 0x200){
				strcat(host->ufs_info.ufs_status, "-v2.0-");
				ufs_spec_ver = "v2.0";
			}
			else if (spec_ver == 0x210){
				strcat(host->ufs_info.ufs_status, "-v2.1-");
				ufs_spec_ver = "v2.1";
			}
			else if (spec_ver == 0x300){
				strcat(host->ufs_info.ufs_status, "-v3.0-");
				ufs_spec_ver = "v3.0";
			}
			else if (spec_ver == 0x310){
				strcat(host->ufs_info.ufs_status, "-v3.1-");
				ufs_spec_ver = "v3.1";
			}

			if(!init_flag_size)
				get_ufs_size(hba);

			strcat(host->ufs_info.ufs_status, host->ufs_info.ufs_total_size);
			strcat(host->ufs_info.ufs_status, "G");

			return;
		}
	}

	strcpy(host->ufs_info.ufs_status, "Unknown");
	return;
}

static ssize_t ufshcd_ufs_total_size_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int curr_len;

	if(!init_flag_size)
		get_ufs_size(hba);

	curr_len = snprintf(buf, PAGE_SIZE,"%s\n", host->ufs_info.ufs_total_size);
	return curr_len;
}

static ssize_t ufshcd_ufs_total_size_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}


static void ufshcd_add_ufs_total_size_sysfs_nodes(struct ufs_qcom_host *host)
{
//struct device *dev = host->hba->dev;
	host->ufs_info.ufs_total_size_attr.show = ufshcd_ufs_total_size_show;
	host->ufs_info.ufs_total_size_attr.store = ufshcd_ufs_total_size_store;
	sysfs_attr_init(&host->ufs_info.ufs_total_size_attr.attr);
	host->ufs_info.ufs_total_size_attr.attr.name = "ufs_total_size";
	host->ufs_info.ufs_total_size_attr.attr.mode = S_IRUGO | S_IWUSR;
	if (device_create_file(host->hba->dev, &host->ufs_info.ufs_total_size_attr))
		dev_err(host->hba->dev, "Failed to create sysfs for ufs_total_size\n");
}

static ssize_t ufshcd_ufs_size_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int curr_len;

	if(!init_flag_size)
		get_ufs_size(hba);

	curr_len = snprintf(buf, PAGE_SIZE,"0x%llx\n", host->ufs_info.ufs_size);

	return curr_len;
}

static ssize_t ufshcd_ufs_size_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static void ufshcd_add_ufs_size_sysfs_nodes(struct ufs_qcom_host *host)
{
	host->ufs_info.ufs_size_attr.show = ufshcd_ufs_size_show;
	host->ufs_info.ufs_size_attr.store = ufshcd_ufs_size_store;
	sysfs_attr_init(&host->ufs_info.ufs_size_attr.attr);
	host->ufs_info.ufs_size_attr.attr.name = "ufs_size";
	host->ufs_info.ufs_size_attr.attr.mode = S_IRUGO | S_IWUSR;
	if (device_create_file(host->hba->dev, &host->ufs_info.ufs_size_attr))
		dev_err(host->hba->dev, "Failed to create sysfs for ufs_size\n");
}

static ssize_t ufshcd_ufs_preEOL_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	int curr_len;
	u8 index = 0x02, val;

	val = get_ufs_health(hba, index);

	curr_len = snprintf(buf, PAGE_SIZE,"0x%02x\n", val);
	return curr_len;
}

static ssize_t ufshcd_ufs_preEOL_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static void ufshcd_add_ufs_preEOL_sysfs_nodes(struct ufs_qcom_host *host)
{
	host->ufs_info.ufs_preEOL_attr.show = ufshcd_ufs_preEOL_show;
	host->ufs_info.ufs_preEOL_attr.store = ufshcd_ufs_preEOL_store;
	sysfs_attr_init(&host->ufs_info.ufs_preEOL_attr.attr);
	host->ufs_info.ufs_preEOL_attr.attr.name = "ufs_preEOL";
	host->ufs_info.ufs_preEOL_attr.attr.mode = S_IRUGO | S_IWUSR;
	if (device_create_file(host->hba->dev, &host->ufs_info.ufs_preEOL_attr))
		dev_err(host->hba->dev, "Failed to create sysfs for ufs_preEOL\n");
}

static ssize_t ufshcd_ufs_health_A_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	int curr_len;
	u8 index = 0x03, val;

	val = get_ufs_health(hba, index);

	curr_len = snprintf(buf, PAGE_SIZE,"0x%02x\n", val);
	return curr_len;
}

static ssize_t ufshcd_ufs_health_A_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static void ufshcd_add_ufs_health_A_sysfs_nodes(struct ufs_qcom_host *host)
{
	host->ufs_info.ufs_health_A_attr.show = ufshcd_ufs_health_A_show;
	host->ufs_info.ufs_health_A_attr.store = ufshcd_ufs_health_A_store;
	sysfs_attr_init(&host->ufs_info.ufs_health_A_attr.attr);
	host->ufs_info.ufs_health_A_attr.attr.name = "ufs_health_A";
	host->ufs_info.ufs_health_A_attr.attr.mode = S_IRUGO | S_IWUSR;
	if (device_create_file(host->hba->dev, &host->ufs_info.ufs_health_A_attr))
		dev_err(host->hba->dev, "Failed to create sysfs for ufs_health_A_attr\n");
}

static ssize_t ufshcd_ufs_health_B_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	int curr_len;
	u8 index = 0x04, val;

	val = get_ufs_health(hba, index);

	curr_len = snprintf(buf, PAGE_SIZE,"0x%02x\n", val);
	return curr_len;
}

static ssize_t ufshcd_ufs_health_B_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static void ufshcd_add_ufs_health_B_sysfs_nodes(struct ufs_qcom_host *host)
{
	host->ufs_info.ufs_health_B_attr.show = ufshcd_ufs_health_B_show;
	host->ufs_info.ufs_health_B_attr.store = ufshcd_ufs_health_B_store;
	sysfs_attr_init(&host->ufs_info.ufs_health_B_attr.attr);
	host->ufs_info.ufs_health_B_attr.attr.name = "ufs_health_B";
	host->ufs_info.ufs_health_B_attr.attr.mode = S_IRUGO | S_IWUSR;
	if (device_create_file(host->hba->dev, &host->ufs_info.ufs_health_B_attr))
		dev_err(host->hba->dev, "Failed to create sysfs for ufs_health_B_attr\n");
}

static ssize_t ufshcd_ufs_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int curr_len;

	get_ufs_status(hba);

	curr_len = snprintf(buf, PAGE_SIZE,"%s\n", host->ufs_info.ufs_status);
	return curr_len;
}

static ssize_t ufshcd_ufs_status_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static void ufshcd_add_ufs_status_sysfs_nodes(struct ufs_qcom_host *host)
{
	host->ufs_info.ufs_status_attr.show = ufshcd_ufs_status_show;
	host->ufs_info.ufs_status_attr.store = ufshcd_ufs_status_store;
	sysfs_attr_init(&host->ufs_info.ufs_status_attr.attr);
	host->ufs_info.ufs_status_attr.attr.name = "ufs_status";
	host->ufs_info.ufs_status_attr.attr.mode = S_IRUGO | S_IWUSR;
	if (device_create_file(host->hba->dev, &host->ufs_info.ufs_status_attr))
		dev_err(host->hba->dev, "Failed to create sysfs for ufs_status_attr\n");
}

static ssize_t ufshcd_ufs_productID_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufs_dev_info *dev_info = &hba->dev_info;
	int curr_len;

	curr_len = snprintf(buf, PAGE_SIZE,"%s\n", dev_info->model);
	return curr_len;
}

static ssize_t ufshcd_ufs_productID_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static void ufshcd_add_ufs_productID_sysfs_nodes(struct ufs_qcom_host *host)
{
	host->ufs_info.ufs_productID_attr.show = ufshcd_ufs_productID_show;
	host->ufs_info.ufs_productID_attr.store = ufshcd_ufs_productID_store;
	sysfs_attr_init(&host->ufs_info.ufs_productID_attr.attr);
	host->ufs_info.ufs_productID_attr.attr.name = "ufs_productID";
	host->ufs_info.ufs_productID_attr.attr.mode = S_IRUGO | S_IWUSR;
	if (device_create_file(host->hba->dev, &host->ufs_info.ufs_productID_attr))
		dev_err(host->hba->dev, "Failed to create sysfs for ufs_productID_attr\n");
}

static ssize_t ufshcd_ufs_fw_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	int curr_len;
	char rev_buf[256] = { 0, };

	get_ufs_fw_version(hba, rev_buf, false);

	curr_len = snprintf(buf, PAGE_SIZE,"%s\n", rev_buf);
	return curr_len;
}

static ssize_t ufshcd_ufs_fw_version_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static void ufshcd_add_ufs_fw_version_sysfs_nodes(struct ufs_qcom_host *host)
{
	host->ufs_info.ufs_fw_version_attr.show = ufshcd_ufs_fw_version_show;
	host->ufs_info.ufs_fw_version_attr.store = ufshcd_ufs_fw_version_store;
	sysfs_attr_init(&host->ufs_info.ufs_fw_version_attr.attr);
	host->ufs_info.ufs_fw_version_attr.attr.name = "ufs_fw_version";
	host->ufs_info.ufs_fw_version_attr.attr.mode = S_IRUGO | S_IWUSR;
	if (device_create_file(host->hba->dev, &host->ufs_info.ufs_fw_version_attr))
		dev_err(host->hba->dev, "Failed to create sysfs for ufs_fw_version_attr\n");
}

void ufshcd_add_sysfs_nodes(struct ufs_qcom_host *host)
{
	ufshcd_add_ufs_total_size_sysfs_nodes(host);
	ufshcd_add_ufs_size_sysfs_nodes(host);
	ufshcd_add_ufs_preEOL_sysfs_nodes(host);
	ufshcd_add_ufs_health_A_sysfs_nodes(host);
	ufshcd_add_ufs_health_B_sysfs_nodes(host);
	ufshcd_add_ufs_status_sysfs_nodes(host);
	ufshcd_add_ufs_productID_sysfs_nodes(host);
	ufshcd_add_ufs_fw_version_sysfs_nodes(host);
}
//ASUS_BSP Deeo : Get UFS info ---

void ufshcd_remove_sysfs_nodes(struct ufs_qcom_host *host)
{
	device_remove_file(host->hba->dev, &host->ufs_info.ufs_total_size_attr);
	device_remove_file(host->hba->dev, &host->ufs_info.ufs_size_attr);
	device_remove_file(host->hba->dev, &host->ufs_info.ufs_preEOL_attr);
	device_remove_file(host->hba->dev, &host->ufs_info.ufs_health_A_attr);
	device_remove_file(host->hba->dev, &host->ufs_info.ufs_health_B_attr);
	device_remove_file(host->hba->dev, &host->ufs_info.ufs_status_attr);
	device_remove_file(host->hba->dev, &host->ufs_info.ufs_productID_attr);
	device_remove_file(host->hba->dev, &host->ufs_info.ufs_fw_version_attr);
}

static void ufs_asusevent_log(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int err = 0;
    int buff_len = QUERY_DESC_HEALTH_DEF_SIZE;
	u8 desc_buf[QUERY_DESC_HEALTH_DEF_SIZE];
	char *rev_buf;



    pm_runtime_get_sync(hba->dev);
    err = ufshcd_read_health_desc(hba, desc_buf, buff_len);
	if(err){
		pr_info("ufs_asusevent_log get health failed");
		return ;
	}
    pm_runtime_put_sync(hba->dev);

	if(!init_ufs_health_flag){
		pre_Pre_EOL = desc_buf[2];
		pre_life_time_A = desc_buf[3];
		pre_life_time_B = desc_buf[4];
		get_ufs_status(hba);

		rev_buf=kzalloc(255, GFP_KERNEL);
		memset(rev_buf, 0, 255);
		get_ufs_fw_version(hba, rev_buf, false);

		//printk("[UFS] vendor: %s, ufs_version=%s, ufs_size=%sG, fw_version=%s, lifeA=0x%02x, lifeB=0x%02x, preEOL=0x%02x\n", ufs_vendor, ufs_spec_ver, hba->ufs_total_size, dev_info->version, pre_life_time_A, pre_life_time_B, pre_Pre_EOL);
		//ASUSEvtlog("[UFS] vendor: %s, ufs_version=%s, ufs_size=%sG, fw_version=%s, lifeA=0x%02x, lifeB=0x%02x, preEOL=0x%02x\n", ufs_vendor, ufs_spec_ver, hba->ufs_total_size, dev_info->version, pre_life_time_A, pre_life_time_B, pre_Pre_EOL);
		printk("[UFS] vendor: %s, ufs_version=%s, ufs_size=%sG, fw_version=%s, lifeA=0x%02x, lifeB=0x%02x, preEOL=0x%02x\n", ufs_vendor, ufs_spec_ver, host->ufs_info.ufs_total_size, rev_buf, pre_life_time_A, pre_life_time_B, pre_Pre_EOL);

		init_ufs_health_flag = true;
	}else{
		if(pre_Pre_EOL != desc_buf[2] || pre_life_time_A != desc_buf[3] || pre_life_time_B != desc_buf[4]){

			//printk("[UFS] vendor: %s, ufs_version=%s, ufs_size=%sG, fw_version=%s, lifeA=0x%02x, lifeB=0x%02x, preEOL=0x%02x\n", ufs_vendor, ufs_spec_ver, hba->ufs_total_size, dev_info->version, pre_life_time_A, pre_life_time_B, pre_Pre_EOL);
			//ASUSEvtlog("[UFS] vendor: %s, ufs_version=%s, ufs_size=%sG, fw_version=%s, lifeA=0x%02x, lifeB=0x%02x, preEOL=0x%02x\n", ufs_vendor, ufs_spec_ver, hba->ufs_total_size, dev_info->version, pre_life_time_A, pre_life_time_B, pre_Pre_EOL);
			printk("[UFS] vendor: %s, ufs_version=%s, ufs_size=%sG, fw_version=%s, lifeA=0x%02x, lifeB=0x%02x, preEOL=0x%02x\n", ufs_vendor, ufs_spec_ver, host->ufs_info.ufs_total_size, ufs_fw_rev, pre_life_time_A, pre_life_time_B, pre_Pre_EOL);

			pre_Pre_EOL = desc_buf[2];
			pre_life_time_A = desc_buf[3];
			pre_life_time_B = desc_buf[4];
		}
	}
}

int asus_ufshcd_pltfrm_resume(struct device *dev)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	int ret=0;
	ret=ufshcd_system_resume(hba);
	ufs_asusevent_log(hba);
	return ret;
}

#if IS_ENABLED(CONFIG_EXPANDMEM)
int eswap_ufshcd_get_health_info(struct block_device *bdev,
		u8 *pre_eol_info, u8 *life_time_est_a, u8 *life_time_est_b)
{
	int err = 0;
	int buff_len = QUERY_DESC_HEALTH_DEF_SIZE;
	u8 desc_buf[QUERY_DESC_HEALTH_DEF_SIZE];
	struct ufs_hba *hba = NULL;
	struct scsi_disk *sdkp;
	struct scsi_device *sdev;

	if ((!bdev) || (!pre_eol_info) || (!life_time_est_a) ||
			(!life_time_est_b) || (!bdev->bd_disk)) {
		pr_err("%s dvices null", __func__);
		return -EFAULT;
	}

	sdkp = scsi_disk(bdev->bd_disk);
	if (!sdkp) {
		pr_err("%s scsi disk null", __func__);
		return -EFAULT;
	}

	sdev = sdkp->device;
	if (!sdkp) {
		pr_err("%s scsi device null", __func__);
		return -EFAULT;
	}

	hba = shost_priv(sdev->host);
	if (!hba || !hba->dev) {
		pr_err("%s  hba null", __func__);
		return -EFAULT;
	}

	pm_runtime_get_sync(hba->dev);
	err = ufshcd_read_health_desc(hba, desc_buf, buff_len);
	pm_runtime_put_sync(hba->dev);
	if(err){
		pr_err("%s  get health failed", __func__);
		return err;
	}

	*pre_eol_info = desc_buf[2];
	*life_time_est_a = desc_buf[3];
	*life_time_est_b = desc_buf[4];

	return 0;
}
EXPORT_SYMBOL(eswap_ufshcd_get_health_info);
#endif