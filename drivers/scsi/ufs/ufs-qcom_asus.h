#include <linux/device.h>
#include "ufs-qcom.h"

void ufshcd_add_sysfs_nodes(struct ufs_qcom_host *host);
void ufshcd_remove_sysfs_nodes(struct ufs_qcom_host *host);
int asus_ufshcd_pltfrm_resume(struct device *dev);
