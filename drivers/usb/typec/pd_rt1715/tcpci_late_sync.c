
#include <linux/usb/tcpci.h>
#include <linux/usb/tcpci_typec.h>

#if IS_ENABLED(CONFIG_USB_POWER_DELIVERY)
#include "pd_dpm_prv.h"
#include <linux/usb/rt_tcpm.h>
#endif /* CONFIG_USB_POWER_DELIVERY */

#define TCPC_CORE_VERSION		"2.0.10_G"

#ifdef CONFIG_TCPC_NOTIFIER_LATE_SYNC
static int __tcpc_class_complete_work(struct device *dev, void *data)
{
	struct tcpc_device *tcpc = dev_get_drvdata(dev);

	printk("%s\n", __func__);
	if (tcpc != NULL) {
		printk("%s = %s\n", __func__, dev_name(dev));
#if 0
		tcpc_device_irq_enable(tcpc);
#else
		schedule_delayed_work(&tcpc->init_work,
			msecs_to_jiffies(1000));
#endif
	}
	return 0;
}

static int __init tcpc_class_complete_init(void)
{
	printk("%s +++\n", __func__);
	if (!IS_ERR(tcpc_class)) {
		class_for_each_device(tcpc_class, NULL, NULL,
			__tcpc_class_complete_work);
	}
	printk("%s ---\n", __func__);
	return 0;
}
late_initcall_sync(tcpc_class_complete_init);
#endif /* CONFIG_TCPC_NOTIFIER_LATE_SYNC */

MODULE_DESCRIPTION("Richtek TypeC Port Late Sync Driver");
MODULE_AUTHOR("Jeff Chang <jeff_chang@richtek.com>");
MODULE_VERSION(TCPC_CORE_VERSION);
MODULE_LICENSE("GPL");
MODULE_SOFTDEP("pre: rt_pd_manager");
