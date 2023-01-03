// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2010-2011, 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2014, Sony Mobile Communications Inc.
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/rtc.h>
#include <linux/timekeeping.h>

#define PON_REV2			0x01

#define PON_SUBTYPE			0x05

#define PON_SUBTYPE_PRIMARY		0x01
#define PON_SUBTYPE_SECONDARY		0x02
#define PON_SUBTYPE_1REG		0x03
#define PON_SUBTYPE_GEN2_PRIMARY	0x04
#define PON_SUBTYPE_GEN2_SECONDARY	0x05
#define PON_SUBTYPE_GEN3_PBS		0x08
#define PON_SUBTYPE_GEN3_HLOS		0x09

#define PON_RT_STS			0x10
#define  PON_KPDPWR_N_SET		BIT(0)
#define  PON_RESIN_N_SET		BIT(1)
#define  PON_GEN3_RESIN_N_SET		BIT(6)
#define  PON_GEN3_KPDPWR_N_SET		BIT(7)

#define PON_PS_HOLD_RST_CTL		0x5a
#define PON_PS_HOLD_RST_CTL2		0x5b
#define  PON_PS_HOLD_ENABLE		BIT(7)
#define  PON_PS_HOLD_TYPE_MASK		0x0f
#define  PON_PS_HOLD_TYPE_SHUTDOWN	4
#define  PON_PS_HOLD_TYPE_HARD_RESET	7

#define PON_PULL_CTL			0x70
#define  PON_KPDPWR_PULL_UP		BIT(1)
#define  PON_RESIN_PULL_UP		BIT(0)

#define PON_DBC_CTL			0x71
#define  PON_DBC_DELAY_MASK		0x7

struct pm8941_data {
	unsigned int	pull_up_bit;
	unsigned int	status_bit;
	bool		supports_ps_hold_poff_config;
	bool		supports_debounce_config;
	bool		needs_sw_debounce;
	bool		has_pon_pbs;
	const char	*name;
	const char	*phys;
};

struct pm8941_pwrkey {
	struct device *dev;
	int irq;
	u32 baseaddr;
	u32 pon_pbs_baseaddr;
	struct regmap *regmap;
	struct input_dev *input;

	unsigned int revision;
	unsigned int subtype;
	struct notifier_block reboot_notifier;

	u32 code;
	u32 sw_debounce_time_us;
	ktime_t last_release_time;
	bool last_status;
	bool log_kpd_event;
	const struct pm8941_data *data;
};

/* ASUS_BSP : long press power key + vol down key 6sec reset device +++ */
static int boot_after_60sec = 0;
static bool is_boot_after_60sec(void);

static int power_key_6s_running = 0;
static int voldown_key_6s_running = 0;
static int power_key_3s_running = 0;
static int voldown_key_3s_running = 0;
static struct work_struct pwr_press_work;
static struct work_struct volDown_press_work;

static unsigned long press_time;
struct timer_list pwr_press_timer;
struct timer_list voldown_press_timer;
/* ASUS_BSP : long press power key + vol down key 6sec reset device --- */

static bool gresin_irq_enable = false;

bool volume_key_wake_en = 0; /* /sys/module/pm8941_pwrkey/parameters/volume_key_wake_en, default is N */
module_param(volume_key_wake_en, bool, 0644);
MODULE_PARM_DESC(volume_key_wake_en, "Enable/Disable volume key wakeup");

int vol_up_irq = 0;
module_param(vol_up_irq, int, 0644);
MODULE_PARM_DESC(vol_up_irq, "vol_up_irq");

static int pm8941_reboot_notify(struct notifier_block *nb,
				unsigned long code, void *unused)
{
	struct pm8941_pwrkey *pwrkey = container_of(nb, struct pm8941_pwrkey,
						    reboot_notifier);
	unsigned int enable_reg;
	unsigned int reset_type;
	int error;

	/* PMICs with revision 0 have the enable bit in same register as ctrl */
	if (pwrkey->revision == 0)
		enable_reg = PON_PS_HOLD_RST_CTL;
	else
		enable_reg = PON_PS_HOLD_RST_CTL2;

	error = regmap_update_bits(pwrkey->regmap,
				   pwrkey->baseaddr + enable_reg,
				   PON_PS_HOLD_ENABLE,
				   0);
	if (error)
		dev_err(pwrkey->dev,
			"unable to clear ps hold reset enable: %d\n",
			error);

	/*
	 * Updates of PON_PS_HOLD_ENABLE requires 3 sleep cycles between
	 * writes.
	 */
	usleep_range(100, 1000);

	switch (code) {
	case SYS_HALT:
	case SYS_POWER_OFF:
		reset_type = PON_PS_HOLD_TYPE_SHUTDOWN;
		break;
	case SYS_RESTART:
	default:
		reset_type = PON_PS_HOLD_TYPE_HARD_RESET;
		break;
	}

	error = regmap_update_bits(pwrkey->regmap,
				   pwrkey->baseaddr + PON_PS_HOLD_RST_CTL,
				   PON_PS_HOLD_TYPE_MASK,
				   reset_type);
	if (error)
		dev_err(pwrkey->dev, "unable to set ps hold reset type: %d\n",
			error);

	error = regmap_update_bits(pwrkey->regmap,
				   pwrkey->baseaddr + enable_reg,
				   PON_PS_HOLD_ENABLE,
				   PON_PS_HOLD_ENABLE);
	if (error)
		dev_err(pwrkey->dev, "unable to re-set enable: %d\n", error);

	return NOTIFY_DONE;
}

//ASUS_bsp add module for powerkey+++++++
#if (defined(ASUS_FTM_BUILD) && (defined(ASUS_AI2202_PROJECT)))
static int pwrkey_mode = 0;
static int pwrkeyMode_function(const char *val,const struct kernel_param *kp)
{
    int ret = 0;
    int old_val = pwrkey_mode;
    if(ret)
     return ret;
    if(pwrkey_mode > 0xf) {
        pwrkey_mode = old_val;
        return -EINVAL;
    }
    ret= param_set_int(val,kp);
    if(pwrkey_mode == 0){
        printk("[mid_powerbtn] Normal_Mode!\n");
    }else if(pwrkey_mode==1){
        printk("[mid_powerbtn] Debug_Mode! \n");
    }
    printk("[mid_powerbtn]pwrkeyMode_function pwrkey_mode =  %d\n",pwrkey_mode);
    return 0;
}

module_param_call(pwrkey_mode,pwrkeyMode_function,param_get_int,&pwrkey_mode,0644);
#endif
//ASUS_BSP add module for powerkey-----------

static irqreturn_t pm8941_pwrkey_irq(int irq, void *_data)
{
	struct pm8941_pwrkey *pwrkey = _data;
	unsigned int sts;
	int error;
	u64 elapsed_us;

	if (pwrkey->sw_debounce_time_us) {
		elapsed_us = ktime_us_delta(ktime_get(),
					    pwrkey->last_release_time);
		if (elapsed_us < pwrkey->sw_debounce_time_us) {
			dev_dbg(pwrkey->dev, "ignoring key event received after %llu us, debounce time=%u us\n",
				elapsed_us, pwrkey->sw_debounce_time_us);
			return IRQ_HANDLED;
		}
	}

	error = regmap_read(pwrkey->regmap,
			    pwrkey->baseaddr + PON_RT_STS, &sts);
	if (error)
		return IRQ_HANDLED;

	sts &= pwrkey->data->status_bit;

	if (!boot_after_60sec) {
		is_boot_after_60sec();
	}

	if (boot_after_60sec) {
		if (pwrkey->code == 116 && sts) {
			press_time = jiffies;
			mod_timer(&pwr_press_timer, jiffies + msecs_to_jiffies(3000));
		}
		if (pwrkey->code == 116 && !sts) {
			power_key_6s_running = 0;
			power_key_3s_running = 0;
			del_timer(&pwr_press_timer);
			press_time = 0xFFFFFFFF;
		}
		if (pwrkey->code == 114) {
			if (sts) {
				mod_timer(&voldown_press_timer, jiffies + msecs_to_jiffies(3000));
			}
			else {
				voldown_key_6s_running = 0;
				voldown_key_3s_running = 0;
				del_timer(&voldown_press_timer);
			}
		}
	}

	if (pwrkey->sw_debounce_time_us && !sts)
		pwrkey->last_release_time = ktime_get();

	if (pwrkey->log_kpd_event)
		pr_info_ratelimited("PMIC input: KPDPWR status=0x%02x, KPDPWR_ON=%d\n",
			sts, (sts & PON_KPDPWR_N_SET));

	/*
	 * Simulate a press event in case a release event occurred without a
	 * corresponding press event.
	 */
	if (!pwrkey->last_status && !sts) {
		input_report_key(pwrkey->input, pwrkey->code, 1);
		input_sync(pwrkey->input);
	}
	pwrkey->last_status = sts;

// ASUS_BSP +++ keypad debug log
    printk("[keypad][pm8941-pwrkey.c] keycode=%d, state=%s\n", pwrkey->code, sts?"press":"release");
// ASUS_BSP --- keypad debug log

// ASUS_BSP +++ keypad add wakeup for COS
    pm_wakeup_dev_event(pwrkey->dev, 1000, true);
// ASUS_BSP --- keypad add wakeup for COS

#if (defined(ASUS_FTM_BUILD) && (defined(ASUS_AI2202_PROJECT) ))
//<ASUS-BSP>for add module for powerkey+++++++
	if(pwrkey_mode == 1){
		if(pwrkey->code == KEY_POWER){
			printk("[mid_powerbtn]pwrkey mode\n");
			 pwrkey->code = KEY_A;
		}
	}else{
		if(pwrkey->code == KEY_A){
		    pwrkey->code = KEY_POWER;
		}
		printk("[mid_powerbtn]normal mode\n");
	}
    printk("[mid_powerbtn]pwrkey->code = %d, state = %s\n", pwrkey->code, sts?"press":"release");
    printk("[mid_powerbtn]pwrkey_mode = %d\n", pwrkey_mode);
#endif
//<ASUS-BSP>for add module for powerkey------

	input_report_key(pwrkey->input, pwrkey->code, sts);
	input_sync(pwrkey->input);

	return IRQ_HANDLED;
}

static int pm8941_pwrkey_sw_debounce_init(struct pm8941_pwrkey *pwrkey)
{
	unsigned int val, addr;
	int error;

	if (pwrkey->data->has_pon_pbs && !pwrkey->pon_pbs_baseaddr) {
		dev_err(pwrkey->dev, "PON_PBS address missing, can't read HW debounce time\n");
		return 0;
	}

	if (pwrkey->pon_pbs_baseaddr)
		addr = pwrkey->pon_pbs_baseaddr + PON_DBC_CTL;
	else
		addr = pwrkey->baseaddr + PON_DBC_CTL;
	error = regmap_read(pwrkey->regmap, addr, &val);
	if (error)
		return error;

	if (pwrkey->subtype >= PON_SUBTYPE_GEN2_PRIMARY)
		pwrkey->sw_debounce_time_us = 2 * USEC_PER_SEC /
						(1 << (0xf - (val & 0xf)));
	else
		pwrkey->sw_debounce_time_us = 2 * USEC_PER_SEC /
						(1 << (0x7 - (val & 0x7)));

	dev_dbg(pwrkey->dev, "SW debounce time = %u us\n",
		pwrkey->sw_debounce_time_us);

	return 0;
}

static int __maybe_unused pm8941_pwrkey_suspend(struct device *dev)
{
	struct pm8941_pwrkey *pwrkey = dev_get_drvdata(dev);

	if (device_may_wakeup(dev)) {
		if (!strcmp (pwrkey->data->name, "pmic_resin")) {
			if(volume_key_wake_en) {
				gresin_irq_enable = true;
				enable_irq_wake(pwrkey->irq);
				enable_irq_wake(vol_up_irq);
			}
		} else {
			enable_irq_wake(pwrkey->irq);
		}
	}
	return 0;
}

static int __maybe_unused pm8941_pwrkey_resume(struct device *dev)
{
	struct pm8941_pwrkey *pwrkey = dev_get_drvdata(dev);

	if (device_may_wakeup(dev)) {
		if (!strcmp (pwrkey->data->name, "pmic_resin")) {
			if(gresin_irq_enable) {
				gresin_irq_enable = false;
				disable_irq_wake(pwrkey->irq);
				disable_irq_wake(vol_up_irq);
			}
		} else {
			disable_irq_wake(pwrkey->irq);
		}
	}
	return 0;
}

static SIMPLE_DEV_PM_OPS(pm8941_pwr_key_pm_ops,
			 pm8941_pwrkey_suspend, pm8941_pwrkey_resume);

#if (defined(ASUS_FTM_BUILD) && (defined(ASUS_AI2202_PROJECT)))
static ssize_t printklog_write (struct file *filp, const char *userbuf, size_t size, loff_t *loff_p)
{
	char str[128];
	memset(str, 0, sizeof(str));

	if(size > 127)
		size = 127;

	if (copy_from_user(str, userbuf, size))
	{
		pr_err("copy from bus failed!\n");
		return -EFAULT;
	}

	printk(KERN_ERR"[factool log]:%s",str);
	return size;
}

static struct proc_ops printklog_fops = {
	.proc_write = printklog_write,
};
#endif

/* ASUS_BSP : long press power key + vol down key 6sec reset device +++ */
static bool is_boot_after_60sec(void)
{
	struct rtc_time tm;
	struct timespec64 ts;
	ktime_get_real_ts64(&ts);
	ts.tv_sec -= sys_tz.tz_minuteswest * 60;
	rtc_time64_to_tm(ts.tv_sec, &tm);
	ktime_get_raw_ts64(&ts);

	if (boot_after_60sec == 0 && ts.tv_sec >= 60) {
		boot_after_60sec = 1;
	}

	return boot_after_60sec;
}

void pwr_press_timer_callback(struct timer_list *t)
{
	schedule_work(&pwr_press_work);
}

void pwr_press_workqueue(struct work_struct *work)
{
	if(power_key_3s_running  == 0){
		power_key_3s_running = 1;
		mod_timer(&pwr_press_timer, jiffies + msecs_to_jiffies(4000));
	}else {
		power_key_6s_running= 1;
		if (voldown_key_6s_running ) {
				pr_info("ASDF: reset device after power press 8 sec \n");
				pr_info("[Reboot] Power key long press 8 sec\n");
				msleep(200);
				printk("force reset device!!\n");
				kernel_restart("asus_force_reboot");
		}
	}
}

void volDown_press_timer_callback(struct timer_list *t)
{
	schedule_work(&volDown_press_work);
}

void volDown_press_workqueue(struct work_struct *work)
{
	if(voldown_key_3s_running  == 0){
		voldown_key_3s_running = 1;
		mod_timer(&voldown_press_timer, jiffies + msecs_to_jiffies(4000));
	}else {
		voldown_key_6s_running = 1;
		if (power_key_6s_running ) {
				pr_info("ASDF: reset device after power press 8 sec \n");
				pr_info("[Reboot] Power key long press 8 sec\n");
				msleep(200);
				printk("force reset device!!\n");
				kernel_restart("asus_force_reboot");
		}
	}
}
/* ASUS_BSP : long press power key + vol down key 6sec reset device ---*/

static int pm8941_pwrkey_probe(struct platform_device *pdev)
{
	struct pm8941_pwrkey *pwrkey;
	bool pull_up;
	struct device *parent;
	struct device_node *regmap_node;
	const __be32 *addr;
	u32 req_delay;
	unsigned int sts;
	int error;

	if (of_property_read_u32(pdev->dev.of_node, "debounce", &req_delay))
		req_delay = 15625;

	if (req_delay > 2000000 || req_delay == 0) {
		dev_err(&pdev->dev, "invalid debounce time: %u\n", req_delay);
		return -EINVAL;
	}

	pull_up = of_property_read_bool(pdev->dev.of_node, "bias-pull-up");

	pwrkey = devm_kzalloc(&pdev->dev, sizeof(*pwrkey), GFP_KERNEL);
	if (!pwrkey)
		return -ENOMEM;

	pwrkey->dev = &pdev->dev;
	pwrkey->data = of_device_get_match_data(&pdev->dev);
	if (!pwrkey->data) {
		dev_err(&pdev->dev, "match data not found\n");
		return -ENODEV;
	}

	parent = pdev->dev.parent;
	regmap_node = pdev->dev.of_node;
	pwrkey->regmap = dev_get_regmap(parent, NULL);
	if (!pwrkey->regmap) {
		regmap_node = parent->of_node;
		/*
		 * We failed to get regmap for parent. Let's see if we are
		 * a child of pon node and read regmap and reg from its
		 * parent.
		 */
		pwrkey->regmap = dev_get_regmap(parent->parent, NULL);
		if (!pwrkey->regmap) {
			dev_err(&pdev->dev, "failed to locate regmap\n");
			return -ENODEV;
		}
	}

	addr = of_get_address(regmap_node, 0, NULL, NULL);
	if (!addr) {
		dev_err(&pdev->dev, "reg property missing\n");
		return -EINVAL;
	}
	pwrkey->baseaddr = be32_to_cpu(*addr);

	if (pwrkey->data->has_pon_pbs) {
		/* PON_PBS base address is optional */
		addr = of_get_address(regmap_node, 1, NULL, NULL);
		if (addr)
			pwrkey->pon_pbs_baseaddr = be32_to_cpu(*addr);
	}

	pwrkey->irq = platform_get_irq(pdev, 0);
	if (pwrkey->irq < 0)
		return pwrkey->irq;

	error = regmap_read(pwrkey->regmap, pwrkey->baseaddr + PON_REV2,
			    &pwrkey->revision);
	if (error) {
		dev_err(&pdev->dev, "failed to read revision: %d\n", error);
		return error;
	}

	error = regmap_read(pwrkey->regmap, pwrkey->baseaddr + PON_SUBTYPE,
			    &pwrkey->subtype);
	if (error) {
		dev_err(&pdev->dev, "failed to read subtype: %d\n", error);
		return error;
	}

	error = of_property_read_u32(pdev->dev.of_node, "linux,code",
				     &pwrkey->code);
	if (error) {
		dev_dbg(&pdev->dev,
			"no linux,code assuming power (%d)\n", error);
		pwrkey->code = KEY_POWER;
	}

	pwrkey->input = devm_input_allocate_device(&pdev->dev);
	if (!pwrkey->input) {
		dev_dbg(&pdev->dev, "unable to allocate input device\n");
		return -ENOMEM;
	}

	input_set_capability(pwrkey->input, EV_KEY, pwrkey->code);

#if (defined(ASUS_FTM_BUILD) && (defined(ASUS_AI2202_PROJECT)))
    input_set_capability(pwrkey->input, EV_KEY, KEY_A);
#endif

	pwrkey->input->name = pwrkey->data->name;
	pwrkey->input->phys = pwrkey->data->phys;

	if (pwrkey->data->supports_debounce_config) {
		req_delay = (req_delay << 6) / USEC_PER_SEC;
		req_delay = ilog2(req_delay);

		error = regmap_update_bits(pwrkey->regmap,
					   pwrkey->baseaddr + PON_DBC_CTL,
					   PON_DBC_DELAY_MASK,
					   req_delay);
		if (error) {
			dev_err(&pdev->dev, "failed to set debounce: %d\n",
				error);
			return error;
		}
	}

	if (pwrkey->data->needs_sw_debounce) {
		error = pm8941_pwrkey_sw_debounce_init(pwrkey);
		if (error)
			return error;
	}

	if (pwrkey->data->pull_up_bit) {
		error = regmap_update_bits(pwrkey->regmap,
					   pwrkey->baseaddr + PON_PULL_CTL,
					   pwrkey->data->pull_up_bit,
					   pull_up ? pwrkey->data->pull_up_bit :
						     0);
		if (error) {
			dev_err(&pdev->dev, "failed to set pull: %d\n", error);
			return error;
		}
	}

	pwrkey->log_kpd_event = of_property_read_bool(pdev->dev.of_node, "qcom,log-kpd-event");

	if (pwrkey->log_kpd_event) {
		error = regmap_read(pwrkey->regmap,
				    pwrkey->baseaddr + PON_RT_STS, &sts);
		if (error)
			dev_err(&pdev->dev, "failed to read PON_RT_STS rc=%d\n", error);
		else
			pr_info("KPDPWR status at init=0x%02x, KPDPWR_ON=%d\n",
				sts, (sts & PON_KPDPWR_N_SET));
	}

	error = devm_request_threaded_irq(&pdev->dev, pwrkey->irq,
					  NULL, pm8941_pwrkey_irq,
					  IRQF_ONESHOT,
					  pwrkey->data->name, pwrkey);
	if (error) {
		dev_err(&pdev->dev, "failed requesting IRQ: %d\n", error);
		return error;
	}

	error = input_register_device(pwrkey->input);
	if (error) {
		dev_err(&pdev->dev, "failed to register input device: %d\n",
			error);
		return error;
	}

	if (pwrkey->data->supports_ps_hold_poff_config) {
		pwrkey->reboot_notifier.notifier_call = pm8941_reboot_notify,
		error = register_reboot_notifier(&pwrkey->reboot_notifier);
		if (error) {
			dev_err(&pdev->dev, "failed to register reboot notifier: %d\n",
				error);
			return error;
		}
	}

	platform_set_drvdata(pdev, pwrkey);
	device_init_wakeup(&pdev->dev, 1);

#if (defined(ASUS_FTM_BUILD) && (defined(ASUS_AI2202_PROJECT)))
	if(proc_create("fac_printklog", 0222, NULL, &printklog_fops) == NULL)
	{
		printk(KERN_ERR"create printklog node is error\n");
	}
#endif

	/* ASUS_BSP : long press power key + vol down key 6sec reset device +++ */
	timer_setup(&pwr_press_timer, pwr_press_timer_callback, 0);
	timer_setup(&voldown_press_timer, volDown_press_timer_callback, 0);
	INIT_WORK(&pwr_press_work, pwr_press_workqueue);
	INIT_WORK(&volDown_press_work, volDown_press_workqueue);
	/* ASUS_BSP : long press power key + vol down key 6sec reset device ---*/

	return 0;
}

static int pm8941_pwrkey_remove(struct platform_device *pdev)
{
	struct pm8941_pwrkey *pwrkey = platform_get_drvdata(pdev);

	if (pwrkey->data->supports_ps_hold_poff_config)
		unregister_reboot_notifier(&pwrkey->reboot_notifier);

	return 0;
}

static const struct pm8941_data pwrkey_data = {
	.pull_up_bit = PON_KPDPWR_PULL_UP,
	.status_bit = PON_KPDPWR_N_SET,
	.name = "pm8941_pwrkey",
	.phys = "pm8941_pwrkey/input0",
	.supports_ps_hold_poff_config = true,
	.supports_debounce_config = true,
	.needs_sw_debounce = true,
	.has_pon_pbs = false,
};

static const struct pm8941_data resin_data = {
	.pull_up_bit = PON_RESIN_PULL_UP,
	.status_bit = PON_RESIN_N_SET,
	.name = "pm8941_resin",
	.phys = "pm8941_resin/input0",
	.supports_ps_hold_poff_config = true,
	.supports_debounce_config = true,
	.needs_sw_debounce = true,
	.has_pon_pbs = false,
};

static const struct pm8941_data pon_gen3_pwrkey_data = {
	.status_bit = PON_GEN3_KPDPWR_N_SET,
	.name = "pmic_pwrkey",
	.phys = "pmic_pwrkey/input0",
	.supports_ps_hold_poff_config = false,
	.supports_debounce_config = false,
	.needs_sw_debounce = true,
	.has_pon_pbs = true,
};

static const struct pm8941_data pon_gen3_resin_data = {
	.status_bit = PON_GEN3_RESIN_N_SET,
	.name = "pmic_resin",
	.phys = "pmic_resin/input0",
	.supports_ps_hold_poff_config = false,
	.supports_debounce_config = false,
	.needs_sw_debounce = true,
	.has_pon_pbs = true,
};

static const struct of_device_id pm8941_pwr_key_id_table[] = {
	{ .compatible = "qcom,pm8941-pwrkey", .data = &pwrkey_data },
	{ .compatible = "qcom,pm8941-resin", .data = &resin_data },
	{ .compatible = "qcom,pmk8350-pwrkey", .data = &pon_gen3_pwrkey_data },
	{ .compatible = "qcom,pmk8350-resin", .data = &pon_gen3_resin_data },
	{ }
};
MODULE_DEVICE_TABLE(of, pm8941_pwr_key_id_table);

static struct platform_driver pm8941_pwrkey_driver = {
	.probe = pm8941_pwrkey_probe,
	.remove = pm8941_pwrkey_remove,
	.driver = {
		.name = "pm8941-pwrkey",
		.pm = &pm8941_pwr_key_pm_ops,
		.of_match_table = of_match_ptr(pm8941_pwr_key_id_table),
	},
};
module_platform_driver(pm8941_pwrkey_driver);

MODULE_DESCRIPTION("PM8941 Power Key driver");
MODULE_LICENSE("GPL v2");
