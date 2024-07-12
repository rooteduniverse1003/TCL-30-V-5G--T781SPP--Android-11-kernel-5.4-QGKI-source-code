/* /kernel-4.9/drivers/power/supply/mediatek/charger/pogo_driver_v2.c
 *
 * Copyright  (C)  2010 - 2020 TCL., Ltd.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 *  History 1.0 : created by dapeng.qiao for task 9732872 on 2020-08-03
 */

#define pr_fmt(fmt) "[POGO]: " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/printk.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/kernel.h>
#include "pogo_davinci.h"

enum print_reason {
    PR_INTERRUPT    = BIT(0),
    PR_MISC            = BIT(1),
    PR_GPIO            = BIT(2),
};

#define pogo_err(info, fmt, ...)        \
    pr_err_ratelimited("%s: " fmt, \
        __func__, ##__VA_ARGS__)
#define pogo_dbg(info, reason, fmt, ...)            \
    do {                            \
        if (*info->debug_mask & (reason))        \
            pr_err_ratelimited("%s: " fmt, \
                __func__, ##__VA_ARGS__);    \
        else                        \
            pr_debug_ratelimited("%s: " fmt, \
                __func__, ##__VA_ARGS__);    \
    } while (0)

#define CHANGE_GPIO_DELAY_MS    50
#define TCPC_OTG_DEV_NAME        "type_c_port0"
#define REGISTER_OTG_WORK_DELAY    200


static int pogo_chk_cnt;    //added by dapeng.qiao for task 10157989 on 2020-09-30
bool flag_probe_finished;
static int __debug_mask = 0xff;
static struct pogo_info *pinfo;
static const struct of_device_id pogo_of_match[] = {
    {.compatible = "qcom, pogo_v2"},
    {},
};

enum smb_irq_index {
    POGO_ID_IRQ = 0,
    VBUS_CON_DET_IRQ,
    VDOCK_IN_DET_IRQ,
};

char * evn_str[2] ={"kbd=0", "kbd=1"};

void send_pogo_uevent(struct pogo_info *info)
{
    char *envp[2];

    if(info->keyboard_present)
        envp[0] = evn_str[1];
    else
        envp[0] = evn_str[0];

    envp[1] = NULL;

    kobject_uevent_env(&info->dev->kobj, KOBJ_CHANGE, envp);

    pr_err("send pogo_keyboard uevent %d, kobj %s\n", info->keyboard_present, info->dev->kobj.name);
}

void set_current_limit_for_pogo_charger(unsigned int force_cnt)
{
	//struct charger_data *pdata;
	//struct charger_manager *cm_info = pinfo->pbat_consumer->cm;
	//struct switch_charging_alg_data *swchgalg = cm_info->algorithm_data;
	int charge_status = 0;
	int last_charge_status = -1;
	int vdock_in_det_status = -1;
	int vbus_con_det_status = -1;
	int s_pogo_Vdock_Set_Vbus_flag = -1;

	charge_status = pinfo->current_pogo_fsm;
	//pr_err("pogo [%s] 1 charge_status =%x\n", __func__, charge_status);
	charge_status =  (POGO_DOCK_CHARGER | POGO_USB_CHARGER);
	//pr_err("pogo [%s] 2 charge_status =%x\n", __func__, charge_status);
	charge_status = (pinfo->current_pogo_fsm & (POGO_DOCK_CHARGER | POGO_USB_CHARGER));
	//pr_err("pogo [%s] 3 charge_status =%x\n", __func__, charge_status);
	//pdata = &cm_info->chg1_data;
	if(force_cnt){
		last_charge_status = -1;
	}
	if(pogo_chk_cnt){
		pogo_chk_cnt--;
		last_charge_status = -1;
	}
	//mutex_lock(&swchgalg->ichg_aicr_access_mutex);
	/*pr_err("pogo [%s] charge_status=%x,last_charge_status=%x,force_cnt =%d\n", __func__,\
		 charge_status, last_charge_status, force_cnt);*/
	if(last_charge_status != charge_status)
	{
		last_charge_status = charge_status;
		//need check vdock
		vdock_in_det_status = gpio_get_value(pinfo->vdock_in_det_gpio);
		vbus_con_det_status = gpio_get_value(pinfo->vbus_con_det_gpio);
		/*pr_err("pogo [%s] vdock_in_det_status =%d,vbus_con_det_status =%d\n", __func__, \
			vdock_in_det_status, vbus_con_det_status);*/
		if (pinfo->keyboard_present) {
			//pr_err("pogo [%s] keyboard present,ignore dock vbus\n", __func__);
			pinfo->dock_present = false;
			pinfo->current_pogo_fsm &= ~POGO_DOCK_CHARGER;
		} else {
			if (!vdock_in_det_status) {
				pinfo->dock_present = true;
				pinfo->current_pogo_fsm |= POGO_DOCK_CHARGER;
				s_pogo_Vdock_Set_Vbus_flag = 1;
			} else {
				pinfo->dock_present = false;
				pinfo->current_pogo_fsm &= ~POGO_DOCK_CHARGER;
			}
		}
		//also need check vbus
		if(!s_pogo_Vdock_Set_Vbus_flag){
			if (pinfo->otg_present) {
				pinfo->usb_present = false;
				pinfo->current_pogo_fsm &= ~POGO_USB_CHARGER;
			} else {
				if (!vbus_con_det_status) {
					pinfo->usb_present = true;
					pinfo->current_pogo_fsm |= POGO_USB_CHARGER;
				} else {
					pinfo->usb_present = false;
					pinfo->current_pogo_fsm &= ~POGO_USB_CHARGER;
				}
			}
		}
		if(pinfo->current_pogo_fsm & POGO_DOCK_CHARGER){
			//cm_info->chk_pogo_chg = 2;
			pinfo->chk_pogo_chg = 2; // weijun++
		}
		if(pinfo->current_pogo_fsm & POGO_USB_CHARGER){
			//cm_info->chk_pogo_chg = 1;
			pinfo->chk_pogo_chg = 1;  //weijun++
		}
	}
	else
	{
		//cm_info->chk_pogo_chg = 0;
		pinfo->chk_pogo_chg = 0;  //weijun++
	}
	//mutex_unlock(&swchgalg->ichg_aicr_access_mutex);

    //pr_err("pogo [%s] cm_info->chk_pogo_chg\n", __func__);
}

int get_dock_pin_status(void)
{
	return pinfo->current_pogo_fsm;
}

/*
*
*sbm5 enable OTG detect
*
*/
void pogo_set_otg_detect(bool enable)
{
	int rc =0;
	pr_err("%s: enable otg=%d\n", __func__, enable);
	if (enable) {
		// dock_en pin
		rc = pinctrl_select_state(pinfo->pinctrl, pinfo->pinctrl_dock_en_high);
		if (rc < 0)
			pr_err("Couldn't select state DPI_CK_en_high rc=%d\n", rc);
		// otg_in_en pin
		#if 0 // enable otg output later
		rc = pinctrl_select_state(pinfo->pinctrl, pinfo->pinctrl_otg_en_high);
		if (rc < 0)
			pr_err("Couldn't select state pinctrl_otg_en_high rc=%d\n", rc);
		#endif
	} else {
		if (!(pinfo->current_pogo_fsm & POGO_DOCK_KEYBOARD)) {
			// dock_en pin
			rc = pinctrl_select_state(pinfo->pinctrl, pinfo->pinctrl_dock_en_low);
			if (rc < 0)
				pr_err("Couldn't select state DPI_CK_en_high rc=%d\n", rc);
		}
		// otg_in_en pin
		rc = pinctrl_select_state(pinfo->pinctrl, pinfo->pinctrl_otg_en_low);
		if (rc < 0)
			pr_err("Couldn't select state pinctrl_otg_en_low rc=%d\n", rc);
	}
}

/*
*
*sbm5 notify if OTG attach/detach
*
*/
void pogo_update_otg_status(bool attached)
{
	int rc =0;
	pr_err("%s: otg attached=%d\n", __func__, attached);
	if (attached) {
		pinfo->current_pogo_fsm |= POGO_OTG_DEVICE;
		pinfo->otg_present = true;

		// dock_en pin
		rc = pinctrl_select_state(pinfo->pinctrl, pinfo->pinctrl_dock_en_high);
		if (rc < 0)
			pr_err("Couldn't select state DPI_CK_en_high rc=%d\n", rc);
		// otg_in_en pin
		rc = pinctrl_select_state(pinfo->pinctrl, pinfo->pinctrl_otg_en_high);
		if (rc < 0)
			pr_err("Couldn't select state pinctrl_otg_en_high rc=%d\n", rc);
	} else {
		pinfo->current_pogo_fsm &= ~POGO_OTG_DEVICE;
		pinfo->otg_present = false;

		if (!(pinfo->current_pogo_fsm & POGO_DOCK_KEYBOARD)) {
			// dock_en pin
			rc = pinctrl_select_state(pinfo->pinctrl, pinfo->pinctrl_dock_en_low);
			if (rc < 0)
				pr_err("Couldn't select state DPI_CK_en_high rc=%d\n", rc);
		}
		// otg_in_en pin
		rc = pinctrl_select_state(pinfo->pinctrl, pinfo->pinctrl_otg_en_low);
		if (rc < 0)
			pr_err("Couldn't select state pinctrl_otg_en_low rc=%d\n", rc);
	}
}

static void pogo_id_work(struct work_struct *work)
{
    struct pogo_info *info = container_of(work, struct pogo_info, keyboard_detect_work.work);
    int pogo_id_state = info->keyboard_present;
    int i;

    for(i=0; i<10; i++)
    {
        if(pogo_id_state != (!gpio_get_value(info->pogo_id_gpio)))
            return;
        msleep(2);
    }

    send_pogo_uevent(info);
}

irqreturn_t pogo_id_irq_handler(int irq, void *data) {
    struct pogo_irq_data *irq_data = data;
    struct pogo_info *info = irq_data->parent_data;
    unsigned long flags = 0;
    int rc = 0;

    if(!flag_probe_finished)
        return IRQ_NONE;
    spin_lock_irqsave(&info->slock, flags);
    info->keyboard_present = !gpio_get_value(info->pogo_id_gpio);
    pr_err("keyboard status is %d\n", !info->keyboard_present);
    if (info->keyboard_present) {
        info->current_pogo_fsm |= POGO_KEYBOARD;
        rc = pinctrl_select_state(info->pinctrl, info->pinctrl_dock_en_high);
        if (rc < 0)
            pr_err("Couldn't select state pinctrl_dock_en_high rc=%d\n", rc);
        rc = pinctrl_select_state(info->pinctrl, info->pinctrl_vpower_out_en_high);
        if (rc < 0)
            pr_err("Couldn't select state vpower_out_en_high rc=%d\n", rc);
    } else {
        info->current_pogo_fsm &= ~POGO_KEYBOARD;
        rc = pinctrl_select_state(info->pinctrl, info->pinctrl_vpower_out_en_low);
        if (rc < 0)
            pr_err("Couldn't select state vpower_out_en_low rc=%d\n", rc);
        rc = pinctrl_select_state(info->pinctrl, info->pinctrl_dock_en_low);
        if (rc < 0)
            pr_err("Couldn't select state pinctrl_dock_en_low rc=%d\n", rc);
    }

    pr_err("IRQ: %s, fsm=%s\n", irq_data->name, pogo_fsm_name[info->current_pogo_fsm]);
    spin_unlock_irqrestore(&info->slock, flags);

    cancel_delayed_work(&info->keyboard_detect_work);
    schedule_delayed_work(&info->keyboard_detect_work, msecs_to_jiffies(50));

    return IRQ_HANDLED;
}

/*
*
* cut off vbus for 500ms if dock charger is on.
*
*/
static void pogo_fchg_detect_work(struct work_struct *work)
{
    struct pogo_info *info = container_of(work, struct pogo_info, fchg_detect_work.work);
	int rc = 0;

	if (info->current_pogo_fsm & POGO_OTG_DEVICE) {
		return;
	}

	// VBUS_CON_EN pin
	rc = pinctrl_select_state(info->pinctrl, info->pinctrl_vbus_con_en_high);
	if (rc < 0)
		pr_err("Couldn't select state pinctrl_vbus_con_en_low rc=%d\n", rc);

	msleep(500);

	// VBUS_CON_EN pin
	rc = pinctrl_select_state(info->pinctrl, info->pinctrl_vbus_con_en_low);
	if (rc < 0)
		pr_err("Couldn't select state pinctrl_vbus_con_en_low rc=%d\n", rc);

	pogo_dbg(info, PR_GPIO, "fast charging detect work\n");
}

/*
*
* enable dock charge.
*
*/
static void dock_chg_work(struct work_struct *work)
{
    struct pogo_info *info = container_of(work, struct pogo_info, dock_chg_work.work);
	int rc = 0;

	if (info->current_pogo_fsm & POGO_OTG_DEVICE) {
		return;
	}

	// dock_en pin high
	rc = pinctrl_select_state(info->pinctrl, info->pinctrl_dock_en_high);
	if (rc < 0)
		pr_err("Couldn't select state DPI_CK_en_high rc=%d\n", rc);

	msleep(500);

	// dock_en pin low
	rc = pinctrl_select_state(info->pinctrl, info->pinctrl_dock_en_low);
	if (rc < 0)
		pr_err("Couldn't select state DPI_CK_en_high rc=%d\n", rc);

	pogo_dbg(info, PR_GPIO, "dock_chg_work\n");
}


irqreturn_t vbus_con_det_irq_handler(int irq, void *data) {
    struct pogo_irq_data *irq_data = data;
    struct pogo_info *info = irq_data->parent_data;
    unsigned long flags = 0;
    int vbus_con_det_status = 0;

    if(!flag_probe_finished)
        return IRQ_NONE;
    spin_lock_irqsave(&info->slock, flags);
    vbus_con_det_status = gpio_get_value(info->vbus_con_det_gpio);
    pogo_dbg(info, PR_GPIO, "usb status is %d\n", vbus_con_det_status);
    if (info->otg_present) {
        pogo_dbg(info, PR_MISC, "otg present,ignore usb vbus\n");
        info->usb_present = false;
        info->current_pogo_fsm &= ~POGO_USB_CHARGER;
    } else {
        if (!vbus_con_det_status) {
            info->usb_present = true;
            info->current_pogo_fsm |= POGO_USB_CHARGER;
        } else {
            info->usb_present = false;
            info->current_pogo_fsm &= ~POGO_USB_CHARGER;

            if (info->current_pogo_fsm & POGO_DOCK_CHARGER)
                 schedule_delayed_work(&info->dock_chg_work, 0);
        }
    }

	if ((info->current_pogo_fsm & POGO_DOCK_CHARGER)
			&& (info->current_pogo_fsm & POGO_USB_CHARGER)
			&& !(info->current_pogo_fsm & POGO_OTG_DEVICE)) {
		pogo_dbg(info, PR_GPIO, "schedule detect work\n");
		cancel_delayed_work(&info->fchg_detect_work);
	    schedule_delayed_work(&info->fchg_detect_work, 0);
	}

    pr_err("IRQ: %s, fsm=%s\n", irq_data->name, pogo_fsm_name[info->current_pogo_fsm]);
    spin_unlock_irqrestore(&info->slock, flags);
    return IRQ_HANDLED;
}

irqreturn_t vdock_in_det_irq_handler(int irq, void *data) {
    struct pogo_irq_data *irq_data = data;
    struct pogo_info *info = irq_data->parent_data;
    unsigned long flags = 0;
    int vdock_in_det_status = 0;

    if(!flag_probe_finished)
        return IRQ_NONE;
    spin_lock_irqsave(&info->slock, flags);
    vdock_in_det_status = gpio_get_value(info->vdock_in_det_gpio);
    pr_err("dock status is %d\n", vdock_in_det_status);
    if (info->keyboard_present) {
        pogo_dbg(info, PR_MISC, "keyboard present,ignore dock vbus\n");
        info->dock_present = false;
        info->current_pogo_fsm &= ~POGO_DOCK_CHARGER;

    } else {
        if (!vdock_in_det_status) {
            info->dock_present = true;
            info->current_pogo_fsm |= POGO_DOCK_CHARGER;
        } else {
            info->dock_present = false;
            info->current_pogo_fsm &= ~POGO_DOCK_CHARGER;
        }
    }
    set_current_limit_for_pogo_charger(true);
    pr_err("IRQ: %s, fsm=%s\n", irq_data->name, pogo_fsm_name[info->current_pogo_fsm]);
    spin_unlock_irqrestore(&info->slock, flags);
    return IRQ_HANDLED;
}

static int pogo_determine_initial_status(struct pogo_info *info)
{
    struct pogo_irq_data irq_data = {info, "determine-initial-status"};
	int rc = 0;

	// otg_in_en pin
	rc = pinctrl_select_state(info->pinctrl, info->pinctrl_otg_en_low);
    if (rc < 0)
        pr_err("Couldn't select state pinctrl_otg_en_low rc=%d\n", rc);

	// boost power out
	rc = pinctrl_select_state(info->pinctrl, info->pinctrl_vpower_out_en_low);
    if (rc < 0)
        pr_err("Couldn't select state pinctrl_vpower_out_en_low rc=%d\n", rc);

	// dock_en pin
	rc = pinctrl_select_state(info->pinctrl, info->pinctrl_dock_en_low);
    if (rc < 0)
        pr_err("Couldn't select state pinctrl_dock_en_low rc=%d\n", rc);

	// VBUS_CON_EN pin
	rc = pinctrl_select_state(info->pinctrl, info->pinctrl_vbus_con_en_low);
	if (rc < 0)
		pr_err("Couldn't select state pinctrl_vbus_con_en_low rc=%d\n", rc);

	//interrupt gpio init
	rc = pinctrl_select_state(info->pinctrl, info->vbus_con_irq_active);
    if (rc < 0)
        pr_err("Couldn't select state vbus_con_irq_active rc=%d\n", rc);

	rc = pinctrl_select_state(info->pinctrl, info->id_irq_active);
    if (rc < 0)
        pr_err("Couldn't select state id_irq_active rc=%d\n", rc);

	rc = pinctrl_select_state(info->pinctrl, info->vdock_in_irq_active);
    if (rc < 0)
        pr_err("Couldn't select state vdock_in_irq_active rc=%d\n", rc);



    if (!info->kpoc)
        pogo_id_irq_handler(0, &irq_data);
    vbus_con_det_irq_handler(0, &irq_data);
    vdock_in_det_irq_handler(0, &irq_data);

    return 0;
}

static struct pogo_irq_info pogo_irqs[] = {
    [POGO_ID_IRQ] = {
        .name        = "pogo-id",
        .handler    = pogo_id_irq_handler,
        .wake        = true,
    },
    [VBUS_CON_DET_IRQ] = {
        .name        = "vbus-con-det",
        .handler    = vbus_con_det_irq_handler,
        .wake        = true,
    },
    [VDOCK_IN_DET_IRQ] = {
        .name        = "vdock-in-det",
        .handler    = vdock_in_det_irq_handler,
        .wake        = true,
    },
};

static int pogo_get_irq_index_byname(const char *irq_name) {
    int i;

    for (i = 0; i < ARRAY_SIZE(pogo_irqs); i++) {
        if (strcmp(pogo_irqs[i].name, irq_name) == 0)
            return i;
    }

    return -ENOENT;
}

static int pogo_request_interrupt(struct pogo_info *info,
                struct device_node *node, const char *irq_name) {
    int rc, irq, irq_index;
    struct pogo_irq_data *irq_data;

    irq_index = pogo_get_irq_index_byname(irq_name);
    if (irq_index < 0) {
        pr_err("%s is not a defined irq\n", irq_name);
        return irq_index;
    }

    if (!pogo_irqs[irq_index].handler)
        return 0;

    if((info->kpoc) && (irq_index == POGO_ID_IRQ)) {
        pr_err("kpoc ignore keyboard interrupts\n");
        return 0;
    }

    irq_data = devm_kzalloc(info->dev, sizeof(*irq_data), GFP_KERNEL);
    if (!irq_data)
        return -ENOMEM;

    irq_data->parent_data = info;
    irq_data->name = irq_name;
	irq = pogo_irqs[irq_index].irq;

    rc = devm_request_threaded_irq(info->dev, irq, NULL,
                    pogo_irqs[irq_index].handler,
                    IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
                    irq_name, irq_data);
    if (rc < 0) {
        pr_err("Couldn't request irq %d\n", irq);
        return rc;
    }

    pogo_irqs[irq_index].enabled = true;
    //pogo_irqs[irq_index].irq = irq;
    pogo_irqs[irq_index].irq_data = irq_data;
    if (pogo_irqs[irq_index].wake)
        enable_irq_wake(irq);

    return rc;
}

static int pogo_request_interrupts(struct pogo_info *info) {
    struct device_node *node = info->dev->of_node;
    int rc = 0;
    const char *name;
    struct property *prop;

    of_property_for_each_string(node, "interrupt-names", prop, name) {
        rc = pogo_request_interrupt(info, node, name);
        if (rc < 0)
            return rc;
    }

    return rc;
}

static void pogo_free_interrupts(struct pogo_info *info) {
    int i;

    for (i = 0; i < ARRAY_SIZE(pogo_irqs); i++) {
        if (pogo_irqs[i].irq > 0) {
            if (pogo_irqs[i].wake)
                disable_irq_wake(pogo_irqs[i].irq);

            devm_free_irq(info->dev, pogo_irqs[i].irq,
                        pogo_irqs[i].irq_data);
        }
    }
}

static void pogo_disable_interrupts(struct pogo_info *info)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(pogo_irqs); i++) {
        if (pogo_irqs[i].irq > 0)
            disable_irq(pogo_irqs[i].irq);
    }
}

static int pogo_parse_dt(struct pogo_info *info) {
    int rc = 0;
	int irq = 0;
	int index = 0;
    struct device_node *node = info->dev->of_node;

    if (!node) {
        pr_err("device tree node missing\n");
        return -EINVAL;
    }

    rc = of_get_named_gpio(node, "pogo,vdock_in_en_gpio", 0);
    if (rc < 0) {
        pr_err("get pogo,vdock_in_en_gpio failed rc=%d\n", rc);
        return rc;
    }
    info->vdock_in_en_gpio = rc;

    rc = of_get_named_gpio(node, "pogo,vpower_out_en_gpio", 0);
    if (rc < 0) {
        pr_err("get pogo,vpower_out_en_gpio failed rc=%d\n", rc);
        return rc;
    }
    info->vpower_out_en_gpio = rc;

    rc = of_get_named_gpio(node, "pogo,pogo_id_gpio", 0);
    if (rc < 0) {
        pr_err("get pogo,pogo_id_gpio failed rc=%d\n", rc);
        return rc;
    }
    info->pogo_id_gpio = rc;

	irq = gpio_to_irq(info->pogo_id_gpio);
	index = pogo_get_irq_index_byname("pogo-id");
	pogo_irqs[index].irq = irq;

    rc = of_get_named_gpio(node, "pogo,vbus_con_det_gpio", 0);
    if (rc < 0) {
        pr_err("get pogo,vbus_con_det_gpio failed rc=%d\n", rc);
        return rc;
    }
    info->vbus_con_det_gpio = rc;

	irq = gpio_to_irq(info->vbus_con_det_gpio);
	index = pogo_get_irq_index_byname("vbus-con-det");
	pogo_irqs[index].irq = irq;

    rc = of_get_named_gpio(node, "pogo,vdock_in_det_gpio", 0);
    if (rc < 0) {
        pr_err("get pogo,vdock_in_det_gpio failed rc=%d\n", rc);
        return rc;
    }
    info->vdock_in_det_gpio = rc;

	irq = gpio_to_irq(info->vdock_in_det_gpio);
	index = pogo_get_irq_index_byname("vdock-in-det");
	pogo_irqs[index].irq = irq;

    rc = of_get_named_gpio(node, "pogo,vbus_con_en_gpio", 0);
    if (rc < 0) {
        pr_err("get pogo,vbus_con_en_gpio failed rc=%d\n", rc);
        return rc;
    }
    info->vbus_con_en_gpio = rc;

    rc = of_get_named_gpio(node, "pogo,dock_en_gpio", 0);
    if (rc < 0) {
        pr_err("get pogo,dock_en_gpio failed rc=%d\n", rc);
        return rc;
    }
    info->dock_en_gpio = rc;

    info->pinctrl = devm_pinctrl_get(info->dev);
    if (IS_ERR(info->pinctrl)) {
        pr_err("Couldn't get pinctrl rc=%ld\n", PTR_ERR(info->pinctrl));
        info->pinctrl = NULL;
        return PTR_ERR(info->pinctrl);
    }

    if (info->pinctrl) {
        info->pinctrl_default = pinctrl_lookup_state(info->pinctrl,
                        "default");
        if (IS_ERR(info->pinctrl_default)) {
            rc = PTR_ERR(info->pinctrl_default);
            pr_err("Couldn't get pinctrl default rc=%d\n", rc);
            //return rc;
        }

        info->pinctrl_otg_en_high = pinctrl_lookup_state(info->pinctrl,
                        "vdock_in_en_high");
        if (IS_ERR(info->pinctrl_otg_en_high)) {
            rc = PTR_ERR(info->pinctrl_otg_en_high);
            pr_err("Couldn't get pinctrl vdock_in_en_high rc=%d\n", rc);
            return rc;
        }

        info->pinctrl_otg_en_low = pinctrl_lookup_state(info->pinctrl,
                        "vdock_in_en_low");
        if (IS_ERR(info->pinctrl_otg_en_low)) {
            rc = PTR_ERR(info->pinctrl_otg_en_low);
            pr_err("Couldn't get pinctrl vdock_in_en_low rc=%d\n", rc);
            return rc;
        }

        info->pinctrl_vpower_out_en_high = pinctrl_lookup_state(info->pinctrl,
                        "vpower_out_en_high");
        if (IS_ERR(info->pinctrl_vpower_out_en_high)) {
            rc = PTR_ERR(info->pinctrl_vpower_out_en_high);
            pr_err("Couldn't get pinctrl vpower_out_en_high rc=%d\n", rc);
            return rc;
        }

        info->pinctrl_vpower_out_en_low = pinctrl_lookup_state(info->pinctrl,
                        "vpower_out_en_low");
        if (IS_ERR(info->pinctrl_vpower_out_en_low)) {
            rc = PTR_ERR(info->pinctrl_vpower_out_en_low);
            pr_err("Couldn't get pinctrl vpower_out_en_low rc=%d\n", rc);
            return rc;
        }

        info->pinctrl_vbus_con_en_high = pinctrl_lookup_state(info->pinctrl,
                        "vbus_con_en_high");
        if (IS_ERR(info->pinctrl_vbus_con_en_high)) {
            rc = PTR_ERR(info->pinctrl_vbus_con_en_high);
            pr_err("Couldn't get pinctrl vbus_con_en_high rc=%d\n", rc);
            return rc;
        }

        info->pinctrl_vbus_con_en_low = pinctrl_lookup_state(info->pinctrl,
                        "vbus_con_en_low");
        if (IS_ERR(info->pinctrl_vbus_con_en_low)) {
            rc = PTR_ERR(info->pinctrl_vbus_con_en_low);
            pr_err("Couldn't get pinctrl vbus_con_en_low rc=%d\n", rc);
            return rc;
        }

		info->pinctrl_dock_en_high = pinctrl_lookup_state(info->pinctrl,
						"dock_en_high");
		if (IS_ERR(info->pinctrl_dock_en_high)) {
			rc = PTR_ERR(info->pinctrl_dock_en_high);
			pr_err("Couldn't get pinctrl dock_en_high rc=%d\n", rc);
			return rc;
		}

		info->pinctrl_dock_en_low = pinctrl_lookup_state(info->pinctrl,
						"dock_en_low");
		if (IS_ERR(info->pinctrl_dock_en_low)) {
			rc = PTR_ERR(info->pinctrl_dock_en_low);
			pr_err("Couldn't get pinctrl dock_en_low rc=%d\n", rc);
			return rc;
		}

		info->vbus_con_irq_active = pinctrl_lookup_state(info->pinctrl,
						"vbus_con_irq_active");
		if (IS_ERR(info->vbus_con_irq_active)) {
			rc = PTR_ERR(info->vbus_con_irq_active);
			pr_err("Couldn't get pinctrl vbus_con_irq_active rc=%d\n", rc);
			return rc;
		}

		info->id_irq_active = pinctrl_lookup_state(info->pinctrl,
						"id_irq_active");
		if (IS_ERR(info->id_irq_active)) {
			rc = PTR_ERR(info->id_irq_active);
			pr_err("Couldn't get pinctrl id_irq_active rc=%d\n", rc);
			return rc;
		}

		info->vdock_in_irq_active = pinctrl_lookup_state(info->pinctrl,
						"dock_en_low");
		if (IS_ERR(info->vdock_in_irq_active)) {
			rc = PTR_ERR(info->vdock_in_irq_active);
			pr_err("Couldn't get pinctrl vdock_in_irq_active rc=%d\n", rc);
			return rc;
		}

    }
    return 0;
}

#if defined(CONFIG_DEBUG_FS)
static void pogo_create_debugfs(struct pogo_info *info)
{
    struct dentry *file;

    info->dfs_root = debugfs_create_dir("pogo", NULL);
    if (IS_ERR_OR_NULL(info->dfs_root)) {
        pr_err("Couldn't create charger debugfs rc=%ld\n",
            (long)info->dfs_root);
        return;
    }

    file = debugfs_create_u32("debug_mask", 0600, info->dfs_root,
            &__debug_mask);
    if (IS_ERR_OR_NULL(file))
        pr_err("Couldn't create debug_mask file rc=%ld\n", (long)file);
}

#else
static void pogo_create_debugfs(struct pogo_info *info)
{}
#endif

static ssize_t pogo_fsm_show(struct device *dev, struct device_attribute
                    *attr, char *buf)
{
    struct pogo_info *info = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%s\n", pogo_fsm_name[info->current_pogo_fsm]);
}
static DEVICE_ATTR_RO(pogo_fsm);


static ssize_t pogo_id_status_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    struct pogo_info *info = dev_get_drvdata(dev);

    schedule_delayed_work(&info->keyboard_detect_work, 0);

    return size;
}

static ssize_t pogo_id_status_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct pogo_info *info = dev_get_drvdata(dev);

    return sprintf(buf, "keyboard_present %d\n", info->keyboard_present);
}
static DEVICE_ATTR_RW(pogo_id_status);

static struct attribute *pogo_attrs[] = {
    &dev_attr_pogo_fsm.attr,
    &dev_attr_pogo_id_status.attr,
    NULL,
};
ATTRIBUTE_GROUPS(pogo);

static int pogo_driver_probe(struct platform_device *pdev) {
    struct pogo_info *info;
    int rc = 0;
    struct pogo_irq_data irq_data;

	pr_err("%s begin \n", __func__);
    flag_probe_finished = 0;
    info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
    info->dev = &pdev->dev;
    info->debug_mask = &__debug_mask;
    info->irq_info = pogo_irqs;
    spin_lock_init(&info->slock);

    INIT_DELAYED_WORK(&info->keyboard_detect_work, pogo_id_work);
	INIT_DELAYED_WORK(&info->fchg_detect_work, pogo_fchg_detect_work);
	INIT_DELAYED_WORK(&info->dock_chg_work, dock_chg_work);
    info->current_pogo_fsm = POGO_DEFAULT;

    rc = pogo_parse_dt(info);
    if (rc < 0) {
        pr_err("Couldn't parse device tree rc=%d\n", rc);
        return rc;
    }

    platform_set_drvdata(pdev, info);

    rc = pogo_determine_initial_status(info);
    if (rc < 0) {
        pr_err("Couldn't determine initial status rc=%d\n", rc);
        goto cleanup;
    }

    rc = pogo_request_interrupts(info);
    if (rc < 0) {
        pr_err("Couldn't request interrupts rc=%d\n", rc);
        goto cleanup;
    }

    pogo_create_debugfs(info);

    rc = sysfs_create_groups(&info->dev->kobj, pogo_groups);
    if (rc < 0) {
        pr_err("Couldn't create sysfs files rc=%d\n", rc);
        goto free_irq;
    }

    device_init_wakeup(info->dev, true);
    pinfo = info;

    pr_err("%s:pogo probed successfully\n", __func__);
    flag_probe_finished = 1;
    pogo_chk_cnt = 5;

    irq_data.parent_data = info;
    irq_data.name = "probe-initial-status";
    pogo_id_irq_handler(0, &irq_data);
    vbus_con_det_irq_handler(0, &irq_data);
    vdock_in_det_irq_handler(0, &irq_data);

    return rc;

free_irq:
    pogo_free_interrupts(info);
cleanup:
    platform_set_drvdata(pdev, NULL);
    pinfo = NULL;

    return rc;
}

static int pogo_remove(struct platform_device *pdev)
{
    struct pogo_info *info = platform_get_drvdata(pdev);

    pogo_free_interrupts(info);
    platform_set_drvdata(pdev, NULL);

    return 0;
}

static void pogo_shutdown(struct platform_device *pdev)
{
    struct pogo_info *info = platform_get_drvdata(pdev);

    /* disable all interrupts */
    pogo_disable_interrupts(info);
}

static struct platform_driver pogo_driver = {
    .probe        = pogo_driver_probe,
    .remove        = pogo_remove,
    .shutdown    = pogo_shutdown,
    .driver = {
        .name = "qcom, pogo_v2",
        .of_match_table = pogo_of_match,
    },
};

static int __init pogo_driver_init(void)
{
    return platform_driver_register(&pogo_driver);
}

static void __exit pogo_driver_exit(void)
{
    platform_driver_unregister(&pogo_driver);
}

module_init(pogo_driver_init);
module_exit(pogo_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("pogo Driver V2");
MODULE_AUTHOR("hailong.chen@tcl.com");
