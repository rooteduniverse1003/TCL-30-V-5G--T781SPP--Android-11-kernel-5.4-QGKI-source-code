/*
 * Copyright (C) 2020 tct Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef __POGO_DAVINCI_H
#define __POGO_DAVINCI_H

enum POGO_HOST_DEVICE {
    POGO_USB_CHARGER = BIT(0),
    POGO_DOCK_CHARGER = BIT(1),
    POGO_OTG_DEVICE = BIT(2),
    POGO_DOCK_KEYBOARD = BIT(3),
};

enum POGO_FSM {
    POGO_DEFAULT = 0x0,
    POGO_USB = POGO_USB_CHARGER,//USB CHARGER 0x1
    POGO_DOCK = POGO_DOCK_CHARGER,//DOCK CHARGER 0x2
    POGO_DOCK_USB = POGO_DOCK_CHARGER | POGO_USB_CHARGER,//DOCK CHARGER + USB CHARGER 0x3
    POGO_OTG = POGO_OTG_DEVICE,//OTG DEVICE 0x4
    POGO_OTG_USB = POGO_OTG_DEVICE | POGO_USB_CHARGER,//Error 0x5
    POGO_OTG_DOCK = POGO_OTG_DEVICE | POGO_DOCK_CHARGER,//OTG DEVICE + DOCK CHARGER 0x6
    POGO_OTG_DOCK_USB = POGO_OTG_DEVICE | POGO_DOCK_CHARGER | POGO_USB_CHARGER,//Error 0x7
    POGO_KEYBOARD = POGO_DOCK_KEYBOARD,//DOCK KEYBOARD 0x8
    POGO_KEYBOARD_USB = POGO_DOCK_KEYBOARD | POGO_USB_CHARGER,//DOCK KEYBOARD + USB CHARGER 0x9
    POGO_KEYBOARD_DOCK = POGO_DOCK_KEYBOARD | POGO_DOCK_CHARGER,//Error 0xA
    POGO_KEYBOARD_DOCK_USB = POGO_DOCK_KEYBOARD | POGO_DOCK_CHARGER | POGO_USB_CHARGER,//Error 0xB
    POGO_KEYBOARD_OTG = POGO_DOCK_KEYBOARD | POGO_OTG_DEVICE,// DOCK KEYBOARD + OTG DEVICE 0xC
    POGO_KEYBOARD_OTG_USB = POGO_DOCK_KEYBOARD | POGO_OTG_DEVICE | POGO_USB_CHARGER,//Error 0xD
    POGO_KEYBOARD_OTG_DOCK = POGO_DOCK_KEYBOARD | POGO_OTG_DEVICE | POGO_DOCK_CHARGER,//Error 0xE
    POGO_KEYBOARD_OTG_DOCK_USB = POGO_DOCK_KEYBOARD | POGO_OTG_DEVICE | POGO_DOCK_CHARGER | POGO_USB_CHARGER,//Error 0xF
};

static const char * const pogo_fsm_name[] = {
    [POGO_DEFAULT]                        = "NONE",
    [POGO_USB]                            = "USB CHARGER",
    [POGO_DOCK]                            = "DOCK CHARGER",
    [POGO_DOCK_USB]                        = "DOCK CHARGER + USB CHARGER",
    [POGO_OTG]                            = "OTG",
    [POGO_OTG_USB]                        = "OTG + USB CHARGER,Error",
    [POGO_OTG_DOCK]                        = "OTG + DOCK CHARGER",
    [POGO_OTG_DOCK_USB]                    = "OTG + DOCK CHARGER + USB CHARGER,Error",
    [POGO_KEYBOARD]                        = "KEYBOARD",
    [POGO_KEYBOARD_USB]                    = "KEYBOARD + USB CHARGER",
    [POGO_KEYBOARD_DOCK]                = "KEYBOARD + DOCK CHARGER,Error",
    [POGO_KEYBOARD_DOCK_USB]            = "KEYBOARD + DOCK CHARGER + USB CHARGER,Error",
    [POGO_KEYBOARD_OTG]                    = "KEYBOARD + OTG",
    [POGO_KEYBOARD_OTG_USB]                = "KEYBOARD + OTG + USB CHARGER,Error",
    [POGO_KEYBOARD_OTG_DOCK]            = "KEYBOARD + OTG + DOCK CHARGER,Error",
    [POGO_KEYBOARD_OTG_DOCK_USB]        = "KEYBOARD + OTG + DOCK CHARGER + USB CHARGER,Error",
};

struct pogo_irq_data {
    void            *parent_data;
    const char        *name;
};

struct pogo_irq_info {
    const char                *name;
    const irq_handler_t        handler;
    const bool                wake;
    struct pogo_irq_data    *irq_data;
    int                        irq;
    bool                    enabled;
};

struct pogo_info {
    struct device                *dev;
    char                        *name;
    struct pogo_irq_info        *irq_info;
    struct dentry                *dfs_root;
    int                            *debug_mask;

    struct pinctrl                *pinctrl;
    struct pinctrl_state        *pinctrl_default;
    struct pinctrl_state        *pinctrl_otg_en_high;
    struct pinctrl_state        *pinctrl_otg_en_low;
    struct pinctrl_state        *pinctrl_vpower_out_en_high;
    struct pinctrl_state        *pinctrl_vpower_out_en_low;
    struct pinctrl_state        *pinctrl_DPI_CK_en_high;
    struct pinctrl_state        *pinctrl_DPI_CK_en_low;
    struct pinctrl_state        *pinctrl_vbus_con_en_high;
    struct pinctrl_state        *pinctrl_vbus_con_en_low;
    struct pinctrl_state        *pinctrl_wifi_blue_high;
    struct pinctrl_state        *pinctrl_wifi_blue_low;
    struct pinctrl_state        *pinctrl_charge_green_high;
    struct pinctrl_state        *pinctrl_charge_green_low;
    struct pinctrl_state        *pinctrl_DPI_DE_en_high;
    struct pinctrl_state        *pinctrl_DPI_DE_en_low;
	struct pinctrl_state        *pinctrl_dock_en_high;
    struct pinctrl_state        *pinctrl_dock_en_low;
	struct pinctrl_state		*vbus_con_irq_active;
	struct pinctrl_state		*id_irq_active;
	struct pinctrl_state		*vdock_in_irq_active;

    unsigned int                vdock_in_en_gpio;
    unsigned int                vpower_out_en_gpio;
    unsigned int                pogo_id_gpio;
    unsigned int                vbus_con_det_gpio;
    unsigned int                vdock_in_det_gpio;
    unsigned int                vbus_con_en_gpio;
	unsigned int                dock_en_gpio;

    int                            keyboard_present;
    int                            otg_present;
    int                            usb_present;
    int                            dock_present;
	int                            chk_pogo_chg; //weijun++

    enum POGO_FSM                current_pogo_fsm;

    struct delayed_work         keyboard_detect_work;
	struct delayed_work         fchg_detect_work;
	struct delayed_work         dock_chg_work;
    spinlock_t                    slock;
    bool kpoc;
};

extern void set_current_limit_for_pogo_charger(unsigned int force_cnt);
extern int get_dock_pin_status(void);
extern void pogo_update_otg_status(bool attached);
void pogo_set_otg_detect(bool enable);
#endif /* __POGO_DAVINCI_H */
