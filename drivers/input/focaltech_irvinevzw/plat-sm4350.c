/**
 * plat-msm8916.c
 *
**/

#include <linux/stddef.h>
#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>

#include "ff_log.h"
#include "ff_ctl.h"

# undef LOG_TAG
#define LOG_TAG "sm4350"

#define FF_COMPATIBLE_NODE "focaltech,fingerprint"

/* Define pinctrl state types. */
typedef enum {
    FF_PINCTRL_STATE_PWR_ACT,
    FF_PINCTRL_STATE_PWR_CLR,
    FF_PINCTRL_STATE_RST_CLR,
    FF_PINCTRL_STATE_RST_ACT,
    FF_PINCTRL_STATE_INT_ACT,
    FF_PINCTRL_STATE_MAXIMUM /* Array size */
} ff_pinctrl_state_t;

/* Define pinctrl state names. */
static const char *g_pinctrl_state_names[FF_PINCTRL_STATE_MAXIMUM] = {
    "focalfp_pwr_active",
    "focalfp_pwr_reset",
    "focalfp_reset_reset",
    "focalfp_reset_active",
    "focalfp_irq_active"
};

/* Native context and its singleton instance. */
typedef struct {
    struct pinctrl *pinctrl;
    struct pinctrl_state *pin_states[FF_PINCTRL_STATE_MAXIMUM];
} ff_msm8916_context_t;
static ff_msm8916_context_t ff_msm8916_context, *g_context = &ff_msm8916_context;

int ff_ctl_enable_power(bool on);

int ff_ctl_init_pins(int *irq_num)
{
    int err = 0, i;
	int irq_num1 = 0;
    struct device_node *dev_node = NULL;
    struct platform_device *pdev = NULL;

    FF_LOGV("'%s' enter.", __func__);

    /* Find device tree node. */
    dev_node = of_find_compatible_node(NULL, NULL, FF_COMPATIBLE_NODE);
    if (!dev_node) {
        FF_LOGV("of_find_compatible_node(.., '%s') failed.", FF_COMPATIBLE_NODE);
        return (-ENODEV);
    }
    FF_LOGV("dev_node :%s",dev_node->name);

	irq_num1 = irq_of_parse_and_map(dev_node, 0);
	*irq_num = irq_num1;
    FF_LOGV("irq number is %d.", irq_num1);

    /* Convert to platform device. */
    pdev = of_find_device_by_node(dev_node);
    if (!pdev) {
        FF_LOGV("of_find_device_by_node(..) failed.");
        return (-ENODEV);
    }

    /* Retrieve the pinctrl handler. */
    g_context->pinctrl = devm_pinctrl_get(&pdev->dev);
    if (!g_context->pinctrl) {
        FF_LOGV("devm_pinctrl_get(..) failed.");
        return (-ENODEV);
    }

    FF_LOGV("register pins.");
    /* Register all pins. */
    for (i = 0; i < FF_PINCTRL_STATE_MAXIMUM; ++i) {
        g_context->pin_states[i] = pinctrl_lookup_state(g_context->pinctrl, g_pinctrl_state_names[i]);
        if (!g_context->pin_states[i]) {
            FF_LOGV("can't find pinctrl state for '%s'.", g_pinctrl_state_names[i]);
            err = (-ENODEV);
            break;
        }
    }
    if (i < FF_PINCTRL_STATE_MAXIMUM) {
        return (-ENODEV);
    }

    ff_ctl_enable_power(1);

    /* Initialize the INT pin. */
    FF_LOGV("init int pin.");
    err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_INT_ACT]);
    if(err){
		FF_LOGE("pinctrl_select_state(.., \"irq\") failed.");
    }
    else{
        FF_LOGE("pinctrl_select_state(.., \"irq\") successed.");
    }
//[TCTNB][FINGERPRINT]Modified end by Jiancong.Huang for P11424937 at 2021/7/26

    FF_LOGV("'%s' leave.", __func__);
    return err;
}

int ff_ctl_free_pins(void)
{
    int err = 0;
    FF_LOGV("'%s' enter.", __func__);

    ff_ctl_enable_power(0);

	if (g_context->pinctrl) {
        pinctrl_put(g_context->pinctrl);
        g_context->pinctrl = NULL;
    }
    FF_LOGV("'%s' leave.", __func__);
    return err;
}

int ff_ctl_enable_spiclk(bool on)
{
    int err = 0;
    FF_LOGV("'%s' enter.", __func__);
    FF_LOGD("clock: '%s'.", on ? "enable" : "disabled");

    if (on) {
        // TODO:
    } else {
        // TODO:
    }

    FF_LOGV("'%s' leave.", __func__);
    return err;
}

int ff_ctl_enable_power(bool on)
{
    int err = 0;
    static bool flag_power = 0;
    FF_LOGV("'%s' enter.", __func__);
    FF_LOGV("power: '%s'.", on ? "on" : "off");
    FF_LOGV("flag_power: '%s'.", flag_power ? "on" : "off");

    if (unlikely(!g_context->pinctrl)) {
        return (-ENOSYS);
    }

    if (!flag_power && on) {
        err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_RST_CLR]);
        mdelay(3);
        err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_PWR_ACT]);
        mdelay(5);
        err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_RST_ACT]);
        flag_power = 1;
    } else if (flag_power && !on) {
        err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_RST_CLR]);
        mdelay(3);
        err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_PWR_CLR]);
        mdelay(5);
        flag_power = 0;
    }

    FF_LOGV("'%s' leave.", __func__);
    return err;
}

int ff_ctl_reset_device(void)
{
    int err = 0;
    FF_LOGV("'%s' enter.", __func__);

    if (unlikely(!g_context->pinctrl)) {
        return (-ENOSYS);
    }

	err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_RST_ACT]);
    FF_LOGV("pull rst up");
	mdelay(5);
    /* 3-1: Pull down RST pin. */
	err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_RST_CLR]);
    FF_LOGV("pull rst down");
    /* 3-2: Delay for 10ms. */
    mdelay(5);
    /* Pull up RST pin. */
    err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_RST_ACT]);
    FF_LOGV("pull rst up");
     
    FF_LOGV("'%s' leave.", __func__);
    return err;
}

const char *ff_ctl_arch_str(void)
{
    return "sm4350";
}

