/**
 * plat-msm8916.c
 *
 **/

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "ff_log.h"
#include "ff_ctl.h"

# undef LOG_TAG
#define LOG_TAG "sm4350"

#define FF_COMPATIBLE_NODE "focaltech,fingerprint"

/*
 * GPIO context definition and its singleton instance.
 */
typedef struct {
    int32_t gpio_rst_pin;
    int32_t gpio_int_pin;
    int32_t gpio_power_pin;
    int32_t gpio_iovcc_pin;
} ff_plat_context_t;

static ff_plat_context_t ff_plat_context = {
    .gpio_rst_pin   = -1,
    .gpio_int_pin   = -1,
    .gpio_power_pin = -1,
    .gpio_iovcc_pin = -1,
}, *g_context = &ff_plat_context;

int ff_ctl_init_pins(int *irq_num)
{
    int err = 0, gpio;
    struct device_node *dev_node = NULL;
    enum of_gpio_flags flags;
    bool b_config_dirtied = false;
    FF_LOGV("'%s' enter.", __func__);

    if (unlikely(!g_context)) {
        return (-ENOSYS);
    }

    /* Find device tree node. */
    dev_node = of_find_compatible_node(NULL, NULL, FF_COMPATIBLE_NODE);
    if (!dev_node) {
        FF_LOGE("of_find_compatible_node(.., '%s') failed.", FF_COMPATIBLE_NODE);
        return (-ENODEV);
    }

    /* Initialize RST pin. */
    gpio = of_get_named_gpio_flags(dev_node, "fp,reset_gpio", 0, &flags);
    if (gpio > 0) {
        g_context->gpio_rst_pin = gpio;
        b_config_dirtied = true;
    }
    if (!gpio_is_valid(g_context->gpio_rst_pin)) {
        FF_LOGE("g_context->gpio_rst_pin(%d) is invalid.", g_context->gpio_rst_pin);
        return (-ENODEV);
    }
    err = gpio_request(g_context->gpio_rst_pin, "ff_gpio_rst_pin");
    if (err) {
        FF_LOGE("gpio_request(%d) = %d.", g_context->gpio_rst_pin, err);
        return err;
    }
    err = gpio_direction_output(g_context->gpio_rst_pin, 0);
    if (err) {
        FF_LOGE("gpio_direction_output(%d, 1) = %d.", g_context->gpio_rst_pin, err);
        return err;
    }

    /* Initialize PWR/VDD pin. */
    gpio = of_get_named_gpio_flags(dev_node, "fp,vdd_gpio", 0, &flags);
    if (gpio > 0) {
        g_context->gpio_power_pin = gpio;
        b_config_dirtied = true;
    }
    if (!gpio_is_valid(g_context->gpio_power_pin)) {
        FF_LOGE("g_context->gpio_power_pin(%d) is invalid.", g_context->gpio_power_pin);
        return (-ENODEV);
    }
    err = gpio_request(g_context->gpio_power_pin, "ff_gpio_power_pin");
    if (err) {
        FF_LOGE("gpio_request(%d) = %d.", g_context->gpio_power_pin, err);
        return err;
    }
    err = gpio_direction_output(g_context->gpio_power_pin, 0); // power off.
    if (err) {
        FF_LOGE("gpio_direction_output(%d, 0) = %d.", g_context->gpio_power_pin, err);
        return err;
    }

#if 0
    /* Initialize IOVCC pin. */
    gpio = of_get_named_gpio_flags(dev_node, "fp,iovcc_gpio", 0, &flags);
    if (gpio > 0) {
        g_context->gpio_iovcc_pin = gpio;
        b_config_dirtied = true;
    }
    if (!gpio_is_valid(g_context->gpio_iovcc_pin)) {
        FF_LOGE("g_context->gpio_iovcc_pin(%d) is invalid.", g_context->gpio_iovcc_pin);
        return (-ENODEV);
    }
    err = gpio_request(g_context->gpio_iovcc_pin, "ff_gpio_iovcc_pin");
    if (err) {
        FF_LOGE("gpio_request(%d) = %d.", g_context->gpio_iovcc_pin, err);
        return err;
    }
    err = gpio_direction_output(g_context->gpio_iovcc_pin, 0); // power off.
    if (err) {
        FF_LOGE("gpio_direction_output(%d, 0) = %d.", g_context->gpio_iovcc_pin, err);
        return err;
    }
#endif

    /* Initialize INT pin. */
    gpio = of_get_named_gpio_flags(dev_node, "fp,irq_gpio", 0, &flags);
    if (gpio > 0) {
        g_context->gpio_int_pin = gpio;
        b_config_dirtied = true;
    }
    if (!gpio_is_valid(g_context->gpio_int_pin)) {
        FF_LOGE("g_context->gpio_int_pin(%d) is invalid.", g_context->gpio_int_pin);
        return (-ENODEV);
    }
    err = gpio_request(g_context->gpio_int_pin, "ff_gpio_int_pin");
    if (err) {
        FF_LOGE("gpio_request(%d) = %d.", g_context->gpio_int_pin, err);
        return err;
    }
    err = gpio_direction_input(g_context->gpio_int_pin);
    if (err) {
        FF_LOGE("gpio_direction_input(%d) = %d.", g_context->gpio_int_pin, err);
        return err;
    }

    /* Retrieve the IRQ number. */
    *irq_num = gpio_to_irq(g_context->gpio_int_pin);
    if (*irq_num < 0) {
        FF_LOGE("gpio_to_irq(%d) failed.", g_context->gpio_int_pin);
        return (-EIO);
    } else {
        FF_LOGD("gpio_to_irq(%d) = %d.", g_context->gpio_int_pin, *irq_num);
    }

//post reset sequence
    mdelay(1);
    err = gpio_direction_output(g_context->gpio_power_pin, 1); // power on.
    if (err) {
        FF_LOGE("gpio_direction_output(%d, 0) = %d.", g_context->gpio_power_pin, err);
        return err;
    }
    mdelay(3);
    err = gpio_direction_output(g_context->gpio_rst_pin, 1);
    if (err) {
        FF_LOGE("gpio_direction_output(%d, 1) = %d.", g_context->gpio_rst_pin, err);
        return err;
    }


    /* Configuration is dirty, must sync back to HAL. */
    if (!err && b_config_dirtied) {
        err = 1;
    }

    FF_LOGV("'%s' leave.", __func__);
    return err;
}

int ff_ctl_free_pins(void)
{
    int err = 0;
    FF_LOGV("'%s' enter.", __func__);

	if (unlikely(!g_context)) {
        return (-ENOSYS);
    }

    /* Release GPIO resources. */
    gpio_free(g_context->gpio_rst_pin  );
    gpio_free(g_context->gpio_int_pin  );
//    gpio_free(g_context->gpio_iovcc_pin);
    gpio_free(g_context->gpio_power_pin);

    g_context->gpio_rst_pin   = -1;
    g_context->gpio_int_pin   = -1;
    g_context->gpio_power_pin = -1;
//    g_context->gpio_iovcc_pin = -1;

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
    FF_LOGV("'%s' enter.", __func__);
    FF_LOGD("power: '%s'.", on ? "on" : "off");

    if (unlikely(!g_context)) {
        return (-ENOSYS);
    }

    if (on) {
        err = gpio_direction_output(g_context->gpio_power_pin, 1);
        msleep(5);
//        err = gpio_direction_output(g_context->gpio_iovcc_pin, 1);
    } else {
       // err = gpio_direction_output(g_context->gpio_iovcc_pin, 0);
        err = gpio_direction_output(g_context->gpio_power_pin, 0);
        msleep(5);
    }

    FF_LOGV("'%s' leave.", __func__);
    return err;
}

int ff_ctl_reset_device(void)
{
    int err = 0;
    FF_LOGV("'%s' enter.", __func__);

    if (unlikely(!g_context)) {
        return (-ENOSYS);
    }
    /* Pull up RST pin. */
    err = gpio_direction_output(g_context->gpio_rst_pin, 1);
    mdelay(1);

    /* 3-1: Pull down RST pin. */
    err = gpio_direction_output(g_context->gpio_rst_pin, 0);

    /* 3-2: Delay for 10ms. */
    mdelay(10);

    /* Pull up RST pin. */
    err = gpio_direction_output(g_context->gpio_rst_pin, 1);

    FF_LOGV("'%s' leave.", __func__);
    return err;
}

const char *ff_ctl_arch_str(void)
{
    return "sm4350";
}

