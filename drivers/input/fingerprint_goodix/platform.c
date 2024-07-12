/*
 * platform indepent driver interface
 *
 * Coypritht (c) 2017 Goodix
 */
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/input.h>

#include "gf_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif
#ifdef CONFIG_DRM
#include <drm/drm_panel.h>
#include <linux/of.h>
#elif defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#if IS_ENABLED(CONFIG_TCT_DEVICE_INFO)
extern unsigned char FP[128];
#endif
/* MODIFIED-BEGIN by zhikui.li, Added  bright screen notification*/
static int check_dt(struct device_node *np)
{
	int i = 0;
	int count = 0;
	struct device_node *node = NULL;
	struct drm_panel *panel = NULL;

	count = of_count_phandle_with_args(np, "panel", NULL);
	pr_err("panel number = %d", count);
	if (count <= 0) {
		pr_err("Not find panel");
		return 0;
	}

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, "panel", i);
		panel = of_drm_find_panel(node);
		of_node_put(node);
		if (!IS_ERR(panel)) {
			active_panel = panel;
		    pr_err("Find active_panel ok\n");
			return 0;
		}
	}

	pr_err("Find active_panel fail");
	return -ENODEV;
}
/* MODIFIED-end by zhikui.li, Added  bright screen notification*/
int gf_parse_dts(struct gf_dev *gf_dev)
{
	int rc = 0;
	struct device *dev = &gf_dev->spi->dev;
	struct device_node *np = dev->of_node;

/* MODIFIED-BEGIN by zhikui.li, Added  bright screen notification*/
	if (check_dt(np)) {
	     rc=check_dt(np);
	     if(rc<0)
	        pr_err("goodix finger Find active_panel fail");
    }
#if defined(CONFIG_DRM)
	if (active_panel) {
	     rc = drm_panel_notifier_register(active_panel, &gf_dev->notifier);
	     if (rc) {
	          pr_err("[FB]Unable to register goodix finger notifier: %d", rc);
	     }
	}
#endif
/* MODIFIED-end by zhikui.li, Added  bright screen notification*/
	gf_dev->reset_gpio = of_get_named_gpio(np, "fp-gpio-reset", 0);
	if (gf_dev->reset_gpio < 0) {
		pr_err("falied to get reset gpio!\n");
		return gf_dev->reset_gpio;
	}

	rc = gpio_request(gf_dev->reset_gpio, "goodix_reset");
	if (rc) {
		pr_err("failed to request reset gpio, rc = %d\n", rc);
		goto err_reset;
	}
	gpio_direction_output(gf_dev->reset_gpio, 1);

	gf_dev->irq_gpio = of_get_named_gpio(np, "fp-gpio-irq", 0);
	if (gf_dev->irq_gpio < 0) {
		pr_err("falied to get irq gpio!\n");
		return gf_dev->irq_gpio;
	}

	rc = gpio_request(gf_dev->irq_gpio, "goodix_irq");
	if (rc) {
		pr_err("failed to request irq gpio, rc = %d\n", rc);
		goto err_irq;
	}
	gpio_direction_input(gf_dev->irq_gpio);


    gf_dev->pwr_gpio = of_get_named_gpio(np, "fp-gpio-pwr", 0);
    if (gf_dev->pwr_gpio < 0) {
        pr_err("falied to get pwr gpio!\n");
        return gf_dev->pwr_gpio;
    }
    rc = gpio_request(gf_dev->pwr_gpio, "goodix_gpio_power_pin");
    if (rc) {
		pr_err("failed to request pwr gpio, rc = %d\n", rc);
		goto err_irq;
    }
    rc = gpio_direction_output(gf_dev->pwr_gpio, 0); // power off.
    if (rc) {
		pr_err("failed to gpio_direction_output pwr gpio, rc = %d\n", rc);
    }
    pr_err("goodix gpio request ok\n");
#if IS_ENABLED(CONFIG_TCT_DEVICE_INFO)
    strcpy(FP, "3626:goodix");
#endif
err_irq:
	gpio_free(gf_dev->reset_gpio);
err_reset:
	return rc;
}

void gf_cleanup(struct gf_dev *gf_dev)
{
	pr_info("[info] %s\n", __func__);
#if IS_ENABLED(CONFIG_TCT_DEVICE_INFO)
    strcpy(FP, "NA");
#endif
	if (gpio_is_valid(gf_dev->irq_gpio)) {
		gpio_free(gf_dev->irq_gpio);
		pr_info("remove irq_gpio success\n");
	}
	if (gpio_is_valid(gf_dev->reset_gpio)) {
		gpio_free(gf_dev->reset_gpio);
		pr_info("remove reset_gpio success\n");
	}
	if (gpio_is_valid(gf_dev->pwr_gpio)) {
		gpio_free(gf_dev->pwr_gpio);
		pr_info("remove pwr_gpio success\n");
	}

	// start, guang,xiao@tcl.com, 20210715, unregister input and notifier
	if(gf_dev->input) {
		input_unregister_device(gf_dev->input);
		//input_free_device(gf_dev->input);
		gf_dev->input = NULL;
		pr_info("remove input event success\n");
	}

	#ifdef CONFIG_DRM
    //msm_drm_unregister_client(&gf_dev->notifier);
    if (active_panel){
		if (drm_panel_notifier_unregister(active_panel, &gf_dev->notifier))
			pr_err("Error occurred while unregistering goodix finger_notifier.");
		pr_info("remove goodix drm panel notifier suceess\n");
	}
	#elif defined(CONFIG_FB)
		fb_unregister_client(&gf_dev->notifier);
		pr_info("remove goodix fb panel notifier suceess\n");
	#endif
	// end, guang,xiao@tcl.com, 20210715, unregister input and notifier
}

int gf_power_on(struct gf_dev *gf_dev)
{
	int rc = 0;

	rc = gpio_direction_output(gf_dev->pwr_gpio, 1);
    msleep(5);
    pr_err("gf_power_on\n");
	/* TODO: add your power control here */
	return rc;
}

int gf_power_off(struct gf_dev *gf_dev)
{
	int rc = 0;

    rc = gpio_direction_output(gf_dev->pwr_gpio, 0);
	msleep(5);
	/* TODO: add your power control here */
    pr_err("gf_power_off\n");
	return rc;
}

int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
	if (gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return -1;
	}
	gpio_direction_output(gf_dev->reset_gpio, 1);
	gpio_set_value(gf_dev->reset_gpio, 0);
	mdelay(3);
	gpio_set_value(gf_dev->reset_gpio, 1);
	mdelay(delay_ms);
	return 0;
}

int gf_irq_num(struct gf_dev *gf_dev)
{
	if (gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return -1;
	} else {
		return gpio_to_irq(gf_dev->irq_gpio);
	}
}
