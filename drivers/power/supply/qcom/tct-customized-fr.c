#define pr_fmt(fmt) "[SMBLIB5]: " fmt

#include <linux/init.h>
#include <linux/power_supply.h>
#include "smb5-lib.h"
#if defined(CONFIG_DRM)
#if defined(CONFIG_DRM_PANEL)
#include <drm/drm_panel.h>
#else
#include <linux/msm_drm_notify.h>
#endif
#elif defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/iio/consumer.h>

#define smblib_err(chg, fmt, ...)		\
	pr_err_ratelimited("%s: " fmt, \
		__func__, ##__VA_ARGS__)
#define smblib_dbg(chg, reason, fmt, ...)			\
	do {							\
		if (*chg->debug_mask & (reason))		\
			pr_err_ratelimited("%s: " fmt, \
				__func__, ##__VA_ARGS__);	\
		else						\
			pr_debug_ratelimited("%s: " fmt, \
				__func__, ##__VA_ARGS__);	\
	} while (0)

/************** battery current thermal for screen on or off ******************/
/* smb5-lib.h qpnp-smb5.c add some patch */
#if defined(CONFIG_DRM)
#if defined(CONFIG_DRM_PANEL)
static int smblib_check_panel_dt(struct smb_charger *chg)
{
	int i;
	int count = 0;
	struct device_node *node, *np = chg->dev->of_node;
	struct drm_panel *panel;
	
	if (!np) {
		smblib_err(chg, "Failed to find device-tree node\n");
		return -ENXIO;
	}
	count = of_count_phandle_with_args(np, "panel", NULL);
	if (count <= 0)
		return -ENODEV;

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, "panel", i);
		panel = of_drm_find_panel(node);
		of_node_put(node);
		if (!IS_ERR(panel)) {
			chg->active_panel = panel;
			return 0;
		}
	}

	return -ENODEV;
}

static int smblib_drm_notifier_callback(struct notifier_block *self,
						unsigned long event, void *data)
{
	struct drm_panel_notifier *evdata = data;
	int *blank = NULL;
	union power_supply_propval pval;
	struct smb_charger *chg = container_of(self, struct smb_charger, fb_nb);
	//smblib_err(chg,"pige smblib_drm_notifier_callback enter");
	if (!evdata) {
		smblib_err(chg, "evdata is null\n");
		return 0;
	}

	if (event != DRM_PANEL_EVENT_BLANK) {
		//smblib_err(chg,"DRM event(%lu) do not need process\n", event);
		return 0;
	}

	if (evdata->data && event == DRM_PANEL_EVENT_BLANK && chg) {
		blank = evdata->data;
		pval.intval = chg->system_temp_level;
		switch (*blank) {
		case DRM_PANEL_BLANK_UNBLANK:
			chg->is_screen_on = true;
			smblib_set_prop_system_temp_level(chg, &pval);
			break;
		case DRM_PANEL_BLANK_POWERDOWN:
			chg->is_screen_on = false;
			smblib_set_prop_system_temp_level(chg, &pval);
			break;
		default:
			//smblib_err(chg, "DRM blank(%d) do not need process\n", *blank);
			break;
		}
	}
	//smblib_err(chg,"pige smblib_drm_notifier_callback leave");
	return 0;

}
#else
static int smblib_drm_notifier_callback(struct notifier_block *self,
										unsigned long event, void *data)
{
	struct msm_drm_notifier *evdata = data;
	int *blank = NULL;
	union power_supply_propval pval;
	struct smb_charger *chg = container_of(self, struct smb_charger, fb_nb);

	if (!evdata) {
		smblib_err(chg, "evdata is null\n");
		return 0;
	}

	if (event != MSM_DRM_EVENT_BLANK) {
		smblib_dbg(chg, PR_MISC, "DRM event(%lu) do not need process\n", event);
		return 0;
	}

	if (evdata->data && event == MSM_DRM_EVENT_BLANK && chg) {
		blank = evdata->data;
		pval.intval = chg->system_temp_level;
		switch (*blank) {
		case MSM_DRM_BLANK_UNBLANK:
			chg->is_screen_on = true;
			smblib_set_prop_system_temp_level(chg, &pval);
			break;
		case MSM_DRM_BLANK_POWERDOWN:
			chg->is_screen_on = false;
			smblib_set_prop_system_temp_level(chg, &pval);
			break;
		default:
			smblib_dbg(chg, PR_MISC, "DRM blank(%d) do not need process\n", *blank);
			break;
		}
	}

	return 0;

}
#endif
#elif defined(CONFIG_FB)
static int smblib_fb_notifier_callback(struct notifier_block *self,
										unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank = NULL;
	union power_supply_propval pval;
	struct smb_charger *chg = container_of(self, struct smb_charger, fb_nb);

	if (!evdata) {
		smblib_err(chg, "evdata is null\n");
		return 0;
	}

	if (event != FB_EVENT_BLANK) {
		smblib_dbg(chg, PR_MISC, "FB event(%lu) do not need process\n", event);
		return 0;
	}

	if (evdata->data && event == FB_EVENT_BLANK && chg) {
		blank = evdata->data;
		pval.intval = chg->system_temp_level;
		switch (*blank) {
		case FB_BLANK_UNBLANK:
			chg->is_screen_on = true;
			smblib_set_prop_system_temp_level(chg, &pval);
			break;
		case FB_BLANK_POWERDOWN:
			chg->is_screen_on = false;
			smblib_set_prop_system_temp_level(chg, &pval);
			break;
		default:
			smblib_dbg(chg, PR_MISC, "FB blank(%d) do not need process\n", *blank);
			break;
		}
	}

	return 0;
}
#endif

static int __init tct_thermal_init(void)
{
	struct power_supply	*batt_psy = NULL;
	struct smb_charger *chg = NULL;
	int rc = 0;
	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy)
		return -ENODEV;

	chg = power_supply_get_drvdata(batt_psy);
	if (chg) {//chg
		chg->is_screen_on = true;
#if defined(CONFIG_DRM)
		chg->fb_nb.notifier_call = smblib_drm_notifier_callback;
#if defined(CONFIG_DRM_PANEL)
		smblib_check_panel_dt(chg);
		if (chg->active_panel) {
			smblib_err(chg, "find active_panel\n");
			rc = drm_panel_notifier_register(chg->active_panel, &chg->fb_nb);
			if (rc < 0) {
				smblib_err(chg, "Couldn't register DRM_PANEL notifier rc = %d\n", rc);
				goto err_notifier_register;
			}
		} else {
			smblib_err(chg, "Couldn't find active_panel\n");
			goto err_notifier_register;
		}
#else
		rc = msm_drm_register_client(&chg->fb_nb);
		if (rc < 0) {
			smblib_err(chg, "Couldn't register DRM notifier rc = %d\n", rc);
			goto err_notifier_register;
		}
#endif
#elif defined(CONFIG_FB)
		chg->fb_nb.notifier_call = smblib_fb_notifier_callback;
		rc = fb_register_client(&chg->fb_nb);
		if (rc < 0) {
			smblib_err(chg, "Couldn't register FB notifier rc = %d\n", rc);
			goto err_notifier_register;
		}
#endif
	} else {
		pr_err("[%s]chg is null\n", __func__);
		return -ENODATA;
	}

	return 0;
err_notifier_register:
	chg->is_screen_on = false;
	return rc;
}
late_initcall(tct_thermal_init);
