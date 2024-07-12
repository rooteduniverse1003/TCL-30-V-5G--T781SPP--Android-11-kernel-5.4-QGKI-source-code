#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/thermal.h>
#include <linux/iio/consumer.h>
#include "smb5-lib.h"
#include <linux/rtc.h>
#include <linux/iio/consumer.h>
#include <linux/iio/iio.h>
//#include <dt-bindings/iio/qti_power_supply_iio.h>

#define TCT_NORMAL_CHARGE_LOG (0<<1)
#define TCT_DUMP_VOTE (1<<1)

static int g_log_en = 0;
static int alarm_battery_log_is_on = 0;

static struct delayed_work print_tct_charge_worker;
static struct alarm	fg_log_thread_alarm;
//extern struct smb_charger *g_qcom_chg;
//extern int bat_real_temp;

struct power_supply *usb_psy;// = power_supply_get_by_name("usb");
struct power_supply *batt_psy;

static const char * const power_supply_status_text[] = {
	"Unknown", "Charging", "Discharging", "Not charging", "Full"
};

static const char * const power_supply_charge_type_text[] = {
	"Unknown", "N/A", "Trickle", "Fast", "Taper"
};

static const char * const power_supply_health_text[] = {
	"Unknown", "Good", "Overheat", "Dead", "Over voltage",
	"Unspecified failure", "Cold", "Watchdog timer expire",
	"Safety timer expire",
	"Warm", "Cool", "Hot"
};

extern int dump_voter_client(struct votable *votable, char *buf);

static int print_time(char *buf)
{
	struct rtc_time tm;
	unsigned long sec;

	
	sec = get_seconds();
	sec -= sys_tz.tz_minuteswest*60;
	rtc_time_to_tm(sec, &tm);

	return sprintf(buf, "[%d-%02d-%02d %02d:%02d:%02d]",tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
				tm.tm_hour, tm.tm_min, tm.tm_sec);
}

static void dump_tct_charge_log(void)
{

	int skin_temp = 0;
	int quiet_temp = 0;
	//int chg_temp = 0;
	//int smb_temp = 0;
	//int mid_vol = 0;
	char *buf = NULL;
	int len = 0;
	//union power_supply_propval bat_voltage = {0};
	//union power_supply_propval bat_current = {0};
	//union power_supply_propval bat_temp = {0};
	struct thermal_zone_device *thermal_dev;

	union power_supply_propval val = {0};

	if(usb_psy==NULL)
		usb_psy= power_supply_get_by_name("usb");
	if(usb_psy==NULL)
		return;

	if(batt_psy==NULL)
		batt_psy= power_supply_get_by_name("battery");
	if(batt_psy==NULL)
		return;

	buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf) 
		return;

	if (g_log_en) {
		len += print_time(buf+len);
		//len += sprintf(buf+len, "=============== print_tct_charge_work START ======================\n");

		/*if (g_qcom_chg != NULL)*/ {

			power_supply_get_property(usb_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
			len += sprintf(buf+len, "usbin_v=%dmV, ", val.intval/1000);

			power_supply_get_property(usb_psy, POWER_SUPPLY_PROP_CURRENT_NOW, &val);
			len += sprintf(buf+len, "usbin_i=%dmA, ", val.intval/1000);




			/*smblib_read_iio_channel(g_qcom_chg, g_qcom_chg->iio.temp_chan, 100, &chg_temp);
			len += sprintf(buf+len, " chg_temp=%d  ", chg_temp);

			smblib_read_iio_channel(g_qcom_chg, g_qcom_chg->iio.smb_temp_chan, 100, &smb_temp);
			*/

			power_supply_get_property(batt_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
			len += sprintf(buf+len, " bat_voltage=%dmV   ", val.intval/1000);

			power_supply_get_property(batt_psy, POWER_SUPPLY_PROP_TEMP, &val);
			len += sprintf(buf+len, " bat_temp=%d", val.intval);
			//len += sprintf(buf+len, " bat_temp=%d", bat_real_temp);

			power_supply_get_property(batt_psy, POWER_SUPPLY_PROP_CURRENT_NOW, &val);
			len += sprintf(buf+len, " bat_current=%dmA, \n", val.intval/1000);

			//smblib_read_iio_channel(g_qcom_chg, g_qcom_chg->iio.mid_chan, 100, &mid_vol);
			//len += sprintf(buf+len, " mid_vol=%dmv   ", mid_vol);

		}

		thermal_dev = thermal_zone_get_zone_by_name("skin-therm-adc");
		if (!IS_ERR_OR_NULL(thermal_dev)) {
			thermal_zone_get_temp(thermal_dev, &skin_temp);
			len += sprintf(buf+len, " skin_temp=%d   ", skin_temp/100);
		}

		thermal_dev = thermal_zone_get_zone_by_name("quiet-therm-adc");
		if (!IS_ERR_OR_NULL(thermal_dev)) {
			thermal_zone_get_temp(thermal_dev, &quiet_temp);
			len += sprintf(buf+len, " quiet_temp=%d\n", quiet_temp/100);
		}

		//len += sprintf(buf+len, "================== print_tct_charge_work END ======================\n");
		pr_err("%s", buf);
	}

	//if (g_log_en) {
	//	dump_voter_client(g_qcom_chg->fcc_votable, NULL);
	//	dump_voter_client(g_qcom_chg->fcc_main_votable, NULL);
	//	dump_voter_client(g_qcom_chg->usb_icl_votable, NULL);
	//}

	kfree(buf);

}

void print_log_for_fg(void)
{
	struct timespec ts;
	struct rtc_time tm;

	static int batt_voltage=0, batt_curr=0xFFFF, batt_temp=-500, batt_cap=-1;
	static int status=0;
	static int health=0;
	static int charging_type=0;
	static int charger_input_voltage=0;

	if(usb_psy==NULL)
		usb_psy= power_supply_get_by_name("usb");
	if(usb_psy==NULL)
		return;

	if(batt_psy==NULL)
		batt_psy= power_supply_get_by_name("battery");
	if(batt_psy==NULL)
		return;


	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);


	power_supply_get_property(batt_psy, POWER_SUPPLY_PROP_STATUS, (union power_supply_propval *)&status);
	power_supply_get_property(batt_psy, POWER_SUPPLY_PROP_HEALTH, (union power_supply_propval *)&health);
	power_supply_get_property(batt_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, (union power_supply_propval *)&batt_voltage);
	power_supply_get_property(batt_psy, POWER_SUPPLY_PROP_CAPACITY, (union power_supply_propval *)&batt_cap);
	power_supply_get_property(batt_psy, POWER_SUPPLY_PROP_CURRENT_NOW, (union power_supply_propval *)&batt_curr);
	power_supply_get_property(batt_psy, POWER_SUPPLY_PROP_TEMP, (union power_supply_propval *)&batt_temp);
	power_supply_get_property(batt_psy, POWER_SUPPLY_PROP_CHARGE_TYPE, (union power_supply_propval *)&charging_type);
	power_supply_get_property(usb_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, (union power_supply_propval *)&charger_input_voltage);


	printk(KERN_ERR "%s:%d-%02d-%02d %02d:%02d:%02d status:%s,health:%s,voltage:%dmV,capacity:%d,current:%dmA,temperature:%s%d.%dC,chgvoltage:%dmV,charging_type:%s\n",__func__,
		    tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,tm.tm_hour, tm.tm_min, tm.tm_sec,
			    power_supply_status_text[status],power_supply_health_text[health],batt_voltage/1000,batt_cap,batt_curr/1000,(batt_temp<0&&(batt_temp/10==0))?"-":"", batt_temp/10,(int)abs(batt_temp%10),charger_input_voltage,power_supply_charge_type_text[charging_type]);
}

static void print_tct_charge_work(struct work_struct *work)
{
	if (g_log_en)
		dump_tct_charge_log();

	if (alarm_battery_log_is_on)
		print_log_for_fg();
}

static ssize_t enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d", g_log_en);
}

static ssize_t enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	int val = 0;

	val = simple_strtoul(buf, NULL, 16);
	g_log_en = val;

	if (val) {
		alarm_start_relative(&fg_log_thread_alarm, ns_to_ktime(10LL * NSEC_PER_SEC));
	}

	return len;
}

static DEVICE_ATTR(enable, S_IWUSR|S_IRUGO,
	enable_show, enable_store);

static ssize_t store_set_alarm_log_on(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	unsigned long val = 0;
	int ret;
	if(alarm_battery_log_is_on>0)
		return size;

	if (buf != NULL && size != 0) {
		pr_err("[%s] buf is %s\n",__func__, buf);
		ret = kstrtoul(buf, 10, &val);

		alarm_battery_log_is_on=1;
		alarm_start_relative(&fg_log_thread_alarm, ns_to_ktime(10LL * NSEC_PER_SEC));
	}
	return size;
}

static ssize_t show_set_alarm_log_on(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", alarm_battery_log_is_on);
}

static DEVICE_ATTR(set_alarm_log_on, 0664, show_set_alarm_log_on, store_set_alarm_log_on);

static struct device_attribute *tct_charge_log_attrs[] = {
	[0]		= &dev_attr_enable,
	[1]		= &dev_attr_set_alarm_log_on,
};

static enum alarmtimer_restart fg_log_thread_func(struct alarm *alarm, ktime_t now)
{
	//print_log_for_fg();
	queue_delayed_work(system_power_efficient_wq/*private_chg_wq*/, &print_tct_charge_worker, 0);
	alarm_forward_now(&fg_log_thread_alarm, ns_to_ktime(10LL * NSEC_PER_SEC));
	return ALARMTIMER_RESTART;
}

static void fg_log_thread_init(void)
{
	alarm_init(&fg_log_thread_alarm, ALARM_REALTIME, fg_log_thread_func);
}

static int __init tct_charge_hw_med_test_log_init(void)
{
	struct platform_device *dev = NULL;

	dev = platform_device_register_simple("tct_charge_log", -1, NULL, 0);

	device_create_file(&dev->dev, tct_charge_log_attrs[0]);
	device_create_file(&dev->dev, tct_charge_log_attrs[1]);

	fg_log_thread_init();

	INIT_DELAYED_WORK(&print_tct_charge_worker, print_tct_charge_work);
	if (g_log_en || alarm_battery_log_is_on)
		alarm_start_relative(&fg_log_thread_alarm, ns_to_ktime(10LL * NSEC_PER_SEC));

	return 0;
}

static void __exit tct_charge_hw_med_test_log_exit(void)
{
}

MODULE_LICENSE("GPL");

module_init(tct_charge_hw_med_test_log_init);
module_exit(tct_charge_hw_med_test_log_exit);
