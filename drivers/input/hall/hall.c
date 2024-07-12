#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/input/mt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <asm/uaccess.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/jiffies.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>

#include "wakelock.h"

struct hall_data {
	int irq;
	int irq_gpio;
	struct workqueue_struct *hall_wq;
	struct work_struct hall_work;
	struct delayed_work delay_hall_work;
	struct platform_device	*pdev;
};

/*defined input key*/
#define HALL_KEYCODE           252     /* enable hall*/

static struct pinctrl         *hall_eint_pinctrl;
static struct pinctrl_state   *hall_eint_active;
#ifdef HALL_POWER_PIN
static struct pinctrl_state   *hall_power_en_enable;
static struct pinctrl_state   *hall_power_en_disable;
#endif

//static struct pinctrl_state *hall_eint_default;
//static unsigned int eint_debounce;

static int prob_flag =0;
struct hall_data *hall = NULL;
struct input_dev *hall_input = NULL;
static unsigned int hall_status = 0;
static struct wake_lock hall_wakelock;	

static struct class *hall_class = NULL;
static struct device *hall_dev = NULL;
static unsigned int hall_power_on  = 0;

static struct of_device_id hall_of_match[] = {
	{.compatible = "hall-switch,tct", },
	{},
};


static void do_hall_work(struct work_struct *work)
{
	unsigned int gpio_status;
	wake_lock_timeout(&hall_wakelock, HZ/2);
#ifndef HALL_POWER_PIN
	//gpio_direction_input(hall->irq_gpio);//pbw
#endif
	gpio_status = gpio_get_value(hall->irq_gpio);
	if(!gpio_status) {
		input_report_key(hall_input, HALL_KEYCODE, 0);
		input_sync(hall_input);
	} else {
		input_report_key(hall_input, HALL_KEYCODE, 1);
		input_sync(hall_input);
	}

	printk("%s,gpio_value=%d,report hall key 252\n",__func__,gpio_status);

	enable_irq(hall->irq);
}

static irqreturn_t interrupt_hall_irq(int irq, void *dev)
{
	if (prob_flag==0)
		return IRQ_HANDLED;
	prob_flag = 2;
	disable_irq_nosync(hall->irq);
	queue_work(hall->hall_wq, &hall->hall_work);

	return IRQ_HANDLED;
}

static ssize_t hall_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc = 0;
	rc = gpio_direction_input(hall->irq_gpio);
	if (rc) {
		printk("gpio_direction_input(%d) = %d.", hall->irq_gpio, rc);
	} else {
		printk("hall gpio_direction_input ok irq_gpio=%d",hall->irq_gpio);
	}

	hall_status = gpio_get_value(hall->irq_gpio);
	sprintf(buf,"%d\n", hall_status);//0->enable hall, 1->disable hall
	printk("hall_status_show=%d\n",hall_status);

	return strlen(buf);
}
static ssize_t hall_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	sprintf(buf,"%d\n", hall_power_on);//0->power off, 1->power on
	printk("hall_enable_show=%d\n", hall_power_on);

	return strlen(buf);
}
static ssize_t hall_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 16, &value);

	if(ret < 0) {
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
	}
#ifdef HALL_POWER_PIN
	if ((value>0) && (hall_power_on == 0)) {
		hall_power_on = 1;//power on
		enable_irq(hall->irq);
		pinctrl_select_state(hall_eint_pinctrl, hall_power_en_enable);
	} else if ((value==0) && (hall_power_on == 1)) {
           hall_power_on = 0;//power off
           disable_irq_nosync(hall->irq);
           pinctrl_select_state(hall_eint_pinctrl, hall_power_en_disable);
        }
#endif
	printk("hall_enable_store en=%d\n", hall_power_on);

	return size;
}

static DEVICE_ATTR(hall_status, 0444, hall_status_show, NULL);
static DEVICE_ATTR(hall_enable, 0644, hall_enable_show, hall_enable_store);

extern unsigned int irq_of_parse_and_map(struct device_node *dev, int index);
static int hall_pinctrl_init(struct device *dev)
{
    int ret = 0;
    hall_eint_pinctrl = devm_pinctrl_get(dev);
    if (IS_ERR(hall_eint_pinctrl)) {
        ret = PTR_ERR(hall_eint_pinctrl);
        printk(KERN_CRIT"%s: get hall_eint_pinctrl failed!\n", __func__);
        return ret;
    }
    hall_eint_active = pinctrl_lookup_state(hall_eint_pinctrl, "hall_irq_active");
    if (IS_ERR(hall_eint_active)) {
        ret = PTR_ERR(hall_eint_active);
        printk(KERN_CRIT"%s: get hall_eint_active failed!\n", __func__);
        return ret;
    }
#ifdef HALL_POWER_PIN
    hall_power_en_enable = pinctrl_lookup_state(hall_eint_pinctrl, "hall_power_enable");
    if (IS_ERR(hall_power_en_enable)) {
        ret = PTR_ERR(hall_power_en_enable);
        printk(KERN_CRIT"%s: get hall_power_en_enable failed!\n", __func__);
        return ret;
    }
    hall_power_en_disable = pinctrl_lookup_state(hall_eint_pinctrl, "hall_power_disable");
    if (IS_ERR(hall_power_en_disable)) {
        ret = PTR_ERR(hall_power_en_disable);
        printk(KERN_CRIT"%s: get hall_power_en_disable failed!\n", __func__);
        return ret;
    }
#endif
    return 0;
}

static int hall_probe(struct platform_device *pdev)
{
	
	int rc = 0, gpio;
	struct device_node *node = NULL;	
	enum of_gpio_flags flags;
	hall = kzalloc( sizeof(struct hall_data), GFP_KERNEL);

	if (hall == NULL)
	{
		printk(KERN_INFO"%s:%d Unable to allocate memory\n", __func__, __LINE__);
		return -ENOMEM;
	}
	printk("hall probe begin\n");
#if 1
	rc = hall_pinctrl_init(&pdev->dev);
	if (rc != 0) {
		printk(KERN_CRIT"%s: hall pinctrl init failed!", __func__);
	} else {
		printk(KERN_CRIT"%s: hall pinctrl ok", __func__);
	}

	pinctrl_select_state(hall_eint_pinctrl, hall_eint_active);
#endif
	node = of_find_compatible_node(NULL, NULL, "hall-switch,tct");
	if (node)
	{
		/* Initialize INT pin. */
		gpio = of_get_named_gpio_flags(node, "hall-gpio-irq", 0, &flags);
		printk("hall irq-gpio = %d", gpio);
		if (gpio > 0) {
			hall->irq_gpio = gpio;
		}
		if (!gpio_is_valid(hall->irq_gpio)) {
			printk("hall->irq_gpio(%d) is invalid.",hall->irq_gpio);
			return (-ENODEV);
		}
		rc = gpio_request(hall->irq_gpio, "hall_gpio_int_pin");
		if (rc) {
			printk("gpio_request(%d) = %d.", hall->irq_gpio, rc);
			return rc;
		}else{
			printk("hall gpio_request ok");
		}

		rc = gpio_direction_input(hall->irq_gpio);
		if (rc) {
			printk("gpio_direction_input(%d) = %d.", hall->irq_gpio, rc);
			return rc;
		}else{
			printk("hall gpio_direction_input ok");
		}
		/* Retrieve the IRQ number. */
		hall->irq = gpio_to_irq(hall->irq_gpio);
		if (hall->irq < 0) {
			printk("gpio_to_irq(%d) failed.", hall->irq_gpio);
			return (-EIO);
		} else {
			printk("gpio_to_irq(%d) = %d.", hall->irq_gpio, hall->irq);
		}
		rc = request_irq(hall->irq,interrupt_hall_irq, IRQ_TYPE_EDGE_BOTH, "hall-eint", NULL);
		if (rc)
		{
			rc = -1;
			printk("%s : requesting IRQ error\n", __func__);
			return rc;
		} 
		else
		{
			printk("%s : requesting IRQ %d\n", __func__, hall->irq);
		}
        gpio_direction_input(hall->irq_gpio);
	} else {
		printk("%s : can not find hall eint compatible node\n",  __func__);
	}

	hall->pdev = pdev;
	dev_set_drvdata(&pdev->dev, hall);

	hall_input = input_allocate_device();
	if (!hall_input)
	{
		printk("hall.c: Not enough memory\n");
		return -ENOMEM;
	}

	hall_input->name = "hall_switch";
	input_set_capability(hall_input, EV_KEY, KEY_POWER);
	input_set_capability(hall_input, EV_KEY, HALL_KEYCODE);
	//input_set_capability(hall_input, EV_KEY, HALL_DISABLE);

	rc = input_register_device(hall_input);
	if (rc)
	{
		printk("hall.c: Failed to register device\n");
		return rc;
	}

	hall->hall_wq = create_singlethread_workqueue("hall_wq");
        if (!hall->hall_wq) {
              printk(KERN_CRIT"%s: create thread error!\n", __func__);
        }

	INIT_WORK(&hall->hall_work, do_hall_work);
	enable_irq_wake(hall->irq);
#if 0
        disable_irq_nosync(hall->irq);
#endif
	hall_class= class_create(THIS_MODULE, "hall_switch");
	hall_dev = device_create(hall_class, NULL, 0, NULL, "hall_switch");
	if (IS_ERR(hall_dev))
        	printk( "Failed to create device(hall_dev)!\n");

	if (device_create_file(hall_dev, &dev_attr_hall_status) < 0)
        	printk( "Failed to create device(hall_dev)'s node hall_status!\n");
	if (device_create_file(hall_dev, &dev_attr_hall_enable) < 0)
            printk( "Failed to create device(hall_dev)'s node hall_enable!\n");
	wake_lock_init(&hall_wakelock,WAKE_LOCK_SUSPEND, "hall_wakelock");

        prob_flag=1;

        gpio_direction_input(hall->irq_gpio);
        printk("hall probe completed\n");

	return 0;
}

int hall_remove(struct platform_device *pdev)
{
	hall = platform_get_drvdata(pdev);

	free_irq(hall->irq, pdev);
	wake_lock_destroy(&hall_wakelock);	

	input_unregister_device(hall_input);
	if (hall_input) {
		input_free_device(hall_input);
		hall_input = NULL;
	}

	return 0;
}


static struct platform_driver hall_driver = {
	.probe = hall_probe,
	.remove = hall_remove,
	.driver = {
		   .name = "hall_switch",
		   .owner = THIS_MODULE,
		   .of_match_table = hall_of_match,
	}
};

static int __init hall_init(void)
{
	return platform_driver_register(&hall_driver);
}

static void __init hall_exit(void)
{
	platform_driver_unregister(&hall_driver);
}

late_initcall(hall_init);//module_init
module_exit(hall_exit);
MODULE_AUTHOR("baiwei.peng@tcl.com");
MODULE_LICENSE("GPL");
