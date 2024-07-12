

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/platform_device.h>

#include <linux/proc_fs.h>
#include <linux/printk.h>
#include <linux/uaccess.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

static const unsigned char kbd_keycode[256] = {
	  0,   0,   0,   0,  30,  48,  46,  32,  18,  33,  34,  35,  23,  36,  37,  38, //00 ~ 0F
	 50,  49,  24,  25,  16,  19,  31,  20,  22,  47,  17,  45,  21,  44,   2,   3, //10 ~ 1F
	  4,   5,   6,   7,   8,   9,  10,  11,  28,   1,  14,  15,  57,  12,  13,  26, //20 ~ 2F
	 27,  43,  43,  39,  40,  41,  51,  52,  53,  58,  59,  60,  61,  62,  63,  64, //30 ~ 3F
	 65,  66,  67,  68,  87,  88,  99,  70, 119, 110, 102, 104, 111, 107, 109, 106, //40 ~ 4F
	105, 108, 103,  69,  98,  55,  74,  78,  96,  79,  80,  81,  75,  76,  77,  71, //50 ~ 5F
	 72,  73,  82,  83,  86, 127, 116, 117, 183, 184, 185, 186, 187, 188, 189, 190, //60 ~ 6F
	191, 192, 193, 194, 134, 138, 130, 132, 128, 129, 131, 137, 133, 135, 136, 113, //70 ~ 7F
	115, 114,   0,   0,   0, 121,   0,  89,  93, 124,  92,  94,  95,   0,   0,   0, //80 ~ 8F
	122, 123,  90,  91,  85, 172, 141, 224, 225,   0,   0,   0,   0,   0,   0,   0, //90 ~ 9F
	  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, //A0 ~ AF
	  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, //B0 ~ BF
	  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, //C0 ~ CF
	  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, //D0 ~ DF
	 29,  42,  56, 125,  97,  54, 100, 126, 164, 166, 165, 163, 161, 115, 114, 113, //E0 ~ EF
	150, 158, 159, 128, 136, 177, 178, 176, 142, 152, 173, 140, 227, 228,   0,   0  //F0 ~ FF
};


struct serial_kbd {
    struct platform_device *pdev;
	struct input_dev *dev;
	struct mutex kbd_lock;
	
	int kbd_connect_state;
	
	unsigned char old[8];
	unsigned char *new;
};

struct serial_kbd g_kbd;

static void kbd_report(struct serial_kbd *kbd)
{
	int i;

	for (i = 0; i < 8; i++)
		input_report_key(kbd->dev, kbd_keycode[i + 224], (kbd->new[0] >> i) & 1);

	for (i = 2; i < 8; i++) {
		if (kbd->old[i] > 3 && memscan(kbd->new + 2, kbd->old[i], 6) == kbd->new + 8) {
			if (kbd_keycode[kbd->old[i]])	
				input_report_key(kbd->dev, kbd_keycode[kbd->old[i]], 0);
			else
				printk("Unknown key (scancode %#x) released.\n", kbd->old[i]);
		}

		if (kbd->new[i] > 3 && memscan(kbd->old + 2, kbd->new[i], 6) == kbd->old + 8) {
			if (kbd_keycode[kbd->new[i]])
				input_report_key(kbd->dev, kbd_keycode[kbd->new[i]], 1);
			else
				printk("Unknown key (scancode %#x) pressed.\n", kbd->new[i]);
		}
	}

	input_sync(kbd->dev);

	memcpy(kbd->old, kbd->new, 8);
}


int kbd_register_device(void)
{
	struct input_dev *input_dev;
	int error = -ENOMEM;
	int i;

    mutex_lock(&g_kbd.kbd_lock);

    if(g_kbd.kbd_connect_state == 1)
        return 0;

	input_dev = input_allocate_device();
	if (!input_dev)
		goto fail;

	input_dev->name = "Serial HID";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x3000;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0001;
	input_dev->dev.parent = &g_kbd.pdev->dev;	
	
	input_set_drvdata(input_dev, &g_kbd);

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REP);

	for (i = 0; i < 255; i++)
		set_bit(kbd_keycode[i], input_dev->keybit);
		
	clear_bit(0, input_dev->keybit);

    g_kbd.dev = input_dev;

    input_register_device(g_kbd.dev);
    g_kbd.kbd_connect_state = 1;

    mutex_unlock(&g_kbd.kbd_lock);

	return 0;

fail:	
	input_free_device(input_dev);
	return error;
}

void kbd_unregister_device(void)
{
    mutex_lock(&g_kbd.kbd_lock);
    if (g_kbd.dev && (g_kbd.kbd_connect_state == 1)) {
		input_unregister_device(g_kbd.dev);
		g_kbd.kbd_connect_state = 0;
		memset(g_kbd.old, 0, 8);
	}
	mutex_unlock(&g_kbd.kbd_lock);
}


void kbd_connect(int enable)
{
    if(enable && g_kbd.pdev)
    {
        kbd_register_device();
        printk("register kbd\n");
    }
    else
    {
        kbd_unregister_device();
        printk("unregister kbd\n");
    }
}

static ssize_t kbd_connect_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned long status;

    kstrtoul(buf, 10, &status);

    if(status)
        kbd_connect(1);
    else
        kbd_connect(0);

	return size;
}

static ssize_t kbd_connect_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{ 
	return sprintf(buf, "kbd connect %d\n", g_kbd.kbd_connect_state);
}
static DEVICE_ATTR_RW(kbd_connect);

static struct attribute *kbd_attrs[] = {
	&dev_attr_kbd_connect.attr,
	NULL,
};
ATTRIBUTE_GROUPS(kbd);

static int kbd_probe(struct platform_device *pdev)
{

    g_kbd.pdev = pdev;
    mutex_init(&g_kbd.kbd_lock);

    sysfs_create_groups(&pdev->dev.kobj, kbd_groups);
	printk(KERN_ERR "tct_serial_keyboard: Probe end\n");

	return 0;
}

static const struct of_device_id kbd_of_match[] = {
	{.compatible = "tct,serial-keyboard"},
	{},
};

static struct platform_driver tct_kbd = {
	.probe = kbd_probe,
	.driver = {
		   .name = "tct-serial-keyboard",
		   .owner = THIS_MODULE,
		   .of_match_table = kbd_of_match,
		   },
};

module_platform_driver(tct_kbd);

MODULE_AUTHOR("chen-liang");
MODULE_DESCRIPTION("TCT serial Keyboard Driver");
MODULE_LICENSE("GPL");

static tct_serial_hid_show(struct seq_file *m, void *v)
{
	return 0;
}

static int tct_serial_hid_open(struct inode *inode, struct file *file)
{
	return single_open(file, tct_serial_hid_show, inode->i_private);
}

static ssize_t tct_serial_hid_write(struct file *filp, const char *ubuf, size_t cnt, loff_t *data)
{
	char buf[64];

	if (cnt >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(&buf, ubuf, cnt))
		return -EFAULT;

	buf[cnt] = 0;

    printk("kbd reveiced data(%d)(%x %x %x %x %x %x %x %x).\n", cnt, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);

    g_kbd.new = buf;
	kbd_report(&g_kbd);
	
	return cnt;
}

static const struct file_operations tct_serial_hid_fops = {
	.open = tct_serial_hid_open,
	.write = tct_serial_hid_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init init_tct_serial_hid_ctrl(void)
{
	struct proc_dir_entry *pe;

	pe = proc_create("tct_serial_hid", 0664, NULL, &tct_serial_hid_fops);
	if (!pe)
		return -ENOMEM;
	return 0;
}

device_initcall(init_tct_serial_hid_ctrl);

