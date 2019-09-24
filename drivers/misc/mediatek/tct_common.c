#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/lcd_monitor.h>

#define CONFIG_CUSTOM_KERNEL_HALL_SENSOR

#define BASE_DIR_NAME    "tct_debug"
#define TP_DIR_NAME      "tp"
#define LCD_DIR_NAME     "lcd"
#ifdef CONFIG_CUSTOM_KERNEL_HALL_SENSOR
    #define HALL_DIR_NAME    "hall"
#endif
/*[BUGFIX]-Mod-BEGIN by TCTSZ.(gaoxiang.zou@tcl.com), for fw_ffu  11/22/2016*/

#ifdef CONFIG_MMC_FFU
#define FFU_DIR_NAME      "ffu"
#endif
/*[BUGFIX]-Mod-END by TCTSZ.(gaoxiang.zou@tcl.com),*/

struct ctp_vendor_info {
    char md5[32];
    char name[64];
    u8   id;
};

static struct proc_dir_entry *base;
static struct proc_dir_entry *tp_dir, *lcd_dir;
static struct proc_dir_entry *tp_entry, *lcd_entry, *gesture_entry;
static struct proc_dir_entry *smart_cover_entry, *palm_entry;
static struct proc_dir_entry *gtp_cfg_entry, *gtp_fw_entry, *gtp_id_entry;
static struct class *device_info_class;
static struct device *device_info_dev;
#ifdef CONFIG_CUSTOM_KERNEL_HALL_SENSOR
    static struct proc_dir_entry *hall_dir, *hall_status_entry;
#endif
/*[BUGFIX]-Mod-BEGIN by TCTSZ.(gaoxiang.zou@tcl.com), for fw_ffu  11/22/2016*/

#ifdef CONFIG_MMC_FFU
static struct proc_dir_entry *ffu_dir ,*ffu_entry;
#endif

/*[BUGFIX]-Mod-END by TCTSZ.(gaoxiang.zou@tcl.com),*/


static struct ctp_vendor_info all_ctp_vendor_info[] = {
#ifdef KERNEL_FIRST_TP
    {KERNEL_FIRST_TP, "LWGB1011010_PIXI5104G_FT5526_FW_20161209", 0x87},
#endif
#ifdef KERNEL_SECOND_TP
    {KERNEL_SECOND_TP, "DPT10112-5A6028ACY_PIXI5104G_GT9271_FW_20161216", 0x00},
#endif
#ifdef KERNEL_THIRD_TP
    {KERNEL_THIRD_TP, "CTPXXXXXXXXX", 0xFF},
	
#endif
};

char tp_info[512];
EXPORT_SYMBOL(tp_info);

char lcd_info[512];
EXPORT_SYMBOL(lcd_info);

int smart_cover_flag = 0;
EXPORT_SYMBOL(smart_cover_flag);
void (*smart_cover_flag_charge_notify)(int flag) = NULL;
EXPORT_SYMBOL(smart_cover_flag_charge_notify);

int gesture_wakeup_flag = 0;
EXPORT_SYMBOL(gesture_wakeup_flag);
/*[BUGFIX]-Mod-BEGIN by TCTSZ.(gaoxiang.zou@tcl.com), for fw_ffu  11/22/2016*/

#ifdef CONFIG_MMC_FFU
int ffu_flag = 1;
EXPORT_SYMBOL(ffu_flag);
#endif
/*[BUGFIX]-Mod-END by TCTSZ.(gaoxiang.zou@tcl.com),*/

bool  palm_suspend_flag = false;
EXPORT_SYMBOL(palm_suspend_flag);

unsigned int gtp_cfg_version = 0;
EXPORT_SYMBOL(gtp_cfg_version);

unsigned int gtp_fw_version = 0;
EXPORT_SYMBOL(gtp_fw_version);

u8 gtp_sensor_id = 0;
EXPORT_SYMBOL(gtp_sensor_id);

#ifdef CONFIG_CUSTOM_KERNEL_HALL_SENSOR
    u16 g_hall_status = 1;
    EXPORT_SYMBOL(g_hall_status);
#endif

static ssize_t tp_info_read(struct file *filp, char __user *page,
                            size_t size, loff_t *ppos)
{
    char *ptr = page;
    static bool is_tp_info_loaded = false;
    int i, count = -1, len;
    count = sizeof(all_ctp_vendor_info) / sizeof(all_ctp_vendor_info[0]);

    if (*ppos)
        return 0;

    if (strlen(tp_info) == 0) {
        for (i = 0; i < count; i++) {
            len = strlen(all_ctp_vendor_info[i].name);

            if (len != 0) {
                strncat(tp_info, all_ctp_vendor_info[i].name, len);
            } else {
                strncat(tp_info, "CTPXXXX", 7);
            }

            strncat(tp_info, "_", 1);
            len = strlen(all_ctp_vendor_info[i].md5);

            if (len != 0 && len >= 8) {
                strncat(tp_info, all_ctp_vendor_info[i].md5, 8);
            } else {
                strncat(tp_info, "MD5XXXX", 7);
            }

            if (gtp_sensor_id == all_ctp_vendor_info[i].id)
                strncat(tp_info, "*", 1);

            strncat(tp_info, "\n", 1);
            is_tp_info_loaded = true;
        }
    }

    if (is_tp_info_loaded == false) {
        strncat(tp_info, "UnKown\n", 7);
        is_tp_info_loaded = true;
    }

    ptr += sprintf(ptr, "%s", tp_info);
    *ppos += (ptr - page);
    return (ptr - page);
}

static const struct file_operations tp_info_proc_ops = {
    .owner = THIS_MODULE,
    .read = tp_info_read,
    .write = NULL,
};

#include <lcm_drv.h>
extern char mtkfb_lcm_name[256];
extern LCM_DRIVER *lcm_driver_list[];
extern unsigned int lcm_count;
static char all_disp_lcm_name[256];
static ssize_t lcd_info_read(struct file *filp, char __user *page,
                             size_t size, loff_t *ppos)
{
    int i;
    char md5_lk[128], md5_kernel[128], md5[128];
    char *ptr = page;

    if (*ppos)
        return 0;

    memset(all_disp_lcm_name, 0, sizeof(all_disp_lcm_name));

    for(i = 0; i < lcm_count; i++) {
        memset(md5_lk, 0, sizeof(md5_lk));
        memset(md5_kernel, 0, sizeof(md5_kernel));
/*[BUGFIX]-mod-begin by scdtablet.jinghuang@tcl.com,2016.12.12,3708473*/
/*add the funtion of lcd version for pixi5104g global*/
#ifdef CONFIG_PROJECT_PIXI5104G

#if defined(LK_THIRD_LCD)&&defined(KERNEL_THIRD_LCD)

        if(i == 1) {
            strncpy(md5_lk, LK_THIRD_LCD, 4);
            strncpy(md5_kernel, KERNEL_THIRD_LCD, 4);
            sprintf(md5, "%s%s", md5_lk, md5_kernel);
        }

#endif
#if defined(LK_FOURTH_LCD)&&defined(KERNEL_FOURTH_LCD)

        if(i == 0) {
            strncpy(md5_lk, LK_FOURTH_LCD, 4);
            strncpy(md5_kernel, KERNEL_FOURTH_LCD, 4);
            sprintf(md5, "%s%s", md5_lk, md5_kernel);
        }

#endif
#else

#if defined(LK_FIRST_LCD)&&defined(KERNEL_FIRST_LCD)

        if(i == 0) {
            strncpy(md5_lk, LK_FIRST_LCD, 4);
            strncpy(md5_kernel, KERNEL_FIRST_LCD, 4);
            sprintf(md5, "%s%s", md5_lk, md5_kernel);
        }

#endif
#if defined(LK_SECOND_LCD)&&defined(KERNEL_SECOND_LCD)

        if(i == 1) {
            strncpy(md5_lk, LK_SECOND_LCD, 4);
            strncpy(md5_kernel, KERNEL_SECOND_LCD, 4);
            sprintf(md5, "%s%s", md5_lk, md5_kernel);
        }

#endif
#if defined(LK_FIFTH_LCD)&&defined(KERNEL_FIFTH_LCD)

        if(i == 2) {
            strncpy(md5_lk, LK_FIFTH_LCD, 4);
            strncpy(md5_kernel, KERNEL_FIFTH_LCD, 4);
            sprintf(md5, "%s%s", md5_lk, md5_kernel);
        }

#endif

#if defined(LK_SIXTH_LCD)&&defined(KERNEL_SIXTH_LCD)

        if(i == 2) {
            strncpy(md5_lk, LK_SIXTH_LCD, 4);
            strncpy(md5_kernel, KERNEL_SIXTH_LCD, 4);
            sprintf(md5, "%s%s", md5_lk, md5_kernel);
        }

#endif

/*[BUGFIX]-mod-end by scdtablet.jinghuang@tcl.com*/

#endif
        strcat(all_disp_lcm_name, lcm_driver_list[i]->name);
        strcat(all_disp_lcm_name, "_");
        strcat(all_disp_lcm_name, md5);

        if(!strcmp(mtkfb_lcm_name, lcm_driver_list[i]->name))
            strcat(all_disp_lcm_name, "*");

        strcat(all_disp_lcm_name, "\n");
    }

    //	strcat(all_disp_lcm_name,"\n");
    ptr += sprintf(ptr, "%s", all_disp_lcm_name);
    *ppos += (ptr - page);
    return (ptr - page);
}


static const struct file_operations lcd_info_proc_ops = {
    .owner = THIS_MODULE,
    .read = lcd_info_read,
    .write = NULL,
};

static ssize_t gesture_wakeup_read(struct file *filp, char __user *page,
                                   size_t size, loff_t *ppos)
{
    char *ptr = page;

    if (*ppos)
        return 0;

    ptr += sprintf(ptr, "%d\n", gesture_wakeup_flag);
    *ppos += (ptr - page);
    return (ptr - page);
}

static ssize_t gesture_wakeup_write(struct file *filp, const char __user *page,
                                    size_t size, loff_t *ppos)
{
    char buf[64];
    int val, ret;

    if (size >= sizeof(buf))
        return -EINVAL;

    if (copy_from_user(&buf, page, size))
        return -EFAULT;

    buf[size] = 0;
    ret = kstrtoul(buf, 10, (unsigned long *)&val);

    if (val == 0) {
        gesture_wakeup_flag = 0;
    } else if (val == 1) {
        gesture_wakeup_flag = 1;
    }

    if (ret < 0)
        return ret;

    pr_err("%s gesture wakeup function!\n",
           (gesture_wakeup_flag ? "Enable" : "Disable"));
    return size;
}

static struct file_operations gesture_wakeup_ops = {
    .owner = THIS_MODULE,
    .read = gesture_wakeup_read,
    .write = gesture_wakeup_write,
};
/*[BUGFIX]-Mod-BEGIN by TCTSZ.(gaoxiang.zou@tcl.com), for fw_ffu  11/22/2016*/

#ifdef CONFIG_MMC_FFU
static ssize_t ffu_read(struct file *filp, char __user *page,
                                   size_t size, loff_t *ppos)
{

#if 0
    printk("ffu_read is read now!\n");
    if (copy_to_user(page,&ffu_flag,sizeof(int))) 
    {
        return -EFAULT;
    }
    printk("ffu_read is read: buf=%d\n",*page);

    return *page;

#else

	char *ptr = page;
	
	if (*ppos)
		return 0;
	
	ptr += sprintf(ptr, "%d\n", ffu_flag);
	*ppos += (ptr - page);

	return (ptr - page);

#endif
}

static ssize_t ffu_write(struct file *filp, const char __user *page,
                                    size_t size, loff_t *ppos)
{

#if 0
	printk("ffu_write is write now!\n");
    printk("ffu_write is write: buf=%d\n",*page);

    if (copy_from_user(&ffu_flag,page,sizeof(int))) 
    {
        return -EFAULT;
    }
    return *page;
#else

     char buf[64];
	 int val, ret;
 
	 if (size >= sizeof(buf))
		 return -EINVAL;
 
	 if (copy_from_user(&buf, page, size))
		 return -EFAULT;
 
	 buf[size] = 0;
	 ret = kstrtoul(buf, 10, (unsigned long *)&val);
 
	 if (val == 0) {
		 ffu_flag = 0;
	 } else if (val == 1) {
		 ffu_flag = 1;
	 }
 
	 if (ret < 0)
		 return ret;
 
	 pr_err("%s ffu_flag function!\n",
			(ffu_flag ? "Enable" : "Disable"));
	 return size;

#endif
    
}

static struct file_operations ffu_ops = {
    .owner = THIS_MODULE,
    .read = ffu_read,
    .write = ffu_write,
};
#endif
/*[BUGFIX]-Mod-END by TCTSZ.(gaoxiang.zou@tcl.com),*/

static ssize_t smart_cover_read(struct file *filp, char __user *page,
                                size_t size, loff_t *ppos)
{
    char *ptr = page;

    if (*ppos)
        return 0;

    ptr += sprintf(ptr, "%d\n", smart_cover_flag);
    *ppos += (ptr - page);
    return (ptr - page);
}

static ssize_t smart_cover_write(struct file *filp, const char __user *page,
                                 size_t size, loff_t *ppos)
{
    char buf[64];
    int val, ret;

    if (size >= sizeof(buf))
        return -EINVAL;

    if (copy_from_user(&buf, page, size))
        return -EFAULT;

    buf[size] = 0;
    ret = kstrtoul(buf, 10, (unsigned long *)&val);

    if (val == 0) {
        smart_cover_flag = 0;
    } else if (val == 1) {
        smart_cover_flag = 1;
    }

    if(smart_cover_flag_charge_notify != NULL)
        smart_cover_flag_charge_notify(smart_cover_flag);

    if (ret < 0)
        return ret;

    pr_err("%s smart cover flag function!\n",
           (smart_cover_flag ? "Enable" : "Disable"));
    return size;
}

static struct file_operations smart_cover_ops = {
    .owner = THIS_MODULE,
    .read = smart_cover_read,
    .write = smart_cover_write,
};

static ssize_t palm_suspend_read(struct file *filp, char __user *page,
                                 size_t size, loff_t *ppos)
{
    char *ptr = page;

    if (*ppos)
        return 0;

    ptr += sprintf(ptr, "%d\n", palm_suspend_flag);
    *ppos += (ptr - page);
    return (ptr - page);
}

static ssize_t palm_suspend_write(struct file *filp, const char __user *page,
                                  size_t size, loff_t *ppos)
{
    char buf[64];
    int val, ret;

    if (size >= sizeof(buf))
        return -EINVAL;

    if (copy_from_user(&buf, page, size))
        return -EFAULT;

    buf[size] = 0;
    ret = kstrtoul(buf, 10, (unsigned long *)&val);

    if (val == 0) {
        palm_suspend_flag = false;
    } else if (val == 1) {
        palm_suspend_flag = true;
    }

    if (ret < 0)
        return ret;

    pr_err("%s palm suspend function!\n",
           (palm_suspend_flag ? "Enable" : "Disable"));
    return size;
}

static struct file_operations palm_suspend_ops = {
    .owner = THIS_MODULE,
    .read = palm_suspend_read,
    .write = palm_suspend_write,
};

static ssize_t gtp_cfg_version_read(struct file *filp, char __user *page,
                                    size_t size, loff_t *ppos)
{
    char *ptr = page;

    if (*ppos)
        return 0;

    ptr += sprintf(ptr, "0x%x\n", gtp_cfg_version);
    *ppos += (ptr - page);
    return (ptr - page);
}

static struct file_operations gtp_cfg_ops = {
    .owner = THIS_MODULE,
    .read = gtp_cfg_version_read,
    .write = NULL,
};

static ssize_t gtp_fw_version_read(struct file *filp, char __user *page,
                                   size_t size, loff_t *ppos)
{
    char *ptr = page;

    if (*ppos)
        return 0;

    ptr += sprintf(ptr, "0x%x\n", gtp_fw_version);
    *ppos += (ptr - page);
    return (ptr - page);
}

static struct file_operations gtp_fw_ops = {
    .owner = THIS_MODULE,
    .read = gtp_fw_version_read,
    .write = NULL,
};

static ssize_t gtp_sensor_id_read(struct file *filp, char __user *page,
                                  size_t size, loff_t *ppos)
{
    char *ptr = page;

    if (*ppos)
        return 0;

    ptr += sprintf(ptr, "%x\n", gtp_sensor_id);
    *ppos += (ptr - page);
    return (ptr - page);
}

static struct file_operations gtp_id_ops = {
    .owner = THIS_MODULE,
    .read = gtp_sensor_id_read,
    .write = NULL,
};

#ifdef CONFIG_CUSTOM_KERNEL_HALL_SENSOR
static ssize_t hall_status_read(struct file *filp, char __user *page,
                                size_t size, loff_t *ppos)
{
    char *ptr = page;

    if (*ppos)
        return 0;

    ptr += sprintf(ptr, "%d\n", g_hall_status);
    *ppos += (ptr - page);
    return (ptr - page);
}

static struct file_operations hall_status_ops = {
    .owner = THIS_MODULE,
    .read = hall_status_read,
    .write = NULL,
};
#endif

static int create_device_info_class_and_dev(void)
{
    if (!device_info_class) {
        device_info_class = class_create(THIS_MODULE, "deviceinfo");
        if (IS_ERR(device_info_class)) {
            pr_err("Failed to create class for device info!\n");
            return -1;
        }

        device_info_dev = device_create(device_info_class, NULL, 0, NULL,
                "device_info");
        if (IS_ERR(device_info_dev)) {
            pr_err("Failed to create dev for device info!\n");
            return -2;
        }
    }

    return 0;
}

struct device* get_deviceinfo_dev(void)
{
    if (device_info_dev)
        return device_info_dev;
    return NULL;
}
EXPORT_SYMBOL(get_deviceinfo_dev);

static int __init tct_debug_init(void)
{
    base = proc_mkdir(BASE_DIR_NAME, NULL);
    tp_dir = proc_mkdir(TP_DIR_NAME, base);
    lcd_dir = proc_mkdir(LCD_DIR_NAME, base);
#ifdef CONFIG_CUSTOM_KERNEL_HALL_SENSOR
    hall_dir = proc_mkdir(HALL_DIR_NAME, base);
#endif
/*[BUGFIX]-Mod-BEGIN by TCTSZ.(gaoxiang.zou@tcl.com), for fw_ffu  11/22/2016*/

#ifdef CONFIG_MMC_FFU
	ffu_dir = proc_mkdir(FFU_DIR_NAME, base);
#endif
	/*[BUGFIX]-Mod-END by TCTSZ.(gaoxiang.zou@tcl.com),*/

    tp_entry = proc_create("tp_fw_cfg.ver", 0444, tp_dir, &tp_info_proc_ops);

    if (tp_entry == NULL) {
        pr_err("Create tp proc entry failed!\n");
        goto err_create_tp_info_entry;
    }

    lcd_entry = proc_create("initcode.ver", 0444, lcd_dir, &lcd_info_proc_ops);

    if (lcd_entry == NULL) {
        pr_err("Create lcd proc entry failed!\n");
        goto err_create_lcd_info_entry;
    }

    gesture_entry = proc_create("gesture_enable", 0666, tp_dir,
                                &gesture_wakeup_ops);

    if (gesture_entry == NULL) {
        pr_err("Create gesture enable/disable entry failed!\n");
        goto err_create_gesture_wakeup_entry;
    }
/*[BUGFIX]-Mod-BEGIN by TCTSZ.(gaoxiang.zou@tcl.com), for fw_ffu  11/22/2016*/
#ifdef CONFIG_MMC_FFU	
	ffu_entry = proc_create("ffu_enable", 0666, ffu_dir,
										&ffu_ops);
	if (ffu_entry == NULL) {
        pr_err("Create ffu_entry failed!\n");
        goto err_create_ffu_entry;
    }
#endif
/*[BUGFIX]-Mod-END by TCTSZ.(gaoxiang.zou@tcl.com),*/

    smart_cover_entry = proc_create("smart_cover_enable", 0666, tp_dir,
                                    &smart_cover_ops);

    if (smart_cover_entry == NULL) {
        pr_err("Create smart cover enable/disable entry failed!\n");
        goto err_create_smart_cover_entry;
    }

    palm_entry = proc_create("palm_suspend_enable", 0666, tp_dir,
                             &palm_suspend_ops);

    if (palm_entry == NULL) {
        pr_err("Create palm suspend enable/disable entry failed!\n");
        goto err_create_palm_suspend_entry;
    }

    gtp_cfg_entry = proc_create("cfg_version", 0664, tp_dir,
                                &gtp_cfg_ops);

    if (gtp_cfg_entry == NULL) {
        pr_err("Create gtp cfg version entery failed!\n");
        goto err_create_gtp_cfg_entry;
    }

    gtp_fw_entry = proc_create("firmware_version", 0664, tp_dir,
                               &gtp_fw_ops);

    if (gtp_fw_entry == NULL) {
        pr_err("Create gtp firmware version entery failed!\n");
        goto err_create_gtp_fw_entry;
    }

    gtp_id_entry = proc_create("sensor_id", 0664, tp_dir,
                               &gtp_id_ops);

    if (gtp_id_entry == NULL) {
        pr_err("Create gtp sensor id entry failed!\n");
        goto err_create_gtp_sensor_id_entry;
    }

#ifdef CONFIG_CUSTOM_KERNEL_HALL_SENSOR
    hall_status_entry = proc_create("hall_status", 0664, hall_dir,
                                    &hall_status_ops);

    if (hall_status_entry == NULL) {
        pr_err("Create hall status entry failed!\n");
        goto err_create_hall_status_entry;
    }

#endif

    if (create_device_info_class_and_dev())
        pr_err("Failed to create device info class and dev!\n");

    return 0;
	/*[BUGFIX]-Mod-BEGIN by TCTSZ.(gaoxiang.zou@tcl.com), for fw_ffu  11/22/2016*/

#ifdef CONFIG_MMC_FFU
err_create_ffu_entry:
	proc_remove(ffu_entry);
#endif
/*[BUGFIX]-Mod-END by TCTSZ.(gaoxiang.zou@tcl.com),*/

err_create_hall_status_entry:
    proc_remove(gtp_id_entry);
err_create_gtp_sensor_id_entry:
    proc_remove(gtp_fw_entry);
err_create_gtp_fw_entry:
    proc_remove(gtp_cfg_entry);
err_create_gtp_cfg_entry:
    proc_remove(palm_entry);
err_create_palm_suspend_entry:
    proc_remove(smart_cover_entry);
err_create_smart_cover_entry:
    proc_remove(gesture_entry);
err_create_gesture_wakeup_entry:
    proc_remove(lcd_entry);
err_create_lcd_info_entry:
    proc_remove(tp_entry);
err_create_tp_info_entry:
    proc_remove(lcd_dir);
    proc_remove(tp_dir);
    proc_remove(base);
    return -1;
}

static void __exit tct_debug_exit(void)
{
#ifdef CONFIG_CUSTOM_KERNEL_HALL_SENSOR
    proc_remove(hall_status_entry);
#endif
    proc_remove(gtp_id_entry);
    proc_remove(gtp_fw_entry);
    proc_remove(gtp_cfg_entry);
    proc_remove(gesture_entry);
	/*[BUGFIX]-Mod-BEGIN by TCTSZ.(gaoxiang.zou@tcl.com), for fw_ffu  11/22/2016*/
#ifdef CONFIG_MMC_FFU
	proc_remove(ffu_entry);
#endif
	/*[BUGFIX]-Mod-END by TCTSZ.(gaoxiang.zou@tcl.com),*/

    proc_remove(palm_entry);
    proc_remove(lcd_entry);
    proc_remove(tp_entry);
    proc_remove(lcd_dir);
    proc_remove(tp_dir);
    proc_remove(base);
    return;
}

module_init(tct_debug_init);
module_exit(tct_debug_exit);
MODULE_LICENSE("GPL");

