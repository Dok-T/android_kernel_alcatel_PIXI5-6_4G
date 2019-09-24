#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/printk.h>
#include <linux/input/mt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include "hall.h"



static struct input_dev *hall_input = NULL;
static struct hall_data *hall = NULL;
static struct class *hall_class = NULL;
static struct device *hall_dev = NULL;
static int hall_status = 1;

extern int mt_get_gpio_in(unsigned long pin);

static void do_hall_work(struct work_struct *work)
{
    unsigned int gpio_code;
    unsigned int gpio_status;
    int err;
    gpio_status = mt_get_gpio_in(hall->irq_gpio | 0x80000000);

    hall_status = gpio_status;
    if(!gpio_status) {
        gpio_code = FLIP_COVER_LOCK;
        err = irq_set_irq_type(hall->irq, IRQ_TYPE_EDGE_RISING);
        printk("%s err = %d\n", __func__, err);

    } else {
        gpio_code = FLIP_COVER_UNLOCK;
        err = irq_set_irq_type(hall->irq, IRQ_TYPE_EDGE_FALLING);
        printk("%s err = %d\n", __func__, err);
    }

    printk("%s,%d,report kry\n", __func__, __LINE__);
    input_report_key(hall_input, gpio_code, 1);
    input_sync(hall_input);
    input_report_key(hall_input, gpio_code, 0);
    input_sync(hall_input);
    enable_irq(hall->irq);
}

static irqreturn_t interrupt_hall_irq(int irq, void *dev)
{
    disable_irq_nosync(hall->irq);
    printk("%s,%d,irq_value = %d\n", __func__, __LINE__,
           mt_get_gpio_in(hall->irq_gpio | 0x80000000));
    queue_work(hall->hall_wq, &hall->hall_work);
    return IRQ_HANDLED;
}

static ssize_t hall_status_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    /* hall_status = mt_get_gpio_in(hall->irq_gpio | 0x80000000);*/
    sprintf(buf, "%d\n", hall_status);
    return strlen(buf);
}

static ssize_t gpio_status_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    int gpio_status = -1;

    gpio_status = mt_get_gpio_in(hall->irq_gpio | 0x80000000);
    sprintf(buf, "%d\n", gpio_status);
    return strlen(buf);
}

static DEVICE_ATTR(hall_status, 0444, hall_status_show, NULL);
static DEVICE_ATTR(gpio_status, 0444, gpio_status_show, NULL);

int hall_probe(struct platform_device *pdev)
{
    int rc = 0;
    struct device_node *node;
    u32 ints[2] = { 0, 0 };
    hall = kzalloc( sizeof(struct hall_data), GFP_KERNEL);

    if (hall == NULL) {
        printk( "%s:%d Unable to allocate memory\n", __func__, __LINE__);
        return -ENOMEM;
    }

    node = of_find_compatible_node(NULL, NULL, "mediatek,hall_switch");

    if (node) {
        of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
        gpio_set_debounce(ints[0], ints[1]);
        hall->irq_gpio = ints[0];
        hall->irq = irq_of_parse_and_map(node,  0);
        rc = request_irq(hall->irq ,  interrupt_hall_irq, IRQ_TYPE_EDGE_FALLING,
                         "hall-eint", NULL);

        if (rc) {
            rc = -1;
            printk("%s : requesting IRQ error\n", __func__);
            return rc;

        } else
            printk("%s : requesting IRQ %d\n", __func__, hall->irq);

    } else {
        printk("%s : can not find hall eint compatible node\n",  __func__);
    }

    hall->pdev = pdev;
    dev_set_drvdata(&pdev->dev, hall);
    hall_input = input_allocate_device();

    if (!hall_input) {
        printk("hall.c: Not enough memory\n");
        return -ENOMEM;
    }

    hall_input->name = "hall_switch";
    input_set_capability(hall_input, EV_KEY, KEY_POWER);
    input_set_capability(hall_input, EV_KEY, FLIP_COVER_LOCK);
    input_set_capability(hall_input, EV_KEY, FLIP_COVER_UNLOCK);
    rc = input_register_device(hall_input);

    if (rc) {
        printk("hall.c: Failed to register device\n");
        return rc;
    }

    hall->hall_wq = create_singlethread_workqueue("hall_wq");
    INIT_WORK(&hall->hall_work, do_hall_work);
    enable_irq_wake(hall->irq);
    hall_class = class_create(THIS_MODULE, "hall_switch");
    hall_dev = device_create(hall_class, NULL, 0, NULL, "hall_switch");

    if (IS_ERR(hall_dev))
        printk("Failed to create device(hall_dev)!\n");

    if (device_create_file(hall_dev, &dev_attr_hall_status) < 0)
        printk("Failed to create device(hall_dev)'s node hall_status!\n");

    if (device_create_file(hall_dev, &dev_attr_gpio_status) < 0)
        printk("Failed to create device(hall_dev)'s node gpio_status!\n");

    printk("hall probe completed\n");
    return 0;
}

int hall_remove(struct platform_device *pdev)
{
    hall = platform_get_drvdata(pdev);
    free_irq(hall->irq, pdev);
    input_unregister_device(hall_input);

    if (hall_input) {
        input_free_device(hall_input);
        hall_input = NULL;
    }

    return 0;
}

static struct of_device_id hall_of_match[] = {
    {.compatible = "mediatek,hall_switch",},
    {},
};

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

module_init(hall_init);
module_exit(hall_exit);
MODULE_LICENSE("GPL");

