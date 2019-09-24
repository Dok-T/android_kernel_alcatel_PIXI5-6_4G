/*
 * Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <asm/io.h>
#include <linux/proc_fs.h>
#include <linux/cdev.h>

#include "fp_drv.h"


#define MAX_DRV_NUM (3)
//#define GF_DEV_NAME "goodix_fp"
#define SL_DEV_NAME  "silead_fp_dev"
#define FT_DEV_NAME  "focal_fp"
/*[BUGFIX]-Mod-BEGIN by SCDTABLET.jinghuang@tcl.com,06/15/2016,2354741*/
/*compatible for fingerprint.*/

extern int select_hw_configuration_func(void);

///////////////////////////////////////////////////////////////////
static int fp_probe(struct platform_device *pdev);
static int fp_remove(struct platform_device *pdev);
///////////////////////////////////////////////////////////////////

static struct platform_driver fp_driver = {
	.probe = fp_probe,
	.remove = fp_remove,
	.driver = {
		   .name = "fp_drv",
	},
};

struct platform_device fp_device = {
    .name   	= "fp_drv",
    .id        	= -1,
};

static struct fp_driver_t *g_fp_drv = NULL;
static struct fp_driver_t fp_driver_list[MAX_DRV_NUM];

int fp_driver_load(struct fp_driver_t *drv)
{
	int i;
	int ret=-1;

	__FUN();

	if((drv == NULL) ||(g_fp_drv != NULL))
	{
		return -1;
	}
/*[BUGFIX]-Mod-BEGIN by ke.huang.sz,3303757*/
/*don't load fignerprint driver on lamata board*/
	ret =select_hw_configuration_func();
	if(ret==2||ret==3||ret==6) //add ret=6 for pixi564g new pa compatible
		{
			printk("[%s]:hw configuration LAMATA,exit\n",__func__);
			return -EINVAL;
		}
/*[BUGFIX]-Mod-END by ke.huang.sz*/
	for(i = 0; i < MAX_DRV_NUM; i++)
	{
		if(fp_driver_list[i].device_name == NULL)
		{
			fp_driver_list[i].device_name = drv->device_name;
			fp_driver_list[i].local_init = drv->local_init;
			fp_driver_list[i].local_uninit = drv->local_uninit;
			fp_driver_list[i].init_ok = drv->init_ok;
			printk("---Add [%s] to list-----\n", fp_driver_list[i].device_name);
		}

		if (strcmp(fp_driver_list[i].device_name, drv->device_name) == 0) {
			break;
		}
	}

	return 0;
}

int fp_driver_remove(struct fp_driver_t *drv)
{
	int i = 0;
	
	__FUN();
	
	if (drv == NULL) {
		return -1;
	}
	
	for (i = 0; i < MAX_DRV_NUM; i++) 
	{
		if(strcmp(fp_driver_list[i].device_name, drv->device_name) == 0) {
			memset(&fp_driver_list[i], 0, sizeof(struct fp_driver_t));
			printk("remove drv:%s\n", drv->device_name);
			break;
		}
	}
	
	return 0;
}

static ssize_t info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	char fp_dev_info[32] = "NULL:NULL:NULL:NULL";

/* [PLATFORM]-Removed-BEGIN by TCTSZ.lizhi.wu, 2016.07.27*/
#if 0
	/*[BUGFIX]-Mod-BEGIN by SCDTABLET.jinghuang@tcl.com,06/15/2016,2354741*/
/*compatible for fingerprint.*/
//	#ifdef  CONFIG_POP464G
	if (FP_FLG==1){
		sprintf(fp_dev_info, "%s:%s:%s:%s", "goodix", "GC3208", "SUP1", "NULL");
		return sprintf(buf, "%s\n", fp_dev_info);
	}
	else{
		sprintf(fp_dev_info, "%s:%s:%s:%s", "N0", "NULL", "NULL", "NULL"); 
		return sprintf(buf, "%s\n", fp_dev_info);
	}
//	#else
#endif
/* [PLATFORM]-Removed-End by TCTSZ.lizhi.wu, 2016.07.27*/
	if (g_fp_drv == NULL)
	{
		sprintf(fp_dev_info, "%s:%s:%s:%s", "N0", "NULL", "NULL", "NULL"); 
		return sprintf(buf, "%s\n", fp_dev_info);
	}

    if (strcmp(g_fp_drv->device_name, SL_DEV_NAME) == 0)  //silead fignerprint
	{
		sprintf(fp_dev_info, "%s:%s:%s:%s", "silead", "6163S", "SUP2", "NULL");
	} 
	else if (strcmp(g_fp_drv->device_name, FT_DEV_NAME) == 0) //focal fignerprint
    {
		sprintf(fp_dev_info, "%s:%s:%s:%s", "focal", "FT9338", "SUP3", "NULL");
    }
	return sprintf(buf, "%s\n", fp_dev_info);  
//	#endif
	/*[BUGFIX]-Mod-END by SCDTABLET.jinghuang@tcl.com*/ 
}

DEVICE_ATTR(FP, S_IRUGO, info_show, NULL);
extern struct device *get_deviceinfo_dev(void);
static int create_fp_node(void)
{
	struct device *fp_dev;
	fp_dev = get_deviceinfo_dev();
	if (device_create_file(fp_dev, &dev_attr_FP) < 0)
	{
		printk("Failed to create fp node.\n");
	}
	return 0;
}

static int fp_probe(struct platform_device *pdev)
{
	int i;
	int found = 0;
	__FUN();

	for(i = 0; i < MAX_DRV_NUM; i++)
	{
		if(fp_driver_list[i].device_name)
		{
			printk("------Start drv init :[%s]------\n", fp_driver_list[i].device_name);
			fp_driver_list[i].local_init();
			msleep(200);
			if(fp_driver_list[i].init_ok())
			{
				found = 1;
				g_fp_drv = &fp_driver_list[i];
				break;
			}
		
			fp_driver_list[i].local_uninit();
		}
	}

	if(!found)
	{
		printk("#########Not match any fp-spi device, please check driver !!!#######\n");
	}

	create_fp_node();

	return 0;
}

static int fp_remove(struct platform_device *pdev)
{
	__FUN();
	device_remove_file(&pdev->dev, &dev_attr_FP);
	return 0;
}

static int __init fp_drv_init(void)
{
	__FUN();

	if (platform_device_register(&fp_device) != 0) {
		printk( "device_register fail!.\n");
		return -1;
	
	}
	
	if (platform_driver_register(&fp_driver) != 0) {
		printk( "driver_register fail!.\n");
		return -1;
	}
	
	return 0;
}

static void __exit fp_drv_exit(void)
{
	__FUN();
	platform_driver_unregister(&fp_driver);
}

late_initcall(fp_drv_init);
module_exit(fp_drv_exit);

