/*
 *
 * FocalTech TouchScreen driver.
 * 
 * Copyright (c) 2010-2015, Focaltech Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/errno.h>
#include <linux/spi/spi.h>
//#include <mach/gpio.h>
//#include <plat/gpio-cfg.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
//#include <cust_eint.h>
//#include <mach/eint.h>
//#include <mach/mt_gpio.h>
//#include <mach/mt_pm_ldo.h>
#include "focal_fp_sensor_mtk.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
/*create apk debug channel*/
#define PROC_UPGRADE							0
#define PROC_READ_REGISTER					1
#define PROC_WRITE_REGISTER					2
#define PROC_AUTOCLB							4
#define PROC_UPGRADE_INFO						5
#define PROC_WRITE_DATA						6
#define PROC_READ_DATA						7
#define PROC_SET_TEST_FLAG						8
#define PROC_MCU								9
#define FTS_DEBUG_DIR_NAME					"fts_debug"
#define PROC_NAME								"ft-finger-debug"
#define WRITE_BUF_SIZE							1200
#define READ_BUF_SIZE							1200
unsigned char readbuf[READ_BUF_SIZE];
int num_read_chars = 0;
struct spi_device *fts_i2c_client=NULL;
/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/


/*******************************************************************************
* Static variables
*******************************************************************************/
static unsigned char proc_operate_mode 			= PROC_UPGRADE;
static struct proc_dir_entry *fts_proc_entry;
/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
//#if FT_ESD_PROTECT

//#endif
/*******************************************************************************
* Static function prototypes
*******************************************************************************/

static ssize_t fts_debug_write(struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
    unsigned char writebuf[WRITE_BUF_SIZE];

    int buflen = count;
    int writelen = 0;
    int ret = 0,ii=0;
    if (copy_from_user(&writebuf, buff, buflen)) {
        dev_err(&fts_i2c_client->dev, "%s:copy from user error\n", __func__);
        return -EFAULT;
    }

    writelen = buflen-1;
    for(ii=0;ii<writelen;ii++)
        writebuf[ii]=writebuf[ii+1];
    //printk("zax write2 =%x\n",writebuf[1]);
    ret=focal_fp_sensor_fw_rw(g_fp_spi_sensor,  writebuf, writelen);
    if (ret < 0) {
        dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
        return ret;
    }
	
    return count;
}

/* interface of read proc */
/************************************************************************
*   Name: fts_debug_read
*  Brief:interface of read proc
* Input: point to the data, no use, no use, read len, no use, no use 
* Output: page point to data
* Return: read char number
***********************************************************************/
static ssize_t fts_debug_read(struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
    unsigned char writebuf[WRITE_BUF_SIZE];

    int buflen = count;
    int writelen = 0;
    int ret = 0,ii=0;
    if (copy_from_user(&writebuf, buff, buflen)) {
        dev_err(&fts_i2c_client->dev, "%s:copy from user error\n", __func__);
        return -EFAULT;
    }
    proc_operate_mode = writebuf[0];
    switch (proc_operate_mode)
    {
        case PROC_READ_DATA:
            writelen = buflen-1;
            for(ii=0;ii<writelen;ii++)
                writebuf[ii]=writebuf[ii+1];
            ret=focal_fp_sensor_fw_rw(g_fp_spi_sensor,  writebuf, writelen);
            memcpy(readbuf, writebuf, writelen);

            if (ret < 0) {
                dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
                return ret;
            }

            ret = copy_to_user(buff,readbuf, writelen);
            if (ret != 0)
                pr_err("fts_debug_read copy_to_user error!");
            break;
        case PROC_MCU:
            readbuf[0]=getmcustatus();
            printk("zax sss PROC_MCU=%d\n",readbuf[0]);
            ret = copy_to_user(buff,readbuf, 1);
            if (ret != 0)
                pr_err("fts_debug_read copy_to_user error!");
            break;
	default:
		break;
	}
	
	return count;
}

static const struct file_operations fts_proc_fops = {
		.owner 	= THIS_MODULE,
		.read 	= fts_debug_read,
		.write 	= fts_debug_write,
		
};

/************************************************************************
* Name: fts_create_apk_debug_channel_1
* Brief:  create apk debug channel
* Input: i2c info
* Output: no
* Return: success =0
***********************************************************************/
int fts_create_apk_debug_channel_1(struct spi_device *spi)
{
	fts_i2c_client=spi;
	
	fts_proc_entry = proc_create(PROC_NAME, 0777, NULL, &fts_proc_fops);		
	
	if (NULL == fts_proc_entry) 
	{
		dev_err(&spi->dev, "Couldn't create proc entry!\n");
		
		return -ENOMEM;
	} 
	else 
	{
		dev_info(&spi->dev, "Create proc entry success!\n");
		
	}
	return 0;
}
/************************************************************************
* Name: fts_release_apk_debug_channel_1
* Brief:  release apk debug channel
* Input: no
* Output: no
* Return: no
***********************************************************************/
void fts_release_apk_debug_channel_1(void)
{
	
	//if (fts_proc_entry)
	
			//remove_proc_entry(PROC_NAME, NULL);


	if (fts_proc_entry)
		//#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0))
			proc_remove(fts_proc_entry);
		//#else
		//	remove_proc_entry(PROC_NAME, NULL);
		//#endif
	
}


