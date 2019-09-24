/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>


#include <mach/gpio_const.h>
#include <mt_gpio.h>

/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */

#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)

/*#define DEBUG_LEDS_STROBE*/
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#else
#define PK_DBG(a, ...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */


static u32 strobe_Res;
static u32 strobe_Timeus;
static BOOL g_strobe_On;

static int g_duty = -1;
static int g_timeOutTimeMs;

static DEFINE_MUTEX(g_strobeSem);


//#define STROBE_DEVICE_ID 0x30
#define STROBE_DEVICE_I2C_ADDR  0x63
#define I2C_STROBE_MAIN_CHANNEL 2


static struct work_struct workTimeOut;

#define GPIO_CAM_FLASH_EN_PIN     (GPIO20 | 0x80000000)


#define KTD2682_REG_ChipID 0x0c
#define KTD2682_REG_FLASH_Current_Set_LED1 3
#define KTD2682_REG_FLASH_Current_Set_LED2 4
#define KTD2682_REG_TORCH_Current_Set_LED1 5
#define KTD2682_REG_TORCH_Current_Set_LED2 6

#define KTD2682_REG_Flash_Timer 8
#define KTD2682_REG_Control 1
#define KTD2682_REG_Fault1 0xa
#define KTD2682_REG_Fault2 0xb
#define KTD2682_REG_Current_Boost 7
//static int g_bLtVersion;
extern u32 gl_camera_index;

//extern void mtkcam_flash_en_pin_set(int val);
/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);

static struct i2c_client *KTD2682_i2c_client = NULL;

int FL_Init(void);



struct KTD2682_platform_data {
	u8 torch_pin_enable;    // 1:  TX1/TORCH pin isa hardware TORCH enable
	u8 pam_sync_pin_enable; // 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input
	u8 thermal_comp_mode_enable;// 1: LEDI/NTC pin in Thermal Comparator Mode
	u8 strobe_pin_disable;  // 1 : STROBE Input disabled
	u8 vout_mode_enable;  // 1 : Voltage Out Mode enable
};

struct KTD2682_chip_data {
	struct i2c_client *client;

	/* struct led_classdev cdev_flash; */
	/* struct led_classdev cdev_torch; */
	/* struct led_classdev cdev_indicator; */

	struct KTD2682_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};

static int KTD2682_write_reg(struct i2c_client *client, u8 reg, u8 val)
{

	int ret=0;
	struct KTD2682_chip_data *chip;
	if (client == NULL){
		printk("%s error!\n",__func__);
		return -1;
	}
	chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		printk("failed writting at 0x%02x\n", reg);
	return ret;
}

static int KTD2682_read_reg(struct i2c_client *client, u8 reg)
{

	int val=0;
	struct KTD2682_chip_data *chip;
	if (client == NULL){
		printk("%s error!\n",__func__);
		return -1;
	}
	chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);


	return val;
}


static void KTD2682_chip_enable(void)
{
	//mtkcam_flash_en_pin_set(1);
	//mdelay(1);
	if(mt_set_gpio_mode(GPIO_CAM_FLASH_EN_PIN,GPIO_MODE_00)){printk("[constant_flashlight] set gpio mode failed!! \n");}
	if(mt_set_gpio_dir(GPIO_CAM_FLASH_EN_PIN,GPIO_DIR_OUT)){printk("[constant_flashlight] set gpio dir failed!! \n");}
	if(mt_set_gpio_out(GPIO_CAM_FLASH_EN_PIN,GPIO_OUT_ONE)){printk("[constant_flashlight] set gpio failed!! \n");}
	mdelay(1);
}

static void KTD2682_chip_disable(void)
{
	//mtkcam_flash_en_pin_set(0);
	//mdelay(1);
	if(mt_set_gpio_mode(GPIO_CAM_FLASH_EN_PIN,GPIO_MODE_00)){printk("[constant_flashlight] set gpio mode failed!! \n");}
	if(mt_set_gpio_dir(GPIO_CAM_FLASH_EN_PIN,GPIO_DIR_OUT)){printk("[constant_flashlight] set gpio dir failed!! \n");}
	if(mt_set_gpio_out(GPIO_CAM_FLASH_EN_PIN,GPIO_OUT_ZERO)){printk("[constant_flashlight] set gpio failed!! \n");}
	mdelay(1);
}

static int KTD2682_chip_init(struct KTD2682_chip_data *chip)
{

	KTD2682_chip_enable();
	//KTD2682 CHECK DEVICE ID
	if (KTD2682_read_reg(chip->client,KTD2682_REG_ChipID) != 0)
	{
		printk("KTD2682 check device ID error!\n");
	//  chengming.xiang 2015-12-15 Modify :
	//  flashlight ic with lot no '1H/FUAB' can't work during factory test 
	//  the reason is i2c read fail here ,and than cause nop in i2c write operation (i2c write is ok ,only read can't sucess)
	//	KTD2682_chip_disable();
	//	return -1;
	}

	KTD2682_chip_disable();
	return 0;
}

static int KTD2682_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct KTD2682_chip_data *chip;
	struct KTD2682_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	gl_camera_index = 0;
	PK_DBG("cuckoo KTD2682_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		printk(KERN_ERR  "KTD2682 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct KTD2682_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if(pdata == NULL){ //values are set to Zero.
		//printk("KTD2682 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct KTD2682_platform_data),GFP_KERNEL);
		chip->pdata  = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata  = pdata;
	if(KTD2682_chip_init(chip)<0)
		goto err_chip_init;

	KTD2682_i2c_client = client;
	PK_DBG("KTD2682 Initializing is done \n");

	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	printk("KTD2682 probe is failed \n");
	return -ENODEV;
}

static int KTD2682_remove(struct i2c_client *client)
{
	struct KTD2682_chip_data *chip = i2c_get_clientdata(client);

	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}


#define KTD2682_NAME "leds-KTD2682"
static const struct i2c_device_id KTD2682_id[] = {
	{KTD2682_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id KTD2682_of_match[] = {
	{.compatible = "mediatek,strobe_main"},
	{},
};
#endif

static struct i2c_driver KTD2682_i2c_driver = {
	.driver = {
		   .name = KTD2682_NAME,
#ifdef CONFIG_OF
		   .of_match_table = KTD2682_of_match,
#endif
		   },
	.probe = KTD2682_probe,
	.remove = KTD2682_remove,
	.id_table = KTD2682_id,
};

#ifndef CONFIG_OF
struct KTD2682_platform_data KTD2682_pdata = {0, 0, 0, 0, 0};
static struct i2c_board_info __initdata i2c_KTD2682={ I2C_BOARD_INFO(KTD2682_NAME, STROBE_DEVICE_I2C_ADDR), \
													.platform_data = &KTD2682_pdata,};
#endif													

static int __init KTD2682_init(void)
{
	PK_DBG("KTD2682_init\n");
   #ifndef CONFIG_OF
   i2c_register_board_info(I2C_STROBE_MAIN_CHANNEL, &i2c_KTD2682, 1);
   #endif
	return i2c_add_driver(&KTD2682_i2c_driver);
}

static void __exit KTD2682_exit(void)
{
	i2c_del_driver(&KTD2682_i2c_driver);
}


module_init(KTD2682_init);
module_exit(KTD2682_exit);

MODULE_DESCRIPTION("Flash driver for KTD2682");
MODULE_AUTHOR("pw <pengwei@mediatek.com>");
MODULE_LICENSE("GPL v2");

int readReg(int reg)
{

    int val;
    val = KTD2682_read_reg(KTD2682_i2c_client, reg);
	//PK_DBG("cuckoo read reg 0x%x val is:0x%x\n",reg,val);
    return (int)val;
}

int FL_Enable(void)
{
	//mode setting and out on
	
	KTD2682_chip_enable();
	/* main camera pre flash and main flash: g_duty = 1 and 0  sub camera pre flash and main flash: g_duty = 1 and 6*/
	printk("cuckoo g_duty: %d camPOS: %d %s %d\n",g_duty,gl_camera_index,__func__,__LINE__);
	//if(g_duty > 0 && g_duty < 16){
	if(g_duty > 1){
		KTD2682_write_reg(KTD2682_i2c_client,KTD2682_REG_Flash_Timer, 0x1d);   //1d 300ms
		KTD2682_write_reg(KTD2682_i2c_client,KTD2682_REG_FLASH_Current_Set_LED1, (0x6+3*g_duty));  // 328ma - 656ma
	//	KTD2682_write_reg(KTD2682_i2c_client,KTD2682_REG_FLASH_Current_Set_LED2, (0x02+3*g_duty));  //  < 50ma
		KTD2682_write_reg(KTD2682_i2c_client,KTD2682_REG_FLASH_Current_Set_LED2, (3*g_duty));  //  < 50ma
		if (gl_camera_index == 0) 
			KTD2682_write_reg(KTD2682_i2c_client,KTD2682_REG_Control, 0x0d);   //main camera flash mode
		else
			KTD2682_write_reg(KTD2682_i2c_client,KTD2682_REG_Control, 0x0e);   //sub camera flash mode

	}else{
		KTD2682_write_reg(KTD2682_i2c_client,KTD2682_REG_TORCH_Current_Set_LED1, 0x4f);  //  120.5ma
		KTD2682_write_reg(KTD2682_i2c_client,KTD2682_REG_TORCH_Current_Set_LED2, 0x06);  //  < 20ma
		if (gl_camera_index == 0) 
			KTD2682_write_reg(KTD2682_i2c_client,KTD2682_REG_Control, 0x09);   //main camera torch mode and Pre flash
		else
			KTD2682_write_reg(KTD2682_i2c_client,KTD2682_REG_Control, 0x0a);   //sub camera torch mode  and Pre flash

	}
	return 0;
}



int FL_Disable(void)
{

	KTD2682_chip_disable();
	PK_DBG("cuckoo FL_Disable line=%d\n",__LINE__);
	return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG("cuckoo duty: %d FL_dim_duty line=%d\n",duty,__LINE__);
	g_duty = duty;
	if (g_duty > 15)
		g_duty = 15;
	if (g_duty < 0)
		g_duty = 0;
	return 0;
}




int FL_Init(void)
{

	if (KTD2682_i2c_client == NULL){
		printk("ERROR:KTD2682 I2C CLIENT IS NULL !\n");
		return -1;
	}
	PK_DBG(" cuckoo SUB camera index : %d FL_Init line=%d\n",gl_camera_index,__LINE__);
	return 0;
}


int FL_Uninit(void)
{
	PK_DBG(" cuckoo FL_UNInit line=%d\n",__LINE__);
	FL_Disable();
	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	PK_DBG("ledTimeOut_callback\n");
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
	static int init_flag;

	if (init_flag == 0) {
		init_flag = 1;
		INIT_WORK(&workTimeOut, work_timeOutFunc);
		g_timeOutTimeMs = 1000;
		hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		g_timeOutTimer.function = ledTimeOutCallback;
	}
}


static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;

	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
/*	PK_DBG
	    ("LM3642 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",
	     __LINE__, ior_shift, iow_shift, iowr_shift, (int)arg);
*/
	switch (cmd) {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		g_timeOutTimeMs = arg;
		break;


	case FLASH_IOC_SET_DUTY:
		PK_DBG("FLASHLIGHT_DUTY: %d\n", (int)arg);
		FL_dim_duty(arg);
		break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

	case FLASH_IOC_SET_ONOFF:
		PK_DBG("FLASHLIGHT_ONOFF: %d\n", (int)arg);
		if (arg == 1) {

			int s;
			int ms;

			if (g_timeOutTimeMs > 1000) {
				s = g_timeOutTimeMs / 1000;
				ms = g_timeOutTimeMs - s * 1000;
			} else {
				s = 0;
				ms = g_timeOutTimeMs;
			}

			if (g_timeOutTimeMs != 0) {
				ktime_t ktime;

				ktime = ktime_set(s, ms * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
			FL_Enable();
		} else {
			FL_Disable();
			hrtimer_cancel(&g_timeOutTimer);
		}
		break;
	default:
		PK_DBG(" No such command\n");
		i4RetValue = -EPERM;
		break;
	}
	return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;

	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		PK_DBG(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
	PK_DBG(" constant_flashlight_release\n");

	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	PK_DBG(" Done\n");

	return 0;

}


FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{

	gl_camera_index = 0;
	if (pfFunc != NULL){
		*pfFunc = &constantFlashlightFunc;
	}
	return 0;
}
//EXPORT_SYMBOL(constantFlashlightInit);

MUINT32 subStrobeInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{

	gl_camera_index = 1; 
	if (pfFunc != NULL)
	{
		*pfFunc = &constantFlashlightFunc;
	}

	return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

	return 0;
}
EXPORT_SYMBOL(strobe_VDIrq);
