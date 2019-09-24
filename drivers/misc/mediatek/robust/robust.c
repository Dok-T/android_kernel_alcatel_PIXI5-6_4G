/* Copyright Statement:
*
* This software/firmware and related documentation ("MediaTek Software") are
* protected under relevant copyright laws. The information contained herein
* is confidential and proprietary to MediaTek Inc. and/or its licensors.
* Without the prior written permission of MediaTek inc. and/or its licensors,
* any reproduction, modification, use or disclosure of MediaTek Software,
* and information contained herein, in whole or in part, shall be strictly prohibited.
*/
/* MediaTek Inc. (C) 2010. All rights reserved.
*
* BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
* THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
* RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
* AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
* NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
* SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
* SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
* THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
* THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
* CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
* SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
* STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
* CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
* AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
* OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
* MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
* The following software/firmware and/or related documentation ("MediaTek Software")
* have been modified by MediaTek Inc. All revisions are subject to any receiver's
* applicable license agreements with MediaTek Inc.
*/

/*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#include <robust.h>
#include <linux/cdev.h>
#define MAX_ROBUST_NODE_NUM MAX_NODE
//extern struct class* get_i2cdevinfo_class(void);
#ifdef CONFIG_CUSTOM_KERNEL_ACCELEROMETER
extern int acc_load_status;
#endif
#ifdef CONFIG_CUSTOM_KERNEL_MAGNETOMETER
extern int mag_init_flag;
#endif
extern int led_initialise_status;
extern int tpd_load_status ;
#ifdef CONFIG_CUSTOM_KERNEL_GYROSCOPE
extern int gyro_init_flag;
#endif
#ifdef CONFIG_CUSTOM_KERNEL_ALSPS
extern int proximity_initialise_status;
#endif

#ifdef CONFIG_CUSTOM_KERNEL_GYROSCOPE
static ssize_t  gyroscope_status_show(struct device *dev, struct device_attribute *attr, char *buf){
	int status=0;

	if(gyro_init_flag == 1)
		status = 1;

	if (!dev ) {
		return 0;
	}
	printk("gyroscope_status_show: %d \n",status);
	return snprintf(buf, PAGE_SIZE, "%d\n", status);
}

static ssize_t gyroscope_status_store(struct device* dev, struct device_attribute *attr, const char *buf, size_t count){
	return count;
}
#endif

#ifdef CONFIG_CUSTOM_KERNEL_ACCELEROMETER
static ssize_t  accelerometer_status_show(struct device *dev, struct device_attribute *attr, char *buf){
	if (!dev ) {
		return 0;
	}
	printk("accelerometer_status_show %d \n",acc_load_status);
	return snprintf(buf, PAGE_SIZE, "%d\n", acc_load_status);
}

static ssize_t accelerometer_status_store(struct device* dev, struct device_attribute *attr, const char *buf, size_t count){
	return count;
}
#endif

#ifdef CONFIG_CUSTOM_KERNEL_MAGNETOMETER
static ssize_t  magnetometer_status_show(struct device *dev, struct device_attribute *attr, char *buf){
	int status=0;

	if(mag_init_flag  == 1)
		status = 1;

	if (!dev ) {
		return 0;
	}
	printk("magnetometer_status_show %d \n",status);
	return snprintf(buf, PAGE_SIZE, "%d\n", status);
}

static ssize_t magnetometer_status_store(struct device* dev, struct device_attribute *attr, const char *buf, size_t count){
	return count;
}
#endif

static ssize_t  led_status_show(struct device *dev, struct device_attribute *attr, char *buf){
	int status=0;

	if(led_initialise_status == 1)
		status = 1;

	if (!dev ) {
		return 0;
	}
	printk("led_status_show: %d \n",status);
	return snprintf(buf, PAGE_SIZE, "%d\n", status);
}

static ssize_t led_status_store(struct device* dev, struct device_attribute *attr, const char *buf, size_t count){
	return count;
}

static ssize_t  touchpanel_status_show(struct device *dev, struct device_attribute *attr, char *buf){
	if (!dev ) {
		return 0;
	}

	printk("touchpanel_status_show: %d \n",tpd_load_status);
	return snprintf(buf, PAGE_SIZE, "%d\n", tpd_load_status);
}

static ssize_t touchpanel_status_store(struct device* dev, struct device_attribute *attr, const char *buf, size_t count){
	return count;
}

#ifdef CONFIG_CUSTOM_KERNEL_ALSPS
static ssize_t  proximity_status_show(struct device *dev, struct device_attribute *attr, char *buf){
	int status=0;

	if(proximity_initialise_status  == 1)
		status = 1;

	if (!dev ){
		return 0;
	}

	printk("proximity_status_show: %d \n",status);
	return snprintf(buf, PAGE_SIZE, "%d\n", status);
}

static ssize_t proximity_status_store(struct device* dev, struct device_attribute *attr, const char *buf, size_t count){
	return count;
}

static ssize_t  light_status_show(struct device *dev, struct device_attribute *attr, char *buf){
	int status=0;

	if(proximity_initialise_status  == 1)
		status = 1;

	if (!dev ) {
		return 0;
	}

	printk("light_status_show: %d \n",status);
	return snprintf(buf, PAGE_SIZE, "%d\n", status);
}

static ssize_t light_status_store(struct device* dev, struct device_attribute *attr, const char *buf, size_t count){
	return count;
}
#endif

#if 0
static ssize_t  smartpa_status_show(struct device *dev, struct device_attribute *attr, char *buf){
	int status=0;

	if(1  == 1)
		status = 1;

	if (!dev ) {
		return 0;
	}
	printk("smartpa_status_show: %d \n",status);
	return snprintf(buf, PAGE_SIZE, "%d\n", status);
}

static ssize_t smartpa_status_store(struct device* dev, struct device_attribute *attr, const char *buf, size_t count){
	return count;
}

static ssize_t  hifi_status_show(struct device *dev, struct device_attribute *attr, char *buf){
	int status=0;

	if(1  == 1)
		status = 1;

	if (!dev ) {
		return 0;
	}
	printk("hifi_status_show: %d \n",status);
	return snprintf(buf, PAGE_SIZE, "%d\n", status);
}

static ssize_t hifi_status_store(struct device* dev, struct device_attribute *attr, const char *buf, size_t count){
	return count;
}
#endif
static const robust_node_info robust_node[MAX_ROBUST_NODE_NUM] = {
#ifdef CONFIG_CUSTOM_KERNEL_GYROSCOPE
	{"GyroscopeSensor" , "status", gyroscope_status_show , gyroscope_status_store},
#endif
#ifdef CONFIG_CUSTOM_KERNEL_ACCELEROMETER
	{"AccelerometerSensor" , "status", accelerometer_status_show , accelerometer_status_store},
#endif
#ifdef CONFIG_CUSTOM_KERNEL_MAGNETOMETER
	{"MagnetometerSensor" , "status", magnetometer_status_show , magnetometer_status_store},
#endif
	{"indicator_led" , "status", led_status_show , led_status_store},
	{"touch_panel" , "status", touchpanel_status_show , touchpanel_status_store},
#ifdef CONFIG_CUSTOM_KERNEL_ALSPS
	{"ProximitySensor" , "status", proximity_status_show , proximity_status_store},
	{"LightSensor" , "status", light_status_show , light_status_store},
#endif
//	{"audio_smpart_pa" , "status", smartpa_status_show , smartpa_status_store},
//	{"audio_hifi" , "status", hifi_status_show , hifi_status_store},
};

struct device_attribute dev_attr[MAX_ROBUST_NODE_NUM];
static const struct file_operations cdevOper = {
	.owner = THIS_MODULE,
};
static struct class *probustClass;
static dev_t midia1_devno;
static dev_t midia2_devno;
static int __init robust_init(void){
	int i , retval = 1;
	int err = 0;
	struct class *i2cdev_class = NULL;
	struct device *i2cdev_dev = NULL;
	//static struct cdev *cdev;

	ROBUST_FUN;

	//i2c check
	i2cdev_class = class_create(THIS_MODULE, "i2c_check");
	if (IS_ERR(i2cdev_class)){
		printk("Failed to create class(sys/class/i2c_check/)!\n");
		return 0;
	}

	for(i = 0 ; i < sizeof(robust_node)/sizeof(robust_node_info) ; i++){
		i2cdev_dev = device_create(i2cdev_class,NULL, 0, NULL, robust_node[i].dir);
		if (IS_ERR(i2cdev_dev)){
			pr_err("Failed to create device(sys/class/i2c_check/%s)!\n" , robust_node[i].dir);
			retval = 0;
			break;
		}

		dev_attr[i].attr.name = robust_node[i].node ;
		dev_attr[i].attr.mode = S_IWUSR | S_IRUGO ;
		dev_attr[i].show = robust_node[i].show;
		dev_attr[i].store = robust_node[i].store;
		if(device_create_file(i2cdev_dev, &dev_attr[i]) < 0){
			pr_err("Failed to create device file(sys/class/i2c_check/%s/%s)!\n" , robust_node[i].dir , robust_node[i].node);
			retval = 0;
			break;
		}
	}

	//camera check
	probustClass = class_create(THIS_MODULE, "robust");

	if(1 == 1){
		err = alloc_chrdev_region(&midia1_devno, 0, 1, "media1");
		if (err < 0) {
			pr_err("%s:alloc_chrdev_region failed  error code=%d\n", __func__, err);
			return err;
		}
		device_create(probustClass, NULL, midia1_devno, NULL, "media1");
	}

	if(1 == 1){
		err = alloc_chrdev_region(&midia2_devno, 0, 1, "media2");
		if (err < 0) {
			pr_err("%s:alloc_chrdev_region failed  error code=%d\n", __func__, err);
			return err;
		}
		device_create(probustClass, NULL, midia2_devno, NULL, "media2");
	}
#if 0
	cdev = cdev_alloc();
	if (!cdev) {
		err = -ENOMEM;
		unregister_chrdev_region(devno, 1);
		pr_err("%s:media_drv_major failed\n", __func__);
		return err;
	}

	cdev_init(cdev, &cdevOper);

	cdev->owner = THIS_MODULE;
	err = cdev_add(cdev, devno, 1);
	if (err)
		pr_err("media_drv_major %d adding LED%d", err, err);
#endif

	return retval;
}
/*----------------------------------------------------------------------------*/

static void __exit robust_exit(void){
	ROBUST_FUN;
}
/*----------------------------------------------------------------------------*/
module_init(robust_init);
module_exit(robust_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("ersen.shang");
MODULE_DESCRIPTION("robust driver");
MODULE_LICENSE("GPL");
