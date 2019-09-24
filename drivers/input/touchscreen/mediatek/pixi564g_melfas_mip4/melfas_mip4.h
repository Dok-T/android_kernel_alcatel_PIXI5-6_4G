/*
 * MELFAS MIP4 Touchscreen
 *
 * Copyright (C) 2015 MELFAS Inc.
 *
 *
 * melfas_mip4.h : Platform data
 *
 */

#ifndef _LINUX_MIP_TOUCH_H
#define _LINUX_MIP_TOUCH_H

#ifdef CONFIG_OF
    #define MIP_USE_DEVICETREE		1
#else
    #define MIP_USE_DEVICETREE		0
#endif

#define MIP_DEVICE_NAME	"mip4_ts"

/**
* Platform Data
*/
struct melfas_platform_data {
    unsigned int max_x;
    unsigned int max_y;

    int gpio_intr;			//Required (interrupt signal)
    int gpio_ce;			//Optional (chip enable, reset)

    //int gpio_vdd_en;		//Optional (power control)

    //int gpio_sda;			//Optional
    //int gpio_scl;			//Optional
};

#endif

