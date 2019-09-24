/*
 * include/linux/spi/spidev.h
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
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

#ifndef __FP_DRV_H
#define __FP_DRV_H

#include <linux/types.h>
#include <linux/spi/spi.h>

#define FP_CONBAT  (1)

///////////////////////////////////////////////////////////////////////////////////////////
struct fp_driver_t {
	char *device_name;
	int (*local_init) (void);
	int (*local_uninit) (void);
	int (*init_ok) (void);
};

extern int fp_driver_load(struct fp_driver_t *drv);
extern int fp_driver_remove(struct fp_driver_t *drv);
///////////////////////////////////////////////////////////////////////////////////////////

#define LOG_TAG  "[fingerprint][fp_drv]:" 
#define __FUN(f)   printk(KERN_ERR LOG_TAG "~~~~~~~~~~~~ %s ~~~~~~~~~\n", __FUNCTION__)
#define klog(fmt, args...)    printk(KERN_ERR LOG_TAG fmt, ##args)
///////////////////////////////////////////////////////////////////////////////////////////


#endif /* __FP_DRV_H */
