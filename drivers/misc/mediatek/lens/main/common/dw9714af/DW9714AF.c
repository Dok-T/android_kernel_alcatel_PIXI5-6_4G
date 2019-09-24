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

/*
 * DW9714AF voice coil motor driver
 *
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#include "lens_info.h"


#define AF_DRVNAME "DW9714AF_DRV"
#define AF_I2C_SLAVE_ADDR        0x18

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...) pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

#ifdef lkl
   #undef lkl
#endif

static struct i2c_client *g_pstAF_I2Cclient;
static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;


extern unsigned long g_u4AF_ov8856_INF;
extern unsigned long g_u4AF_ov8856_MACRO;
static unsigned long g_u4AF_INF;
static unsigned long g_u4AF_MACRO = 1023;
static unsigned long g_u4TargetPosition;
static unsigned long g_u4CurrPosition;

static int s4AF_ReadReg(unsigned short *a_pu2Result)
{

    int  i4RetValue = 0;                                                                                                                                                 
    char pBuff[2];

    i4RetValue = i2c_master_recv(g_pstAF_I2Cclient, pBuff , 2);

    if (i4RetValue < 0)
    {
        printk("I2C read failed!! \n");
        return -1;
    }

    *a_pu2Result = (((u16)pBuff[0]) << 4) + (pBuff[1] >> 4);

    return 0;

}

static int s4AF_WriteReg(u16 a_u2Data)
{

    int  i4RetValue = 0;
    //chengming.xiang 2015-12-03 Add Dw9714 LSC 
    char puSendCmd[2] = {(char)(a_u2Data >> 4) , (char)((((a_u2Data & 0xF) << 4) & 0xF0) | 0x05)};
    //char puSendCmd[2] = {(char)(a_u2Data >> 4) , (char)((a_u2Data & 0xF) << 4)};
    g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;
    g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;
    i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 2);
    if (i4RetValue < 0)
    {
        printk("%s I2C send failed!! \n",__func__);
        return -1;
    }

    return 0;

}

static int dw9714_WriteReg(u8 Data1 ,u8 Data2)                                                                                                                           
{
    int  i4RetValue = 0;

    char puSendCmd[2] = {Data1 , Data2};

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;
	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;
    //LOG_INF("g_sr %d, write %d \n", g_sr, a_u2Data);
    //g_pstAF_I2Cclient->ext_flag |= I2C_A_FILTER_MSG;
    i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 2);

    if (i4RetValue < 0)
    {
        printk("%s I2C send failed!! \n",__func__);
        return -1;
    }

    return 0;
}


static inline int getAFInfo(__user stAF_MotorInfo *pstMotorInfo)
{
	stAF_MotorInfo stMotorInfo;

	stMotorInfo.u4MacroPosition = g_u4AF_MACRO;
	stMotorInfo.u4InfPosition = g_u4AF_INF;
	stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
	stMotorInfo.bIsSupportSR = 1;

	stMotorInfo.bIsMotorMoving = 1;

	if (*g_pAF_Opened >= 1)
		stMotorInfo.bIsMotorOpen = 1;
	else
		stMotorInfo.bIsMotorOpen = 0;
#ifdef lkl
    printk("lkl getAFInfo stMotorInfo.u4MacroPosition=%d  stMotorInfo.u4InfPosition=%d  stMotorInfo.u4CurrentPosition=%d\n",stMotorInfo.u4MacroPosition,stMotorInfo.u4InfPosition,stMotorInfo.u4CurrentPosition);
#endif
	if (copy_to_user(pstMotorInfo, &stMotorInfo, sizeof(stAF_MotorInfo)))
		LOG_INF("copy to user failed when getting motor information\n");

	return 0;
}

static inline int moveAF(unsigned long a_u4Position)
{
	int ret = 0;
#ifdef lkl
    printk("lkl moveAF a_u4Position=%ld",a_u4Position);
#endif
	if ((a_u4Position > g_u4AF_MACRO) || (a_u4Position < g_u4AF_INF)) {
		LOG_INF("out of range\n");
		return -EINVAL;
	}

	if (*g_pAF_Opened == 1) {
		unsigned short InitPos;

		ret = s4AF_ReadReg(&InitPos);

		if (ret == 0) {
			LOG_INF("Init Pos %6d\n", InitPos);

			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = (unsigned long)InitPos;
			spin_unlock(g_pAF_SpinLock);

		} else {
			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = 0;
			spin_unlock(g_pAF_SpinLock);
		}

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 2;
		spin_unlock(g_pAF_SpinLock);
	}

	if (g_u4CurrPosition == a_u4Position)
		return 0;

	spin_lock(g_pAF_SpinLock);
	g_u4TargetPosition = a_u4Position;
	spin_unlock(g_pAF_SpinLock);

	// LOG_INF("move [curr] %ld [target] %ld\n", g_u4CurrPosition, g_u4TargetPosition);


	if (s4AF_WriteReg((unsigned short)g_u4TargetPosition) == 0) {
		spin_lock(g_pAF_SpinLock);
		g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
		spin_unlock(g_pAF_SpinLock);
	} else {
		LOG_INF("set I2C failed when moving the motor\n");
	}

	return 0;
}

static inline int setAFInf(unsigned long a_u4Position)
{
#ifdef lkl
    printk("lkl setAFInf a_u4Position=%ld",a_u4Position);
#endif
	spin_lock(g_pAF_SpinLock);
	g_u4AF_INF = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

static inline int setAFMacro(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_MACRO = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

/* ////////////////////////////////////////////////////////////// */
long DW9714AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case AFIOC_G_MOTORINFO:
		i4RetValue = getAFInfo((__user stAF_MotorInfo *) (a_u4Param));
		break;

	case AFIOC_T_MOVETO:
		i4RetValue = moveAF(a_u4Param);
		break;

	case AFIOC_T_SETINFPOS:
		i4RetValue = setAFInf(a_u4Param);
		break;

	case AFIOC_T_SETMACROPOS:
		i4RetValue = setAFMacro(a_u4Param);
		break;

	default:
		LOG_INF("No CMD\n");
		i4RetValue = -EPERM;
		break;
	}

	return i4RetValue;
}

static void DW9714AF_reduce_lens_power_down_noise(void)
{
#if 0
        if (g_u4CurrPosition > 300){
                s4AF_WriteReg(300);
                mdelay(15);
                s4AF_WriteReg(250);
                mdelay(15);
                s4AF_WriteReg(200);
                mdelay(15);
                s4AF_WriteReg(150);
                mdelay(15);
                s4AF_WriteReg(100);
                mdelay(15);
                s4AF_WriteReg(50);
                mdelay(15);
        }else{
                s4AF_WriteReg(200);
                mdelay(15);
                s4AF_WriteReg(150);
                mdelay(15);
                s4AF_WriteReg(100);
                mdelay(15);
                s4AF_WriteReg(50);
                mdelay(15);
        }
#endif
}


static void DW9714AF_Init(void)
{
	//printk("cuckoo %s\n",__func__);
    dw9714_WriteReg(0xEC ,0xA3);
    dw9714_WriteReg(0xA1 ,0x05);                                                                                                                                         
    dw9714_WriteReg(0xF2 ,0x78);
    dw9714_WriteReg(0xDC ,0x51);
}

static void DW9714AF_DeInit(void)
{
	//printk("cuckoo %s\n",__func__);
	DW9714AF_reduce_lens_power_down_noise();
}



/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
int DW9714AF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	LOG_INF("Start\n");

	if (*g_pAF_Opened == 2) {
		LOG_INF("Wait\n");
		s4AF_WriteReg(200);
		msleep(20);
		s4AF_WriteReg(100);
		msleep(20);
	}

	if (*g_pAF_Opened) {
		LOG_INF("Free\n");

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 0;
		spin_unlock(g_pAF_SpinLock);
		DW9714AF_DeInit();
	}

	LOG_INF("End\n");

	return 0;
}

void DW9714AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;
	if (*g_pAF_Opened)
		DW9714AF_Init();
}
