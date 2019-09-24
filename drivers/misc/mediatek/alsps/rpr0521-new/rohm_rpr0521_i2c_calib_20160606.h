/******************************************************************************
 * MODULE       : rohm_rpr0521_i2c.h
 * FUNCTION     : Driver header for RPR0521 Ambient Light Sensor(RGB) IC
 * AUTHOR       : Shengfan Wen
 * PROGRAMMED   : Software Development & Consulting, ArcharMind
 * MODIFICATION : Modified by Shengfan Wen, DEC/18/2015
 * REMARKS      :
 * COPYRIGHT    : Copyright (C) 2015 - ROHM CO.,LTD.
 *              : This program is free software; you can redistribute it and/or
 *              : modify it under the terms of the GNU General Public License
 *              : as published by the Free Software Foundation; either version 2
 *              : of the License, or (at your option) any later version.
 *              :
 *              : This program is distributed in the hope that it will be useful,
 *              : but WITHOUT ANY WARRANTY; without even the implied warranty of
 *              : MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *              : GNU General Public License for more details.
 *              :
 *              : You should have received a copy of the GNU General Public
 *              : License along with this program; if not, write to the Free
 *              : Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 *              : Boston, MA 02110-1301, USA.
 *****************************************************************************/
#ifndef _ROHM_RPR0521_I2C_H_
#define _ROHM_RPR0521_I2C_H_


/* comment out for closing debug info */
//#define RPR0521_DEBUG  (NULL)
#define RPR             "[RPR0521-ALS/PS]"

#define  RPR0521_ERR(fmt, arg...)       \
    do { \
        printk(KERN_ERR RPR "ERROR (%s, %d):" fmt, __func__,  __LINE__, ##arg);\
    } while (0)

#define  RPR0521_WARNING(fmt, arg...)   \
    do { \
        printk(KERN_WARNING RPR "(%s, %d):" fmt, __func__,  __LINE__, ##arg);\
    } while (0)


#ifdef RPR0521_DEBUG
#define  RPR0521_FUN(f)                 \
    do { \
        printk(KERN_WARNING RPR "(%s, %d)\n", __func__,  __LINE__);\
    } while (0)
#define  RPR0521_INFO(fmt, arg...)      \
    do { \
        printk(KERN_INFO RPR "INFO (%s, %d):" fmt, __func__,  __LINE__, ##arg);\
    } while (0)
#define  RPR0521_DBG(fmt, arg...)       \
    do { \
        printk(KERN_DEBUG RPR "DEBUG (%s, %d):" fmt, __func__,  __LINE__, ##arg);\
    } while (0)
#else
#define  RPR0521_FUN()                  do {} while (0)
#define  RPR0521_INFO(f, a...)          do {} while (0)
#define  RPR0521_DBG(f, a...)           do {} while (0)
#endif


/* *************************************************/

/*if the customer do not want to calibrate on call, comment out this macro*/
//#define RPR0521_PS_CALIBRATIOIN_ON_CALL    (NULL)

/*if the customer do not want to calibrate on boot, comment out this macro*/
//#define RPR0521_PS_CALIBRATIOIN_ON_START   (NULL)


#define ROHM_CALIBRATE          (1)
#define ROHM_ENABLE_GAIN_SWITCH (1)


#define RPR521_PS_CALI_LOOP     (5)     //average of 5 times
#define RPR521_PS_NOISE_DEFAULT (60) 	//default ps noise data

/* ldo power on/off */
#define POWER_ON    (1)  //on
#define POWER_OFF   (0)  //off

#define RPR0521_AMBIENT_IR_FLAG              (6)  /* ambient ir flag:
                                                   * 00: infrared low
                                                   * 01: infrared high
                                                   * 02: infrared too hight */

/* *************************************************/


/************ definition to dependent on sensor IC ************/
#define RPR0521_I2C_NAME        ("rpr0521_i2c_name")
#define RPR0521_I2C_DRIVER_NAME ("rpr0521_i2c_driver_name")
#define RPR0521_DRIVER_VER      ("rpr0521.2016.05.06")

/************ define register for IC ************/
/* RPR0521 REGSTER */
/* Register Addresses define */
#define RPR0521_REG_ID_ADDR         (0x40)
#define RPR0521_REG_ID_VALUE        (0x0A)      /*ID Value*/
#define RPR0521_REG_ENABLE_ADDR     (0x41)
#define RPR0521_REG_ALS_ADDR        (0x42)
#define RPR0521_REG_PRX_ADDR        (0x43)
#define RPR0521_REG_PDATAL_ADDR     (0x44)
#define RPR0521_REG_PDATAH_ADDR     (0x45)
#define RPR0521_REG_ADATA0L_ADDR    (0x46)
#define RPR0521_REG_ADATA0H_ADDR    (0x47)
#define RPR0521_REG_ADATA1L_ADDR    (0x48)
#define RPR0521_REG_ADATA1H_ADDR    (0x49)
#define RPR0521_REG_INTERRUPT_ADDR  (0x4A)
#define RPR0521_REG_PIHTL_ADDR      (0x4B)
#define RPR0521_REG_PIHTH_ADDR      (0x4C)
#define RPR0521_REG_PILTL_ADDR      (0x4D)
#define RPR0521_REG_PILTH_ADDR      (0x4E)
#define RPR0521_REG_AIHTL_ADDR      (0x4F)
#define RPR0521_REG_AIHTH_ADDR      (0x50)
#define RPR0521_REG_AILTL_ADDR      (0x51)
#define RPR0521_REG_AILTH_ADDR      (0x52)

/* Register Value define : STATUS */
#define RPR0521_REG_PINT_STATUS     (0x80)  /* PRX interrupt status */
#define RPR0521_REG_AINT_STATUS     (0x40)  /* ALS interrupt status */


#define RPR0521_PIHT_NEAR           (0x0FFF)
#define RPR0521_PILT_FAR            (0)

typedef enum {
    ALS_GAIN_1X    = 0,    /* 1x AGAIN */
    ALS_GAIN_2X    = 1,    /* 2x AGAIN */
    ALS_GAIN_64X   = 2,    /* 64x AGAIN */
    ALS_GAIN_128X  = 3     /* 128x AGAIN */
} als_gain_e;

typedef enum {
    PS_GAIN_1X   = 0,
    PS_GAIN_2X   = 1,
    PS_GAIN_4X   = 2,
    PS_GAIN_INVALID  = 3,
} ps_gain_e;


//grace modify in 2014.4.11 begin
#define ALS_EN          (0x1 << 7)
#define ALS_OFF         (0x0 << 7)
#define PS_EN           (0x1 << 6)
#define PS_OFF          (0x0 << 6)
#define BOTH_STANDBY    (0)
#define PS10MS          (0x1)
#define PS40MS          (0x2)
#define PS100MS         (0x3)
#define ALS100MS_PS50MS (0x5)
#define BOTH100MS       (0x6)
#define BOTH400MS       (0xB)

#define CALC_MEASURE_100MS   (100)
#define CALC_MEASURE_400MS   (400)


#define LEDCURRENT_025MA    (0)
#define LEDCURRENT_050MA    (1)
#define LEDCURRENT_100MA    (2)
#define LEDCURRENT_200MA    (3)

/* PS_TH_H and PS_TH_L are effective as outside detection */
#define PS_THH_ONLY         (0b00 << 4)

/* PS_TH_H and PS_TH_L are effective as outside detection */
#define PS_THH_BOTH_HYS     (0b01 << 4)

/* PS_TH_H and PS_TH_L are effective as outside detection */
#define PS_THH_BOTH_OUTSIDE (0b10 << 4)

/* 
 * 0:Interrupt output "L" is stable if newer measurement result
 * is also interrupt active
 */
#define POLA_ACTIVEL        (0b00 << 3)

/* 
 * 1:Interrupt output "L" is de-assert and re-assert if newer
 * measurement result is also interrupt active
 */
#define POLA_INACTIVEL      (0b01 << 3)

/* 0:INT pin is latched until INTERRUPT register is read or initialized */
#define OUTPUT_ANYTIME      (0b00 << 2)

/* 1:INT pin is updated after each measurement */
#define OUTPUT_LATCH        (0b01 << 2)

#define MODE_NONUSE         (0)
#define MODE_PROXIMITY      (1)
#define MODE_ILLUMINANCE    (2)
#define MODE_BOTH           (3)
#define INT_TRIG_IS_INACTIVE    (0b00)   /* INT pin is inactive */
#define INT_TRIG_BY_ONLY_PS     (0b01)   /* Triggerded by only PS */
#define INT_TRIG_BY_ONLY_ALS    (0b10)   /* Triggerded by only ALS */
#define INT_TRIG_BY_PS_AND_ALS  (0b11)   /* Triggerded by PS and ALS */

/* interrupt is updated at each measurement end */
#define PS_PERSISTENCE_SETTING      (1)



#define PS_DOUBLE_PULSE           (1 << 5)
/* grace modify in 2015.2.16 */
#define PS_ALS_SET_MODE_CONTROL   (PS_DOUBLE_PULSE | ALS100MS_PS50MS)

//grace modify in 2015.2.16
#define PS_ALS_SET_INTR           (PS_THH_BOTH_OUTSIDE | POLA_ACTIVEL | \
                                   OUTPUT_LATCH | MODE_NONUSE)
#define PS_ALS_SET_PS_TH          (1000)	//grace modify in 2015.2.16
#define PS_ALS_SET_PS_TL          (999)	    //grace modify in 2015.2.16



/************ typedef struct ************/
/* structure to read data value from sensor */
typedef struct {
    u16 pdata;         /* data value of pdata from sensor  */
    u16 adata0;        /* data value of adata0 from sensor */
    u16 adata1;        /* data value of adata1 from sensor */
} READ_DATA_ARG;



/******************************* define *******************************/
/* structure of peculiarity to use by system */

/* proximity status */
typedef enum {
    PRX_NEAR_BY,
    PRX_FAR_AWAY,
    PRX_NEAR_BY_UNKNOWN
} prx_nearby_type;


typedef struct {
    struct i2c_client *client;      /* structure pointer for i2c bus */
    struct alsps_hw  *hw;           /* hw resource */
    struct delayed_work eint_work;  /* interrupt work queue */

    /* interrupt node in device tree  which was configed in dws */
    struct device_node *irq_node;

    prx_nearby_type
    prx_detection_state;    /* proximity status: Far away or Near by */
    prx_nearby_type  last_nearby;           /* last proximity status */

    bool als_en_status;                     /* current als enable status */
    bool ps_en_status;                      /* current ps enable status  */
    u16 ps_raw_data;             /* proximity raw data */
    u16 thresh_near;             /* ps near threshold */
    u16 thresh_far;              /* ps near threshold */
    u16
    als_measure_time;                       /* als gain for Lux calculation */
    u16 als_gain_index;          /* als gain for Lux calculation */
    int irq;                                /* als and ps interrupt number */
    int polling_mode_ps;              /* 1: polling mode ; 0:interrupt mode */
    int polling_mode_als;             /* 1: polling mode ; 0:interrupt mode */
    u32 data_mlux;                 /* als lux(unit:micro lux)*/


#ifdef ROHM_CALIBRATE
    u16 last_ct;
    u16 psa;
    u16 psi;
    u16 psi_set;
    u16 boot_ct;
    u16 min_ct;
    u16 boot_cali;
    struct hrtimer ps_tune0_timer;
    struct workqueue_struct *rpr_ps_tune0_wq;
    struct work_struct rpr_ps_tune0_work;
    ktime_t ps_tune0_delay;
    bool tune_zero_init_proc;
    u32 ps_stat_data[3];
    int data_count;
    int rpr_max_min_diff;
    int ps_nf;
    int rpr_lt_n_ct;
    int rpr_ht_n_ct;
    u16 ps_th_h_boot;
    u16 ps_th_l_boot;
    bool first_boot;
    u16  ps;
#endif

} RPR0521_DATA;

typedef struct {
    u64  data;
    u64  data0;
    u64  data1;
    unsigned char       gain_data0;
    unsigned char       gain_data1;
    u32       dev_unit;
    u16      als_time; //grace modify in 2014.4.11
    u16      als_data0;
    u16      als_data1;
} calc_data_type;

typedef struct {
    u32 positive;
    u32 decimal;
} calc_ans_type;


#endif /* _ROHM_RPR0521_I2C_H_ */
