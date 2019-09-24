/******************************************************************************
 * MODULE       : rohm_rpr0521_i2c.c
 * FUNCTION     : Driver source for RPR0521, Ambient Light Sensor(RGB) IC
 * AUTHOR       : Shengfan Wen
 * PROGRAMMED   : Software Development & Consulting, ArcharMind
 * MODIFICATION : Modified by Shengfan Wen, DEC/24/2015
 * NOTICE       : This software had been verified using MT6795.
 *              : When you use this code and document, Please verify all
 *              : operation in your operating system.
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
 *              : License along with this program.if not, write to the Free
 *              : Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 *              : Boston, MA  02110-1301, USA.
 *****************************************************************************/
#ifndef MT6795_ROHM
    //#define MT6795_ROHM (1)		/* for mt6795 */
#endif



#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <asm/div64.h>
#ifdef MT6795_ROHM
    #include <mach/mt_gpio.h>
    #include <mach/mt_pm_ldo.h>
    #include <linux/hwmsensor.h>
    #include <mach/eint.h>
#else
    #include <linux/gpio.h>
#endif

#include <cust_alsps.h>
#include <alsps.h>
#include "rohm_rpr0521_i2c_calib_20160606.h"

#ifdef MT6795_ROHM
static struct alsps_hw *hw ;
#else
/* Maintain alsps cust info here */
static struct alsps_hw alsps_cust;
static struct alsps_hw *hw = &alsps_cust;
struct platform_device *alspsPltFmDev;
static int alsps_init_flag = -1; /*0 -> ok, -1 -> fail*/
int proximity_initialise_status = 0;
EXPORT_SYMBOL(proximity_initialise_status);
#if ROHM_ENABLE_GAIN_SWITCH
static bool low_gain_enabled = true;
static bool high_gain_enabled = true;
#endif

/* For alsp driver get cust info */
struct alsps_hw *get_cust_alsps(void)
{
    return &alsps_cust;
}
#endif

/* *************************************************/
#define CHECK_RESULT(result)                        \
    if ( result < 0 )                               \
    {                                               \
        RPR0521_ERR("Error occur !!!.\n");          \
        return result;                              \
    }
/* *************************************************/


/*========================================================*/
#define COEFFICIENT               (4)
/*Modify begin by falin.luo for Pop4-6-4G*/
const u64 data0_coefficient[COEFFICIENT] = {8195, 5072, 3160, 2294};
const u64 data1_coefficient[COEFFICIENT] = {5253, 2531,  1338,  891};
const u64 judge_coefficient[COEFFICIENT] = {1292,  1603,  1934, 2574};
/*Modify end by falin.luo for Pop4-6-4G*/


/* gain table */
#define GAIN_FACTOR (16)
static const struct GAIN_TABLE {
    char data0;
    char data1;
} gain_table[GAIN_FACTOR] = {
    {  1,   1},   /*  0 */
    {  1,   2},   /*  1 */
    {  1,  64},   /*  2 */
    {  1, 128},   /*  3 */
    {  2,   1},   /*  4 */
    {  2,   2},   /*  5 */
    {  2,  64},   /*  6 */
    {  2, 128},   /*  7 */
    { 64,   1},   /*  8 */
    { 64,   2},   /*  9 */
    { 64,  64},   /* 10 */
    { 64, 128},   /* 11 */ //grace modify in 2014.4.11
    {128,   1},   /* 12 */
    {128,   2},   /* 13 */
    {128,  64},   /* 14 */
    {128, 128}    /* 15 */
};



/* logical functions */
static int __init           rpr0521_init(void);
static void __exit          rpr0521_exit(void);
static int                  rpr0521_probe(struct i2c_client *client,
        const struct i2c_device_id *id);
static int                  rpr0521_remove(struct i2c_client *client);
static void                 rpr0521_power(struct alsps_hw *hw, u32 on);
static int                  rpr0521_als_open_report_data(int open);
static int                  rpr0521_als_enable_nodata(int en);
static int                  rpr0521_als_set_delay(u64 delay);
static int                  rpr0521_als_get_data(int *als_value, int *status);
static int                  rpr0521_local_init(void);
static int                  rpr0521_local_remove(void);
static int                  rpr0521_ps_open_report_data(int open);
static int                  rpr0521_ps_enable_nodata(int en);
static int                  rpr0521_ps_set_delay(u64 delay);
static int                  rpr0521_ps_get_data(int *ps_value, int *status);
static int                  rpr0521_driver_init(struct i2c_client *client);
static int                  rpr0521_driver_shutdown(struct i2c_client *client);
static int                  rpr0521_driver_reset(struct i2c_client *client);
static int                  rpr0521_driver_als_power_on_off(
    struct i2c_client *client, unsigned char data);
static int                  rpr0521_driver_ps_power_on_off(
    struct i2c_client *client, unsigned char data);
static int                  rpr0521_driver_read_data(struct i2c_client *client,
        READ_DATA_ARG *data);
static void                 rpr0521_set_prx_thresh( u16 pilt,
        u16 piht );


#if defined(RPR0521_PS_CALIBRATIOIN_ON_CALL)\
    || defined(RPR0521_PS_CALIBRATIOIN_ON_START)
    static int              rpr0521_ps_calibration( RPR0521_DATA *obj);
#endif

/**************************** variable declaration ****************************/
static RPR0521_DATA             *obj = NULL;
static u64       int_top_time = 0;
static const char               rpr0521_driver_ver[] = RPR0521_DRIVER_VER;

/*************************** structure declaration ****************************/
/* I2C device IDs supported by this driver */
static const struct i2c_device_id rpr0521_id[] = {
    { RPR0521_I2C_NAME, 0 }, /* rohm bh1745 driver */
    { }
};

#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
    {.compatible = "mediatek,alsps"},
    {},
};
#endif
/* represent an I2C device driver */
static struct i2c_driver rpr0521_driver = {
    .driver = {                     /* device driver model driver */
        .owner = THIS_MODULE,
        .name  = RPR0521_I2C_DRIVER_NAME,
#ifdef CONFIG_OF
        .of_match_table = alsps_of_match,
#endif
    },
    .probe    = rpr0521_probe,  /* callback for device binding */
    .remove   = rpr0521_remove, /* callback for device unbinding */
    .shutdown = NULL,
    .suspend  = NULL,
    .resume   = NULL,
    .id_table = rpr0521_id, /* list of I2C devices supported by this driver */
};

/* MediaTek alsps information */
static struct alsps_init_info rpr0521_init_info = {
    .name = RPR0521_I2C_NAME,        /* Alsps driver name */
    .init = rpr0521_local_init,      /* Initialize alsps driver */
    .uninit = rpr0521_local_remove,  /* Uninitialize alsps driver */
};

/**
 * @Brief: rpr0521_register_dump print register value for debug
 *
 * @Param: reg_address regsiter address
 *
 * @return: no return
 */
static void rpr0521_register_dump(int addr)
{
    int result;
    unsigned char  read_data[6] = {0}, length = 0;

    if (NULL == obj->client) {
        RPR0521_ERR(" Parameter error \n");
        return ;
    }

    length = sizeof(read_data);
    RPR0521_WARNING( "length = %d\n", length);

    if((length + addr) >  RPR0521_REG_AILTH_ADDR) {
        RPR0521_WARNING( "length = %d\n is out of range!!!", length);
        length = RPR0521_REG_AILTH_ADDR - addr;
    }

    /* block read */
    result = i2c_smbus_read_i2c_block_data(obj->client, addr, length, read_data);

    if (result < 0)
        RPR0521_ERR( "ps_rpr0521_driver_general_read : transfer error \n");

    RPR0521_WARNING( "reg(0x%x) = 0x%x, reg(0x%x) = 0x%x, reg(0x%x) = 0x%x,"
                     "reg(0x%x) = 0x%x, reg(0x%x) = 0x%x, reg(0x%x) = 0x%x  \n",
                     (addr + 0), read_data[0], (addr + 1), read_data[1],
                     (addr + 2), read_data[2],
                     (addr + 3), read_data[3],
                     (addr + 4), read_data[4],
                     (addr + 5), read_data[5]);
}


/*----------------------------------------------------------------------------*/
static struct miscdevice rpr0521_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "rpr0521_device",
    //.fops = &rpr0521_fops,

};
/*----------------------------------------------------------------------------*/
static ssize_t show_als_value(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = obj->client;
    int res;
    int als_value = 0;
    int status = 0;

    if (NULL == client) {
        RPR0521_ERR("i2c client is NULL!!!\n");
        return 0;
    }

    rpr0521_als_get_data(&als_value, &status);
    res = snprintf(buf, PAGE_SIZE, "%d\n", als_value);
    return res;
}

static ssize_t show_ps_value(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = obj->client;
    READ_DATA_ARG       data;
    int res;

    if (NULL == client) {
        RPR0521_ERR("i2c client is NULL!!!\n");
        return 0;
    }

    res = rpr0521_driver_read_data(obj->client, &data);

    if(res < 0)
        RPR0521_ERR("read ps data fialed\n");

    res = snprintf(buf, PAGE_SIZE, "%d;%d;%d;thresh_near:%d;tresh_far:%d\n",
                   data.pdata, data.adata0, data.adata1, obj->thresh_near,
                   obj->thresh_far);
    return res;
}

static ssize_t rpr0521_show_allreg(struct device_driver *ddri, char *buf)
{
    if(!obj) {
        RPR0521_ERR("obj is null!!\n");
        return 0;
    }

    rpr0521_register_dump(0x40);
    rpr0521_register_dump(0x46);
    rpr0521_register_dump(0x4C);
    RPR0521_WARNING("obj->thresh_near = 0x%x, obj->thresh_far = 0x%x,"
                    "obj->polling_mode_ps = 0x%x obj->polling_mode_als = 0x%x \n",
                    obj->thresh_near,
                    obj->thresh_far,
                    obj->polling_mode_ps,
                    obj->polling_mode_als);
    return scnprintf(buf, PAGE_SIZE, " %d registers are read\n", 18);
}

static ssize_t rpr0521_store_reg(struct device_driver *ddri, const char *buf,
        size_t count)
{
#define MAX_LENGTH (3)
    int reg = 0, i2c_data = 0;
    int i = 0;
    int ret = 0;
    char *str_dest[MAX_LENGTH] = {0};
    char str_src[128];
    char delims[] = " ";
    char *str_result = NULL;
    char *cur_str = str_src;

    if(!obj) {
        RPR0521_ERR("obj is null !!!\n");
        return 0;
    }

    memcpy(str_src, buf, count);
    RPR0521_WARNING("Your input buf is: %s\n", str_src );

    /*spilt buf by space(" "), and seperated string are saved in str_src[]*/
    while(( str_result = strsep( &cur_str, delims ))) {
        if( i < MAX_LENGTH)   //max length should be 3
            str_dest[i++] = str_result;

        else {
            break;
        }
    }

    if (!strncmp(str_dest[0], "r", 1)) {
        reg = simple_strtol(str_dest[1], NULL, 16);

        /*check reg valid*/
        if(((reg & 0xFF) > RPR0521_REG_AILTH_ADDR)
                || ((reg & 0xFF) < RPR0521_REG_ID_ADDR)) {
            RPR0521_ERR("reg=0x%x is out of range !!!\n", reg );
            return -1;
        }

        /*read i2c data*/
        rpr0521_register_dump(reg & 0xFF);

    } else if (!strncmp(str_dest[0], "w",  1)) {
        reg      = simple_strtol(str_dest[1], NULL, 16);
        i2c_data = simple_strtol(str_dest[2], NULL, 16);

        /*check reg valid*/
        if(((reg & 0xFF) > RPR0521_REG_AILTH_ADDR)
                || ((reg & 0xFF) < RPR0521_REG_ID_ADDR)) {
            RPR0521_ERR("reg=0x%x is out of range !!!\n", reg );
            return -1;
        }

        /*write i2c data*/
        ret = i2c_smbus_write_byte_data(obj->client, (reg & 0xFF),
                                        (i2c_data & 0xFF));

        if (ret < 0) {
            RPR0521_ERR( " I2C read error !!!  \n" );
            return -1;
        }

        RPR0521_WARNING("writing...reg=0x%x, i2c_data=0x%x success\n",
                        reg, i2c_data);

    } else
        RPR0521_ERR("Please input right format: \"r 0x40\", \"w 0x40 0xFF\"\n");

    RPR0521_WARNING( "rpr0521_store_reg count=%d\n", (int)count);
    return count;
}

static ssize_t show_ps_pdata_value(struct device_driver *ddri, char *buf)
{
    int value = -1;
    struct i2c_client *client = obj->client;

    value = i2c_smbus_read_word_data(client, RPR0521_REG_PDATAL_ADDR);

    if (value < 0)
        return snprintf(buf, PAGE_SIZE, "Failed to read pdata!\n");

#ifdef ROHM_CALIBRATE
    return snprintf(buf, PAGE_SIZE, "%d %d %d\n",
            value, obj->min_ct, obj->boot_ct);
#else
    return snprintf(buf, PAGE_SIZE, "%d %d\n", value, obj->boot_ct);
#endif
}

static ssize_t show_ps_near_value(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", obj->thresh_near);
}

static ssize_t show_ps_far_value(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", obj->thresh_far);
}

static ssize_t show_als_data0_value(struct device_driver *ddri, char *buf)
{
    int value = -1;
    struct i2c_client *client = obj->client;

    value = i2c_smbus_read_word_data(client, RPR0521_REG_ADATA0L_ADDR);
    if (value < 0)
        return snprintf(buf, PAGE_SIZE, "Failed to read data0!\n");

    return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t show_als_data1_value(struct device_driver *ddri, char *buf)
{
    int value = -1;
    struct i2c_client *client = obj->client;

    value = i2c_smbus_read_word_data(client, RPR0521_REG_ADATA1L_ADDR);
    if (value < 0)
        return snprintf(buf, PAGE_SIZE, "Failed to read data1!\n");

    return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

#if ROHM_ENABLE_GAIN_SWITCH
static ssize_t show_als_gain_status(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "high_gain_enabled = %s\n"
                    "low_gain_enabled = %s\nals_gain_index = %u\n",
                    (high_gain_enabled ? "true" : "false"),
                    (low_gain_enabled ? "true" : "false"),
                    obj->als_gain_index);
}
#endif

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(rpr05xx_reg, S_IWUSR | S_IRUGO, rpr0521_show_allreg,
                   rpr0521_store_reg);
static DRIVER_ATTR(als, S_IWUSR | S_IRUGO, show_als_value, NULL);
static DRIVER_ATTR(ps, S_IWUSR | S_IRUGO, show_ps_value, NULL);
static DRIVER_ATTR(ct, S_IRUGO, show_ps_pdata_value, NULL);
static DRIVER_ATTR(near, S_IRUGO, show_ps_near_value, NULL);
static DRIVER_ATTR(far, S_IRUGO, show_ps_far_value, NULL);
static DRIVER_ATTR(data0, S_IRUGO, show_als_data0_value, NULL);
static DRIVER_ATTR(data1, S_IRUGO, show_als_data1_value, NULL);
#if ROHM_ENABLE_GAIN_SWITCH
static DRIVER_ATTR(als_gain, S_IRUGO, show_als_gain_status, NULL);
#endif

/*----------------------------------------------------------------------------*/
static struct driver_attribute *rpr0521_attr_list[] = {
    &driver_attr_rpr05xx_reg,
    &driver_attr_als,               /* read als value */
    &driver_attr_ps,                /* read ps value */
    &driver_attr_ct,
    &driver_attr_near,
    &driver_attr_far,
    &driver_attr_data0,
    &driver_attr_data1,
#if ROHM_ENABLE_GAIN_SWITCH
    &driver_attr_als_gain,
#endif
};

/*----------------------------------------------------------------------------*/
static int rpr0521_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(rpr0521_attr_list) / sizeof(rpr0521_attr_list[0]));

    if (driver == NULL)
        return -EINVAL;

    for(idx = 0; idx < num; idx++) {
        if((err = driver_create_file(driver, rpr0521_attr_list[idx]))) {
            RPR0521_WARNING("driver_create_file (%s) = %d\n",
                            rpr0521_attr_list[idx]->attr.name, err);
            break;
        }
    }

    return err;
}
/*----------------------------------------------------------------------------*/
static int rpr0521_delete_attr(struct device_driver *driver)
{
    int idx , err = 0;
    int num = (int)(sizeof(rpr0521_attr_list) / sizeof(rpr0521_attr_list[0]));

    if (!driver)
        return -EINVAL;

    for (idx = 0; idx < num; idx++)
        driver_remove_file(driver, rpr0521_attr_list[idx]);

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @Brief: rpr0521_power Power control for rpr0521 hardware
 *
 * @Param: hw BM1383 hardware ldo and voltage
 * @Param: on True for power on,flase for power off
 */
static void rpr0521_power(struct alsps_hw *hw, u32 on)
{
#ifdef MT6795_ROHM
    static u32 power_on;

    /* check parameter */
    if (NULL == hw) {
        RPR0521_ERR(" Parameter error \n");
        return ;
    }

    if (hw->power_id != MT65XX_POWER_NONE) {
        if (power_on == on)
            RPR0521_INFO("ignore power control: %d\n", on);

        else if (on) {
            if (!hwPowerOn(hw->power_id, hw->power_vol, RPR0521_I2C_NAME))
                RPR0521_ERR("power on fails!!\n");

        } else {
            if (!hwPowerDown(hw->power_id, RPR0521_I2C_NAME))
                RPR0521_ERR("power off fail!!\n");
        }
    }

    power_on = on;
#endif
}

#ifdef ROHM_CALIBRATE

int rpr521_read_ps(struct i2c_client *client, u16 *data)
{
    int tmp = 0;

    if(client == NULL)
        return -1;

    tmp = i2c_smbus_read_word_data(client, RPR0521_REG_PDATAL_ADDR);

    if(tmp < 0) {
        RPR0521_ERR("i2c read ps data fail.\n");
        return tmp;
    }

    *data = (u16)(tmp & 0xFFF);
    return 0;
}

int rpr521_judge_infra(struct i2c_client *client)
{
    int tmp = 0;
    unsigned char infrared_data;

    if(client == NULL)
        return -1;

    tmp = i2c_smbus_read_byte_data(client, RPR0521_REG_PRX_ADDR);

    if(tmp < 0) {
        RPR0521_ERR("i2c read ps infra data fail.\n");
        return tmp;
    }

    infrared_data = tmp;

    if(infrared_data >> 6)
        return 0;

    return 1;
}


static int rpr_get_offset(RPR0521_DATA *als_ps , u16 ct_value)
{
    if(ct_value <= 100) {
        als_ps->rpr_max_min_diff = 100;
        als_ps->rpr_ht_n_ct = 50;
        als_ps->rpr_lt_n_ct = 20;

    } else if(300 > ct_value && ct_value <= 200) {
        als_ps->rpr_max_min_diff = 50;
        als_ps->rpr_ht_n_ct = 40;
        als_ps->rpr_lt_n_ct = 15;

    } else if(400 > ct_value && ct_value <= 300) {
        als_ps->rpr_max_min_diff = 50;
        als_ps->rpr_ht_n_ct = 40;
        als_ps->rpr_lt_n_ct = 15;

    } else if(500 > ct_value && ct_value <= 400) {
        als_ps->rpr_max_min_diff = 100;
        als_ps->rpr_ht_n_ct = 50;
        als_ps->rpr_lt_n_ct = 20;

    } else if(700 > ct_value && ct_value <= 500) {
        als_ps->rpr_max_min_diff = 100;
        als_ps->rpr_ht_n_ct = 50;
        als_ps->rpr_lt_n_ct = 20;

    } else {
        als_ps->rpr_max_min_diff = 300;
        als_ps->rpr_ht_n_ct = 50;
        als_ps->rpr_lt_n_ct = 20;
    }

    RPR0521_DBG("chenge diff = %d, htnct = %d, ltnct = %d\n",
            als_ps->rpr_max_min_diff, als_ps->rpr_ht_n_ct, als_ps->rpr_lt_n_ct);
    return 0;
}


static int rpr_ps_tune_zero_final(RPR0521_DATA *als_ps)
{
    int res = 0;
    als_ps->tune_zero_init_proc = false;
    res = i2c_smbus_write_byte_data(als_ps->client, RPR0521_REG_ENABLE_ADDR,
            PS_ALS_SET_MODE_CONTROL); /*disable ps interrupt*/

    if(res < 0) {
        RPR0521_ERR("write i2c error\n");
        return res;
    }

    res = i2c_smbus_write_byte_data(als_ps->client, RPR0521_REG_INTERRUPT_ADDR,
            PS_ALS_SET_INTR | MODE_PROXIMITY); /*disable ps interrupt*/

    if(res < 0) {
        RPR0521_ERR("write i2c error\n");
        return res;
    }

    if(als_ps->data_count == -1) {
        als_ps->ps_th_h_boot = PS_ALS_SET_PS_TH;
        als_ps->ps_th_l_boot = PS_ALS_SET_PS_TL;
        als_ps->thresh_near = als_ps->ps_th_h_boot;
        als_ps->thresh_far = als_ps->ps_th_l_boot;
        als_ps->boot_cali = 0;
        RPR0521_ERR("exceed limit\n");
        hrtimer_cancel(&als_ps->ps_tune0_timer);
        return 0;
    }

    als_ps->psa = als_ps->ps_stat_data[0];
    als_ps->psi = als_ps->ps_stat_data[2];
    als_ps->boot_ct = als_ps->ps_stat_data[1];
    als_ps->min_ct = als_ps->ps_stat_data[1];
    rpr_get_offset(als_ps, als_ps->ps_stat_data[1]);
    als_ps->ps_th_h_boot = als_ps->ps_stat_data[1] + als_ps->rpr_ht_n_ct;
    als_ps->ps_th_l_boot = als_ps->ps_stat_data[1] + als_ps->rpr_lt_n_ct;
    als_ps->boot_cali = 40;
    als_ps->thresh_near = als_ps->ps_th_h_boot;
    als_ps->thresh_far = als_ps->ps_th_l_boot;
    als_ps->last_ct = als_ps->ps_stat_data[2];
    rpr0521_set_prx_thresh(als_ps->ps_th_l_boot, als_ps->ps_th_h_boot);
    RPR0521_WARNING("set HT=%d,LT=%d\n",
            als_ps->thresh_near, als_ps->thresh_far);
    hrtimer_cancel(&als_ps->ps_tune0_timer);
    return 0;
}


static int rpr_tune_zero_get_ps_data(RPR0521_DATA *als_ps)
{
    u16 ps_adc;
    int ret;
    unsigned char infra_flag = 0;
    ret = rpr521_read_ps(als_ps->client, &als_ps->ps);

    if(ret < 0) {
        als_ps->data_count = -1;
        rpr_ps_tune_zero_final(als_ps);
        return 0;
    }

    ps_adc = als_ps->ps;
    RPR0521_WARNING("ps_adc #%d = %d\n", als_ps->data_count, ps_adc);
    infra_flag = rpr521_judge_infra(als_ps->client);

    if(infra_flag != 1) {
        als_ps->data_count = -1;
        rpr_ps_tune_zero_final(als_ps);
        return 0;
    }

    als_ps->ps_stat_data[1]  +=  ps_adc;

    if(ps_adc > als_ps->ps_stat_data[0])
        als_ps->ps_stat_data[0] = ps_adc;

    if(ps_adc < als_ps->ps_stat_data[2])
        als_ps->ps_stat_data[2] = ps_adc;

    als_ps->data_count++;

    if(als_ps->data_count == 5) {
        als_ps->ps_stat_data[1]  /= als_ps->data_count;
        rpr_ps_tune_zero_final(als_ps);
    }

    return 0;
}


static int rpr_ps_tune_zero_func_fae(RPR0521_DATA *als_ps)
{
    u16 word_data;
    int ret, diff = 0;
    unsigned char infra_flag = 0;
    static int tune_times = 0;

    if(!als_ps->ps_en_status) {
        RPR0521_WARNING("ps wasn't enabled!\n");
        return 0;
    }

    ret = rpr521_read_ps(als_ps->client, &als_ps->ps);

    if(ret < 0)
        RPR0521_ERR("read ps failed, ret = 0x%x\n", ret);

    word_data = als_ps->ps;

    if(word_data == 0)
        return 0xFFFF;

    infra_flag = rpr521_judge_infra(als_ps->client);

    if(infra_flag != 1) {
        RPR0521_ERR("invalid infra data, infra_flag = 0x%x\n", infra_flag);
        return 0xFFFF;
    }

    if(word_data > als_ps->psa) {
        als_ps->psa = word_data;
        RPR0521_WARNING("update psa: psa = %d, psi = %d\n",
                als_ps->psa, als_ps->psi);
    }

    if(word_data < als_ps->psi) {
        als_ps->psi = word_data;
        RPR0521_WARNING("update psi: psa = %d, psi = %d\n",
                als_ps->psa, als_ps->psi);
    }

    if (word_data < als_ps->min_ct)
        als_ps->min_ct = word_data;

#if 0

    if(als_ps->psi_set) {
        if(word_data < als_ps->last_ct) {
            rpr_get_offset( als_ps, word_data);
            als_ps->thresh_near = word_data + als_ps->rpr_ht_n_ct;
            als_ps->thresh_far = word_data + als_ps->rpr_lt_n_ct;
            //rpr0521_set_prx_thresh(als_ps->thresh_far, als_ps->thresh_near);
            rpr0521_set_prx_thresh(RPR0521_PILT_FAR, als_ps->thresh_near);
            als_ps->last_ct = word_data;
        }

        //grace modify begin
        if (word_data >= als_ps->thresh_near ) {
            if (word_data > 1000) {
                als_ps->thresh_near = als_ps->psi + 2 * als_ps->rpr_ht_n_ct;
                als_ps->thresh_far = als_ps->psi + 3 * als_ps->rpr_lt_n_ct;
                als_ps->last_ct = als_ps->psi;
                rpr0521_set_prx_thresh(als_ps->thresh_far, RPR0521_PIHT_NEAR);
            }
        }

        //grace modify end
        RPR0521_WARNING("tune1 : update HT = %d, LT = %d, word_data = %d "
                "als_ps->last_ct=%d  line=%d\n",
                als_ps->thresh_near, als_ps->thresh_far, word_data,
                als_ps->last_ct, __LINE__);

    } else
#endif
    {
        diff = als_ps->psa - als_ps->psi;
        rpr_get_offset(als_ps, word_data);

        if(diff > als_ps->rpr_max_min_diff) {
            als_ps->psi_set = als_ps->psi;
            rpr_get_offset( als_ps, als_ps->psi);
            als_ps->thresh_near = als_ps->psi + als_ps->rpr_ht_n_ct;
            als_ps->thresh_far = als_ps->psi + als_ps->rpr_lt_n_ct;

            if((als_ps->thresh_near > als_ps->ps_th_h_boot + 120)
                    && (als_ps->boot_cali > 0)) {
                als_ps->thresh_near = als_ps->ps_th_h_boot + 120;
                als_ps->thresh_far  = als_ps->ps_th_l_boot + 100;
                RPR0521_WARNING("update boot HT = %d, LT = %d\n",
                        als_ps->ps_th_h_boot, als_ps->ps_th_l_boot);
            }

#if 0
            if (als_ps->boot_ct < 500 && als_ps->thresh_far < als_ps->boot_ct) {
                als_ps->thresh_near = als_ps->ps_th_h_boot;
                als_ps->thresh_far  = als_ps->ps_th_l_boot;
                RPR0521_WARNING("Reset the psi and min ct value!\n");
                als_ps->psi = 0xFFF;
                als_ps->min_ct = 0xFFF;
            }
#endif
            rpr0521_set_prx_thresh(als_ps->thresh_far, als_ps->thresh_near);
            als_ps->last_ct = als_ps->psi;
            RPR0521_WARNING("update HT = %d, LT = %d\n",
                    als_ps->thresh_near, als_ps->thresh_far);
            if (tune_times == 10) {
                tune_times = 0;
                hrtimer_cancel(&als_ps->ps_tune0_timer);
            } else {
                RPR0521_WARNING("tune times = %d\n", tune_times);
                tune_times += 1;
            }
        }
    }
    return 0;
}

static void rpr_ps_tune0_work_func(struct work_struct *work)
{
    RPR0521_DATA *als_ps = container_of(work, RPR0521_DATA, rpr_ps_tune0_work);

    if(als_ps->tune_zero_init_proc)
        rpr_tune_zero_get_ps_data(als_ps);

    else
        rpr_ps_tune_zero_func_fae(als_ps);

    return;
}

static enum hrtimer_restart rpr_ps_tune0_timer_func(struct hrtimer *timer)
{
    RPR0521_DATA *als_ps = container_of(timer, RPR0521_DATA, ps_tune0_timer);
    queue_work(als_ps->rpr_ps_tune0_wq, &als_ps->rpr_ps_tune0_work);
    hrtimer_forward_now(&als_ps->ps_tune0_timer, als_ps->ps_tune0_delay);
    return HRTIMER_RESTART;
}

#endif

/**
 * @Brief: rpr0521_als_open_report_data RPR0521 initialization or uninitialization
 *
 * @Param: open 1 for initialize,0 for uninitialize
 *
 * @Returns: 0 for success,other for failed.
 */
static int rpr0521_als_open_report_data(int open)
{
    int result = 0;
    RPR0521_WARNING(" open=%d \n", open);

    /* check parameter */
    if (NULL == obj) {
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

    RPR0521_WARNING(" als_en_status=%d ps_en_status=%d\n", obj->als_en_status,
                    obj->ps_en_status);

    if (open) {
        //First, we check ps enable status
        if(false == obj->ps_en_status)
            result = rpr0521_driver_init(obj->client);

        obj->als_en_status = true;

    } else {
        //First, we check ps enable status
        if(false == obj->ps_en_status)
            result = rpr0521_driver_shutdown(obj->client);

        obj->als_en_status = false;
    }

    return result;
}

/**
 * @Brief: rpr0521_als_enable_nodata Enable or disable RPR0521
 *
 * @Param: en 1 for enable,0 for disable
 *
 * @Returns: 0 for success,others for failed.
 */
static int rpr0521_als_enable_nodata(int en)
{
    int result = 0;
    RPR0521_FUN();

    if (NULL == obj) {
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

    result = rpr0521_driver_als_power_on_off(obj->client, en);
    return result;
}

/**
 * @Brief: rpr0521_als_set_delay Set delay,not used for now.
 *
 * @Param: delay Delay time.
 *
 * @Returns: 0 for success,other for failed.
 */
static int rpr0521_als_set_delay(u64 delay)
{
    return 0;
}


/*****************************************************************************
 * @Brief: rpr0521_ps_calibration is used to get ps noise raw data.
 *         we just read 5 times ps raw data and get average, and set this 
 *         average as ps noise raw data. Be note: als is disabled during this 
 *         process, this process will cost 5*15ms at least
 * @Param: obj_ptr point to global RPR0521_DATA
 *
 * @Returns: 0 for success, other for failed.
 *
 *****************************************************************************/
#if (defined(RPR0521_PS_CALIBRATIOIN_ON_CALL) \
        || defined(RPR0521_PS_CALIBRATIOIN_ON_START))
static int rpr0521_ps_calibration( RPR0521_DATA *obj_ptr)
{
    int ret = 0;
    int i = 0, result = 0, average = 0;
    unsigned char i2c_read_data[2] = {0}, i2c_data = 0, enable_old_setting = 0;
    unsigned char interrupt_old_setting = 0;
    unsigned char infrared_data = 0;
    u16 pdata = 0;
    RPR0521_WARNING("rpr0521_calibration IN \n");

    if(NULL == obj_ptr || NULL == obj_ptr->client) {
        RPR0521_ERR(" Parameter error \n");
        return -1;
    }

    /* disable ps interrupt begin */
    interrupt_old_setting = i2c_smbus_read_byte_data(obj_ptr->client,
                            RPR0521_REG_INTERRUPT_ADDR);

    if (interrupt_old_setting < 0) {
        RPR0521_ERR( " I2C read error !!!  \n" );
        return -1;
    }

    i2c_data = interrupt_old_setting & 0XFE;
    result = i2c_smbus_write_byte_data(obj_ptr->client,
                                       RPR0521_REG_INTERRUPT_ADDR, i2c_data);

    if (result < 0) {
        RPR0521_ERR( " I2C read error !!!  \n" );
        return -1;
    }

    /* disable ps interrupt end */
    /* set ps measurment time to 10ms begin */
    enable_old_setting = i2c_smbus_read_byte_data(obj_ptr->client,
                         RPR0521_REG_ENABLE_ADDR);

    if (enable_old_setting < 0) {
        RPR0521_ERR( " I2C read error !!!  \n" );
        ret = -1;
        goto err_interrupt_status;
    }

    /*just keep ps_pulse and ps operating mode*/
    i2c_data = ((enable_old_setting & 0x30)  | PS_EN | PS10MS);
    result = i2c_smbus_write_byte_data(obj_ptr->client, RPR0521_REG_ENABLE_ADDR,
                                       i2c_data);

    if(result < 0) {
        RPR0521_ERR( " I2C read error !!!  \n" );
        ret = -1;
        goto err_interrupt_status;
    }

    /* set ps measurment time to 10ms end */
    /* check the infrared valid */
    i2c_data = i2c_smbus_read_byte_data(obj_ptr->client, RPR0521_REG_PRX_ADDR);

    if(i2c_data < 0) {
        RPR0521_ERR( " I2C read error !!!  \n" );
        ret = -1;
        goto err_exit;
    }

    infrared_data = i2c_data;

    if(infrared_data >> 6) { //ambient infrared level is high(too high)
        ret = -1;
        goto err_exit;
    }

    /* begin read ps raw data */
    for(i = 0; i < RPR521_PS_CALI_LOOP; i ++) {
        msleep(15); //sleep, wait IC to finish this measure
        result = i2c_smbus_read_i2c_block_data(
                     obj_ptr->client,
                     RPR0521_REG_PDATAL_ADDR,
                     sizeof(i2c_read_data),
                     i2c_read_data);

        if(result < 0) {
            RPR0521_ERR( " I2C read error !!!  \n" );
            ret = -1;
            goto err_exit;
        }

        pdata = (u16)(((u16)i2c_read_data[1] << 8) |
                                 i2c_read_data[0]);
        average += pdata & 0xFFF;
        RPR0521_WARNING(" pdata = %d i = %d\n", pdata, i);
    }

    average /= RPR521_PS_CALI_LOOP;  //average
    RPR0521_WARNING("rpr0521_ps_calib average=%d \n", average);
    //obj_ptr->thresh_far  = average + hw->ps_threshold_low;
    //obj_ptr->thresh_near = average + hw->ps_threshold_high;
    obj_ptr->thresh_far  = average + 35;
    obj_ptr->thresh_near = average + 85;
err_exit:
    i2c_smbus_write_byte_data(
        obj_ptr->client,
        RPR0521_REG_ENABLE_ADDR,
        enable_old_setting);
err_interrupt_status:
    i2c_smbus_write_byte_data(
        obj_ptr->client,
        RPR0521_REG_INTERRUPT_ADDR,
        interrupt_old_setting);
    RPR0521_WARNING("rpr521 PS calibration end\r\n");
    return ret;
}
#endif

static void rpr0521_ps_init_data(RPR0521_DATA *obj_ptr)
{
    RPR0521_FUN();

    if(obj_ptr == NULL || obj_ptr->hw == NULL ) {
        RPR0521_ERR("parameter error! \n");
        return ;
    }

    /* Initialize */
    obj_ptr->last_nearby = PRX_NEAR_BY_UNKNOWN;
    /*obj_ptr->thresh_near = RPR521_PS_NOISE_DEFAULT + hw->ps_threshold_high;*/
    /*obj_ptr->thresh_far  = RPR521_PS_NOISE_DEFAULT + hw->ps_threshold_low;*/
    obj_ptr->thresh_near = RPR521_PS_NOISE_DEFAULT + 26;
    obj_ptr->thresh_far  = RPR521_PS_NOISE_DEFAULT + 70;
    obj_ptr->polling_mode_ps   =  obj_ptr->hw->polling_mode_ps;
    obj_ptr->polling_mode_als  =  obj_ptr->hw->polling_mode_als;
}



/*===========================================================================

  FUNCTION      rpr0521_set_prx_thresh

  DESCRIPTION   set ps interrupt threshold

  DEPENDENCIES  None

  RETURN VALUE  None

  SIDE EFFECT   None

  ===========================================================================*/
static void rpr0521_set_prx_thresh(u16 pilt, u16 piht)
{
    int result;
    unsigned char thresh[4];
    u16  pilt_tmp = 0, piht_tmp = 0;
#ifdef RPR0521_DEBUG
    unsigned char          read_thresh[4] = {0};
#endif
    RPR0521_FUN();

    if (NULL == obj || NULL == obj->client) {
        RPR0521_ERR(" Parameter error \n");
        return ;
    }

    pilt_tmp = pilt;
    piht_tmp = piht;

    if(piht >= 0x0FFF) {
        piht_tmp = 0x0FFF;
        pilt_tmp = 0x0FFF - (piht - pilt);
        RPR0521_WARNING("piht is out of range!!!\n");
    }

    RPR0521_WARNING("pilt low: 0x%x, piht hig: 0x%x\n", pilt_tmp, piht_tmp);
    thresh[2] = (pilt_tmp & 0xFF); /* PILTL */
    thresh[3] = (pilt_tmp >> 8);   /* PILTH */
    thresh[0] = (piht_tmp & 0xFF); /* PIHTL */
    thresh[1] = (piht_tmp >> 8);   /* PIHTH */
    RPR0521_WARNING(" befor write reg(0x4B) = 0x%x, reg(0x4C) = 0x%x,"
                    "reg(0x4D) = 0x%x, reg(0x4E) = 0x%x\n",
                    thresh[0], thresh[1], thresh[2], thresh[3] );
    /*  write block failed, and change to write byte */
    result  = i2c_smbus_write_byte_data(
                  obj->client,
                  RPR0521_REG_PIHTL_ADDR,
                  thresh[0]);
    result |= i2c_smbus_write_byte_data(
                  obj->client,
                  RPR0521_REG_PIHTH_ADDR,
                  thresh[1]);
    result |= i2c_smbus_write_byte_data(
                  obj->client,
                  RPR0521_REG_PILTL_ADDR,
                  thresh[2]);
    result |= i2c_smbus_write_byte_data(
                  obj->client,
                  RPR0521_REG_PILTH_ADDR,
                  thresh[3]);

    if ( result < 0 ) {
        RPR0521_ERR("write data from IC error.\n");
        return ;
    }

#ifdef RPR0521_DEBUG
    /*  check write value is successful, or not */
    result = i2c_smbus_read_i2c_block_data(
                 obj->client,
                 RPR0521_REG_PIHTL_ADDR,
                 sizeof(read_thresh),
                 read_thresh);

    if ( result < 0 ) {
        RPR0521_ERR("Read data from IC error.\n");
        return ;
    }

    RPR0521_WARNING(" after write reg(0x4B) = 0x%x, reg(0x4C) = 0x%x,"
                    "reg(0x4D) = 0x%x, reg(0x4E) = 0x%x\n",
                    read_thresh[0], read_thresh[1], read_thresh[2], 
                    read_thresh[3]);
#endif
}

/**
 * @Brief: RPR0521 initialization or uninitialization
 *
 * @Param: open 1 for initialize,0 for uninitialize
 *
 * @Returns: 0 for success,other for failed.
 */
static int rpr0521_ps_open_report_data(int open)
{
    int result = 0;
    RPR0521_WARNING(" open=%d \n", open);

    /* Check parameter */
    if (NULL == obj || NULL == obj->client) {
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

    RPR0521_WARNING(" als_en_status=%d ps_en_status=%d\n",
                    obj->als_en_status, obj->ps_en_status);

    if (open) {
        /* First, we check als enable status */
        if(obj->als_en_status == false)
            result = rpr0521_driver_init(obj->client);

        obj->ps_en_status = true;
#ifdef RPR0521_PS_CALIBRATIOIN_ON_CALL
        /* calibration */
        result = rpr0521_ps_calibration(obj);

        if(result == 0)
            RPR0521_WARNING("ps calibration success! \n");

        else
            RPR0521_WARNING("ps calibration failed! \n");

        if(obj->polling_mode_ps == 0)   // set ps interrupt threshold
            rpr0521_set_prx_thresh(obj->thresh_far, obj->thresh_near);
#endif

    } else {
        /*  First, we check als enable status */
        if(obj->als_en_status == false)
            result = rpr0521_driver_shutdown(obj->client);

        obj->ps_en_status = false;
    }

    return result;
}

/**
 * @Brief: rpr0521_ps_enable_nodata Enable or disable RPR0521
 *
 * @Param: en 1 for enable,0 for disable
 *
 * @Returns: 0 for success,others for failed.
 */
static int rpr0521_ps_enable_nodata(int en)
{
    int result = 0;
    RPR0521_FUN();

    if (NULL == obj || NULL == obj->client) {
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

    result = rpr0521_driver_ps_power_on_off(obj->client, en);
    return result;
}

/**
 * @Brief: rpr0521_als_set_delay Set delay,not used for now.
 *
 * @Param: Delay time: unit ns.
 *
 * @Returns: 0 for success,other for failed.
 */
static int rpr0521_ps_set_delay(u64 delay)
{
    return 0;
}


/*===========================================================================
  FUNCTION      rpr0521_ps_get_real_value
  DESCRIPTION   This function is called to get real proximity status which
                will be saved to obj->prx_detection_state
  DEPENDENCIES  None
  RETURN VALUE  None
  SIDE EFFECT   None
  ===========================================================================*/
static void rpr0521_ps_get_real_value(void)
{
    int result;
    u16 pdata ;

    /* check parameter */
    if (NULL == obj || NULL == obj->client) {
        RPR0521_ERR(" Parameter error \n");
        return ;
    }

    pdata = obj->ps_raw_data;
    RPR0521_WARNING("obj->thresh_near(0x%x) obj->thresh_far(0x%x)\n",
                    obj->thresh_near, obj->thresh_far);

    if ( pdata >= obj->thresh_near ) {
        /* grace modify for special case, ired is too large ,or not */
        result = i2c_smbus_read_byte_data(obj->client, RPR0521_REG_PRX_ADDR);

        if ( result < 0 ) {
            RPR0521_ERR("Read data from IC error.\n");
            return ;
        }

        RPR0521_WARNING("RPR0521_PRX_ADDR(0x%x) value = 0x%x\n",
                        RPR0521_REG_PRX_ADDR, result);

        if ( obj->last_nearby != PRX_NEAR_BY )
            obj->prx_detection_state = PRX_NEAR_BY;  //get ps status

        obj->last_nearby = PRX_NEAR_BY;

        if (obj->polling_mode_ps == 0) { //interrupt
            rpr0521_set_prx_thresh( obj->thresh_far, RPR0521_PIHT_NEAR );  //grace modify
        }

#if 0

        if ( 0 == (result >>  RPR0521_AMBIENT_IR_FLAG) ) { //ir is low
            /* check last ps status */
            if ( obj->last_nearby != PRX_NEAR_BY ) {
                obj->prx_detection_state = PRX_NEAR_BY;  //get ps status
            }

            obj->last_nearby = PRX_NEAR_BY;

            if (obj->polling_mode_ps == 0) { //interrupt
                rpr0521_set_prx_thresh( obj->thresh_far,
                                        RPR0521_PIHT_NEAR ); /* set threshold for last interrupt */
            }

        } else { //special case: ir is high
            /* check last ps status */
            if ( obj->last_nearby != PRX_FAR_AWAY ) {
                obj->prx_detection_state = PRX_FAR_AWAY; //get ps status
                obj->last_nearby = PRX_FAR_AWAY;
                rpr0521_set_prx_thresh(RPR0521_PILT_FAR,
                                       obj->thresh_near);  /* set threshold for last interrupt */
            }
        }

#endif

    } else if ( pdata < obj->thresh_far ) {
        /* check last ps status */
        if ( obj->last_nearby != PRX_FAR_AWAY ) {
            obj->prx_detection_state = PRX_FAR_AWAY; //get ps status
        }

        obj->last_nearby = PRX_FAR_AWAY;

        if (obj->polling_mode_ps == 0) { //interrupt
#if 0
            obj->thresh_near = pdata + obj->rpr_ht_n_ct;
            obj->thresh_far = pdata + obj->rpr_lt_n_ct;
            obj->last_ct = pdata;
#endif
            rpr0521_set_prx_thresh(RPR0521_PILT_FAR, obj->thresh_near);
        }
    }
}


/*===========================================================================
  FUNCTION      rpr0521_als_convert_to_mlux
  DESCRIPTION   Convert a raw data to a real milli lux
  DEPENDENCIES  None
  RETURN VALUE  milli lux value or 0 if there was a error
  SIDE EFFECT   None
  ===========================================================================*/
u32 rpr0521_als_convert_to_mlux(u16 data0,
        u16 data1, u16 gain_index, u16 time)
{
#define JUDGE_FIXED_COEF (1000)
#define MAX_OUTRANGE     (65535)  //grace modify in 2014.4.9
#define MAXRANGE_NMODE   (0xFFFF)
#define MAXSET_CASE      (4)
#define MLUX_UNIT        (1000)    // Lux to mLux
#define CALC_ERROR       (0xFFFFFFFF)
    u32       final_data;
    calc_data_type     calc_data;
    calc_ans_type      calc_ans;
    u64      calc_judge;
    unsigned char      set_case;
    u64      max_range;
    unsigned char      gain_factor;
    RPR0521_DBG("rpr0521 calc als\n");
    /* set the value of measured als data */
    calc_data.als_data0  = data0;
    calc_data.als_data1  = data1;
    gain_factor          = gain_index & 0x0F;
    calc_data.gain_data0 = gain_table[gain_factor].data0;
    max_range = (u64)MAX_OUTRANGE;

    /* calculate data */
    if (calc_data.als_data0 == MAXRANGE_NMODE)
        calc_ans.positive = max_range;

    else {
        /* get the value which is measured from power table */
        calc_data.als_time = time ;

        if (calc_data.als_time == 0) {
            /* issue error value when time is 0 */
            RPR0521_ERR("calc_data.als_time  == 0\n"); //grace modify in 2014.4.9
            return (CALC_ERROR);
        }

        calc_judge = calc_data.als_data1 * JUDGE_FIXED_COEF;

        if (calc_judge < (calc_data.als_data0 * judge_coefficient[0]))
            set_case = 0;

        else if (calc_judge < (calc_data.als_data0 * judge_coefficient[1]))
            set_case = 1;

        else if (calc_judge < (calc_data.als_data0 * judge_coefficient[2]))
            set_case = 2;

        else if (calc_judge < (calc_data.als_data0 * judge_coefficient[3]))
            set_case = 3;

        else
            set_case = MAXSET_CASE;

        RPR0521_DBG("rpr0521-set_case = %d , calc_data.als_time= %d\n",
                        set_case, calc_data.als_time);

        if (set_case >= MAXSET_CASE) {
            calc_ans.positive = 0; //which means that lux output is 0

        } else {
            calc_data.gain_data1 = gain_table[gain_factor].data1;
            RPR0521_DBG("calc_data.gain_data0 = %d calc_data.gain_data1 = %d\n",
                            calc_data.gain_data0, calc_data.gain_data1);
            calc_data.data0 = (u64 )(data0_coefficient[set_case] *
                                                    calc_data.als_data0) * calc_data.gain_data1;
            calc_data.data1 = (u64 )(data1_coefficient[set_case] *
                                                    calc_data.als_data1) * calc_data.gain_data0;

            if(calc_data.data0 < calc_data.data1) {
                /*In this case, data will be less than 0. As data is unsigned
                  long long, it will become extremely big.
                 */
                RPR0521_ERR("rpr0521 calc_data.data0 < calc_data.data1\n");
                return (CALC_ERROR);
            }

            RPR0521_DBG("calc_data.data0 = %llu calc_data.data1 = %llu\n",
                            calc_data.data0, calc_data.data1);
            calc_data.data = calc_data.data0 - calc_data.data1;
            /* 24 bit at max (128 * 128 * 100 * 10) */
            calc_data.dev_unit = calc_data.gain_data0 * calc_data.gain_data1 *
                                 calc_data.als_time * 10;

            if (calc_data.dev_unit == 0) {
                /* issue error value when dev_unit is 0 */
                RPR0521_ERR("rpr0521 calc_data.dev_unit == 0\n");
                return (CALC_ERROR);
            }

            /* calculate a positive number */
            //calc_ans.positive = (u64)((calc_data.data) / calc_data.dev_unit);
            do_div(calc_data.data, calc_data.dev_unit);
            calc_ans.positive = calc_data.data;

            if (calc_ans.positive > max_range)
                calc_ans.positive = max_range;
        }
    }

    final_data = calc_ans.positive * MLUX_UNIT;
    RPR0521_DBG("Final data = %d\n", final_data);
    return (final_data);
#undef JUDGE_FIXED_COEF
#undef MAX_OUTRANGE
#undef MAXRANGE_NMODE
#undef MAXSET_CASE
#undef MLUX_UNIT
}

/**
 * @Brief: rpr0521_als_get_data Get data from RPR0521 hardware.
 *
 * @Param: als_value Return value including lux and rgbc.
 * @Param: status Return rpr0521 status.
 *
 * @Returns: 0 for success,other for failed.
 */
static int rpr0521_als_get_data(int *als_value, int *status)
{
    int result;
    READ_DATA_ARG       data;
#if ROHM_ENABLE_GAIN_SWITCH
    unsigned char mode;
#endif
    RPR0521_FUN();

    if((NULL == als_value) || (NULL == status) || (NULL == obj)) {
        RPR0521_ERR(" Parameter error \n");
        return -EINVAL;
    }

    // set default value to ALSPS_INVALID_VALUE if it can't get an valid data
    *als_value = ALSPS_INVALID_VALUE;
    //read adata0 adata1 and pdata
    result = rpr0521_driver_read_data(obj->client, &data);
    CHECK_RESULT(result);
    RPR0521_DBG("pdata = 0x%x, adata0 = 0x%x, adata1 = 0x%x \n",
                    data.pdata, data.adata0, data.adata1);
    obj->data_mlux = rpr0521_als_convert_to_mlux(data.adata0, data.adata1,
                     obj->als_gain_index, obj->als_measure_time);
    *als_value = obj->data_mlux / 1000;  //Should transfer mlux to lux
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;
#if ROHM_ENABLE_GAIN_SWITCH
    if (*als_value > 1000 && high_gain_enabled) {
        mode = (LEDCURRENT_100MA | (ALS_GAIN_2X << 4) | (ALS_GAIN_2X << 2));
        high_gain_enabled = false;
        low_gain_enabled = true;
        i2c_smbus_write_byte_data(obj->client, RPR0521_REG_ALS_ADDR, mode);
        obj->als_gain_index = (ALS_GAIN_2X << 2) | (ALS_GAIN_2X);
    }

    if (*als_value < 900 && low_gain_enabled) {
        mode = (LEDCURRENT_100MA | (ALS_GAIN_64X << 4) | (ALS_GAIN_64X << 2));
        low_gain_enabled = false;
        high_gain_enabled = true;
        i2c_smbus_write_byte_data(obj->client, RPR0521_REG_ALS_ADDR, mode);
        obj->als_gain_index = (ALS_GAIN_64X << 2) | (ALS_GAIN_64X);
    }
#endif
#ifdef RPR0521_DEBUG
    //debug ps interrupt
    rpr0521_register_dump(RPR0521_REG_ID_ADDR);
    rpr0521_register_dump(RPR0521_REG_INTERRUPT_ADDR);
#endif
    return 0;
}

/**
 * @Brief: rpr0521_ps_get_data Get data from RPR0521 hardware.
 *
 * @Param: als_value Return value including lux and rgbc.
 * @Param: status Return rpr0521 status.
 *
 * @Returns: 0 for success,other for failed.
 */
static int rpr0521_ps_get_data(int *ps_value, int *status)
{
    int result;
    READ_DATA_ARG       data;

    if(ps_value == NULL || status == NULL || obj == NULL) {
        RPR0521_ERR(" Parameter error \n");
        return -EINVAL;
    }

    RPR0521_FUN();
    //set default value to ALSPS_INVALID_VALUE if it can't get an valid data
    *ps_value = ALSPS_INVALID_VALUE;
    //read adata0 adata1 and pdata
    result = rpr0521_driver_read_data(obj->client, &data);   //get ps raw data
    CHECK_RESULT(result);
    RPR0521_WARNING("pdata = 0x%x, adata0 = 0x%x, adata1 = 0x%x \n",
                    data.pdata, data.adata0, data.adata1);
    /*calculate ps status:
     far or near, and the result is saved to obj->prx_detection_state
     */
    rpr0521_ps_get_real_value();
    RPR0521_WARNING("ps raw 0x%x -> ps status = %d\n",
                    obj->ps_raw_data, obj->prx_detection_state);
    *ps_value = obj->prx_detection_state;   //get ready to report
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    return 0;
}

/**
 * @Brief: rpr0521_local_init Initial RPR0521 driver.
 *
 * @Returns: 0 for success,others for failed.
 */
static int rpr0521_local_init(void)
{
    RPR0521_FUN();

    /* check parameter */
    if (NULL == hw) {
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

    /* Power on */
    rpr0521_power(hw, POWER_ON);

    if (i2c_add_driver(&rpr0521_driver)) {
        RPR0521_ERR("failed to add i2c driver!\n");
        return -1;
    }

    if (alsps_init_flag == -1)
        return -1;

    return 0;
}

/**
 * @Brief: rpr0521_remove Remove RPR0521 driver.
 *
 * @Returns: 0 for success,others for failed.
 */
static int rpr0521_local_remove(void)
{
    RPR0521_FUN();

    /* check parameter */
    if (NULL == hw) {
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

    rpr0521_power(hw, POWER_OFF);
    i2c_del_driver(&rpr0521_driver);
    return 0;
}


/*----------------------------------------------------------------------------*/
static void rpr0521_eint_work(struct work_struct *work)
{
    int result;
#ifdef MT6795_ROHM
    hwm_sensor_data sensor_data;
#endif
    READ_DATA_ARG       data;

    if(NULL == obj || NULL == obj->client) {
        RPR0521_ERR(" rpr0521_eint_work \n" );
        enable_irq(obj->irq);
        return;
    }

#ifdef MT6795_ROHM
    memset(&sensor_data, 0, sizeof(sensor_data));
#endif
    RPR0521_DBG("rpr0521 int top half time = %lld\n", int_top_time);
    //check interrtup status
    result = i2c_smbus_read_byte_data(obj->client, RPR0521_REG_INTERRUPT_ADDR);

    if ( result < 0 ) {
        RPR0521_ERR("Read data from IC error.\n");
        enable_irq(obj->irq);
        return;
    }

    RPR0521_WARNING("RPR0521_REG_INTERRUPT_ADDR(0x4A) value = 0x%x\n", result);

    /* Check which interrupts occured */
    if ((result & RPR0521_REG_PINT_STATUS ) && (obj->polling_mode_ps == 0)) {
        //get adata0 adata1 and pdata
        result = rpr0521_driver_read_data(obj->client, &data);

        if ( result < 0 ) {
            RPR0521_ERR("Read data from IC error.\n");
            enable_irq(obj->irq);
            return;
        }

        RPR0521_WARNING("pdata = 0x%x, adata0 = 0x%x, adata1 = 0x%x \n",
                        data.pdata, data.adata0, data.adata1);
        /*calculate ps status:
         far or near, and the result is saved to obj->prx_detection_state
         */
        rpr0521_ps_get_real_value();
#ifdef MT6795_ROHM
        sensor_data.values[0] = obj->prx_detection_state; // get to report
        sensor_data.value_divide = 1;
        sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
        RPR0521_WARNING("ps raw 0x%x -> value 0x%x \n", obj->ps_raw_data,
                        sensor_data.values[0]);

        //let up layer to know
        if(ps_report_interrupt_data(sensor_data.values[0]))   // report
            RPR0521_ERR("call ps_report_interrupt_data fail\n");

#else
        RPR0521_WARNING("ps raw 0x%x -> value 0x%x \n", obj->ps_raw_data,
                        obj->prx_detection_state);

        //let up layer to know
        if(ps_report_interrupt_data(obj->prx_detection_state))   // report
            RPR0521_ERR("call ps_report_interrupt_data fail\n");

#endif
    }

    enable_irq(obj->irq);
    RPR0521_DBG(" irq handler end\n");
}


/*----------------------------------------------------------------------------*/
static irqreturn_t rpr0521_eint_handler(int irq, void *desc)
{
    RPR0521_DBG(" interrupt handler\n");

    if (NULL == obj) {
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

    int_top_time = sched_clock();

    if(obj->polling_mode_ps == 0)
        schedule_delayed_work(&obj->eint_work, 0);

    disable_irq_nosync(obj->irq);
    return IRQ_HANDLED;
}


/*----------------------------------------------------------------------------*/
static int rpr0521_setup_eint(RPR0521_DATA *obj_ptr)
{
#if defined(CONFIG_OF)
    u32 ints[2] = {0, 0};
#endif
#ifndef MT6795_ROHM
    int ret = 0;
    struct pinctrl *pinctrl;
    struct pinctrl_state *pins_default;
    struct pinctrl_state *pins_cfg;
#endif

    if (NULL == obj_ptr || NULL == obj_ptr->client) {
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

#ifdef MT6795_ROHM
    /* configure to GPIO function, external interrupt */
    mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
    mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
    mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);
#else
    alspsPltFmDev = get_alsps_platformdev();
    /* gpio setting */
    pinctrl = devm_pinctrl_get(&alspsPltFmDev->dev);

    if (IS_ERR(pinctrl)) {
        ret = PTR_ERR(pinctrl);
        RPR0521_ERR("Cannot find alsps pinctrl!\n");
        return ret;
    }

    pins_default = pinctrl_lookup_state(pinctrl, "pin_default");

    if (IS_ERR(pins_default)) {
        ret = PTR_ERR(pins_default);
        RPR0521_ERR("Cannot find alsps pinctrl default!\n");
    }

    pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");

    if (IS_ERR(pins_cfg)) {
        ret = PTR_ERR(pins_cfg);
        RPR0521_ERR("Cannot find alsps pinctrl pin_cfg!\n");
        return ret;
    }

    pinctrl_select_state(pinctrl, pins_cfg);
#endif
#if defined(CONFIG_OF)

    if (obj_ptr->irq_node) {
        of_property_read_u32_array(obj_ptr->irq_node, "debounce", ints,
                                   ARRAY_SIZE(ints)); // read from dts(dws)
#ifdef MT6795_ROHM
        mt_gpio_set_debounce(ints[0], ints[1]);
#else
        gpio_set_debounce(ints[0], ints[1]);
#endif
        RPR0521_WARNING("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);
        obj_ptr->irq = irq_of_parse_and_map(obj_ptr->irq_node, 0);
        RPR0521_WARNING("obj_ptr->irq = %d\n", obj_ptr->irq);

        if (!obj_ptr->irq) {
            RPR0521_ERR("irq_of_parse_and_map fail!!\n");
            return -EINVAL;
        }

        /*register irq*/
        if(request_irq(obj_ptr->irq, rpr0521_eint_handler, IRQF_TRIGGER_FALLING,
                       "ALS-eint", NULL)) {
            RPR0521_ERR("IRQ LINE NOT AVAILABLE!!\n");
            return -EINVAL;
        }

    } else {
        RPR0521_ERR("null irq node!!\n");
        return -EINVAL;
    }

#endif
    return 0;
}

/*----------------------------------------------------------------------------*/
static int rpr0521_init_client(RPR0521_DATA *obj_ptr)
{
    int result;
    RPR0521_FUN();

    if (NULL == obj_ptr || NULL == obj_ptr->client) {
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

    if((result = rpr0521_driver_reset(obj_ptr->client))) {
        RPR0521_ERR("software reset error, err=%d \n", result);
        return result;
    }

#ifdef ROHM_CALIBRATE
    obj_ptr->psi_set = 0;
    obj_ptr->ps_stat_data[0] = 0;
    obj_ptr->ps_stat_data[2] = 4095;
    obj_ptr->ps_stat_data[1] = 0;
    obj_ptr->data_count = 0;
    obj_ptr->ps_th_h_boot = PS_ALS_SET_PS_TH;
    obj_ptr->ps_th_l_boot = PS_ALS_SET_PS_TL;
    obj_ptr->tune_zero_init_proc = true;
    obj_ptr->boot_ct = 0xFFF;
    obj_ptr->ps_nf = 1;
    obj_ptr->first_boot = true;
    rpr0521_set_prx_thresh(obj_ptr->ps_th_l_boot, obj_ptr->ps_th_h_boot);
    result = i2c_smbus_write_byte_data(
                 obj_ptr->client,
                 RPR0521_REG_ENABLE_ADDR,
                 PS_ALS_SET_MODE_CONTROL | PS_EN);	//soft-reset

    if (result != 0)
        return (result);

    hrtimer_start(&obj_ptr->ps_tune0_timer, obj_ptr->ps_tune0_delay,
                  HRTIMER_MODE_REL);
#endif

    if(obj_ptr->polling_mode_ps == 0 ) {
        RPR0521_FUN();

        if((result = rpr0521_setup_eint(obj_ptr))) {
            RPR0521_ERR("setup eint error: %d\n", result);
            return result;
        }
    }

    return 0;
}

/******************************************************************************
 * NAME       : rpr0521_probe
 * FUNCTION   : initialize system
 * REMARKS    :
 *****************************************************************************/
static int rpr0521_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    RPR0521_DATA            *rpr0521_data;
    int                     result;
    struct als_control_path als_ctl = {0};   //als control function pointer
    struct als_data_path    als_data = {0};  //als report funcation pointer
    struct ps_control_path  ps_ctl = {0};    //ps control function pointer
    struct ps_data_path     ps_data = {0};   //ps report funcation pointer
    RPR0521_WARNING("called rpr0521_probe for RPR0521 \n");

    /* check parameter */
    if (NULL == client) {
        RPR0521_ERR(" Parameter error !!! \n");
        return EINVAL;
    }

    result = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);

    if (!result) {
        RPR0521_ERR( "need I2C_FUNC_I2C !!!\n");
        result = -ENODEV;
        return result;
    }

    /* read IC id and check valid */
    result = i2c_smbus_read_byte_data(client, RPR0521_REG_ID_ADDR);

    if (result < 0) {
        RPR0521_ERR("Read data from IC error !!!\n");
        result = -EIO;
        return result;
    }

    RPR0521_WARNING("MANUFACT_VALUE=0x%x\n", result);

    if((result & 0x3F) != RPR0521_REG_ID_VALUE) {
        RPR0521_ERR("Error, IC value NOT correct !!! \n");
        result = -EINVAL;
        return result;
    }

    rpr0521_data = kzalloc(sizeof(*rpr0521_data), GFP_KERNEL);

    if (rpr0521_data == NULL) {
        result = -ENOMEM;
        return result;
    }

    //initilize to zero
    memset(rpr0521_data, 0 , sizeof(*rpr0521_data));
    obj = rpr0521_data;
    rpr0521_data->client = client;
    rpr0521_data->hw = hw;
    i2c_set_clientdata(client, rpr0521_data);
#ifdef MT6795_ROHM
    rpr0521_data->irq_node = of_find_compatible_node(NULL, NULL,
                             "mediatek, ALS-eint"); //read irq_node from dws
#else
    rpr0521_data->irq_node = of_find_compatible_node(NULL, NULL,
                             "mediatek, als-eint"); //read irq_node from dws
#endif

    if(rpr0521_data->irq_node) {
        if(rpr0521_data->irq_node->name) {
            RPR0521_WARNING("rpr0521_data->irq_node name = %s\n",
                            rpr0521_data->irq_node->name);

        } else
            RPR0521_WARNING("rpr0521_data->irq_node name is NULL \n");

    } else
        RPR0521_WARNING("rpr0521_data->irq_node is NULL \n");

    rpr0521_ps_init_data(rpr0521_data); /* Initialize data*/
#ifdef RPR0521_PS_CALIBRATIOIN_ON_START
    /*Here, we should initilize ps if we want to calibrate ps*/
    rpr0521_driver_init(client);
    /* calibration */
    result = rpr0521_ps_calibration(rpr0521_data);

    if(0 == result)
        RPR0521_WARNING("ps calibration success! \n");

    else
        RPR0521_WARNING("ps calibration failed! \n");

#endif
    INIT_DELAYED_WORK(&rpr0521_data->eint_work, rpr0521_eint_work);
#ifdef ROHM_CALIBRATE
    rpr0521_data->rpr_ps_tune0_wq =
        create_singlethread_workqueue("rpr_ps_tune0_wq");
    INIT_WORK(&rpr0521_data->rpr_ps_tune0_work, rpr_ps_tune0_work_func);
    hrtimer_init(&rpr0521_data->ps_tune0_timer, CLOCK_MONOTONIC,
                 HRTIMER_MODE_REL);
    rpr0521_data->ps_tune0_delay = ns_to_ktime(120 * NSEC_PER_MSEC);
    rpr0521_data->ps_tune0_timer.function = rpr_ps_tune0_timer_func;
#endif

    if((result = rpr0521_init_client(rpr0521_data)))
        goto err_power_failed;

    if((result = misc_register(&rpr0521_device))) {
        RPR0521_WARNING("rpr0521_device register failed result = %d \n",
                        result);
        goto exit_misc_device_register_failed;
    }

    if((result = rpr0521_create_attr(
                     &(rpr0521_init_info.platform_diver_addr->driver)))) {
        RPR0521_WARNING("create attribute err = %d\n", result);
        goto exit_create_attr_failed;
    }

    /* als initialize */
    als_ctl.open_report_data = rpr0521_als_open_report_data;
    als_ctl.enable_nodata    = rpr0521_als_enable_nodata;
    als_ctl.set_delay        = rpr0521_als_set_delay;
    als_ctl.is_use_common_factory = false;
    result = als_register_control_path(&als_ctl);

    if (result) {
        RPR0521_ERR("als_register_control_path failed, error = %d\n", result);
        goto err_power_failed;
    }

    als_data.get_data   = rpr0521_als_get_data;
    als_data.vender_div = 1;
    result = als_register_data_path(&als_data);

    if (result) {
        RPR0521_ERR("als_register_data_path failed, error = %d\n", result);
        goto err_power_failed;
    }

    /* ps initialize */
    ps_ctl.open_report_data = rpr0521_ps_open_report_data;
    ps_ctl.enable_nodata    = rpr0521_ps_enable_nodata;
    ps_ctl.set_delay        = rpr0521_ps_set_delay;
    ps_ctl.is_use_common_factory = false;
    result = ps_register_control_path(&ps_ctl);

    if (result) {
        RPR0521_ERR("ps_register_control_path failed, error = %d\n", result);
        goto err_power_failed;
    }

    ps_data.get_data   = rpr0521_ps_get_data;
    ps_data.vender_div = 1;
    result = ps_register_data_path(&ps_data);

    if (result) {
        RPR0521_ERR("ps_register_data_path failed, error = %d\n", result);
        goto err_power_failed;
    }

    RPR0521_WARNING(" rpr0521_probe for RPR0521 OK \n");
    alsps_init_flag = 0;
    proximity_initialise_status = 1;
    return 0;
exit_create_attr_failed:
    misc_deregister(&rpr0521_device);
exit_misc_device_register_failed:
err_power_failed:
    kfree(rpr0521_data);
    return (result);
}

/******************************************************************************
 * NAME       : rpr0521_remove
 * FUNCTION   : close system
 * REMARKS    :
 *****************************************************************************/
static int rpr0521_remove(struct i2c_client *client)
{
    int err;
    RPR0521_DATA *rpr0521_data;

    if (NULL == client) {
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

    if((err = rpr0521_delete_attr(
                  &(rpr0521_init_info.platform_diver_addr->driver))))
        RPR0521_ERR("rpr0521_delete_attr fail: %d\n", err);

    if((err = misc_deregister(&rpr0521_device)))
        RPR0521_ERR("misc_deregister fail: %d\n", err);

    rpr0521_data    = i2c_get_clientdata(client);
    kfree(rpr0521_data);
    return (0);
}

/************************************************************
 * NAME       : rpr0521_driver_init
 * FUNCTION   : initialize RPR0521
 * REMARKS    :
 ***********************************************************/
static int rpr0521_driver_init(struct i2c_client *client)
{
    unsigned char w_mode[4];
    int result;
    RPR0521_FUN();

    if (NULL == client) {
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

    /* execute software reset */
    result = rpr0521_driver_reset(client);

    if (result != 0)
        return (result);

    /* led current 100 mA */
    w_mode[0] = (LEDCURRENT_100MA | (ALS_GAIN_2X << 4) | (ALS_GAIN_2X << 2));
    w_mode[1] = (PS_GAIN_2X << 4) | PS_PERSISTENCE_SETTING;
    w_mode[2] = (PS_THH_BOTH_OUTSIDE | POLA_ACTIVEL | OUTPUT_LATCH |
                 INT_TRIG_BY_ONLY_PS);
    w_mode[3] = (BOTH100MS);
    result = i2c_smbus_write_byte_data(client, RPR0521_REG_ALS_ADDR, w_mode[0]);

    if (result == 0) {
        result = i2c_smbus_write_byte_data(
                     client,
                     RPR0521_REG_PRX_ADDR,
                     w_mode[1]);

        if (result == 0) {
            result = i2c_smbus_write_byte_data(
                         client,
                         RPR0521_REG_INTERRUPT_ADDR,
                         w_mode[2]);

            if (result == 0)
                result = i2c_smbus_write_byte_data(
                             client,
                             RPR0521_REG_ENABLE_ADDR,
                             w_mode[3]);
        }
    }

    //set measure time
    if(BOTH100MS == w_mode[3])
        obj->als_measure_time = CALC_MEASURE_100MS;

    else if(BOTH400MS == w_mode[3])
        obj->als_measure_time = CALC_MEASURE_400MS;

    //set measure gain
    obj->als_gain_index = (ALS_GAIN_2X << 2) | (ALS_GAIN_2X);  //for cal lux
#if ROHM_ENABLE_GAIN_SWITCH
    high_gain_enabled = true;
    low_gain_enabled = true;
#endif
#ifdef RPR0521_DEBUG
    rpr0521_register_dump(RPR0521_REG_ID_ADDR);
#endif
    return (result);
}

/******************************************************************************
 * NAME       : rpr0521_driver_shutdown
 * FUNCTION   : shutdown RPR0521
 * REMARKS    :
 *****************************************************************************/
static int rpr0521_driver_shutdown(struct i2c_client *client)
{
    int result;

    if (NULL == client) {
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

    /* set soft ware reset */
    result = rpr0521_driver_reset(client);
    return (result);
}

/******************************************************************************
 * NAME       : rpr0521_driver_reset
 * FUNCTION   : reset RPR0521 register
 * REMARKS    :
 *****************************************************************************/
static int rpr0521_driver_reset(struct i2c_client *client)
{
    int result = 0;
    RPR0521_FUN();

    if (NULL == client) {
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

    /* set soft ware reset */
    result = i2c_smbus_write_byte_data(client, RPR0521_REG_ID_ADDR, 0xC0);
    CHECK_RESULT(result);
    return 0;
}

/******************************************************************************
 * NAME       : rpr0521_driver_power_on_off
 * FUNCTION   : power on and off RPR0521
 * REMARKS    :
 *****************************************************************************/
static int rpr0521_driver_als_power_on_off(struct i2c_client *client,
        unsigned char data)
{
    int           result;
    unsigned char mode_ctl2;
    unsigned char power_set;
    unsigned char write_data;

    if (NULL == client) {
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

    RPR0521_WARNING(" data=%d\n", data);
    /* read enable_addr register */
    result = i2c_smbus_read_byte_data(client, RPR0521_REG_ENABLE_ADDR);
    CHECK_RESULT(result);

    if (data == 0)
        power_set = ALS_OFF;

    else
        power_set = ALS_EN;

    /* read enable_addr and mask ALS_EN  */
    mode_ctl2  = (unsigned char)(result & ~ALS_EN);
    write_data = mode_ctl2 | power_set;
    result = i2c_smbus_write_byte_data(
                 client,
                 RPR0521_REG_ENABLE_ADDR,
                 write_data);
    CHECK_RESULT(result);
    return (0);
}


/******************************************************************************
 * NAME       : rpr0521_driver_power_on_off
 * FUNCTION   : power on and off RPR0521
 * REMARKS    :
 *****************************************************************************/
static int rpr0521_driver_ps_power_on_off(struct i2c_client *client,
        unsigned char data)
{
    int          result;
    unsigned char mode_ctl2;
    unsigned char power_set;
    unsigned char write_data;

    if (NULL == client) {
        RPR0521_ERR("Parameter error\n");
        return EINVAL;
    }

#ifdef ROHM_CALIBRATE

    if (!data) {
        hrtimer_cancel(&obj->ps_tune0_timer);
        cancel_work_sync(&obj->rpr_ps_tune0_work);
    }

    if(obj->first_boot == true)
        obj->first_boot = false;

    if(data) {
        obj->psi_set = 0;
        obj->psa = 0;
        obj->psi = 0xFFF;
        obj->last_ct = 0xFFF;
        if (obj->min_ct * 3 < 0xFFF)
            obj->thresh_near = obj->min_ct * 3;
        else
            obj->thresh_near = 0xFFF;
        obj->thresh_far  = 0x0;
        obj->min_ct = 0xFFF;
        rpr0521_set_prx_thresh(obj->thresh_far, obj->thresh_near);
        obj->prx_detection_state = PRX_FAR_AWAY; //grace modify
        obj->last_nearby = PRX_FAR_AWAY;

        //report far away firstly after enable
        if(ps_report_interrupt_data(obj->prx_detection_state))   // report
            RPR0521_ERR("call ps_report_interrupt_data fail\n");
    }

#endif
    RPR0521_WARNING("data = %d\n", data);
    /* read enable_addr register */
    result = i2c_smbus_read_byte_data(client, RPR0521_REG_ENABLE_ADDR);
    CHECK_RESULT(result);

    if (data == POWER_OFF)
        power_set = PS_OFF;

    else
        power_set = PS_EN;

    RPR0521_WARNING("RPR0521_REG_ENABLE_ADDR(0x41) = 0x%x\n", result);
    /* read enable_addr and mask ALS_EN  */
    mode_ctl2  = (unsigned char)(result & ~PS_EN);
    write_data = mode_ctl2 | power_set;
    RPR0521_WARNING("write_data = 0x%x\n", write_data);
    result = i2c_smbus_write_byte_data(
                 client,
                 RPR0521_REG_ENABLE_ADDR,
                 write_data);
    CHECK_RESULT(result);

    if (data) {
#ifdef ROHM_CALIBRATE
        msleep(200);
        hrtimer_start(&obj->ps_tune0_timer, obj->ps_tune0_delay,
                      HRTIMER_MODE_REL);
#endif
    }

#ifdef RPR0521_DEBUG
    rpr0521_register_dump(RPR0521_REG_ID_ADDR);
#endif
    return (0);
}

/******************************************************************************
 * NAME       : rpr0521_driver_read_data
 * FUNCTION   : read the value of RGB data and status in RPR0521
 * REMARKS    :
 *****************************************************************************/
static int rpr0521_driver_read_data(struct i2c_client *client,
        READ_DATA_ARG *data)
{
    int result;
    unsigned char  read_data[6] = {0};

    if (NULL == client || NULL == data) {
        RPR0521_ERR(" Parameter error \n");
        return EINVAL;
    }

    /* block read */
    result = i2c_smbus_read_i2c_block_data(
                 client,
                 RPR0521_REG_PDATAL_ADDR,
                 sizeof(read_data),
                 read_data);

    if (result < 0)
        RPR0521_ERR( "ps_rpr0521_driver_general_read : transfer error \n");

    else {
        data->pdata =  (u16 )(((u16 )read_data[1] << 8) |
                                         read_data[0]);
        data->adata0 = (u16 )(((u16 )read_data[3] << 8) |
                                         read_data[2]);
        data->adata1 = (u16 )(((u16 )read_data[5] << 8) |
                                         read_data[4]);
        result = 0;
    }

    obj->ps_raw_data = data->pdata;  //save ps raw data
#ifdef ROHM_CALIBRATE
    if (obj->ps_raw_data != 0 && obj->ps_raw_data < obj->min_ct)
        obj->min_ct = obj->ps_raw_data;
#endif
    return (result);
}




/******************************************************************************
 * NAME       : rpr0521_init
 * FUNCTION   : register driver to kernel
 * REMARKS    :
 *****************************************************************************/
static int __init rpr0521_init(void)
{
#ifdef MT6795_ROHM
    hw = get_cust_alsps_hw();
#else
    const char *name = "mediatek,rpr0521";
    RPR0521_WARNING("rpr0521_init\n");
    hw =   get_alsps_dts_func(name, hw);

    if (!hw)
        RPR0521_ERR("get dts info fail\n");

#endif
    alsps_driver_add(&rpr0521_init_info);
    return 0;
}

/******************************************************************************
 * NAME       : rpr0521_exit
 * FUNCTION   : remove driver from kernel
 * REMARKS    :
 *****************************************************************************/
static void __exit rpr0521_exit(void)
{
    //nothing to do
    return;
}


MODULE_DESCRIPTION("ROHM Ambient Light And Proximity Sensor Driver");
MODULE_LICENSE("GPL");

module_init(rpr0521_init);
module_exit(rpr0521_exit);
