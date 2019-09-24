/*
 * MELFAS MIP4 Touchscreen for MediaTek
 *
 * Copyright (C) 2015 MELFAS Inc.
 *
 *
 * mip4.c : Main functions
 *
 *
 * Version : 2015.08.23
 *
 */

#include "mip4.h"
#include <linux/of_irq.h>
#include <mt_boot_common.h>

/**
* MTK
*/
extern struct tpd_device *tpd;
struct mip_ts_info *mip_info;
extern unsigned int gtp_fw_version;
extern unsigned int gtp_cfg_version;
extern int gesture_wakeup_flag;
//zhongzhu add start for TP type indentify  in tct_common
extern u8 gtp_sensor_id;
//zhongzhu add end for TP type indentify  in tct_common
static int mip_gesture_wakeup_flag;
static bool force_updated = false;
static bool normal_update_failed = false;

#if defined (MIP_FLIP_COVER_SWITCH)
    #define FLIP_COVER_MODE_ON 1
    #define FLIP_COVER_MODE_OFF 0
    #define FLIP_COVER_MODE_ADDR 0x061B
    extern int smart_cover_flag;
    extern void (*hall_state_charge_notify)(int state);
    void mip_flip_cover_switch(int state);
    extern void (*smart_cover_flag_charge_notify)(int flag);
    void mip_flip_cover_flag_charge(int flag);
    int mip_cover_state = 1;
#endif

#define MIP_I2C_SPEED 		300	//kHz
#define MIP_I2C_DMA_LENGTH 	9 	//Bytes
static u8 *i2c_dma_buf_va = NULL;
static dma_addr_t i2c_dma_buf_pa = 0;

struct task_struct *mtk_eint_thread = NULL;
int mtk_eint_flag = 0;
int mtk_eint_count = 0;
DECLARE_WAIT_QUEUE_HEAD(waiter);

int mip_tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;

#ifdef TPD_HAVE_BUTTON
    int mip_tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif
//static irqreturn_t mip_interrupt(int irq, void *dev_id);

/**
* Reboot chip
*
* Caution : IRQ must be disabled before mip_reboot and enabled after mip_reboot.
*/
void mip_reboot(struct mip_ts_info *info)
{
    struct i2c_adapter *adapter = to_i2c_adapter(info->client->dev.parent);
    MIP_INFO("%s [START]\n", __func__);
    i2c_lock_adapter(adapter);
    mip_power_off(info);
    mip_power_on(info);
    i2c_unlock_adapter(adapter);
    MIP_INFO("%s [DONE]\n", __func__);
}

/**
* I2C Read (Normal)
*/
int mip_i2c_read_normal(struct mip_ts_info *info, char *write_buf,
                        unsigned int write_len, char *read_buf, unsigned int read_len)
{
    int res;
    struct i2c_msg msg[] = {
        {
            //.addr = info->client->addr,
            .addr = ((info->client->addr & I2C_MASK_FLAG) | I2C_ENEXT_FLAG),
            .flags = 0,
            .buf = write_buf,
            .len = write_len,
            .timing = MIP_I2C_SPEED
        }, {
            //.addr = info->client->addr,
            .addr = ((info->client->addr & I2C_MASK_FLAG) | I2C_ENEXT_FLAG),
            .flags = I2C_M_RD,
            .buf = read_buf,
            .len = read_len,
            .timing = MIP_I2C_SPEED
        },
    };
    MIP_DEBUG("%s [START]\n", __func__);
    res = i2c_transfer(info->client->adapter, msg, ARRAY_SIZE(msg));

    if(res == ARRAY_SIZE(msg))
        goto DONE;

    else if(res < 0)
        dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - errno[%d]\n",
                __func__, res);

    else if(res != ARRAY_SIZE(msg)) {
        dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - size[%u result[%d]\n",
                __func__, ARRAY_SIZE(msg), res);

    } else {
        dev_err(&info->client->dev, "%s [ERROR] unknown error [%d]\n",
                __func__, res);
    }

    goto ERROR;
ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
DONE:
    MIP_DEBUG("%s [DONE]\n", __func__);
    return 0;
}

/**
* I2C Read (DMA)
*/
int mip_i2c_read_dma(struct mip_ts_info *info, char *write_buf,
                     unsigned int write_len, char *read_buf, unsigned int read_len)
{
    int res;
    int i = 0;
    struct i2c_msg msg[] = {
        {
            //.addr = info->client->addr,
            .addr = ((info->client->addr & I2C_MASK_FLAG) | I2C_ENEXT_FLAG),
            .flags = 0,
            .buf = write_buf,
            .len = write_len,
            .timing = MIP_I2C_SPEED
        }, {
            //.addr = info->client->addr,
            .addr = ((info->client->addr & I2C_MASK_FLAG) | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
            .flags = I2C_M_RD,
            //.buf = read_buf,
            .buf = (unsigned char *)i2c_dma_buf_pa,
            .len = read_len,
            .timing = MIP_I2C_SPEED
        },
    };
    MIP_DEBUG("%s [START]\n", __func__);
    res = i2c_transfer(info->client->adapter, msg, ARRAY_SIZE(msg));

    if(res == ARRAY_SIZE(msg)) {
        for(i = 0; i < read_len; i++)
            read_buf[i] = i2c_dma_buf_va[i];

        goto DONE;

    } else if(res < 0)
        dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - errno[%d]\n",
                __func__, res);

    else if(res != ARRAY_SIZE(msg)) {
        dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - size[%u] result[%d]\n",
                __func__, ARRAY_SIZE(msg), res);

    } else {
        dev_err(&info->client->dev, "%s [ERROR] unknown error [%d]\n",
                __func__, res);
    }

    goto ERROR;
ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
DONE:
    MIP_DEBUG("%s [DONE]\n", __func__);
    return 0;
}

/**
* I2C Read
*/
int mip_i2c_read(struct mip_ts_info *info, char *write_buf,
                 unsigned int write_len, char *read_buf, unsigned int read_len)
{
    int retry = I2C_RETRY_COUNT;
    MIP_DEBUG("%s [START]\n", __func__);
    MIP_DEBUG("%s - write_len[%d] read_len[%d]\n", __func__, write_len, read_len);

    while(retry--) {
        if(read_len < MIP_I2C_DMA_LENGTH) {
            if(mip_i2c_read_normal(info, write_buf, write_len, read_buf, read_len) == 0)
                goto EXIT;

        } else {
            if(mip_i2c_read_dma(info, write_buf, write_len, read_buf, read_len) == 0)
                goto EXIT;
        }
    }

    goto ERROR;
ERROR:
#if RESET_ON_I2C_ERROR
    mip_reboot(info);
#endif
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return -1;
EXIT:
    MIP_DEBUG("%s [DONE]\n", __func__);
    return 0;
}

/**
* I2C Write (Normal)
*/
int mip_i2c_write_normal(struct mip_ts_info *info, char *write_buf,
                         unsigned int write_len)
{
    int res;
    struct i2c_msg msg[] = {
        {
            //.addr = info->client->addr,
            .addr = ((info->client->addr & I2C_MASK_FLAG) | I2C_ENEXT_FLAG),
            .flags = 0,
            .buf = write_buf,
            .len = write_len,
            .timing = MIP_I2C_SPEED
        },
    };
    dev_dbg(&info->client->dev, "%s [START]\n", __func__);
    res = i2c_transfer(info->client->adapter, msg, ARRAY_SIZE(msg));

    if(res == ARRAY_SIZE(msg))
        goto DONE;

    else if(res < 0)
        dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - errno[%d]\n",
                __func__, res);

    else if(res != ARRAY_SIZE(msg)) {
        dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - size[%u] result[%d]\n",
                __func__, ARRAY_SIZE(msg), res);

    } else {
        dev_err(&info->client->dev, "%s [ERROR] unknown error [%d]\n",
                __func__, res);
    }

    goto ERROR;
ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
DONE:
    MIP_DEBUG("%s [DONE]\n", __func__);
    return 0;
}

/**
* I2C Write (DMA)
*/
int mip_i2c_write_dma(struct mip_ts_info *info, char *write_buf,
                      unsigned int write_len)
{
    int res;
    int i = 0;
    struct i2c_msg msg[] = {
        {
            //.addr = info->client->addr,
            .addr = ((info->client->addr & I2C_MASK_FLAG) | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
            .flags = 0,
            //.buf = write_buf,
            .buf = (unsigned char *)i2c_dma_buf_pa,
            .len = write_len,
            .timing = MIP_I2C_SPEED
        },
    };
    MIP_DEBUG("%s [START]\n", __func__);

    for(i = 0; i < write_len; i++)
        i2c_dma_buf_va[i] = write_buf[i];

    res = i2c_transfer(info->client->adapter, msg, ARRAY_SIZE(msg));

    if(res == ARRAY_SIZE(msg))
        goto DONE;

    else if(res < 0)
        dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - errno[%d]\n",
                __func__, res);

    else if(res != ARRAY_SIZE(msg)) {
        dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - size[%u] result[%d]\n",
                __func__, ARRAY_SIZE(msg), res);

    } else {
        dev_err(&info->client->dev, "%s [ERROR] unknown error [%d]\n",
                __func__, res);
    }

    goto ERROR;
ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
DONE:
    MIP_DEBUG("%s [DONE]\n", __func__);
    return 0;
}

/**
* I2C Write
*/
int mip_i2c_write(struct mip_ts_info *info, char *write_buf,
                  unsigned int write_len)
{
    int retry = I2C_RETRY_COUNT;
    MIP_DEBUG("%s [START]\n", __func__);
    MIP_DEBUG("%s - write_len[%d]\n", __func__, write_len);

    while(retry--) {
        if(write_len < MIP_I2C_DMA_LENGTH) {
            if(mip_i2c_write_normal(info, write_buf, write_len) == 0)
                goto EXIT;

        } else {
            if(mip_i2c_write_dma(info, write_buf, write_len) == 0)
                goto EXIT;
        }
    }

    goto ERROR;
ERROR:
#if RESET_ON_I2C_ERROR
    mip_reboot(info);
#endif
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
EXIT:
    MIP_DEBUG("%s [DONE]\n", __func__);
    return 0;
}

/**
* MTK tpd interrupt
*/
static irqreturn_t mip_tpd_interrupt_handler(int irq, void *dev_id)
{
    MIP_DEBUG("%s [START]\n", __func__);
    mtk_eint_count++;
    MIP_DEBUG("%s - eint_count[%d]\n", __func__, mtk_eint_count);
    mtk_eint_flag = 1;
    wake_up_interruptible(&waiter);
    MIP_DEBUG("%s [DONE]\n", __func__);
    return IRQ_HANDLED;
}

static int tpd_irq_registration(struct mip_ts_info *info)
{
    struct device_node *node = NULL;
    int ret = 0;
    u32 ints[2] = { 0, 0 };
    unsigned int touch_irq = -1;
    MIP_INFO("Device Tree Tpd_irq_registration!\n");
    node = of_find_matching_node(node, touch_of_match);

    if (node) {
        of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
        gpio_set_debounce(ints[0], ints[1]);
        touch_irq = irq_of_parse_and_map(node, 0);
        ret = request_irq(touch_irq,  mip_tpd_interrupt_handler,
                          IRQF_TRIGGER_FALLING, "TOUCH_PANEL-eint", NULL);

        if (ret > 0) {
            ret = -1;
            MIP_INFO("tpd request_irq IRQ LINE NOT AVAILABLE!.");
        }

    } else {
        MIP_INFO("tpd request_irq can not find touch eint device node!.");
        ret = -1;
    }

    info->irq = touch_irq;
    MIP_INFO("[%s]irq:%d, debounce:%d-%d:\n", __func__,
             touch_irq, ints[0], ints[1]);
    return ret;
}

/**
* Enable device
*/
int mip_enable(struct mip_ts_info *info)
{
    MIP_DEBUG("%s [START]\n", __func__);
    MIP_INFO("%s info->enabled = %d\n", __func__, info->enabled);

    if (info->enabled) {
        dev_err(&info->client->dev, "%s [ERROR] device already enabled\n",
                __func__);
        goto EXIT;
    }

    mip_power_on(info);

    if(info->disable_esd == true) {
        //Disable ESD alert
        mip_disable_esd_alert(info);
    }

    mutex_lock(&info->lock);
    //enable_irq(info->client->irq);
    mip_irq_enable(info);
    info->enabled = true;
    mutex_unlock(&info->lock);
EXIT:
    MIP_INFO(MIP_DEVICE_NAME" - Enabled\n");
    MIP_DEBUG("%s [DONE]\n", __func__);
    return 0;
}

/**
* Disable device
*/
int mip_disable(struct mip_ts_info *info)
{
    MIP_DEBUG("%s [START]\n", __func__);
    MIP_INFO("%s info->enabled = %d\n", __func__, info->enabled);

    if (!info->enabled) {
        dev_err(&info->client->dev, "%s [ERROR] device already disabled\n",
                __func__);
        goto EXIT;
    }

    mutex_lock(&info->lock);
    //disable_irq(info->client->irq);
    mip_irq_disable(info);
    mip_power_off(info);
    mutex_unlock(&info->lock);
    info->enabled = false;
EXIT:
    MIP_INFO(MIP_DEVICE_NAME" - Disabled\n");
    MIP_DEBUG("%s [DONE]\n", __func__);
    return 0;
}

#if defined (MIP_FLIP_COVER_SWITCH)

void mip_flip_cover_read_reg(void)
{
    u8 wbuf[16];
    u8 rbuf[16];
    int ret = 0;

    if(mip_info == NULL)
        return;

    wbuf[0] = 0x06;
    wbuf[1] = 0x1B;
    ret = mip_i2c_read(mip_info, wbuf, 2, rbuf, 1);
    MIP_INFO("%s ret = %d rbuf[0] = %d\n", __func__, ret, rbuf[0]);
}

void mip_flip_cover_write_reg(int isEnable)
{
    u8 wbuf[16];
    int ret = 0;

    if(mip_info == NULL)
        return;

    wbuf[0] = 0x06;
    wbuf[1] = 0x1B;
    wbuf[2] = isEnable;
    ret = mip_i2c_write(mip_info, wbuf, 3);
    MIP_INFO("%s ret = %d isEnable = %d\n", __func__, ret, isEnable);
}

void mip_flip_cover_update(void)
{
    MIP_INFO("%s smart_cover_flag = %d mip_cover_state = %d\n", __func__,
             smart_cover_flag, mip_cover_state);

    if(smart_cover_flag == 0)
        return;

    if(mip_cover_state)
        mip_flip_cover_write_reg(FLIP_COVER_MODE_OFF);

    else
        mip_flip_cover_write_reg(FLIP_COVER_MODE_ON);

    mip_flip_cover_read_reg();
}

void mip_flip_cover_flag_charge(int flag)
{
    MIP_INFO("%s flag = %d\n", __func__, flag);

    if(smart_cover_flag == 0)
        mip_flip_cover_write_reg(0);

    else if(smart_cover_flag == 1 && mip_cover_state == 0)
        mip_flip_cover_write_reg(1);

    mip_flip_cover_read_reg();
}

void mip_flip_cover_switch(int state)
{
    MIP_INFO("%s state = %d\n", __func__, state);
    mip_cover_state = state;
    mip_flip_cover_update();
}

#endif

#if MIP_USE_INPUT_OPEN_CLOSE
/**
* Open input device
*/
static int mip_input_open(struct input_dev *dev)
{
    struct mip_ts_info *info = input_get_drvdata(dev);
    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    if(info->init == true)
        info->init = false;

    else
        mip_enable(info);

    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;
}

/**
* Close input device
*/
static void mip_input_close(struct input_dev *dev)
{
    struct mip_ts_info *info = input_get_drvdata(dev);
    dev_dbg(&info->client->dev, "%s [START]\n", __func__);
    mip_disable(info);
    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return;
}
#endif

/**
* Get ready status
*/
int mip_get_ready_status(struct mip_ts_info *info)
{
    u8 wbuf[16];
    u8 rbuf[16];
    int ret = 0;
    MIP_DEBUG("%s [START]\n", __func__);
    wbuf[0] = MIP_R0_CTRL;
    wbuf[1] = MIP_R1_CTRL_READY_STATUS;

    if(mip_i2c_read(info, wbuf, 2, rbuf, 1)) {
        dev_err(&info->client->dev, "%s [ERROR] mip_i2c_read\n", __func__);
        goto ERROR;
    }

    ret = rbuf[0];

    //check status
    if((ret == MIP_CTRL_STATUS_NONE) 
            || (ret == MIP_CTRL_STATUS_LOG)
            || (ret == MIP_CTRL_STATUS_READY)) {
        MIP_DEBUG("%s - status [0x%02X]\n", __func__, ret);
    } else {
        dev_err(&info->client->dev, "%s [ERROR] Unknown status [0x%02X]\n",
                __func__, ret);
        goto ERROR;
    }

    if(ret == MIP_CTRL_STATUS_LOG) {
        //skip log event
        wbuf[0] = MIP_R0_LOG;
        wbuf[1] = MIP_R1_LOG_TRIGGER;
        wbuf[2] = 0;

        if(mip_i2c_write(info, wbuf, 3))
            dev_err(&info->client->dev, "%s [ERROR] mip_i2c_write\n", __func__);
    }

    MIP_DEBUG("%s [DONE]\n", __func__);
    return ret;
ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return -1;
}

/**
* Read chip firmware version
*/
int mip_get_fw_version(struct mip_ts_info *info, u8 *ver_buf)
{
    u8 rbuf[8];
    u8 wbuf[2];
    int i;
    wbuf[0] = MIP_R0_INFO;
    wbuf[1] = MIP_R1_INFO_VERSION_BOOT;

    if(mip_i2c_read(info, wbuf, 2, rbuf, 8))
        goto ERROR;;

    for(i = 0; i < MIP_FW_MAX_SECT_NUM; i++) {
        ver_buf[0 + i * 2] = rbuf[1 + i * 2];
        ver_buf[1 + i * 2] = rbuf[0 + i * 2];
    }

    return 0;
ERROR:
    memset(ver_buf, 0xFF, sizeof(*ver_buf));
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return -1;
}

/**
* Read chip firmware version for u16
*/
int mip_get_fw_version_u16(struct mip_ts_info *info, u16 *ver_buf_u16)
{
    u8 rbuf[8];
    int i;

    if(mip_get_fw_version(info, rbuf))
        goto ERROR;

    for(i = 0; i < MIP_FW_MAX_SECT_NUM; i++)
        ver_buf_u16[i] = (rbuf[0 + i * 2] << 8) | rbuf[1 + i * 2];

    return 0;
ERROR:
    memset(ver_buf_u16, 0xFFFF, sizeof(*ver_buf_u16));
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return -1;
}

/**
* Read bin(file) firmware version
*/
int mip_get_fw_version_from_bin(struct mip_ts_info *info, u8 *ver_buf)
{
    const struct firmware *fw;
    dev_dbg(&info->client->dev, "%s [START]\n", __func__);
    request_firmware(&fw, FW_PATH_INTERNAL, &info->client->dev);

    if(!fw) {
        dev_err(&info->client->dev, "%s [ERROR] request_firmware\n", __func__);
        goto ERROR;
    }

    if(mip_bin_fw_version(info, fw->data, fw->size, ver_buf)) {
        memset(ver_buf, 0xFF, sizeof(*ver_buf));
        dev_err(&info->client->dev, "%s [ERROR] mip_bin_fw_version\n", __func__);
        goto ERROR;
    }

    release_firmware(fw);
    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;
ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
}

/**
* Set power state
*/
int mip_set_power_state(struct mip_ts_info *info, u8 mode)
{
    u8 wbuf[3];
    wbuf[0] = MIP_R0_CTRL;
    wbuf[1] = MIP_R1_CTRL_POWER_STATE;
    wbuf[2] = mode;
    MIP_DEBUG("%s [START]\n", __func__);
    MIP_DEBUG("%s - mode[%02X]\n", __func__, mode);

    if(mip_i2c_write(info, wbuf, 3)) {
        dev_err(&info->client->dev, "%s [ERROR] mip_i2c_write\n", __func__);
        goto ERROR;
    }

    MIP_DEBUG("%s [DONE]\n", __func__);
    return 0;
ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
}

/**
* Set wake-up gesture type
*/
int mip_set_wakeup_gesture_type(struct mip_ts_info *info, u32 type)
{
    u8 wbuf[6];
    MIP_DEBUG("%s [START]\n", __func__);
    MIP_DEBUG("%s - type[%08X]\n", __func__, type);
    wbuf[0] = MIP_R0_CTRL;
    wbuf[1] = MIP_R1_CTRL_GESTURE_TYPE;
    wbuf[2] = (type >> 24) & 0xFF;
    wbuf[3] = (type >> 16) & 0xFF;
    wbuf[4] = (type >> 8) & 0xFF;
    wbuf[5] = type & 0xFF;

    if(mip_i2c_write(info, wbuf, 6)) {
        dev_err(&info->client->dev, "%s [ERROR] mip_i2c_write\n", __func__);
        goto ERROR;
    }

    MIP_DEBUG("%s [DONE]\n", __func__);
    return 0;
ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
}

/**
* Disable ESD alert
*/
int mip_disable_esd_alert(struct mip_ts_info *info)
{
    u8 wbuf[4];
    u8 rbuf[4];
    dev_dbg(&info->client->dev, "%s [START]\n", __func__);
    wbuf[0] = MIP_R0_CTRL;
    wbuf[1] = MIP_R1_CTRL_DISABLE_ESD_ALERT;
    wbuf[2] = 1;

    if(mip_i2c_write(info, wbuf, 3)) {
        dev_err(&info->client->dev, "%s [ERROR] mip_i2c_write\n", __func__);
        goto ERROR;
    }

    if(mip_i2c_read(info, wbuf, 2, rbuf, 1)) {
        dev_err(&info->client->dev, "%s [ERROR] mip_i2c_read\n", __func__);
        goto ERROR;
    }

    if(rbuf[0] != 1) {
        dev_dbg(&info->client->dev, "%s [ERROR] failed\n", __func__);
        goto ERROR;
    }

    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;
ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
}

/**
* Alert event handler - ESD
*/
static int mip_alert_handler_esd(struct mip_ts_info *info, u8 *rbuf)
{
    u8 frame_cnt = rbuf[1];
    dev_dbg(&info->client->dev, "%s [START]\n", __func__);
    dev_dbg(&info->client->dev, "%s - frame_cnt[%d]\n", __func__, frame_cnt);

    if(frame_cnt == 0) {
        //sensor crack, not ESD
        info->esd_cnt++;
        dev_dbg(&info->client->dev, "%s - esd_cnt[%d]\n", __func__, info->esd_cnt);

        if(info->disable_esd == true) {
            mip_disable_esd_alert(info);
            info->esd_cnt = 0;

        } else if(info->esd_cnt > ESD_COUNT_FOR_DISABLE) {
            //Disable ESD alert
            if(mip_disable_esd_alert(info)) {
            } else {
                info->disable_esd = true;
                info->esd_cnt = 0;
            }

        } else {
            //Reset chip
            mip_reboot(info);
        }

    } else {
        //ESD detected
        //Reset chip
        mip_reboot(info);
        info->esd_cnt = 0;
    }

    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;
    //ERROR:
    //dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    //return 1;
}

/**
* Alert event handler - Wake-up
*/
static int mip_alert_handler_wakeup(struct mip_ts_info *info, u8 *rbuf)
{
    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    if(mip_wakeup_event_handler(info, rbuf))
        goto ERROR;

    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;
ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
}

/**
* Alert event handler - Input type
*/
static int mip_alert_handler_inputtype(struct mip_ts_info *info, u8 *rbuf)
{
    MIP_DEBUG("%s [START]\n", __func__);
    //...
    MIP_DEBUG("%s [DONE]\n", __func__);
    return 0;
}

/**
* Interrupt handler
*/
/*static irqreturn_t mip_interrupt(int irq, void *dev_id) {
    struct mip_ts_info *info = dev_id;
    struct i2c_client *client = info->client;
    u8 wbuf[8];
    u8 rbuf[256];
    unsigned int size = 0;
    //int event_size = info->event_size;
    u8 category = 0;
    u8 alert_type = 0;
    printk("[TCL][MARK]%s [START]\n", __func__);
    dev_dbg(&client->dev, "%s [START]\n", __func__);

    //Read packet info
    wbuf[0] = MIP_R0_EVENT;
    wbuf[1] = MIP_R1_EVENT_PACKET_INFO;
    if(mip_i2c_read(info, wbuf, 2, rbuf, 1)) {
        dev_err(&client->dev, "%s [ERROR] Read packet info\n", __func__);
        goto ERROR;
    }

    size = (rbuf[0] & 0x7F);
    category = ((rbuf[0] >> 7) & 0x1);
    dev_dbg(&client->dev, "%s - packet info : size[%d] category[%d]\n", __func__, size, category);

    //Check size
    if(size <= 0) {
        dev_err(&client->dev, "%s [ERROR] Packet size [%d]\n", __func__, size);
        goto EXIT;
    }

    //Read packet data
    wbuf[0] = MIP_R0_EVENT;
    wbuf[1] = MIP_R1_EVENT_PACKET_DATA;
    if(mip_i2c_read(info, wbuf, 2, rbuf, size)) {
        dev_err(&client->dev, "%s [ERROR] Read packet data\n", __func__);
        goto ERROR;
    }

    //Event handler
    if(category == 0) {
        //Touch event
        info->esd_cnt = 0;

        mip_input_event_handler(info, size, rbuf);
    } else {
        //Alert event
        alert_type = rbuf[0];

        dev_dbg(&client->dev, "%s - alert type [%d]\n", __func__, alert_type);

        if(alert_type == MIP_ALERT_ESD) {
            //ESD detection
            if(mip_alert_handler_esd(info, rbuf)) {
                goto ERROR;
            }
        } else if(alert_type == MIP_ALERT_WAKEUP) {
            //Wake-up gesture
            if(mip_alert_handler_wakeup(info, rbuf)) {
                goto ERROR;
            }
        } else if(alert_type == MIP_ALERT_INPUT_TYPE) {
            //Input type changed
            if(mip_alert_handler_inputtype(info, rbuf)) {
                goto ERROR;
            }
        } else {
            dev_err(&client->dev, "%s [ERROR] Unknown alert type [%d]\n", __func__, alert_type);
            goto ERROR;
        }
    }

EXIT:
    dev_dbg(&client->dev, "%s [DONE]\n", __func__);
    return IRQ_HANDLED;

ERROR:
    if(RESET_ON_EVENT_ERROR) {
        dev_info(&client->dev, "%s - Reset on error\n", __func__);

        mip_disable(info);
        mip_clear_input(info);
        mip_enable(info);
    }

    dev_err(&client->dev, "%s [ERROR]\n", __func__);
    return IRQ_HANDLED;
}
*/

/**
* Interrupt handler for MTK
*/
int mip_interrupt_mtk(void *data)
{
    struct mip_ts_info *info = data;
    struct i2c_client *client = info->client;
    u8 wbuf[8];
    u8 rbuf[256];
    unsigned int size = 0;
    //int event_size = info->event_size;
    u8 category = 0;
    u8 alert_type = 0;
    struct sched_param param = {
        .sched_priority = RTPM_PRIO_TPD
    };
    sched_setscheduler(current, SCHED_RR, &param);
    MIP_DEBUG("%s [START]\n", __func__);

    do {
        MIP_DEBUG("%s - wait\n", __func__);
        set_current_state(TASK_INTERRUPTIBLE);
        wait_event_interruptible(waiter, mtk_eint_flag != 0);
        mtk_eint_flag = 0;
        MIP_DEBUG("%s - eint_count[%d]\n", __func__, mtk_eint_count);

        if(info->enabled == false) {
            MIP_INFO("%s - skip : enabled [false]\n", __func__);
            msleep(1);
            goto NEXT;
        }

        if(info->init == true) {
            info->init = false;
            MIP_INFO("%s - skip : init [false]\n", __func__);
            goto NEXT;
        }

        if(info->irq_pause == true) {
            MIP_INFO("%s - skip : irq_pause\n", __func__);
            goto NEXT;
        }

        set_current_state(TASK_RUNNING);
        MIP_DEBUG("%s - run\n", __func__);
        //Read packet info
        wbuf[0] = MIP_R0_EVENT;
        wbuf[1] = MIP_R1_EVENT_PACKET_INFO;

        if(mip_i2c_read(info, wbuf, 2, rbuf, 1)) {
            dev_err(&client->dev, "%s [ERROR] Read packet info\n", __func__);
            goto ERROR;
        }

        size = (rbuf[0] & 0x7F);
        category = ((rbuf[0] >> 7) & 0x1);
        MIP_DEBUG("%s - packet info : size[%d] category[%d]\n",
                  __func__, size, category);

        //Check size
        if(size <= 0) {
            dev_err(&client->dev, "%s [ERROR] Packet size [%d]\n",
                    __func__, size);
            goto NEXT;
        }

        //Read packet data
        wbuf[0] = MIP_R0_EVENT;
        wbuf[1] = MIP_R1_EVENT_PACKET_DATA;

        if(mip_i2c_read(info, wbuf, 2, rbuf, size)) {
            dev_err(&client->dev, "%s [ERROR] Read packet data\n", __func__);
            goto ERROR;
        }

        //Event handler
        if(category == 0) {
            //Touch event
            info->esd_cnt = 0;
            mip_input_event_handler(info, size, rbuf);

        } else {
            //Alert event
            alert_type = rbuf[0];
            MIP_INFO("%s - alert type [%d]\n", __func__, alert_type);

            if(alert_type == MIP_ALERT_ESD) {
                //ESD detection
                if(mip_alert_handler_esd(info, rbuf))
                    goto ERROR;

            } else if(alert_type == MIP_ALERT_WAKEUP) {
                //Wake-up gesture
                if(mip_alert_handler_wakeup(info, rbuf))
                    goto ERROR;

            } else if(alert_type == MIP_ALERT_INPUT_TYPE) {
                //Input type changed
                if(mip_alert_handler_inputtype(info, rbuf))
                    goto ERROR;

            } else {
                dev_err(&client->dev, "%s [ERROR] Unknown alert type [%d]\n", __func__,
                        alert_type);
                goto ERROR;
            }
        }

        goto NEXT;
ERROR:

        if(RESET_ON_EVENT_ERROR) {
            dev_info(&client->dev, "%s - Reset on error\n", __func__);
            mip_disable(info);
            mip_clear_input(info);
            mip_enable(info);
        }

        dev_err(&client->dev, "%s [ERROR]\n", __func__);
NEXT:
        mip_irq_enable(info);
        mtk_eint_count--;
    } while(!kthread_should_stop());

    info->irq_enabled = false;
    MIP_DEBUG("%s [DONE]\n", __func__);
    return 0;
}

/**
* Start interrupt handler for MTK
*/
int mip_interrupt_mtk_start(struct mip_ts_info *info)
{
    MIP_DEBUG("%s [START]\n", __func__);
    info->irq_pause = false;
    //mtk_eint_thread = kthread_run(mip_interrupt_mtk, info, TPD_DEVICE);
    mtk_eint_thread = kthread_run(mip_interrupt_mtk, info, MIP_DEVICE_NAME);

    if(IS_ERR(mtk_eint_thread)) {
        dev_err(&info->client->dev, "%s [ERROR] kthread_run\n", __func__);
        goto ERROR;
    }

    info->irq_enabled = true;
    MIP_DEBUG("%s [DONE]\n", __func__);
    return 0;
ERROR:
    info->irq_enabled = false;
    dev_dbg(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
}

/**
* Update firmware from kernel built-in binary
*/
int mip_fw_update_from_kernel(struct mip_ts_info *info)
{
    const char *fw_name = FW_PATH_INTERNAL;
    const struct firmware *fw;
    int retires = 3;
    int ret = fw_err_none;
    MIP_DEBUG("%s [START]\n", __func__);
    //Disable IRQ
    mutex_lock(&info->lock);
    //disable_irq(info->client->irq);
    mip_irq_disable(info);
    //Get firmware
    request_firmware(&fw, fw_name, &info->client->dev);

    if (!fw) {
        dev_err(&info->client->dev, "%s [ERROR] request_firmware\n", __func__);
        goto ERROR;
    }

    //Update firmware
    do {
        ret = mip_flash_fw(info, fw->data, fw->size, false, true);

        if(ret >= fw_err_none)
            break;
    } while (--retires);

    if (!retires) {
        dev_err(&info->client->dev, "%s [ERROR] mip_flash_fw failed\n", __func__);
        ret = fw_err_download;
    }

    release_firmware(fw);
    //Enable IRQ
    //enable_irq(info->client->irq);
    mip_irq_enable(info);
    mutex_unlock(&info->lock);

    if(ret < fw_err_none)
        goto ERROR;

    MIP_DEBUG("%s [DONE]\n", __func__);
    return 0;
ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return -1;
}

const u8 MELFAS_wind_binary[] = {
#include "cfg.i"
};

struct firmware melfas_fw = {
    .size = sizeof(MELFAS_wind_binary),
    .data = &MELFAS_wind_binary[0],
};
#if MIP_USE_AUTO_FW_UPDATE
static int mip_fw_update(struct mip_ts_info *info, struct i2c_client *client)
{
    int retry = 3;
    int ret;

    MIP_DEBUG("mip_fw_update_controller \n");

    if (get_boot_mode() == META_BOOT) {
        MIP_INFO("Ignore firmware update in meta mode!\n");
        return fw_err_none;
    }

    if (force_updated == true)
        MIP_INFO("FW in the flash was abnormal!\n");

    do {
        ret = mip_flash_fw(info, melfas_fw.data, melfas_fw.size,
                           force_updated, true);

        if(ret >= fw_err_none)
            break;
    } while (--retry);

    if (!retry && (force_updated == false)) {
        MIP_INFO("Failed to flash firmware after 3 retires!\n");
        MIP_INFO("Forced driver to upgrade frimware!\n");
        retry = 1;
        normal_update_failed = true;

        do {
            ret = mip_flash_fw(info, melfas_fw.data, melfas_fw.size,
                               true, true);

            if (ret >= fw_err_none)
                break;
        } while (--retry);

        if (!retry)
            MIP_INFO("Failed to force driver to upgrade firmware!\n");
    }

    return ret;
}
#endif
/**
* Update firmware from external storage
*/
int mip_fw_update_from_storage(struct mip_ts_info *info, char *path, bool force)
{
    struct file *fp;
    mm_segment_t old_fs;
    size_t fw_size, nread;
    int ret = fw_err_none;
    MIP_DEBUG("%s [START]\n", __func__);
    //Disable IRQ
    mutex_lock(&info->lock);
    //disable_irq(info->client->irq);
    mip_irq_disable(info);
    //Get firmware
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    fp = filp_open(path, O_RDONLY, S_IRUSR);

    if (IS_ERR(fp)) {
        dev_err(&info->client->dev, "%s [ERROR] file_open - path[%s]\n",
                __func__, path);
        ret = fw_err_file_open;
        goto ERROR;
    }

    fw_size = fp->f_path.dentry->d_inode->i_size;

    if (0 < fw_size) {
        //Read firmware
        unsigned char *fw_data;
        fw_data = kzalloc(fw_size, GFP_KERNEL);
        nread = vfs_read(fp, (char __user *)fw_data, fw_size, &fp->f_pos);
        MIP_INFO("%s - path[%s] size[%ld]\n", __func__, path, (long int)fw_size);

        if (nread != fw_size) {
            dev_err(&info->client->dev, "%s [ERROR] vfs_read - size[%ld] read[%ld]\n",
                    __func__, (long int)fw_size, (long int)nread);
            ret = fw_err_file_read;

        } else {
            //Update firmware
            ret = mip_flash_fw(info, fw_data, fw_size, force, true);
        }

        kfree(fw_data);

    } else {
        dev_err(&info->client->dev, "%s [ERROR] fw_size [%ld]\n",
                __func__, (long int)fw_size);
        ret = fw_err_file_read;
    }

    filp_close(fp, current->files);
ERROR:
    set_fs(old_fs);
    //Enable IRQ
    //enable_irq(info->client->irq);
    mip_irq_enable(info);
    mutex_unlock(&info->lock);

    if(ret < fw_err_none)
        dev_err(&info->client->dev, "%s [ERROR]\n", __func__);

    else
        MIP_DEBUG("%s [DONE]\n", __func__);

    return ret;
}

static ssize_t mip_sys_fw_update(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct mip_ts_info *info = i2c_get_clientdata(client);
    int result = 0;
    u8 data[255];
    int ret = 0;
    memset(info->print_buf, 0, PAGE_SIZE);
    MIP_DEBUG("%s [START]\n", __func__);
    ret = mip_fw_update_from_storage(info, info->fw_path_ext, true);

    switch(ret) {
    case fw_err_none:
        sprintf(data, "F/W update success.\n");
        break;

    case fw_err_uptodate:
        sprintf(data, "F/W is already up-to-date.\n");
        break;

    case fw_err_download:
        sprintf(data, "F/W update failed : Download error\n");
        break;

    case fw_err_file_type:
        sprintf(data, "F/W update failed : File type error\n");
        break;

    case fw_err_file_open:
        sprintf(data, "F/W update failed : File open error [%s]\n", info->fw_path_ext);
        break;

    case fw_err_file_read:
        sprintf(data, "F/W update failed : File read error\n");
        break;

    default:
        sprintf(data, "F/W update failed.\n");
        break;
    }

    MIP_DEBUG("%s [DONE]\n", __func__);
    strcat(info->print_buf, data);
    result = snprintf(buf, PAGE_SIZE, "%s\n", info->print_buf);
    return result;
}

static ssize_t mip_sys_fw_update_status(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf)
{
    return snprintf(buf, PAGE_SIZE, "Update status --> (%d %d)\n",
            force_updated, normal_update_failed);
}

static DEVICE_ATTR(fw_update, S_IRUSR, mip_sys_fw_update, NULL);
static DEVICE_ATTR(update_status, S_IRUGO, mip_sys_fw_update_status, NULL);
/**
* Sysfs attr info
*/
static struct attribute *mip_attrs[] = {
    &dev_attr_fw_update.attr,
    &dev_attr_update_status.attr,
    NULL,
};

/**
* Sysfs attr group info
*/
static const struct attribute_group mip_attr_group = {
    .attrs = mip_attrs,
};

static int mip_read_test(struct mip_ts_info *info)
{
    int ret = 0;
    u8 rbuf[64];
    ret = mip_get_fw_version(info, rbuf);

    if (ret == 0) {
        MIP_INFO("%s - F/W Version : "
                 "%02X.%02X %02X.%02X %02X.%02X %02X.%02X\n", __func__, rbuf[0],
                 rbuf[1], rbuf[2], rbuf[3], rbuf[4], rbuf[5], rbuf[6], rbuf[7]);
        if (((rbuf[4] == 0xFF) && (rbuf[5] == 0xFF))
                || ((rbuf[6] == 0xFF) && (rbuf[7] == 0xFF))) {
            force_updated = true;
        }

    } else
        MIP_INFO("[ERROR] mip read test failed!\n");

    return ret;
}


/**
* Initial config
*/
static int mip_init_config(struct mip_ts_info *info)
{
    u8 wbuf[8];
    u8 rbuf[64];
    int ret = 0;
    MIP_DEBUG("%s [START]\n", __func__);
    //Product name
    wbuf[0] = MIP_R0_INFO;
    wbuf[1] = MIP_R1_INFO_PRODUCT_NAME;
    ret = mip_i2c_read(info, wbuf, 2, rbuf, 16);

    if(ret)
        goto ERROR;

    memcpy(info->product_name, rbuf, 16);
    MIP_INFO("%s - product_name[%s]\n", __func__, info->product_name);
    //Firmware version
    ret = mip_get_fw_version(info, rbuf);

    if(ret) {
        MIP_INFO("mip_get_fw_version err ret = %d\n", ret);
        goto ERROR;
    }

    memcpy(info->fw_version, rbuf, 8);
    MIP_INFO("%s - F/W Version : %02X.%02X %02X.%02X %02X.%02X %02X.%02X\n",
             __func__,
             info->fw_version[0], info->fw_version[1], info->fw_version[2],
             info->fw_version[3], info->fw_version[4], info->fw_version[5],
             info->fw_version[6], info->fw_version[7]);
    gtp_fw_version = rbuf[0] << 24 | rbuf[1] << 16 | rbuf[2] << 8 | rbuf[3];
    gtp_cfg_version = rbuf[4] << 24 | rbuf[5] << 16 | rbuf[6] << 8 | rbuf[7];
    //Resolution
    wbuf[0] = MIP_R0_INFO;
    wbuf[1] = MIP_R1_INFO_RESOLUTION_X;
    ret = mip_i2c_read(info, wbuf, 2, rbuf, 7);

    if(ret)
        goto ERROR;

#if MIP_AUTOSET_RESOLUTION
    //Set resolution using chip info
    info->max_x = (rbuf[0]) | (rbuf[1] << 8);
    info->max_y = (rbuf[2]) | (rbuf[3] << 8);
#else
    //Set resolution using platform data
    info->max_x = 720;//info->pdata->max_x;
    info->max_y = 1280;//info->pdata->max_y;
#endif
    MIP_INFO("%s - max_x[%d] max_y[%d]\n", __func__, info->max_x, info->max_y);
    //Node info
    info->node_x = rbuf[4];
    info->node_y = rbuf[5];
    info->node_key = rbuf[6];
    MIP_INFO("%s - node_x[%d] node_y[%d] node_key[%d]\n", __func__,
             info->node_x, info->node_y, info->node_key);

    //Key info
    if(info->node_key > 0) {
        //Enable touchkey
        info->key_enable = true;
        info->key_num = info->node_key;
    }

    //Protocol
#if MIP_AUTOSET_EVENT_FORMAT
    wbuf[0] = MIP_R0_EVENT;
    wbuf[1] = MIP_R1_EVENT_SUPPORTED_FUNC;
    mip_i2c_read(info, wbuf, 2, rbuf, 7);
    info->event_format = (rbuf[4]) | (rbuf[5] << 8);
    info->event_size = rbuf[6];
#else
    info->event_format = 0;
    info->event_size = 6;
#endif
    MIP_INFO("%s - event_format[%d] event_size[%d] \n",
             __func__, info->event_format, info->event_size);
    MIP_DEBUG("%s [DONE]\n", __func__);
    return 0;
ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return -1;
}

/**
* Initialize driver
*/
static int mip_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct mip_ts_info *info;
    struct input_dev *input_dev;
    int ret = 0;
    MIP_INFO("%s [START]\n", __func__);

    if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "[ERROR] i2c_check_functionality\n");
        return -EIO;
    }

    //Init info data
    info = kzalloc(sizeof(struct mip_ts_info), GFP_KERNEL);

    if (!info) {
        dev_err(&client->dev, "Failed to malloc mem for mip4 info!\n");
        return -ENOMEM;

    } else
        mip_info = info;

    info->client = client;
    info->irq = -1;
    info->init = true;
    info->power_state = -1;
    info->fw_path_ext = kstrdup(FW_PATH_EXTERNAL, GFP_KERNEL);
    mutex_init(&info->lock);
    //Power on
    ret = regulator_set_voltage(tpd->reg, 2800000, 2800000);
    ret += regulator_enable(tpd->reg);

    if (ret) {
        dev_err(&client->dev, "Could not set regulator to 2800mv.\n");
        goto POWER_ON_ERROR;
    }

    //Config GPIO
    mip_config_gpio(info);
//    tpd_gpio_as_int(GTP_INT_PORT);
    msleep(100);
    mip_reboot(info);
    ret = mip_read_test(info);

    if(ret)
        goto READ_TEST_ERROR;

    //Init input device
    input_dev = input_allocate_device();

    if (!input_dev) {
        dev_err(&client->dev, "Failed to malloc mem for input device!\n");
        ret = -ENOMEM;
        goto INPUT_DEV_DERROR;

    } else
        info->input_dev = input_dev;

    info->input_dev->name = "MELFAS_" CHIP_NAME "_Touchscreen";
    snprintf(info->phys, sizeof(info->phys), "%s/input1", info->input_dev->name);
    info->input_dev->phys = info->phys;
    info->input_dev->id.bustype = BUS_I2C;
    info->input_dev->dev.parent = &client->dev;
#if MIP_USE_INPUT_OPEN_CLOSE
    info->input_dev->open = mip_input_open;
    info->input_dev->close = mip_input_close;
#endif
    //Set info data
    input_set_drvdata(input_dev, info);
    i2c_set_clientdata(client, info);
	tpd->dev->dev.coherent_dma_mask = 0xffffffff;
    //Set I2C DMA
    if(i2c_dma_buf_va == NULL)
        i2c_dma_buf_va = (u8 *)dma_alloc_coherent(&tpd->dev->dev, 4096,
                         &i2c_dma_buf_pa, GFP_KERNEL);

    if(!i2c_dma_buf_va) {
        dev_err(&client->dev, "%s [ERROR] dma_alloc_coherent\n", __func__);
        ret = -ENOMEM;
        goto DMA_BUF_ERROR;
    }

    //Firmware update
#if MIP_USE_AUTO_FW_UPDATE
    ret = mip_fw_update(info, client);

    if(ret < fw_err_none) {
        dev_err(&client->dev, "%s [ERROR] mip_fw_update_from_kernel\n", __func__);
        goto FW_UPDATE_ERROR;
    }

#endif
    //Initial config
    ret = mip_init_config(info);

    if(ret) {
        dev_err(&client->dev, "mip_init_config err ret = %d\n", ret);
        goto INIT_CONFIG_ERROR;
    }

    MIP_DEBUG("gtp_fw_version 0x%x gtp_cfg_version = 0x%x\n",
              gtp_fw_version, gtp_cfg_version);
    //Config input interface
    mip_config_input(info);
    //Register input device
    ret = input_register_device(input_dev);

    if (ret) {
        dev_err(&client->dev, "%s [ERROR] input_register_device\n", __func__);
        goto INPUT_REGISTER_ERROR;
    }

    //Set interrupt handler
    tpd_irq_registration(info);
    //Enable device
    mip_enable(info);
    //disable_irq(client->irq);
    //mip_irq_disable(info);
    mip_interrupt_mtk_start(info);
    tpd_load_status = 1;
	//zhongzhu add start for TP type indentify	in tct_common
	gtp_sensor_id = 0x48;
	//zhongzhu add end for TP type indentify  in tct_common
#if defined (MIP_FLIP_COVER_SWITCH)
    //hall_state_charge_notify = mip_flip_cover_switch;
    //smart_cover_flag_charge_notify = mip_flip_cover_flag_charge;
#endif
#if MIP_USE_DEV

    //Create dev node (optional)
    if(mip_dev_create(info))
        dev_err(&client->dev, "%s [ERROR] mip_dev_create\n", __func__);

    //Create dev
    info->class = class_create(THIS_MODULE, MIP_DEVICE_NAME);
    device_create(info->class, NULL, info->mip_dev, NULL, MIP_DEVICE_NAME);
#endif
#if MIP_USE_SYS
    //Create sysfs for test mode (optional)
    if (mip_sysfs_create(info)) {
           dev_err(&client->dev, "%s [ERROR] mip_sysfs_create\n", __func__);
     }
#endif

    //Create sysfs
    if (sysfs_create_group(&client->dev.kobj, &mip_attr_group))
        dev_err(&client->dev, "%s [ERROR] sysfs_create_group\n", __func__);

    if (sysfs_create_link(NULL, &client->dev.kobj, MIP_DEVICE_NAME))
        dev_err(&client->dev, "%s [ERROR] sysfs_create_link\n", __func__);

    MIP_INFO("%s [DONE]\n", __func__);
    MIP_INFO("MELFAS " CHIP_NAME " Touchscreen is initialized successfully.\n");
    return 0;
INPUT_REGISTER_ERROR:
INIT_CONFIG_ERROR:
#if MIP_USE_AUTO_FW_UPDATE
FW_UPDATE_ERROR:
#endif
    dma_free_coherent(&tpd->dev->dev, 4096, i2c_dma_buf_va, i2c_dma_buf_pa);
DMA_BUF_ERROR:
    input_set_drvdata(input_dev, NULL);
    i2c_set_clientdata(client, NULL);
    input_free_device(input_dev);
INPUT_DEV_DERROR:
READ_TEST_ERROR:
    regulator_disable(tpd->reg);
    regulator_put(tpd->reg);
POWER_ON_ERROR:
    kfree(info);
    MIP_INFO("MELFAS " CHIP_NAME " Touchscreen initialization failed.\n");
    return ret;
}

/**
* Remove driver
*/
static int mip_remove(struct i2c_client *client)
{
    struct mip_ts_info *info = i2c_get_clientdata(client);

    sysfs_remove_link(NULL, MIP_DEVICE_NAME);
    sysfs_remove_group(&info->client->dev.kobj, &mip_attr_group);
#if MIP_USE_SYS
    mip_sysfs_remove(info);
#endif
#if MIP_USE_DEV
    device_destroy(info->class, info->mip_dev);
    class_destroy(info->class);
#endif

    //IRQ
    if (info->irq >= 0)
        free_irq(info->irq, NULL);

    input_unregister_device(info->input_dev);

    //I2C DMA
    if(i2c_dma_buf_va) {
        dma_free_coherent(NULL, 4096, i2c_dma_buf_va, i2c_dma_buf_pa);
        i2c_dma_buf_va = NULL;
        i2c_dma_buf_pa = 0;
    }

    if (info->input_dev)
        input_free_device(info->input_dev);

    kfree(info);
    return 0;
}

#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
/**
* Device suspend event handler
*/
int mip_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct mip_ts_info *info = i2c_get_clientdata(client);
	int ret = 0;
	
    MIP_INFO("%s [START]\n", __func__);
#if defined (MIP_FLIP_COVER_SWITCH)
    mip_flip_cover_update();
#endif
#if MIP_USE_WAKEUP_GESTURE
    mip_gesture_wakeup_flag = gesture_wakeup_flag;
    MIP_INFO("mip_gesture_wakeup_flag = %d\n", mip_gesture_wakeup_flag);

    if(mip_gesture_wakeup_flag) {
        info->wakeup_gesture_code = 0;
        //mip_set_wakeup_gesture_type(info, MIP_EVENT_GESTURE_ALL);
        mip_set_power_state(info, MIP_CTRL_POWER_LOW);
        info->nap_mode = true;
        MIP_INFO("%s - nap mode : on\n", __func__);

    } else {
        MIP_INFO("MIP CTRL POWER LOW\n");
        mip_disable(info);
		ret = regulator_disable(tpd->reg);
        if(ret)
	       printk("Could not set regulator_disable to 2800mv.\n");
    }

#else
    mip_disable(info);
#endif
    mip_clear_input(info);
    MIP_INFO("%s [DONE]\n", __func__);
    return 0;
}

/**
* Device resume event handler
*/
int mip_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct mip_ts_info *info = i2c_get_clientdata(client);
    int ret = 0;

    MIP_INFO("%s [START]\n", __func__);
#if MIP_USE_WAKEUP_GESTURE
    MIP_INFO("gesture_wakeup_flag = %d mip_gesture_wakeup_flag = %d\n",
           gesture_wakeup_flag, mip_gesture_wakeup_flag);

    if(mip_gesture_wakeup_flag) {
//start,TP doesn't work sometimes when resume the system by power key ,PR 2601228
		mip_power_off(info);
        mip_power_on(info);
//end,TP doesn't work sometimes when resume the system by power key ,PR 2601228
        mip_set_power_state(info, MIP_CTRL_POWER_ACTIVE);
        info->nap_mode = false;
        MIP_INFO("%s - nap mode : off\n", __func__);

    } else {
        mip_power_off(info);
		ret = regulator_enable(tpd->reg);
		if(ret)		
			printk("Could not set regulator_enable to 2800mv.\n");
		mip_power_off(info);
        mip_power_on(info);
        mip_enable(info);
    }

#else
    mip_enable(info);
#endif
#if defined (MIP_FLIP_COVER_SWITCH)
    mip_flip_cover_update();
#endif
    MIP_INFO("%s [DONE]\n", __func__);
    return ret;
}
#endif

#if 1 //CONFIG_HAS_EARLYSUSPEND
/**
* Early suspend handler
*/
void mip_early_suspend(struct device *h)
{
    struct mip_ts_info *info = mip_info;
    mip_suspend(&info->client->dev);
}

/**
* Late resume handler
*/
void mip_late_resume(struct device *h)
{
    struct mip_ts_info *info = mip_info;
    mip_resume(&info->client->dev);
}
#endif



#if MIP_USE_DEVICETREE
/**
* Device tree match table
*/
static const struct of_device_id mip_match_table[] = {
    {.compatible = "mediatek,cap_touch1"},
    {},
};
MODULE_DEVICE_TABLE(of, mip_match_table);
#endif

/**
* I2C Device ID
*/
static const struct i2c_device_id mip_id[] = {
    {MIP_DEVICE_NAME, 0},
};
MODULE_DEVICE_TABLE(i2c, mip_id);

/**
* I2C detect
*/
static int mip_detect (struct i2c_client *client, struct i2c_board_info *info)
{
    strcpy(info->type, TPD_DEVICE);
    return 0;
}

static unsigned short force[] = { 0, 0x48, I2C_CLIENT_END, I2C_CLIENT_END };
static const unsigned short *const forces[] = { force, NULL };
/**
* I2C driver info
*/
static struct i2c_driver mip_driver = {
    .id_table = mip_id,
    .probe = mip_probe,
    .remove = mip_remove,
    .detect = mip_detect,
    .address_list = (const unsigned short *)forces,
    .driver = {
        .name = MIP_DEVICE_NAME,
        //        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = mip_match_table,
#endif
    },
};




/**
* MTK tpd local init
*/
static int mip_tpd_local_init(void)
{
    MIP_INFO("%s [START]\n", __func__);
    tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");

    if (tpd_load_status == 1) {
        MIP_INFO("A suitable touch driver was already found!\n");
        return -1;
    }

    if(i2c_add_driver(&mip_driver) != 0) {
        MIP_INFO("%s [ERROR] i2c_add_driver\n", __func__);
        goto ERROR;
    }

    if (tpd_load_status == 0) {
        i2c_del_driver(&mip_driver);
        MIP_INFO("Failed to add mip4 touch driver!\n");
        goto ERROR;
    }

#ifdef TPD_HAVE_BUTTON
    tpd_button_setting(TPD_KEY_COUNT, mip_tpd_keys_local, mip_tpd_keys_dim_local);
#endif
    tpd_type_cap = 1;
    MIP_INFO("%s [DONE]\n", __func__);
    return 0;
ERROR:
    MIP_INFO("%s [ERROR]\n", __func__);
    return -1;
}

/**
* MTK tpd driver info
*/
static struct tpd_driver_t mip_tpd_driver = {
    .tpd_device_name = MIP_DEVICE_NAME,
    .tpd_local_init = mip_tpd_local_init,
    .suspend = mip_early_suspend,
    .resume = mip_late_resume,
#ifdef TPD_HAVE_BUTTON
    .tpd_have_button = 1,
#else
    .tpd_have_button = 0,
#endif
};

/**
* MTK tpd driver init
*/
static int __init tpd_driver_init(void)
{
    MIP_INFO("MediaTek Melfas touch panel driver init!\n");
    tpd_get_dts_info();

    if(tpd_driver_add(&mip_tpd_driver) < 0)
        MIP_INFO("%s [ERROR] tpd_driver_add\n", __func__);

    return 0;
}

/**
* MTK tpd driver exit
*/
static void __exit tpd_driver_exit(void)
{
    MIP_DEBUG("%s [START]\n", __func__);

    //input_unregister_device(tpd->dev);
    if(tpd_driver_remove(&mip_tpd_driver))
        MIP_INFO("%s [ERROR] tpd_driver_remove\n", __func__);

    MIP_DEBUG("%s [DONE]\n", __func__);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);


MODULE_DESCRIPTION("MELFAS MIP4 Touchscreen for MediaTek");
MODULE_VERSION("2015.06.22");
MODULE_AUTHOR("Jee, SangWon <jeesw@melfas.com>");
MODULE_LICENSE("GPL");

