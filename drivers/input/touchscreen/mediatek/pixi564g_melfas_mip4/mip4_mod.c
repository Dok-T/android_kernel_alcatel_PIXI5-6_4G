/*
 * MELFAS MIP4 Touchscreen for MediaTek
 *
 * Copyright (C) 2015 MELFAS Inc.
 *
 *
 * mip4_mod.c : Model dependent functions
 *
 */

#include "mip4.h"

/**
* MTK
*/
extern struct tpd_device *tpd;
extern int mip_tpd_keys_local[TPD_KEY_COUNT];

#ifdef TPD_HAVE_BUTTON
    extern int mip_tpd_keys_dim_local[TPD_KEY_COUNT][4];
#endif

/**
* Enable IRQ
*/
void mip_irq_enable(struct mip_ts_info *info)
{
    MIP_DEBUG("%s [START]\n", __func__);

    /*
    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
    */
    if (info->irq_pause) {
        enable_irq(info->client->irq);
        info->irq_pause = false;
    }

    MIP_DEBUG("%s [DONE]\n", __func__);
}

/**
* Disable IRQ
*/
void mip_irq_disable(struct mip_ts_info *info)
{
    MIP_DEBUG("%s [START]\n", __func__);

    if (!info->irq_pause) {
        disable_irq(info->client->irq);
        info->irq_pause = true;
    }

    /*
    mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    */
    MIP_DEBUG("%s [DONE]\n", __func__);
}

/**
* Control regulator
*/
int mip_regulator_control(struct mip_ts_info *info, int enable)
{
    //////////////////////////
    // PLEASE MODIFY HERE !!!
    //
#if MIP_USE_DEVICETREE
    struct regulator *regulator_vio;
    struct regulator *regulator_vd33;
    int on = enable;
    int ret = 0;
    dev_dbg(&info->client->dev, "%s [START]\n", __func__);
    dev_dbg(&info->client->dev, "%s - switch : %d\n", __func__, on);

    if(info->power_state == on) {
        dev_dbg(&info->client->dev, "%s - skip\n", __func__);
        goto EXIT;
    }

    //regulator_vio = regulator_get(NULL, "tsp_vio");
    regulator_vio = regulator_get(&info->client->dev, "tsp_vio");

    if(IS_ERR(regulator_vio)) {
        dev_err(&info->client->dev, "%s [ERROR] regulator_get tsp_vio\n", __func__);
        ret = PTR_ERR(regulator_vio);
        goto ERROR;
    }

    //regulator_vd33 = regulator_get(NULL, "tsp_vd33");
    regulator_vd33 = regulator_get(&info->client->dev, "tsp_vd33");

    if(IS_ERR(regulator_vd33)) {
        dev_err(&info->client->dev, "%s [ERROR] regulator_get tsp_vd33\n", __func__);
        ret = PTR_ERR(regulator_vd33);
        goto ERROR;
    }

    if(on) {
        ret = regulator_enable(regulator_vio);

        if(ret) {
            dev_err(&info->client->dev, "%s [ERROR] regulator_enable vio\n", __func__);
            goto ERROR;
        }

        ret = regulator_enable(regulator_vd33);

        if(ret) {
            dev_err(&info->client->dev, "%s [ERROR] regulator_enable vd33\n", __func__);
            goto ERROR;
        }

        ret = pinctrl_select_state(info->pinctrl, info->pins_enable);

        if(ret < 0)
            dev_err(&info->client->dev, "%s [ERROR] pinctrl_select_state pins_enable\n",
                    __func__);

    } else {
        if(regulator_is_enabled(regulator_vio))
            regulator_disable(regulator_vio);

        if(regulator_is_enabled(regulator_vd33))
            regulator_disable(regulator_vd33);

        ret = pinctrl_select_state(info->pinctrl, info->pins_disable);

        if(ret < 0)
            dev_err(&info->client->dev, "%s [ERROR] pinctrl_select_state pins_disable\n",
                    __func__);
    }

    regulator_put(regulator_vio);
    regulator_put(regulator_vd33);
    info->power_state = on;
    goto EXIT;
    //
    //////////////////////////
ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return ret;
EXIT:
    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
#endif
    return 0;
}

/**
* Turn off power supply
*/
int mip_power_off(struct mip_ts_info *info)
{
    MIP_DEBUG("%s [START]\n", __func__);
    //////////////////////////
    // PLEASE MODIFY HERE !!!
    //
    //CE(Reset) Pin
    //mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    tpd_gpio_output(GTP_RST_PORT, 0);
    MIP_DEBUG("%s - Reset Pin [0]\n", __func__);
    //VD33 (Power supply)
    info->power_state = 0;
    msleep(10);
    MIP_DEBUG("%s [DONE]\n", __func__);
    return 0;
}

/**
* Turn on power supply
*/
int mip_power_on(struct mip_ts_info *info)
{
    MIP_DEBUG("%s [START]\n", __func__);
    //////////////////////////
    // PLEASE MODIFY HERE !!!
    //
    //CE(Reset) Pin
    //mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    tpd_gpio_output(GTP_RST_PORT, 1);
    MIP_DEBUG("%s - Reset Pin [1]\n", __func__);
    info->power_state = 1;
    msleep(60);
    MIP_DEBUG("%s [DONE]\n", __func__);
    return 0;
}

/**
* Clear touch input event status in the set
*/
void mip_clear_input(struct mip_ts_info *info)
{
    int i;
    MIP_DEBUG("%s [START]\n", __func__);

    //Screen
    for(i = 0; i < MAX_FINGER_NUM; i++) {
        /////////////////////////////////
        // PLEASE MODIFY HERE !!!
        //
#if MIP_INPUT_REPORT_TYPE
        /*
        input_mt_slot(info->input_dev, i);
        input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, false);
        */
        input_mt_slot(tpd->dev, i);
        input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
#else
        input_report_key(tpd->dev, BTN_TOUCH, 0);
        //input_mt_sync(tpd->dev);
#endif
        info->touch_state[i] = 0;
        //
        /////////////////////////////////
    }

    //Key
    if(info->key_enable == true) {
        for(i = 0; i < info->key_num; i++)
            input_report_key(info->input_dev, info->key_code[i], 0);
    }

    //input_sync(info->input_dev);
    input_sync(tpd->dev);
    MIP_DEBUG("%s [DONE]\n", __func__);
    return;
}

/**
* Input event handler - Report touch input event
*/
void mip_input_event_handler(struct mip_ts_info *info, u8 sz, u8 *buf)
{
    int i;
    int id, x, y;
    int pressure = 0;
    int size = 0;
    int touch_major = 0;
    int touch_minor = 0;
    int palm = 0;
    int finger_id = 0;
    int finger_cnt = 0;

    MIP_DEBUG("%s [START]\n", __func__);
    MIP_DEBUG("%s - sz[%d] buf[0x%02X]\n", __func__, sz, buf[0]);
#if MIP_FW_UPDATE_DEBUG
    print_hex_dump(KERN_ERR, MIP_DEVICE_NAME " Event Packet : ",
                   DUMP_PREFIX_OFFSET, 16, 1, buf, sz, false);
#endif

    for (i = 0; i < sz; i += info->event_size) {
        u8 *tmp = &buf[i];

        //Report input data
        if ((tmp[0] & MIP_EVENT_INPUT_SCREEN) == 0) {
            //Touchkey Event
            int key = tmp[0] & 0xf;
            int key_state = (tmp[0] & MIP_EVENT_INPUT_PRESS) ? 1 : 0;
            int key_code = 0;

            //Report touchkey event
            if((key > 0) && (key <= info->key_num)) {
                //key_code = info->key_code[key - 1];
                key_code = mip_tpd_keys_local[key - 1];
                //input_report_key(info->input_dev, key_code, key_state);
                input_report_key(tpd->dev, key_code, key_state);
                MIP_DEBUG("%s - Key : ID[%d] Code[%d] State[%d]\n",
                          __func__, key, key_code, key_state);

            } else {
                dev_err(&info->client->dev, "%s [ERROR] Unknown key id [%d]\n", __func__, key);
                continue;
            }

        } else {
            //Touchscreen Event

            //Protocol Type
            if(info->event_format == 0) {
                id = (tmp[0] & 0xf) - 1;
                x = tmp[2] | ((tmp[1] & 0xf) << 8);
                y = tmp[3] | (((tmp[1] >> 4) & 0xf) << 8);
                pressure = tmp[4];
                touch_major = tmp[5];
                palm = (tmp[0] & MIP_EVENT_INPUT_PALM) >> 4;

            } else if(info->event_format == 1) {
                id = (tmp[0] & 0xf) - 1;
                x = tmp[2] | ((tmp[1] & 0xf) << 8);
                y = tmp[3] | (((tmp[1] >> 4) & 0xf) << 8);
                pressure = tmp[4];
                size = tmp[5];
                touch_major = tmp[6];
                touch_minor = tmp[7];
                palm = (tmp[0] & MIP_EVENT_INPUT_PALM) >> 4;

            } else if(info->event_format == 2) {
                id = (tmp[0] & 0xf) - 1;
                x = tmp[2] | ((tmp[1] & 0xf) << 8);
                y = tmp[3] | (((tmp[1] >> 4) & 0xf) << 8);
                pressure = tmp[4];
                touch_major = tmp[5];
                touch_minor = tmp[6];
                palm = (tmp[0] & MIP_EVENT_INPUT_PALM) >> 4;

            } else {
                dev_err(&info->client->dev, "%s [ERROR] Unknown event format [%d]\n",
                        __func__, info->event_format);
                goto ERROR;
            }

            /////////////////////////////////
            // PLEASE MODIFY HERE !!!
            //

            //Report touchscreen event
            if((tmp[0] & MIP_EVENT_INPUT_PRESS) == 0) {
                //Release
#if MIP_INPUT_REPORT_TYPE
                //input_mt_slot(info->input_dev, id);
                //input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, 0);
                input_mt_slot(tpd->dev, id);
                input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, 0);
#else
                //input_report_key(tpd->dev, BTN_TOUCH, 0);
                //input_mt_sync(tpd->input_dev);
#endif
                info->touch_state[id] = 0;
                MIP_DEBUG("%s - Touch : ID[%d] Release\n", __func__, id);
                continue;
            }

            //Press or Move
#if !MIP_REPORT_ALL_FINGER
#if MIP_INPUT_REPORT_TYPE
            /*
            input_mt_slot(info->input_dev, id);
            input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, true);
            input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
            input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);
            input_report_abs(info->input_dev, ABS_MT_PRESSURE, pressure);
            input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR, touch_major);
            */
            input_mt_slot(tpd->dev, id);
            input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, true);
            input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
            input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
            input_report_abs(tpd->dev, ABS_MT_PRESSURE, pressure);
            input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, touch_major);
#else
            input_report_abs(tpd->dev, ABS_MT_PRESSURE, pressure);
            input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, touch_major);
            input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
            input_report_key(tpd->dev, BTN_TOUCH, 1);
            input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
            input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
            input_mt_sync(tpd->dev);
#endif
#endif
            info->touch_state[id] = 1;
#if MIP_REPORT_ALL_FINGER
            info->touch_x[id] = x;
            info->touch_y[id] = y;
            info->touch_pressure[id] = pressure;
            info->touch_major[id] = touch_major;
#else
            MIP_DEBUG("%s - Touch : ID[%d] X[%d] Y[%d] Z[%d] Major[%d] Minor[%d] Size[%d] Palm[%d]\n",
                      __func__, id, x, y, pressure, touch_major, touch_minor, size, palm);
#endif
        }
    }

#if MIP_REPORT_ALL_FINGER

    for(finger_id = 0; finger_id < MAX_FINGER_NUM; finger_id++) {
        if(info->touch_state[finger_id] != 0) {
            input_report_abs(tpd->dev, ABS_MT_PRESSURE, info->touch_pressure[finger_id]);
            input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, info->touch_major[finger_id]);
            input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, finger_id);
            input_report_key(tpd->dev, BTN_TOUCH, 1);
            input_report_abs(tpd->dev, ABS_MT_POSITION_X, info->touch_x[finger_id]);
            input_report_abs(tpd->dev, ABS_MT_POSITION_Y, info->touch_y[finger_id]);
            input_mt_sync(tpd->dev);
            MIP_DEBUG("%s - Touch : ID[%d] X[%d] Y[%d] Z[%d] Major[%d]\n",
                      __func__, finger_id, info->touch_x[finger_id],
                      info->touch_y[finger_id], info->touch_pressure[finger_id],
                      info->touch_major[finger_id]);
        }
    }

#endif
#if MIP_INPUT_REPORT_TYPE
#else
    //Release last finger
    finger_cnt = 0;

    for(finger_id = 0; finger_id < MAX_FINGER_NUM; finger_id++) {
        if(info->touch_state[finger_id] != 0) {
            finger_cnt++;
            break;
        }
    }

    if(finger_cnt == 0) {
        input_report_key(tpd->dev, BTN_TOUCH, 0);
        input_mt_sync(tpd->dev);
        MIP_DEBUG("%s - Touch : Release\n", __func__);
    }

#endif
    //input_sync(info->input_dev);
    input_sync(tpd->dev);
    MIP_DEBUG("%s [DONE]\n", __func__);
    return;
ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return;
}

/**
* Wake-up event handler
*/
int mip_wakeup_event_handler(struct mip_ts_info *info, u8 *rbuf)
{
    u8 wbuf[4];
    u8 gesture_code = rbuf[1];
    MIP_DEBUG("%s [START]\n", __func__);
    /////////////////////////////////
    // PLEASE MODIFY HERE !!!
    //
    //Report wake-up event
    MIP_INFO("%s - gesture[%d]\n", __func__, gesture_code);
    info->wakeup_gesture_code = gesture_code;

    switch(gesture_code) {
    case MIP_EVENT_GESTURE_C:
    case MIP_EVENT_GESTURE_W:
    case MIP_EVENT_GESTURE_V:
    case MIP_EVENT_GESTURE_M:
    case MIP_EVENT_GESTURE_S:
    case MIP_EVENT_GESTURE_Z:
    case MIP_EVENT_GESTURE_O:
    case MIP_EVENT_GESTURE_E:
    case MIP_EVENT_GESTURE_V_90:
    case MIP_EVENT_GESTURE_V_180:
    case MIP_EVENT_GESTURE_FLICK_RIGHT:
    case MIP_EVENT_GESTURE_FLICK_DOWN:
    case MIP_EVENT_GESTURE_FLICK_LEFT:
    case MIP_EVENT_GESTURE_FLICK_UP:
    case MIP_EVENT_GESTURE_DOUBLE_TAP:
        //Example : emulate power key
#ifdef CUSTOM_LIGHTUP_SCREEN
        input_report_key(tpd->dev, KEY_LIGHTUP, 1);
        input_sync(tpd->dev);
        input_report_key(tpd->dev, KEY_LIGHTUP, 0);
        input_sync(tpd->dev);
#else
        //input_report_key(info->input_dev, KEY_POWER, 1);
        input_report_key(tpd->dev, KEY_POWER, 1);
        //input_sync(info->input_dev);
        input_sync(tpd->dev);
        //input_report_key(info->input_dev, KEY_POWER, 0);
        input_report_key(tpd->dev, KEY_POWER, 0);
        //input_sync(info->input_dev);
        input_sync(tpd->dev);
#endif
        break;

    default:
        //Re-enter nap mode
        wbuf[0] = MIP_R0_CTRL;
        wbuf[1] = MIP_R1_CTRL_POWER_STATE;
        wbuf[2] = MIP_CTRL_POWER_LOW;

        if(mip_i2c_write(info, wbuf, 3)) {
            dev_err(&info->client->dev, "%s [ERROR] mip_i2c_write\n", __func__);
            goto ERROR;
        }

        break;
    }

    MIP_DEBUG("%s [DONE]\n", __func__);
    return 0;
ERROR:
    return 1;
}

#if MIP_USE_DEVICETREE
/**
* Parse device tree
*/
int mip_parse_devicetree(struct device *dev, struct mip_ts_info *info)
{
    //   struct i2c_client *client = to_i2c_client(dev);
    //    struct mip_ts_info *info = (struct mip_ts_info *)i2c_get_clientdata(client);
    struct device_node *np = dev->of_node;
    int ret;
    //u32 val;
    dev_dbg(dev, "%s [START]\n", __func__);
    /////////////////////////////////
    // PLEASE MODIFY HERE !!!
    //
    //Read property
    /*
    ret = of_property_read_u32(np, MIP_DEVICE_NAME",max_x", &val);
    if (ret) {
    	dev_err(dev, "%s [ERROR] max_x\n", __func__);
    	info->pdata->max_x = 1080;
    }
    else {
    	info->pdata->max_x = val;
    }

    ret = of_property_read_u32(np, MIP_DEVICE_NAME",max_y", &val);
    if (ret) {
    	dev_err(dev, "%s [ERROR] max_y\n", __func__);
    	info->pdata->max_y = 1920;
    }
    else {
    	info->pdata->max_y = val;
    }
    */
    //Get GPIO
    ret = of_get_named_gpio(np, MIP_DEVICE_NAME",gpio_irq", 0);

    if (!gpio_is_valid(ret)) {
        dev_err(dev, "%s [ERROR] of_get_named_gpio : gpio_irq\n", __func__);
        goto ERROR;

    } else
        info->pdata->gpio_intr = ret;

    /*
    ret = of_get_named_gpio(np, MIP_DEVICE_NAME",gpio_reset", 0);
    if (!gpio_is_valid(ret)) {
    	dev_err(dev, "%s [ERROR] of_get_named_gpio : gpio_reset\n", __func__);
    	goto ERROR;
    }
    else{
    	info->pdata->gpio_reset = ret;
    }
    */
    //Config GPIO
    ret = gpio_request(info->pdata->gpio_intr, "gpio_irq");

    if (ret < 0) {
        dev_err(dev, "%s [ERROR] gpio_request : gpio_irq\n", __func__);
        goto ERROR;
    }

    gpio_direction_input(info->pdata->gpio_intr);
    /*
    ret = gpio_request(info->pdata->gpio_reset, "gpio_reset");
    if (ret < 0) {
    	dev_err(dev, "%s [ERROR] gpio_request : gpio_reset\n", __func__);
    	goto ERROR;
    }
    gpio_direction_output(info->pdata->gpio_reset, 1);
    */
    //Set IRQ
    info->client->irq = gpio_to_irq(info->pdata->gpio_intr);
    //dev_dbg(dev, "%s - gpio_to_irq : irq[%d]\n", __func__, info->client->irq);
    //Get Pinctrl
    /*    info->pinctrl = devm_pinctrl_get(&info->client->dev);
        if (IS_ERR(info->pinctrl)) {
            dev_err(dev, "%s [ERROR] devm_pinctrl_get\n", __func__);
            goto ERROR;
        }

        info->pins_default = pinctrl_lookup_state(info->pinctrl, "on_state");
        if (IS_ERR(info->pins_default)) {
            dev_err(dev, "%s [ERROR] pinctrl_lookup_state on_state\n", __func__);
        }

        info->pins_sleep = pinctrl_lookup_state(info->pinctrl, "off_state");
        if (IS_ERR(info->pins_sleep)) {
            dev_err(dev, "%s [ERROR] pinctrl_lookup_state off_state\n", __func__);
        }*/
    //
    /////////////////////////////////
    dev_dbg(dev, "%s [DONE]\n", __func__);
    return 0;
ERROR:
    dev_err(dev, "%s [ERROR]\n", __func__);
    return 1;
}
#endif

/**
* Config platform data
*/
int mip_config_platform_data(struct i2c_client *client,
                             struct mip_ts_info *info)
{
    dev_dbg(&client->dev, "%s [START]\n", __func__);
    /*    info->pdata = devm_kzalloc(&client->dev, sizeof(struct melfas_platform_data), GFP_KERNEL);
        if (!info->pdata) {
            dev_err(&client->dev, "%s [ERROR] pdata devm_kzalloc\n", __func__);
            goto ERROR;
        }

        //Get Resolution
        info->pdata->max_x = LCD_X;
        info->pdata->max_y = LCD_Y;

        //Config GPIO
        info->pdata->gpio_ce = GPIO_CTP_RST_PIN;
        info->pdata->gpio_intr = GPIO_CTP_EINT_PIN;

        dev_dbg(&client->dev, "%s [DONE]\n", __func__);
        return 0;

    ERROR:
        dev_err(&client->dev, "%s [ERROR]\n", __func__);*/
    return 1;
}

/**
* Config GPIO
*/
void mip_config_gpio(struct mip_ts_info *info)
{
    MIP_DEBUG("%s [START]\n", __func__);
    return;
}

/**
* Config input interface
*/
void mip_config_input(struct mip_ts_info *info)
{
    int i = 0;
    MIP_DEBUG("%s [START]\n", __func__);
    /////////////////////////////
    // PLEASE MODIFY HERE !!!
    //
    /*
    set_bit(EV_SYN, input_dev->evbit);
    set_bit(EV_ABS, input_dev->evbit);
    set_bit(EV_KEY, input_dev->evbit);
    */
    set_bit(EV_SYN, tpd->dev->evbit);
    set_bit(EV_ABS, tpd->dev->evbit);
    set_bit(EV_KEY, tpd->dev->evbit);
    //Screen
#if MIP_INPUT_REPORT_TYPE
    /*
    set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
    //input_mt_init_slots(input_dev, MAX_FINGER_NUM);
    input_mt_init_slots(input_dev, MAX_FINGER_NUM, INPUT_MT_DIRECT);
    input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, info->max_x, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, info->max_y, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, INPUT_PRESSURE_MAX, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, INPUT_TOUCH_MAJOR_MAX, 0, 0);
    */
    set_bit(INPUT_PROP_DIRECT, tpd->dev->propbit);
    //input_mt_init_slots(tpd->dev, MAX_FINGER_NUM);
    input_mt_init_slots(tpd->dev, MAX_FINGER_NUM, INPUT_MT_DIRECT);
    input_set_abs_params(tpd->dev, ABS_MT_POSITION_X, 0, info->max_x, 0, 0);
    input_set_abs_params(tpd->dev, ABS_MT_POSITION_Y, 0, info->max_y, 0, 0);
    input_set_abs_params(tpd->dev, ABS_MT_PRESSURE, 0, INPUT_PRESSURE_MAX, 0, 0);
    input_set_abs_params(tpd->dev, ABS_MT_TOUCH_MAJOR, 0,
                         INPUT_TOUCH_MAJOR_MAX, 0, 0);
#else
    set_bit(BTN_TOUCH, tpd->dev->keybit);
    input_set_abs_params(tpd->dev, ABS_MT_TRACKING_ID, 0, MAX_FINGER_NUM, 0, 0);
    input_set_abs_params(tpd->dev, ABS_MT_POSITION_X, 0, info->max_x, 0, 0);
    input_set_abs_params(tpd->dev, ABS_MT_POSITION_Y, 0, info->max_y, 0, 0);
    input_set_abs_params(tpd->dev, ABS_MT_PRESSURE, 0, INPUT_PRESSURE_MAX, 0, 0);
    input_set_abs_params(tpd->dev, ABS_MT_TOUCH_MAJOR, 0,
                         INPUT_TOUCH_MAJOR_MAX, 0, 0);
#endif

    //Key
    /*
    //set_bit(KEY_BACK, input_dev->keybit);
    //set_bit(KEY_MENU, input_dev->keybit);

    //info->key_code[0] = KEY_BACK;
    //info->key_code[1] = KEY_MENU;

    #if MIP_USE_WAKEUP_GESTURE
    set_bit(KEY_POWER, input_dev->keybit);
    #endif
    */

    for(i = 0; i < TPD_KEY_COUNT; i++)
        set_bit(mip_tpd_keys_local[i], tpd->dev->keybit);

#if MIP_USE_WAKEUP_GESTURE
#ifdef CUSTOM_LIGHTUP_SCREEN
    set_bit(KEY_LIGHTUP, tpd->dev->keybit);
#else
    set_bit(KEY_POWER, tpd->dev->keybit);
#endif
#endif
    MIP_DEBUG("%s [DONE]\n", __func__);
    return;
}

