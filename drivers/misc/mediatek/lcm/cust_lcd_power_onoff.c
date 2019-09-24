#include <linux/string.h>
#include "lcm_drv.h"
#include <mt-plat/mt_gpio.h>
#include <mach/gpio_const.h>
#include <linux/of_platform.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#define LCD_VCC_EP        			(GPIO63 | 0x80000000)
#define LCD_VCC_EN        			(GPIO64 | 0x80000000)
#define GPIO_LCD_IO        		    	(GPIO58 | 0x80000000)
#define GPIO_LCD_RST        			(GPIO146 | 0x80000000)

#if 0
#define TPS65132_SLAVE_ADDR_WRITE 	0x7C
#define TPS65132_SLAVE_ADDR_READ	0x7D
#define TPS65132_I2C_ID 		I2C3

static struct mt_i2c_t tps65132_i2c;
#define LCD_DEBUG(fmt, args...) printf(fmt, ##args)

#ifndef kal_uint8
typedef unsigned char   kal_uint8;
#endif
#ifndef kal_uint16
typedef unsigned short  kal_uint16;
#endif

static int tps65132_read_byte(kal_uint8 addr, kal_uint8 *dataBuffer)
{
	int ret_code = I2C_OK;
	kal_uint16 len;

	tps65132_i2c.id = TPS65132_I2C_ID;
	tps65132_i2c.addr = (TPS65132_SLAVE_ADDR_WRITE >> 1);
	tps65132_i2c.mode = ST_MODE;
	tps65132_i2c.speed = 100;
	len = 1;

	ret_code = i2c_write_read(&tps65132_i2c, dataBuffer, len, len);
	
	return ret_code;
}

static int tps65132_write_byte(kal_uint8 addr, kal_uint8 value)
{
	int ret_code = I2C_OK;
	kal_uint8 write_data[2];
	kal_uint16 len;

	write_data[0] = addr;
	write_data[1] = value;
	
	tps65132_i2c.id = TPS65132_I2C_ID;
	tps65132_i2c.addr = (TPS65132_SLAVE_ADDR_WRITE >> 1);
	tps65132_i2c.mode = ST_MODE;
	tps65132_i2c.speed = 100;
	len = 2;
	
	ret_code = i2c_write(&tps65132_i2c, write_data, len);

	return ret_code;
}

static void cust_tps65132_init(void)
{
	kal_uint8 addr;
	kal_uint8 value;
	int ret_code;
	int count = 0;

step1:	
	//pos
	addr = 0x0;
	value = 0x0F;
	ret_code = tps65132_write_byte(addr, value);
	if (ret_code)
	{
		printk(0, "[LK]tps65321 i2c write failed! ret_code = %d, count = %d\n", ret_code, count);
		if (count++ < 5)
			goto step1;
		else
			return;
	}
	else
	{
		count = 0;
		printk(0, "[LK]tps65321 i2c write success!\n");
	}
	//ne

step2:
	addr = 0x1;
        value = 0x0F;
	ret_code = tps65132_write_byte(addr, value);
        if (ret_code)
        {
		printk(0, "[LK]tps65321 i2c write failed! ret_code = %d, count = %d\n", ret_code, count);
		if (count++ < 5)
                        goto step2;
                else
                        return;
        }
        else
	{
		count = 0;
                printk(0, "[LK]tps65321 i2c write success!\n");
	}

step3:	
	addr = 0x03;
	value = 0x03;
	ret_code = tps65132_write_byte(addr, value);
        if (ret_code)
        {
		printk(0, "[LK]tps65321 i2c write failed! ret_code = %d, count = %d\n", ret_code, count);
                if (count++ < 5)
                        goto step3;
                else
                        return;
        }
        else
	{
		count = 0;
                printk(0, "[LK]tps65321 i2c write success!\n");
	}

step4:
	addr = 0xFF;
	value = 0xF0;
	ret_code = tps65132_write_byte(addr, value);
        if (ret_code)
        {
		printk(0, "[LK]tps65321 i2c write failed! ret_code = %d, count = %d\n", ret_code, count);
                if (count++ < 5)
                        goto step4;
                else
                        return;
        }
        else
	{
		count = 0;
                printk(0, "[LK]tps65321 i2c write success!\n");
	}
	mdelay(50);

}
#endif
static void lcd_reset(unsigned int enabled)
{
    if (enabled)
    {
		mt_set_gpio_mode(GPIO_LCD_RST, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_LCD_RST, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCD_RST, GPIO_OUT_ONE);
    }
    else
    {	
		mt_set_gpio_mode(GPIO_LCD_RST, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_LCD_RST, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCD_RST, GPIO_OUT_ZERO);    
    }
}
static void lcd_vcc_ep(unsigned int enabled)
{
    if (enabled)
    {
		mt_set_gpio_mode(LCD_VCC_EP, GPIO_MODE_00);
		mt_set_gpio_dir(LCD_VCC_EP, GPIO_DIR_OUT);
		mt_set_gpio_out(LCD_VCC_EP, GPIO_OUT_ONE);

    }
    else
    {	
		mt_set_gpio_mode(LCD_VCC_EP, GPIO_MODE_00);
		mt_set_gpio_dir(LCD_VCC_EP, GPIO_DIR_OUT);
		mt_set_gpio_out(LCD_VCC_EP, GPIO_OUT_ZERO);
    }
}
static void lcd_vcc_en(unsigned int enabled)
{
    if (enabled)
    {
		mt_set_gpio_mode(LCD_VCC_EN, GPIO_MODE_00);
		mt_set_gpio_dir(LCD_VCC_EN, GPIO_DIR_OUT);
		mt_set_gpio_out(LCD_VCC_EN, GPIO_OUT_ONE);
    }
    else
    {	
		mt_set_gpio_mode(LCD_VCC_EN, GPIO_MODE_00);
		mt_set_gpio_dir(LCD_VCC_EN, GPIO_DIR_OUT);
		mt_set_gpio_out(LCD_VCC_EN, GPIO_OUT_ZERO);
	
    }
}
static void lcd_io_en(unsigned int enabled)
{
    if (enabled)
    {
		mt_set_gpio_mode(GPIO_LCD_IO, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_LCD_IO, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCD_IO, GPIO_OUT_ONE);

    }
    else
    {	
		mt_set_gpio_mode(GPIO_LCD_IO, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_LCD_IO, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCD_IO, GPIO_OUT_ZERO);   	
    }
}
void cust_lcd_init_power_sequence(unsigned int enable)
	{
	
		if(enable==1){
			printk("enter cust_lcd_init_power_sequence,enable\n");
			lcd_vcc_en(0);
			lcd_vcc_ep(0);
			lcd_reset(0);
			mdelay(10);
			//power on 3.3v
			lcd_io_en(1);
			mdelay(15);
			/*
			//reset
			lcd_reset(1);
			mdelay(1);
			lcd_reset(0);
			mdelay(10);
			lcd_reset(1);
          	       mdelay(10);*/
			
			//power on -+5V
			lcd_vcc_ep(1);
			mdelay(5);
			lcd_vcc_en(1);
			mdelay(10);
			/*
			cust_tps65132_init();
			mdelay(10);*/
            /*
			//reset
			lcd_reset(0);
			mdelay(10);
			lcd_reset(1);
			mdelay(30);*/
			
		}
		else
			{
			
			    printk("enter cust_lcd_init_power_sequence,disable\n");
				lcd_reset(0);
				mdelay(20);
				lcd_vcc_en(0);
				mdelay(5);
			    lcd_vcc_ep(0);
				mdelay(130);
				lcd_io_en(0);
				mdelay(500);
				
			}

}
