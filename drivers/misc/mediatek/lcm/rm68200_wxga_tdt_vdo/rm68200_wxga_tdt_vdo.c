#ifndef BUILD_LK
#include <linux/string.h>
#endif

#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <string.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
#include <mt-plat/mt_gpio.h>
#endif
#include "../code/third/cfg.h"
//#include <linux/delay.h>
#include <mach/gpio_const.h>

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------


#define FRAME_WIDTH  (800)
#define FRAME_HEIGHT (1280)

#define LCM_DSI_CMD_MODE									0
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

extern void cust_lcd_init_power_sequence(unsigned int enable);
#define GPIO_LCD_RST        			(GPIO146 | 0x80000000)

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);				
       	}
		
    }
	
}

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

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy((void*)&lcm_util, (void*)util, sizeof(LCM_UTIL_FUNCS));
}




static void lcm_get_params(LCM_PARAMS *params)
{
		memset((void*)params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;
#endif
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;

		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
		params->dsi.vertical_sync_active				= 4;
		params->dsi.vertical_backporch					= 8;
		params->dsi.vertical_frontporch					= 8;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 4;
		params->dsi.horizontal_backporch				= 132;
		params->dsi.horizontal_frontporch				= 24;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

                /*[BUGFIX]-mod-begin,by scdtablet jinghuang@tcl.com,6836722*/
                /*modify lcd clk mode to noncontinous mode*/
	       //params->dsi.clk_lp_per_line_enable = 1;
	       //params->dsi.cont_clock = 0;
               /*[BUGFIX]-mod-end*/
		/*[FEATURE]-add-begin by scdtablet jinghuang@tcl.com,2016.12.21,3532945*/
		/*add esd recovery function for txd*/
/*esd*/
		params->dsi.esd_check_enable=1;
		params->dsi.customization_esd_check_enable=1;
		params->dsi.lcm_esd_check_table[0].cmd=0x0a;
		params->dsi.lcm_esd_check_table[0].count=0x1;
		params->dsi.lcm_esd_check_table[0].para_list[0]=0x9c;
		/*params->dsi.lcm_esd_check_table[1].cmd=0xac;
		params->dsi.lcm_esd_check_table[1].count=0x1;
		params->dsi.lcm_esd_check_table[1].para_list[0]=0x0;*/

/**/
		/*[FEATURE]-add-end*/

                /*[BUGFIX]-mod-begin,by scdtablet jinghuang@tcl.com,6836722*/
                /*modify lcd clk mode to noncontinous mode*/
		params->dsi.noncont_clock=1;
                /*[BUGFIX]-mod-end*/
		params->dsi.PLL_CLOCK=224;
}

static void lcm_init(void)
{
#ifdef BUILD_LK
    printf("%s, LK \n", __func__);
#else
    printk("%s, kernel", __func__);
#endif
		/*reset*/
    lcd_reset(1);
	MDELAY(10);
	lcd_reset(0);
	MDELAY(10);
	lcd_reset(1);
	MDELAY(50);
	push_table(lcm_vdo_initialization_setting, sizeof(lcm_vdo_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
#ifdef BUILD_LK
    printf("%s, LK \n", __func__);
#else
    printk("%s, kernel", __func__);
#endif

	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(60);
	cust_lcd_init_power_sequence(0);

}

static void lcm_resume(void)
{
#ifdef BUILD_LK
	printf("%s, LK \n", __func__);
#else
	printk("%s, kernel", __func__);
#endif

	lcm_init();
}



// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER rm68200_tdt_wxga_dsi_vdo_lcm_drv = 
{
    .name			= "rm68200_tdt_wxga_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	//.compare_id     = lcm_compare_id,
	//.esd_check   	= lcm_esd_check,
    //.esd_recover	= lcm_esd_recover,
};
