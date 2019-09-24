
#include "lcm_drv.h"
#include <mach/gpio_const.h>
#include <mt_gpio.h>
#include "../code/first/cfg.h"
#define LCM_PRINT printk

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)

#define REGFLAG_DELAY             							0xFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
 
static LCM_UTIL_FUNCS lcm_util = {0};

//#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()

#define GPIO_LCM_PAVDD_EN_PIN           (GPIO63 | 0x80000000)
#define GPIO_LCM_NAVDD_EN_PIN           (GPIO64 | 0x80000000)
#define GPIO_LCM_ID1_PIN                (GPIO78 | 0x80000000)
#define GPIO_LCM_ID2_PIN                (GPIO79 | 0x80000000)
#define GPIO_LCM_RESET_PIN 		(GPIO146 | 0x80000000)
#define GPIO_LCM_DSI_TE_PIN		(GPIO147 | 0x80000000)
#define GPIO_LCM_1V8_EN			(GPIO58 | 0x80000000)


#if 0
struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
	
	/*
	Note :

	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/
	{0xB0, 1, {0x04}},
	{0xE3, 1, {0x01}},
	{0xE5, 1, {0x03}},
	{0xB6, 1, {0x32}},
	{0xC0, 6, {0x23, 0xB2, 0x0D, 0x10, 0x02, 0x7F}},
	{0xC1, 7, {0x23, 0x80, 0xA0, 0x23, 0x00, 0x00, 0x55}},//0x0A, 0x0A -> 0x00, 0x00
	{0xC4, 3, {0xB4, 0xB0, 0x00}},
	{0xC3, 1, {0x20}},
	{0xC5, 3, {0x06, 0x02, 0x00}},
	{0xC6, 1, {0xC1}},
	{0xC8, 28, {0x51, 0x6B, 0xAD, 0xB5, 0xD6, 0x00, 0x2A, 0x7C, 0xAE, 0x2D, 0x76, 0x90, 0x8C, 0xB5, 0xD6, 0x1A, 0x25, 0x3A, 0x6C, 0x35, 0xF7, 0x60, 0x05, 0xA1, 0xD6, 0x5A, 0x6B, 0xAD}},
	{0xCA, 30, {0x04, 0x0D, 0x13, 0x19, 0x22, 0x27, 0x2A, 0x28, 0x1B, 0x1B, 0x19, 0x14, 0x12, 0x0F, 0x0E, 0x01, 0x0D, 0x12, 0x19, 0x22, 0x27, 0x2A, 0x28, 0x1B, 0x1B, 0x19, 0x14, 0x12, 0x0F, 0x0E}},
	{0xD0, 3, {0x05, 0x10, 0x4B}},
	{0xD1, 1, {0x03}},
	{0xD2, 2, {0x91, 0x1F}},
	{0xD3, 2, {0x44, 0x33}},//0x11, 0x33 -> 0x44, 0x33
	{0xD4, 1, {0x2B}},//0x36
	{0xD5, 2, {0x34, 0x34}},
	{0xD6, 1, {0x01}},
	{0x35, 1, {0x00}},
        {0x11, 0, {}},
	{REGFLAG_DELAY, 150, {}},
	{0x29, 0, {}},
	{REGFLAG_DELAY, 50, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}},
};
#endif

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	{0x28, 0, {}},
	{REGFLAG_DELAY, 50, {}},
	{0x10, 0, {}},
	{REGFLAG_DELAY, 150, {}},
	{0xB0, 1, {0x04}},
	{0xB1, 1, {0x01}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}

};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for(i = 0; i < count; i++)
	{
		unsigned cmd;
		cmd = table[i].cmd;

		switch (cmd)
		{
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

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));

    params->type   = LCM_TYPE_DSI;
    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

    // enable tearing-free
    params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    params->dsi.packet_size=256;
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active				= 5;
    params->dsi.vertical_backporch					= 8;
    params->dsi.vertical_frontporch					= 17;
    params->dsi.vertical_active_line				= FRAME_HEIGHT;

    params->dsi.horizontal_sync_active				= 15;
    params->dsi.horizontal_backporch				= 25;
    params->dsi.horizontal_frontporch				= 65;
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
    params->dsi.PLL_CLOCK = 208;		//288, div by 26
	 /*add by sansan.gan,enable mipi non-continue clock begin*/  	
   	params->dsi.noncont_clock = 1;
	 /*add by sansan.gan,enable mipi non-continue clock end*/

	/*add by sansan.gan for disable lcm ssc func begin*/
	params->dsi.ssc_disable = 1;
	/*add by sansan.gan for disable lcm ssc func end*/

	/*add by sansan.gan for esd check by EXT TE begin*/
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 0;
	/*add by sansan.gan for esd check by EXT TE end*/
}

/*static void lcm_1v8_power_on(void)
{
	mt_set_gpio_mode(GPIO_LCM_1V8_EN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCM_1V8_EN, GPIO_DIR_OUT);

	mt_set_gpio_out(GPIO_LCM_1V8_EN, GPIO_OUT_ONE);
}*/

static void lcm_1v8_power_off(void)
{
	mt_set_gpio_mode(GPIO_LCM_1V8_EN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCM_1V8_EN, GPIO_DIR_OUT);

        mt_set_gpio_out(GPIO_LCM_1V8_EN, GPIO_OUT_ZERO);
}
/*
void lcm_pre_init(void)
{
	printk("[LCD] r61318_tdt_dsi_vdo lcm_pre_init sansan test \n");

	mt_set_gpio_mode(GPIO_LCM_DSI_TE_PIN, GPIO_MODE_01);
        mt_set_gpio_dir(GPIO_LCM_DSI_TE_PIN, GPIO_DIR_IN);

	mt_set_gpio_mode(GPIO_LCM_RESET_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCM_RESET_PIN, GPIO_DIR_OUT);

	mt_set_gpio_mode(GPIO_LCM_PAVDD_EN_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCM_PAVDD_EN_PIN, GPIO_DIR_OUT);
        mt_set_gpio_mode(GPIO_LCM_NAVDD_EN_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCM_NAVDD_EN_PIN, GPIO_DIR_OUT);

	mt_set_gpio_out(GPIO_LCM_PAVDD_EN_PIN, GPIO_OUT_ZERO);
        mt_set_gpio_out(GPIO_LCM_NAVDD_EN_PIN, GPIO_OUT_ZERO);
	mt_set_gpio_out(GPIO_LCM_RESET_PIN, GPIO_OUT_ZERO); 

	MDELAY(10);

        lcm_1v8_power_on();

	MDELAY(10);

        mt_set_gpio_out(GPIO_LCM_PAVDD_EN_PIN, GPIO_OUT_ONE);
        mt_set_gpio_out(GPIO_LCM_NAVDD_EN_PIN, GPIO_OUT_ONE);

	MDELAY(10);
}*/
static void lcm_init(void)
{
	LCM_PRINT("[LCD] r61318_tdt_dsi_vdo init \n");

	/*add by sansan for modifying lcd recovery power on sequence2016-8-30 16:20:14 begin*/
	#ifndef CONFIG_CUSTOM_KERNEL_LCM_PWRON
	lcm_pre_init();
	#endif
	/*add by sansan for modifying lcd recovery power on sequence2016-8-30 16:20:14 end*/
	mt_set_gpio_mode(GPIO_LCM_RESET_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM_RESET_PIN, GPIO_DIR_OUT);

	mt_set_gpio_out(GPIO_LCM_RESET_PIN, GPIO_OUT_ONE);
	MDELAY(10);
	mt_set_gpio_out(GPIO_LCM_RESET_PIN, GPIO_OUT_ZERO);
	MDELAY(10);
	mt_set_gpio_out(GPIO_LCM_RESET_PIN, GPIO_OUT_ONE);
	MDELAY(20);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);

	mt_set_gpio_mode(GPIO_LCM_RESET_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCM_RESET_PIN, GPIO_DIR_OUT);
	mt_set_gpio_mode(GPIO_LCM_PAVDD_EN_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCM_PAVDD_EN_PIN, GPIO_DIR_OUT);
        mt_set_gpio_mode(GPIO_LCM_NAVDD_EN_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCM_NAVDD_EN_PIN, GPIO_DIR_OUT);

	mt_set_gpio_out(GPIO_LCM_NAVDD_EN_PIN, GPIO_OUT_ZERO);
        mt_set_gpio_out(GPIO_LCM_PAVDD_EN_PIN, GPIO_OUT_ZERO);

	MDELAY(10);
	
	mt_set_gpio_out(GPIO_LCM_RESET_PIN, GPIO_OUT_ZERO);
        lcm_1v8_power_off();

	LCM_PRINT("[LCD] lcm_suspend \n");
}


static void lcm_resume(void)
{
	lcm_init();

	LCM_PRINT("[LCD] lcm_suspend \n");
}

#if 0
static unsigned int lcm_compare_id(void)
{
        unsigned int id1_state = 0;
        unsigned int id2_state = 0;

        mt_set_gpio_mode(GPIO_LCM_ID1_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCM_ID1_PIN, GPIO_DIR_IN);

        mt_set_gpio_mode(GPIO_LCM_ID2_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCM_ID2_PIN, GPIO_DIR_IN);

        lcm_1v8_power_on();
        
        MDELAY(10);

        id1_state = mt_get_gpio_in(GPIO_LCM_ID1_PIN);
        id2_state = mt_get_gpio_in(GPIO_LCM_ID2_PIN);

        if ((id1_state == 0) && (id2_state == 0))
        {
                LCM_PRINT("[LCD]r61318_tdt lcd selected!\n");
                lcm_1v8_power_off();
                return 1;
        }
        else
        {
                lcm_1v8_power_off();
                return 0;
        }
}
#endif

LCM_DRIVER r61318_tdt_dsi_vdo_lcm_drv = 
{
    .name			= "r61318_tdt_dsi_vdo",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
//    .compare_id     = lcm_compare_id,
};
