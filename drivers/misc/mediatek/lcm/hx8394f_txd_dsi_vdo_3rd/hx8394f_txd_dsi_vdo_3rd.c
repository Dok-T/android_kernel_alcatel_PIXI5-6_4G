
#include "lcm_drv.h"
#include <mach/gpio_const.h>
#include <mt_gpio.h>
#define LCM_PRINT printk

#include "../code/fifth/cfg.h"
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

/*
struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
{0xB9,03,{0xFF,0x83,0x94}},	         
{0xB1,10,{0x50,0x15,0x75,0x09,0x32,0x44,0x71,0x31,0x55,0x2F}}, 	     
{0xBA,06,{0x63,0x03,0x68,0x6B,0xB2,0xC0}},  	    
{0xD2,01,{0x88}},	      
{0xB2,05,{0x00,0x80,0x64,0x10,0x07}},	      
{0xB4,21,{0x01,0x65,0x01,0x65,0x01,0x65,0x01,0x05,0x7E,0x25,0x00,0x3F,0x01,0x65,0x01,0x65,0x01,0x65,0x01,0x05,0x7E}},    	   
{0xD3,33,{0x00,0x00,0x0F,0x0F,0x40,0x1E,0x08,0x00,0x32,0x10,0x08,0x00,0x08,0x54,0x15,0x10,0x05,0x04,0x02,0x12,0x10,0x05,0x07,0x23,0x23,0x0C,0x0C,0x27,0x10,0x07,0x07,0x10,0x40}}, 	     
{0xD5,44,{0x04,0x05,0x06,0x07,0x00,0x01,0x02,0x03,0x20,0x21,0x22,0x23,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x19,0x19,0x18,0x18,0x18,0x18,0x1B,0x1B,0x1A,0x1A,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},  	       
{0xD6,44,{0x03,0x02,0x01,0x00,0x07,0x06,0x05,0x04,0x23,0x22,0x21,0x20,0x18,0x18,0x18,0x18,0x18,0x18,0x58,0x58,0x18,0x18,0x19,0x19,0x18,0x18,0x1B,0x1B,0x1A,0x1A,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},  	       
{0xE0,58,{0x00,0x09,0x15,0x1D,0x20,0x24,0x27,0x26,0x4F,0x60,0x72,0x71,0x7A,0x8B,0x8F,0x94,0xA0,0xA3,0x9D,0xAC,0xBA,0x5C,0x5C,0x5F,0x63,0x66,0x6B,0x7F,0x7F,0x00,0x09,0x15,0x1C,0x20,0x24,0x27,0x26,0x4F,0x60,0x72,0x71,0x7A,0x8C,0x91,0x94,0xA0,0xA3,0x9F,0xAC,0xBB,0x5D,0x5B,0x60,0x65,0x68,0x73,0x7F,0x7F}},	          
{0xCC,01,{0x03}},	         
{0xC0,02,{0x1F,0x73}}, 	        
{0xB6,02,{0x46,0x46}},	        
{0xD4,01,{0x02}}, 	       
{0xBD,01,{0x01}},	      
{0xB1,01,{0x60}}, 	       
{0xBD,01,{0x00}}, 	        
{0xBF,07,{0x40,0x81,0x50,0x00,0x1A,0xFC,0x01}}, 
{0x35,1,{0x00}},//TE  
{0x36,1,{0x02}},	    
{0x11,01,{0x00}},
{REGFLAG_DELAY, 120, {}},        
{0x29,01,{0x00}},
{REGFLAG_DELAY, 10, {}},
};*/



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

   params->dsi.vertical_sync_active				= 2;
    params->dsi.vertical_backporch					= 2;
    params->dsi.vertical_frontporch					= 2;
    params->dsi.vertical_active_line				= FRAME_HEIGHT;

    params->dsi.horizontal_sync_active				= 6;
    params->dsi.horizontal_backporch				= 24;
    params->dsi.horizontal_frontporch				= 12;
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
    params->dsi.PLL_CLOCK = 176;		//288, div by 26

 
	 /*add by sansan.gan,enable mipi non-continue clock begin*/
	params->dsi.noncont_clock = 1;	
	 /*add by sansan.gan,enable mipi non-continue clock end*/
   	/*add by sansan.gan for disable lcm ssc func begin*/
	params->dsi.ssc_disable = 1;
	/*add by sansan.gan for disable lcm ssc func end*/
    params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 0;
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

/*void lcm_pre_init(void)
{
	LCM_PRINT("[LCD] r61318_tdt_dsi_vdo pre init \n");

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
	

	mdelay(10);
	
        lcm_1v8_power_on();
		

	mdelay(10);

        mt_set_gpio_out(GPIO_LCM_PAVDD_EN_PIN, GPIO_OUT_ONE);
        mt_set_gpio_out(GPIO_LCM_NAVDD_EN_PIN, GPIO_OUT_ONE);
		

	tps65132_init();
	

	mdelay(10);
}*/

static void lcm_init(void)
{
	LCM_PRINT("sansan[LCD] 3rd[LCD]hx8394f_txd init \n");
	/*add by sansan.gan for modify the lcd power on consequence 2016-8-30 13:38:25 begin*/

	//lcm_pre_init();
	/*add by sansan.gan for modify the lcd power on consequence 2016-8-30 13:38:25 end*/

	mt_set_gpio_mode(GPIO_LCM_RESET_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM_RESET_PIN, GPIO_DIR_OUT);

	//MDELAY(10);
	mt_set_gpio_out(GPIO_LCM_RESET_PIN, GPIO_OUT_ONE);//mt_set_gpio_out(GPIO_LCM_RESET_PIN, GPIO_OUT_ONE);for 4thlcd
	MDELAY(10);
	mt_set_gpio_out(GPIO_LCM_RESET_PIN, GPIO_OUT_ZERO);
	MDELAY(10);
	mt_set_gpio_out(GPIO_LCM_RESET_PIN, GPIO_OUT_ONE);
	MDELAY(50);

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

	LCM_PRINT("[LCD] lcm_resume \n");
}

#if 0
static unsigned int lcm_compare_id(void)
{
        unsigned int id1_state = 0;
        unsigned int id2_state = 0;
 
        mt_set_gpio_mode(GPIO_LCM_ID1_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCM_ID1_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_LCM_ID1_PIN, GPIO_PULL_DISABLE);// def 0

        mt_set_gpio_mode(GPIO_LCM_ID2_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCM_ID2_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_LCM_ID2_PIN, GPIO_PULL_DISABLE);// def 0
        lcm_1v8_power_on();
        
        mdelay(10);

        id1_state = mt_get_gpio_in(GPIO_LCM_ID1_PIN);
        id2_state = mt_get_gpio_in(GPIO_LCM_ID2_PIN);
	LCM_PRINT("4th[LCD]hx8394d_tdt id1_state:%d,id2_state:%d!\n",id1_state,id2_state);
        if ((id1_state == 1) && (id2_state == 1))
        {
                LCM_PRINT("4th[LCD]hx8394d_tdt lcd selected!\n");
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
	   
LCM_DRIVER hx8394f_txd_dsi_vdo_lcm_drv_3rd = 
{
    .name			= "hx8394f_txd_dsi_vdo_3rd",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
//    .compare_id     = lcm_compare_id,
};

