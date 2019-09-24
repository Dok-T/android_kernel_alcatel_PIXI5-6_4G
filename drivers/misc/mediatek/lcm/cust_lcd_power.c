#include <linux/string.h>
#include "lcm_drv.h"
#include <mt-plat/mt_gpio.h>
#include <mach/gpio_const.h>
#include <linux/of_platform.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <mach/gpio_const.h>
#include <mt_gpio.h>

//static LCM_UTIL_FUNCS lcm_util = {0};
//#define UDELAY(n) (lcm_util.udelay(n))
//#define MDELAY(n) (lcm_util.mdelay(n))

//extern int tps65132_write_bytes(unsigned char addr, unsigned char value);
#define LCD_DEBUG(fmt, args...) printk(fmt, ##args)
#define LCM_PRINT printk



#define GPIO_LCM_PAVDD_EN_PIN           (GPIO63 | 0x80000000)
#define GPIO_LCM_NAVDD_EN_PIN           (GPIO64 | 0x80000000)
#define GPIO_LCM_ID1_PIN                (GPIO78 | 0x80000000)
#define GPIO_LCM_ID2_PIN                (GPIO79 | 0x80000000)
#define GPIO_LCM_RESET_PIN 		(GPIO146 | 0x80000000)
#define GPIO_LCM_DSI_TE_PIN		(GPIO147 | 0x80000000)
#define GPIO_LCM_1V8_EN			(GPIO58 | 0x80000000)
/*[BUGFIX]-Mod-BEGIN by ke.huang.sz,3303757*/
/*modify for hw board id define*/
#define CUST_BOARD_ID_PIN_GPIO90        	(GPIO90  | 0x80000000) 
#define CUST_BOARD_ID_PIN_GPIO95  			(GPIO95  | 0x80000000)
#define CUST_BOARD_ID_PIN_GPIO122  			(GPIO122 | 0x80000000)	
/********************************************************************
PIXI564G:
GPIO95  GPIO90   GPIO122     Value
 0       0        0    	     EU1S
 0       0        1    	     EU2S
 0       1        0          LATAM1S
 0       1        1          LATAM2S
 1       0        0          MEA
 1       0        1          APAC
 1       1        0          US
 1       1        1          reserved
**********************************************************************/
int select_hw_configuration_func(void)
{
	int board_num=0;
	int pin1=0,pin2=0,pin3=0;	
	mt_set_gpio_mode(CUST_BOARD_ID_PIN_GPIO90,GPIO_MODE_GPIO);
	mt_set_gpio_mode(CUST_BOARD_ID_PIN_GPIO95,GPIO_MODE_GPIO);
	mt_set_gpio_mode(CUST_BOARD_ID_PIN_GPIO122,GPIO_MODE_GPIO);

	mt_set_gpio_dir(CUST_BOARD_ID_PIN_GPIO90,GPIO_DIR_IN);
	mt_set_gpio_dir(CUST_BOARD_ID_PIN_GPIO95,GPIO_DIR_IN);
	mt_set_gpio_dir(CUST_BOARD_ID_PIN_GPIO122,GPIO_DIR_IN);

	pin1 = mt_get_gpio_in(CUST_BOARD_ID_PIN_GPIO90);
	pin2 = mt_get_gpio_in(CUST_BOARD_ID_PIN_GPIO95);
	pin3 = mt_get_gpio_in(CUST_BOARD_ID_PIN_GPIO122);

	board_num = ((pin1 & 1)<< 1)|((pin2 & 1)<< 2)|(pin3 & 1);
	printk("get hw board numeber 0x%x\n",board_num);
	printk("#####current hw configuration#####\n");
	switch(board_num)
	{
		case 0:
		printk("#####EU1S#####\n");
		break;
		case 1:
		printk("#####EU2S#####\n");
		break;
		case 2:
		printk("#####LATAM1S#####\n");
		break;
		case 3:
		printk("#####LATAM2S#####\n");
		break;
		case 4:
		printk("#####MEA#####\n");
		break;
		case 5:
		printk("#####APAC#####\n");
		break;
		case 6:
		printk("#####US#####\n");
		break;
		case 7:
		printk("#####RESERVED#####\n");
		break;
		default:
		printk("#####unkown gpio num#####\n");
		break;		
	}
	printk("############################\n");
	return board_num;
}
EXPORT_SYMBOL(select_hw_configuration_func);
/*[BUGFIX]-Mod-END by ke.huang.sz*/
static void lcm_1v8_power_on(void)
{
	mt_set_gpio_mode(GPIO_LCM_1V8_EN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCM_1V8_EN, GPIO_DIR_OUT);

	mt_set_gpio_out(GPIO_LCM_1V8_EN, GPIO_OUT_ONE);
}

static void lcm_1v8_power_off(void)
{
	mt_set_gpio_mode(GPIO_LCM_1V8_EN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCM_1V8_EN, GPIO_DIR_OUT);

        mt_set_gpio_out(GPIO_LCM_1V8_EN, GPIO_OUT_ZERO);
}




static unsigned int lcm_compatible_id(void)
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
	LCM_PRINT("lcm_compatible_id id1_state:%d,id2_state:%d!\n",id1_state,id2_state);
	if ((id1_state == 0) && (id2_state == 0))
	{
	    LCM_PRINT("1st[LCD]r61318_tdt lcd selected!\n");
	    lcm_1v8_power_off();
	    return 1;
	}
	else if((id1_state == 0) && (id2_state == 1))
	{
		LCM_PRINT("2nd[LCD]r61318_tdt lcd selected!\n");
	    lcm_1v8_power_off();
	    return 2;
	}
	else if((id1_state == 1) && (id2_state == 0))
	{
		LCM_PRINT("3rd[LCD]hx8394f_txd lcd selected!\n");
	    lcm_1v8_power_off();
	    return 3;
	}
	else if((id1_state == 1) && (id2_state == 1))
	{
		LCM_PRINT("4th[LCD]hx8394d_tdt lcd selected!\n");
	    lcm_1v8_power_off();
	    return 4;		
	}
	else
	{
	    lcm_1v8_power_off();
	    return 0;
	}
}

void lcm1st_pre_init(void)
{
	LCM_PRINT("[LCD]1st r61318_tdt_dsi_vdo lcm_pre_init sansan test \n");

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

	/*add by sansan.gan for add bias ic init begin*/
	//tps65132_init();		
	/*add by sansan.gan for add bias ic init end*/
	mdelay(10);
}


void lcm2nd_pre_init(void)
{
	LCM_PRINT("[LCD]2nd r61318_tdt_dsi_vdo lcm_pre_init sansan test \n");

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

	/*add by sansan.gan for add bias ic init begin*/
	//tps65132_init();		
	/*add by sansan.gan for add bias ic init end*/
	mdelay(10);
}

void lcm3rd_pre_init(void)
{
	LCM_PRINT("[LCD]3rd hx8394f_txd_dsi_vdo lcm_pre_init sansan test \n");

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

	mdelay(5);

	lcm_1v8_power_on();

	mdelay(10);

	mt_set_gpio_out(GPIO_LCM_PAVDD_EN_PIN, GPIO_OUT_ONE);
	mdelay(2);
	mt_set_gpio_out(GPIO_LCM_NAVDD_EN_PIN, GPIO_OUT_ONE);

	/*add by sansan.gan for add bias ic init begin*/
	//tps65132_init();		
	/*add by sansan.gan for add bias ic init end*/
	mdelay(10);
}

void lcm4th_pre_init(void)
{
	LCM_PRINT("[LCD]4th hx8394d_tdt_dsi_vdo lcm_pre_init sansan test \n");

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
	mdelay(5);
	mt_set_gpio_out(GPIO_LCM_RESET_PIN, GPIO_OUT_ONE); 


	mdelay(3);
	mt_set_gpio_out(GPIO_LCM_RESET_PIN, GPIO_OUT_ZERO);
	mdelay(5);
	mt_set_gpio_out(GPIO_LCM_RESET_PIN, GPIO_OUT_ONE);
	mdelay(3);
	mt_set_gpio_out(GPIO_LCM_PAVDD_EN_PIN, GPIO_OUT_ONE);
	mdelay(10);
	mt_set_gpio_out(GPIO_LCM_NAVDD_EN_PIN, GPIO_OUT_ONE);

	/*add by sansan.gan for add bias ic init begin*/
	//tps65132_init();	
	/*add by sansan.gan for add bias ic init end*/
	mdelay(10);
}


void lcm_pre_init(void)
{
	unsigned int ret;
	LCM_PRINT("kernel [LCD]lcm_pre_init...sansan test\n");
	ret = lcm_compatible_id();
	if(ret == 1)
	{
		lcm1st_pre_init();
	}
	else if(ret == 2)
	{
		lcm2nd_pre_init();	
	}
	else if(ret == 3)
	{
		lcm3rd_pre_init();
	}
	else if(ret == 4)
	{
		lcm4th_pre_init();
	}

}

EXPORT_SYMBOL(lcm_pre_init);
