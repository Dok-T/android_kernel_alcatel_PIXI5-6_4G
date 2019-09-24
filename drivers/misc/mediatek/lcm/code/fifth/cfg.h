#ifndef __FIFTH_LCM_INIT_CODE_H__
#define __FIFTH_LCM_INIT_CODE_H__
#include "lcm_drv.h"

#define REGFLAG_DELAY             							0xFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER

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
};



#endif