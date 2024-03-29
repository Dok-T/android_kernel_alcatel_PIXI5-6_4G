#ifndef __SECOND_LCM_INIT_CODE_H__
#define __SECOND_LCM_INIT_CODE_H__
#include "lcm_drv.h"


#define REGFLAG_DELAY             							0xFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER

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
	{0xC0, 6, {0x23,0xB2,0x0f,0x0f,0x02,0x7F}},
	{0xC1, 7, {0x23,0x80,0xA0,0x23,0x00,0x00,0x55}},
	{0xC4, 3, {0xB4,0xB0,0x00}},
	{0xC3, 1, {0x20}},
	{0xC5, 3, {0x06, 0x02, 0x00}},
	{0xC6, 1, {0xC1}},
	{0xC8, 28, {0x51,0x6B,0xAD,0xB5,0xD6,0x00,0x2A,0x7C,0xAE,0x2D,0x76,0x90,0x8C,0xB5,0xD6,0x1A,0x25,0x3A,0x6C,0x35,0xF7,0x60,0x05,0xA1,0xD6,0x5A,0x6B,0xAD}},
	{0xCA, 30, {0x04,0x0F,0x16,0x1c,0x23,0x29,0x2B,0x29,0x1B,0x1B,0x18,0x14,0x0E,0x0D,0x0E,0x01,0x0F,0x16,0x1C,0x23,0x29,0x2B,0x29,0x1B,0x1B,0x18,0x14,0x0E,0x0D,0x0E}},
	{0xD0, 3, {0x05,0x10,0x4B}},
	{0xD1, 1, {0x03}},
	{0xD2, 2, {0x91, 0x1F}},
	{0xD3, 2, {0x44, 0x33}},//0x33,0x33
	{0xD4, 1, {0x2B}},//0x33
	{0xD5, 2, {0x34, 0x34}},//0x30,0x30
	{0xD6, 1, {0x01}},
	{0x35, 1, {0x00}},
        {0x11, 0, {}},
	{REGFLAG_DELAY, 150, {}},
	{0x29, 0, {}},
	{REGFLAG_DELAY, 50, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}},
};

#endif
