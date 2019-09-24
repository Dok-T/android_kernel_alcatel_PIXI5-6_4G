#ifndef __FIRST_LCM_INIT_CODE_H__
#define __FIRST_LCM_INIT_CODE_H__

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0XFD   // END OF REGISTERS MARKER

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_vdo_initialization_setting[] = {

	{0x00,1,{0x00}}, 
	{0x11,1,{0x00}}, //
	{REGFLAG_DELAY, 190, {}},
	{0x00,1,{0x00}}, 
	{0x29,1,{0x00}}, //
	{REGFLAG_DELAY, 70, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 40, {}},
	// Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 185, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

#endif
