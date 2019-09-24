/*
 * MELFAS MMS400 Touchscreen for MediaTek
 *
 * Copyright (C) 2014 MELFAS Inc.
 *
 */

#ifndef TOUCHPANEL_H__
#define TOUCHPANEL_H__
//#include <pmic_drv.h>

/* Pre-defined definition */
#define TPD_TYPE_CAPACITIVE

//#define TPD_POWER_SOURCE	MT6323_POWER_LDO_VGP1
#define TPD_POWER_SOURCE_CUSTOM PMIC_APP_CAP_TOUCH_VDD
#define TPD_I2C_BUS		1
#define TPD_I2C_ADDR		0x48	//0x34

//#define TPD_WAKEUP_TRIAL	60
//#define TPD_WAKEUP_DELAY	100

//#define TPD_HAVE_TREMBLE_ELIMINATION
//#define TPD_HAVE_CALIBRATION

//Screen
#define TPD_RES_X	1080
#define TPD_RES_Y	1920
#define LCD_X TPD_RES_X
#define LCD_Y TPD_RES_Y

//#define	TPD_XY_INVERT
//#define	TPD_X_INVERT
//#define	TPD_Y_INVERT

//Key
/* Define the virtual button mapping */
//#define TPD_HAVE_BUTTON
#define TPD_KEY_COUNT		3
#define TPD_KEYS			{KEY_BACK, KEY_HOMEPAGE, KEY_APPSELECT}
#define TPD_BUTTON_HEIGHT	40
#define TPD_KEYS_DIM        {{90,1200,20,TPD_BUTTON_HEIGHT}, {270,1200,20,TPD_BUTTON_HEIGHT}, {450,1200,20,TPD_BUTTON_HEIGHT}}


/* Define the touch dimension */
#ifdef TPD_HAVE_BUTTON
//#define TPD_TOUCH_HEIGHT_RATIO	80
//#define TPD_DISPLAY_HEIGHT_RATIO	73
#endif

/* Define the 0D button mapping */
#ifdef TPD_HAVE_BUTTON
//#define TPD_0D_BUTTON {KEY_MENU, KEY_HOMEPAGE, KEY_BACK}
#endif

#define CUSTOM_LIGHTUP_SCREEN 1
#ifdef CUSTOM_LIGHTUP_SCREEN
#define  KEY_LIGHTUP KEY_WAKEUP
#endif

#endif /* TOUCHPANEL_H__ */

