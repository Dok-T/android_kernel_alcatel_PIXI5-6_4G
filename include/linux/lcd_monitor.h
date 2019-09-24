#ifndef __LCD_MONITOR_H__
#define __LCD_MONITOR_H__
#define KERNEL_FIRST_LCD "cb47c4865e33e51d745c20920dc7d285"
#define KERNEL_FIFTH_LCD "cb47c4865e33e51d745c20920dc7d285"
#define KERNEL_FIRST_TP "2943c4663772a5a0a9c9a00e937a1e85"
#define KERNEL_SECOND_LCD "42925270d87268f098a4b04f68a213c1"
#define KERNEL_SIXTH_LCD "42925270d87268f098a4b04f68a213c1"
#define KERNEL_SECOND_TP "920f62c902292f39be6de72d33796ec6"
#define LK_FIRST_LCD "90a88955f2705a9863cd32b5c17968d8"
#define LK_SECOND_LCD "6e718e6d6734fcad8d7ec09418943eac"
#define LK_FIFTH_LCD "6e718e6d6734fcad8d7ec09418943eac"
#define LK_SIXTH_LCD "6e718e6d6734fcad8d7ec09418943eac"
/*[BUGFIX]-mod-begin by scdtablet.jinghuang@tcl.com,2016.12.12,3708473*/
/*add the funtion of lcd version for pixi5104g global*/

#ifdef CONFIG_PROJECT_PIXI5104G
#define LK_FOURTH_LCD "abcdef"
#define KERNEL_FOURTH_LCD "abcdef"
#endif
/*[BUGFIX]-mod-end by scdtablet.jinghuang@tcl.com*/

#endif
