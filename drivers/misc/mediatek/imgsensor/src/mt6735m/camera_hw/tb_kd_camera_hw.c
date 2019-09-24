#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"

#include <mach/gpio_const.h>
#include <mt_gpio.h>
/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, args...)    pr_debug(PFX  fmt, ##args)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...) pr_err(fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...)  pr_debug(PFX  fmt, ##args)

#else
#define PK_DBG(a, ...)
#define PK_ERR(a, ...)
#define PK_XLOG_INFO(fmt, args...)
#endif

/* GPIO Pin control*/
struct platform_device *cam_plt_dev = NULL;
struct pinctrl *camctrl = NULL;
struct pinctrl_state *cam0_pnd_h = NULL;
struct pinctrl_state *cam0_pnd_l = NULL;
struct pinctrl_state *cam0_rst_h = NULL;
struct pinctrl_state *cam0_rst_l = NULL;
struct pinctrl_state *cam1_pnd_h = NULL;
struct pinctrl_state *cam1_pnd_l = NULL;
struct pinctrl_state *cam1_rst_h = NULL;
struct pinctrl_state *cam1_rst_l = NULL;
struct pinctrl_state *cam_ldo0_h = NULL;
struct pinctrl_state *cam_ldo0_l = NULL;
struct pinctrl_state *cam_flash_en_h = NULL;
struct pinctrl_state *cam_flash_en_l = NULL;
struct pinctrl_state *cam_main_id = NULL;
struct pinctrl_state *cam_sub_id = NULL;

//xuejian
#define GPIO_CAM_AVDD_EN_PIN			(GPIO19 | 0x80000000)
#define GPIO_CAM_AF_EN_PIN			(GPIO57 | 0x80000000)
#define GPIO_CAM_MAIN_PDN_PIN                   (GPIO82 | 0x80000000)

#if 0
static void enable_main_pdn_pin(int val)
{
	if (val){
		mt_set_gpio_mode(GPIO_CAM_MAIN_PDN_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_CAM_MAIN_PDN_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CAM_MAIN_PDN_PIN, GPIO_OUT_ONE);
	}else{
		mt_set_gpio_mode(GPIO_CAM_MAIN_PDN_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_CAM_MAIN_PDN_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CAM_MAIN_PDN_PIN, GPIO_OUT_ZERO);
	}

}
#endif


static void enable_avdd_pin(int val)
{
	if (val){
		mt_set_gpio_mode(GPIO_CAM_AVDD_EN_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_CAM_AVDD_EN_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CAM_AVDD_EN_PIN, GPIO_OUT_ONE);
	}else{
		mt_set_gpio_mode(GPIO_CAM_AVDD_EN_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_CAM_AVDD_EN_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CAM_AVDD_EN_PIN, GPIO_OUT_ZERO);
	}

}

static void AF_en_pin(int val)
{
	if (val){
		mt_set_gpio_mode(GPIO_CAM_AF_EN_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_CAM_AF_EN_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CAM_AF_EN_PIN, GPIO_OUT_ONE);
	}else{
		mt_set_gpio_mode(GPIO_CAM_AF_EN_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_CAM_AF_EN_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CAM_AF_EN_PIN, GPIO_OUT_ZERO);
	}
}


int mtkcam_gpio_init(struct platform_device *pdev)
{
	int ret = 0;

	camctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(camctrl)) {
		dev_err(&pdev->dev, "Cannot find camera pinctrl!");
		ret = PTR_ERR(camctrl);
	}
	/*Cam0 Power/Rst Ping initialization */
	cam0_pnd_h = pinctrl_lookup_state(camctrl, "cam0_pnd1");
	if (IS_ERR(cam0_pnd_h)) {
		ret = PTR_ERR(cam0_pnd_h);
		pr_debug("%s : pinctrl err, cam0_pnd_h\n", __func__);
	}

	cam0_pnd_l = pinctrl_lookup_state(camctrl, "cam0_pnd0");
	if (IS_ERR(cam0_pnd_l)) {
		ret = PTR_ERR(cam0_pnd_l);
		pr_debug("%s : pinctrl err, cam0_pnd_l\n", __func__);
	}


	cam0_rst_h = pinctrl_lookup_state(camctrl, "cam0_rst1");
	if (IS_ERR(cam0_rst_h)) {
		ret = PTR_ERR(cam0_rst_h);
		pr_debug("%s : pinctrl err, cam0_rst_h\n", __func__);
	}

	cam0_rst_l = pinctrl_lookup_state(camctrl, "cam0_rst0");
	if (IS_ERR(cam0_rst_l)) {
		ret = PTR_ERR(cam0_rst_l);
		pr_debug("%s : pinctrl err, cam0_rst_l\n", __func__);
	}

	/*Cam1 Power/Rst Ping initialization */
	cam1_pnd_h = pinctrl_lookup_state(camctrl, "cam1_pnd1");
	if (IS_ERR(cam1_pnd_h)) {
		ret = PTR_ERR(cam1_pnd_h);
		pr_debug("%s : pinctrl err, cam1_pnd_h\n", __func__);
	}

	cam1_pnd_l = pinctrl_lookup_state(camctrl, "cam1_pnd0");
	if (IS_ERR(cam1_pnd_l)) {
		ret = PTR_ERR(cam1_pnd_l);
		pr_debug("%s : pinctrl err, cam1_pnd_l\n", __func__);
	}


	cam1_rst_h = pinctrl_lookup_state(camctrl, "cam1_rst1");
	if (IS_ERR(cam1_rst_h)) {
		ret = PTR_ERR(cam1_rst_h);
		pr_debug("%s : pinctrl err, cam1_rst_h\n", __func__);
	}


	cam1_rst_l = pinctrl_lookup_state(camctrl, "cam1_rst0");
	if (IS_ERR(cam1_rst_l)) {
		ret = PTR_ERR(cam1_rst_l);
		pr_debug("%s : pinctrl err, cam1_rst_l\n", __func__);
	}
	/*externel LDO enable */
	cam_ldo0_h = pinctrl_lookup_state(camctrl, "cam_ldo0_1");
	if (IS_ERR(cam_ldo0_h)) {
		ret = PTR_ERR(cam_ldo0_h);
		pr_debug("%s : pinctrl err, cam_ldo0_h\n", __func__);
	}


	cam_ldo0_l = pinctrl_lookup_state(camctrl, "cam_ldo0_0");
	if (IS_ERR(cam_ldo0_l)) {
		ret = PTR_ERR(cam_ldo0_l);
		pr_debug("%s : pinctrl err, cam_ldo0_l\n", __func__);
	}
	/*flash en pin*/
#if 0
	cam_flash_en_h = pinctrl_lookup_state(camctrl, "cam_flash1");
	if (IS_ERR(cam_flash_en_h)) {
		ret = PTR_ERR(cam_flash_en_h);
		pr_debug("%s : pinctrl err, cam_flash_en_h\n", __func__);
	}


	cam_flash_en_l = pinctrl_lookup_state(camctrl, "cam_flash0");
	if (IS_ERR(cam_flash_en_l)) {
		ret = PTR_ERR(cam_flash_en_l);
		pr_debug("%s : pinctrl err, cam_flash_en_l\n", __func__);
	}
#endif

	/*camera id init state */
	cam_main_id = pinctrl_lookup_state(camctrl, "main_ctype");
	if (IS_ERR(cam_main_id)) {
		ret = PTR_ERR(cam_main_id);
		pr_debug("%s : pinctrl err, cam_main_id\n", __func__);
	}else{
		pinctrl_select_state(camctrl, cam_main_id);
	}

	cam_sub_id = pinctrl_lookup_state(camctrl, "sub_ctype");
	if (IS_ERR(cam_sub_id)) {
		ret = PTR_ERR(cam_sub_id);
		pr_debug("%s : pinctrl err, cam_sub_id\n", __func__);
	}else{
		pinctrl_select_state(camctrl, cam_sub_id);
	}
	//end

	return ret;
}

//xuejian.zhong add contrl flash en pin state
#if 0
void mtkcam_flash_en_pin_set(int val)
{
		if (val == 0)
			pinctrl_select_state(camctrl, cam_flash_en_l);
		else
			pinctrl_select_state(camctrl, cam_flash_en_h);
}
EXPORT_SYMBOL(mtkcam_flash_en_pin_set);
#endif
//end

int mtkcam_gpio_set(int PinIdx, int PwrType, int Val)
{
	int ret = 0;

	switch (PwrType) {
	case CAMRST:
		if (PinIdx == 0) {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam0_rst_l);
			else
				pinctrl_select_state(camctrl, cam0_rst_h);
		} else {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam1_rst_l);
			else
				pinctrl_select_state(camctrl, cam1_rst_h);
		}
		break;
	case CAMPDN:
		if (PinIdx == 0) {
#if 1
			if (Val == 0)
				pinctrl_select_state(camctrl, cam0_pnd_l);
			else
				pinctrl_select_state(camctrl, cam0_pnd_h);
#else
			enable_main_pdn_pin(Val);
#endif
		} else {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam1_pnd_l);
			else
				pinctrl_select_state(camctrl, cam1_pnd_h);
		}

		break;
	case CAMLDO:
#if 0
		if (Val == 0)
			pinctrl_select_state(camctrl, cam_ldo0_l);
		else
			pinctrl_select_state(camctrl, cam_ldo0_h);
#else
		enable_avdd_pin(Val);
#endif
		break;
	default:
		PK_DBG("PwrType(%d) is invalid !!\n", PwrType);
		break;
	};

	PK_DBG("PinIdx(%d) PwrType(%d) val(%d)\n", PinIdx, PwrType, Val);

	return ret;
}




int cntVCAMD = 0;
int cntVCAMA = 0;
int cntVCAMIO = 0;
int cntVCAMAF = 0;
int cntVCAMD_SUB = 0;

static DEFINE_SPINLOCK(kdsensor_pw_cnt_lock);


bool _hwPowerOnCnt(KD_REGULATOR_TYPE_T powerId, int powerVolt, char *mode_name)
{
	if (_hwPowerOn(powerId, powerVolt)) {
		spin_lock(&kdsensor_pw_cnt_lock);
		if (powerId == VCAMD)
			cntVCAMD += 1;
		else if (powerId == VCAMA)
			cntVCAMA += 1;
		else if (powerId == VCAMIO)
			cntVCAMIO += 1;
		else if (powerId == VCAMAF)
			cntVCAMAF += 1;
		else if (powerId == SUB_VCAMD)
			cntVCAMD_SUB += 1;
		spin_unlock(&kdsensor_pw_cnt_lock);
		return true;
	}
	return false;
}

bool _hwPowerDownCnt(KD_REGULATOR_TYPE_T powerId, char *mode_name)
{

	if (_hwPowerDown(powerId)) {
		spin_lock(&kdsensor_pw_cnt_lock);
		if (powerId == VCAMD)
			cntVCAMD -= 1;
		else if (powerId == VCAMA)
			cntVCAMA -= 1;
		else if (powerId == VCAMIO)
			cntVCAMIO -= 1;
		else if (powerId == VCAMAF)
			cntVCAMAF -= 1;
		else if (powerId == SUB_VCAMD)
			cntVCAMD_SUB -= 1;
		spin_unlock(&kdsensor_pw_cnt_lock);
		return true;
	}
	return false;
}

void checkPowerBeforClose(char *mode_name)
{

	int i = 0;

	PK_DBG
	    ("[checkPowerBeforClose]cntVCAMD:%d, cntVCAMA:%d,cntVCAMIO:%d, cntVCAMAF:%d, cntVCAMD_SUB:%d,\n",
	     cntVCAMD, cntVCAMA, cntVCAMIO, cntVCAMAF, cntVCAMD_SUB);


	for (i = 0; i < cntVCAMD; i++)
		_hwPowerDown(VCAMD);
	for (i = 0; i < cntVCAMA; i++)
		_hwPowerDown(VCAMA);
	for (i = 0; i < cntVCAMIO; i++)
		_hwPowerDown(VCAMIO);
	for (i = 0; i < cntVCAMAF; i++)
		_hwPowerDown(VCAMAF);
	for (i = 0; i < cntVCAMD_SUB; i++)
		_hwPowerDown(SUB_VCAMD);

	cntVCAMD = 0;
	cntVCAMA = 0;
	cntVCAMIO = 0;
	cntVCAMAF = 0;
	cntVCAMD_SUB = 0;

}


u32 gl_camera_index;
int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, bool On,
		       char *mode_name)
{

	u32 pinSetIdx = 0;

#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4
#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3

#define VOL_2800 2800000
#define VOL_1800 1800000
#define VOL_1500 1500000
#define VOL_1200 1200000
#define VOL_1000 1000000

	u32 pinSet[3][8] = {

		{CAMERA_CMRST_PIN,
		 CAMERA_CMRST_PIN_M_GPIO,	/* mode */
		 GPIO_OUT_ONE,	/* ON state */
		 GPIO_OUT_ZERO,	/* OFF state */
		 CAMERA_CMPDN_PIN,
		 CAMERA_CMPDN_PIN_M_GPIO,
		 GPIO_OUT_ONE,
		 GPIO_OUT_ZERO,
		 },
		{CAMERA_CMRST1_PIN,
		 CAMERA_CMRST1_PIN_M_GPIO,
		 GPIO_OUT_ONE,
		 GPIO_OUT_ZERO,
		 CAMERA_CMPDN1_PIN,
		 CAMERA_CMPDN1_PIN_M_GPIO,
		 GPIO_OUT_ONE,
		 GPIO_OUT_ZERO,
		 },
		{GPIO_CAMERA_INVALID,
		 GPIO_CAMERA_INVALID,	/* mode */
		 GPIO_OUT_ONE,	/* ON state */
		 GPIO_OUT_ZERO,	/* OFF state */
		 GPIO_CAMERA_INVALID,
		 GPIO_CAMERA_INVALID,
		 GPIO_OUT_ONE,
		 GPIO_OUT_ZERO,
		 }
	};


	if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx)
		pinSetIdx = 0;
	 else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx)
		pinSetIdx = 1;
	 else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx)
		pinSetIdx = 2;

//xuejian.zhong change
	gl_camera_index = pinSetIdx;
//end

	printk("cuckoo currSensorName : %s camPos: %d powerOn: %d\n",currSensorName,pinSetIdx,On);
	if (On) {

		if (pinSetIdx == 1 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_GC5005_MIPI_RAW))) {
			PK_DBG("cuckoo gc5005 no need enable mclk at here!\n");
		} else if (currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_HI553_MIPI_RAW))) {
			PK_DBG("cuckoo hi553 no need enable mclk at here!\n");
		} else if (pinSetIdx == 0 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_HI843_MIPI_RAW))) {
			PK_DBG("cuckoo hi843 no need enable mclk at here!\n");
			//ISP_MCLK1_EN(0);
		}	else
			ISP_MCLK1_EN(1);
		
		if (pinSetIdx == 1 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_GC5005_MIPI_RAW))) {
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
						mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]);
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
						mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
					mdelay(1);
					/* VCAM_IO */
					if(TRUE != _hwPowerOnCnt(VCAMIO, VOL_1800, mode_name))
					{
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
					}

					mdelay(1);
					/* VCAMD */
					if(TRUE != _hwPowerOnCnt(VCAMD, VOL_1200, mode_name))
					{
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
					}
					mdelay(1);

					/* VCAMA */
					mtkcam_gpio_set(pinSetIdx, CAMLDO, 1);
					mdelay(5);

					ISP_MCLK1_EN(1);
					mdelay(10);

					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
						mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
					mdelay(1);

					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
						mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);
					mdelay(1);

		}
		else if (pinSetIdx == 1 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_HI553_MIPI_RAW))) {
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
						mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
						mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
					mdelay(1);
					/* VCAM_IO */
					if(TRUE != _hwPowerOnCnt(VCAMIO, VOL_1800, mode_name))
					{
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
					}

					mdelay(1);
					/* VCAMA */
					mtkcam_gpio_set(pinSetIdx, CAMLDO, 1);
					mdelay(5);

					/* VCAMD */
					if(TRUE != _hwPowerOnCnt(VCAMD, VOL_1200, mode_name))
					{
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
					}

					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
						mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]);
					mdelay(1);
					ISP_MCLK1_EN(1);
					mdelay(10);

					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
						mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);
					mdelay(1);

		}
		else if (pinSetIdx == 0 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_HI843_MIPI_RAW))) {
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
						mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
						mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
					mdelay(1);
					/* VCAM_IO */
					if(TRUE != _hwPowerOnCnt(VCAMIO, VOL_1800, mode_name))
					{
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
					}

					mdelay(1);
					/* VCAMA */
					mtkcam_gpio_set(pinSetIdx, CAMLDO, 1);
					mdelay(5);

					/* VCAMD */
					if(TRUE != _hwPowerOnCnt(VCAMD, VOL_1200, mode_name))
					{
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
					}
					mdelay(1);
					ISP_MCLK1_EN(1);
					mdelay(1);

					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
						mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]);
					mdelay(10);

					/* VCAMAF */
					if(TRUE != _hwPowerOnCnt(VCAMAF, VOL_2800, mode_name))
					{
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_AF), power id = %d \n", VCAMAF);
						goto _kdCISModulePowerOn_exit_;
					}
					/*AF EN PIN*/
					AF_en_pin(1);

					mdelay(1);
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
						mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);
					mdelay(1);

		}
		else if (pinSetIdx == 0 && currSensorName && ((0 == strcmp(currSensorName, SENSOR_DRVNAME_OV13858_MIPI_RAW)) ||
		            (0 == strcmp(currSensorName, SENSOR_DRVNAME_OV13858S_MIPI_RAW)))) {
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
						mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
					mdelay(1);
					/* VCAMA */
					mtkcam_gpio_set(pinSetIdx, CAMLDO, 1);
					mdelay(5);

					/* VCAM_IO */
					if(TRUE != _hwPowerOnCnt(VCAMIO, VOL_1800, mode_name))
					{
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
					}
					mdelay(1);

					/* VCAMD */
					if(TRUE != _hwPowerOnCnt(VCAMD, VOL_1200, mode_name))
					{
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
					}

					mdelay(1);
					/* VCAMAF */
					if(TRUE != _hwPowerOnCnt(VCAMAF, VOL_2800, mode_name))
					{
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_AF), power id = %d \n", VCAMAF);
						goto _kdCISModulePowerOn_exit_;
					}
					/*AF EN PIN*/
					AF_en_pin(1);
					mdelay(1);
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
						mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);

		}
		#ifdef CONFIG_PROJECT_PIXI5104G
		else if (pinSetIdx == 0 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_OV5675_MIPI_RAW))) {
		#else
		else if (pinSetIdx == 1 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_OV5675_MIPI_RAW))) {
		#endif
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
						mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
						mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
					mdelay(1);

          #ifdef CONFIG_PROJECT_PIXI5104G
					/* VCAMA */
			    if (TRUE != _hwPowerOnCnt(VCAMA, VOL_2800, mode_name)) {
				           goto _kdCISModulePowerOn_exit_;
			    }
			    #else
			    mtkcam_gpio_set(pinSetIdx, CAMLDO, 1);
			    #endif
			    
					mdelay(1);
					/* VCAM_IO */
					if(TRUE != _hwPowerOnCnt(VCAMIO, VOL_1800, mode_name))
					{
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
					}
					mdelay(1);

					/* VCAMD */
					if(TRUE != _hwPowerOnCnt(VCAMD, VOL_1200, mode_name))
					{
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
					}
					mdelay(5);

					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
						mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]);
					mdelay(1);

					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
						mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);
					mdelay(1);

		}
		else if (pinSetIdx == 0 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_OV8856_MIPI_RAW))) {
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
						mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
					mdelay(1);
					/* VCAM_IO */
					if(TRUE != _hwPowerOnCnt(VCAMIO, VOL_1800, mode_name))
					{
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
					}

					mdelay(1);
					/* VCAMD */
					if(TRUE != _hwPowerOnCnt(VCAMD, VOL_1200, mode_name))
					{
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
					}

					mdelay(1);
					/* VCAMA */
					mtkcam_gpio_set(pinSetIdx, CAMLDO, 1);

					/* VCAMAF */
					if(TRUE != _hwPowerOnCnt(VCAMAF, VOL_2800, mode_name))
					{
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_AF), power id = %d \n", VCAMAF);
						goto _kdCISModulePowerOn_exit_;
					}
					/*AF EN PIN*/
					AF_en_pin(1);

					mdelay(5);
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
						mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);
		}
		else if (pinSetIdx == 0 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_GC2145_MIPI_YUV))) {
					/* First Power Pin low and Reset Pin Low */
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
						mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]);
		
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
						mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
		
		
					mdelay(1);
					/* VCAM_IO */
					if(TRUE != _hwPowerOnCnt(VCAMIO, VOL_1800, mode_name))
					{
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
					}
		
					mdelay(2);
					
					/* VCAM_A */
						mtkcam_gpio_set(pinSetIdx, CAMLDO, 1);
					
					mdelay(1);
		
					/* enable active sensor */		 
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
						mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
					
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
						mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);
		} 
		else if (pinSetIdx == 0 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_HI553_REAR))) {
			
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
						mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
						mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
					mdelay(1);
					/* VCAM_IO */
					if(TRUE != _hwPowerOnCnt(VCAMIO, VOL_1800, mode_name))
					{
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
					}

					mdelay(1);
					/* VCAMA */
			    if (TRUE != _hwPowerOnCnt(VCAMA, VOL_2800, mode_name)) {
				           goto _kdCISModulePowerOn_exit_;
			    }
					mdelay(5);

					/* VCAMD */
					if(TRUE != _hwPowerOnCnt(VCAMD, VOL_1200, mode_name))
					{
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
					}

					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
						mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]);
					mdelay(1);
					ISP_MCLK1_EN(1);
					mdelay(2);

					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
						mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);
					mdelay(1);

		}
	  else if (pinSetIdx == 1 && currSensorName && (0 == strcmp(SENSOR_DRVNAME_SP2509_RAW ,currSensorName))){

        printk("SP2509_power_off_sequence\n");
     
		    if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
						mtkcam_gpio_set(pinSetIdx, CAMPDN, GPIO_OUT_ZERO);
						
		    if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
						mtkcam_gpio_set(pinSetIdx, CAMRST, GPIO_OUT_ZERO);	 
		 
        mdelay(2);
        mtkcam_gpio_set(pinSetIdx, CAMPDN, GPIO_OUT_ONE);
        mdelay(2);    	
			  /* VCAM_IO */
			  if(TRUE != _hwPowerOnCnt(VCAMIO, VOL_1800, mode_name))
        {
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
		    }    
        mdelay(2);
			  /* VCAM_A */
			  if (TRUE != _hwPowerOnCnt(VCAMA, VOL_2800, mode_name)) {
				           goto _kdCISModulePowerOn_exit_;
			  }		
			  mdelay(1);        
        ISP_MCLK1_EN(1);
			  mdelay(1);
			  mtkcam_gpio_set(pinSetIdx, CAMPDN, GPIO_OUT_ZERO);
			  mdelay(2); 
        mtkcam_gpio_set(pinSetIdx, CAMRST, GPIO_OUT_ONE);	 
		
		
		} else if (pinSetIdx == 1 && currSensorName && (0 == strcmp(SENSOR_DRVNAME_GC2375_MIPI_RAW, currSensorName))) {

        printk("lkl SP2375_power_on_sequence\n");
			/* First Power Pin low and Reset Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
						mtkcam_gpio_set(pinSetIdx, CAMPDN, GPIO_OUT_ZERO);


		    if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
						mtkcam_gpio_set(pinSetIdx, CAMRST, GPIO_OUT_ZERO);	 
            mdelay(2);
            ISP_MCLK1_EN(1);
            mdelay(2);
            mtkcam_gpio_set(pinSetIdx, CAMPDN, GPIO_OUT_ONE);
			  mdelay(2);
			  /* VCAM_IO */
			  if(TRUE != _hwPowerOnCnt(VCAMIO, VOL_1800, mode_name))
        {
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
					//	goto _kdCISModulePowerOn_exit_;
		    }   
            mdelay(2);

            /* VCAMD  SUB_VCAMD*/
			  if(TRUE != _hwPowerOnCnt(SUB_VCAMD, VOL_1800, mode_name))
        {
						PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
		    } 
		    
			/* VCAM_A */
			if (TRUE != _hwPowerOnCnt(VCAMA, VOL_2800, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to enable analog power (VCAM_A),power id = %d\n",
				     VCAMA);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(2);

			  mdelay(1);
			  mtkcam_gpio_set(pinSetIdx, CAMPDN, GPIO_OUT_ZERO);
			mdelay(2);
			/* enable active sensor */
        mtkcam_gpio_set(pinSetIdx, CAMRST, GPIO_OUT_ONE);	 

		}
		else {

			/* VCAM_IO */
			if (TRUE != _hwPowerOnCnt(VCAMIO, VOL_1800, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable IO power (VCAM_IO), power id = %d\n", VCAMIO);
				goto _kdCISModulePowerOn_exit_;
			}

		}
	} else { /* power OFF */
		
				PK_DBG("[PowerOFF]pinSetIdx:%d\n", pinSetIdx);
				if (pinSetIdx == 1 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_GC5005_MIPI_RAW))){
					PK_DBG("cuckoo gc5005 no need disable MCLK at here!\n");
				} else if (currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_HI553_MIPI_RAW))){
					PK_DBG("cuckoo hi553 no need disable MCLK at here!\n");
				}	else if (pinSetIdx == 0 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_HI843_MIPI_RAW))){
					PK_DBG("cuckoo hi843 no need disable MCLK at here!\n");
				} else 
					 ISP_MCLK1_EN(0);
		
				if (pinSetIdx == 1 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_GC5005_MIPI_RAW))){
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
						mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]);
					mdelay(1);
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
						mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
					mdelay(1);
					ISP_MCLK1_EN(0);
					mdelay(5);
			
					mtkcam_gpio_set(pinSetIdx, CAMLDO, 0);
					mdelay(5);

					/* VCAMD */
					if(TRUE != _hwPowerDownCnt(VCAMD, mode_name)) {
						PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", VCAMD);
						goto _kdCISModulePowerOn_exit_;
					}
					mdelay(1);

					/* VCAM_IO */
					if(TRUE != _hwPowerDownCnt(VCAMIO, mode_name)) {
						PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
					}
					mdelay(1);

				}
				else if (pinSetIdx == 1 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_HI553_MIPI_RAW))){
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
						mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
					mdelay(1);
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
						mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
					mdelay(1);
					ISP_MCLK1_EN(0);
					mdelay(1);
			
					/* VCAMD */
					if(TRUE != _hwPowerDownCnt(VCAMD, mode_name)) {
						PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", VCAMD);
						goto _kdCISModulePowerOn_exit_;
					}
					mdelay(1);

					mtkcam_gpio_set(pinSetIdx, CAMLDO, 0);
					mdelay(1);

					/* VCAM_IO */
					if(TRUE != _hwPowerDownCnt(VCAMIO, mode_name)) {
						PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
					}
					mdelay(1);

				}
				else if (pinSetIdx == 0 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_HI843_MIPI_RAW))){
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
						mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
					mdelay(1);
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
						mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
					mdelay(1);
					ISP_MCLK1_EN(0);
					mdelay(1);

					/*AF EN PIN*/
					AF_en_pin(0);
					mdelay(1);
					/* VCAMAF */
					if(TRUE != _hwPowerDownCnt(VCAMAF, mode_name)) {
						PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_AF), power id = %d \n", VCAMAF);
						goto _kdCISModulePowerOn_exit_;
					}

					/* VCAMD */
					if(TRUE != _hwPowerDownCnt(VCAMD, mode_name)) {
						PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", VCAMD);
						goto _kdCISModulePowerOn_exit_;
					}
					mdelay(1);

					mtkcam_gpio_set(pinSetIdx, CAMLDO, 0);
					mdelay(1);

					/* VCAM_IO */
					if(TRUE != _hwPowerDownCnt(VCAMIO, mode_name)) {
						PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
					}
					mdelay(1);

				}
				else if (pinSetIdx == 0 && currSensorName && ((0 == strcmp(currSensorName, SENSOR_DRVNAME_OV13858_MIPI_RAW)) ||
				            (0 == strcmp(currSensorName, SENSOR_DRVNAME_OV13858S_MIPI_RAW)))){
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
						mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
					AF_en_pin(0);
					mdelay(1);
					/* VCAMAF */
					if(TRUE != _hwPowerDownCnt(VCAMAF, mode_name)) {
						PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_AF), power id = %d \n", VCAMAF);
						goto _kdCISModulePowerOn_exit_;
					}
					/* VCAMD */
					if(TRUE != _hwPowerDownCnt(VCAMD, mode_name)) {
						PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", VCAMD);
						goto _kdCISModulePowerOn_exit_;
					}
					mdelay(1);

					/* VCAMA */
					mtkcam_gpio_set(pinSetIdx, CAMLDO, 0);
					mdelay(1);

					/* VCAM_IO */
					if(TRUE != _hwPowerDownCnt(VCAMIO, mode_name)) {
						PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
					}
				}
				#ifdef CONFIG_PROJECT_PIXI5104G
				else if (pinSetIdx == 0 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_OV5675_MIPI_RAW))){
				#else
				else if (pinSetIdx == 1 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_OV5675_MIPI_RAW))){
				#endif
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
						mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
					mdelay(1);
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
						mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
					mdelay(1);
					/* VCAMD */
					if(TRUE != _hwPowerDownCnt(VCAMD, mode_name)) {
						PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", VCAMD);
						goto _kdCISModulePowerOn_exit_;
					}
					mdelay(1);

					/* VCAM_IO */
					if(TRUE != _hwPowerDownCnt(VCAMIO, mode_name)) {
						PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
					}
					mdelay(1);
					#ifdef CONFIG_PROJECT_PIXI5104G
					if(TRUE != _hwPowerDownCnt(VCAMA, mode_name)) {
						goto _kdCISModulePowerOn_exit_;
					}
					#else
					mtkcam_gpio_set(pinSetIdx, CAMLDO, 0);
					#endif
					mdelay(1);
				}
				else if (pinSetIdx == 0 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_OV8856_MIPI_RAW))){
					mdelay(1);
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
						mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
					/*AF EN PIN*/
					AF_en_pin(0);
					mdelay(1);
					/* VCAMAF */
					if(TRUE != _hwPowerDownCnt(VCAMAF, mode_name)) {
						PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_AF), power id = %d \n", VCAMAF);
						goto _kdCISModulePowerOn_exit_;
					}
					/* VCAMD */
					if(TRUE != _hwPowerDownCnt(VCAMD, mode_name)) {
						PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", VCAMD);
						goto _kdCISModulePowerOn_exit_;
					}

					/* VCAM_IO */
					if(TRUE != _hwPowerDownCnt(VCAMIO, mode_name)) {
						PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
					}

					mtkcam_gpio_set(pinSetIdx, CAMLDO, 0);
					mdelay(1);
				}
				else if (pinSetIdx == 0 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_GC2145_MIPI_YUV)))
				{
		
					/* Set Power Pin low and Reset Pin Low */
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
						mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]);
					
					/* Set Reset Pin Low */
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
						mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
						 mtkcam_gpio_set(pinSetIdx, CAMLDO, 0);
        					
					/* VCAM_IO */
					if(TRUE != _hwPowerDownCnt(VCAMIO, mode_name)) {
						PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
					}
		
      				   
				}  
				else if (pinSetIdx == 0 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_HI553_REAR)))
				{	
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
						mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
					mdelay(1);
					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
						mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
					mdelay(1);
					ISP_MCLK1_EN(0);
					mdelay(1);
			
					/* VCAMD */
					if(TRUE != _hwPowerDownCnt(VCAMD, mode_name)) {
						PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", VCAMD);
						goto _kdCISModulePowerOn_exit_;
					}
					mdelay(1);

					if(TRUE != _hwPowerDownCnt(VCAMA, mode_name)) {
						goto _kdCISModulePowerOn_exit_;
					}
					mdelay(1);

					/* VCAM_IO */
					if(TRUE != _hwPowerDownCnt(VCAMIO, mode_name)) {
						PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
					}
					mdelay(1);
				}
				else if (pinSetIdx == 1 && currSensorName && (0 == strcmp(SENSOR_DRVNAME_SP2509_RAW ,currSensorName))){

    	     printk("SP2509_power_off_sequence\n"); 

					if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
						mtkcam_gpio_set(pinSetIdx, CAMRST, GPIO_OUT_ZERO);
			     
    	     mdelay(2);    	
    	     ISP_MCLK1_EN(0);
    	     
					 if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
						mtkcam_gpio_set(pinSetIdx, CAMPDN, GPIO_OUT_ONE);
						
    	     mdelay(2);
			     /* VCAM_A */
					if(TRUE != _hwPowerDownCnt(VCAMA, mode_name)) {
						goto _kdCISModulePowerOn_exit_;
					}
			     mdelay(2); 
			     /* VCAM_IO */
					if(TRUE != _hwPowerDownCnt(VCAMIO, mode_name)) {
						PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
					}
			     mdelay(2);
			     mtkcam_gpio_set(pinSetIdx, CAMPDN, GPIO_OUT_ZERO);
				 
		} else if (pinSetIdx == 1 && currSensorName && (0 == strcmp(SENSOR_DRVNAME_GC2375_MIPI_RAW, currSensorName))){
			/* Set Power Pin low and Reset Pin Low */
    	     printk("lkl gc2375_power_off_sequence\n"); 
			     mtkcam_gpio_set(pinSetIdx, CAMPDN, GPIO_OUT_ONE);
    	     mdelay(2); 
           mtkcam_gpio_set(pinSetIdx, CAMRST, GPIO_OUT_ZERO);	
			     
    	     mdelay(2);    	
			     ISP_MCLK1_EN(0);

    	     mdelay(2);
			     /* VCAM_A */
					if(TRUE != _hwPowerDownCnt(VCAMA, mode_name)) {
						goto _kdCISModulePowerOn_exit_;
					}
           mdelay(2);  // SUB_VCAMD
           
					if(TRUE != _hwPowerDownCnt(SUB_VCAMD, mode_name)) {
						PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
					}
					
			     mdelay(2); 
			     /* VCAM_IO */
					if(TRUE != _hwPowerDownCnt(VCAMIO, mode_name)) {
						PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", VCAMIO);
						goto _kdCISModulePowerOn_exit_;
					}
			     mdelay(2);
 
				 
        }  
		else {

			/* VCAM_IO */
			if (TRUE != _hwPowerDownCnt(VCAMIO, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d\n", VCAMIO);
				/* return -EIO; */
				goto _kdCISModulePowerOn_exit_;
			}

		}

	}
	return 0;
		
_kdCISModulePowerOn_exit_:
	return -EIO;

}

EXPORT_SYMBOL(kdCISModulePowerOn);

/* !-- */
/*  */
