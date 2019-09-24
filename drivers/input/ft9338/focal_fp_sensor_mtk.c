/* Focaltech's FT9338/FT9336
 *  fingerprint sensor linux driver for TEE
 *
 * 2010 - 2016 Focaltech-Electronics Co.,LTD
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#include <linux/device.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/fb.h>
#include <linux/rtpm_prio.h>//zengfl add

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#else
#include <linux/notifier.h>
#endif

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#endif

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#ifdef CONFIG_MTK_CLKMGR
#include "mach/mt_clkmgr.h"
#else
#include <linux/clk.h>
#endif
#include <linux/wait.h>
#include <net/sock.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <linux/wakelock.h>
/*MTK header*/
#include "mt_spi.h"
#include "mt_spi_hal.h"
#include "mt_gpio.h"
#include "mach/gpio_const.h"

#include "focal_fp_sensor_mtk.h"
#include "../fp_drv.h"
#include <mt_gpio.h>
static int init_flag = 0;
#define FOCAL_FP_DEBUG

/*[BUGFIX]-Mod-BEGIN by ke.huang.sz,3408015*/
/*modify the figner key value*/
#ifndef GF_INPUT_HOME_KEY
#define GF_INPUT_HOME_KEY 255
#endif
/*[BUGFIX]-Mod-END by ke.huang.sz*/
//#ifdef FOCAL_FP_DEBUG
#define FT_DBG(fmt, args...)	printk(KERN_ERR"  focal  " fmt, ##args);
//#else
//#define FT_DBG(fmt, args...) 	do{}while(0)
//#endif

#define MTK_SPI_DMA_MODE
#ifdef MTK_SPI_DMA_MODE
#define MAX_SPI_RW_LEN		1024
#else
#define MAX_SPI_RW_LEN		32
#endif

#ifdef FOCAL_FP_SYSFS_DBG
u8 g_captureimage_mode = 0;
#endif


#define FOCAL_ENABLE_FP_SENSOR 		1
#define FOCAL_DISABLE_FP_SENSOR		0
#define FOCAL_FP_FINGER_ON				1		
#define FOCAL_FP_FINGER_OFF			0
#define FOCAL_ENABLE_INTERRUPT_WAKE	1
#define FOCAL_DISABLE_INTERRUPT_WAKE	0

#define FOCAL_MCU_IDLE               1
#define FOCAL_MCU_BUSY              0

#define TABLENUM	8
#define DOWNLOAD_COUNT      3

#define FOCAL_FP_INTERRUPT          0
#define FOCAL_FP_RESUME                1
#define FOCAL_FP_SUSPEND              2

static u8 g_enableSensor = FOCAL_ENABLE_FP_SENSOR;
static u8 g_fpfingeron = FOCAL_FP_FINGER_OFF;
static u8 g_fpinterruptwake = FOCAL_ENABLE_INTERRUPT_WAKE;
static u8 g_fpmcustatus = FOCAL_MCU_BUSY;

#define FOCAL_FP_DETECT_FUNC

//#define SPI_DEV_NAME   "fp_spi"
//#define SPI_DEV_NAME	"silead_fp"
/*device name after register in charater*/
#define DEV_NAME "focal_fp_spi"

#define CHRD_DRIVER_NAME                "focal_fp"
#define CLASS_NAME                      "focal_fp_spi"
#define SPIDEV_MAJOR                    158     /* assigned */
u8 rbuf[40]= {0};
u8 tbuf[32]={0};
dev_t devno;
struct wake_lock    fp_irq_lock;
struct wake_lock    fp_ca_lock;

unsigned int eint_gpio;
struct pinctrl *pinctrl_fp = NULL;
struct pinctrl_state *ftsFP_pins_default;
//struct pinctrl_state *pins_miso_spi, *pins_miso_pullhigh, *pins_miso_pulllow;
struct pinctrl_state *pins_reset_high, *pins_reset_low, *pins_power_on, *pins_power_off, *pins_irq_gpio;

struct focal_fp_sensor_data *g_fp_spi_sensor;
static struct focal_fp_rw_operate g_fp_rw_stru;
static u8 *g_rw_buf;

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned bufsiz = 1024;

extern int focal_fp_fw_upgrade(struct focal_fp_sensor_data *spi_sensor);
extern int focal_fp_fw_download(struct focal_fp_sensor_data *spi_sensor);

module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");							
/*-------------------------------------------------------------------------*/
static int focal_fp_enable_sensor(struct focal_fp_sensor_data *spi_sensor, u8 enable);

#ifdef CONFIG_OF
/* This match string should be changed according the platform's DTS infomation */
#define DTS_INFO_MATCH_STR	"mediatek,focal-fp"
static const struct of_device_id focal_of_match[] = {
	{ .compatible = DTS_INFO_MATCH_STR, },
	{},
};

#if 0
static  struct spi_device_id spi_id_table = {
	"fingerprint",0
};
#endif

MODULE_DEVICE_TABLE(of, focal_of_match);
#endif
static struct mt_chip_conf focal_chip_config = {
		/*
		* These two determine the SPI_CLK frequency
		* SPI_CLK = 134300 / (high_time + low_time), unit KHz
		* To set 25MHz, set each to 3, then is 22.383 MHz
		* To set 10MHz, set each to 7, then is 9.592 MHz
		* To set 2MHz, set each to 33, then is 2.034 M
		*/
	.setuptime = 20,//500,//20,
	.holdtime = 20,//500,//20,
	.high_time = 50,//50,
	.low_time = 50,//50,
	.cs_idletime = 20,
	.ulthgh_thrsh = 0,
	.cpol = 0,//0,
	.cpha = 0,//0,
	.rx_mlsb = 1,//1,
	.tx_mlsb = 1,//1,
	.tx_endian = 0,//0,
	.rx_endian =  0,//0,
#if 1//def MTK_SPI_DMA_MODE
	.com_mod = DMA_TRANSFER,
#else
	.com_mod = FIFO_TRANSFER,
#endif
	.pause = PAUSE_MODE_ENABLE,//PAUSE_MODE_ENABLE,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};
/* -------------------------------------------------------------------- */
/* fingerprint chip platform hardware configuration								  */
/* -------------------------------------------------------------------- */

static void focal_fp_sensor_complete(void *arg)
{
	complete(arg);
}

static ssize_t
focal_fp_sensor_sync(struct focal_fp_sensor_data *spi_sensor, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message->complete = focal_fp_sensor_complete;
	message->context = &done;

	spin_lock_irq(&spi_sensor->spi_lock);
	if (spi_sensor->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(spi_sensor->spi, message);
	spin_unlock_irq(&spi_sensor->spi_lock);
	//printk("alvin......focal_fp_sensor_sync,status=%d\n",status);
	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
		if (status == 0)
			status = message->actual_length;
	}
	
	return status;
}

static inline ssize_t
focal_fp_sensor_sync_write(struct focal_fp_sensor_data *spi_sensor, size_t len)
{
	struct spi_transfer	t = {
					.tx_buf	= spi_sensor->buffer,
					.len	= len,
				};
	struct spi_message m;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	
	return focal_fp_sensor_sync(spi_sensor, &m);
}

static inline ssize_t
focal_fp_sensor_sync_read(struct focal_fp_sensor_data *spi_sensor, size_t len)
{
	struct spi_transfer	t = {
			.tx_buf		= spi_sensor->buffer,
			.rx_buf		= spi_sensor->buffer,
			.len		= len,
		};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return focal_fp_sensor_sync(spi_sensor, &m);
}

static inline ssize_t 
focal_fp_sensor_rw(struct focal_fp_sensor_data *spi_sensor, u8 *buf, u16 buflen)
{
	int err = 0;	
	struct spi_message m;	
	struct spi_transfer t = {			
		.tx_buf	= buf,
		.rx_buf	= buf,
		.len		= buflen,		
	};	
	spi_message_init(&m);	
	spi_message_add_tail(&t, &m);	
	err = focal_fp_sensor_sync(spi_sensor, &m);	
	if( err < 0) {		
		FT_DBG("%s:write error----------\n", __func__);
	}		

	return err;
}

inline ssize_t getmcustatus(void)
{
        g_fpmcustatus = mt_get_gpio_in(eint_gpio | 0x80000000);

	printk("sss getmcustatus=%d\n",g_fpmcustatus);

	return g_fpmcustatus;
}

ssize_t 
focal_fp_sensor_fw_rw(struct focal_fp_sensor_data *spi_sensor, u8 *buf, u16 buflen)
{
	int ret = 0;
	struct spi_message m;	
	struct spi_transfer t = {			
		.tx_buf	= buf,
		.rx_buf	= buf,
		.len		= buflen,
		.delay_usecs = 5,
	};	
	spi_message_init(&m);	
	spi_message_add_tail(&t, &m);
	
	ret = focal_fp_sensor_sync(spi_sensor, &m);	
	if( ret < 0) {		
		printk("%s:write error---------err = %d\n", __func__,ret);
	}
	return ret;
}


/* Read-only message with current device setup */
static ssize_t
focal_fp_sensor_client_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct focal_fp_sensor_data	*spi_sensor;
	ssize_t	status = 0;
       int ret;

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	spi_sensor = filp->private_data;

	mutex_lock(&spi_sensor->buf_lock);
	ret = copy_from_user(spi_sensor->buffer, buf, count);
       if (ret != 0) {
            pr_err("focal_fp_sensor_client_read  copy_from_user error!\n");
       }
            
	status = focal_fp_sensor_sync_read(spi_sensor, count);
	if (status > 0) {
		unsigned long	missing;
	
		missing = copy_to_user(buf, spi_sensor->buffer, status);
		if (missing == status)
			status = -EFAULT;
		else
			status = status - missing;
	}
	mutex_unlock(&spi_sensor->buf_lock);
	if(status > 0)
	     status = 0;
	return status;
}

/* Write-only message with current device setup */
static ssize_t
focal_fp_sensor_client_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	ssize_t status = 0;
	unsigned long	missing;
	struct focal_fp_sensor_data	*spi_sensor;

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	spi_sensor = filp->private_data;

	mutex_lock(&spi_sensor->buf_lock);
	missing = copy_from_user(spi_sensor->buffer, buf, count);
	if (missing == 0) {
		status = focal_fp_sensor_sync_write(spi_sensor, count);
	} else
		status = -EFAULT;
	mutex_unlock(&spi_sensor->buf_lock);

	return status;
}

static int focal_fp_sensor_message(struct focal_fp_sensor_data *spi_sensor,
		struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
	struct spi_message msg;
	struct spi_transfer	*k_xfers;
	struct spi_transfer	*k_tmp;
	struct spi_ioc_transfer *u_tmp;
	unsigned		n, total;
	u8			*buf;
	int			status = -EFAULT;

	spi_message_init(&msg);
	k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
	if (k_xfers == NULL)
		return -ENOMEM;
	//FT_DBG("%s %d \n",__func__,__LINE__);
	/* Construct spi_message, copying any tx data to bounce buffer.
	 * We walk the array of user-provided transfers, using each one
	 * to initialize a kernel version of the same transfer.
	 */
	buf = spi_sensor->buffer;
	total = 0;
	for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
			n;
			n--, k_tmp++, u_tmp++) {
		k_tmp->len = u_tmp->len;

		total += k_tmp->len;
		if (total > bufsiz) {
			status = -EMSGSIZE;
			goto done;
		}

		if (u_tmp->rx_buf) {
			k_tmp->rx_buf = buf;
			if (!access_ok(VERIFY_WRITE, (u8 __user *)
						(uintptr_t) u_tmp->rx_buf,
						u_tmp->len))
				goto done;
		}
		if (u_tmp->tx_buf) {
			k_tmp->tx_buf = buf;
			if (copy_from_user(buf, (const u8 __user *)
						(uintptr_t) u_tmp->tx_buf,
					u_tmp->len))
				goto done;
		}
		buf += k_tmp->len;
		k_tmp->cs_change = !!u_tmp->cs_change;
		k_tmp->bits_per_word = u_tmp->bits_per_word;
		k_tmp->delay_usecs = u_tmp->delay_usecs;
		k_tmp->speed_hz = u_tmp->speed_hz;
#ifdef VERBOSE
		dev_dbg(&spi_sensor->spi->dev,
			"  xfer len %zd %s%s%s%dbits %u usec %uHz\n",
			u_tmp->len,
			u_tmp->rx_buf ? "rx " : "",
			u_tmp->tx_buf ? "tx " : "",
			u_tmp->cs_change ? "cs " : "",
			u_tmp->bits_per_word ? : spidev->spi->bits_per_word,
			u_tmp->delay_usecs,
			u_tmp->speed_hz ? : spidev->spi->max_speed_hz);
#endif
		spi_message_add_tail(k_tmp, &msg);
	}
	status = focal_fp_sensor_sync(spi_sensor, &msg);
	if (status < 0)
		goto done;

	/* copy any rx data out of bounce buffer */
	buf = spi_sensor->buffer;
	for (n = n_xfers, u_tmp = u_xfers; n; n--, u_tmp++) {
		if (u_tmp->rx_buf) {
			if (__copy_to_user((u8 __user *)
					(uintptr_t) u_tmp->rx_buf, buf,
					u_tmp->len)) {
				status = -EFAULT;

				goto done;
			}
		}
		buf += u_tmp->len;
	}
	status = total;
done:
	kfree(k_xfers);
	return status;
}

void focal_fp_sensor_reset(struct focal_fp_sensor_data *spi_sensor)
{
    pinctrl_select_state(pinctrl_fp, pins_reset_low);
    msleep(2);
    pinctrl_select_state(pinctrl_fp, pins_reset_high);
}

void focal_fp_sensor_power_on(struct focal_fp_sensor_data *spi_sensor)
{
    pinctrl_select_state(pinctrl_fp, pins_power_on);
}

void focal_fp_sensor_power_down(struct focal_fp_sensor_data *spi_sensor)
{
    pinctrl_select_state(pinctrl_fp, pins_power_off);
}

static int focal_fp_lightscreen(void)
{
    input_report_key(g_fp_spi_sensor->input, KEY_POWER, 1);
    input_sync(g_fp_spi_sensor->input);
    msleep(10);
    input_report_key(g_fp_spi_sensor->input, KEY_POWER, 0);
    input_sync(g_fp_spi_sensor->input);

    return 0;
}

static int focal_fp_wakeupSys(void)
{
    input_report_key(g_fp_spi_sensor->input, KEY_WAKEUP, 1);
    input_sync(g_fp_spi_sensor->input);
    msleep(10);
    input_report_key(g_fp_spi_sensor->input, KEY_WAKEUP, 0);
    input_sync(g_fp_spi_sensor->input);

    return 0;
}

static int focal_fp_firmware_download(struct focal_fp_sensor_data *spi_sensor)
{
    int ret;
	focal_fp_sensor_reset(spi_sensor);
    ret = focal_fp_fw_download(spi_sensor);
    if(ret < 0)
		printk("focal_fp_fw_download fail!\n");
	
    return ret;
}
/*[BUGFIX]-Mod-BEGIN by ke.huang.sz,3651144*/
/*define spiclk enable&&disable func*/
static void ftsfp_spiclk_enable(struct focal_fp_sensor_data *spi_sensor)
{
 	if ( spi_sensor->spiclk_enabled)
	{		
		printk("%s: spi clk has aready enabled!\n",__func__);
	}
	else
	{
#ifdef CONFIG_MTK_CLKMGR
		enable_clock(MT_CG_PERI_SPI0, "spi");
#else
		secspi_enable_clk(g_fp_spi_sensor->spi);
#endif
 		spi_sensor->spiclk_enabled = 1;
	}
 }

static void ftsfp_spiclk_disable(struct focal_fp_sensor_data *spi_sensor)
{
	if ( spi_sensor->spiclk_enabled == 0 )
	{		
		printk("%s: spi clk has aready disabled\n",__func__);
	}
	else
	{
#ifdef CONFIG_MTK_CLKMGR
		disable_clock(MT_CG_PERI_SPI0, "spi");
#else
		secspi_disable_clk(g_fp_spi_sensor->spi);
#endif
		spi_sensor->spiclk_enabled = 0;
	}
 }
/*[BUGFIX]-Mod-END by ke.huang.sz*/

static void ftsfp_ca_lock_hold(void)
{
	if (wake_lock_active(&fp_ca_lock) != 0)
	{
        wake_unlock(&fp_ca_lock);
	}
	wake_lock_timeout(&fp_ca_lock, 3 * HZ); /* it will auto unlock in 60min */	
    printk("fp_ca_lock locked\n");
}

static void ftsfp_ca_lock_release(void)
{
	if (wake_lock_active(&fp_ca_lock) == 0)
	{
        wake_unlock(&fp_ca_lock);
	}	
    printk("fp_ca_lock unlocked\n");
}

static long focal_fp_sensor_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int	err = 0;
	int	retval = 0;
	unsigned long stru_address = 0;
	u32	tmp;
	unsigned n_ioc;
	struct spi_ioc_transfer	*ioc;
	struct focal_fp_sensor_data	*spi_sensor;
	struct spi_device	*spi;

	if (_IOC_TYPE(cmd) != FOCAL_FP_IOC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	spi_sensor = filp->private_data;
	spin_lock_irq(&spi_sensor->spi_lock);
	spi = spi_dev_get(spi_sensor->spi);
	spin_unlock_irq(&spi_sensor->spi_lock);

	if (spi == NULL)
		return -ESHUTDOWN;

	/* use the buffer lock here for double duty:
	 *  - prevent I/O (from us) so calling spi_setup() is safe;
	 *  - prevent concurrent SPI_IOC_WR_* from morphing
	 *    data fields while SPI_IOC_RD_* reads them;
	 */
	mutex_lock(&spi_sensor->buf_lock);

	switch (cmd) {
	/* read requests */
	case SPI_IOC_RD_MODE:
		retval = __put_user(spi->mode & SPI_MODE_MASK,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_LSB_FIRST:
		retval = __put_user((spi->mode & SPI_LSB_FIRST) ?  1 : 0,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_BITS_PER_WORD:
		retval = __put_user(spi->bits_per_word, (__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MAX_SPEED_HZ:
		retval = __put_user(spi->max_speed_hz, (__u32 __user *)arg);
		break;
	case FOCAL_FP_IOC_GET_SENSOR_STATUS:
		retval = __put_user(g_enableSensor, (__u8 __user *)arg);
		break;
	case FOCAL_FP_IOC_IS_FINGER_ON:
		FT_DBG("g_fpfingeron = %d \n ", g_fpfingeron);
		retval = __put_user(g_fpfingeron, (__u8 __user *)arg);
		break;
       case FOCAL_FP_GET_MCU_STATUS:
             //g_fpmcustatus = gpio_get_value(eint_gpio);
	     g_fpmcustatus = mt_get_gpio_in(eint_gpio | 0x80000000);
             printk("mcu status is %d\n", g_fpmcustatus);
             retval = __put_user(g_fpmcustatus, (__u8 __user *)arg);
             break;
       case FOCAL_FP_SET_IRQ_MASK:
             retval = __get_user(tmp, (u8 __user *)arg);
             if (FOCAL_ENABLE_INTERRUPT_WAKE == tmp) {
                    //mt_eint_unmask(CUST_EINT_FP1_NUM);
                    enable_irq(g_fp_spi_sensor->irq);
             } else if (FOCAL_DISABLE_INTERRUPT_WAKE == tmp) {
                    //mt_eint_mask(CUST_EINT_FP1_NUM);//yanj
                    disable_irq(g_fp_spi_sensor->irq);
             }
             break;
	case FOCAL_FP_WRITE_READ_DATA:
		FT_DBG("FOCAL_FP_READ_DATA_______\n");
		if (__copy_from_user(&g_fp_rw_stru, (void __user *)arg, 
			sizeof(struct focal_fp_rw_operate))) {
			retval = -EFAULT;
		}
        
		if (__copy_from_user(g_rw_buf, (void __user *)g_fp_rw_stru.buf, 
			 g_fp_rw_stru.len)) {
			retval = -EFAULT;
		}
		stru_address = (unsigned long)(&g_fp_rw_stru.buf[0]);
	//	FT_DBG("struct sizeof = %lu\n", sizeof(struct focal_fp_rw_operate));
		FT_DBG("____len = %x\n", g_fp_rw_stru.len);
		FT_DBG("____flag = %x\n", g_fp_rw_stru.flag);
		FT_DBG("_____data address = %lu\n", stru_address);

		
		if (g_fp_rw_stru.len > MAX_SPI_RW_LEN) {
			retval = -EFAULT;
			break;
		}
			
		retval = focal_fp_sensor_rw(spi_sensor, g_rw_buf, g_fp_rw_stru.len);
		if(retval < 0) {
			retval = -EFAULT;
			break;
		}
		FT_DBG("____buf[3] = %x\n", g_rw_buf[3]);
		retval = __copy_to_user((__u8 __user *)g_fp_rw_stru.buf, g_rw_buf, 
			g_fp_rw_stru.len);
		if(retval < 0) {
			FT_DBG("FOCAL_FP_READ_DATA: error copying to user\n");
		}

		break;
	/* write requests */
	case SPI_IOC_WR_MODE:
		retval = __get_user(tmp, (u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->mode;

			if (tmp & ~SPI_MODE_MASK) {
				retval = -EINVAL;
				break;
			}

			tmp |= spi->mode & ~SPI_MODE_MASK;
			spi->mode = (u8)tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "spi mode %02x\n", tmp);
		}
		break;
	case SPI_IOC_WR_LSB_FIRST:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->mode;

			if (tmp)
				spi->mode |= SPI_LSB_FIRST;
			else
				spi->mode &= ~SPI_LSB_FIRST;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "%csb first\n",
						tmp ? 'l' : 'm');
		}
		break;
	case SPI_IOC_WR_BITS_PER_WORD:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->bits_per_word;

			spi->bits_per_word = tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->bits_per_word = save;
			else
				dev_dbg(&spi->dev, "%d bits per word\n", tmp);
		}
		break;
	case SPI_IOC_WR_MAX_SPEED_HZ:
		retval = __get_user(tmp, (__u32 __user *)arg);
		if (retval == 0) {
			u32	save = spi->max_speed_hz;

			spi->max_speed_hz = tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->max_speed_hz = save;
			else
				dev_dbg(&spi->dev, "%d Hz (max)\n", tmp);
		}
		break;
	case FOCAL_FP_IOC_RST_SENSOR:
		FT_DBG("_____FOCAL_FP_IOC_RST_SENSOR____\n");
		focal_fp_sensor_reset(spi_sensor);
		break;
	case FOCAL_FP_IOC_RST_DOWNLOAD_SENSOR:
		focal_fp_firmware_download(spi_sensor);
		break;
	case FOCAL_FP_IOC_SET_SENSOR_STATUS:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (0 == retval) {
			retval = focal_fp_enable_sensor(spi_sensor, tmp);
		}
		break;
	case FOCAL_FP_SET_INTERRUPT_WAKE_STATUS:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (0 == retval) {
			g_fpinterruptwake = tmp;
		}
		break;
	case FOCAL_FP_LIGHTSCREEN:
		focal_fp_lightscreen();
		break;
	case FOCAL_FP_WAKEUP_SYSTEM:
		focal_fp_wakeupSys();
		break;
	case FOCAL_FP_SET_KEY_MASK:
             retval = __get_user(tmp, (u8 __user *)arg);
             if (1 == tmp) {
                printk("----------------------zax 1 focal key up interrupt-----------\n");
		#if 0           //modify by ke.huang
                input_report_key(g_fp_spi_sensor->input, KEY_UP, 1);
                input_sync(g_fp_spi_sensor->input);
                msleep(2);
                input_report_key(g_fp_spi_sensor->input, KEY_UP, 0);
                input_sync(g_fp_spi_sensor->input);
		#endif
		input_report_key(g_fp_spi_sensor->input, GF_INPUT_HOME_KEY, 0);
        input_sync(g_fp_spi_sensor->input);
            }
            else if (tmp == 0)
            {
                printk("----------------------zax 2 focal key down interrupt-----------\n");
		#if 0       //modify by ke.huang
                input_report_key(g_fp_spi_sensor->input, KEY_DOWN, 1);
                input_sync(g_fp_spi_sensor->input);
                msleep(2);
                input_report_key(g_fp_spi_sensor->input, KEY_DOWN, 0);
                input_sync(g_fp_spi_sensor->input);
		#endif
		input_report_key(g_fp_spi_sensor->input, GF_INPUT_HOME_KEY, 1);
		input_sync(g_fp_spi_sensor->input);
            }
	    break;
		case FOCAL_FP_SPI_CLK_ONOFF:   //add interface for ioctl ,modify by ke.huang 20161202
			retval = __get_user(tmp, (u8 __user *)arg);
			if (1 == tmp) 
			{
				ftsfp_spiclk_enable(g_fp_spi_sensor);
			} 
			else if (0 == tmp) 
			{
				ftsfp_spiclk_disable(g_fp_spi_sensor);
			}
			break;
		case FOCAL_FP_CA_LOCK_ONOFF:   //add interface for ioctl ,modify by focal 2017.2.23
			retval = __get_user(tmp, (u8 __user *)arg);
			if (1 == tmp) 
			{
				ftsfp_ca_lock_hold();
			} 
			else if (0 == tmp) 
			{
				ftsfp_ca_lock_release();
			}
			break;
		
	default:
		/* segmented and/or full-duplex I/O request */
		if (_IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))
				|| _IOC_DIR(cmd) != _IOC_WRITE) {
			retval = -ENOTTY;
			break;
		}
		tmp = _IOC_SIZE(cmd);
		if ((tmp % sizeof(struct spi_ioc_transfer)) != 0) {
			retval = -EINVAL;
			break;
		}
		n_ioc = tmp / sizeof(struct spi_ioc_transfer);
		if (n_ioc == 0)
			break;
		/* copy into scratch area */
		ioc = kmalloc(tmp, GFP_KERNEL);
		if (!ioc) {
			retval = -ENOMEM;
			break;
		}

		if (__copy_from_user(ioc, (void __user *)arg, tmp)) {
			kfree(ioc);
			retval = -EFAULT;
			break;
		}

		/* translate to spi_message, execute */
		retval = focal_fp_sensor_message(spi_sensor, ioc, n_ioc);
		kfree(ioc);
		break;
	}

	mutex_unlock(&spi_sensor->buf_lock);
	spi_dev_put(spi);
	
	return retval;
}

#ifdef CONFIG_COMPAT
static long
focal_fp_sensor_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return focal_fp_sensor_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define focal_fp_sensor_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int focal_fp_sensor_open(struct inode *inode, struct file *filp)
{
	
	int	status = -ENXIO;
	struct focal_fp_sensor_data	*spi_sensor;

      FT_DBG("__%s %d__\n", __func__, __LINE__);

	mutex_lock(&device_list_lock);
	list_for_each_entry(spi_sensor, &device_list, device_entry) {
		if (spi_sensor->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}
	if (spi_sensor->users > 0) {
		mutex_unlock(&device_list_lock);
		FT_DBG(KERN_ERR "Spi sensor: %s: Too many users\n", __func__);
		return -EPERM;
	}
	if (status == 0) {
		if (!spi_sensor->buffer) {
			spi_sensor->buffer = kmalloc(bufsiz, GFP_KERNEL);
			if (!spi_sensor->buffer) {
				dev_dbg(&spi_sensor->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			}
		}
		if (status == 0) {
			spi_sensor->users++;
			filp->private_data = spi_sensor;
			nonseekable_open(inode, filp);
		}
	} else
		FT_DBG("spi_sensor: nothing for minor %d\n", iminor(inode));
	mutex_unlock(&device_list_lock);
	
	return status;
}


/*
1-enable--leave sleep mode and enter standby mode
0-disable--enter sleep mode for power consumption
*/
static int focal_fp_enable_sensor(struct focal_fp_sensor_data *spi_sensor, u8 enable)
{
	
	if (FOCAL_DISABLE_FP_SENSOR == enable) {
		
	} else {
	
	}

	g_enableSensor = enable;
    
	return 0;
}

static void fp_sensor_generate_event(int event)
{
	char *envp[2];

       if (FOCAL_FP_INTERRUPT == event) {
	    envp[0] = "FOCAL=fingeron";
	    envp[1] = NULL;
       } else if (FOCAL_FP_RESUME == event) {
           envp[0] = "FOCAL=resume";
	    envp[1] = NULL;
       } else if (FOCAL_FP_SUSPEND == event) {
           envp[0] = "FOCAL=suspend";
	    envp[1] = NULL;     
       }

	kobject_uevent_env(&g_fp_spi_sensor->spi->dev.kobj, KOBJ_CHANGE, envp);	
}

static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int focal_fp_flag = 0;

static irqreturn_t fp_interrupt_handler(int irq, void* handle) {
    printk("__%s %d__\n", __func__, __LINE__);
 //   printk("focal_intrupt!\n");
    //Modified by ke.huang for Task 4256127 on 2017/02/20 begin
	if (wake_lock_active(&fp_irq_lock) != 0)
	{
        wake_unlock(&fp_irq_lock);
        printk("wake unlock \n");
	}
	wake_lock_timeout(&fp_irq_lock, 3 * HZ); /* it will auto unlock in 60min */
    //Modified by ke.huang for Task 4256127 on 2017/02/20 end
    //disable_irq_nosync(g_fp_spi_sensor->irq);	//lizhi
    focal_fp_flag = 1; 
    wake_up_interruptible(&waiter);
    return IRQ_HANDLED;
}

static int focal_fp_event_handler(void *spi_sensor)
{
    //struct focal_fp_sensor_data	*m_spi_sensor = spi_sensor;
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
    FT_DBG("focal enter focal_fp_event_handler\n");
    sched_setscheduler(current, SCHED_RR, &param);
    do
    {
        set_current_state(TASK_INTERRUPTIBLE);
        wait_event_interruptible(waiter, focal_fp_flag != 0);
        focal_fp_flag = 0;
        set_current_state(TASK_RUNNING);
        FT_DBG("----------------------focal sleep interrupt-----------\n");
	fp_sensor_generate_event(FOCAL_FP_INTERRUPT);
    }
    while (!kthread_should_stop());

    return 0;
}


/******************fcoal fingerprint sysfs debug********************/
#ifdef FOCAL_FP_SYSFS_DBG
static struct mutex g_focal_fp_device_mutex;

static ssize_t focalfp_enablesensor_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	/*place holder for future use*/
	return -EPERM;
}

static ssize_t focalfp_enablesensor_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
    return 0;
}

static ssize_t focalfp_captureimage_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t num_read_chars = 0;
	//struct spi_device *spi = container_of(dev, struct spi_device, dev);
	//struct focal_fp_sensor_data *data = spi_get_drvdata(spi);
	
	mutex_lock(&g_focal_fp_device_mutex);
	g_captureimage_mode = 1;
	
	g_captureimage_mode = 0;
	mutex_unlock(&g_focal_fp_device_mutex);
	return num_read_chars;
}

static ssize_t focalfp_captureimage_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	/*place holder for future use*/
	return -EPERM;
}

static ssize_t focalfp_initcfg_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t num_read_chars = 0;
	//struct spi_device *spi = container_of(dev, struct spi_device, dev);
	//struct focal_fp_sensor_data *data = spi_get_drvdata(spi);
	
	mutex_lock(&g_focal_fp_device_mutex);
	
	focal_fp_firmware_download(g_fp_spi_sensor);
	mutex_unlock(&g_focal_fp_device_mutex);

	return num_read_chars;
}

static ssize_t focalfp_initcfg_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	/*place holder for future use*/
	return -EPERM;
}

static DEVICE_ATTR(focalfpenablesensor, S_IRUGO | S_IWUSR,
			focalfp_enablesensor_show,
			focalfp_enablesensor_store);

static DEVICE_ATTR(focalfpcaptureimage, S_IRUGO | S_IWUSR,
			focalfp_captureimage_show,
			focalfp_captureimage_store);

static DEVICE_ATTR(focalfpinitcfg, S_IRUGO | S_IWUSR,
			focalfp_initcfg_show,
			focalfp_initcfg_store);

/*add your attr in here*/
static struct attribute *focal_fp_attributes[] = {
	&dev_attr_focalfpenablesensor.attr,
	&dev_attr_focalfpcaptureimage.attr,
	&dev_attr_focalfpinitcfg.attr,
	NULL
};

static struct attribute_group focal_fp_attribute_group = {
	.attrs = focal_fp_attributes,
};

/*create sysfs for debug*/
static int focal_fp_create_sysfs(struct spi_device *spi)
{
	int err = 0;
	err = sysfs_create_group(&spi->dev.kobj, &focal_fp_attribute_group);
	if (0 != err) {
		dev_err(&spi->dev,
					 "%s() - ERROR: sysfs_create_group() failed.\n",
					 __func__);
		sysfs_remove_group(&spi->dev.kobj, &focal_fp_attribute_group);
		return -EIO;
	} else {
		mutex_init(&g_focal_fp_device_mutex);
		FT_DBG("focal_fp:%s() - sysfs_create_group() succeeded.\n",
				__func__);
	}
	
	return err;
}

static void focal_fp_release_sysfs(struct spi_device *spi)
{
	sysfs_remove_group(&spi->dev.kobj, &focal_fp_attribute_group);
	mutex_destroy(&g_focal_fp_device_mutex);
}

#endif
/**************************************************************/

static int focal_fp_sensor_release(struct inode *inode, struct file *filp)
{
	int	status = 0;
	struct focal_fp_sensor_data	*spi_sensor;
	
	FT_DBG("__%s %d__\n", __func__, __LINE__);
	mutex_lock(&device_list_lock);
	spi_sensor = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	spi_sensor->users--;
	if (!spi_sensor->users) {
		int	dofree;
		kfree(spi_sensor->buffer);
		spi_sensor->buffer = NULL;
		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&spi_sensor->spi_lock);
		dofree = (spi_sensor->spi == NULL);
		spin_unlock_irq(&spi_sensor->spi_lock);
		if (dofree)
			kfree(spi_sensor);
	}
	mutex_unlock(&device_list_lock);

	return status;
}

static struct class *focal_spi_class;

static const struct file_operations focal_fp_sensor_fops = {
    .owner =	THIS_MODULE,
    .write = focal_fp_sensor_client_write,
    .read = focal_fp_sensor_client_read,
    .unlocked_ioctl = focal_fp_sensor_ioctl,
    .compat_ioctl = focal_fp_sensor_compat_ioctl,
    .open = focal_fp_sensor_open,
    .release = focal_fp_sensor_release,
    .llseek =	no_llseek,
};

/*-------------------------------------------------------------------------*/
/*
static struct miscdevice focal_misc_dev =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = FOCAL_FP_DEVICE_NAME,
    .fops = &focal_fp_sensor_fops,
};
*/
#ifdef CONFIG_HAS_EARLYSUSPEND
static void focal_fp_suspend(struct early_suspend *h)
{
    fp_sensor_generate_event(FOCAL_FP_SUSPEND);
    msleep(60);
}

static void focal_fp_resume(struct early_suspend *h)
{
    fp_sensor_generate_event(FOCAL_FP_RESUME);
}
#endif

static int focal_get_irq_info(void)
{
    struct device_node *node;
    int irq = 0;
    int ints[2] = {0, 0};
    
    node = of_find_compatible_node(NULL, NULL,"mediatek,fpc1145");
    if(node) {
        if (of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints)))
	{
		pr_err("get dtsi debounce array fail\n");
	}
	else
	{
        	gpio_set_debounce(ints[0], ints[1]);		//lizhi
		eint_gpio = ints[0];
		printk("focal  eint_gpio number is %d %d\n", eint_gpio,ints[1]);
	}
        irq = irq_of_parse_and_map(node, 0);
        if(!irq) {
    	    printk("can't map FP irq");
        }
        g_fp_spi_sensor->irq = irq;
        printk("focal %s irq num = %d(%d)\n", __func__, irq, ints[0]);
	irq = request_irq(g_fp_spi_sensor->irq,fp_interrupt_handler,IRQF_TRIGGER_RISING,"focal_fp",NULL);
        if(irq)
        {
    	    printk("focal request_irq fail");
            return -1;
        }   
	disable_irq(g_fp_spi_sensor->irq); 	
    }
    
    return 0;
  }

static int focal_get_gpio_info(void)
{
#ifdef CONFIG_OF
    int ret;
    struct device_node	*fp_node = NULL; 
    struct platform_device *pdev = NULL;

    fp_node = of_find_compatible_node(NULL, NULL, DTS_INFO_MATCH_STR);//Linc 20160715
    if (fp_node == NULL) {
        pr_err("fingerprint - get fp node failed\n");
    } else {
#if 0
        if (of_property_read_u32_index(fp_node, "irq_gpio", 0, &eint_gpio)) {
            pr_err("get dtsi eint_gpio fail\n");
        }
        printk("focal  eint_gpio number is %d\n", eint_gpio);
#endif
        pdev = of_find_device_by_node(fp_node);
        if (pdev) {
            pinctrl_fp = devm_pinctrl_get(&pdev->dev);
            if (IS_ERR(pinctrl_fp)) {
                ret = PTR_ERR(pinctrl_fp);
                dev_err(&pdev->dev, "Cannot find fp pinctrl!\n");
                return ret;
            }
        } else {
            pr_err("%s platform device is null\n", __func__);
            return -1;
        }
    }

/* it's normal that get "default" will failed */
	ftsFP_pins_default = pinctrl_lookup_state(pinctrl_fp, "default");
	if (IS_ERR(ftsFP_pins_default)) {
		ret = PTR_ERR(ftsFP_pins_default);
		pr_err( "%s can't find fingerprint pinctrl default\n", __func__);
		/* return ret; */
	}
	#if 0
	pins_miso_spi = pinctrl_lookup_state(pinctrl_fp, "miso_spi");
	if (IS_ERR(pins_miso_spi)) {
		ret = PTR_ERR(pins_miso_spi);
		pr_err( "%s can't find fingerprint pinctrl miso_spi\n", __func__);
		return ret;
	}
	pins_miso_pullhigh = pinctrl_lookup_state(pinctrl_fp, "miso_pullhigh");
	if (IS_ERR(pins_miso_pullhigh)) {
		ret = PTR_ERR(pins_miso_pullhigh);
		pr_err( "%s can't find fingerprint pinctrl miso_pullhigh\n", __func__);
		return ret;
	}
	pins_miso_pulllow= pinctrl_lookup_state(pinctrl_fp, "miso_pulllow");
	if (IS_ERR(pins_miso_pulllow)) {
		ret = PTR_ERR(pins_miso_pulllow);
		pr_err( "%s can't find fingerprint pinctrl miso_pulllow\n", __func__);
		return ret;
	}
	#endif
	pins_reset_high = pinctrl_lookup_state(pinctrl_fp, "reset_high");
	if (IS_ERR(pins_reset_high)) {
		ret = PTR_ERR(pins_reset_high);
		pr_err( "%s can't find fingerprint pinctrl reset_high\n", __func__);
		return ret;
	}
	pins_reset_low = pinctrl_lookup_state(pinctrl_fp, "reset_low");
	if (IS_ERR(pins_reset_low)) {
		ret = PTR_ERR(pins_reset_low);
		pr_err( "%s can't find fingerprint pinctrl reset_low\n", __func__);
		return ret;
	}
	pins_power_on = pinctrl_lookup_state(pinctrl_fp, "power_on");
	if (IS_ERR(pins_power_on)) {
		ret = PTR_ERR(pins_power_on);
		pr_err( "%s can't find fingerprint pinctrl power_on\n", __func__);
		return ret;
	}
	pins_power_off = pinctrl_lookup_state(pinctrl_fp, "power_off");
	if (IS_ERR(pins_power_off)) {
		ret = PTR_ERR(pins_power_off);
		pr_err( "%s can't find fingerprint pinctrl power_off\n", __func__);
		return ret;
	}
	pins_irq_gpio = pinctrl_lookup_state(pinctrl_fp, "irq_gpio");
	if (IS_ERR(pins_irq_gpio)) {
		ret = PTR_ERR(pins_irq_gpio);
		pr_err( "%s can't find fingerprint pinctrl irq_gpio\n", __func__);
		return ret;
	}
	printk("%s, get pinctrl success!\n", __func__);
#endif    
    return 0;    
}

static int focal_get_fw_id(void)
{
	u8 buf[10];
	u32 i = 0;

	buf[0] = 0x10;
	buf[1] = 0xef;
	buf[2] = 0x1a;
	buf[3] = 0x00;
	buf[4] = 0x00;
	buf[5] = 0x00;
	focal_fp_sensor_fw_rw(g_fp_spi_sensor, buf, 6);

	for (i = 0; i < 6; i++)
	{
		printk("buf[%d]:0x%x;\n", i, buf[i]);
	}

	return 0;
}

static inline ssize_t
focal_fp_sensor_write_sfr_register(struct focal_fp_sensor_data *spi_sensor,
	u8 reg_addr, u8 reg_value)
{
	u8 buff[4] = {0x00};
	struct spi_message m;
	struct spi_transfer t = {
		.tx_buf 	= buff,
		.len		= 4,
		};
	buff[0] = 0x09;//CMD_WRITE_SFR_SINGLE_1;
        buff[1] = 0xf6;//CMD_WRITE_SFR_SINGLE_2;
        buff[2] = reg_addr;
        buff[3] = reg_value;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	pr_info("func=%s[register:0x%02x].[value:0x%02x]\n", __func__, reg_addr, reg_value);
	return focal_fp_sensor_sync(spi_sensor, &m);
}
#if 0
static int focal_fp_sensor_version(struct focal_fp_sensor_data *spi_sensor)
{
	
	//int i=0,n=0,g_fpicversion=0;
	int i = 0;
	//unsigned short read_len = 8;
	//unsigned short recv_len = 8;
	int	retval = 0;
	//0229 u16 startAddr=0x1b00;
	unsigned char status[15];

	
	int bulkreadlen = 0;
	//0229 int bulkloop = read_len / (MAX_SPI_RW_LEN - 6) + 1;
	//mutex_lock(&g_focal_fp_device_mutex);
	//for (n = 0; n < 1; n++) {
		bulkreadlen = 0;
		//0229 if (recv_len >= (MAX_SPI_RW_LEN - 6))
		//0229	bulkreadlen = MAX_SPI_RW_LEN - 6;
		//0229 else
			bulkreadlen = 8;//0229 recv_len % (MAX_SPI_RW_LEN - 6);

		//memset(status, 0, sizeof(status));
		status[0] = 0x04;
		status[1] = 0xfb;
		status[2] = 0x98;//0229 startAddr >> 8;
		status[3] = 0x80;//0229 startAddr & 0xff;
		status[4] = 0;//0229 (bulkreadlen - 2) >> 8;
		status[5] = 6;//0229 (bulkreadlen - 2) & 0xff;

		/*0229g_fp_rw_stru1.flag = 1;
		g_fp_rw_stru1.len = bulkreadlen + 6;//MAX_SPI_RW_LEN;
		g_rw_buf = status;


		
		if (g_fp_rw_stru1.len > MAX_SPI_RW_LEN) 
		{
			retval = -EFAULT;
			//break;
		}
		*/
		retval = focal_fp_sensor_rw(spi_sensor, status, 14);//8+6
		
		if(retval < 0) 
		{
			retval = -EFAULT;
			//break;
		}
		
			
		//startAddr += (MAX_SPI_RW_LEN - 6) / 2;
		//recv_len -= (MAX_SPI_RW_LEN - 6);
		
		//memcpy(read + (MAX_SPI_RW_LEN - 6) * n, (g_rw_buf + 6), bulkreadlen);

		msleep(100);
	//}
	for (i = 0; i < 8; i++)
	{
		 printk("\n zax zax value=%x, \n", status[5+i]);
	}
	if( (status[6]==0x01)&& (status[7]==0xf4)&&(status[8]==0x03) && (status[9]==0xe8))
	{
		return 0;
	}
	else
		return -1;
		
	//	g_fpicversion=11;
	//else
	//	g_fpicversion=0;
	//for(i=0;i<8;i++)
	//{
		
		

		//	printk("\n zax zax value=%x, \n",status[5+i]);//,reg_val_i2c[0]);
			
			
			
		
		
	//}
	//mutex_unlock(&g_focal_fp_device_mutex);
}
#endif
/*-------------------------------------------------------------------------*/
struct task_struct *thread_fp = NULL;
static int focal_fp_sensor_probe(struct spi_device *spi)
{
	int status = 0, ret = 0, count = 0;
	struct focal_fp_sensor_data	*spi_sensor;
	struct input_dev *input_dev = NULL;
	//u8 buf[32]= {0};
	printk("alvin........focal_fp_sensor_probe start\n");
	spi_sensor = kzalloc(sizeof(*spi_sensor), GFP_KERNEL);
	if (!spi_sensor){
			printk("spi_sensor kzalloc request error!\n");
			status = -ENOMEM;
			goto probe_fail;
	}else{
			printk("spi_sensor kzalloc request success!\n");
	}
	g_rw_buf = kzalloc(MAX_SPI_RW_LEN, GFP_KERNEL);
	if (!g_rw_buf){
			printk("g_rw_buf kzalloc request error!\n");
			status = -ENOMEM;
                        goto probe_fail;
	}else{
			printk("g_rw_buf kzalloc request success!\n");
	}
	
	g_fp_spi_sensor = spi_sensor;

	input_dev = input_allocate_device();
	if (!input_dev) {
		printk("Failed to allocate memory\n");
		status = -ENOMEM;
		goto err_free_mem2;
	}
	printk("alvin===========input register device start!\n");
 
	input_dev->name = "focal_fp_btn";
	spi_sensor->input = input_dev;

	__set_bit(EV_KEY, input_dev->evbit);

	input_set_capability(input_dev, EV_KEY, KEY_POWER);
	input_set_capability(input_dev, EV_KEY, KEY_WAKEUP);
	#if 0       //modify by ke.huang
	input_set_capability(input_dev, EV_KEY, KEY_UP);
	input_set_capability(input_dev, EV_KEY, KEY_DOWN);
	#endif
	input_set_capability(input_dev, EV_KEY, GF_INPUT_HOME_KEY);
	
	status = input_register_device(input_dev);
	if (status) {
		printk("%s: failed to register input device: %s\n",
			__func__, dev_name(&spi->dev));
		goto exit_input_register_device_failed;
	}
	printk("input register device success!\n");

    spi->controller_data = &focal_chip_config;
    /* setup SPI parameters */
    /* CPOL=CPHA=0, speed 1MHz */
    spi->mode = SPI_MODE_0;

    spi->bits_per_word = 8;
    spi_sensor->spi = spi;
    spin_lock_init(&spi_sensor->spi_lock);
    mutex_init(&spi_sensor->buf_lock);
	wake_lock_init(&fp_irq_lock, WAKE_LOCK_SUSPEND, "ftsfp  irq  wakelock");
	wake_lock_init(&fp_ca_lock,WAKE_LOCK_SUSPEND, "ftsfp ca wakelock");

    focal_get_irq_info();
    focal_get_gpio_info();
	
 
    thread_fp = kthread_run(focal_fp_event_handler, 0, "focal_fp");

    //mt_eint_registration(CUST_EINT_FP_NUM, EINTF_TRIGGER_RISING, fp_interrupt_handler, 1);
    //mt_eint_unmask(CUST_EINT_FP_NUM);
	//FT_DBG("=========focal interrupt setting ok=============\n");
 
    focal_fp_sensor_power_on(spi_sensor);
	msleep(10);

	
    init_flag = 1;
    while (count < DOWNLOAD_COUNT) {
	focal_fp_sensor_reset(spi_sensor);
	printk("alvin........firmware start download\n");
    ret = focal_fp_fw_download(spi_sensor);
    if (ret==0)
		{
			printk("alvin........firmware download %d success\n",count); 
			break;
		} 
	printk("alvin........firmware download %d fail!\n",count);
    count++;
    }
    if ( count >= 3)
    {
		init_flag = 0;
		status = -1;
		goto err_firmware_download;
    }
    printk("focal download count = %d\n", count);

#ifdef CONFIG_HAS_EARLYSUSPEND
    spi_sensor->focal_early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2;
    spi_sensor->focal_early_suspend.suspend = focal_fp_suspend;
    spi_sensor->focal_early_suspend.resume = focal_fp_resume;
    register_early_suspend(&spi_sensor->focal_early_suspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */
    
    INIT_LIST_HEAD(&spi_sensor->device_entry);
	
    mutex_lock(&device_list_lock);
	
    spi_sensor->devt = MKDEV(SPIDEV_MAJOR, 0);
    list_add(&spi_sensor->device_entry, &device_list);
	
    mutex_unlock(&device_list_lock);
    spi_set_drvdata(spi, spi_sensor);
#if 0
//add for detect begin
    printk("=========focal:%s============\n",__func__);
    //u8 buf2;
    //focal_fp_sensor_read_sfr_register(spi_sensor, 0x10, &buf2);
    //focal_fp_sensor_read_sfr_register(spi_sensor, 0x14, &buf2);
    focal_fp_sensor_reset(spi_sensor);
    buf[0] = 0x55;
    buf[1] = 0xaa;
    focal_fp_sensor_fw_rw(spi_sensor, buf, 2);
    focal_fp_sensor_write_sfr_register(spi_sensor,0xc8,0xFF);
    focal_fp_sensor_write_sfr_register(spi_sensor,0xca,0xFF);
    focal_fp_sensor_write_sfr_register(spi_sensor,0xcb,0xFF);	

    ret = focal_fp_sensor_version(spi_sensor);
    focal_fp_sensor_reset(spi_sensor);
    if (ret < 0)
    {
	init_flag = 0;
	status = ret;
	goto exit_input_register_device_failed;
	//return ret;
    }
#endif
//add for detect end

#ifdef FTS_APK_DEBUG	
    fts_create_apk_debug_channel_1(spi);
#endif
#ifdef FOCAL_FP_SYSFS_DBG
    focal_fp_create_sysfs(spi);
#endif

    printk("alvin........focal_get_fw_id start\n");
	msleep(100);

    focal_get_fw_id();
    pinctrl_select_state(pinctrl_fp, pins_irq_gpio);		//lizhi
    enable_irq(g_fp_spi_sensor->irq);
	g_fp_spi_sensor->spiclk_enabled = 0;
	ftsfp_spiclk_enable(g_fp_spi_sensor);
    printk("alvin........focal_get_fw_id end\n");
    printk("__%s %d__end_\n", __func__, __LINE__);
    
//    return status;
    goto probe_fail;
	
#ifdef FOCAL_FP_DETECT_FUNC
//exit_standby_irq_request_failed:
	//free_irq(spi_sensor->standby_irq, spi_sensor);
#endif
err_firmware_download:
    focal_fp_sensor_power_down(spi_sensor);

exit_input_register_device_failed:
	input_unregister_device(input_dev);
	input_dev = NULL;
err_free_mem2:
	//kfree(g_rw_buf);
	//kfree(spi_sensor);
probe_fail:
	return status;
}

//static int __devexit focal_fp_sensor_remove(struct spi_device *spi)
static int focal_fp_sensor_remove(struct spi_device *spi)
{
	struct focal_fp_sensor_data	*spi_sensor = (struct focal_fp_sensor_data *)spi_get_drvdata(spi);
	
	 focal_fp_sensor_power_down(spi_sensor);
	 ftsfp_spiclk_disable(g_fp_spi_sensor);
#ifdef CONFIG_HAS_EARLYSUSPEND
       unregister_early_suspend(&spi_sensor->focal_early_suspend);
#endif/* CONFIG_HAS_EARLYSUSPEND */

#ifdef FOCAL_FP_SYSFS_DBG
	focal_fp_release_sysfs(spi_sensor->spi);
#endif
 #ifdef FTS_APK_DEBUG     		
	 fts_release_apk_debug_channel_1();
 #endif
	input_unregister_device(spi_sensor->input);
	spi_sensor->input = NULL;
	
	init_flag = 0;

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&spi_sensor->spi_lock);
	spi_sensor->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&spi_sensor->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&spi_sensor->device_entry);
	//clear_bit(MINOR(spi_sensor->devt), minors);
	//kfree(g_rw_user_buf);
	kfree(g_rw_buf);
	if (spi_sensor->users == 0)
		kfree(spi_sensor);
	mutex_unlock(&device_list_lock);

	return 0;
}

static struct spi_driver focal_fp_sensor_driver = {
	.driver = {
		.name 		= CHRD_DRIVER_NAME,
		.bus 		= &spi_bus_type,
		.owner 		= THIS_MODULE,
#ifdef CONFIG_OF
	       .of_match_table = focal_of_match,
#endif
	},
	.probe = focal_fp_sensor_probe,
	.remove = focal_fp_sensor_remove,
//	.id_table = &spi_id_table,
};
#if 1
static struct spi_board_info spi_board_devs[] __initdata = {
	[0] = {
		.modalias= CHRD_DRIVER_NAME,
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_0,
	//	.controller_data = &focal_chip_config,
	},
};
#endif

/*-------------------------------------------------------------------------*/
static int fp_logcat_init_ok = 0;		//lizhi
static int ftsFP_local_init(void)
{
	int status=0;;	
	printk("alvin........focal_fp_sensor_init start\n");
    mt_set_gpio_mode((97 | 0x80000000), GPIO_MODE_00);
    mt_set_gpio_dir((97 | 0x80000000), GPIO_DIR_IN);
    mt_set_gpio_pull_enable((97 | 0x80000000), GPIO_PULL_DISABLE);
    mt_set_gpio_mode((98 | 0x80000000), GPIO_MODE_00);
    mt_set_gpio_dir((98 | 0x80000000), GPIO_DIR_OUT);
    mt_set_gpio_out((98 | 0x80000000), GPIO_OUT_ONE);
	msleep(10);

if (mt_get_gpio_in(GPIO97 | 0x80000000) == 0){
	printk("focal:........current plug in focal sensor fignerprint\n");
	spi_register_board_info(spi_board_devs, ARRAY_SIZE(spi_board_devs));
	devno = MKDEV(SPIDEV_MAJOR,0);  
	status = register_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME, &focal_fp_sensor_fops);
    	if (status < 0){
	    	printk("%s, Failed to register char device!\n", __func__);
	     	goto init_fail;
        }

        focal_spi_class = class_create(THIS_MODULE, CLASS_NAME);
        if (IS_ERR(focal_spi_class)) {
                unregister_chrdev(SPIDEV_MAJOR, focal_fp_sensor_driver.driver.name);
                printk("%s, Failed to create class.\n", __func__);
		status = PTR_ERR(focal_spi_class);
                goto init_fail;
        }
	device_create(focal_spi_class,NULL,devno,NULL,"focal_fp");
	status = spi_register_driver(&focal_fp_sensor_driver);
	if (status < 0) {
		device_destroy(focal_spi_class, MKDEV(SPIDEV_MAJOR,0));                 //lizhi
		class_destroy(focal_spi_class);
       	unregister_chrdev(SPIDEV_MAJOR, focal_fp_sensor_driver.driver.name);
		printk("spi_register_driver error!\n");
		status = -EINVAL;
		goto init_fail;
	}
	fp_logcat_init_ok = 1;
	printk("alvin........focal_fp_sensor_init end\n");
    return status;

init_fail:
	printk("alvin........focal_fp_sensor_init error\n");
    return status;

}
else{
	mt_set_gpio_mode((98 | 0x80000000), GPIO_MODE_00);
    mt_set_gpio_dir((98 | 0x80000000), GPIO_DIR_OUT);
    mt_set_gpio_out((98 | 0x80000000), GPIO_OUT_ONE);
	
	}
return status;

}

static int ftsFP_local_uninit(void)
{
	if (fp_logcat_init_ok == 1)
	{
		spi_unregister_driver(&focal_fp_sensor_driver);
		device_destroy(focal_spi_class, MKDEV(SPIDEV_MAJOR,0));			//lizhi
		class_destroy(focal_spi_class);
		unregister_chrdev(SPIDEV_MAJOR, focal_fp_sensor_driver.driver.name);
	}
	return 0;
}

static int ftsFP_init_ok(void)
{
	return init_flag;
}

static struct fp_driver_t device_driver = {
   .device_name 	= "focal_fp", 
   .local_init 		= ftsFP_local_init,
   .local_uninit 	= ftsFP_local_uninit,
   .init_ok 		= ftsFP_init_ok,
};

static int __init ftsFP_init(void)
{
	printk("focaltech fingerprint_init entry!");
	fp_driver_load(&device_driver);
	printk("focaltech fingerprint_init out!");
	return 0;
}

static void __exit ftsFP_exit(void)
{
	fp_driver_remove(&device_driver);  
	printk("focaltech fingerprint_exit entry!");
}

module_init(ftsFP_init);
module_exit(ftsFP_exit);


MODULE_AUTHOR("focaltech");
MODULE_DESCRIPTION("Focaltech Fingerprint chip FT9338/FT9336 TEE driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:ftsFP_spi");
