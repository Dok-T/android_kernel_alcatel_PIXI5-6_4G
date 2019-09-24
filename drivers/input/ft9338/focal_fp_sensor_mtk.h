#ifndef FOCAL_FP_SENSOR_H
#define FOCAL_FP_SENSOR_H

#include <linux/types.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

/**************************debug******************************/
#define ERR_LOG  	(0)
#define INFO_LOG 	(1)
#define DEBUG_LOG 	(2)


#define ftsFP_DBG(level, fmt, args...) 	do { \
						if (g_debug_level >= level) {\
							pr_warn("[gf] " fmt, ##args); \
						} \
					} while (0)

#define FUNC_ENTRY()  	ftsFP_DBG(DEBUG_LOG, "%s, %d, enter\n", __func__, __LINE__)
#define FUNC_EXIT()  	ftsFP_DBG(DEBUG_LOG, "%s, %d, exit\n",  __func__, __LINE__)

/**************************IOC_MAGIC******************************/

#define FOCAL_FP_IOC_MAGIC		'k'
#define FOCAL_FP_SYSFS_DBG
#define FOCAL_FP_SENSOR_MAJOR		150	
#define N_SPI_MINORS			32	

#define SUPPLY_3V3          		3300000UL

#define SPI_CPHA			0x01
#define SPI_CPOL			0x02

#define SPI_MODE_0			(0|0)
#define SPI_MODE_1			(0|SPI_CPHA)
#define SPI_MODE_2			(SPI_CPOL|0)
#define SPI_MODE_3			(SPI_CPOL|SPI_CPHA)

#define SPI_CS_HIGH		0x04
#define SPI_LSB_FIRST		0x08
#define SPI_3WIRE		0x10
#define SPI_LOOP		0x20
#define SPI_NO_CS		0x40
#define SPI_READY		0x80


#define IMAGE_COLUMN			88//160 
#define IMAGE_ROW				176//88 * 2//320 //160 * 2
#define IMAGE_TOTAL_SZ			(IMAGE_COLUMN * IMAGE_ROW)	
#define IMAGE_READ_SIZE		32// 176//160
#define IMAGE_READ_LINE		484// 88//320
#define IMAGE_ADDRESS_OFFSET	16// 88//80

#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY)

#define FOCAL_WRITE	0
#define FOCAL_READ		1

struct focal_fp_sensor_data {
	dev_t				devt;
	spinlock_t			spi_lock;
	struct spi_device		*spi;
	struct list_head		device_entry;
	struct mutex			buf_lock;
	unsigned				users;
	u8					*buffer;
	unsigned int			standby_irq;
	unsigned int			sleep_irq;
	unsigned int			irq;
	int 					spiclk_enabled;
	struct input_dev     	*input;	//report power key event
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend 	focal_early_suspend;
#endif
};
#define FTS_APK_DEBUG
extern void fts_release_apk_debug_channel_1(void);
extern int fts_create_apk_debug_channel_1(struct spi_device *spi);
extern ssize_t focal_fp_sensor_fw_rw(struct focal_fp_sensor_data *spi_sensor, u8 *buf,u16 buflen);
extern ssize_t getmcustatus(void);
extern struct focal_fp_sensor_data *g_fp_spi_sensor;

struct focal_fp_rw_operate {
	unsigned short len;
	unsigned char flag;		//0-write 1-read
	unsigned char __user *buf;
};
#if 0
struct spi_ioc_transfer {
	__u64		tx_buf;
	__u64		rx_buf;

	__u32		len;
	__u32		speed_hz;

	__u16		delay_usecs;
	__u8		bits_per_word;
	__u8		cs_change;
	__u32		pad;
};

/* not all platforms use <asm-generic/ioctl.h> or _IOC_TYPECHECK() ... */
#define SPI_MSGSIZE(N) \
	((((N)*(sizeof (struct spi_ioc_transfer))) < (1 << _IOC_SIZEBITS)) \
		? ((N)*(sizeof (struct spi_ioc_transfer))) : 0)
#define SPI_IOC_MESSAGE(N) _IOW(FOCAL_FP_IOC_MAGIC, 0, char[SPI_MSGSIZE(N)])
#endif

/* Read / Write of SPI mode (SPI_MODE_0..SPI_MODE_3) */
//#define SPI_IOC_RD_MODE				_IOR(FOCAL_FP_IOC_MAGIC, 1, __u8)
//#define SPI_IOC_WR_MODE				_IOW(FOCAL_FP_IOC_MAGIC, 1, __u8)

/* Read / Write SPI bit justification */
//#define SPI_IOC_RD_LSB_FIRST			_IOR(FOCAL_FP_IOC_MAGIC, 2, __u8)
//#define SPI_IOC_WR_LSB_FIRST			_IOW(FOCAL_FP_IOC_MAGIC, 2, __u8)

/* Read / Write SPI device word length (1..N) */
//#define SPI_IOC_RD_BITS_PER_WORD		_IOR(FOCAL_FP_IOC_MAGIC, 3, __u8)
//#define SPI_IOC_WR_BITS_PER_WORD		_IOW(FOCAL_FP_IOC_MAGIC, 3, __u8)

/* Read / Write SPI device default max speed hz */
//#define SPI_IOC_RD_MAX_SPEED_HZ			_IOR(FOCAL_FP_IOC_MAGIC, 4, __u32)
//#define SPI_IOC_WR_MAX_SPEED_HZ			_IOW(FOCAL_FP_IOC_MAGIC, 4, __u32)

/* Write the value to be used by the GPIO */

#define FOCAL_FP_IOC_RST_SENSOR			_IO(FOCAL_FP_IOC_MAGIC, 5)

#define FOCAL_FP_IOC_RST_DOWNLOAD_SENSOR	_IO(FOCAL_FP_IOC_MAGIC, 6)

#define FOCAL_FP_IOC_SET_SENSOR_STATUS		_IOW(FOCAL_FP_IOC_MAGIC, 7, __u8)
#define FOCAL_FP_IOC_GET_SENSOR_STATUS		_IOR(FOCAL_FP_IOC_MAGIC, 7, __u8)

#define FOCAL_FP_IOC_IS_FINGER_ON		_IOR(FOCAL_FP_IOC_MAGIC, 10, __u8)

#define FOCAL_FP_LIGHTSCREEN			_IOW(FOCAL_FP_IOC_MAGIC, 12, __u8)

#define FOCAL_FP_WAKEUP_SYSTEM			_IOW(FOCAL_FP_IOC_MAGIC, 13, __u8)

#define FOCAL_FP_SET_INTERRUPT_WAKE_STATUS	_IOW(FOCAL_FP_IOC_MAGIC, 14, __u8)

#define FOCAL_FP_WRITE_READ_DATA		_IOWR(FOCAL_FP_IOC_MAGIC, 16, __u8*)

#define FOCAL_FP_GET_MCU_STATUS         	_IOR(FOCAL_FP_IOC_MAGIC, 17, __u8)
	
#define FOCAL_FP_SET_IRQ_MASK             	_IOW(FOCAL_FP_IOC_MAGIC, 18, __u8)  
#define FOCAL_FP_SET_KEY_MASK			_IOW(FOCAL_FP_IOC_MAGIC, 19, __u8)
#define FOCAL_FP_SPI_CLK_ONOFF			_IOW(FOCAL_FP_IOC_MAGIC, 20, __u8)
#define FOCAL_FP_CA_LOCK_ONOFF			_IOW(FOCAL_FP_IOC_MAGIC, 21, __u8)
#endif 
