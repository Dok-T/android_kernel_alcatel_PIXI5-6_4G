#include <linux/delay.h>
#include <linux/string.h>
#include <linux/slab.h>
#include "focal_fp_sensor_mtk.h"

#define FOCAL_FP_UPGRADE_DEBUG
#ifdef FOCAL_FP_UPGRADE_DEBUG
#define FT_DBG(fmt, args...)	printk(KERN_ERR"  focal  " fmt, ##args);
#else
#define FT_DBG(fmt, args...) 	do{}while(0)
#endif


extern ssize_t focal_fp_sensor_fw_rw(struct focal_fp_sensor_data *spi_sensor, u8 *buf, u16 buflen);
extern void focal_fp_sensor_reset(struct focal_fp_sensor_data *spi_sensor);

static unsigned char FOCALFP_FW[] = {
	#include "FT9338_Coating_V2E_20170213_app.i"
};

int focal_fp_fw_upgrade(struct focal_fp_sensor_data *spi_sensor)
{
    u8 buf[24] = {0};
    //u32 fw_length = sizeof(FOCALFP_FW);

    buf[0] = 0x55;
    focal_fp_sensor_fw_rw(spi_sensor, buf, 1);
    buf[0] = 0x90;
    buf[0] = 0x00;
    buf[0] = 0x00;
    focal_fp_sensor_fw_rw(spi_sensor, buf, 3);
   
    FT_DBG("focal__buf[1] = %d buf[2] = %d\n", buf[1], buf[2]);
    if (0x93 != buf[1] || 0x08 != buf[2])
    {
        printk("the id number is error! %d %d\n", buf[1], buf[2]);
        return -1;
    }
    FT_DBG("focal___ id = %d  %d", buf[1], buf[2]);
    
    return 0;
}


#define COPY_LENGTH     24//26
int focal_fp_fw_download(struct focal_fp_sensor_data *spi_sensor)
{
		u32 i, loop, remainder;
		u8 buf[32]= {0};
		u32 fw_length = sizeof(FOCALFP_FW);
		
		u8 *check_buf = kzalloc(fw_length, GFP_KERNEL);
		if (!check_buf)
		{
			printk("focal check_buf kzalloc failed!\n");
			return -1;
		}
		
		buf[0] = 0x55;
		buf[1] = 0xaa;
		focal_fp_sensor_fw_rw(spi_sensor, buf, 2);	
		msleep(1);	
	
		buf[0] = 0x09;
		buf[1] = 0xf6;
		buf[2] = 0xc8;
		buf[3] = 0xff;
		buf[4] = 0x00;
		focal_fp_sensor_fw_rw(spi_sensor, buf, 5);
	
		buf[0] = 0x09;
		buf[1] = 0xf6;
		buf[2] = 0xca;
		buf[3] = 0xff;
		buf[4] = 0x00;
		focal_fp_sensor_fw_rw(spi_sensor, buf, 5);
	
		buf[0] = 0x09;
		buf[1] = 0xf6;
		buf[2] = 0xcb;
		buf[3] = 0xff;
		buf[4] = 0x00;
		focal_fp_sensor_fw_rw(spi_sensor, buf, 5);
	
		buf[0] = 0x09;
		buf[1] = 0xf6;
		buf[2] = 0xb9;
		buf[3] = 0xbf;
		buf[4] = 0x00;
		focal_fp_sensor_fw_rw(spi_sensor, buf, 5);
	
		buf[0] = 0x09;
		buf[1] = 0xf6;
		buf[2] = 0xb9;
		buf[3] = 0xff;
		buf[4] = 0x00;
		focal_fp_sensor_fw_rw(spi_sensor, buf, 5);
	
		mdelay(20);
		
		loop = fw_length / COPY_LENGTH;
		remainder = fw_length % COPY_LENGTH;
		FT_DBG("fw_length = %d focal loop = %d remainder = %d\n",fw_length, loop, remainder);
		for (i = 0; i < loop; i++)
		{
			buf[0] = 0x05;
			buf[1] = 0xfa;
			buf[2] = (0x00 + (((i * COPY_LENGTH >> 8) / 2) & 0xff));
			buf[3] = (0x00 + (((i * COPY_LENGTH) / 2) & 0xff));
			buf[4] = (COPY_LENGTH >> 8) & 0xff;
			buf[5] = COPY_LENGTH & 0xff;
			memcpy(&buf[6], FOCALFP_FW + i * COPY_LENGTH, COPY_LENGTH);
			buf[6 + COPY_LENGTH] = 0x00;
			buf[6 + COPY_LENGTH + 1] = 0x00;
			focal_fp_sensor_fw_rw(spi_sensor, buf, COPY_LENGTH + 6 + 2);
		}
		if (0 != remainder)
		{
			buf[0] = 0x05;
			buf[1] = 0xfa;
			buf[2] = (0x00 + (((loop * COPY_LENGTH >> 8) / 2) & 0xff));
			buf[3] = (0x00 + (((loop * COPY_LENGTH) / 2) & 0xff));
			buf[4] = (remainder >> 8) & 0xff;
			buf[5] = remainder & 0xff;
			memcpy(&buf[6], FOCALFP_FW + loop * COPY_LENGTH, remainder);
			buf[6 + remainder] = 0x00;
			buf[6 + remainder + 1] = 0x00;
			focal_fp_sensor_fw_rw(spi_sensor, buf, remainder + 6 + 2);
		}
		mdelay(2);
	
		//check right
		for (i = 0; i < loop; i++)
		{
			buf[0] = 0x04;
			buf[1] = 0xfb;
			buf[2] = (0x00 + (((i * COPY_LENGTH >> 8) / 2) & 0xff));
			buf[3] = (0x00 + (((i * COPY_LENGTH) / 2) & 0xff));
			buf[4] = (COPY_LENGTH >> 8) & 0xff;
			buf[5] = COPY_LENGTH & 0xff;
			focal_fp_sensor_fw_rw(spi_sensor, buf, COPY_LENGTH + 6);
			memcpy(check_buf + (i * COPY_LENGTH), &buf[6], COPY_LENGTH);
		}
		if (0 != remainder)
		{
			buf[0] = 0x04;
			buf[1] = 0xfb;
			buf[2] = (0x00 + (((loop * COPY_LENGTH >> 8) / 2) & 0xff));
			buf[3] = (0x00 + (((loop * COPY_LENGTH) / 2) & 0xff));
			buf[4] = (remainder >> 8) & 0xff;
			buf[5] = remainder & 0xff;
			focal_fp_sensor_fw_rw(spi_sensor, buf, remainder + 6);
			memcpy(check_buf + (loop * COPY_LENGTH), &buf[6], remainder);
		}
	
		for (i = 0; i < fw_length; i++)
		{
			if (FOCALFP_FW[i] != check_buf[i])
			{
				printk("focal check error!!  i = %d _%x %x_\n", i, FOCALFP_FW[i], check_buf[i]);
				return -1;
			}
			//else
			//printk("focal check success!  i = %d FW=%x check_buf=%x\n", i, FOCALFP_FW[i], check_buf[i]);
		}
		FT_DBG("focal check success!!!\n");
		kfree(check_buf);
#if 0	
		buf[0] = 0x0a;
		buf[1] = 0xf5;
		buf[2] = 0x00;
		focal_fp_sensor_fw_rw(spi_sensor, buf, 3);

		mdelay(100);
#else
		focal_fp_sensor_reset(spi_sensor);
		msleep(10);
		focal_fp_sensor_reset(spi_sensor);
#endif		
#if 0
/*
		buf[0] = 0x11;
		buf[1] = 0xee;
		buf[2] = 0x00;
		buf[3] = 0x6b;
		buf[4] = 0x00;
		focal_fp_sensor_fw_rw(spi_sensor, buf, 5);	one intrupt
	*/
		buf[0] = 0x11;
		buf[1] = 0xee;
		buf[2] = 0x1f;
		buf[3] = 0x01;
		buf[4] = 0x00;
		focal_fp_sensor_fw_rw(spi_sensor, buf, 5);

		mdelay(2);

		buf[0] = 0x11;
		buf[1] = 0xee;
		buf[2] = 0x1e;
		buf[3] = 0x01;
		buf[4] = 0x00;
		focal_fp_sensor_fw_rw(spi_sensor, buf, 5);


		mdelay(20);

		buf[0] = 0x10;
		buf[1] = 0xef;
		buf[2] = 0x1d;
		buf[3] = 0x00;
		buf[4] = 0x00;
		focal_fp_sensor_fw_rw(spi_sensor, buf, 5);
		//printk("alvin__buf=%x %x %x %x %x\n",buf[0],buf[1],buf[2],buf[3],buf[4]);

		/*
		*for test
		
		mdelay(10);
		buf[0] = 0x11;
		buf[1] = 0xee;
		buf[2] = 0x1e;
		buf[3] = 0x01;
		buf[4] = 0x00;
		focal_fp_sensor_fw_rw(spi_sensor, buf, 5);
		*/
#endif

		
		return 0;	 
	}
