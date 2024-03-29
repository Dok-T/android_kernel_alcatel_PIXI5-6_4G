/*
 * MELFAS MIP4 Touchscreen for MediaTek
 *
 * Copyright (C) 2015 MELFAS Inc.
 *
 *
 * mip4_fw_mms438.c : Firmware update functions for MMS438/449/458
 *
 *
 * Version : 2015.08.23
 *
 */

#include "mip4.h"

//Firmware Info
#ifdef CONFIG_TOUCHSCREEN_MTK_MELFAS_MMS438
    #define FW_CHIP_CODE	"M4H0"
#endif
#ifdef CONFIG_TOUCHSCREEN_MTK_MELFAS_MMS449
    #define FW_CHIP_CODE	"M4HP"
#endif
#ifdef CONFIG_TOUCHSCREEN_MTK_MELFAS_MMS458
    #define FW_CHIP_CODE	"M4HP"
#endif

#if 0
    #define FW_TYPE_MFSB
#else
    #define FW_TYPE_TAIL
#endif

//ISC Info
#define ISC_PAGE_SIZE				128

//ISC Command
#define ISC_CMD_ERASE_PAGE		{0xFB,0x4A,0x00,0x8F,0x00,0x00}
#define ISC_CMD_READ_PAGE		{0xFB,0x4A,0x00,0xC2,0x00,0x00}
#define ISC_CMD_PROGRAM_PAGE	{0xFB,0x4A,0x00,0x54,0x00,0x00}
#define ISC_CMD_READ_STATUS		{0xFB,0x4A,0x36,0xC2,0x00,0x00}
#define ISC_CMD_EXIT			{0xFB,0x4A,0x00,0x66,0x00,0x00}

//ISC Status
#define ISC_STATUS_BUSY			0x96
#define ISC_STATUS_DONE			0xAD

#ifdef FW_TYPE_MFSB
/**
* Firmware binary header info
*/
struct mip_bin_hdr {
    char tag[8];
    u16	core_version;
    u16	section_num;
    u16	contains_full_binary;
    u16	reserved0;

    u32	binary_offset;
    u32	binary_length;
    u32	extention_offset;
    u32	reserved1;
} __attribute__ ((packed));

/**
* Firmware image info
*/
struct mip_fw_img {
    u16	type;
    u16	version;
    u16	start_page;
    u16	end_page;

    u32	offset;
    u32	length;
} __attribute__ ((packed));
#endif

#ifdef FW_TYPE_TAIL
/**
* Firmware binary tail info
*/
struct mip_bin_tail {
    u8 tail_mark[4];
    char chip_name[4];
    u32 bin_start_addr;
    u32 bin_length;

    u16 ver_boot;
    u16 ver_core;
    u16 ver_app;
    u16 ver_param;
    u8 boot_start;
    u8 boot_end;
    u8 core_start;
    u8 core_end;
    u8 app_start;
    u8 app_end;
    u8 param_start;
    u8 param_end;

    u8 checksum_type;
    u8 hw_category;
    u16 param_id;
    u32 param_length;
    u32 build_date;
    u32 build_time;

    u32 reserved1;
    u32 reserved2;
    u16 reserved3;
    u16 tail_size;
    u32 crc;
} __attribute__ ((packed));

#define MIP_BIN_TAIL_MARK		{0x4D, 0x42, 0x54, 0x01}	// M B T 0x01
#define MIP_BIN_TAIL_SIZE		64
#endif

/**
* Read ISC status
*/
static int mip_isc_read_status(struct mip_ts_info *info)
{
    u8 cmd[6] = ISC_CMD_READ_STATUS;
    u8 result = 0;
    int cnt = 100;
    int ret = 0;
    u8 rbuf[8];
    MIP_DEBUG("%s [START]\n", __func__);

    do {
        if(mip_i2c_read(info, cmd, 6, rbuf, 1)) {
            dev_err(&info->client->dev, "%s [ERROR] mip_i2c_read\n", __func__);
            ret = -1;
            goto ERROR;
        }

        result = rbuf[0];

        if(result == ISC_STATUS_DONE) {
            ret = 0;
            break;

        } else if(result == ISC_STATUS_BUSY) {
            ret = -1;
            msleep(1);

        } else {
            dev_err(&info->client->dev, "%s [ERROR] wrong value [0x%02X]\n",
                    __func__, result);
            ret = -1;
            msleep(1);
        }
    } while (--cnt);

    if (!cnt) {
        dev_err(&info->client->dev,
                "%s [ERROR] count overflow - cnt [%d] status [0x%02X]\n",
                __func__, cnt, result);
        goto ERROR;
    }

    MIP_DEBUG("%s [DONE]\n", __func__);
    return ret;
ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return ret;
}

/**
* Command : Erase Page
*/
static int mip_isc_erase_page(struct mip_ts_info *info, int offset)
{
    u8 write_buf[6] = ISC_CMD_ERASE_PAGE;
    write_buf[4] = (u8)(((offset) >> 8) & 0xFF );
    write_buf[5] = (u8)(((offset) >> 0) & 0xFF );
    MIP_INFO("%s [START]\n", __func__);

    if(mip_i2c_write(info, write_buf, 6)) {
        dev_err(&info->client->dev, "%s [ERROR] mip_i2c_write\n", __func__);
        goto ERROR;
    }

    if(mip_isc_read_status(info) != 0)
        goto ERROR;

    MIP_INFO("%s [DONE] - Offset [0x%04X]\n", __func__, offset);
    return 0;
ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return -1;
}

/**
* Command : Read Page
*/
static int mip_isc_read_page(struct mip_ts_info *info, int offset, u8 *data)
{
    u8 write_buf[6] = ISC_CMD_READ_PAGE;
    write_buf[4] = (u8)(((offset) >> 8) & 0xFF );
    write_buf[5] = (u8)(((offset) >> 0) & 0xFF );
    MIP_DEBUG("%s [START]\n", __func__);

    if(mip_i2c_read(info, write_buf, 6, data, ISC_PAGE_SIZE)) {
        dev_err(&info->client->dev, "%s [ERROR] mip_i2c_read\n", __func__);
        goto ERROR;
    }

    MIP_DEBUG("%s [DONE] - Offset [0x%04X]\n", __func__, offset);
    return 0;
ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return -1;
}

/**
* Command : Program Page
*/
static int mip_isc_program_page(struct mip_ts_info *info, int offset,
                                const u8 *data, int length)
{
    u8 write_buf[6 + ISC_PAGE_SIZE] = ISC_CMD_PROGRAM_PAGE;
    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    if(length > ISC_PAGE_SIZE) {
        dev_err(&info->client->dev, "%s [ERROR] page length overflow\n",
                __func__);
        goto ERROR;
    }

    write_buf[4] = (u8)(((offset) >> 8) & 0xFF );
    write_buf[5] = (u8)(((offset) >> 0) & 0xFF );
    memcpy(&write_buf[6], data, length);

    if(mip_i2c_write(info, write_buf, (6 + length))) {
        dev_err(&info->client->dev, "%s [ERROR] mip_i2c_write\n", __func__);
        goto ERROR;
    }

    if(mip_isc_read_status(info) != 0)
        goto ERROR;

    dev_dbg(&info->client->dev, "%s [DONE] - Offset[0x%04X] Length[%d]\n",
            __func__, offset, length);
    return 0;
ERROR:
    return -1;
}

/**
* Command : Exit ISC
*/
static int mip_isc_exit(struct mip_ts_info *info)
{
    u8 write_buf[6] = ISC_CMD_EXIT;
    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    if(mip_i2c_write(info, write_buf, 6)) {
        dev_err(&info->client->dev, "%s [ERROR] mip_i2c_write\n", __func__);
        goto ERROR;
    }

    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;
ERROR:
    return -1;
}

#ifdef FW_TYPE_MFSB
/**
* Flash chip firmware (main function)
*/
int mip_flash_fw(struct mip_ts_info *info, const u8 *fw_data, size_t fw_size,
                 bool force, bool section)
{
    struct mip_bin_hdr *fw_hdr;
    struct mip_fw_img **img;
    struct i2c_client *client = info->client;
    int i;
    int retires = 3;
    int nRet;
    int nStartAddr;
    int nWriteLength;
    int nLast;
    int nOffset;
    int nTransferLength;
    int size;
    u8 *data;
    u8 *cpydata;
    int offset = sizeof(struct mip_bin_hdr);
    bool update_flag = false;
    bool update_flags[MIP_FW_MAX_SECT_NUM] = {false, };
    u16 ver_chip[MIP_FW_MAX_SECT_NUM];
    u16 ver_file[MIP_FW_MAX_SECT_NUM];
    int offsetStart = 0;
    u8 initData[ISC_PAGE_SIZE];
    memset(initData, 0xFF, sizeof(initData));
    dev_dbg(&client->dev, "%s [START]\n", __func__);
    //Read firmware file
    fw_hdr = (struct mip_bin_hdr *)fw_data;
    img = kzalloc(sizeof(*img) * fw_hdr->section_num, GFP_KERNEL);

    //Check firmware file
    /*   if(memcmp(FW_CHIP_CODE, &fw_hdr->tag[4], 4)) {
           dev_err(&client->dev, "%s [ERROR] F/W file is not for %s\n", __func__, CHIP_NAME);

           nRet = fw_err_file_type;
           goto ERROR;
       }*/

    //Reboot chip
    //    mip_reboot(info);

    //Check chip firmware version
    while (retires--) {
        if (!mip_get_fw_version_u16(info, ver_chip))
            break;

        else {
            //          mip_reboot(info);
        }
    }

    if (retires < 0) {
        dev_err(&client->dev, "%s [ERROR] cannot read chip firmware version\n",
                __func__);
        memset(ver_chip, 0xFFFF, sizeof(ver_chip));
        dev_info(&client->dev, "%s - Chip firmware version is set to [0xFFFF]\n",
                 __func__);

    } else
        dev_info(&client->dev,
                 "%s - Chip firmware version [0x%04X 0x%04X 0x%04X 0x%04X]\n", __func__,
                 ver_chip[0], ver_chip[1], ver_chip[2], ver_chip[3]);

    //Set update flag
    dev_info(&client->dev,
             "%s - Firmware file info : Sections[%d] Offset[0x%08X] Length[0x%08X]\n",
             __func__, fw_hdr->section_num, fw_hdr->binary_offset, fw_hdr->binary_length);

    for (i = 0; i < fw_hdr->section_num; i++, offset += sizeof(struct mip_fw_img)) {
        img[i] = (struct mip_fw_img *)(fw_data + offset);
        ver_file[i] = img[i]->version;
        dev_info(&client->dev,
                 "%s - Section info : Section[%d] Version[0x%04X] StartPage[%d] EndPage[%d] Offset[0x%08X] Length[0x%08X]\n",
                 __func__, i, img[i]->version, img[i]->start_page, img[i]->end_page,
                 img[i]->offset, img[i]->length);

        //Compare section version
        if (ver_chip[i] != ver_file[i]) {
            //Set update flag
            update_flag = true;
            update_flags[i] = true;
            dev_info(&client->dev,
                     "%s - Section [%d] is need to be updated. Version : Chip[0x%04X] File[0x%04X]\n",
                     __func__, i, ver_chip[i], ver_file[i]);
        }
    }

    //Set force update flag
    if(force == true) {
        update_flag = true;
        update_flags[0] = true;
        update_flags[1] = true;
        update_flags[2] = true;
        update_flags[3] = true;
        dev_info(&client->dev, "%s - Force update\n", __func__);
    }

    //Exit when up-to-date
    if(update_flag == false) {
        nRet = fw_err_uptodate;
        dev_dbg(&client->dev, "%s [DONE] Chip firmware is already up-to-date\n",
                __func__);
        goto EXIT;
    }

    //Set start addr offset
    if(section == true) {
        if(update_flags[0] == true) {
            //boot
            offsetStart = img[0]->start_page;

        } else if(update_flags[1] == true) {
            //core
            offsetStart = img[1]->start_page;

        } else if(update_flags[2] == true) {
            //custom
            offsetStart = img[2]->start_page;

        } else if(update_flags[3] == true) {
            //param
            offsetStart = img[3]->start_page;
        }

    } else
        offsetStart = 0;

    offsetStart = offsetStart * 1024;
    //Load firmware data
    data = kzalloc(sizeof(u8) * fw_hdr->binary_length, GFP_KERNEL);
    size = fw_hdr->binary_length;
    cpydata = kzalloc(ISC_PAGE_SIZE, GFP_KERNEL);

    //Check firmware size
    if(size % ISC_PAGE_SIZE != 0)
        size += ( ISC_PAGE_SIZE - (size % ISC_PAGE_SIZE) );

    nStartAddr = 0;
    nWriteLength = size;
    nLast = nStartAddr + nWriteLength;

    if((nLast) % 8 != 0) {
        nRet = fw_err_file_type;
        dev_err(&client->dev, "%s [ERROR] Firmware size mismatch\n", __func__);
        goto ERROR;

    } else
        memcpy(data, fw_data + fw_hdr->binary_offset, fw_hdr->binary_length);

    //Set address
    nOffset = nStartAddr + nWriteLength - ISC_PAGE_SIZE;
    nTransferLength = ISC_PAGE_SIZE;
    //Erase first page
    dev_info(&client->dev, "%s - Erase first page : Offset[0x%04X]\n", __func__,
             offsetStart);
    nRet = mip_isc_erase_page(info, offsetStart);

    if( nRet != 0 ) {
        dev_err(&client->dev, "%s [ERROR] clear first page failed\n", __func__);
        goto ERROR;
    }

    //Flash firmware
    dev_info(&client->dev,
             "%s - Start Download : Offset Start[0x%04X] End[0x%04X]\n", __func__, nOffset,
             offsetStart);

    while( nOffset >= offsetStart ) {
        dev_info(&client->dev, "%s - Downloading : Offset[0x%04X]\n", __func__, nOffset);
        //Program (erase and write) a page
        nRet = mip_isc_program_page(info, nOffset, &data[nOffset], nTransferLength);

        if( nRet != 0 ) {
            dev_err(&client->dev, "%s [ERROR] isc_program_page\n", __func__);
            goto ERROR;
        }

        //Verify (read and compare)
        if (mip_isc_read_page(info, nOffset, cpydata)) {
            dev_err(&client->dev, "%s [ERROR] mip_isc_read_page\n", __func__);
            goto ERROR;
        }

        if (memcmp(&data[nOffset], cpydata, ISC_PAGE_SIZE)) {
#if MIP_FW_UPDATE_DEBUG
            print_hex_dump(KERN_ERR, "Firmware Page Write : ", DUMP_PREFIX_OFFSET, 16, 1,
                           data, ISC_PAGE_SIZE, false);
            print_hex_dump(KERN_ERR, "Firmware Page Read : ", DUMP_PREFIX_OFFSET, 16, 1,
                           cpydata, ISC_PAGE_SIZE, false);
#endif
            dev_err(&client->dev, "%s [ERROR] verify page failed\n", __func__);
            nRet = -1;
            goto ERROR;
        }

        nOffset -= nTransferLength;
    }

    //Exit ISC
    nRet = mip_isc_exit(info);

    if( nRet != 0 ) {
        dev_err(&client->dev, "%s [ERROR] mip_isc_exit\n", __func__);
        goto ERROR;
    }

    //Reboot chip
    //    mip_reboot(info);

    //Check chip firmware version
    if (mip_get_fw_version_u16(info, ver_chip)) {
        dev_err(&client->dev,
                "%s [ERROR] cannot read chip firmware version after flash\n", __func__);
        nRet = -1;
        goto ERROR;

    } else {
        for (i = 0; i < fw_hdr->section_num; i++) {
            if (ver_chip[i] != ver_file[i]) {
                dev_err(&client->dev,
                        "%s [ERROR] version mismatch after flash. Section[%d] : Chip[0x%04X] != File[0x%04X]\n",
                        __func__, i, ver_chip[i], ver_file[i]);
                nRet = -1;
                goto ERROR;
            }
        }
    }

    nRet = 0;
    dev_dbg(&client->dev, "%s [DONE]\n", __func__);
    dev_info(&client->dev, "Firmware update completed\n");
    goto EXIT;
ERROR:
    dev_err(&client->dev, "%s [ERROR]\n", __func__);
    dev_err(&client->dev, "Firmware update failed\n");
    goto EXIT;
EXIT:
    kfree(img);
    return nRet;
}

/**
* Get version of F/W bin file
*/
int mip_bin_fw_version(struct mip_ts_info *info, const u8 *fw_data,
                       size_t fw_size, u8 *ver_buf)
{
    struct mip_bin_hdr *fw_hdr;
    struct mip_fw_img **img;
    int offset = sizeof(struct mip_bin_hdr);
    int i = 0;
    dev_dbg(&info->client->dev, "%s [START]\n", __func__);
    fw_hdr = (struct mip_bin_hdr *)fw_data;
    img = kzalloc(sizeof(*img) * fw_hdr->section_num, GFP_KERNEL);

    for(i = 0; i < fw_hdr->section_num; i++, offset += sizeof(struct mip_fw_img)) {
        img[i] = (struct mip_fw_img *)(fw_data + offset);
        ver_buf[i * 2] = ((img[i]->version) >> 8) & 0xFF;
        ver_buf[i * 2 + 1] = (img[i]->version) & 0xFF;
    }

    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;
}

void mip_fw_update_controller(struct mip_ts_info *info,
                              const struct firmware *fw, struct i2c_client *client)
{
    int retry = 3;
    int ret;
    printk("mip_fw_update_controller \n");

    do {
        ret = mip_flash_fw(info, fw->data, fw->size, false, true);

        //ret = mms_flash_fw(info, fw->data, fw->size, true, true);
        if(ret >= fw_err_none)
            break;
    } while (--retry);

    if (!retry)
        printk("failed to flash firmware after retires\n");
}

#endif

#ifdef FW_TYPE_TAIL
/**
* Flash chip firmware (main function)
*/
int mip_flash_fw(struct mip_ts_info *info, const u8 *fw_data, size_t fw_size,
                 bool force, bool section)
{
    struct i2c_client *client = info->client;
    struct mip_bin_tail *bin_info;
    int ret = 0;
    int retry = 3;
    u8 rbuf[ISC_PAGE_SIZE];
    int offset = 0;
    int offset_start = 0;
    int bin_size = 0;
    u8 *bin_data;
    u16 tail_size = 0;
    u8 tail_mark[4] = MIP_BIN_TAIL_MARK;
    u16 ver_chip[MIP_FW_MAX_SECT_NUM];
    MIP_INFO("%s [START]\n", __func__);
    //Check tail size
    tail_size = (fw_data[fw_size - 5] << 8) | fw_data[fw_size - 6];

    if(tail_size != MIP_BIN_TAIL_SIZE) {
        dev_err(&client->dev, "%s [ERROR] wrong tail size [%d]\n", __func__, tail_size);
        ret = fw_err_file_type;
        goto ERROR;
    }

    //Check bin format
    if(memcmp(&fw_data[fw_size - tail_size], tail_mark, 4)) {
        dev_err(&client->dev, "%s [ERROR] wrong tail mark\n", __func__);
        ret = fw_err_file_type;
        goto ERROR;
    }

    //Read bin info
    bin_info = (struct mip_bin_tail *)&fw_data[fw_size - tail_size];
    MIP_INFO("%s - bin_info : bin_len[%d] hw_cat[0x%2X] date[%4X] time[%4X] "
             "tail_size[%d]\n", __func__, bin_info->bin_length,
             bin_info->hw_category, bin_info->build_date,
             bin_info->build_time, bin_info->tail_size);
#if MIP_FW_UPDATE_DEBUG
    print_hex_dump(KERN_ERR, MIP_DEVICE_NAME " Bin Info : ", DUMP_PREFIX_OFFSET,
                   16, 1, bin_info, tail_size, false);
#endif

    //Check chip code
    if(memcmp(bin_info->chip_name, FW_CHIP_CODE, 4)) {
        dev_err(&client->dev, "%s [ERROR] F/W file is not for %s\n", __func__,
                CHIP_NAME);
        ret = fw_err_file_type;
        goto ERROR;
    }

    //Check F/W version
    MIP_INFO("%s - F/W file version [0x%04X 0x%04X 0x%04X 0x%04X]\n",
             __func__, bin_info->ver_boot, bin_info->ver_core, bin_info->ver_app,
             bin_info->ver_param);

    if(force == true) {
        //Force update
        MIP_INFO("%s - Skip chip firmware version check\n", __func__);

    } else {
        //Read firmware version from chip
        while(retry--) {
            if(!mip_get_fw_version_u16(info, ver_chip))
                break;

            else
                mip_reboot(info);
        }

        if(retry < 0)
            dev_err(&client->dev, "%s [ERROR] Unknown chip firmware version\n",
                    __func__);

        else {
            MIP_INFO("%s - Chip firmware version [0x%04X 0x%04X 0x%04X 0x%04X]\n",
                     __func__, ver_chip[0], ver_chip[1], ver_chip[2],
                     ver_chip[3]);

            //Compare version
            if((ver_chip[2] >= bin_info->ver_app)
                    && (ver_chip[3] >= bin_info->ver_param)) {
                MIP_INFO("%s - Chip firmware is already up-to-date\n", __func__);
                ret = fw_err_uptodate;
                goto EXIT;
            }
        }
    }

    //Read bin data
    bin_size = bin_info->bin_length;
    bin_data = kzalloc(sizeof(u8) * (bin_size), GFP_KERNEL);
    memcpy(bin_data, fw_data, bin_size);
    //Erase first page
    offset = 0;
    MIP_INFO("%s - Erase first page : Offset[0x%04X]\n", __func__, offset);
    ret = mip_isc_erase_page(info, offset);

    if(ret != 0) {
        dev_err(&client->dev, "%s [ERROR] mip_isc_erase_page\n", __func__);
        ret = fw_err_download;
        goto ERROR;
    }

    //Program & Verify
    MIP_INFO("%s - Program & Verify\n", __func__);
    offset_start = 0;
    offset = bin_size - ISC_PAGE_SIZE;

    while(offset >= offset_start) {
        //Program page
        if(mip_isc_program_page(info, offset, &bin_data[offset], ISC_PAGE_SIZE)) {
            dev_err(&client->dev, "%s [ERROR] mip_isc_program_page : offset[0x%04X]\n",
                    __func__, offset);
            ret = fw_err_download;
            goto ERROR;
        }

        MIP_DEBUG("%s - mip_isc_program_page : offset[0x%04X]\n",
                  __func__, offset);

        //Verify page
        if(mip_isc_read_page(info, offset, rbuf)) {
            dev_err(&client->dev, "%s [ERROR] mip_isc_read_page : offset[0x%04X]\n",
                    __func__, offset);
            ret = fw_err_download;
            goto ERROR;
        }

        MIP_DEBUG("%s - mip_isc_read_page : offset[0x%04X]\n", __func__, offset);
#if MIP_FW_UPDATE_DEBUG
        print_hex_dump(KERN_ERR, MIP_DEVICE_NAME " F/W File : ",
                       DUMP_PREFIX_OFFSET, 16, 1, &bin_data[offset], ISC_PAGE_SIZE, false);
        print_hex_dump(KERN_ERR, MIP_DEVICE_NAME " F/W Chip : ",
                       DUMP_PREFIX_OFFSET, 16, 1, rbuf, ISC_PAGE_SIZE, false);
#endif

        if(memcmp(rbuf, &bin_data[offset], ISC_PAGE_SIZE)) {
            dev_err(&client->dev, "%s [ERROR] Verify failed : offset[0x%04X]\n",
                    __func__, offset);
            ret = fw_err_download;
            goto ERROR;
        }

        offset -= ISC_PAGE_SIZE;
    }

    //Exit ISC mode
    MIP_INFO("%s - Exit\n", __func__);
    mip_isc_exit(info);
    //Reset chip
    mip_reboot(info);

    //Check chip firmware version
    if(mip_get_fw_version_u16(info, ver_chip)) {
        dev_err(&client->dev, "%s [ERROR] Unknown chip firmware version\n",
                __func__);
        ret = fw_err_download;
        goto ERROR;

    } else {
        if((ver_chip[0] == bin_info->ver_boot) 
                && (ver_chip[1] == bin_info->ver_core)
                && (ver_chip[2] == bin_info->ver_app) 
                && (ver_chip[3] == bin_info->ver_param)) {
            ret = fw_err_none;
            MIP_INFO("%s - Version check OK\n", __func__);

        } else {
            dev_err(&client->dev, "%s [ERROR] Version mismatch after flash. "
                    "Chip[0x%04X 0x%04X 0x%04X 0x%04X]"
                    "File[0x%04X 0x%04X 0x%04X 0x%04X]\n",
                    __func__, ver_chip[0], ver_chip[1], ver_chip[2],
                    ver_chip[3], bin_info->ver_boot, bin_info->ver_core,
                    bin_info->ver_app, bin_info->ver_param);
            ret = fw_err_download;
            goto ERROR;
        }
    }

    goto EXIT;
ERROR:
    //Reset chip
    mip_reboot(info);
    dev_err(&client->dev, "%s [ERROR]\n", __func__);
EXIT:
    MIP_INFO("%s [DONE]\n", __func__);
    return ret;
}

/**
* Get version of F/W bin file
*/
int mip_bin_fw_version(struct mip_ts_info *info, const u8 *fw_data,
                       size_t fw_size, u8 *ver_buf)
{
    struct mip_bin_tail *bin_info;
    u16 tail_size = 0;
    u8 tail_mark[4] = MIP_BIN_TAIL_MARK;
    MIP_DEBUG("%s [START]\n", __func__);
    //Check tail size
    tail_size = (fw_data[fw_size - 5] << 8) | fw_data[fw_size - 6];

    if(tail_size != MIP_BIN_TAIL_SIZE) {
        dev_err(&info->client->dev, "%s [ERROR] wrong tail size [%d]\n",
                __func__, tail_size);
        goto ERROR;
    }

    //Check bin format
    if(memcmp(&fw_data[fw_size - tail_size], tail_mark, 4)) {
        dev_err(&info->client->dev, "%s [ERROR] wrong tail mark\n", __func__);
        goto ERROR;
    }

    //Read bin info
    bin_info = (struct mip_bin_tail *)&fw_data[fw_size - tail_size];
    //F/W version
    ver_buf[0] = (bin_info->ver_boot >> 8) & 0xFF;
    ver_buf[1] = (bin_info->ver_boot) & 0xFF;
    ver_buf[2] = (bin_info->ver_core >> 8) & 0xFF;
    ver_buf[3] = (bin_info->ver_core) & 0xFF;
    ver_buf[4] = (bin_info->ver_app >> 8) & 0xFF;
    ver_buf[5] = (bin_info->ver_app) & 0xFF;
    ver_buf[6] = (bin_info->ver_param >> 8) & 0xFF;
    ver_buf[7] = (bin_info->ver_param) & 0xFF;
    MIP_DEBUG("%s [DONE]\n", __func__);
    return 0;
ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
}

void mip_fw_update_controller(struct mip_ts_info *info,
                              const struct firmware *fw, struct i2c_client *client)
{
    int retry = 3;
    int ret;

    MIP_DEBUG("mip_fw_update_controller \n");

    do {
        ret = mip_flash_fw(info, fw->data, fw->size, false, true);

        if(ret >= fw_err_none)
            break;
    } while (--retry);

    if (!retry)
        MIP_INFO("Failed to flash firmware after retires\n");
}

void mip_kernel_fw_force_update_controller(struct mip_ts_info *info,
        const struct firmware *fw, struct i2c_client *client)
{
    int retry = 8;
    int ret;
    printk("mip_kernel_fw_force_update_controller \n");

    do {
        //ret = mip_flash_fw(info, fw->data, fw->size, false, true);
        ret = mip_flash_fw(info, fw->data, fw->size, true, true);

        if(ret >= fw_err_none)
            break;
    } while (--retry);

    if (!retry)
        printk("failed to flash firmware after retires\n");
}

#endif

