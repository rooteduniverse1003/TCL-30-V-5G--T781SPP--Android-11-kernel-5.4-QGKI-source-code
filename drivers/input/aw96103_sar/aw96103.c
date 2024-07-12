/*
* aw96103.c
*
* Version: v0.1.4
*
* Copyright (c) 2020 AWINIC Technology CO., LTD
*
* Author: Alex <zhangpengbiao@awinic.com>
*
* This program is free software; you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the
* Free Software Foundation; either version 2 of the License, or (at your
* option) any later version.
*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>
#include <linux/string.h>
#include <linux/jiffies.h>
#include "aw_bin_parse0.h"
#include "aw96103.h"
#include "aw96103_reg.h"

#define AW96103_I2C_NAME "aw96103_sar"
#define AW96103_DRIVER_VERSION "v0.1.4"

#define AW_READ_CHIPID_RETRIES 5
#define AW_I2C_RETRIES 5
#define AW96103_SCAN_DEFAULT_TIME 10000
#define CALI_FILE_MAX_SIZE 128
#define AWINIC_CALI_FILE "/mnt/aw_cali.bin"
static char *aw96103_cfg_name = "aw96103.bin";

/******************************************************
*
* aw9610x i2c write/read
*
******************************************************/

static int32_t
i2c_write(struct aw96103 *aw96103, uint16_t reg_addr16, uint32_t reg_data32)
{	int32_t ret =  -ENOMEM;
	struct i2c_client *i2c = aw96103->i2c;
	struct i2c_msg msg;
	uint8_t w_buf[6];

	/*reg_addr*/
	w_buf[0] = (u8)(reg_addr16>>8);
	w_buf[1] = (u8)(reg_addr16);
	/*data*/
	w_buf[2] = (u8)(reg_data32 >> 24);
	w_buf[3] = (u8)(reg_data32 >> 16);
	w_buf[4] = (u8)(reg_data32 >> 8);
	w_buf[5] = (u8)(reg_data32);

	msg.addr = i2c->addr;
	msg.flags = AW96103_I2C_WR;
	msg.len = 6;
	/*2 bytes regaddr + 4 bytes data*/
	msg.buf = (unsigned char *)w_buf;

	ret = i2c_transfer(i2c->adapter, &msg, 1);
	if (ret < 0)
		pr_info("%s: i2c write reg 0x%x error %d\n", __func__,
							reg_addr16, ret);

	return ret;
}

static int32_t
i2c_read(struct aw96103 *aw96103, uint16_t reg_addr16, uint32_t *reg_data32)
{
	int32_t ret =  -ENOMEM;
	struct i2c_client *i2c = aw96103->i2c;
	struct i2c_msg msg[2];
	uint8_t w_buf[2];
	uint8_t buf[4];

	w_buf[0] = (unsigned char)(reg_addr16 >> 8);
	w_buf[1] = (unsigned char)(reg_addr16);
	msg[0].addr = i2c->addr;
	msg[0].flags = AW96103_I2C_WR;
	msg[0].len = 2;
	msg[0].buf = (unsigned char *)w_buf;

	msg[1].addr = i2c->addr;
	msg[1].flags = AW96103_I2C_RD;
	msg[1].len = 4;
	msg[1].buf = (unsigned char *)buf;

	ret = i2c_transfer(i2c->adapter, msg, 2);
	if (ret < 0)
		pr_info("%s: i2c read reg 0x%x error %d\n", __func__,
							reg_addr16, ret);

	reg_data32[0] = ((u32)buf[3]) | ((u32)buf[2]<<8) |
			((u32)buf[1]<<16) | ((u32)buf[0]<<24);

	return ret;
}

static int32_t aw96103_i2c_write(struct aw96103 *aw96103,
				uint16_t reg_addr16, uint32_t reg_data32)
{
	int32_t ret = -1;
	uint8_t cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_write(aw96103, reg_addr16, reg_data32);
		if (ret < 0) {
			pr_err("%s: i2c_write cnt=%d error=%d\n",
							__func__, cnt, ret);
		} else {
			break;
		}
		cnt++;
	}

	return ret;
}

static int32_t aw96103_i2c_read(struct aw96103 *aw96103,
				uint16_t reg_addr16, uint32_t *reg_data32)
{
	int32_t ret = -1;
	uint8_t cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_read(aw96103, reg_addr16, reg_data32);
		if (ret < 0) {
			pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt,
			       ret);
		} else {
			break;
		}
		cnt++;
	}
	return ret;
}

static int32_t aw96103_i2c_write_bits(struct aw96103 *aw96103,
				 uint16_t reg_addr16, uint32_t mask,
				 uint32_t reg_data32)
{
	uint32_t reg_val;

	aw96103_i2c_read(aw96103, reg_addr16, &reg_val);
	reg_val &= mask;
	reg_val |= reg_data32;
	aw96103_i2c_write(aw96103, reg_addr16, reg_val);

	return 0;
}

/******************************************************************************
*
* aw9610x i2c sequential write/read --- one first addr with multiple data.
*
******************************************************************************/
static int32_t i2c_write_seq(struct aw96103 *aw96103)
{
	int32_t ret =  -ENOMEM;
	struct i2c_client *i2c = aw96103->i2c;
	struct i2c_msg msg;
	uint8_t w_buf[228];
	uint8_t addr_bytes = aw96103->aw_i2c_package.addr_bytes;
	uint8_t msg_cnt = 0;
	uint8_t data_bytes = aw96103->aw_i2c_package.data_bytes;
	uint8_t reg_num = aw96103->aw_i2c_package.reg_num;
	uint8_t *p_reg_data = aw96103->aw_i2c_package.p_reg_data;
	uint8_t msg_idx = 0;

	for (msg_idx = 0; msg_idx < addr_bytes; msg_idx++) {
		w_buf[msg_idx] = aw96103->aw_i2c_package.init_addr[msg_idx];
		pr_info("%s: w_buf_addr[%d] = 0x%02x\n",
					__func__, msg_idx, w_buf[msg_idx]);
	}
	msg_cnt = addr_bytes;
	for (msg_idx = 0; msg_idx < data_bytes * reg_num; msg_idx++) {
		w_buf[msg_cnt] = *p_reg_data++;
		pr_info("%s: w_buf_addr[%d] = 0x%02x\n",
					__func__, msg_cnt, w_buf[msg_cnt]);
		msg_cnt++;
	}
	pr_info("%s: %d\n", __func__, msg_cnt);
	p_reg_data = aw96103->aw_i2c_package.p_reg_data;
	msg.addr = i2c->addr;
	msg.flags = AW96103_I2C_WR;
	msg.len = msg_cnt;
	msg.buf = (uint8_t *)w_buf;
	ret = i2c_transfer(i2c->adapter, &msg, 1);

	if (ret < 0)
		pr_info("%s: i2c write seq error %d\n", __func__, ret);

	return ret;
}

static int32_t i2c_read_seq(struct aw96103 *aw96103, uint8_t *reg_data)
{
	int32_t ret =  -ENOMEM;
	struct i2c_client *i2c = aw96103->i2c;
	struct i2c_msg msg[2];
	uint8_t w_buf[4];
	uint8_t buf[228];
	uint8_t data_bytes = aw96103->aw_i2c_package.data_bytes;
	uint8_t reg_num = aw96103->aw_i2c_package.reg_num;
	uint8_t addr_bytes = aw96103->aw_i2c_package.addr_bytes;
	uint8_t msg_idx = 0;
	uint8_t msg_cnt = 0;

	/*
	* step 1 : according to addr_bytes assemble first_addr.
	* step 2 : initialize msg[0] including first_addr transfer to client.
	* step 3 : wait for client return reg_data.
	*/
	for (msg_idx = 0; msg_idx < addr_bytes; msg_idx++) {
		w_buf[msg_idx] = aw96103->aw_i2c_package.init_addr[msg_idx];
		pr_info("%s: w_buf_addr[%d] = 0x%02x\n",
					__func__, msg_idx, w_buf[msg_idx]);
	}
	msg[0].addr = i2c->addr;
	msg[0].flags = AW96103_I2C_WR;
	msg[0].len = msg_idx;
	msg[0].buf = (uint8_t *)w_buf;

	/*
	* recieve client to msg[1].buf.
	*/
	msg_cnt = data_bytes * reg_num;
	msg[1].addr = i2c->addr;
	msg[1].flags = AW96103_I2C_RD;
	msg[1].len = msg_cnt;
	msg[1].buf = (uint8_t *)buf;

	ret = i2c_transfer(i2c->adapter, msg, 2);
	for (msg_idx = 0; msg_idx < msg_cnt; msg_idx++) {
		reg_data[msg_idx] = buf[msg_idx];
		pr_info("%s: buf = 0x%02x\n", __func__, buf[msg_idx]);
	}

	if (ret < 0)
		pr_info("%s: i2c write error %d\n", __func__, ret);

	return ret;
}

static void
aw96103_addrblock_load(struct device *dev, const char *buf)
{
	uint32_t addrbuf[4] = { 0 };
	uint8_t temp_buf[2] = { 0 };
	uint32_t i = 0;
	struct aw96103 *aw96103 = dev_get_drvdata(dev);
	uint8_t addr_bytes = aw96103->aw_i2c_package.addr_bytes;
	uint8_t reg_num = aw96103->aw_i2c_package.reg_num;

	for (i = 0; i < addr_bytes; i++) {
		if (reg_num < attr_buf0[1]) {
			temp_buf[0] = buf[attr_buf0[0] + i * 5];
			temp_buf[1] = buf[attr_buf0[0] + i * 5 + 1];
		} else if (reg_num >= attr_buf0[1] && reg_num < attr_buf0[3]) {
			temp_buf[0] = buf[attr_buf0[2] + i * 5];
			temp_buf[1] = buf[attr_buf0[2] + i * 5 + 1];
		} else if (reg_num >= attr_buf0[3] && reg_num < attr_buf0[5]) {
			temp_buf[0] = buf[attr_buf0[4] + i * 5];
			temp_buf[1] = buf[attr_buf0[4] + i * 5 + 1];
		}
		if (sscanf(temp_buf, "%02x", &addrbuf[i]) == 1)
			aw96103->aw_i2c_package.init_addr[i] =
							(uint8_t)addrbuf[i];
	}
}

/******************************************************
 *
 *the document of storage_spedata
 *
 ******************************************************/
static int32_t aw96103_filedata_deal(struct aw96103 *aw96103)
{
	struct file *fp = NULL;
	mm_segment_t fs;
	int8_t *buf;
	int8_t temp_buf[8] = { 0 };
	uint8_t i = 0;
	uint8_t j = 0;
	int32_t ret;
	uint32_t nv_flag = 0;

	pr_info("%s: enter, cali_node = %d\n", __func__, aw96103->node);

	fp = filp_open(AWINIC_CALI_FILE, O_RDWR | O_CREAT, 0644);

	if (IS_ERR(fp)) {
		pr_err("%s: open failed!\n", __func__);
		return -EINVAL;
	}

	fs = get_fs();
	set_fs(KERNEL_DS);
	buf = (char *)kzalloc(CALI_FILE_MAX_SIZE, GFP_KERNEL);
	if (!buf) {
		pr_err("%s: malloc failed!\n", __func__);
		filp_close(fp, NULL);
		set_fs(fs);
		return -EINVAL;
	}
	ret = vfs_read(fp, buf, CALI_FILE_MAX_SIZE, &(fp->f_pos));
	if (ret < 0) {
		pr_info("%s: read failed\n", __func__);
		return ret;
	} else if (ret == 0) {
		aw96103->nvspe_data[i] = 0;
	} else {
		if (aw96103->node == AW_CALI_NORM_MODE) {
			return 0;
		} else {
			for (i = 0; i < 8; i++) {
				for (j = 0; j < 8; j++)
					temp_buf[j] = buf[8 * i + j];

				if (sscanf(temp_buf, "%08x",
						&aw96103->nvspe_data[i]) == 1)
					pr_info("%s: nv_spe_data[%d]=0x%08x\n",
						__func__, i,
						aw96103->nvspe_data[i]);
			}
		}
	}

	set_fs(fs);

	filp_close(fp, NULL);
	kfree(buf);
	/* nvspe_datas come from nv*/

	for (i = 0; i < 8; i++) {
		nv_flag |= aw96103->nvspe_data[i];
		if (nv_flag != 0)
			break;
	}

	if (aw96103->node == AW_CALI_NORM_MODE) {
		if (nv_flag == 0) {
			aw96103->cali_flag = AW_CALI;
			pr_info("%s: the chip need to cali! nv_flag = 0x%08x\n",
							__func__, nv_flag);
		} else {
			aw96103->cali_flag = AW_NO_CALI;
			pr_info("%s: chip not need to cali! nv_flag = 0x%08x\n",
							__func__, nv_flag);
		}
	}

	return 0;
}

static int32_t aw96103_store_spedata_to_file(struct aw96103 *aw96103, char *buf)
{
	struct file *fp = NULL;
	loff_t pos = 0;
	mm_segment_t fs;

	pr_info("%s: buf = %s\n", __func__, buf);

	fp = filp_open(AWINIC_CALI_FILE, O_RDWR | O_CREAT, 0644);
	if (IS_ERR(fp)) {
		pr_err("%s: open failed!\n", __func__);
		return -EINVAL;
	}

	fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_write(fp, buf, strlen(buf), &pos);

	set_fs(fs);

	pr_info("%s: write successfully!\n", __func__);

	filp_close(fp, NULL);
	return 0;
}

/******************************************************
 *
 *configuration of special reg
 *
 ******************************************************/
static void aw96103_get_calidata(struct aw96103 *aw96103)
{
	uint8_t i = 0;
	uint32_t buf_size = 0;
	int32_t ret;
	uint8_t temp_buf[9] = { 0 };
	uint8_t buf[CALI_FILE_MAX_SIZE] = { 0 };

	pr_info("%s enter\n", __func__);

	/*class 1 special reg*/
	for (i = 0; i < 6; i++) {
		aw96103_i2c_read(aw96103, REG_AFECFG1_CH0 +
				i * AW_CL1SPE_CALI_OS, &aw96103->spedata[i]);
		pr_info("%s: specialdata[%d]=0x%08x\n", __func__, i,
							aw96103->spedata[i]);
	}
	/*class 2 special reg*/
	for (; i < 8; i++) {
		aw96103_i2c_read(aw96103, REG_REFACFG +
						(i - 6) * AW_CL2SPE_CALI_OS,
						&aw96103->spedata[i]);
		pr_info("%s: channel number = 0x%08x\n", __func__,
						aw96103->spedata[i]);
	}
	/* spedatas come from register*/

	/* write spedatas to nv */
	for (i = 0; i < 8; i++) {
		snprintf(temp_buf, sizeof(temp_buf), "%08x",
							aw96103->spedata[i]);
		memcpy(buf + buf_size, temp_buf, strlen(temp_buf));
		buf_size = strlen(buf);
	}
	ret = aw96103_store_spedata_to_file(aw96103, buf);
	if (ret < 0) {
		pr_info("%s: store spedata failed\n", __func__);
		return;
	}

	pr_info("%s: successfully write_spereg_to_file\n", __func__);
}

static void aw96103_class1_reg(struct aw96103 *aw96103)
{
	int32_t i = 0;
	uint32_t reg_val;

	pr_info("%s enter\n", __func__);

	for (i = 0; i < 6; i++) {
		reg_val = (aw96103->nvspe_data[i] >> 16) & 0x0000ffff;
		aw96103_i2c_write_bits(aw96103, REG_INITPROX0_CH0 +
				i * AW_CL1SPE_DEAL_OS, ~(0xffff), reg_val);
	}
}

static void aw96103_class2_reg(struct aw96103 *aw96103)
{
	int32_t i = 0;
	uint32_t reg_val = 0;
	uint32_t ret = 0;

	pr_info("%s enter\n", __func__);

	for (i = 6; i < 8; i++) {
		ret = aw96103->nvspe_data[i] & 0x07;
		switch (ret) {
		case 0x00:
			aw96103_i2c_read(aw96103, REG_VALID_CH0, &reg_val);
			break;
		case 0x01:
			aw96103_i2c_read(aw96103, REG_VALID_CH1, &reg_val);
			break;
		case 0x02:
			aw96103_i2c_read(aw96103, REG_VALID_CH2, &reg_val);
			break;
		case 0x03:
			aw96103_i2c_read(aw96103, REG_VALID_CH3, &reg_val);
			break;
		case 0x04:
			aw96103_i2c_read(aw96103, REG_VALID_CH4, &reg_val);
			break;
		case 0x05:
			aw96103_i2c_read(aw96103, REG_VALID_CH5, &reg_val);
			break;
		default:
			return;
		}

		reg_val = ((reg_val >> 6) & 0x03fffff0) |
					(aw96103->nvspe_data[i] & 0xfc00000f);
		aw96103_i2c_write(aw96103, REG_REFACFG +
					(i - 6) * AW_CL2SPE_DEAL_OS, reg_val);
	}
}

static void aw96103_spereg_deal(struct aw96103 *aw96103)
{
	pr_info("%s enter!\n", __func__);

	aw96103_class1_reg(aw96103);
	aw96103_class2_reg(aw96103);
}

static void aw96103_datablock_load(struct device *dev, const char *buf)
{
	uint32_t i = 0;
	uint8_t reg_data[220] = { 0 };
	uint32_t databuf[220] = { 0 };
	uint8_t temp_buf[2] = { 0 };
	struct aw96103 *aw96103 = dev_get_drvdata(dev);
	uint8_t addr_bytes = aw96103->aw_i2c_package.addr_bytes;
	uint8_t data_bytes = aw96103->aw_i2c_package.data_bytes;
	uint8_t reg_num = aw96103->aw_i2c_package.reg_num;

	for (i = 0; i < data_bytes * reg_num; i++) {
		if (reg_num < attr_buf0[1]) {
			temp_buf[0] = buf[attr_buf0[0] + (addr_bytes + i) * 5];
			temp_buf[1] =
				buf[attr_buf0[0] + (addr_bytes + i) * 5 + 1];
		} else if (reg_num >= attr_buf0[1] && reg_num < attr_buf0[3]) {
			temp_buf[0] = buf[attr_buf0[2] + (addr_bytes + i) * 5];
			temp_buf[1] =
				buf[attr_buf0[2] + (addr_bytes + i) * 5 + 1];
		} else if (reg_num >= attr_buf0[3] && reg_num < attr_buf0[5]) {
			temp_buf[0] = buf[attr_buf0[4] + (addr_bytes + i) * 5];
			temp_buf[1] =
				buf[attr_buf0[4] + (addr_bytes + i) * 5 + 1];
		}
		sscanf(temp_buf, "%02x", &databuf[i]);
		reg_data[i] = (uint8_t)databuf[i];
	}
	aw96103->aw_i2c_package.p_reg_data = reg_data;
	i2c_write_seq(aw96103);
}

static void aw96103_channel_scan_start(struct aw96103 *aw96103)
{
	uint32_t reg_data;
	int32_t ret;
	uint32_t temp_time = AW96103_SCAN_DEFAULT_TIME;

	pr_info("%s: enter\n", __func__);

	if (aw96103->pwprox_dete == true) {
		ret = aw96103_filedata_deal(aw96103);
		if ((aw96103->cali_flag == AW_NO_CALI) && ret >= 0)
			aw96103_spereg_deal(aw96103);
	} else {
		aw96103->cali_flag = AW_NO_CALI;
	}

	aw96103_i2c_write(aw96103, REG_HOSTIRQEN, 0);
	aw96103_i2c_write(aw96103, REG_CMD, 0x0001);
	while ((temp_time)--) {
		aw96103_i2c_read(aw96103, REG_HOSTIRQSRC, &reg_data);
		reg_data = (reg_data >> 4) & 0x01;
		if (reg_data == 1) {
			pr_info("%s: time = %d\n", __func__, temp_time);
			if ((aw96103->cali_flag == AW_CALI) && ret >= 0)
				aw96103_get_calidata(aw96103);
			break;
			msleep(1);
		}
	}
	aw96103_i2c_write(aw96103, REG_HOSTIRQEN, aw96103->hostirqen);
}

static void aw96103_bin_valid_loaded(struct aw96103 *aw96103,
						struct aw_bin *aw_bin_data_s)
{
	uint32_t i;
	uint16_t reg_addr;
	uint32_t reg_data;
	uint32_t start_addr = aw_bin_data_s->header_info[0].valid_data_addr;

	for (i = 0; i < aw_bin_data_s->header_info[0].valid_data_len;
						i += 6, start_addr += 6) {
		reg_addr = (aw_bin_data_s->info.data[start_addr]) |
				aw_bin_data_s->info.data[start_addr + 1] << 8;
		reg_data = aw_bin_data_s->info.data[start_addr + 2] |
			(aw_bin_data_s->info.data[start_addr + 3] << 8) |
			(aw_bin_data_s->info.data[start_addr + 4] << 16) |
			(aw_bin_data_s->info.data[start_addr + 5] << 24);
		if ((reg_addr == REG_EEDA0) || (reg_addr == REG_EEDA1))
			continue;
		aw96103_i2c_write(aw96103, reg_addr, reg_data);
		msleep(1);
		if (reg_addr == REG_HOSTIRQEN)
			aw96103->hostirqen = reg_data;
		pr_info("%s :reg_addr = 0x%04x, reg_data = 0x%08x\n", __func__,
							reg_addr, reg_data);
	}
	pr_info("%s bin writen completely: \n", __func__);

	aw96103_channel_scan_start(aw96103);
}

/***************************************************************************
* para loaded
****************************************************************************/
static int32_t aw96103_para_loaded(struct aw96103 *aw96103)
{
	int32_t i = 0;
	int32_t len =
		sizeof(aw96103_reg_default)/sizeof(aw96103_reg_default[0]);

	pr_info("%s: start to download para!\n", __func__);
	for (i = 0; i < len; i = i + 2) {
		aw96103_i2c_write(aw96103, (uint16_t)aw96103_reg_default[i],
						aw96103_reg_default[i+1]);
		if (aw96103_reg_default[i] == REG_HOSTIRQEN)
			aw96103->hostirqen = aw96103_reg_default[i+1];
		pr_info("%s: reg_addr = 0x%04x, reg_data = 0x%08x\n",
						__func__,
						aw96103_reg_default[i],
						aw96103_reg_default[i+1]);
	}
	pr_info("%s para writen completely: \n", __func__);

	aw96103_channel_scan_start(aw96103);

	return 0;
}

static void aw96103_cfg_all_loaded(const struct firmware *cont, void *context)
{
	int32_t ret;
	struct aw_bin *aw_bin;
	struct aw96103 *aw96103 = context;

	pr_info("%s enter\n", __func__);

	if (!cont) {
		pr_info("%s: %s download failed\n", __func__, aw96103_cfg_name);
		release_firmware(cont);
		return;
	} else {
		pr_info("%s download successfully\n", aw96103_cfg_name);
	}

	aw_bin = kzalloc(cont->size + sizeof(struct aw_bin), GFP_KERNEL);
	if (!aw_bin) {
		kfree(aw_bin);
		release_firmware(cont);
		pr_err("%s: failed to allcating memory!\n", __func__);
		return;
	}
	aw_bin->info.len = cont->size;
	memcpy(aw_bin->info.data, cont->data, cont->size);
	ret = aw_parsing_bin_file0(aw_bin);
	if (ret < 0) {
		pr_info("%s:aw96103 parse bin fail! ret = %d\n", __func__, ret);
		kfree(aw_bin);
		release_firmware(cont);
		return;
	}

	ret = strcmp(aw96103->chip_name, aw_bin->header_info[0].chip_type);
	if (ret != 0) {
		pr_info("%s:chip name(%s) incompatible with bin chip(%s)\n",
					__func__, aw96103->chip_name,
					aw_bin->header_info[0].chip_type);
		kfree(aw_bin);
		release_firmware(cont);
		return;
	}
	aw96103_bin_valid_loaded(aw96103, aw_bin);
	kfree(aw_bin);
	release_firmware(cont);
}

static int32_t aw96103_cfg_update(struct aw96103 *aw96103)
{
	pr_info("%s: enter\n", __func__);

	if (aw96103->firmware_flag == true)
		return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
							aw96103_cfg_name,
							aw96103->dev,
							GFP_KERNEL,
							aw96103,
							aw96103_cfg_all_loaded);
	else
		aw96103_para_loaded(aw96103);

	return AW_SAR_SUCCESS;
}

static void aw96103_cfg_work_routine(struct work_struct *work)
{
	struct aw96103
		*aw96103 = container_of(work, struct aw96103, cfg_work.work);

	pr_info("%s: enter\n", __func__);

	aw96103_cfg_update(aw96103);
}

static int32_t aw96103_sar_cfg_init(struct aw96103 *aw96103, int32_t flag)
{
	uint32_t cfg_timer_val = 0;

	pr_info("%s: enter\n", __func__);

	if (flag == AW_CFG_LOADED) {
		cfg_timer_val = 20;
		aw96103->node = AW_CALI_NODE_MODE;
	} else if (flag == AW_CFG_UNLOAD) {
		cfg_timer_val = 5000;
		aw96103->node = AW_CALI_NORM_MODE;
	} else {
		return -AW_CFG_LOAD_TIME_FAILED;
	}

	pr_info("%s: cali_node = %d\n", __func__, aw96103->node);

	INIT_DELAYED_WORK(&aw96103->cfg_work, aw96103_cfg_work_routine);
	schedule_delayed_work(&aw96103->cfg_work,
					msecs_to_jiffies(cfg_timer_val));

	return AW_SAR_SUCCESS;
}

/*****************************************************
 *
 * first irq clear
 *
 *****************************************************/
static int32_t aw96103_init_irq_handle(struct aw96103 *aw96103)
{
	uint8_t cnt = 20;
	uint32_t reg_data;
	uint32_t trim0 = 0;
	uint32_t trim1 = 0;

	pr_info("%s enter\n", __func__);
	while (cnt--) {
		aw96103_i2c_read(aw96103, REG_HOSTIRQSRC, &reg_data);
		aw96103->first_irq_flag = reg_data & 0x01;
		if (aw96103->first_irq_flag == 1) {
			aw96103_i2c_read(aw96103, REG_EEDA0, &trim0);
			aw96103_i2c_read(aw96103, REG_EEDA1, &trim1);
			if ((trim0 + trim1) == 0) {
				pr_info("%s: aw96103 trim error\n", __func__);
				return -AW_TRIM_ERROR;
			}
			pr_info("%s: cnt = %d\n", __func__, cnt);
			return AW_SAR_SUCCESS;
		}
		msleep(1);
	}
	pr_err("%s: hardware has trouble!\n", __func__);

	return -AW_IRQIO_FAILED;
}

/*****************************************************
 *
 * software reset
 *
 *****************************************************/
static void aw96103_sw_reset(struct aw96103 *aw96103)
{
	pr_info("%s: enter\n", __func__);

	aw96103_i2c_write(aw96103, REG_HOSTCTRL2, 0);
}

/******************************************************
 *
 * sys group attribute
 *
 ******************************************************/
static ssize_t aw96103_set_reg_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct aw96103 *aw96103 = dev_get_drvdata(dev);
	uint32_t databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2)
		aw96103_i2c_write(aw96103, (uint16_t)databuf[0],
							(uint32_t)databuf[1]);

	return count;
}

static ssize_t aw96103_get_reg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw96103 *aw96103 = dev_get_drvdata(dev);
	ssize_t len = 0;
	uint32_t i = 0;
	uint32_t reg_val = 0;
	uint32_t reg_num = 0;

	reg_num = ARRAY_SIZE(aw96103_reg_access);
	for (i = 0; i < reg_num; i++)
		if (aw96103_reg_access[i].rw & REG_RD_ACCESS) {
			aw96103_i2c_read(aw96103, aw96103_reg_access[i].reg,
								&reg_val);
			len += snprintf(buf + len, PAGE_SIZE - len,
						"reg:0x%04x=0x%08x\n",
						aw96103_reg_access[i].reg,
						reg_val);
		}

	return len;
}

static ssize_t aw96103_valid_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw96103 *aw96103 = dev_get_drvdata(dev);
	ssize_t len = 0;
	uint8_t i = 0;
	int32_t reg_val = 0;

	for (i = 0; i < AW_SAR_CAHNNEL_MAX; i++) {
		aw96103_i2c_read(aw96103, REG_VALID_CH0 + i * 4, &reg_val);
		reg_val /= 1024;
		len += snprintf(buf+len, PAGE_SIZE-len, "VALID_CH%d = %d\n", i,
								reg_val);
	}

	return len;
}

static ssize_t aw96103_baseline_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw96103 *aw96103 = dev_get_drvdata(dev);
	ssize_t len = 0;
	uint8_t i = 0;
	int32_t reg_val = 0;

	for (i = 0; i < AW_SAR_CAHNNEL_MAX; i++) {
		aw96103_i2c_read(aw96103, REG_BASELINE_CH0 + i * 4, &reg_val);
		reg_val /= 1024;
		len += snprintf(buf+len, PAGE_SIZE-len, "BASELINE_CH%d = %d\n",
								i, reg_val);
	}

	return len;
}

static ssize_t aw96103_diff_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw96103 *aw96103 = dev_get_drvdata(dev);
	ssize_t len = 0;
	uint8_t i = 0;
	int32_t reg_val = 0;

	for (i = 0; i < AW_SAR_CAHNNEL_MAX; i++) {
		aw96103_i2c_read(aw96103, REG_DIFF_CH0 + i * 4, &reg_val);
		reg_val /= 1024;
		len += snprintf(buf+len, PAGE_SIZE-len, "DIFF_CH%d = %d\n", i,
								reg_val);
	}

	return len;
}

static ssize_t aw96103_raw_data_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw96103 *aw96103 = dev_get_drvdata(dev);
	ssize_t len = 0;
	uint8_t i = 0;
	int32_t reg_val = 0;

	for (i = 0; i < AW_SAR_CAHNNEL_MAX; i++) {
		aw96103_i2c_read(aw96103, REG_RAW_CH0 + i * 4, &reg_val);
		reg_val /= 1024;
		len += snprintf(buf+len, PAGE_SIZE-len, "RAW_DATA_CH%d = %d\n",
								i, reg_val);
	}

	return len;
}

static ssize_t aw96103_awrw_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw96103 *aw96103 = dev_get_drvdata(dev);
	uint8_t reg_data[228] = { 0 };
	uint8_t i = 0;
	ssize_t len = 0;
	uint8_t reg_num = aw96103->aw_i2c_package.reg_num;
	uint8_t data_bytes = aw96103->aw_i2c_package.data_bytes;

	i2c_read_seq(aw96103, reg_data);
	for (i = 0; i < reg_num * data_bytes - 1; i++)
		len += snprintf(buf + len, PAGE_SIZE - len, "0x%02x,",
								reg_data[i]);
	if (!i)
		i2c_read_seq(aw96103, reg_data);
	len += snprintf(buf + len, PAGE_SIZE - len, "0x%02x\n",
								reg_data[i]);

	return len - 1;
}
/*
static ssize_t aw96103_cali_set(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct aw96103 *aw96103 = dev_get_drvdata(dev);
	uint32_t databuf[1] = { 0 };

	if (sscanf(buf, "%d", &databuf[0]) == 1) {
		if ((databuf[0] == 1) && (aw96103->pwprox_dete == true)) {
			aw96103_sw_reset(aw96103);
			aw96103->cali_flag = AW_CALI;
		} else {
			pr_info("%s:aw_unsupport the pw_prox_dete=%d\n",
						__func__, aw96103->pwprox_dete);
			return count;
		}
		aw96103_sar_cfg_init(aw96103, AW_CFG_LOADED);
	}

	return count;
}
*/
static ssize_t aw96103_cali_set(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	ssize_t ret;
	uint32_t state;
	uint32_t data_en = 0;
	struct aw96103 *aw96103 = dev_get_drvdata(dev);
	ret = kstrtouint(buf, 10, &state);
	if (ret)
	{
		pr_err("fail to set aot cali");
		return ret;
	}
	aw96103_i2c_read(aw96103, REG_SCANCTRL0, &data_en);
	if (state != 0)
		aw96103_i2c_write_bits(aw96103, REG_SCANCTRL0, ~(0x3f << 8),
		(data_en & 0x3f) << 8);
	else
		pr_err("fail to set aot cali");
	return count;
}

//modify start by chen-liang for sar sensor MMI calibration 05/21/2021
static ssize_t aw96103_cali_get(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	uint32_t state;
	uint32_t data_en = 0;
	ssize_t len = 0;
	struct aw96103 *aw96103 = dev_get_drvdata(dev);

	aw96103_i2c_read(aw96103, REG_HOSTIRQSRC, &data_en);
	state = (data_en & 0x08) >> 3;

	len += snprintf(buf+len, PAGE_SIZE-len, "%d\n", state);

	return len;
}
//modify end by chen-liang for sar sensor MMI calibration 05/21/2021

static ssize_t aw96103_awrw_set(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct aw96103 *aw96103 = dev_get_drvdata(dev);
	uint32_t datatype[3] = { 0 };

	if (sscanf(buf, "%d %d %d", &datatype[0], &datatype[1],
							&datatype[2]) == 3) {
		aw96103->aw_i2c_package.addr_bytes = (uint8_t)datatype[0];
		aw96103->aw_i2c_package.data_bytes = (uint8_t)datatype[1];
		aw96103->aw_i2c_package.reg_num = (uint8_t)datatype[2];

		aw96103_addrblock_load(dev, buf);
		if (count > 7 + 5 * aw96103->aw_i2c_package.addr_bytes)
			aw96103_datablock_load(dev, buf);
	}

	return count;
}

static ssize_t aw96103_set_update(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	ssize_t ret;
	uint32_t state;
	int32_t cfg_timer_val = 10;
	struct aw96103 *aw96103 = dev_get_drvdata(dev);

	ret = kstrtouint(buf, 10, &state);
	if (ret) {
		pr_err("%s: fail to change str to int\n", __func__);
		return ret;
	}
	if (state)
		schedule_delayed_work(&aw96103->cfg_work,
					msecs_to_jiffies(cfg_timer_val));

	return count;
}

static DEVICE_ATTR(reg, 0664, aw96103_get_reg_show, aw96103_set_reg_store);
static DEVICE_ATTR(valid, 0664, aw96103_valid_show, NULL);
static DEVICE_ATTR(baseline, 0664, aw96103_baseline_show, NULL);
static DEVICE_ATTR(diff, 0664, aw96103_diff_show, NULL);
static DEVICE_ATTR(raw_data, 0664, aw96103_raw_data_show, NULL);
static DEVICE_ATTR(cali, 0664, aw96103_cali_get, aw96103_cali_set);
static DEVICE_ATTR(awrw, 0664, aw96103_awrw_get, aw96103_awrw_set);
static DEVICE_ATTR(update, 0644, NULL, aw96103_set_update);

static struct attribute *aw96103_sar_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_valid.attr,
	&dev_attr_baseline.attr,
	&dev_attr_diff.attr,
	&dev_attr_raw_data.attr,
	&dev_attr_awrw.attr,
	&dev_attr_cali.attr,
	&dev_attr_update.attr,
	NULL
};

static struct attribute_group aw96103_sar_attribute_group = {
	.attrs = aw96103_sar_attributes
};

static int32_t aw96103_input_sys_init(struct aw96103 *aw96103)
{
	int32_t ret = 0;
	int i = 0;
	pr_info("%s: enter\n", __func__);
	aw96103->aw_pad = pad_event;
	aw96103->input = input_allocate_device();
	if (!(aw96103->input))
		return -AW_INPUT_ALLOCATE_FILED;
	aw96103->aw_pad->input = aw96103->input;
	aw96103->input->name = AW96103_I2C_NAME;
	//Mod start by chen-liang for fix framework can not receive sar sensor input key 2021/03/05
	//	__set_bit(EV_KEY, aw96103->input->evbit);
	//	__set_bit(EV_SYN, aw96103->input->evbit);
	//	__set_bit(KEY_F1, aw96103->input->keybit);
	//	input_set_abs_params(aw96103->input, ABS_DISTANCE, 0, 100, 0, 0);
	for (i = 0; i < AW_CHANNEL_MAX; i++) {
		if(aw96103->channel_func & (1 << i)){
			input_set_capability(aw96103->input, EV_KEY, 234 + 2*i);
			input_set_capability(aw96103->input, EV_KEY, 234 + 2*i +1);
	//Mod end by chen-liang for fix framework can not receive sar sensor input key 2021/03/05
		}
	}
	ret = input_register_device(aw96103->input);
	if (ret) {
		pr_err("%s: failed to register input device: %s\n", __func__,
						dev_name(&aw96103->i2c->dev));
		input_free_device(aw96103->input);
		return -AW_INPUT_REGISTER_FAILED;
	}


	return AW_SAR_SUCCESS;
}

/*****************************************************
*
* irq init
*
*****************************************************/

static void aw96103_irq_handle(struct aw96103 *aw96103)
{
	uint32_t curr_status = 0;
	uint8_t i = 0;
//	pr_info("aw96103_irq_handle enter");

	aw96103_i2c_read(aw96103, REG_STAT0, &curr_status);

//	pr_info("pad = 0x%08x", curr_status);
	for (i = 0; i < AW_CHANNEL_MAX; i++) {
		if(aw96103->channel_func & (1 << i)){
			aw96103->aw_pad[i].curr_state =
				(((uint8_t)(curr_status >> (24 + i)) & 0x1)) |
				(((uint8_t)(curr_status >> (16 + i)) & 0x1) << 1) |
				(((uint8_t)(curr_status >> (8 + i)) & 0x1) << 2) |
				(((uint8_t)(curr_status >> (i)) & 0x1) << 3);
//			pr_err("curr_state[%d] = 0x%x", i,
//						aw96103->aw_pad[i].curr_state);

			if (aw96103->aw_pad[i].curr_state !=
						aw96103->aw_pad[i].last_state) {
				switch (aw96103->aw_pad[i].curr_state) {
				case FAR:
					input_report_key(aw96103->input, 234+ i*2 + 1, 1);
					input_sync(aw96103->input);
					input_report_key(aw96103->input, 234+ i*2 + 1, 0);
					input_sync(aw96103->input);
					break;
				case TRIGGER_TH0:
					input_report_key(aw96103->input, 234 + i*2, 1);
					input_sync(aw96103->input);
					input_report_key(aw96103->input, 234 + i*2, 0);
					input_sync(aw96103->input);
					break;
				case TRIGGER_TH1:

					break;
				case TRIGGER_TH2:
					break;
				case TRIGGER_TH3:

					break;
				default:
					pr_err("error abs distance");
					return;
				}
				//input_sync(aw96103->input);
			}
			aw96103->aw_pad[i].last_state =
					aw96103->aw_pad[i].curr_state;
		}
	}
}


static void aw96103_interrupt_clear(struct aw96103 *aw96103)
{
	uint32_t irq_flag = 0;

	//pr_info("%s enter\n", __func__);

	aw96103_i2c_read(aw96103, REG_HOSTIRQSRC, &aw96103->irq_status);
	irq_flag = (aw96103->irq_status >> 4) & 0x01;
	if (irq_flag == 1) {
		aw96103_i2c_write_bits(aw96103, REG_HOSTIRQEN,
					AW96103_BIT_REG_HOSTIRQEN_MASK, 0);
	}
//	pr_info("IRQSRC = 0x%x", aw96103->irq_status);

#if 0
	if ((aw96103->irq_status & 0x0002) == 0x0002) {
		//pr_info("%s approach status = 0x%x\n", __func__,
							//aw96103->irq_status);
		//Mod start by chen-liang for fix framework can not receive sar sensor input key 2021/03/05
		//input_report_abs(aw96103->input, ABS_DISTANCE, 238);
		input_report_key(aw96103->input, 238, 1);
		input_sync(aw96103->input);
		input_report_key(aw96103->input, 238, 0);
		input_sync(aw96103->input);
	} else if ((aw96103->irq_status & 0x0004) == 0x0004) {
		//pr_info("%s far status = 0x%x\n", __func__,
							//aw96103->irq_status);
		//input_report_abs(aw96103->input, ABS_DISTANCE, 239);
		input_report_key(aw96103->input, 239, 1);
		input_sync(aw96103->input);
		input_report_key(aw96103->input, 239, 0);
		input_sync(aw96103->input);
		//Mod end by chen-liang for fix framework can not receive sar sensor input key 2021/03/05
	} else {
		//pr_info("%s other status = 0x%x\n", __func__,
							//aw96103->irq_status);
		/*submit the subsystem of input*/
		input_report_abs(aw96103->input, ABS_DISTANCE, 0);
	}
	/*the end of report*/
	input_sync(aw96103->input);
#endif
	aw96103_irq_handle(aw96103);
}

static irqreturn_t aw96103_irq(int32_t irq, void *data)
{
	struct aw96103 *aw96103 = data;

	//pr_info("%s enter\n", __func__);

	aw96103_interrupt_clear(aw96103);
	//pr_info("%s exit\n", __func__);

	return IRQ_HANDLED;
}


void aw96103_int_output(struct aw96103 *aw96103, int32_t level)
{
	pr_info("%s enter aw96103 int level:%d\n", __func__, level);
	if (level == 0) {
		if (aw96103->pinctrl.pinctrl) {
			pinctrl_select_state(aw96103->pinctrl.pinctrl,
						aw96103->pinctrl.int_out_low);
		} else {
			pr_info("%s Failed set int pin output low\n", __func__);
		}
	} else if (level == 1){
		if (aw96103->pinctrl.pinctrl) {
			pinctrl_select_state(aw96103->pinctrl.pinctrl,
						aw96103->pinctrl.int_out_high);
		}else {
			pr_info("%s Failed set int pin output high\n", __func__);
		}
	}
}

static int32_t aw96103_pinctrl_init(struct aw96103 *aw96103)
{
	struct aw96103_pinctrl *pinctrl = &aw96103->pinctrl;

	pinctrl->pinctrl = devm_pinctrl_get(aw96103->dev);
	if (IS_ERR_OR_NULL(pinctrl->pinctrl)) {
		pr_info("%s:No pinctrl found\n", __func__);
		pinctrl->pinctrl = NULL;
		return 0;
	}

	pinctrl->default_sta = pinctrl_lookup_state(pinctrl->pinctrl,
								"aw-default");
	if (IS_ERR_OR_NULL(pinctrl->default_sta)) {
		pr_info("%s: Failed get pinctrl state:default state\n",
								__func__);
		goto exit_pinctrl_init;
	}

	pinctrl->int_out_high = pinctrl_lookup_state(pinctrl->pinctrl,
							"aw-int-output-high");
	if (IS_ERR_OR_NULL(pinctrl->int_out_high)) {
		pr_info("%s: Failed get pinctrl state:output_high\n", __func__);
		goto exit_pinctrl_init;
	}

	pinctrl->int_out_low = pinctrl_lookup_state(pinctrl->pinctrl,
							"aw-int-output-low");
	if (IS_ERR_OR_NULL(pinctrl->int_out_low)) {
	pr_info("%s: Failed get pinctrl state:output_low\n", __func__);
		goto exit_pinctrl_init;
	}

	pr_info("%s: Success init pinctrl\n", __func__);
	return 0;
exit_pinctrl_init:
	devm_pinctrl_put(pinctrl->pinctrl);
	pinctrl->pinctrl = NULL;
	return -EINVAL;
}

static void aw96103_pinctrl_deinit(struct aw96103 *aw96103)
{
	if (aw96103->pinctrl.pinctrl)
		devm_pinctrl_put(aw96103->pinctrl.pinctrl);
}

static int32_t aw96103_interrupt_init(struct aw96103 *aw96103)
{
	int32_t irq_flags = 0;
	int32_t ret = 0;

	pr_info("%s enter\n", __func__);

	if (gpio_is_valid(aw96103->irq_gpio)) {
		ret = devm_gpio_request_one(&aw96103->i2c->dev,
							aw96103->irq_gpio,
							GPIOF_DIR_IN,
							"aw96103_irq_gpio");
		if (ret) {
			pr_err("%s: request irq gpio failed, ret = %d\n",
							__func__, ret);
			ret = -AW_IRQIO_FAILED;
		} else {
			/* register irq handler */
			irq_flags = IRQF_TRIGGER_LOW | IRQF_ONESHOT;
			ret = devm_request_threaded_irq(&aw96103->i2c->dev,
						gpio_to_irq(aw96103->irq_gpio),
						NULL, aw96103_irq, irq_flags,
						"aw96103_irq", aw96103);
			if (ret != 0) {
				pr_err("%s: failed to request IRQ %d: %d\n",
						__func__,
						gpio_to_irq(aw96103->irq_gpio),
						ret);
				ret = -AW_IRQ_REQUEST_FAILED;
			} else {
				pr_info("%s: IRQ request successfully!\n",
								__func__);
				ret = AW_SAR_SUCCESS;
			}
		}
	} else {
		pr_err("%s: irq gpio invalid!\n", __func__);
		return -AW_IRQIO_FAILED;
	}
	return ret;
}

/*****************************************************
 *
 * parse dts
 *
 *****************************************************/
static void aw96103_parse_dt(struct device *dev, struct aw96103 *aw96103,
							struct device_node *np)
{
	uint32_t val = 0;

	aw96103->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (aw96103->irq_gpio < 0) {
		aw96103->irq_gpio = -1;
		pr_err("%s: no irq gpio provided.\n", __func__);
		return;
	} else {
		pr_info("%s: irq gpio provided ok.\n", __func__);
	}

	val = of_property_read_string(np, "chip_name", &aw96103->chip_name);
	if (val != 0) {
		aw96103->chip_name = NULL;
		pr_info("%s: failed to find chip name\n", __func__);
	} else {
		pr_info("%s: the chip name is %s detected\n", __func__,
							aw96103->chip_name);
	}

	aw96103->firmware_flag =
			of_property_read_bool(np, "aw96103,using-firmware");
	pr_info("%s firmware_flag = <%d>\n", __func__, aw96103->firmware_flag);

	aw96103->pwprox_dete =
		of_property_read_bool(np, "aw96103,using-pwon-prox-dete");
	pr_info("%s pwprox_dete = <%d>\n", __func__, aw96103->pwprox_dete);

	val = of_property_read_u32(np, "channel-func", &aw96103->channel_func);
	if (val != 0) {
		pr_err("channel func failed!");
		return;
	}
}

/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
static int32_t aw96103_read_chipid(struct aw96103 *aw96103)
{
	int32_t ret = -1;
	uint8_t cnt = 0;
	uint32_t reg_val = 0;

	while (cnt < AW_READ_CHIPID_RETRIES) {
		ret = aw96103_i2c_read(aw96103, REG_CHIP_ID, &reg_val);
		if (ret < 0) {
			pr_err("%s: read CHIP ID failed: %d\n", __func__, ret);
		} else {
			reg_val = reg_val >> 16;
			break;
		}

		cnt++;
		usleep_range(2000, 3000);
	}

	if (reg_val == AW96103_CHIP_ID) {
		pr_info("%s aw96103 detected\n", __func__);
		return AW_SAR_SUCCESS;
	} else {
		pr_info("%s unsupported device,the chipid is (0x%04x)\n",
							__func__, reg_val);
	}

	return -AW_CHIPID_FAILED;
}

static void aw96103_i2c_set(struct i2c_client *i2c, struct aw96103 *aw96103)
{
	pr_info("%s: enter\n", __func__);
	aw96103->dev = &i2c->dev;
	aw96103->i2c = i2c;
	i2c_set_clientdata(i2c, aw96103);
}

static int32_t
aw96103_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct aw96103 *aw96103;
	struct device_node *np = i2c->dev.of_node;
	int32_t ret = 0;

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	aw96103 = devm_kzalloc(&i2c->dev, sizeof(struct aw96103), GFP_KERNEL);
	if (aw96103 == NULL) {
		pr_err("%s:failed to malloc memory for aw96103!\n", __func__);
		ret = -AW_MALLOC_FAILED;
		goto err_malloc;
	}

	aw96103_i2c_set(i2c, aw96103);

	/* aw9610x chip id */
	ret = aw96103_read_chipid(aw96103);
	if (ret != AW_SAR_SUCCESS) {
		pr_err("%s: read chipid failed, ret=%d\n", __func__, ret);
		goto err_chipid;
	}

	aw96103_sw_reset(aw96103);

	ret = aw96103_init_irq_handle(aw96103);
	if (ret != AW_SAR_SUCCESS) {
		pr_err("%s: hardware has trouble!, ret=%d\n", __func__, ret);
		goto err_first_irq;
	}

	aw96103_parse_dt(&i2c->dev, aw96103, np);

	ret = aw96103_pinctrl_init(aw96103);
	if (ret < 0) {
		/* if define pinctrl must define the following state
		 * to let int-pin work normally: default, int_output_high,
		 * int_output_low, int_input
		 */
		pr_err("%s: Failed get wanted pinctrl state\n", __func__);
		goto err_pinctrl;
	}

	aw96103_int_output(aw96103, 1);

	ret = aw96103_interrupt_init(aw96103);
	if (ret == -AW_IRQ_REQUEST_FAILED) {
		pr_err("%s: request irq failed!, ret=%d\n", __func__, ret);
		goto err_requst_irq;
	}

	/* input device */
	ret = aw96103_input_sys_init(aw96103);
	if (ret == -AW_INPUT_ALLOCATE_FILED) {
		pr_err("%s:allocate input failed, ret = %d\n", __func__, ret);
		goto exit_input_alloc_failed;
	} else if (ret == -AW_INPUT_REGISTER_FAILED) {
		pr_err("%s:register input failed, ret = %d\n", __func__, ret);
		goto exit_input_register_device_failed;
	}

	/* attribute */
	ret = sysfs_create_group(&i2c->dev.kobj, &aw96103_sar_attribute_group);
	if (ret < 0) {
		dev_info(&i2c->dev, "%s error creating sysfs attr files\n",
			 __func__);
		goto err_sysfs;
	}

	ret = aw96103_sar_cfg_init(aw96103, AW_CFG_UNLOAD);
	if (ret < 0) {
		pr_info("%s: cfg situation not confirmed!\n", __func__);
		goto err_cfg;
	}

	return AW_SAR_SUCCESS;

err_cfg:
err_sysfs:
	sysfs_remove_group(&i2c->dev.kobj, &aw96103_sar_attribute_group);
	input_unregister_device(aw96103->input);
exit_input_register_device_failed:
	input_free_device(aw96103->input);
exit_input_alloc_failed:
err_requst_irq:
	if (gpio_is_valid(aw96103->irq_gpio))
		devm_gpio_free(&i2c->dev, aw96103->irq_gpio);
err_pinctrl:
err_first_irq:
err_chipid:
err_malloc:
	return ret;
}

static int32_t aw96103_i2c_remove(struct i2c_client *i2c)
{
	struct aw96103 *aw96103 = i2c_get_clientdata(i2c);

	aw96103_pinctrl_deinit(aw96103);
	sysfs_remove_group(&i2c->dev.kobj, &aw96103_sar_attribute_group);
	input_unregister_device(aw96103->input);
	if (gpio_is_valid(aw96103->irq_gpio))
		devm_gpio_free(&i2c->dev, aw96103->irq_gpio);

	return 0;
}

static const struct of_device_id aw96103_dt_match[] = {
	{ .compatible = "awinic,aw96103_sar" },
	{ },
};

static const struct i2c_device_id aw96103_i2c_id[] = {
	{ AW96103_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw96103_i2c_id);

static struct i2c_driver aw96103_i2c_driver = {
	.driver = {
		.name = AW96103_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw96103_dt_match),
	},
	.probe = aw96103_i2c_probe,
	.remove = aw96103_i2c_remove,
	.id_table = aw96103_i2c_id,
};

static int32_t __init aw96103_i2c_init(void)
{
	int32_t ret = 0;

	pr_err("aw96103 driver version %s\n", AW96103_DRIVER_VERSION);

	ret = i2c_add_driver(&aw96103_i2c_driver);
	if (ret) {
		pr_err("fail to add aw96103 device into i2c\n");
		return ret;
	}

	return 0;
}

late_initcall(aw96103_i2c_init);
static void __exit aw96103_i2c_exit(void)
{
	i2c_del_driver(&aw96103_i2c_driver);
}
module_exit(aw96103_i2c_exit);
MODULE_DESCRIPTION("AW96103 SAR Driver");

MODULE_LICENSE("GPL v2");
