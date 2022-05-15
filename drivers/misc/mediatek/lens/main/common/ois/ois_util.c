/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include "ois_core.h"

int ois_i2c_write(struct i2c_client *client, u16 addr, u16 data)
{
	struct i2c_msg msg[1]  = {0,};
	char           buf[4]  = {0,};
	int            ret     = 0;

	buf[0] = (char)(addr >> 8);
	buf[1] = (char)(addr & 0xFF);
	buf[2] = (char)(data >> 8);
	buf[3] = (char)(data & 0xFF);

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = ARRAY_SIZE(buf);
	msg[0].buf = buf;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (1 != ret) {
		LOG_OIS_ERR("ois i2c read fail(%d)", ret);
	} else {
		ret = 0;
	}

	return ret;

}

int ois_i2c_write_block(struct i2c_client *client, u16 addr,
		const u8 *data, size_t size)
{
	struct i2c_msg msg[2];
	int            ret           = 0;
	u8             buf[514]      = {0,};
	u16            max_write_size = ARRAY_SIZE(buf) - 2;

	buf[0] = (addr & 0xFF00) >> 8;
	buf[1] = (addr & 0x00FF);

	if (size > max_write_size) {
		LOG_OIS_ERR("write out of buffer(exp=%d max=%d)", size, max_write_size);
		return -1;
	}

	memcpy((void *)(&buf[2]), (void *)data, size);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = (size + 2);
	msg[0].buf = buf;

	ret = i2c_transfer(client->adapter, msg, 1);
	if (1 != ret) {
		LOG_OIS_ERR("ois i2c write block fail(%d)", ret);
	} else
		ret = 0;

	return ret;
}

int ois_i2c_read(struct i2c_client *client, u16 addr, u16 *data)
{
	int            ret          = 0;
	struct i2c_msg msg[2];
	u8             addr_buf[2]  = {0,};
	u8             read_buf[2]  = {0,};

	addr_buf[0] = (char)(addr >> 8);
	addr_buf[1] = (char)(addr & 0xFF);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = addr_buf;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = ARRAY_SIZE(read_buf);
	msg[1].buf = read_buf;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ARRAY_SIZE(msg) != ret) {
		LOG_OIS_ERR("ois i2c read fail(%d)", ret);
	} else {
		ret = 0;
	}

	*data = (read_buf[0] << 8) | read_buf[1];

	return ret;
}

int ois_i2c_read_block(struct i2c_client *client,
		u16 addr, u8 *data, size_t size)
{
	int            ret = 0;
	struct i2c_msg msg[2]        = {0,};
	u8             readbuf[512]  = {0,};
	u8             addrbuf[2]    = {0,};
	u16            max_read_size = ARRAY_SIZE(readbuf);

	if (size > max_read_size) {
		LOG_OIS_ERR("read out of buffer(exp=%d max=%d)", size, max_read_size);
		return -1;
	}

	addrbuf[0] = (addr & 0xff00) >> 8;
	addrbuf[1] = (addr & 0xff);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = addrbuf;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = size;
	msg[1].buf = readbuf;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ARRAY_SIZE(msg) != ret) {
		LOG_OIS_ERR("i2c read failed(%d)", ret);
	} else {
		ret = 0;
	}

	memcpy(data, readbuf, size);

	return ret;
}

