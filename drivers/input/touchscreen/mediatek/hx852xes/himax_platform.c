/* Himax Android Driver Sample Code for Himax chipset
*
* Copyright (C) 2014 Himax Corporation.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

#include "himax_platform.h"

#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)
#define D(x...) printk("[HXTP] " x)
#define I(x...) printk("[HXTP] " x)
#define W(x...) printk("[HXTP][WARNING] " x)
#define E(x...) printk("[HXTP][ERROR] " x)
#endif

int irq_enable_count = 0;
struct himax_i2c_platform_data *private_pdata; //20160517 add

DEFINE_MUTEX(hx_wr_access);

#ifdef MTK_DMA
u8 *gpDMABuf_va = NULL;
u8 *gpDMABuf_pa = NULL;
#endif
/* for CY33 */
/* static int himax_tpd_int_gpio = 10; */
static int himax_tpd_int_gpio = 1; /* ref : CT41_GPIO table_1222.xlsm, EINT1 -> GPIO1 */

int i2c_himax_read(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int retry;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &command,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		}
	};

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 2) == 2)
			break;
		msleep(10);
	}
	if (retry == toRetry) {
		E("%s: i2c_read_block retry over %d\n",
			__func__, toRetry);
		return -EIO;
	}
	return 0;

}

int i2c_himax_write(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int retry/*, loop_i*/;
	uint8_t buf[length + 1];

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	buf[0] = command;
	memcpy(buf+1, data, length);
	
	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		msleep(10);
	}

	if (retry == toRetry) {
		E("%s: i2c_write_block retry over %d\n",
			__func__, toRetry);
		return -EIO;
	}
	return 0;

}

int i2c_himax_read_command(struct i2c_client *client, uint8_t length, uint8_t *data, uint8_t *readlength, uint8_t toRetry)
{
	int retry;
	struct i2c_msg msg[] = {
		{
		.addr = client->addr,
		.flags = I2C_M_RD,
		.len = length,
		.buf = data,
		}
	};

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		msleep(10);
	}
	if (retry == toRetry) {
		E("%s: i2c_read_block retry over %d\n",
		       __func__, toRetry);
		return -EIO;
	}
	return 0;
}

int i2c_himax_write_command(struct i2c_client *client, uint8_t command, uint8_t toRetry)
{
	return i2c_himax_write(client, command, NULL, 0, toRetry);
}

int i2c_himax_master_write(struct i2c_client *client, uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int retry/*, loop_i*/;
	uint8_t buf[length];

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length,
			.buf = buf,
		}
	};

	memcpy(buf, data, length);
	
	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		msleep(10);
	}

	if (retry == toRetry) {
		E("%s: i2c_write_block retry over %d\n",
		       __func__, toRetry);
		return -EIO;
	}
	return 0;
}

void himax_int_enable(int irqnum, int enable, int log_print)
{
	if (enable == 1 && (!atomic_read(&private_pdata->int_state))) {//20160517
		atomic_set(&private_pdata->int_state, 1);//20160517 add
#ifdef CONFIG_OF_TOUCH
		enable_irq(irqnum);
#else
		mt_eint_unmask(irqnum);
#endif
	} else if (enable == 0 && (atomic_read(&private_pdata->int_state))) { //20160517 add
		atomic_set(&private_pdata->int_state, 0);//20160517 add
#ifdef CONFIG_OF_TOUCH
		disable_irq_nosync(irqnum);
#else
		mt_eint_mask(irqnum);
#endif
	}

	if(log_print)
		I("enable=%d, int_state =%d\n", enable,atomic_read(&private_pdata->int_state));//20160517
}

void himax_rst_gpio_set(int pinnum, uint8_t value)
{
	if(value)
		tpd_gpio_output(TP_RST_PIN, 1);
	else
		tpd_gpio_output(TP_RST_PIN, 0);			
}

uint8_t himax_int_gpio_read(int pinnum)
{
	return  gpio_get_value(himax_tpd_int_gpio);
}

int himax_gpio_power_config(struct i2c_client *client,struct himax_i2c_platform_data *pdata)
{

	int retval = 0;

	/* 20160517 add */
	private_pdata = pdata;

	/* 20160521 int_state default value */
	atomic_set(&private_pdata->int_state, 1);

	/* new MTK power control API */
	retval = regulator_enable(tpd->reg);
	if (retval != 0)
		I("Failed to enable reg-vgp6: %d\n", retval);

	msleep(10);
	tpd_gpio_output(TP_RST_PIN, 1);
	msleep(20);
	tpd_gpio_output(TP_RST_PIN, 0);
	msleep(20);
	tpd_gpio_output(TP_RST_PIN, 1);
	msleep(20);

	I("%s: himax reset over\n", __func__);
	
	/* set INT mode */
	tpd_gpio_as_int(TP_INT_PIN);

	return 0;
}
