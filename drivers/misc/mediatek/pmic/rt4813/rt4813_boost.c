/*
 *  Copyright (C) 2016 Richtek Technology Corp.
 *  cy_huang <cy_huang@richtek.com>
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/pm.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>

#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <mt-plat/mtk_gpio.h>
#include <mt-plat/mtk_boot_common.h>

static struct i2c_client *rt4813_i2c;
static struct device *rt4813_dev;

extern int get_cei_hw_id(void);

static ssize_t show_boost_ic_ping(struct device *dev, struct device_attribute *attr, char *buf)
{
	s32 ret = 0;

	ret = i2c_smbus_read_byte_data(rt4813_i2c, 0x03);
	pr_err("%s: return = 0x%02X\n", __func__, ret);

	return sprintf(buf, "0x%02X\n", ret);
}

static ssize_t store_boost_ic_ping(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	pr_err("%s: Not Support Write Function\n", __func__);

	return size;
}

static DEVICE_ATTR(Boost_IC_Ping, 0664, show_boost_ic_ping, store_boost_ic_ping);

static ssize_t show_boost_en_pin(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 999;

	ret = mt_get_gpio_out(25);
	pr_err("%s: gpio 25 out = %d\n", __func__, ret);
	ret = mt_get_gpio_out(20);
	pr_err("%s: gpio 20 out = %d\n", __func__, ret);

	return sprintf(buf, "%u\n", ret);
}

static ssize_t store_boost_en_pin(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int ret = 999;
	int val;

	ret = kstrtoint(buf, 10, &val);
	pr_err("%s: val = %d\n", __func__, val);

	ret = mt_set_gpio_out(25, val);
	pr_err("%s: ret 25 = %d\n", __func__, ret);

	return size;
}

static DEVICE_ATTR(Boost_EN_Pin, 0664, show_boost_en_pin, store_boost_en_pin);


static ssize_t store_boost_output_current(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int ret = 999;
	int val;
	s32 reg_value;

	ret = kstrtoint(buf, 10, &val);
	pr_err("%s: val = %d, ret = %d\n", __func__, val, ret);

	reg_value = i2c_smbus_read_byte_data(rt4813_i2c, 0x03);
	pr_err("%s: default value = %02X\n", __func__, reg_value);

	reg_value &= 0xF0;
	reg_value |= val;

	ret = i2c_smbus_write_byte_data(rt4813_i2c, 0x03, reg_value);
	pr_err("%s: reg value = %02X; ret = %d\n", __func__, reg_value, ret);

	return size;
}

static DEVICE_ATTR(Boost_Current, 0664, NULL, store_boost_output_current);

static ssize_t show_enable_boost(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 999;

	ret = mt_set_gpio_out(25, 1);
	pr_err("%s: ret 25 = %d\n", __func__, ret);
	return sprintf(buf, "%u\n", ret);
}

static DEVICE_ATTR(Enable_Boost, 0664, show_enable_boost, NULL);

static ssize_t show_disable_boost(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 999;

	ret = mt_set_gpio_out(25, 0);
	pr_err("%s: ret 25 = %d\n", __func__, ret);
	return sprintf(buf, "%u\n", ret);
}

static DEVICE_ATTR(Disable_Boost, 0664, show_disable_boost, NULL);

static int rt4813_boost_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	s32 ret;
	s32 reg_value;
	int hw_phase = -1;

	rt4813_i2c = i2c;
	rt4813_dev = &i2c->dev;

	reg_value = i2c_smbus_read_byte_data(rt4813_i2c, 0x03);
	hw_phase = get_cei_hw_id();
	pr_err("%s: default value = %02X, hw_phase = %d\n", __func__, reg_value, hw_phase);

	reg_value &= 0xF0;

	if (hw_phase <= 2) { /* hw phase before PVT */
		reg_value &= 0x0F;
		reg_value |= 0x90;
		pr_err("%s: reg_value for rt4813 soft start = 0x%02X\n", __func__, reg_value);
	}

	if (get_boot_mode() == POWERBANK) {
		pr_err("%s: boot mode == powerbank, set 1.5A\n", __func__);
		reg_value |= 0x09;
	} else {
		pr_err("%s: boot mode != powerbank, set 1A\n", __func__);
		reg_value |= 0x0C;
	}

	ret = i2c_smbus_write_byte_data(i2c, 0x03, reg_value);
	pr_err("%s: write value = %02X; ret = %d\n", __func__, reg_value, ret);

	device_create_file(rt4813_dev, &dev_attr_Boost_IC_Ping);
	device_create_file(rt4813_dev, &dev_attr_Boost_EN_Pin);
	device_create_file(rt4813_dev, &dev_attr_Enable_Boost);
	device_create_file(rt4813_dev, &dev_attr_Disable_Boost);
	device_create_file(rt4813_dev, &dev_attr_Boost_Current);

	return 0;
}

static int rt4813_boost_remove(struct i2c_client *i2c)
{
	return 0;
}

static const struct i2c_device_id rt4813_boost_id_table[] = {
	{"rt4813_boost", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, rt4813_boost_id_table);

static const struct of_device_id rt4813_boost_ofid_table[] = {
	{ .compatible = "mediatek,rt4813_boost",},
	{},
};
MODULE_DEVICE_TABLE(of, rt4813_boost_ofid_table);

static struct i2c_driver rt4813_boost = {
	.driver = {
		.name = "rt4813_boost",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rt4813_boost_ofid_table),
	},
	.probe = rt4813_boost_probe,
	.remove = rt4813_boost_remove,
	.id_table = rt4813_boost_id_table,
};

module_i2c_driver(rt4813_boost);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Michael_Yang <michaelcc_yang@compal.com>");
MODULE_DESCRIPTION("Richtek RT4813 Boost");
MODULE_VERSION("1.0.0");
