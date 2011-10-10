/* drivers/misc/tmp101.c
 *
 * Copyright (C) 2011 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>

MODULE_LICENSE("Dual BSD/GPL");


struct tmp101_data {
	/* I2C client/device and platform specific data (BSP and GPIO
	 * selection)
	 */
	struct i2c_client *i2c_client;
};

#define TMP101_TEMPERATURE_REG_ADDR 0
#define TMP101_CONFIG_REG_ADDR      1
#define TMP101_T_HIGH_REG_ADDR      2
#define TMP101_T_LOW_REG_ADDR       3

#define TMP101_CONFIG_SHUTDOWN_MODE_BIT   (1 << 0)
#define TMP101_CONFIG_THERMOSTAT_MODE_BIT (1 << 1)
#define   TMP101_CONFIG_THERMOSTAT_MODE_COMPARATOR (0 << 1)
#define   TMP101_CONFIG_THERMOSTAT_MODE_INTERRUPT  (1 << 1)
#define TMP101_CONFIG_POLARITY_BIT        (1 << 2)
#define   TMP101_CONFIG_POLARITY_LOW      (0 << 2)
#define   TMP101_CONFIG_POLARITY_HIGH     (1 << 2)
#define TMP101_CONFIG_FAULT_QUEUE_0_BIT   (1 << 3)
#define TMP101_CONFIG_FAULT_QUEUE_1_BIT   (1 << 4)
#define TMP101_CONFIG_RESOLUTION_0_BIT    (1 << 5)
#define TMP101_CONFIG_RESOLUTION_1_BIT    (1 << 6)
#define TMP101_CONFIG_OS_ALERT_BIT        (1 << 7)

static int tmp101_read_temp(struct i2c_client *client, int* milli_celcius)
{
	int rc;
	u8 temp[2];

	/* set config to shutdown mode for now and request an initial
	 * temperature reading.
	 */
	rc = i2c_smbus_write_byte_data(client, TMP101_CONFIG_REG_ADDR,
				       TMP101_CONFIG_SHUTDOWN_MODE_BIT |
				       TMP101_CONFIG_OS_ALERT_BIT);
	if (rc) {
		pr_err("%s: i2c_smbus_write_byte_data() returned err %d\n",
		       __func__, rc);
		return -ENODEV;
	}

	/* at default low resultion of 9 bits (0.5 degrees C), typical
	 * conversion time according to the datasheet is 40ms
	 */
	msleep(40);

	/* the temperature is in two registers.  the upper first byte
	 * is the whole number, from -128 to 127.  The upper four
	 * bits of the second byte holds the fractional component, from
	 * 0 to .9375, in multiples of .0625.
	 */
	rc = i2c_smbus_read_i2c_block_data(client, TMP101_TEMPERATURE_REG_ADDR,
					   sizeof(temp), temp);
	if (rc != sizeof(temp)) {
		pr_err("%s: i2c_smbus_read_i2c_block_data() returned err %d\n",
		       __func__, rc);
		return -ENODEV;
	}
	pr_debug("%s: temp is %d degrees celsius, raw = [%x,%x]\n",
		__func__, temp[0], temp[0], temp[1]);

	*milli_celcius = ((((((int)((s8)temp[0])) << 4) |
					(temp[1] >> 4)) * 125) >> 1);

	return 0;
}

static ssize_t tmp101_temperature_show(
		struct device *d,
		struct device_attribute *attr,
		char *buf)
{
	struct i2c_client* client;
	int milli_celcius;
	client = container_of(d, struct i2c_client, dev);

	if (tmp101_read_temp(client, &milli_celcius))
		return snprintf(buf, PAGE_SIZE, "%s\n", "<read error>");
	else
		return snprintf(buf, PAGE_SIZE, "%d\n", milli_celcius);
}

static DEVICE_ATTR(temperature, S_IRUGO, tmp101_temperature_show, NULL);

static int tmp101_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct tmp101_data *tmp101 = NULL;
	int rc, milli_celcius;

	pr_info("%s:\n", __func__);

	rc = tmp101_read_temp(client, &milli_celcius);
	if (rc)
		goto bailout;

	tmp101 = kzalloc(sizeof(struct tmp101_data), GFP_KERNEL);
	if (!tmp101) {
		pr_err("%s: kzalloc failed for tmp101_data\n", __func__);
		rc = -ENOMEM;
		goto bailout;
	}

	/* sysfs entry to provide user space control to set deepcolor mode */
	rc = device_create_file(&client->dev, &dev_attr_temperature);
	if (rc) {
		pr_err("%s: device_create_file failed for tmp101 (rc %d)\n",
				__func__, rc);
		goto bailout;
	}

	i2c_set_clientdata(client, tmp101);
	tmp101->i2c_client = client;

	return 0;

bailout:
	kfree(tmp101);
	return rc;
}


static int __devexit tmp101_remove(struct i2c_client *client)
{
	struct tmp101_data *tmp101 = i2c_get_clientdata(client);
	i2c_set_clientdata(client, NULL);
	kfree(tmp101);
	return 0;
}

static struct i2c_device_id tmp101_idtable[] = {
	{ "tmp101", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tmp101_idtable);

static struct i2c_driver tmp101_driver = {
	.driver = {
		.name = "tmp101",
	},

	.probe = tmp101_probe,
	.remove = __devexit_p(tmp101_remove),
	.id_table = tmp101_idtable,

	/* TODO implement these optional power management routines. */
	.shutdown = NULL,
	.suspend = NULL,
	.resume = NULL,
};

static int tmp101_init(void)
{
	return i2c_add_driver(&tmp101_driver);
}

static void tmp101_exit(void)
{
	i2c_del_driver(&tmp101_driver);
}

module_init(tmp101_init);
module_exit(tmp101_exit);
