/* linux/driver/input/misc/vcnl4000.c
 * Copyright (C) 2011 Google, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */
/* #define DEBUG */

#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/vcnl4000.h>
#include <linux/input-polldev.h>

#define ADC_BUFFER_NUM	6

#define VCNL4000_COMMAND_REG_ADDR                         0x80
#define VCNL4000_PRODUCT_ID_REG_ADDR                      0x81
#define VCNL4000_IR_LED_CURRENT_REG_ADDR                  0x83
#define VCNL4000_PROXIMITY_RESULT_HIGH_REG_ADDR           0x87
#define VCNL4000_PROXIMITY_RESULT_LOW_REG_ADDR            0x88
#define VCNL4000_PROXIMITY_MODULATOR_TIMING_ADJ_REG_ADDR  0x8A

/* bits in COMMAND_REG */
#define COMMAND_REG_ALS_DATA_RDY  (1 << 6)
#define COMMAND_REG_PROX_DATA_RDY (1 << 5)
#define COMMAND_REG_ALS_OD        (1 << 4)
#define COMMAND_REG_PROX_OD       (1 << 3)

#define READ_TIMEOUT 100000

enum key_state {
	KEY_STATE_UP,
	KEY_STATE_DOWN,
	KEY_STATE_UNKNOWN,
};

/* driver data */
struct vcnl4000_data {
	struct input_polled_dev *poll_dev;
	struct vcnl4000_platform_data *pdata;
	struct i2c_client *i2c_client;
	enum key_state last_key_state;
};

static void vcnl4000_polled_poll(struct input_polled_dev *dev)
{
	struct input_dev *input = dev->input;
	struct vcnl4000_data *vcnl4000 = dev->private;
	u8 reg_value[2];
	u16 proximity;
	int err;
	enum key_state new_key_state;
	int read_timeout = READ_TIMEOUT;

	/* request a proximity measurement to be taken */
	err = i2c_smbus_read_byte_data(vcnl4000->i2c_client,
				       VCNL4000_COMMAND_REG_ADDR);
	if (err < 0) {
		dev_err(&vcnl4000->i2c_client->dev,
			"Error reading command register\n");
		return;
	}
	reg_value[0] = (err & 0xff) | COMMAND_REG_PROX_OD;
	err = i2c_smbus_write_byte_data(vcnl4000->i2c_client,
					VCNL4000_COMMAND_REG_ADDR,
					reg_value[0]);
	if (err) {
		dev_err(&vcnl4000->i2c_client->dev,
			"Error writing command register\n");
		return;
	}
	reg_value[0] &= ~COMMAND_REG_PROX_DATA_RDY;

	/* from empirical testing, the result is ready very quickly
	 * so just poll for completion.
	 */
	do {
		err = i2c_smbus_read_byte_data(vcnl4000->i2c_client,
					       VCNL4000_COMMAND_REG_ADDR);
		if (err < 0) {
			dev_err(&vcnl4000->i2c_client->dev,
				"Error reading command register\n");
			return;
		}
		reg_value[0] = (err & 0xff);
	} while (((reg_value[0] & COMMAND_REG_PROX_DATA_RDY) == 0) &&
		 (read_timeout-- > 0));
	if (read_timeout <= 0) {
		pr_info("%s: timed out reading proximity result\n", __func__);
		return;
	}

	/* read proximity registers.  first reg is high 8 bits,
	 * second reg is low 8 bits.  read in one i2c transfer.
	 */
	err = i2c_smbus_read_i2c_block_data(vcnl4000->i2c_client,
				    VCNL4000_PROXIMITY_RESULT_HIGH_REG_ADDR,
				    sizeof(reg_value), reg_value);
	if (err != 2) {
		dev_err(&vcnl4000->i2c_client->dev,
			"Error reading proximity result registers\n");
		return;
	}
	proximity = (reg_value[0] << 8) | reg_value[1];

	if (proximity > vcnl4000->pdata->detect_threshold)
		new_key_state = KEY_STATE_DOWN;
	else
		new_key_state = KEY_STATE_UP;

	pr_debug("%s: raw proximity = 0x%x (%d), choosing %s (threshold %d)\n",
		 __func__, proximity, proximity,
		 (new_key_state ? "key_down" : "key_up"),
		 vcnl4000->pdata->detect_threshold);

	if (vcnl4000->last_key_state != new_key_state) {
		pr_info("%s: reporting %s, raw proximity = 0x%x (%d),"
			" threshold %d\n", __func__,
			(new_key_state == KEY_STATE_DOWN) ? "key_down" :
			"key_up", proximity, proximity,
			vcnl4000->pdata->detect_threshold);

		vcnl4000->last_key_state = new_key_state;
		input_event(input, EV_KEY, vcnl4000->pdata->code,
			    new_key_state);
		input_sync(input);
	}
}

static int vcnl4000_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	int error = -ENODEV;
	struct device *dev = &client->dev;
	struct input_polled_dev *poll_dev;
	struct input_dev *input;
	struct vcnl4000_data *vcnl4000;
	struct vcnl4000_platform_data *pdata = client->dev.platform_data;
	u8 product_id;

	if (!pdata || !pdata->poll_interval || !pdata->detect_threshold) {
		dev_err(dev, "missing pdata!\n");
		return error;
	}
	if ((pdata->ir_led_current < 0) || (pdata->ir_led_current > 200)) {
		dev_err(dev,
			"invalid ir_led_current %d, must be between 0-200\n",
			pdata->ir_led_current);
		return error;
	}
	if (pdata->ir_led_current % 10) {
		dev_err(dev,
			"invalid ir_led_current %d, must be a multiple of 10\n",
			pdata->ir_led_current);
		return error;
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "i2c functionality check failed!\n");
		return error;
	}

	/* read product id register as a sanity check of the connection */
	error = i2c_smbus_read_byte_data(client, VCNL4000_PRODUCT_ID_REG_ADDR);
	if (error < 0) {
		dev_err(dev, "failed to read product_id\n");
		return error;
	}
	product_id = error & 0xff;

	pr_info("%s: product_id %d, revision %d\n", __func__,
		((product_id & 0xf0) >> 4), ((product_id) & 0x0f));

	/* setup modulator timing */
	/* 129 is value recommended in data sheet */
	error = i2c_smbus_write_byte_data(client,
			  VCNL4000_PROXIMITY_MODULATOR_TIMING_ADJ_REG_ADDR,
			  129);
	if (error) {
		dev_err(dev,
			"Error writing initial value for timing adj\n");
		return error;
	}

	/* setup ir led current level */
	error = i2c_smbus_read_byte_data(client,
					 VCNL4000_IR_LED_CURRENT_REG_ADDR);
	if (error < 0) {
		dev_err(dev,
			"Error reading ir led current reg\n");
		return error;
	}
	/* value to write is requested mA value divided by 10 */
	pr_info("%s: setting ir led current value = 0x%x (%d mA)\n",
		__func__, pdata->ir_led_current / 10, pdata->ir_led_current);
	error = i2c_smbus_write_byte_data(client,
					  VCNL4000_IR_LED_CURRENT_REG_ADDR,
					  pdata->ir_led_current / 10);
	if (error) {
		dev_err(dev,
			"Error writing ir_led_current\n");
		return error;
	}

	vcnl4000 = kzalloc(sizeof(struct vcnl4000_data), GFP_KERNEL);
	if (!vcnl4000) {
		dev_err(dev, "failed to alloc memory for module data\n");
		return -ENOMEM;
	}
	poll_dev = input_allocate_polled_device();
	if (!poll_dev) {
		dev_err(dev, "no memory for polled device\n");
		error = -ENOMEM;
		goto err_input_allocate_polled_device;
	}
	poll_dev->private = vcnl4000;
	poll_dev->poll = vcnl4000_polled_poll;
	poll_dev->poll_interval = pdata->poll_interval;
#if 0
	poll_dev->open = vcnl4000_polled_open;
	poll_dev->close = vcnl4000_polled_close;
#endif

	input = poll_dev->input;
	input->name = "vcnl4000";
	input->phys = "vcnl4000/input0";

	input_set_capability(input, EV_KEY, pdata->code);

	vcnl4000->poll_dev = poll_dev;
	vcnl4000->pdata = pdata;
	vcnl4000->i2c_client = client;
	vcnl4000->last_key_state = KEY_STATE_UNKNOWN;
	i2c_set_clientdata(client, vcnl4000);

	error = input_register_polled_device(poll_dev);
	if (error) {
		dev_err(dev, "unable to register polled device, err=%d\n",
			error);
		goto err_input_register_polled_device;
	}

	return 0;

	/* error, unwind it all */
err_input_register_polled_device:
	input_free_polled_device(poll_dev);
err_input_allocate_polled_device:
	kfree(vcnl4000);
	return error;
}

static int __devexit vcnl4000_i2c_remove(struct i2c_client *client)
{
	struct vcnl4000_data *vcnl4000 = i2c_get_clientdata(client);
	input_unregister_polled_device(vcnl4000->poll_dev);
	input_free_polled_device(vcnl4000->poll_dev);
	kfree(vcnl4000);
	return 0;
}

static const struct i2c_device_id vcnl4000_device_id[] = {
	{"vcnl4000", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, vcnl4000_device_id);

static struct i2c_driver vcnl4000_i2c_driver = {
	.driver = {
		.name = "vcnl4000",
		.owner = THIS_MODULE,
	},
	.probe		= vcnl4000_i2c_probe,
	.remove		= __devexit_p(vcnl4000_i2c_remove),
	.id_table	= vcnl4000_device_id,
};


static int __init vcnl4000_init(void)
{
	return i2c_add_driver(&vcnl4000_i2c_driver);
}

static void __exit vcnl4000_exit(void)
{
	i2c_del_driver(&vcnl4000_i2c_driver);
}

module_init(vcnl4000_init);
module_exit(vcnl4000_exit);

MODULE_AUTHOR("mjchen@google.com");
MODULE_DESCRIPTION("Proximity and Ambient Light Sensor driver for vcnl4000");
MODULE_LICENSE("GPL");
