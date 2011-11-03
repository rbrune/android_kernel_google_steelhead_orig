/* drivers/misc/steelhead_avr.c
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
 *
 * This is a driver that communicates with an Atmel AVR ATmega328P
 * subboard in the Android@Home device via gpios and i2c.  This subboard
 * is Arduino-Compatible and the firmware in it is developed using the
 * Arduino SDK.
 *
 * The functionality implemented by the subboard is a set of capacitive touch
 * keys and many leds.  To keep things simple for now, we have just
 * one driver that implements two input_device exposing the keys and
 * a misc_device exposing custom ioctls for controlling the leds.  We don't
 * use the Linux led driver API because we have too many leds and want
 * a more custom API to be more efficient.  Also, the subboard firmware
 * implements some macro led modes (like volume mode) which doesn't make
 * sense in the led API.
 */

#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/steelhead_avr.h>

#include "steelhead_avr_regs.h"

/* TODO(johngro): remove this include.  No real need to delay here. */
#include <linux/delay.h>

#define LED_BYTE_SZ 3
#define SET_RANGE_OVERHEAD 2

MODULE_LICENSE("Dual BSD/GPL");

static const u16 initial_avr_keymap[] = {
	KEY_MUTE,
	KEY_VOLUMEUP,
	KEY_VOLUMEDOWN,
};

struct avr_driver_state {
	/* Locks */
	struct mutex api_lock;
	struct mutex i2c_lock;

	/* I2C client/device and platform specific data
	 * (BSP and GPIO selection)
	 */
	struct i2c_client *i2c_client;
	struct steelhead_avr_platform_data *pdata;

	/* Device node registation */
	struct miscdevice dev_node;
	int dev_node_registered;

	/* Input device registration */
	struct input_dev *input_dev;
	u16 *keymap;
	int input_node_registered;

	/* State dealing with AVR IRQ. */
	int irq_registered;
	int irq_id;

	/* Current LED state. */
	u8 led_mode;
};

static void cleanup_driver_state(struct avr_driver_state *state)
{
	if (NULL == state)
		return;

	if (state->irq_registered) {
		free_irq(state->irq_id, state);
		state->irq_registered = 0;
	}

	if (state->dev_node_registered) {
		misc_deregister(&state->dev_node);
		state->dev_node_registered = 0;
	}

	if (state->input_node_registered) {
		input_set_drvdata(state->input_dev, NULL);
		input_unregister_device(state->input_dev);
		state->input_node_registered = 0;
	}

	if (NULL != state->keymap) {
		kfree(state->keymap);
		state->keymap = NULL;
	}

	if (NULL != state->input_dev) {
		input_free_device(state->input_dev);
		state->input_dev = NULL;
	}

	i2c_set_clientdata(state->i2c_client, NULL);
	kfree(state);
}

static inline struct avr_driver_state *state_from_file(struct file *file)
{
	struct miscdevice *m = file->private_data;
	struct avr_driver_state *state =
			container_of(m, struct avr_driver_state,
					dev_node);
	return state;
}

static int avr_i2c_read(struct i2c_client *client, u8 cmd, u16 len, u8 *buf)
{
	int rc;
	rc = i2c_smbus_write_byte(client, cmd);
	if (rc)
		return rc;

	/* Need to wait a little bit between the write of the register ID
	 * and the read of the actual data.  Failure to do so will not
	 * result in a NAK, only corrupt data.
	 */
	usleep_range(50, 50);

	rc = i2c_master_recv(client, buf, len);

	if (rc >= 0)
		rc = (rc == len) ? 0 : -EIO;

	return rc;
}

static int avr_get_firmware_rev(struct avr_driver_state *state, u16 *revision)
{
	int rc;
	u8 buf[2];
	if (!state || !revision)
		return -EFAULT;

	mutex_lock(&state->i2c_lock);
	rc = avr_i2c_read(state->i2c_client,
			AVR_FW_VERSION_REG_ADDR, sizeof(buf), buf);
	mutex_unlock(&state->i2c_lock);

	*revision = ((u16)buf[0] << 8) | (u16)buf[1];
	pr_debug("%s: fw revision %d.%d, %u\n", __func__,
		 buf[0], buf[1], *revision);
	return rc;
}

static int avr_get_hardware_type(struct avr_driver_state *state, u8 *type)
{
	int rc;
	if (!state || !type)
		return -EFAULT;

	mutex_lock(&state->i2c_lock);
	rc = avr_i2c_read(state->i2c_client,
			AVR_HW_TYPE_REG_ADDR, 1, type);
	mutex_unlock(&state->i2c_lock);
	return rc;
}

static int avr_get_hardware_rev(struct avr_driver_state *state, u8 *revision)
{
	int rc;
	if (!state || !revision)
		return -EFAULT;

	mutex_lock(&state->i2c_lock);
	rc = avr_i2c_read(state->i2c_client,
			AVR_HW_REVISION_REG_ADDR, 1, revision);
	mutex_unlock(&state->i2c_lock);
	return rc;
}

static int avr_led_set_all_vals(struct avr_driver_state *state,
				struct avr_led_set_all_vals *req)
{
	int rc;

	if (!state || !req)
		return -EFAULT;

	mutex_lock(&state->i2c_lock);
	rc = i2c_smbus_write_i2c_block_data(state->i2c_client,
			AVR_LED_SET_ALL_REG_ADDR,
			sizeof(req->rgb), req->rgb);
	mutex_unlock(&state->i2c_lock);

	return rc;
}

static int avr_led_set_mute(struct avr_driver_state *state,
			    struct avr_led_set_mute_vals *req)
{
	int rc;

	if (!state || !req)
		return -EFAULT;

	mutex_lock(&state->i2c_lock);
	rc = i2c_smbus_write_i2c_block_data(state->i2c_client,
			AVR_LED_SET_MUTE_ADDR,
			sizeof(req->rgb), req->rgb);
	mutex_unlock(&state->i2c_lock);

	return rc;
}

static int avr_led_set_mode(struct avr_driver_state *state, u8 mode)
{
	int rc;
	if (!state)
		return -EFAULT;

	mutex_lock(&state->i2c_lock);
	rc = i2c_smbus_write_byte_data(state->i2c_client,
			AVR_LED_MODE_REG_ADDR, mode);
	mutex_unlock(&state->i2c_lock);

	/* If the command failed, then skip the update of our internal
	 * bookkeeping and just get out.
	 */
	if (rc)
		return rc;

	state->led_mode = mode;
	return rc;
}

static int avr_led_set_vol_indicator(struct avr_driver_state *state, u8 vol)
{
	int rc;
	if (!state)
		return -EFAULT;

	mutex_lock(&state->i2c_lock);
	rc = i2c_smbus_write_byte_data(state->i2c_client,
			AVR_VOLUME_SETTING_REG_ADDR, vol);
	mutex_unlock(&state->i2c_lock);

	return rc;
}

static int avr_led_commit_led_state(struct avr_driver_state *state, u8 val)
{
	int rc;
	if (!state)
		return -EFAULT;

	mutex_lock(&state->i2c_lock);
	rc = i2c_smbus_write_byte_data(state->i2c_client,
			AVR_LED_COMMIT_REG_ADDR, val);
	mutex_unlock(&state->i2c_lock);

	return rc;
}

static int avr_led_set_button_ctrl_reg(struct avr_driver_state *state, u8 val)
{
	int rc;
	if (!state)
		return -EFAULT;

	mutex_lock(&state->i2c_lock);
	rc = i2c_smbus_write_byte_data(state->i2c_client,
			AVR_BUTTON_CONTROL_REG_ADDR, val);
	mutex_unlock(&state->i2c_lock);

	return rc;
}

static int avr_led_set_int_reg(struct avr_driver_state *state, u8 val)
{
	int rc = 0;
	if (!state)
		return -EFAULT;

	mutex_lock(&state->i2c_lock);
	rc = i2c_smbus_write_byte_data(state->i2c_client,
			AVR_BUTTON_INT_REG_ADDR, val);
	mutex_unlock(&state->i2c_lock);

	return rc;
}

#if 0
static int avr_led_get_int_reg(struct avr_driver_state *state, u8 *val)
{
	int rc = 0;
	if (!state)
		return -EFAULT;

	mutex_lock(&state->i2c_lock);
	rc = avr_i2c_read(state->i2c_client,
			AVR_BUTTON_INT_REG_ADDR, 1, val);
	mutex_unlock(&state->i2c_lock);

	return rc;
}
#endif

static int avr_leddev_open(struct inode *inode, struct file *file)
{
	/* nothing special to do here right now. */
	return 0;
}

static long avr_leddev_ioctl(struct file *file, unsigned int cmd,
			     unsigned long arg)
{
	struct avr_driver_state *state = state_from_file(file);
	long rc = 0;

	mutex_lock(&state->api_lock);

	switch (cmd) {
	case AVR_LED_GET_FIRMWARE_REVISION: {
		u16 fw_rev;
		rc = avr_get_firmware_rev(state, &fw_rev);
		if (rc) {
			pr_err("%s: Failed to get avr firmware rev\n",
			       __func__);
			break;
		}
		if (copy_to_user((void __user *)arg, &fw_rev, sizeof(fw_rev)))
			rc = -EFAULT;
	} break;

	case AVR_LED_GET_HARDWARE_TYPE:
	case AVR_LED_GET_HARDWARE_REVISION: {
		u8 data;
		if (cmd == AVR_LED_GET_HARDWARE_TYPE)
			rc = avr_get_hardware_type(state, &data);
		else
			rc = avr_get_hardware_rev(state, &data);
		if (rc) {
			pr_err("%s: Failed to get avr hardware %s\n", __func__,
			       (cmd == AVR_LED_GET_HARDWARE_TYPE) ? "type" :
			       "rev");
			break;
		}
		if (copy_to_user((void __user *)arg, &data, sizeof(data)))
			rc = -EFAULT;
	} break;

	case AVR_LED_GET_MODE: {
		u8 val = state->led_mode;
		if (copy_to_user((void __user *)arg,
				&val, sizeof(val)))
			rc = -EFAULT;
	} break;

	case AVR_LED_SET_MODE: {
		u8 val;
		if (copy_from_user(&val, (const void __user *)arg,
					sizeof(val))) {
			rc = -EFAULT;
			break;
		}

		rc = avr_led_set_mode(state, val);
	} break;

	case AVR_LED_SET_ALL_VALS: {
		struct avr_led_set_all_vals req;

		if (copy_from_user(&req, (const void __user *)arg,
					sizeof(req))) {
			rc = -EFAULT;
			break;
		}

		rc = avr_led_set_all_vals(state, &req);
	} break;

	case AVR_LED_SET_VOLUME_INDICATOR: {
		u8 val;
		if (copy_from_user(&val, (const void __user *)arg,
					sizeof(val))) {
			rc = -EFAULT;
			break;
		}

		rc = avr_led_set_vol_indicator(state, val);
	} break;

	case AVR_LED_COMMIT_LED_STATE: {
		u8 val;
		if (copy_from_user(&val, (const void __user *)arg,
					sizeof(val))) {
			rc = -EFAULT;
			break;
		}

		rc = avr_led_commit_led_state(state, val);
	} break;

	case AVR_LED_RESET: {
		pr_info("resetting avr, driving gpio %d low\n",
			state->pdata->reset_gpio);
		gpio_set_value(state->pdata->reset_gpio, 0);
		msleep(100);
		gpio_set_value(state->pdata->reset_gpio, 1);

		/* TODO: clear our copy of bank state after a reset.
		 *  Not needed if the user sets all soon after.
		 */
		break;
	}

	case AVR_LED_SET_MUTE: {
		struct avr_led_set_mute_vals req;

		if (copy_from_user(&req, (const void __user *)arg,
					sizeof(req))) {
			rc = -EFAULT;
			break;
		}

		rc = avr_led_set_mute(state, &req);
	} break;

	default: {
		rc = -EINVAL;
	} break;

	}

	mutex_unlock(&state->api_lock);
	return rc;
}

static int avr_read_event_fifo(struct avr_driver_state *state, u8 *next_event)
{
	int rc;

	if (!state || !next_event)
		return -EFAULT;

	mutex_lock(&state->i2c_lock);
	rc = avr_i2c_read(state->i2c_client,
			AVR_KEY_EVENT_FIFO_REG_ADDR, 1, next_event);
	pr_debug("%s: avr_i2c_read() returned %d\n", __func__, rc);
	mutex_unlock(&state->i2c_lock);
	return rc;
}

static irqreturn_t avr_irq_thread_handler(int irq, void *data)
{
	struct avr_driver_state *state = data;
	int rc;

	BUG_ON(!state);

	while (1) {
		u8 scan = AVR_KEY_EVENT_EMPTY;
		int was_down;

		rc = avr_read_event_fifo(state, &scan);
		if (rc || (scan == AVR_KEY_EVENT_EMPTY))
			break;

		if (scan == AVR_KEY_EVENT_RESET) {
			/* This is the event we see whenever
			 * the AVR reset/boots.  We always set
			 * the mute color to 0 to indicate kernel
			 * has detected reset.
			 */
			struct avr_led_set_mute_vals mute_color;

			pr_info("%s: reset notification seen\n", __func__);
			mute_color.rgb[0] = 0;
			mute_color.rgb[1] = 0;
			mute_color.rgb[2] = 0;
			avr_led_set_mute(state, &mute_color);

			continue;
		}

		was_down = (scan & AVR_KEY_EVENT_DOWN) != 0;
		scan &= AVR_KEY_EVENT_CODE_MASK;

		/* Generate the raw scan code. */
		input_event(state->input_dev, EV_MSC, MSC_SCAN, scan);

		/* Generate the translated key code. */
		if (scan < state->input_dev->keycodemax) {
			input_report_key(state->input_dev, state->keymap[scan],
					 was_down);
			pr_debug("%s: reporting key '%s' %s\n", __func__,
				scan == AVR_KEY_VOLUME_UP ?
				"VOLUME_UP" : "VOLUME_DOWN",
				was_down ? "down" : "up");
		}

		input_sync(state->input_dev);
	}


	if (rc) {

		/* TODO(johngro) : we should never get an i2c error while
		 * attempting to talk to the AVR.  If we do, however, it
		 * we have to disable it because it is level triggered, or
		 * else we'd get an interrupt storm.
		 *
		 * It would be a good idea to schedule the IRQ to be
		 * re-enable at some point in time in the future.
		 * Perhaps the failure was transient and we will be
		 * able to drain the fifo properly after a second or so.
		 */

		pr_err("I2C Error while reading key event queue, "
		       "disabling IRQ.  No more key events will be "
		       "generated from front panel.");
#if 0
		disable_irq(state->irq_id);
#else
		/* this seems to happen if we rotate the dome a lot.
		 * probably due to the AVR being too busy handling
		 * rotation encoder interrupts to keep up with i2c
		 * requests.  it goes away so we'll ignore for now.
		 * might go with a delayed timer to reenable, but
		 * more likely we'll want to optimize the AVR code
		 * to be able to keep up better.
		 */
		pr_info("ignoring for now");
#endif
	}
	return IRQ_HANDLED;
}

static const struct file_operations avr_led_fops = {
	.owner = THIS_MODULE,
	.open = avr_leddev_open,
	.unlocked_ioctl = avr_leddev_ioctl,
};

static int avr_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct avr_driver_state *state;
	struct avr_led_set_mute_vals clear_led_req;
	int i;
	int rc;

	state = kzalloc(sizeof(struct avr_driver_state),
			GFP_KERNEL);
	if (NULL == state) {
		pr_err("Failed to allocate state structure");
		rc = -ENOMEM;
		goto error;
	}

	/* Set up the locks. */
	mutex_init(&state->api_lock);
	mutex_init(&state->i2c_lock);

	/* Hook up all of the state structures to each other. */
	i2c_set_clientdata(client, state);
	state->i2c_client = client;
	state->pdata = client->dev.platform_data;

	/* init gpio for reset and make sure AVR is not in reset */
	gpio_request(state->pdata->reset_gpio, "avr_reset");
	gpio_direction_output(state->pdata->reset_gpio, 1);

	/* allocate the Input device */
	state->input_dev = input_allocate_device();
	if (NULL == state->input_dev) {
		pr_err("Failed to allocate input device node.");
		rc = -ENOMEM;
		goto error;
	}
	state->dev_node_registered = 1;

	/* Allocate the keymap */
	state->keymap =
		kzalloc(sizeof(initial_avr_keymap), GFP_KERNEL);
	if (NULL == state->keymap) {
		pr_err("Failed to allocate input device keymap.");
		rc = -ENOMEM;
		goto error;
	}

	/* Set up all of the details about our input device. */
	state->input_dev->name = "Steelhead Front Panel";
	state->input_dev->id.bustype = BUS_I2C;
	state->input_dev->id.vendor = 0x0001;
	state->input_dev->id.product = 0x0001;
	state->input_dev->id.version = 0x0001; /* just made up */

	/* our device produces only key events, and uses the stock linux key
	 * repeat mechanism.
	 */
	set_bit(EV_KEY, state->input_dev->evbit);
	set_bit(EV_REP, state->input_dev->evbit);

	/* Set our keymap and report which key codes we can actually generate.
	 */
	memcpy(state->keymap, initial_avr_keymap,
			sizeof(initial_avr_keymap));

	state->input_dev->keycode = state->keymap,
	state->input_dev->keycodesize = sizeof(*state->keymap);
	state->input_dev->keycodemax =
		ARRAY_SIZE(initial_avr_keymap);

	for (i = 0; i < state->input_dev->keycodemax; ++i)
		set_bit(state->keymap[i], state->input_dev->keybit);

	input_set_capability(state->input_dev, EV_MSC, MSC_SCAN);
	input_set_drvdata(state->input_dev, state);

	/* Register the Input device */
	rc = input_register_device(state->input_dev);
	if (rc) {
		pr_err("Failed to register input device node (rc = %d)", rc);
		goto error;
	}
	state->input_node_registered = 1;

	/* register the LED device */
	state->dev_node.minor = MISC_DYNAMIC_MINOR,
	state->dev_node.name  = "leds",
	state->dev_node.fops  = &avr_led_fops,

	rc = misc_register(&state->dev_node);
	if (rc) {
		pr_err("Failed to register LED device node (rc = %d)", rc);
		goto error;
	}

	/* Set up the IRQ line from the AVR */
	state->irq_id = gpio_to_irq(state->pdata->interrupt_gpio);
	rc = request_threaded_irq(state->irq_id, NULL,
				  avr_irq_thread_handler,
				  IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				  "steelhead avr irq", state);
	if (rc) {
		pr_err("Failed to register IRQ (rc = %d)", rc);
		goto error;
	}
	state->irq_registered = 1;

	/* Per UI spec, clear mute led on kernel init. */
	clear_led_req.rgb[0] = 0x00;
	clear_led_req.rgb[1] = 0x00;
	clear_led_req.rgb[2] = 0x00;

	rc = avr_led_set_mute(state, &clear_led_req);
	if (rc) {
		/* don't treat this as fatal.  userland
		 * can still force reset and use bootloader
		 * to update firmware.
		 */
		pr_err("%s: clearing mute led failed\n", __func__);
	}

	pr_info("Steelhead AVR Driver loaded.\n");

	return rc;

error:
	cleanup_driver_state(state);
	return rc;
}


static int __devexit avr_remove(struct i2c_client *client)
{
	struct avr_driver_state *state = i2c_get_clientdata(client);

	/* Try to turn off interrupts. */
	avr_led_set_int_reg(state, 0);

	/* Try to turn off button inside of the AVR. */
	avr_led_set_button_ctrl_reg(state, 0);

	/* Switch back to boot animation mode before we clean out our state and
	 * finish unloading the driver.
	 */
	avr_led_set_mode(state, AVR_LED_MODE_BOOT_ANIMATION);

	/* cleanup our state and get out. */
	cleanup_driver_state(state);
	return 0;
}

static struct i2c_device_id avr_idtable[] = {
	{ "steelhead-avr", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, avr_idtable);

static struct i2c_driver avr_driver = {
	.driver = {
		.name = "steelhead-avr",
	},

	.id_table = avr_idtable,
	.probe = avr_probe,
	.remove = __devexit_p(avr_remove),

	/* TODO(johngro) implement these optional power management routines. */
	.shutdown = NULL,
	.suspend = NULL,
	.resume = NULL,
};

static int avr_init(void)
{
	return i2c_add_driver(&avr_driver);
}

static void avr_exit(void)
{
	i2c_del_driver(&avr_driver);
}

module_init(avr_init);
module_exit(avr_exit);
