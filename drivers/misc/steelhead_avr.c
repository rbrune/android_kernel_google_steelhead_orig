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

struct led_bank_state {
	u8 bank_id;
	u8 led_count;
	const char *name;
	u8 *vals;
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

	/* Misc bookkeeping */
	u16 firmware_rev;
	u8  hardware_type;
	u8  hardware_rev;

	/* State dealing with AVR IRQ. */
	int irq_registered;
	int irq_id;

	/* Current LED state. */
	struct led_bank_state *banks;
	u32 bank_cnt;
	u32 total_led_cnt;
	u8 *bank_scratch;
	u32 bank_scratch_bytes;
	u8 led_mode;
};

static const struct led_bank_state sphere_bank_template[] = {
	{	.bank_id = 0,
		.led_count = 64,
		.name = "DeathStarTrench",
		.vals = NULL  },
};

static const struct led_bank_state rhombus_bank_template[] = {
	{	.bank_id = 0,
		.led_count = 8,
		.name = "Crack_0",
		.vals = NULL  },
	{	.bank_id = 1,
		.led_count = 8,
		.name = "Crack_1",
		.vals = NULL  },
	{	.bank_id = 2,
		.led_count = 8,
		.name = "Crack_2",
		.vals = NULL  },
	{	.bank_id = 3,
		.led_count = 8,
		.name = "Crack_3",
		.vals = NULL  },
	{	.bank_id = 4,
		.led_count = 8,
		.name = "Crack_4",
		.vals = NULL  },
	{	.bank_id = 5,
		.led_count = 8,
		.name = "Crack_5",
		.vals = NULL  },
	{	.bank_id = 6,
		.led_count = 8,
		.name = "Crack_6",
		.vals = NULL  },
	{	.bank_id = 7,
		.led_count = 8,
		.name = "Crack_7",
		.vals = NULL  },
	{	.bank_id = 8,
		.led_count = 8,
		.name = "Crack_8",
		.vals = NULL  },
	{	.bank_id = 9,
		.led_count = 8,
		.name = "Buttons",
		.vals = NULL  },
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

	if (NULL != state->banks) {
		u32 i;
		for (i = 0; i < state->bank_cnt; ++i)
			kfree(state->banks[i].vals);
		kfree(state->banks);
		state->banks = NULL;
		state->bank_cnt = 0;
	}

	if (NULL != state->bank_scratch) {
		kfree(state->bank_scratch);
		state->bank_scratch = NULL;
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

	/* Revision will come back little-endian. */
	*revision = ((u16)buf[1] << 8) | (u16)buf[0];
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
	u32 i, j;

	if (!state || !req)
		return -EFAULT;

	mutex_lock(&state->i2c_lock);
	rc = i2c_smbus_write_i2c_block_data(state->i2c_client,
			AVR_LED_SET_ALL_REG_ADDR,
			sizeof(req->rgb), req->rgb);
	mutex_unlock(&state->i2c_lock);

	/* If the command failed, then skip the update of our internal
	 * bookkeeping and just get out.
	 */
	if (rc)
		return rc;

	/* Update internal LED state. */
	for (i = 0; i < state->bank_cnt; ++i) {
		u32 cnt  = state->banks[i].led_count * LED_BYTE_SZ;
		u8 *data = state->banks[i].vals;
		for (j = 0; j < cnt; j += LED_BYTE_SZ) {
			data[j + 0] = req->rgb[0];
			data[j + 1] = req->rgb[1];
			data[j + 2] = req->rgb[2];
		}
	}

	return rc;
}

static int avr_led_set_bank_vals(struct avr_driver_state *state,
				 struct avr_led_set_bank_vals *req)
{
	u32 cnt, i;
	u8 *data;
	int rc;

	if (!state || !req)
		return -EFAULT;

	if (req->bank_id >= state->bank_cnt)
		return -EINVAL;

	/* Pack the scratch buffer with the command to transmit
	 * to the AVR.
	 */
	BUG_ON(state->bank_scratch_bytes < 4);
	state->bank_scratch[0] = req->bank_id;
	state->bank_scratch[1] = req->rgb[0];
	state->bank_scratch[2] = req->rgb[1];
	state->bank_scratch[3] = req->rgb[2];

	mutex_lock(&state->i2c_lock);
	rc = i2c_smbus_write_i2c_block_data(state->i2c_client,
			AVR_LED_SET_BANK_REG_ADDR,
			4, state->bank_scratch);
	mutex_unlock(&state->i2c_lock);

	/* If the command failed, then skip the update of our internal
	 * bookkeeping and just get out.
	 */
	if (rc) {
		pr_err("SET_BANK_REG_ADDR command failed");
		return rc;
	}

	cnt  = state->banks[req->bank_id].led_count * LED_BYTE_SZ;
	data = state->banks[req->bank_id].vals;
	for (i = 0; i < cnt; i += LED_BYTE_SZ) {
		data[i + 0] = req->rgb[0];
		data[i + 1] = req->rgb[1];
		data[i + 2] = req->rgb[2];
	}

	return rc;
}

static int avr_led_set_range_vals(struct avr_driver_state *state,
				  struct avr_led_set_range_vals *req)
{
	int rc;

	if (!state || !req)
		return -EFAULT;

	if ((((u32)req->start) + req->count) > state->total_led_cnt)
		return -EINVAL;

	/* Scratch buffer should already have been packed by the caller
	 * with the LED values to use (do not use the rgb_vals pointer
	 * inside of request structure as the pointer is almost certainly
	 * a user-mode pointer)
	 */
	BUG_ON((SET_RANGE_OVERHEAD + (LED_BYTE_SZ * (u32)req->count)) >
			state->bank_scratch_bytes);

	state->bank_scratch[0] = req->start;
	state->bank_scratch[1] = req->count;

	mutex_lock(&state->i2c_lock);
	rc = i2c_smbus_write_i2c_block_data(state->i2c_client,
			AVR_LED_SET_RANGE_REG_ADDR,
			SET_RANGE_OVERHEAD + (LED_BYTE_SZ * (u32)req->count),
			state->bank_scratch);
	mutex_unlock(&state->i2c_lock);

	/* If the command failed, then skip the update of our internal
	 * bookkeeping and just get out.
	 */
	if (rc)
		return rc;

	/* TODO(johngro): fix bookkepping! */
#if 0
	memcpy(bank->vals, state->bank_scratch + SET_RANGE_OVERHEAD,
			((u32)req->count * LED_BYTE_SZ));
#endif
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

static int avr_setup_bank_state(struct avr_driver_state *state)
{
	const void *bank_template = NULL;
	u32 bank_template_sz = 0;
	u32 bank_template_cnt = 0;
	u32 i;

	BUG_ON(!state);

	/* Pick a bank topology template base on the hardware revision enum
	 * fetched from the AVR.
	 */
	switch (state->hardware_type) {
	case AVR_HE_TYPE_SPHERE:
		bank_template = sphere_bank_template;
		bank_template_sz = sizeof(sphere_bank_template);
		bank_template_cnt = ARRAY_SIZE(sphere_bank_template);
		break;

	case AVR_HE_TYPE_RHOMBUS:
		bank_template = rhombus_bank_template;
		bank_template_sz = sizeof(rhombus_bank_template);
		bank_template_cnt = ARRAY_SIZE(rhombus_bank_template);
		break;

	default:
		pr_err("Unrecognized hardware revision 0x%02x",
				state->hardware_type);
		return -ENODEV;
	}

	/* Allocate the memory for the bank state array and
	 * copy the template.
	 */
	state->banks = kzalloc(bank_template_sz, GFP_KERNEL);
	if (NULL == state->banks) {
		pr_err("Failed to allocate memory for bank state array.");
		return -ENOMEM;
	}
	state->bank_cnt = bank_template_cnt;
	memcpy(state->banks, bank_template, bank_template_sz);

	/* Allocate the memory for the individual banks. */
	state->bank_scratch_bytes = 0;
	state->total_led_cnt = 0;
	for (i = 0; i < state->bank_cnt; ++i) {
		u32 bank_mem = (LED_BYTE_SZ * state->banks[i].led_count);
		BUG_ON(!bank_mem);

		state->bank_scratch_bytes += bank_mem;
		state->total_led_cnt += state->banks[i].led_count;

		state->banks[i].vals = kzalloc(bank_mem, GFP_KERNEL);
		if (NULL == state->banks[i].vals) {
			pr_err("Failed to allocate %d bytes for led bank #%d",
					bank_mem, i);
			goto err_alloc_bank_vals;
		}
	}

	/* Allocate the memory for the scratch buffer used during bank set
	 * operations.  We need enough scratch buffer for the R G and B values
	 * for the all the banks in the system, as well as 2 bytes of overhead
	 * (for packing the set range command, we need a start and count value
	 * each of which is 8 bits)
	 */
	state->bank_scratch_bytes += SET_RANGE_OVERHEAD;
	state->bank_scratch = kzalloc(state->bank_scratch_bytes, GFP_KERNEL);
	if (NULL == state->bank_scratch) {
		pr_err("Failed to allocate %d bytes for bank scratch memory",
				state->bank_scratch_bytes);
		goto err_alloc_bank_scratch;
	}

	return 0;

 err_alloc_bank_scratch:
	while (--i) {
		kfree(state->banks[i].vals);
	}
 err_alloc_bank_vals:
	kfree(state->banks);
	return -ENOMEM;
}

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
		u16 revision = (cmd == AVR_LED_GET_FIRMWARE_REVISION)
			? state->firmware_rev
			: state->hardware_type;

		if (copy_to_user((void __user *)arg,
				&revision, sizeof(revision)))
			rc = -EFAULT;
	} break;

	case AVR_LED_GET_HARDWARE_TYPE:
	case AVR_LED_GET_HARDWARE_REVISION: {
		u8 val = (cmd == AVR_LED_GET_HARDWARE_TYPE)
			? state->hardware_type
			: state->hardware_rev;

		if (copy_to_user((void __user *)arg,
				&val, sizeof(val)))
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

	case AVR_LED_GET_BANK_COUNT: {
		if (copy_to_user((void __user *)arg,
				&state->bank_cnt, sizeof(state->bank_cnt)))
			rc = -EFAULT;
	} break;

	case AVR_LED_GET_BANK_DESC: {
		u32 req_ndx;
		struct avr_led_bank_desc desc;
		if (copy_from_user(&req_ndx, (const void __user *)arg,
					sizeof(req_ndx))) {
			rc = -EFAULT;
			break;
		}

		if (req_ndx >= state->bank_cnt) {
			rc = -EINVAL;
			break;
		}

		desc.bank_id = state->banks[req_ndx].bank_id;
		desc.led_count = state->banks[req_ndx].led_count;
		strncpy(desc.name, state->banks[req_ndx].name,
				sizeof(desc.name));
		desc.name[sizeof(desc.name) - 1] = 0;

		if (copy_to_user((void __user *)arg, &desc, sizeof(desc)))
			rc = -EFAULT;
	} break;

	case AVR_LED_GET_BANK_VALS: {
		struct avr_led_get_bank_vals req;
		struct led_bank_state *bank;

		if (copy_from_user(&req, (const void __user *)arg,
					sizeof(req))) {
			rc = -EFAULT;
			break;
		}

		if (req.bank_id >= state->bank_cnt) {
			rc = -EINVAL;
			break;
		}

		bank = &(state->banks[req.bank_id]);
		if ((req.start >= bank->led_count) ||
				((req.start + req.cnt) > bank->led_count)) {
			rc = -EINVAL;
			break;
		}

		if (copy_to_user((void __user *)arg,
					&(bank->vals[req.start * LED_BYTE_SZ]),
					req.cnt * LED_BYTE_SZ))
			rc = -EFAULT;
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

	case AVR_LED_SET_BANK_VALS: {
		struct avr_led_set_bank_vals req;

		if (copy_from_user(&req, (const void __user *)arg,
					sizeof(req))) {
			rc = -EFAULT;
			break;
		}

		rc = avr_led_set_bank_vals(state, &req);
	} break;

	case AVR_LED_SET_RANGE_VALS: {
		struct avr_led_set_range_vals req;
		u32 value_bytes;

		if (copy_from_user(&req, (const void __user *)arg,
					sizeof(req))) {
			rc = -EFAULT;
			break;
		}

		BUG_ON(!state->bank_scratch);
		value_bytes = (u32)req.count * LED_BYTE_SZ;
		if ((value_bytes + SET_RANGE_OVERHEAD) >
				state->bank_scratch_bytes) {
			rc = -EINVAL;
			break;
		}

		/* Copy the LED values from userland leaving room in the
		 * scratch buffer to pack the bank_id, start and count
		 * members of the set range command.
		 */
		if (copy_from_user(state->bank_scratch + SET_RANGE_OVERHEAD,
					(const void __user *)(req.rgb_vals),
					value_bytes)) {
			rc = -EFAULT;
			break;
		}

		rc = avr_led_set_range_vals(state, &req);
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

		was_down = (scan & AVR_KEY_EVENT_DOWN) != 0;
		scan &= AVR_KEY_EVENT_CODE_MASK;

		/* Generate the raw scan code. */
		input_event(state->input_dev, EV_MSC, MSC_SCAN, scan);

		/* Generate the translated key code. */
		if (scan < state->input_dev->keycodemax) {
			input_report_key(state->input_dev, state->keymap[scan],
					 was_down);
			pr_info("%s: reporting key '%s' %s\n", __func__,
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
		disable_irq(state->irq_id);
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
	struct avr_led_set_all_vals clear_led_req;
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

	/* Cache the firmware revision (also checks to be sure the AVR is
	 * actually there and talking to us).
	 */
	rc = avr_get_firmware_rev(state, &state->firmware_rev);
	if (rc) {
		pr_err("Failed to fetch AVR firmware revision (rc = %d)", rc);
		goto error;
	}

	/* Cache the hardware type and revision, then use it to determine the
	 * bank topology and setup the internal state for the banks.
	 */
	rc = avr_get_hardware_type(state, &state->hardware_type);
	if (rc) {
		pr_err("Failed to fetch AVR hardware type (rc = %d)", rc);
		goto error;
	}

	rc = avr_get_hardware_rev(state, &state->hardware_rev);
	if (rc) {
		pr_err("Failed to fetch AVR hardware revision (rc = %d)", rc);
		goto error;
	}

	rc = avr_setup_bank_state(state);
	if (rc) {
		pr_err("Failed to setup LED bank state (rc = %d)", rc);
		goto error;
	}

	/* Set the LED state to all off in order to match the internal state we
	 * just established.
	 */
	clear_led_req.rgb[0] = 0x00;
	clear_led_req.rgb[1] = 0x00;
	clear_led_req.rgb[2] = 0x00;
	rc = avr_led_set_all_vals(state, &clear_led_req);
	if (rc) {
		pr_err("Failed to clear LEDs on AVR (rc = %d)", rc);
		goto error;
	}

	rc = avr_led_commit_led_state(state,
			AVR_LED_COMMMIT_IMMEDIATELY);
	if (rc) {
		pr_err("Failed to commit clear of LEDs on AVR (rc = %d)", rc);
		goto error;
	}

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
	state->input_dev->id.version = state->firmware_rev;

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

	rc = avr_led_set_int_reg(state, AVR_BUTTON_INT_ENABLE
			| AVR_BUTTON_INT_CLEAR);
	if (rc) {
		pr_err("Failed to set int control register (rc = %d)", rc);
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

	/* Enable buttons inside of the AVR. */
	rc = avr_led_set_button_ctrl_reg(state, AVR_BUTTON_CONTROL_ENABLE0);
	if (rc) {
		pr_err("Failed to set button control register (rc = %d)", rc);
		goto error;
	}

	/* Switch to host control of the LEDs */
	rc = avr_led_set_mode(state, AVR_LED_MODE_HOST);
	if (rc) {
		pr_err("Failed to switch to host control mode (rc = %d)", rc);
		goto error;
	}

	pr_info("Steelhead AVR Driver loaded.  FW = %d.%d",
		state->firmware_rev >> 8,
		state->firmware_rev & 0xFF);

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
