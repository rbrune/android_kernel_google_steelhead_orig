/*
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


#ifndef __STEELHEAD_AVR_REGS_H
#define __STEELHEAD_AVR_REGS_H

/* key event registers */
#define AVR_KEY_EVENT_FIFO_REG_ADDR	0x00
#define AVR_KEY_EVENT_DOWN		0x80	/* set when key down */
#define AVR_KEY_EVENT_CODE_MASK		0x3F	/* mask for key code
						 * (keeping bit unused)
						 */
#define AVR_KEY_EVENT_RESET		0xFE	/* key event value returned
						 * when AVR first boots
						 */
#define AVR_KEY_EVENT_EMPTY		0xFF	/* key event value returned
						 * when fifo empty
						 */

#define AVR_KEY_MUTE_THRESHOLD_REG_ADDR 0x01

#define AVR_KEY_MUTE			0x00
#define AVR_KEY_VOLUME_UP		(AVR_KEY_MUTE + 1)
#define AVR_KEY_VOLUME_DOWN		(AVR_KEY_VOLUME_UP + 1)
#define AVR_KEYCODE_COUNT		(AVR_KEY_VOLUME_DOWN + 1)


/* led registers */
#define AVR_LED_MODE_REG_ADDR		0x02
#define AVR_LED_MODE_BOOT_ANIMATION	0x00
#define AVR_LED_MODE_HOST_AUTO_COMMIT	0x01
#define AVR_LED_MODE_HOST		0x02
#define AVR_LED_MODE_POWER_UP_ANIMATION	0x03

#define AVR_LED_SET_ALL_REG_ADDR	0x03

#define AVR_LED_SET_RANGE_REG_ADDR	0x04

#define AVR_LED_COMMIT_REG_ADDR		0x05	/* push the buffered host led
						 * values to active display
						 */
#define AVR_LED_COMMIT_IMMEDIATELY	0x00	/* update the active led's
						 * immediately
						 */
#define AVR_LED_COMMIT_INTERPOLATE	0x01	/* animate from the current
						 * led state to the new one?
						 * Later!
						 */

#define AVR_LED_SET_MUTE_ADDR		0x06	/* set mute led color */
#define AVR_LED_GET_COUNT_ADDR		0x07	/* get led count */

/* fw registers */
#define AVR_HW_TYPE_REG_ADDR		0x08
#define AVR_HE_TYPE_UNKNOWN		0x00
#define AVR_HE_TYPE_SPHERE		0x01
#define AVR_HE_TYPE_RHOMBUS		0x02

#define AVR_HW_REVISION_REG_ADDR	0x09

#define AVR_FW_VERSION_REG_ADDR		0x0a	/* 16 bit register 8.8 */

#endif  /* __STEELHEAD_AVR_REGS_H */
