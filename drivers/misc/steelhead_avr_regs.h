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

/* low-level button registers */
#define AVR_BUTTON_CONTROL_REG_ADDR	0x00
#define AVR_BUTTON_CONTROL_ENABLE0	0x01	/* Enable button 0 */
#define AVR_BUTTON_CONTROL_ENABLE1	0x02	/* Enable button 1 */
#define AVR_BUTTON_CONTROL_ENABLE2	0x04	/* Enable button 2 */
#define AVR_BUTTON_CONTROL_DEBUG	0x80	/* Enable debug prints */

#define AVR_BUTTON_LED_CONTROL_REG_ADDR	0x01
#define AVR_BUTTON_LED_CONTROL_ENABLE0	0x01	/* Enable button 0 LED */
#define AVR_BUTTON_LED_CONTROL_ENABLE1	0x02	/* Enable button 1 LED */
#define AVR_BUTTON_LED_CONTROL_ENABLE2	0x04	/* Enable button 2 LED */

#define AVR_BUTTON_STATE_REG_ADDR	0x02
#define AVR_BUTTON_STATE_PRESSED0	0x01	/* Button 0 is pressed */
#define AVR_BUTTON_STATE_PRESSED1	0x02	/* Button 1 is pressed */
#define AVR_BUTTON_STATE_PRESSED2	0x04	/* Button 2 is pressed */

#define AVR_BUTTON_INT_REG_ADDR		0x03
#define AVR_BUTTON_INT_ENABLE		0x01	/* Button/key event
						 * interrupt enabled
						 */
#define AVR_BUTTON_INT_PENDING		0x40	/* Button interrupt pending. */
#define AVR_BUTTON_INT_CLEAR		0x80	/* Button interrupt clear.
						 * Write 1 to clear
						 */

/* key event registers */
#define AVR_KEY_COUNT_REG_ADDR		0x10

#define AVR_KEY_EVENT_FIFO_REG_ADDR	0x11
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

#define AVR_KEY_MUTE			0x00
#define AVR_KEY_VOLUME_UP		(AVR_KEY_MUTE + 1)
#define AVR_KEY_VOLUME_DOWN		(AVR_KEY_VOLUME_UP + 1)
#define AVR_KEYCODE_COUNT		(AVR_KEY_VOLUME_DOWN + 1)


/* led registers */
#define AVR_LED_MODE_REG_ADDR		0x20
#define AVR_LED_MODE_BOOT_ANIMATION	0x00
#define AVR_LED_MODE_VOLUME		0x01
#define AVR_LED_MODE_HOST		0x02

#define AVR_LED_SET_ALL_REG_ADDR	0x21

#define AVR_LED_SET_BANK_REG_ADDR	0x22

#define AVR_LED_SET_RANGE_REG_ADDR	0x23

#define AVR_LED_COMMIT_REG_ADDR		0x24	/* push the buffered host led
						 * values to active display
						 */
#define AVR_LED_COMMMIT_IMMEDIATELY	0x00	/* update the active led's
						 * immediately
						 */
#define AVR_LED_COMMMIT_INTERPOLATE	0x01	/* animate from the current
						 * led state to the new one?
						 * Later!
						 */

#define AVR_LED_SET_RUN_REG_ADDR	0x25	/* set a run of LEDs
						 * (bank independent) to
						 * a single color
						 */
#define AVR_LED_SET_MUTE_ADDR		0x28	/* set mute led color */


/* volume/system registers */
#define AVR_VOLUME_SETTING_REG_ADDR	0x30

/* fw registers */
#define AVR_HW_TYPE_REG_ADDR		0x80
#define AVR_HE_TYPE_UNKNOWN		0x00
#define AVR_HE_TYPE_SPHERE		0x01
#define AVR_HE_TYPE_RHOMBUS		0x02

#define AVR_HW_REVISION_REG_ADDR	0x81

#define AVR_FW_VERSION_REG_ADDR		0x82	/* 16 bit register 8.8 */

#endif  /* __STEELHEAD_AVR_REGS_H */
