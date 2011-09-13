/*
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


#ifndef __LINUX_VCNL4000_H
#define __LINUX_VCNL4000_H

#include <linux/types.h>

#ifdef __KERNEL__
struct vcnl4000_platform_data {
	int poll_interval;    /* interval to poll, in milliseconds */
	int ir_led_current;   /* from 0 to 200mA in multiple of 10 */
	int detect_threshold; /* threshold value to indicate in proximity */
	unsigned int code;    /* input event code (KEY_*) */
};
#endif /* __KERNEL__ */

#endif
