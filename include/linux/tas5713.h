/* include/linux/tas5713.h
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
 */

#ifndef __LINUX_TAS5713_H
#define __LINUX_TAS5713_H

#ifdef __KERNEL__
#include <plat/mcbsp.h>

struct tas5713_platform_data {
	/* GPIO IDs used for controlling the interface enable, reset and power
	 * down signals */
	unsigned reset_gpio;
	unsigned pdn_gpio;

	/* CLK selected by the board to provide MCLK to the TAS5713 */
	struct clk *mclk_out;
};
#endif

#include <linux/ioctl.h>

#define TAS5713_GET_MASTER_VOLUME      _IOR('A', 0xF8, __u8)
#define TAS5713_SET_MASTER_VOLUME      _IOW('A', 0xF9, __u8)

#endif  /* __LINUX_TAS5713_H */
