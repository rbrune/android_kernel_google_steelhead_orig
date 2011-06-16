
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
	/* the ID of the McBSP being used as an I2S port */
	omap_mcbsp_id mcbsp_id;

	/* GPIO IDs used for controlling the reset and power down signals */
	unsigned reset_gpio;
	unsigned pdn_gpio;

	/* CLK selected by the board to provide MCLK to the TAS5713 */
	struct clk *mclk_out;

	/* Functions provided by the platform for getting information about
	 * a clock phase locked to the audio HW.
	 */
	s64 (*get_raw_counter)(void);
	u32 (*get_raw_counter_nominal_freq)(void);
};
#endif

#include <linux/ioctl.h>

#define TAS5713_MAGIC 0xE1

#define TAS5713_FLUSH                  _IO(TAS5713_MAGIC, 0)
#define TAS5713_GET_ERROR_REGISTER     _IOR(TAS5713_MAGIC, 2, __u8)
#define TAS5713_GET_MASTER_VOLUME      _IOR(TAS5713_MAGIC, 3, __u8)
#define TAS5713_SET_MASTER_VOLUME      _IOW(TAS5713_MAGIC, 4, __u8)

#define MAX_TAS5713_REG_SIZE 20
struct tas5713_i2c_request {
	u8 reg;
	u8 len;
	u8 is_write_op;
	u8 data[MAX_TAS5713_REG_SIZE];
};
#define TAS5713_DO_LOW_LEVEL_I2C _IOWR(TAS5713_MAGIC, 5, \
				       struct tas5713_i2c_request)

#define TAS5713_GET_NEXT_WRITE_TIMESTAMP _IOR(TAS5713_MAGIC, 6, s64)

#endif  /* __LINUX_TAS5713_H */
