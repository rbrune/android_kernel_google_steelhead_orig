/* sound/soc/codecs/tas5713_registers.h
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

#ifndef __TAS5713_DEBUGFS_H__
#define __TAS5713_DEBUGFS_H__

#ifdef CONFIG_DEBUG_FS

struct tas5713_debugfs_register {
	uint8_t addr;
	uint8_t len;
	const char* name;
};

static const struct tas5713_debugfs_register tas5713_debugfs_registers[] = {
	{ 0x00,  1, "Clock control register" },
	{ 0x01,  1, "Device ID register" },
	{ 0x02,  1, "Error status register" },
	{ 0x03,  1, "System control register 1" },
	{ 0x04,  1, "Serial data interface register" },
	{ 0x05,  1, "System control register 2" },
	{ 0x06,  1, "Soft mute register" },
	{ 0x07,  1, "Master volume" },
	{ 0x08,  1, "Channel 1 vol" },
	{ 0x09,  1, "Channel 2 vol" },
	{ 0x0A,  1, "Channel 3 vol" },
	{ 0x0E,  1, "Volume configuration register" },
	{ 0x10,  1, "Modulation limit register" },
	{ 0x11,  1, "IC delay channel 1" },
	{ 0x12,  1, "IC delay channel 2" },
	{ 0x13,  1, "IC delay channel 3" },
	{ 0x14,  1, "IC delay channel 4" },
	{ 0x1A,  1, "Start/stop period register" },
	{ 0x1B,  1, "Oscillator trim register" },
	{ 0x1C,  1, "BKND_ERR register" },
	{ 0x20,  4, "Input MUX register" },
	{ 0x21,  4, "Ch 4 source select register" },
	{ 0x25,  4, "PWM MUX register" },
	{ 0x29, 20, "ch1_bq[0]" },
	{ 0x2a, 20, "ch1_bq[1]" },
	{ 0x2b, 20, "ch1_bq[2]" },
	{ 0x2c, 20, "ch1_bq[3]" },
	{ 0x2d, 20, "ch1_bq[4]" },
	{ 0x2e, 20, "ch1_bq[5]" },
	{ 0x2f, 20, "ch1_bq[6]" },
	{ 0x30, 20, "ch2_bq[0]" },
	{ 0x31, 20, "ch2_bq[1]" },
	{ 0x32, 20, "ch2_bq[2]" },
	{ 0x33, 20, "ch2_bq[3]" },
	{ 0x34, 20, "ch2_bq[4]" },
	{ 0x35, 20, "ch2_bq[5]" },
	{ 0x36, 20, "ch2_bq[6]" },
	{ 0x3B,  8, "DRC1 softening filter alpha/omega" },
	{ 0x3C,  8, "DRC1 attack/release rate" },
	{ 0x3E,  8, "DRC2 softening filter alpha/omega" },
	{ 0x3F,  8, "DRC2 attack/release rate" },
	{ 0x40,  8, "DRC1 attack/release threshold" },
	{ 0x43,  8, "DRC2 attack/decay threshold" },
	{ 0x46,  4, "DRC control" },
	{ 0x50,  4, "Bank switch control" },
	{ 0x51,  8, "Ch 1 output mixer" },
	{ 0x52,  8, "Ch 2 output mixer" },
	{ 0x53, 16, "Ch 1 input mixers" },
	{ 0x54, 16, "Ch 2 input mixers" },
	{ 0x56,  4, "Output post-scale" },
	{ 0x57,  4, "Output pre-scale" },
	{ 0x58, 20, "ch1 BQ[7]" },
	{ 0x59, 20, "ch1 BQ[8]" },
	{ 0x5A, 20, "ch4 BQ[0]" },
	{ 0x5B, 20, "ch4 BQ[1]" },
	{ 0x5C, 20, "ch2 BQ[7]" },
	{ 0x5D, 20, "ch2 BQ[8]" },
	{ 0x5E, 20, "ch3 BQ[0]" },
	{ 0x5F, 20, "ch3 BQ[1]" },
	{ 0x62,  4, "IDF post scale" },
	{ 0x70,  4, "ch1 inline mixer" },
	{ 0x71,  4, "inline_DRC_en_mixer_ch1" },
	{ 0x72,  4, "ch1 right_channel_mixer" },
	{ 0x73,  4, "ch1 left_channel_mixer" },
	{ 0x74,  4, "ch2 inline mixer" },
	{ 0x75,  4, "inline_DRC_en_mixer_ch2" },
	{ 0x76,  4, "ch2 right_channel_mixer" },
	{ 0x77,  4, "ch2 left_channel_mixer" },
	{ 0xF8,  4, "Update dev address key" },
	{ 0xF9,  4, "Update dev address reg" },
};

#endif  /* CONFIG_DEBUG_FS */

#endif  /* __TAS5713_DEBUGFS_H__ */
