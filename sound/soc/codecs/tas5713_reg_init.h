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

#ifndef __TAS5713_REG_INIT_H__
#define __TAS5713_REG_INIT_H__

struct tas5713_init_command {
	const int size;
	const char *const data;
};

static const struct tas5713_init_command tas5713_init_sequence[] = {
	/* Master Volume == mute (default) */
	{ .size = 2,  .data = "\x07\xFF" },

	/* System Control Register 1
	 * + PWM high-pass (dc blocking) enabled (default)
	 * + Soft unmute on recovery from clock error (non-default)
	 * + No de-emphasis (default)
	 */
	{ .size = 2, .data = "\x03\x80" },

	/* Serial Data Interface Register (16 bit I2S, non-default) */
	{ .size = 2, .data = "\x04\x03" },

	/* Soft Mute Register (soft mute disabled all-channels, default) */
	{ .size = 2, .data = "\x06\x00" },

	/* Modulation Limit Register %97.7% (default) */
	{ .size = 2, .data = "\x10\x02" },

	/* Start/Stop period register == 125.7mSec (default) */
	{ .size = 2, .data = "\x1A\x0F" },

	/* Interchannel delay registers (defaults for "AD" mode) */
	{ .size = 2, .data = "\x11\xAC" },
	{ .size = 2, .data = "\x12\x54" },
	{ .size = 2, .data = "\x13\xAC" },
	{ .size = 2, .data = "\x14\x54" },

	/* BKND_ERR register
	 * On backend error, attempt to restart PWM after 1047mSec (non-default)
	 */
	{ .size = 2, .data = "\x1C\x07" },

	/* Headphone volume == 0dB (default) */
	{ .size = 2, .data = "\x0A\x30" },

	/* Volume configuration: slew == 1024 steps (non-default) */
	{ .size = 2, .data = "\x0E\x91" },

	/* Clock Control Register: 44.1/48KHz sample clock,
	 * MCLK = 256xFs (defaults)
	 */
	{ .size = 2, .data = "\x00\x6C" },

	/* PWM Output Mux Register
	 * Ch.1     => OUT A
	 * Ch.1.inv => OUT B
	 * Ch.2.inv => OUT C
	 * Ch.2     => OUT D
	 */
	{ .size = 5, .data = "\x25\x01\x02\x31\x45" },

	/* Channel 1 Config
	 * Maps 100% of L and 0% of R channel 1 (default)
	 */
	{ .size = 5, .data = "\x70\x00\x80\x00\x00" },
	{ .size = 5, .data = "\x71\x00\x00\x00\x00" },
	{ .size = 5, .data = "\x72\x00\x00\x00\x00" },
	{ .size = 5, .data = "\x73\x00\x80\x00\x00" },

	/* DRC control (DRC off) */
	{ .size = 5, .data = "\x46\x00\x02\x00\x20" },

	/* Input Mux register
	 * + Left  data (indicated by LRCLK) -> channel 1 (default)
	 * + Right data (indicated by LRCLK) -> channel 2 (default)
	 */
	{ .size = 5, .data = "\x20\x00\x01\x77\x72" },

	/* Channel 2 Config
	 * Maps 100% of R and 0% of L channel 1 (default)
	 */
	{ .size = 5, .data = "\x74\x00\x80\x00\x00" },
	{ .size = 5, .data = "\x75\x00\x00\x00\x00" },
	{ .size = 5, .data = "\x76\x00\x00\x00\x00" },
	{ .size = 5, .data = "\x77\x00\x80\x00\x00" },

	/* Channel 1 Volume == 0dB (default) */
	{ .size = 2, .data = "\x08\x30" },

	/* Channel 2 Volume == 0dB (default) */
	{ .size = 2, .data = "\x09\x30" },

	/* Output post-scale (1/2 of default value) */
	{ .size = 5, .data = "\x56\x00\x40\x00\x00" },

	/* Output pre-scale (default value) */
	{ .size = 5, .data = "\x57\x00\x02\x00\x00" },

	/* Bank Switch and EQ Control register.
	 * + Disable auto bank switching, all filter writes go directly to DAP.
	 * + Equalizer On (default)
	 * + L/R Eq settings are ganged (writes to left channel config are
	 *   automatically written to the right channel as well)
	 */
	{ .size = 5,  .data = "\x50\x00\x00\x00\x10" },

	/* Channel 1 bi-quad values 0-6 (also written to channel 2, see above).
	 * Default values used.
	 */
	{ .size = 21, .data = "\x29\x00\x80\x00\x00\x00\x00\x00\x00\x00"
	  "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" },
	{ .size = 21, .data = "\x2A\x00\x80\x00\x00\x00\x00\x00\x00\x00"
	  "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" },
	{ .size = 21, .data = "\x2B\x00\x80\x00\x00\x00\x00\x00\x00\x00"
	  "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" },
	{ .size = 21, .data = "\x2C\x00\x80\x00\x00\x00\x00\x00\x00\x00"
	  "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" },
	{ .size = 21, .data = "\x2D\x00\x80\x00\x00\x00\x00\x00\x00\x00"
	  "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" },
	{ .size = 21, .data = "\x2E\x00\x80\x00\x00\x00\x00\x00\x00\x00"
	  "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" },
	{ .size = 21, .data = "\x2F\x00\x80\x00\x00\x00\x00\x00\x00\x00"
	  "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" },

	/* DRC1 attack/release thresholds (non-default; no idea what
	 * it is set to)
	 */
	{ .size = 9,  .data = "\x40\x09\x0A\x00\x00\x09\x09\xFF\xFF" },
	/* DRC1 softening filter alpha/omega (non-default; no idea what
	 * it is set to)
	 */
	{ .size = 9,  .data = "\x3B\x00\x20\x00\x00\x00\x40\x00\x00" },
	/* DRC1 attack/release rates (defaults) */
	{ .size = 9,  .data = "\x3C\x00\x00\x10\x00\xFF\xFF\xFF\xFD" },

	/* DRC2 attack/release thresholds (non-default; no idea what it
	 * is set to)
	 */
	{ .size = 9,  .data = "\x43\x04\xF0\x00\x00\x04\xEF\xFF\xFF" },
	/* DRC2 softening filter alpha/omega (non-default; no idea what
	 * it is set to)
	 */
	{ .size = 9,  .data = "\x3E\x00\x20\x00\x00\x00\x40\x00\x00" },
	/* DRC2 attack/release rates (defaults) */
	{ .size = 9,  .data = "\x3F\x00\x08\x00\x00\xFF\xF8\x00\x00" },

	/* Channel 1/2 Output mixer settings (defaults) */
	{ .size = 9,  .data = "\x51\x00\x80\x00\x00\x00\x00\x00\x00" },
	{ .size = 9,  .data = "\x52\x00\x80\x00\x00\x00\x00\x00\x00" },

	/* Channel 1 bi-quad values 7-8 (defaults) */
	{ .size = 21, .data = "\x58\x00\x80\x00\x00\x00\x00\x00\x00"
	  "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" },
	{ .size = 21, .data = "\x59\x00\x80\x00\x00\x00\x00\x00\x00"
	  "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" },
	/* Channel 4 bi-quad values 0-1 (defaults) */
	{ .size = 21, .data = "\x5A\x00\x80\x00\x00\x00\x00\x00\x00"
	  "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" },
	{ .size = 21, .data = "\x5B\x00\x80\x00\x00\x00\x00\x00\x00"
	  "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" },
};

#endif  /* __TAS5713_REG_INIT_H__ */
