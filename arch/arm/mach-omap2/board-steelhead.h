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

#ifndef _MACH_OMAP2_BOARD_STEELHEAD_H_
#define _MACH_OMAP2_BOARD_STEELHEAD_H_

extern void omap4_steelhead_android_usb_init(void);
extern void __init steelhead_platform_init_counter(void);
extern s64 steelhead_get_raw_counter(void);
extern u32 steelhead_get_raw_counter_nominal_freq(void);

#ifdef CONFIG_AAH_TIMESYNC_DEBUG
extern void steelhead_register_timesync_event_handler(
		void *d, void (*handler)(void *d, u64));
#endif

#if 0 /* TBD */
extern int platform_set_counter_slew_rate(int ppm);
#endif

#endif
