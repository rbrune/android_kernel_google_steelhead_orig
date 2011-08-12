/* drivers/linux/aah_localtime.h
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

#ifndef __LINUX_AAH_LOCALTIME_H
#define __LINUX_AAH_LOCALTIME_H

#include <linux/types.h>

#ifdef __KERNEL__

struct aah_localtime_platform_data {
	/* Platform provided free running counter.
	 * Platform counters should be...
	 *
	 * + Monotonic.
	 * + Continuous.
	 * + Of reasonably high resolution (> 1MHz)
	 * + Driven by an oscillator which is locked to and derived from
	 *   the same oscillator which drives the audio and video outputs
	 *   in the system.
	 */
	s64 (*get_raw_counter)(void);
	u32 (*get_raw_counter_nominal_freq)(void);

	/* Optional function for adjusting the counter freq.
	 * If the board doesn't have the ability to slew its counter, set this
	 * to NULL.
	 *
	 * Adjusts the approximate rate at which a platform's counter runs.
	 *
	 * The signed correction argument determines the amount of correction
	 * which should be applied to the platform oscillator's frequency.
	 * Negative values indicate that the platform should attempt to slow
	 * down its counter while positive values indicate that the platform
	 * should attempt to speed up its counter.  The correction parameter is
	 * always absolute, not cumulative and is expressed in ppm.
	 */
	 void (*set_counter_slew_rate)(s16 correction);

#ifdef CONFIG_AAH_TIMESYNC_DEBUG
	/**********************************************************************
	 *                                                                    *
	 *     Hardware timesync event support (only for debugging)           *
	 *                                                                    *
	 **********************************************************************/

	/* Register the global timesync event handler with the platform.  Note,
	 * there is only one such callback permitted at a time.  Subsequent
	 * calls to register_timesync_event_handler will change the existing
	 * handler.  Passing NULL to this function will disable callbacks.
	 * It is expected that the driver on the system which implements the
	 * android@home timesync interface will be the only client of the event
	 * handler callback.
	 *
	 * Platforms which produce timesync events should return 0 to indicate
	 * that the event handler was properly registered and will be called.
	 * Platforms which do not produce timesync events should return -1.
	 */
	void (*register_timesync_event_handler)(void *d,
						void (*handler)(void *d, u64));
#endif
};

#endif

#include <linux/ioctl.h>

#define AAH_LOCALTIME_MAGIC 0xE0

#define AAHLT_IOCTL_LOCALTIME_GET       _IOR(AAH_LOCALTIME_MAGIC, 1, __s64)
#define AAHLT_IOCTL_LOCALTIME_GETFREQ   _IOR(AAH_LOCALTIME_MAGIC, 2, __u64)
#define AAHLT_IOCTL_LOCALTIME_SET_SLEW  _IOW(AAH_LOCALTIME_MAGIC, 3, __s16)

#ifdef CONFIG_AAH_TIMESYNC_DEBUG

struct aah_tsdebug_event_record {
	__s64 local_timesync_event_id;
	__s64 local_time;
};

struct aah_tsdebug_fetch_records_cmd {
	__u32 max_records_out;
	struct aah_tsdebug_event_record __user *records_out;
};
#define AAHLT_IOCTL_FETCH_TSDEBUG_RECORDS _IOWR(AAH_LOCALTIME_MAGIC, 100, \
					struct aah_tsdebug_fetch_records_cmd)
#endif /* CONFIG_AAH_TIMESYNC_DEBUG */

#endif  /* __LINUX_AAH_LOCALTIME_H */
