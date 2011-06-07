/* drivers/misc/aah_clock.h
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

#ifndef __AAH_CLOCK_H
#define __AAH_CLOCK_H

struct timesync_transform {
	__s64 uncorrected_zero;
	__s64 corrected_zero;
	__s32 uncorrected_to_corrected_ticks_n;
	__u32 uncorrected_to_corrected_ticks_d;
};

extern s64 aah_clock_local_to_common_with_trans(
		s64 local, const struct timesync_transform *trans);
extern s64 aah_clock_common_to_local_with_trans(
		s64 common, const struct timesync_transform *trans);
extern int aah_clock_snapshot_current_transform(
		struct timesync_transform *trans_out);
extern int  aah_clock_local_to_common(s64 local, s64 *common_out);
extern int  aah_clock_common_to_local(s64 common, s64 *local_out);
extern void aah_clock_slam(s64 new_common_time);
extern int  aah_clock_get(s64 *out);
extern void aah_clock_set_slew(s32 slew_n, u32 slew_d);
extern int  aah_clock_is_valid(void);
extern void aah_clock_force_basis(s64 uncorrected, s64 corrected, int valid);
extern void aah_clock_init(struct timesync_platform_data *pdata);
extern void aah_clock_shutdown(void);
extern void aah_clock_force_basis(s64 uncorrected, s64 corrected, int valid);

#endif  /* __AAH_CLOCK_H */
