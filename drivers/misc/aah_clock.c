/* drivers/misc/aah_clock.c
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

#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/types.h>

#include <linux/aah_timesync.h>
#include <linux/android_utils.h>
#include "aah_clock.h"

#define kNominalCommonClockHz 1000000
static int g_active_transform_valid;
static DEFINE_SPINLOCK(g_aah_clock_lock);

static s32 g_local_ticks_to_common_ticks_n = 1;
static u32 g_local_ticks_to_common_ticks_d = 1;
static s32 g_common_clock_slew_rate_n = 1;
static u32 g_common_clock_slew_rate_d = 1;

static struct timesync_platform_data *g_pdata;

static struct timesync_transform g_active_transform = {
	.uncorrected_zero = 0,
	.corrected_zero = 0,
	.uncorrected_to_corrected_ticks_n = 1,
	.uncorrected_to_corrected_ticks_d = 1
};

s64 aah_clock_local_to_common_with_trans(s64 local,
		const struct timesync_transform *trans)
{
	return linear_transform_s64_to_s64(
			local,
			trans->uncorrected_zero,
			trans->uncorrected_to_corrected_ticks_n,
			trans->uncorrected_to_corrected_ticks_d,
			trans->corrected_zero);
}

s64 aah_clock_common_to_local_with_trans(s64 common_time,
					 const struct timesync_transform *trans)
{
	return linear_transform_s64_to_s64(
			common_time,
			trans->corrected_zero,
			trans->uncorrected_to_corrected_ticks_d,
			trans->uncorrected_to_corrected_ticks_n,
			trans->uncorrected_zero);
}

/* Take a snapshot of the current common time transform.  Returns 1 if the
 * transform is currently defined (success) or 0 if there is no defined
 * transform currently (failure).
 */
int aah_clock_snapshot_current_transform(struct timesync_transform *trans_out)
{
	unsigned long irq_state;
	int ret_val;
	/* TODO(johngro) : assert that trans_out is non-null.  Failing that, do
	 * a runtime check.
	 */

	spin_lock_irqsave(&g_aah_clock_lock, irq_state);
	ret_val = g_active_transform_valid;
	if (ret_val)
		memcpy(trans_out, &g_active_transform,
				sizeof(g_active_transform));
	spin_unlock_irqrestore(&g_aah_clock_lock, irq_state);

	return ret_val;
}

/* Convert a given local time to common time using the currently active
 * transform.  Returns 1 on success, or 0 if there is no defined transform
 * currently.
 */
int aah_clock_local_to_common(s64 local, s64 *common_time_out)
{

	/* TODO(johngro) : assert that common_time_out is non-null.
	 * Failing that, do a runtime check.
	 */
	struct timesync_transform trans;
	if (!aah_clock_snapshot_current_transform(&trans))
		return 0;

	*common_time_out = aah_clock_local_to_common_with_trans(local, &trans);
	return 1;
}

/* Convert a given common time to local time using the currently active
 * transform.  Returns 1 on success, or 0 if there is no defined transform
 * currently.
 */
int aah_clock_common_to_local(s64 common_time, s64 *local_out)
{
	/* TODO(johngro) : assert that local_out is non-null.  Failing that,
	 * do a runtime check.
	 */
	struct timesync_transform trans;
	if (!aah_clock_snapshot_current_transform(&trans))
		return 0;

	*local_out = aah_clock_common_to_local_with_trans(common_time, &trans);
	return 1;
}

void aah_clock_slam(s64 new_common_time)
{
	unsigned long irq_state;
	spin_lock_irqsave(&g_aah_clock_lock, irq_state);

	g_active_transform.uncorrected_zero = (*g_pdata->get_raw_counter)();
	g_active_transform.corrected_zero = new_common_time;
	g_active_transform_valid = 1;

	spin_unlock_irqrestore(&g_aah_clock_lock, irq_state);
}

int aah_clock_get(s64 *out)
{
	unsigned long irq_state;
	int res;

	BUG_ON(NULL == out);
	spin_lock_irqsave(&g_aah_clock_lock, irq_state);

	res = g_active_transform_valid;
	if (!res)
		*out = 0;
	else {
		s64 now = (*g_pdata->get_raw_counter)();
		*out = aah_clock_local_to_common_with_trans(
						    now,
						    &g_active_transform);
	}

	spin_unlock_irqrestore(&g_aah_clock_lock, irq_state);

	return res;
}

#define ABS(x) (((x) < 0) ? -(x) : (x))
void aah_clock_set_slew(s32 slew_n, u32 slew_d)
{
	unsigned long irq_state;
	u64 now_local;
	u64 now_common = 0;
	s32 tmp_n;
	u32 tmp_d;

	BUG_ON(!slew_n || !slew_d);

	spin_lock_irqsave(&g_aah_clock_lock, irq_state);

	now_local = (*g_pdata->get_raw_counter)();

	/* If the transform is currently valid, be sure to compute the new
	 * corrected zero for the transform.
	 */
	if (g_active_transform_valid)
		now_common = aah_clock_local_to_common_with_trans(
				now_local,
				&g_active_transform);

	/* Record the new slew rate (reduced) and then compute the new overall
	 * ratio between local time and common time.
	 */
	reduce_s32_u32_ratio(&slew_n, &slew_d);
	g_common_clock_slew_rate_n = slew_n;
	g_common_clock_slew_rate_d = slew_d;

	tmp_n = g_local_ticks_to_common_ticks_n;
	tmp_d = g_local_ticks_to_common_ticks_d;

	reduce_s32_u32_ratio(&slew_n, &tmp_d);
	reduce_s32_u32_ratio(&tmp_n,  &slew_d);

	BUG_ON(((u64)ABS(slew_n) * (u64)ABS(tmp_n)) & 0xFFFFFFFF00000000ull);
	BUG_ON(((u64)slew_d * (u64)tmp_d) & 0xFFFFFFFF00000000ull);

	g_active_transform.uncorrected_to_corrected_ticks_n =
		slew_n * tmp_n;
	g_active_transform.uncorrected_to_corrected_ticks_d =
		slew_d * tmp_d;

	/* If the clock is active, record the new uncorrected and corrected
	 * zeros of the transform.
	 */
	if (g_active_transform_valid) {
		g_active_transform.uncorrected_zero = now_local;
		g_active_transform.corrected_zero = now_common;
	}

	spin_unlock_irqrestore(&g_aah_clock_lock, irq_state);
}

int aah_clock_is_valid(void)
{
	return (g_active_transform_valid != 0);
}

void aah_clock_force_basis(s64 uncorrected, s64 corrected, int valid)
{
	unsigned long irq_state;
	spin_lock_irqsave(&g_aah_clock_lock, irq_state);
	g_active_transform.uncorrected_zero = uncorrected;
	g_active_transform.corrected_zero = corrected;
	g_active_transform_valid = valid;
	spin_unlock_irqrestore(&g_aah_clock_lock, irq_state);
}

void aah_clock_init(struct timesync_platform_data *pdata)
{
	g_active_transform_valid = false;
	g_pdata = pdata;

	g_common_clock_slew_rate_n = 1;
	g_common_clock_slew_rate_d = 1;

	g_local_ticks_to_common_ticks_n = kNominalCommonClockHz;
	g_local_ticks_to_common_ticks_d =
		(*pdata->get_raw_counter_nominal_freq)();
	reduce_s32_u32_ratio(&g_local_ticks_to_common_ticks_n,
			     &g_local_ticks_to_common_ticks_d);

	g_active_transform.uncorrected_zero = 0;
	g_active_transform.corrected_zero   = 0;
	g_active_transform.uncorrected_to_corrected_ticks_n =
		g_local_ticks_to_common_ticks_n;
	g_active_transform.uncorrected_to_corrected_ticks_d =
		g_local_ticks_to_common_ticks_d;
}

void aah_clock_shutdown(void)
{
}

