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

#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/aah_timesync.h>
#include <plat/dmtimer.h>

#include "board-steelhead.h"
#include "mux.h"

#define DM_TIMER_ID 8
#define DM_TIMER_CLK OMAP_TIMER_SRC_ABE_SYSCLK

struct counter_state {
	u32 upper;
	u32 lower_last;
	spinlock_t lock;
};

static struct omap_dm_timer	*counter_timer;
static u32			counter_freq = 1;
static unsigned long		rollover_check_timer_period;
static struct timer_list	rollover_check_timer;

static struct counter_state counter_state = {
	.upper = 0,
	.lower_last = 0,
	.lock = __SPIN_LOCK_UNLOCKED(counter_state.lock),
};

#ifdef CONFIG_AAH_TIMESYNC_DEBUG
static DEFINE_SPINLOCK(timesync_event_handler_lock);
static void (*timesync_event_handler)(void *d, u64);
static void                    *timesync_event_handler_data;

static struct counter_state tsdebug_counter_state = {
	.upper = 0,
	.lower_last = 0,
	.lock = __SPIN_LOCK_UNLOCKED(counter_state.lock),
};
#endif

static s64 get_counter_internal_l(struct counter_state *state, u32 lower)
{
	s64 ret;

	/* Check for rollover.  We have rolled over if bit 31 of the lower
	 * was set last time, but is clear this time.
	 */
	if (((lower ^ 0x80000000) & state->lower_last) & 0x80000000)
		++state->upper;

	state->lower_last = lower;

	ret = ((s64)state->upper << 32) | lower;

	return ret;
}

static u32 get_counter_lower(void)
{
	BUG_ON(IS_ERR_OR_NULL(counter_timer));
	return (u32)omap_dm_timer_read_counter(counter_timer);
}

#ifdef CONFIG_AAH_TIMESYNC_DEBUG
static irqreturn_t timer_capture_irq(int irq, void *dev_id)
{
	struct omap_dm_timer *gpt = (struct omap_dm_timer *)dev_id;
	unsigned long irq_state;
	s64 event_time;
	u32 lower;

	/* Read the captured counter and ack the interrupt. */
	lower = omap_dm_timers_read_capture1(gpt);
	omap_dm_timer_write_status(gpt, OMAP_TIMER_INT_CAPTURE);

	spin_lock_irqsave(&counter_state.lock, irq_state);
	event_time = get_counter_internal_l(&tsdebug_counter_state, lower);
	spin_unlock_irqrestore(&counter_state.lock, irq_state);

	if (NULL != timesync_event_handler)
		timesync_event_handler(timesync_event_handler_data, event_time);

	return IRQ_HANDLED;
}
#endif

static void counter_rollover_check(unsigned long arg)
{
	/* Force a rollover check. */
	steelhead_get_raw_counter();

	rollover_check_timer.expires += rollover_check_timer_period;
	add_timer(&rollover_check_timer);
}

void __init steelhead_platform_init_counter(void)
{
	u64 tmp;

	/* Get a hold of the timer we plan to use as the basis for local time
	 * and set it up to count using the sysclock auto resetting every time
	 * it rolls over.
	 */
	struct clk *steelhead_clock;
	BUG_ON(NULL != counter_timer);
	counter_timer = omap_dm_timer_request_specific(DM_TIMER_ID);
	BUG_ON(IS_ERR_OR_NULL(counter_timer));

	omap_dm_timer_enable(counter_timer);
	omap_dm_timer_set_source(counter_timer, DM_TIMER_CLK);
	omap_dm_timer_set_load(counter_timer, 1, 0);

	steelhead_clock = omap_dm_timer_get_fclk(counter_timer);
	BUG_ON(IS_ERR_OR_NULL(steelhead_clock));
	counter_freq = clk_get_rate(steelhead_clock);

	/* Initialize the state we use for extending the counter to 64 bits and
	 * register the rollover check timer to make certain the counter is
	 * called frequently enough to detect rollover.
	 */
	counter_state.upper = 0;
	counter_state.lower_last = get_counter_lower();
#ifdef CONFIG_AAH_TIMESYNC_DEBUG
	tsdebug_counter_state.upper = 0;
	tsdebug_counter_state.lower_last = get_counter_lower();
#endif

	/* Just to be safe, check for rollover every 1/4 of a 32 bit counter
	 * period.
	 */
	tmp = 0x40000000ull * HZ;
	do_div(tmp, steelhead_get_raw_counter_nominal_freq());
	rollover_check_timer_period = (u32)tmp;

	init_timer(&rollover_check_timer);
	rollover_check_timer.function = counter_rollover_check;
	rollover_check_timer.expires  = jiffies + rollover_check_timer_period;
	add_timer(&rollover_check_timer);

#ifdef CONFIG_AAH_TIMESYNC_DEBUG
	/* If we are set up to monitor the local clock rate for debugging
	 * purposes from an externally synchronized event, then set up capture
	 * mode on the OMAP GP timer we have chosen to produce the timesync
	 * event.
	 */
	{
		int status;
		omap_dm_timers_set_capture_mode(counter_timer,
				OMAP_TIMER_CAPTURE_MODE_SINGLE,
				OMAP_TIMER_CAPTURE_EDGE_BOTH);
		status = request_irq(
				omap_dm_timer_get_irq(counter_timer),
				timer_capture_irq,
				IRQF_TIMER | IRQF_IRQPOLL,
				"steelhead tsdebug",
				counter_timer);
		if (status) {
			pr_err("Steelhead: Failed to setup timesync debug "
					"interrupt (status = %d)\n", status);
			BUG_ON(1);
		}
		omap_dm_timer_set_int_enable(counter_timer,
				OMAP_TIMER_INT_CAPTURE);
	}
#endif

	/* Our GP timer should be fully set up now.  Go ahead and start it up.
	 */
	omap_dm_timer_start(counter_timer);
}

/******************************************************************************
 *                                                                            *
 *                          API implementation                                *
 *                                                                            *
 ******************************************************************************/
s64 steelhead_get_raw_counter(void)
{
	unsigned long irq_state;
	s64 ret;

	spin_lock_irqsave(&counter_state.lock, irq_state);

	ret = get_counter_internal_l(&counter_state, get_counter_lower());

	spin_unlock_irqrestore(&counter_state.lock, irq_state);
	return ret;
}

u32 steelhead_get_raw_counter_nominal_freq(void)
{
	return counter_freq;
}

#ifdef CONFIG_AAH_TIMESYNC_DEBUG
void steelhead_register_timesync_event_handler(void *user_data,
					       void (*handler)(void *d, u64))
{
	unsigned long irq_state;

	spin_lock_irqsave(&timesync_event_handler_lock, irq_state);
	timesync_event_handler = handler;
	timesync_event_handler_data = user_data;
	spin_unlock_irqrestore(&timesync_event_handler_lock, irq_state);
}
#endif

#if 0
int platform_set_counter_slew_rate(int ppm)
{
	/* TODO(johngro): implement platform slew. */
	return -1;
}
#endif
