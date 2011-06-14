/* drivers/misc/aah_timesync.c
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

#include <linux/init.h>
#include <linux/module.h>

#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/aah_timesync.h>

#include "aah_clock.h"

#define TIMESYNC_NAME "timesync"

#ifdef CONFIG_AAH_TIMESYNC_DEBUG
#define MAX_LOG_SIZE 16

struct aah_tsdebug_state {
	struct aah_tsdebug_event_record event_log[MAX_LOG_SIZE];
	u32 event_log_wr;
	u32 event_log_rd;
	u64 local_event_count;
};
#endif

struct timesync_data {
	struct timesync_platform_data *pdata;
	struct miscdevice misc_dev;
#ifdef CONFIG_AAH_TIMESYNC_DEBUG
	struct aah_tsdebug_state tsdebug_state;
	spinlock_t tsdebug_state_lock;
#endif
};

#ifdef CONFIG_AAH_TIMESYNC_DEBUG

static void handle_timesync_event(void *user_data, u64 raw_event_time)
{
	unsigned long irq_state;
	struct timesync_data *tdata = user_data;
	struct aah_tsdebug_state *s = &tdata->tsdebug_state;
	struct aah_tsdebug_event_record *e;

	spin_lock_irqsave(&tdata->tsdebug_state_lock, irq_state);

	BUG_ON(s->event_log_wr >= MAX_LOG_SIZE);
	BUG_ON(s->event_log_rd >= MAX_LOG_SIZE);
	e = s->event_log + s->event_log_wr;

	e->local_timesync_event_id = s->local_event_count++;
	e->local_time = raw_event_time;

	if (aah_clock_local_to_common(e->local_time, &e->common_time))
		e->flags = AAH_TSDEBUG_EVENTF_VALID_COMMON_TIME;
	else
		e->flags = 0;

	/* Advance the write pointer.  If it wraps and hits the read pointer
	 * again, move the read pointer up dropping the oldest entry in the log
	 * in the process.
	 */
	s->event_log_wr = (s->event_log_wr + 1) % MAX_LOG_SIZE;
	if (s->event_log_wr == s->event_log_rd)
		s->event_log_rd = (s->event_log_rd + 1) % MAX_LOG_SIZE;

	spin_unlock_irqrestore(&tdata->tsdebug_state_lock, irq_state);
}

static int get_tsdebug_events(struct timesync_data *tdata,
			      struct aah_tsdebug_fetch_records_cmd *cmd)
{
	unsigned long irq_state;
	struct aah_tsdebug_state *s = &tdata->tsdebug_state;
	struct aah_tsdebug_event_record shadow[MAX_LOG_SIZE];
	int ret;

	BUG_ON(!cmd);

	/* Enter the spinlock and copy the data into our stack based shadow
	 * buffer.  Then it should be safe to leave the spinlock and copy the
	 * shadow buffer into the user space buffer.
	 */
	spin_lock_irqsave(&tdata->tsdebug_state_lock, irq_state);

	BUG_ON(s->event_log_wr >= MAX_LOG_SIZE);
	BUG_ON(s->event_log_rd >= MAX_LOG_SIZE);

	ret = 0;
	while ((s->event_log_rd != s->event_log_wr) &&
			(ret < cmd->max_records_out)) {
		shadow[ret++] = s->event_log[s->event_log_rd];
		s->event_log_rd = (s->event_log_rd + 1) % MAX_LOG_SIZE;
	}

	spin_unlock_irqrestore(&tdata->tsdebug_state_lock, irq_state);

	/* Now it should be safe to copy into the user space buffer.*/
	if (copy_to_user((void __user *)cmd->records_out,
				shadow,
				sizeof(*cmd->records_out) * ret))
		return -EFAULT;

	return ret;
}
#endif

static int timesync_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static long timesync_ioctl(struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	struct timesync_data *tdata = container_of(file->private_data,
						   struct timesync_data,
						   misc_dev);
	switch (cmd) {
	case TT_IOCTL_LOCAL_TO_COMMON:
	case TT_IOCTL_COMMON_TO_LOCAL: {
		int success;
		u64 input, output;
		if (copy_from_user(&input, (void __user *) arg, sizeof(input)))
			return -EFAULT;

		if (cmd == TT_IOCTL_LOCAL_TO_COMMON)
			success = aah_clock_local_to_common(input, &output);
		else
			success = aah_clock_common_to_local(input, &output);

		if (!success)
			return -ENODEV;

		if (copy_to_user((void __user *) arg, &output, sizeof(output)))
			return -EFAULT;
	} break;

	case TT_IOCTL_LOCALTIME_GET: {
		u64 counter = (*tdata->pdata->get_raw_counter)();
		if (copy_to_user((void __user *) arg,
				&counter, sizeof(counter))) {
			return -EFAULT;
		}
	} break;

	case TT_IOCTL_LOCALTIME_GETFREQ: {
		/* TODO(johngro) : most other parts of the system seem to
		 * assume that local clock frequencies will never be greater
		 * than 4GHz.  This IOCTL should be changed to just return a
		 * 32 bit value.
		 */
		u64 freq = (*tdata->pdata->get_raw_counter_nominal_freq)();
		if (copy_to_user((void __user *) arg,
				&freq, sizeof(freq))) {
			return -EFAULT;
		}
	} break;

	case TT_IOCTL_COMMONTIME_GET: {
		u64 output;
		if (!aah_clock_get(&output))
			return -ENODEV;

		if (copy_to_user((void __user *) arg, &output, sizeof(output)))
			return -EFAULT;
	} break;

	case TT_IOCTL_COMMONTIME_RESET_BASIS: {
		aah_clock_force_basis(0, 0, 0);
	} break;

	case TT_IOCTL_COMMONTIME_SET_BASIS: {
		struct aah_timesync_basis param;
		if (copy_from_user(&param, (void __user *) arg, sizeof(param)))
			return -EFAULT;

		aah_clock_force_basis(param.local_basis, param.common_basis, 1);
	} break;

	case TT_IOCTL_COMMONTIME_SET_SLEW: {
		s32 ppm = (s32)arg;
		aah_clock_set_slew(1000000 + ppm, 1000000);
	} break;

	case TT_IOCTL_COMMONTIME_IS_VALID: {
		if (!aah_clock_is_valid())
			return -ENODEV;
	} break;

#ifdef CONFIG_AAH_TIMESYNC_DEBUG
	case TT_IOCTL_FETCH_TSDEBUG_RECORDS: {
		struct aah_tsdebug_fetch_records_cmd cmd;
		if (copy_from_user(&cmd, (void __user *) arg, sizeof(cmd)))
			return -EFAULT;

		return get_tsdebug_events(tdata, &cmd);
	} break;
#endif

	default:
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations timesync_fops = {
	.owner = THIS_MODULE,
	.open = timesync_open,
	.unlocked_ioctl = timesync_ioctl,
};

static int __devinit timesync_probe(struct platform_device *pdev)
{
	int err;
	struct timesync_platform_data *pdata = pdev->dev.platform_data;
	struct timesync_data *tdata;

	pr_info("%s\n", __func__);
	if (!pdata || !pdata->get_raw_counter ||
	    !pdata->get_raw_counter_nominal_freq) {
		pr_err("%s: missing pdata\n", __func__);
		return -ENODEV;
	}
#ifdef CONFIG_AAH_TIMESYNC_DEBUG
	if (!pdata->register_timesync_event_handler) {
		pr_err("%s: missing pdata\n", __func__);
		return -ENODEV;
	}
#endif

	tdata = kzalloc(sizeof(struct timesync_data), GFP_KERNEL);
	if (!tdata) {
		pr_err("%s: failed to allocate module data memory\n", __func__);
		return -ENOMEM;
	}
	tdata->pdata = pdata;

#ifdef CONFIG_AAH_TIMESYNC_DEBUG
        spin_lock_init(&tdata->tsdebug_state_lock);

	/* Register our callback for the platform's timesync event. */
	(*(pdata->register_timesync_event_handler))(tdata,
						    handle_timesync_event);
#endif

	aah_clock_init(pdata);

	/* register misc device */
	tdata->misc_dev.minor = MISC_DYNAMIC_MINOR;
	tdata->misc_dev.name = TIMESYNC_NAME;
	tdata->misc_dev.fops = &timesync_fops;
	err = misc_register(&tdata->misc_dev);
	if (err) {
		pr_err("%s: failed to register misc device\n", __func__);
#ifdef CONFIG_AAH_TIMESYNC_DEBUG
		(*(pdata->register_timesync_event_handler))(NULL, NULL);
#endif
		kfree(tdata);
	} else
		platform_set_drvdata(pdev, tdata);
	return err;
}

static int __devexit timesync_remove(struct platform_device *pdev)
{
	struct timesync_data *tdata = dev_get_drvdata(&pdev->dev);

	pr_info("%s\n", __func__);

#ifdef CONFIG_AAH_TIMESYNC_DEBUG
	/* unregister our callback for the platform's timesync event. */
	(*(tdata->pdata->register_timesync_event_handler))(NULL, NULL);
#endif

	/* unregister the device */
	misc_deregister(&tdata->misc_dev);

	aah_clock_shutdown();
	dev_set_drvdata(&pdev->dev, NULL);
	kfree(tdata);

	return 0;
}

static struct platform_driver timesync_driver = {
	.probe		= timesync_probe,
	.remove		= __devexit_p(timesync_remove),
	.driver		= {
		.name	= "timesync",
		.owner	= THIS_MODULE,
	},
};

static int __init timesync_init(void)
{
	return platform_driver_register(&timesync_driver);
}
static void __exit timesync_exit(void)
{
	platform_driver_unregister(&timesync_driver);
}


module_init(timesync_init);
module_exit(timesync_exit);

MODULE_DESCRIPTION("timesync driver");
MODULE_LICENSE("GPL");
