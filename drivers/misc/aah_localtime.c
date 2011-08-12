/* drivers/misc/aah_localtime.c
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
#include <linux/aah_localtime.h>

#define DEV_NODE_NAME "aah_localtime"

#ifdef CONFIG_AAH_TIMESYNC_DEBUG
#define MAX_LOG_SIZE 16

struct aah_tsdebug_state {
	struct aah_tsdebug_event_record event_log[MAX_LOG_SIZE];
	u32 event_log_wr;
	u32 event_log_rd;
	u64 local_event_count;
};
#endif

struct aah_localtime_data {
	struct aah_localtime_platform_data *pdata;
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
	struct aah_localtime_data *tdata = user_data;
	struct aah_tsdebug_state *s = &tdata->tsdebug_state;
	struct aah_tsdebug_event_record *e;

	spin_lock_irqsave(&tdata->tsdebug_state_lock, irq_state);

	BUG_ON(s->event_log_wr >= MAX_LOG_SIZE);
	BUG_ON(s->event_log_rd >= MAX_LOG_SIZE);
	e = s->event_log + s->event_log_wr;

	e->local_timesync_event_id = s->local_event_count++;
	e->local_time = raw_event_time;

	/* Advance the write pointer.  If it wraps and hits the read pointer
	 * again, move the read pointer up dropping the oldest entry in the log
	 * in the process.
	 */
	s->event_log_wr = (s->event_log_wr + 1) % MAX_LOG_SIZE;
	if (s->event_log_wr == s->event_log_rd)
		s->event_log_rd = (s->event_log_rd + 1) % MAX_LOG_SIZE;

	spin_unlock_irqrestore(&tdata->tsdebug_state_lock, irq_state);
}

static int get_tsdebug_events(struct aah_localtime_data *tdata,
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

static int aah_localtime_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static long aah_localtime_ioctl(struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	struct aah_localtime_data *tdata = container_of(file->private_data,
						   struct aah_localtime_data,
						   misc_dev);
	switch (cmd) {
	case AAHLT_IOCTL_LOCALTIME_GET: {
		u64 counter = (*tdata->pdata->get_raw_counter)();
		if (copy_to_user((void __user *) arg,
				&counter, sizeof(counter))) {
			return -EFAULT;
		}
	} break;

	case AAHLT_IOCTL_LOCALTIME_GETFREQ: {
		u64 freq = (*tdata->pdata->get_raw_counter_nominal_freq)();
		if (copy_to_user((void __user *) arg,
				&freq, sizeof(freq))) {
			return -EFAULT;
		}
	} break;

	case AAHLT_IOCTL_LOCALTIME_SET_SLEW: {
		s16 rate = (s16)arg;

		if (!tdata->pdata->set_counter_slew_rate)
			return -EINVAL;

		tdata->pdata->set_counter_slew_rate(rate);
	} break;

#ifdef CONFIG_AAH_TIMESYNC_DEBUG
	case AAHLT_IOCTL_FETCH_TSDEBUG_RECORDS: {
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

static const struct file_operations aah_localtime_fops = {
	.owner = THIS_MODULE,
	.open = aah_localtime_open,
	.unlocked_ioctl = aah_localtime_ioctl,
};

static int __devinit aah_localtime_probe(struct platform_device *pdev)
{
	int err;
	struct aah_localtime_platform_data *pdata = pdev->dev.platform_data;
	struct aah_localtime_data *tdata;

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

	tdata = kzalloc(sizeof(struct aah_localtime_data), GFP_KERNEL);
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

	/* register misc device */
	tdata->misc_dev.minor = MISC_DYNAMIC_MINOR;
	tdata->misc_dev.name = DEV_NODE_NAME;
	tdata->misc_dev.fops = &aah_localtime_fops;
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

static int __devexit aah_localtime_remove(struct platform_device *pdev)
{
	struct aah_localtime_data *tdata = dev_get_drvdata(&pdev->dev);

	pr_info("%s\n", __func__);

#ifdef CONFIG_AAH_TIMESYNC_DEBUG
	/* unregister our callback for the platform's timesync event. */
	(*(tdata->pdata->register_timesync_event_handler))(NULL, NULL);
#endif

	/* unregister the device */
	misc_deregister(&tdata->misc_dev);

	dev_set_drvdata(&pdev->dev, NULL);
	kfree(tdata);

	return 0;
}

static struct platform_driver aah_localtime_driver = {
	.probe		= aah_localtime_probe,
	.remove		= __devexit_p(aah_localtime_remove),
	.driver		= {
		.name	= DEV_NODE_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init aah_localtime_init(void)
{
	return platform_driver_register(&aah_localtime_driver);
}
static void __exit aah_localtime_exit(void)
{
	platform_driver_unregister(&aah_localtime_driver);
}


module_init(aah_localtime_init);
module_exit(aah_localtime_exit);

MODULE_DESCRIPTION("aah localtime driver");
MODULE_LICENSE("GPL");
