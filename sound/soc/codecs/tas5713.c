/* sound/soc/codecs/tas5713.c
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/tas5713.h>
#include <sound/soc.h>

#include "tas5713_reg_init.h"
#include "tas5713_debugfs.h"

#define DRV_NAME "tas5713"

enum tas5713_pwr_state {
	kPoweredDown,
	kPoweredUp
};

struct tas5713_driver_state {
	struct mutex lock;
	struct i2c_client *i2c_client;
	struct tas5713_platform_data *pdata;
	enum tas5713_pwr_state cur_power_state;
	enum tas5713_pwr_state tgt_power_state;
	int power_transition_active;
	u8 vol_reg;

	struct workqueue_struct* pwr_mgmt_workqueue;
	struct work_struct pwr_mgmt_workitem;
	spinlock_t pwr_mgmt_state_lock;

#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_dir;
	struct dentry *debugfs_dump_regs_node;
	struct dentry *debugfs_err_reg_node;
#endif
};

/* From the TAS5713 datasheet, in the section titled "Shutdown Sequence" on page
 * 38 of the August 2010 revision of the document, when entering or exiting the
 * shutdown state, after writing the appropriate value (0x40 or 0x00) to System
 * Control Register 2, the host is supposed to wait at least 1 mSec + 1.3 *
 * tStart, where tStart is the value specified by the Start/Stop period
 * register.  We use the default value of 125.7 mSec for the Start/Stop period
 * and currently do not allow it to be adjusted, so our total wait time in uSec
 * works out to be
 * 1000 + (1.3 * 125700) == 1000 + (13 * 12570)
 */
static const int kEnterExitShutdownWaitTimeUSec = (1000 + (13 * 12570));

/* From the TAS5713 datasheet, in the section titled "Power-Down Sequence" on
 * page 38 of the August 2010 revision of the document, in the case of sudden
 * power loss, the host is supposed to wait 2 mSec after asserting the external
 * power-down signal, but before assering the external reset signal.
 */
static const int kSuddenPDNWaitTime = 2000;

/* From the TAS5713 datasheet, in the section titled "Initialization Sequence"
 * on page 37 of the August 2010 revision of the document, in the case of sudden
 * power loss, the host is supposed to wait 100uSec after deasserting the
 * external power down signal before deasserting the external reset signal.
 * Then after deasserting reset, it is to wait 13.5mSec before attempting to
 * trim the oscillator.  Finally, after trimming the oscillator, it is supposed
 * to wait 50mSec before proceeding with configuring the device.
 */
static const int kInitSeqPostPDNWaitTime = 100;
static const int kInitSeqPostResetWaitTime = 13500;
static const int kInitSeqPostOscTrimWaitTime = 50000;

/* If we fail to power up the TAS5713 asynchronously for whatever reason, wait
 * at least 100mSec before trying again.  Number was chosen arbitrarily.
 */
static const int kPowerUpRetryTimeout = 100000;

#ifdef CONFIG_DEBUG_FS
static void *tas5713_dump_regs_seq_start(struct seq_file *s, loff_t *pos)
{
	loff_t *iter;

	if (*pos >= ARRAY_SIZE(tas5713_debugfs_registers))
		return NULL;

	iter = kmalloc(sizeof(loff_t), GFP_KERNEL);
	if (NULL == iter)
		return NULL;
	*iter = *pos;
	return iter;
}

static void *tas5713_dump_regs_seq_next(struct seq_file *s,
		void *v, loff_t *pos)
{
	loff_t *iter = (loff_t*)v;
	*pos = ++(*iter);

	if ((*pos) >= ARRAY_SIZE(tas5713_debugfs_registers))
		return NULL;

	return v;
}

static void tas5713_dump_regs_seq_stop(struct seq_file *s, void *v)
{
	kfree(v);
}

static int tas5713_dump_regs_seq_show(struct seq_file *s, void *v)
{
	struct tas5713_driver_state *state = s->private;
	const struct tas5713_debugfs_register* reg;
	loff_t *iter = (loff_t*)v;

	if ((*iter) >= ARRAY_SIZE(tas5713_debugfs_registers))
		return -1;

	reg = &(tas5713_debugfs_registers[*iter]);

	seq_printf(s, "%33s[%02x] :", reg->name, reg->addr);

	mutex_lock(&state->lock);
	if (kPoweredUp == state->cur_power_state) {
		u8 buf[32];
		int ret;
		int amt = (sizeof(buf) < reg->len) ? sizeof(buf) : reg->len;

		ret = i2c_smbus_read_i2c_block_data(state->i2c_client,
				reg->addr, amt, buf);

		if (ret >= 0) {
			int i;
			for (i = 0; i < ret; ++i) {
				if (!(i % 4))
					seq_putc(s, ' ');
				seq_printf(s, "%02x", buf[i]);
			}
			seq_printf(s, "\n");
		} else {
			seq_printf(s, " I2C error on read (%d)\n", ret);
		}
	} else {
		seq_printf(s, " (powered down)\n");
	}
	mutex_unlock(&state->lock);

	return 0;
}

static const struct seq_operations tas5713_dump_regs_seq_ops = {
        .start = tas5713_dump_regs_seq_start,
        .next  = tas5713_dump_regs_seq_next,
        .stop  = tas5713_dump_regs_seq_stop,
        .show  = tas5713_dump_regs_seq_show
};

static int tas5713_dump_regs_open(struct inode *inode, struct file *file)
{
	int ret = seq_open(file, &tas5713_dump_regs_seq_ops);

	if (ret >= 0) {
		struct seq_file *sf = file->private_data;
		sf->private = inode->i_private;
	}

	return ret;
}

static const struct file_operations tas5713_dump_regs_file_ops = {
        .owner   = THIS_MODULE,
        .open    = tas5713_dump_regs_open,
        .read    = seq_read,
        .llseek  = seq_lseek,
        .release = seq_release
};

static int tas5713_get_err_reg(void *__state, u64 *out)
{
	struct tas5713_driver_state *state = __state;
	mutex_lock(&state->lock);

	if (kPoweredUp == state->cur_power_state) {
		int ret = i2c_smbus_read_byte_data(state->i2c_client, 0x02);
		if (ret < 0)
			*out = 0xFF01;
		else
			*out = ret;

	} else {
		*out = 0xFF00;
	}

	mutex_unlock(&state->lock);
	return 0;
}

static int tas5713_set_err_reg(void *__state, u64 __unused__)
{
	struct tas5713_driver_state *state = __state;
	mutex_lock(&state->lock);

	i2c_smbus_write_byte_data(state->i2c_client, 0x02, 0x00);

	mutex_unlock(&state->lock);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(tas5713_err_reg_fops,
		tas5713_get_err_reg,
		tas5713_set_err_reg,
		"0x%llx");

static void tas5713_cleanup_debugfs(struct tas5713_driver_state *state)
{
	if (IS_ERR_OR_NULL(state->debugfs_dir))
		debugfs_remove_recursive(state->debugfs_dir);

	state->debugfs_dir = NULL;
	state->debugfs_dump_regs_node = NULL;
	state->debugfs_err_reg_node = NULL;
}

static void tas5713_setup_debugfs(struct tas5713_driver_state *state)
{
	struct device *dev = &state->i2c_client->dev;
	char tmp[256];

	snprintf(tmp, sizeof(tmp), "%s-%s",
			dev_driver_string(dev), dev_name(dev));
	tmp[sizeof(tmp) - 1] = 0;
	state->debugfs_dir = debugfs_create_dir(tmp, NULL);

	if (IS_ERR_OR_NULL(state->debugfs_dir))
		goto err;

	state->debugfs_err_reg_node = debugfs_create_file(
			"err_reg", 0644, state->debugfs_dir,
			state, &tas5713_err_reg_fops);

	if (IS_ERR_OR_NULL(state->debugfs_err_reg_node))
		goto err;

	state->debugfs_dump_regs_node = debugfs_create_file(
			"dump_regs", 0444, state->debugfs_dir,
			state, &tas5713_dump_regs_file_ops);

	if (IS_ERR_OR_NULL(state->debugfs_dump_regs_node))
		goto err;

	return;
err:
	tas5713_cleanup_debugfs(state);
}
#else  /* CONFIG_DEBUG_FS */
static void tas5713_setup_debugfs(struct tas5713_driver_state*) { }
static void tas5713_cleanup_debugfs(struct tas5713_driver_state*) { }
#endif  /* CONFIG_DEBUG_FS */

static void do_before_power_up(struct tas5713_driver_state* state) {
	if (state && state->pdata && state->pdata->before_power_up)
		state->pdata->before_power_up(state->pdata);
}

static void do_after_power_down(struct tas5713_driver_state* state) {
	if (state && state->pdata && state->pdata->after_power_down)
		state->pdata->after_power_down(state->pdata);
}

static int tas5713_set_master_volume_l(struct device* dev,
	struct tas5713_driver_state* state,
	u8 vol_reg)
{
	int ret;

	state->vol_reg = vol_reg;
	ret = i2c_smbus_write_byte_data(state->i2c_client, 0x07, vol_reg);

	if (ret < 0)
		dev_err(dev, "I2C failure (%d) while trying to set master"
				" volume.\n", ret);
	else
		ret = 0;

	return ret;
}

#define DO_RESET_DELAY(amt) \
	do { \
		if (release_lock_during_wait) \
			mutex_unlock(&state->lock); \
		usleep_range(amt, amt); \
		if (release_lock_during_wait) \
			mutex_lock(&state->lock); \
	} while (0)
static int tas5713_do_reset(
		struct tas5713_driver_state* state,
		int start_mclk,
		int release_lock_during_wait)
{
	int ret = 0;

	/* Start by making absolutely certain that the reset and power down
	 * signals are asserted.  Wait 2mSec after asserting pdn/reset to make
	 * sure everything is properly reset before proceeding.
	 */
	gpio_set_value(state->pdata->pdn_gpio, 0);
	gpio_set_value(state->pdata->reset_gpio, 0);
	DO_RESET_DELAY(kSuddenPDNWaitTime);

	if (start_mclk) {
		/* Turn on MCLK */
		ret = clk_enable(state->pdata->mclk_out);
		if (ret) {
			struct device* dev = &state->i2c_client->dev;
			dev_err(dev, "Failed to enable MCLK (ret = %d)\n", ret);
			goto bailout;
		}
	}

	/* Deassert the power down signal and wait the specified 100uSec */
	gpio_set_value(state->pdata->pdn_gpio, 1);
	DO_RESET_DELAY(kInitSeqPostPDNWaitTime);

	/* Now deassert the reset signal and wait the specified 13.5mSec */
	gpio_set_value(state->pdata->reset_gpio, 1);
	DO_RESET_DELAY(kInitSeqPostResetWaitTime);

bailout:
	return ret;
}
#undef DO_RESET_DELAY

static int tas5713_check_device_exists(struct device* dev,
		struct tas5713_driver_state* state)
{
	int ret;
	/* Reset the device.  No need to turn on MCLK or trim the oscillator
	 * here sinve we are just going to read the device ID register.  The
	 * internally tas5713 generated MCLK will be good enough for I2C
	 * acceess.
	 */
	do_before_power_up(state);
	tas5713_do_reset(state, 0, 0);

	/* Check to see that the device exists by attempting to read the device
	 * ID register at sub-address 0x01.
	 */
	ret = i2c_smbus_read_byte_data(state->i2c_client, 0x01);
	if (ret >= 0)  {
		dev_info(dev, "Found TAS5713 device with dev ID = 0x%02x\n",
				ret);
		ret = 0;
	} else {
		dev_err(dev, "Failed to read TAS5713 device ID register.  "
				"(ret = %d)\n", ret);
		ret = -ENODEV;
	}

	/* Place the device back into power-down/reset. */
	gpio_set_value(state->pdata->pdn_gpio, 0);
	gpio_set_value(state->pdata->reset_gpio, 0);
	do_after_power_down(state);

	return ret;
}

static void tas5713_power_up(struct tas5713_driver_state* state)
{
	struct tas5713_platform_data* pdata = state->pdata;
	struct device* dev = &state->i2c_client->dev;
	int i, ret;

	/* Note: the state->lock is already held by tas5713_power_transition
	 * when we enter this function.  We only need to release it during
	 * delays, and need to be sure to hold it upon exit.
	 */

	BUG_ON(!pdata);
	BUG_ON(!pdata->mclk_out);

	/* Make sure the device has been properly reset and that mclk has been
	 * enabled.  If something goes wrong, tas5713_do_reset should have
	 * already logged an error.
	 */
	ret = tas5713_do_reset(state, 1, 1);
	if (ret)
		goto err_setup;

	/* Select factory trimming for the interal OSC used to detect sample
	 * rates, then wait the spec'ed 50mSec before proceeding.
	 */
	ret = i2c_smbus_write_byte_data(state->i2c_client, 0x1B, 0x00);
	if (ret < 0) {
		dev_err(dev, "I2C failure (%d) while trying to trim"
				" oscillator\n", ret);
		goto err_setup;
	}
	mutex_unlock(&state->lock);
	usleep_range(kInitSeqPostOscTrimWaitTime, kInitSeqPostOscTrimWaitTime);
	mutex_lock(&state->lock);

	/* Now program in the default initialization */
	for (i = 0; i < ARRAY_SIZE(tas5713_init_sequence); ++i) {
		ret = i2c_master_send(state->i2c_client,
				     tas5713_init_sequence[i].data,
				     tas5713_init_sequence[i].size);

		if (ret < 0) {
			dev_err(dev, "I2C failure (%d) while trying to"
					" write register 0x%02x\n", ret,
					tas5713_init_sequence[i].data[0]);
			goto err_setup;
		}
	}

	/* Exit shutdown and wait for the required amt of time after exiting
	 * shutdown.
	 */
	ret = i2c_smbus_write_byte_data(state->i2c_client, 0x05, 0x00);
	if (ret < 0) {
		dev_err(dev, "I2C failure (%d) while trying to exit"
				" shutdown\n", ret);
		goto err_setup;
	}
	mutex_unlock(&state->lock);
	usleep_range(kEnterExitShutdownWaitTimeUSec,
			kEnterExitShutdownWaitTimeUSec);
	mutex_lock(&state->lock);

	/* Finally, restore the master volume to whatever it was when we were
	 * started up last time.  If we encounter an error, don't bother
	 * logging.  It has been done already by the set_master_volume function.
	 */
	ret = tas5713_set_master_volume_l(dev, state, state->vol_reg);
	if (ret < 0)
		goto err_setup;

	/* Flag ourselves as being officially powered up and get out. */
	state->cur_power_state = kPoweredUp;

	return;

err_setup:
	clk_disable(pdata->mclk_out);

	/* Something went wrong?  Follow the power down sequence as given by the
	 * datasheet.  We don't whether or not communications with the TAS5713
	 * via I2C is possible right now, so we have to skip the shutdown phase
	 * and go directly to the sudden power loss sequence.
	 */
	gpio_set_value(pdata->pdn_gpio, 0);
	mutex_unlock(&state->lock);
	usleep_range(kSuddenPDNWaitTime, kSuddenPDNWaitTime);
	mutex_lock(&state->lock);
	gpio_set_value(pdata->reset_gpio, 0);

	/* Assert reset, flag ourselves as being powered down, then wait
	 * 100mSec.  It is very likely that our target state is still
	 * powered-up.  Its a good idea to try again, but it might be best to
	 * wait a little bit before doing so. */
	gpio_set_value(pdata->reset_gpio, 0);
	state->cur_power_state = kPoweredDown;

	mutex_unlock(&state->lock);
	do_after_power_down(state);
	usleep_range(kPowerUpRetryTimeout, kPowerUpRetryTimeout);
	mutex_lock(&state->lock);
}

static void tas5713_power_down(struct tas5713_driver_state* state)
{
	struct device* dev = &state->i2c_client->dev;
	int ret;

	/* Note: the state->lock is already held by tas5713_power_transition
	 * when we enter this function.  We only need to release it during
	 * delays, and need to be sure to hold it upon exit.
	 *
	 * Start by flagging ourselves as already powered down.  No matter what
	 * happens, power down will always succeed.  Change the flag first so
	 * that observers of power state (like set master volume) don't try to
	 * talk to the TAS5713 in the middle of a power down operation.
	 */
	state->cur_power_state = kPoweredDown;

	/* Enter shutdown by writing the spec'ed value of 0x40 to System Control
	 * Register 2 (sub-address 0x05) and wait for the required amt of time
	 * after entering shutdown.
	 */
	ret = i2c_smbus_write_byte_data(state->i2c_client, 0x05, 0x40);
	if (ret < 0) {
		dev_err(dev, "I2C failure (%d) while trying to enter"
				" shutdown\n", ret);

		/* We failed to cleanly enter the shutdown state.  Follow the
		 * sudden power loss sequence (waiting 2mSec after asserting PDN
		 * before asserting Reset)
		 */
		gpio_set_value(state->pdata->pdn_gpio, 0);
		mutex_unlock(&state->lock);
		usleep_range(kSuddenPDNWaitTime, kSuddenPDNWaitTime);
		mutex_lock(&state->lock);
	} else {
		mutex_unlock(&state->lock);
		usleep_range(kEnterExitShutdownWaitTimeUSec,
				kEnterExitShutdownWaitTimeUSec);
		mutex_lock(&state->lock);

		/* Assert power down, reset, and then shut off MCLK.  We should
		 * not have to wait between power down and reset since we
		 * already cleanly entered the shutdown state via i2c commands.
		 */
		gpio_set_value(state->pdata->pdn_gpio, 0);
	}

	gpio_set_value(state->pdata->reset_gpio, 0);

	/* Make sure MCLK is now shut off and we are finished. */
	clk_disable(state->pdata->mclk_out);
}

static void tas5713_power_transition(struct work_struct* work)
{
	struct device* dev;
	struct tas5713_driver_state* state;
	unsigned long irq_flags;
	spinlock_t* pwr_lock;

	state = container_of(work, struct tas5713_driver_state,
			pwr_mgmt_workitem);
	dev = &state->i2c_client->dev;
	pwr_lock = &state->pwr_mgmt_state_lock;

	mutex_lock(&state->lock);
	spin_lock_irqsave(pwr_lock, irq_flags);
	while (1) {
		/* Start by checking to see if our current state matches our
		 * target state.  We use the spinlock to observe the target
		 * state because ASoC has IRQs off when it calls tas5713_trigger
		 * (the writer for the target state).  All other synchonization
		 * is performed via the driver state's mutex.
		 */
		if (state->tgt_power_state == state->cur_power_state)
			break;

		if (state->tgt_power_state == kPoweredDown) {
			spin_unlock_irqrestore(pwr_lock, irq_flags);
			tas5713_power_down(state);
			do_after_power_down(state);
			spin_lock_irqsave(pwr_lock, irq_flags);
	  	} else if (state->tgt_power_state == kPoweredUp) {
			spin_unlock_irqrestore(pwr_lock, irq_flags);
			do_before_power_up(state);
			tas5713_power_up(state);
			spin_lock_irqsave(pwr_lock, irq_flags);
	  	} else {
			dev_err(dev, "Unrecognized target state %d in %s\n",
					state->tgt_power_state,
					__PRETTY_FUNCTION__);
			break;
		}
	}
	state->power_transition_active = 0;
	spin_unlock_irqrestore(pwr_lock, irq_flags);
	mutex_unlock(&state->lock);
}

static int tas5713_trigger(struct snd_pcm_substream *substream,
		int cmd, struct snd_soc_dai *dai) {
	struct snd_soc_codec* codec = dai->codec;
	struct tas5713_driver_state* state = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_START: {
		unsigned long irq_flags;
		spin_lock_irqsave(&state->pwr_mgmt_state_lock, irq_flags);

		state->tgt_power_state = (cmd == SNDRV_PCM_TRIGGER_STOP)
			? kPoweredDown
			: kPoweredUp;

		if (!state->power_transition_active) {
			state->power_transition_active = 1;
			queue_work(state->pwr_mgmt_workqueue,
					&state->pwr_mgmt_workitem);
		}

		spin_unlock_irqrestore(&state->pwr_mgmt_state_lock, irq_flags);
	} break;

	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int tas5713_ioctl(
		struct snd_pcm_substream* substream,
		struct snd_soc_dai* dai,
		unsigned int cmd, void *arg)
{
	struct snd_soc_codec* codec = dai->codec;
	struct tas5713_driver_state* state = snd_soc_codec_get_drvdata(codec);
	struct device* dev = &state->i2c_client->dev;
	int ret = -ENOIOCTLCMD;

	mutex_lock(&state->lock);

	switch (cmd) {
	case TAS5713_GET_MASTER_VOLUME: {
		u8 vol = state->vol_reg;

		if (copy_to_user((void __user *)arg,
				&vol, sizeof(vol)))
			ret = -EFAULT;
		else
			ret = 0;
	} break;

	case TAS5713_SET_MASTER_VOLUME: {
		u8 vol;

		if (copy_from_user(&vol, (const void __user *)arg,
					sizeof(vol))) {
			ret = -EFAULT;
			break;
		}

		if (kPoweredDown == state->cur_power_state) {
			state->vol_reg = vol;
			ret = 0;
		} else {
			ret = tas5713_set_master_volume_l(dev, state, vol);
		}
	} break;

	default: {
		ret = -ENOIOCTLCMD;
	} break;
	}

	mutex_unlock(&state->lock);

	return ret;
}

struct snd_soc_dai_ops tas5713_dai_ops = {
	.trigger = tas5713_trigger,
	.ioctl = tas5713_ioctl,
};

static struct snd_soc_codec_driver soc_codec_tas5713;
static struct snd_soc_dai_driver tas5713_dai = {
	.name		= "tas5713-codec",
	.playback 	= {
		.stream_name	= "Playback",
		.channels_min	= 2,
		.channels_max	= 2,
		.rates		= 0,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &tas5713_dai_ops,
};

static int tas5713_compute_available_sample_rates(
		struct tas5713_driver_state *state,
		struct snd_soc_dai_driver* dai)
{
	/* A table of the sample rates permitted by the tas5713 */
	static const int allowed_tas5713_sr_table[][2] = {
		{ 8000, SNDRV_PCM_RATE_8000 },
		{ 11025, SNDRV_PCM_RATE_11025 },
		{ 16000, SNDRV_PCM_RATE_16000 },
		{ 22050, SNDRV_PCM_RATE_22050 },
		{ 44100, SNDRV_PCM_RATE_44100 },
		{ 48000, SNDRV_PCM_RATE_48000 },
	};

	/* A table of the mclk multipliers permitted by the tas5713 */
	static const int allowed_tas5713_mclk_mults[] = {
		64, 128, 192, 256, 384, 512
	};

	struct device* dev = &state->i2c_client->dev;
	int i, ret = 0, rates = 0;
	long mclk_rate;

	if (!state->pdata->mclk_out) {
		dev_err(dev, "Cannot compute available sample rates, missing"
				" mclk\n");
		ret = -ENODEV;
		goto bailout;
	}

	/* Enable the clock so we can grab its rate, then shut it off again. */
	ret = clk_enable(state->pdata->mclk_out);
	if (ret) {
		dev_err(dev, "Failed to enable mclk to compute available sample"
				" rates. (ret = %d)\n", ret);
		goto bailout;
	}

	mclk_rate = clk_get_rate(state->pdata->mclk_out);
	clk_disable(state->pdata->mclk_out);

	for (i = 0; i < ARRAY_SIZE(allowed_tas5713_sr_table); ++i) {
		int tgt  = allowed_tas5713_sr_table[i][0];
		int flag = allowed_tas5713_sr_table[i][1];
		int j;

		for (j = 0; j < ARRAY_SIZE(allowed_tas5713_mclk_mults); ++j) {
			int mult = allowed_tas5713_mclk_mults[j];
			if (!(mclk_rate % mult) && ((mclk_rate / mult) == tgt))
			{
				rates |= flag;
				break;
			}
		}
	}

	if (!rates) {
		dev_err(dev, "Failed to any valid sample rates\n");
		ret = -ENODEV;
		goto bailout;
	}

bailout:
	dai->playback.rates = rates;
	return ret;
}

static int tas5713_i2c_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct tas5713_driver_state *state = NULL;
	struct device* dev = &client->dev;
	int ret;

	state = kzalloc(sizeof(struct tas5713_driver_state), GFP_KERNEL);
	if (!state) {
		ret = -ENOMEM;
		dev_err(dev, "probe: failed to allocate driver state\n");
		goto err_state_alloc;
	}

	i2c_set_clientdata(client, state);

	mutex_init(&state->lock);
	spin_lock_init(&state->pwr_mgmt_state_lock);
	state->i2c_client = client;
	state->pdata = client->dev.platform_data;
	state->cur_power_state = kPoweredDown;
	state->tgt_power_state = kPoweredDown;
	state->power_transition_active = 0;
	state->vol_reg = 0xFF;
	state->pwr_mgmt_workqueue = alloc_workqueue("tas5713_pwr_mgmt", 0, 1);
	INIT_WORK(&state->pwr_mgmt_workitem, tas5713_power_transition);

	if (NULL == state->pwr_mgmt_workqueue) {
		ret = -ENOMEM;
		dev_err(dev, "probe: failed to allocate driver state\n");
		goto err_workqueue_alloc;
	}

	if (state->pdata == NULL) {
		ret = -ENODEV;
		dev_err(dev, "probe: missing platform data\n");
		goto err_missing_platform_data;
	}

	ret = gpio_request_one(state->pdata->reset_gpio,
			      GPIOF_OUT_INIT_LOW,
			      "tas5713_reset");
	if (ret) {
		dev_err(dev, "probe: gpio_request_one(reset_gpio = %d) failed"
				" with error %d\n",
				state->pdata->reset_gpio, ret);
		goto err_request_reset_gpio;
	}

	ret = gpio_request_one(state->pdata->pdn_gpio,
			      GPIOF_OUT_INIT_LOW,
			      "tas5713_pdn");
	if (ret) {
		dev_err(dev, "probe: gpio_request_one(pdn_gpio = %d) failed"
				" with error %d\n",
				state->pdata->pdn_gpio, ret);
		goto err_request_pdn_gpio;
	}

	ret = tas5713_check_device_exists(dev, state);
	if (ret)
		goto err_device_not_found;

	ret = tas5713_compute_available_sample_rates(state, &tas5713_dai);
	if (ret)
		/* No need to log an error, its already been done by the compute
		 * sample rates function.
		 */
		goto err_compute_sample_rates;

	ret = snd_soc_register_codec(
			&client->dev,
			&soc_codec_tas5713,
			&tas5713_dai, 1);
	if (ret) {
		dev_err(dev, "probe: snd_soc_register_codec failed with error"
				" %d\n", ret);
		goto err_register_codec;
	}

	tas5713_setup_debugfs(state);

	return 0;

err_register_codec:
err_device_not_found:
err_compute_sample_rates:
	gpio_free(state->pdata->pdn_gpio);
err_request_pdn_gpio:
	gpio_free(state->pdata->reset_gpio);
err_request_reset_gpio:
err_missing_platform_data:
	destroy_workqueue(state->pwr_mgmt_workqueue);
	state->pwr_mgmt_workqueue = NULL;
err_workqueue_alloc:
	i2c_set_clientdata(client, NULL);
	kfree(state);
err_state_alloc:
	return ret;
}

static int __devexit tas5713_i2c_remove(struct i2c_client *client)
{
	struct tas5713_driver_state *state = i2c_get_clientdata(client);

	snd_soc_unregister_codec(&client->dev);
	i2c_set_clientdata(client, NULL);

	tas5713_cleanup_debugfs(state);

	if (state) {
		flush_workqueue(state->pwr_mgmt_workqueue);
		destroy_workqueue(state->pwr_mgmt_workqueue);
		state->pwr_mgmt_workqueue = NULL;

		gpio_free(state->pdata->pdn_gpio);
		gpio_free(state->pdata->reset_gpio);
		kfree(state);
	}

	return 0;
}

static struct i2c_device_id tas5713_idtable[] = {
	{ DRV_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tas5713_idtable);

static struct i2c_driver tas5713_driver = {
	.driver = {
		.name = DRV_NAME,
	},

	.id_table = tas5713_idtable,
	.probe = tas5713_i2c_probe,
	.remove = __devexit_p(tas5713_i2c_remove),

	.shutdown = NULL,
	.suspend = NULL,
	.resume = NULL,
};

static int tas5713_init(void)
{
	return i2c_add_driver(&tas5713_driver);
}

static void tas5713_exit(void)
{
	i2c_del_driver(&tas5713_driver);
}

module_init(tas5713_init);
module_exit(tas5713_exit);

MODULE_AUTHOR("John Grossman <johngro@google.com>");
MODULE_DESCRIPTION("TAS5713 codec driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
