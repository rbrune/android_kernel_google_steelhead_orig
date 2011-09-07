/* drivers/misc/tas5713.c
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/tas5713.h>
#include <linux/uaccess.h>
#include <plat/dma.h>
#include <plat/mcbsp.h>

MODULE_LICENSE("Dual BSD/GPL");

#define PCM_BUFFER_MAX_SIZE_ORDER PAGE_SHIFT

#define I2S_MAX_NUM_BUFS 4
#define I2S_DEFAULT_TX_NUM_BUFS 2

/* assume 48KHz, 16bps, stereo */
#define BYTES_PER_OUTPUT_SLOT 4
#define SAMPLE_RATE 48000

/* McBSP transmit buffer threshold */
#define MCBSP_TX_THRESHOLD 16

/* McBSP IRQSTATUS register */
#define XUNDFLSTAT 0x0800

/* McBSP IRQENABLE register */
#define XUNDFLEN 0x0800

#define LOG_PREFIX KERN_INFO "TAS5713 : "
#define ERR_LOG_PREFIX KERN_ERR "TAS5713 : "

/* This #def controls a workaround to an issue in HW caused by the 1.8V <-> 3.3v
 * level translator between the OMAP 44xx chip and the TAS5713 chip.  Basically,
 * when android is done sending audio, it sends 3 seconds worth of silence
 * before finally letting the low level underflow.  When the McBSP feeding the
 * TAS underflows, we need to disable the transmit serializer, both to suppress
 * the underflow interrupt as well as to keep the serializer form clocking out
 * data as the pipeline buffers up but before it starts (we need to control the
 * start time very exactly to get our time sync correct).
 *
 * When the serializer is disabled (by setting the XDISABLE bit to 1 in the XCCR
 * register), the OMAP appears to tristate its output instead of just leaving it
 * driven to what it was before.  It does this in sync with the start of the
 * next frame of audio (a very good thing).  Unfortunately, there are pull-ups
 * on both sides of the bi-directional level translator which sits between the
 * TAS and the OMAP.  The line starts to float up for about 1 bit time.
 * Eventually, it crosses the level translator threshold and pops into the high
 * state.  After this, it stays high until the next time the TX serializer
 * starts up.  This means that all 0s are being clocked out just before the
 * underflow, and all 1s just after.  The TAS is seeing 0 for each channel, and
 * then -1 after the underflow.  Right at the underflow, however, the MSB of the
 * left hand channel gets latched as a 0.  By the time the next bit is latched,
 * the line has been pulled up by the level translator and is now stuck high.
 * This means the L, R sequence in the transition from silence to underflow
 * looks like
 *
 * (0, 0), (0, 0), (0, 0), (0x7FFF, -1), (-1, -1), (-1, -1)...
 *
 * which causes a pop in the left hand channel output.
 *
 * For now, we implement the following workaround.  When disabling the
 * serializer, we use the legacy GPIO mode of the McBSP to force the data line
 * to 0, and to disable the frame sync clock.  Holding the data line low keeps
 * "silence" being clocked out, and disableing the frame sync clock should cause
 * the TAS's soft stop logic to engage and do a smooth rampdown.  When enabling
 * the serializer, we first do the enable, then take the pins out of GPIO mode.
 * The TAS sees a garbage initial frame (from an LR clock perspective), but soon
 * will see a good LR clock and perform a soft startup.  In either case, this
 * makes the pops go away.
 *
 * TODO : remove this workaround when rev3 digital boards come back with a
 * pulldown on the 44xx side of the McBSP DX line.
 */
#define LEVEL_TRANSLATOR_POP_WORKAROUND

struct i2s_omap_dma_request {
	struct list_head node;
	struct tas5713_driver_state *state;
	int buf_num;
	unsigned int size;
};

struct tas5713_driver_state {
	int opened;
	int write_in_progress;
	struct mutex lock;

	/* I2C client/device and platform specific data (BSP and GPIO
	 * selection)
	 */
	struct i2c_client *i2c_client;
	struct tas5713_platform_data *pdata;

	/* Device node registation */
	struct miscdevice misc_out;
	struct miscdevice misc_out_ctl;

	/* I2S book-keeping */
	int num_bufs;
	void *buffer[I2S_MAX_NUM_BUFS];
	dma_addr_t buf_phy[I2S_MAX_NUM_BUFS];
	struct completion comp[I2S_MAX_NUM_BUFS];
	struct i2s_omap_dma_request buf_req[I2S_MAX_NUM_BUFS];
	int last_queued;

	int dma_lch;
	bool stop;
	struct completion stop_completion;
	struct list_head dma_req_queue;
	spinlock_t dma_req_lock;
	u8 last_master_vol_reg;

	u64 dma_start_local_time;
	atomic64_t samples_queued_to_dma;
};

static inline bool pending_buffer_requests(struct tas5713_driver_state *state)
{
	int i;
	for (i = 0; i < state->num_bufs; i++)
		if (!completion_done(&state->comp[i]))
			return true;
	return false;
}

static inline int buf_size(
		   struct tas5713_driver_state *s __attribute__((unused)))
{
	return 1 << PCM_BUFFER_MAX_SIZE_ORDER;
}

static inline struct tas5713_driver_state *state_from_misc_out(
							struct file *file)
{
	struct miscdevice *m = file->private_data;
	struct tas5713_driver_state *state;

	BUG_ON(!m);

	state = container_of(m, struct tas5713_driver_state, misc_out);
	return state;
}

static inline struct tas5713_driver_state *state_from_misc_out_ctl(
		struct file *file)
{
	struct miscdevice *m = file->private_data;
	struct tas5713_driver_state *state;

	BUG_ON(!m);

	state = container_of(m, struct tas5713_driver_state, misc_out_ctl);
	return state;
}

static inline void prevent_suspend(struct tas5713_driver_state *state)
{
	/* TODO(jsimmons): implement wakelocks */
}

static inline void allow_suspend(struct tas5713_driver_state *state)
{
	/* TODO(jsimmons): implement wakelocks */
}

static int i2s_fifo_enable(struct tas5713_driver_state *state, int on)
{
	unsigned id = state->pdata->mcbsp_id;
	int was_enabled;
	u32 val = omap_mcbsp_read_reg(id, OMAP_MCBSP_REG_XCCR);

#ifdef LEVEL_TRANSLATOR_POP_WORKAROUND
	if (!on) {
		u32 tmp = omap_mcbsp_read_reg(id, OMAP_MCBSP_REG_PCR0);
		tmp &= ~(1 << 5);
		tmp |=  (1 << 13);
		omap_mcbsp_write_reg(id, OMAP_MCBSP_REG_PCR0, tmp);
	}
#endif

	was_enabled = !(val & XDISABLE);
	val &= ~XDISABLE;
	val |= on ? 0 : XDISABLE;
	omap_mcbsp_write_reg(id, OMAP_MCBSP_REG_XCCR, val);

#ifdef LEVEL_TRANSLATOR_POP_WORKAROUND
	if (on) {
		// Delay for at least one 48kHz audio sample worth of output
		// before taking the lines out of GPIO hack mode.
		udelay(25);
		u32 tmp = omap_mcbsp_read_reg(id, OMAP_MCBSP_REG_PCR0);
		tmp &= ~(1 << 13);
		omap_mcbsp_write_reg(id, OMAP_MCBSP_REG_PCR0, tmp);
	}
#endif

	return was_enabled;
}

static int is_i2s_fifo_enabled(struct tas5713_driver_state *state)
{
	unsigned id = state->pdata->mcbsp_id;
	u32 val = omap_mcbsp_read_reg(id, OMAP_MCBSP_REG_XCCR);
	return !(val & XDISABLE);
}

static irqreturn_t tas5713_mcbsp_irq_handler(int irq, void *dev_id)
{
	struct tas5713_driver_state *state = dev_id;
	unsigned id = state->pdata->mcbsp_id;
	int status = omap_mcbsp_read_reg(id, OMAP_MCBSP_REG_IRQST);

	/* When the TX FIFO underruns, disable the FIFO and reset the sample
	 * count tracking state
	 */
	if (status & XUNDFLSTAT) {
		i2s_fifo_enable(state, 0);
		atomic64_set(&state->samples_queued_to_dma, 0);

		/* clear the underflow interrupt */
		omap_mcbsp_write_reg(id, OMAP_MCBSP_REG_IRQST, XUNDFLSTAT);
	}

	return IRQ_HANDLED;
}

static int setup_mcbsp(struct tas5713_driver_state *state)
{
	unsigned id = state->pdata->mcbsp_id;
	struct omap_mcbsp_reg_cfg regs;
	int framesize = 16 * 2;	 /* 16 bits * 2 channels */
	int rc;

	/* choose polled I/O so that the McBSP driver will not grab
	 * the tx IRQ
	 */
	omap_mcbsp_set_io_type(id, OMAP_MCBSP_POLL_IO);

	rc = omap_mcbsp_request(id);
	if (rc)
		return rc;

	rc = omap2_mcbsp_set_clks_src(id, MCBSP_CLKS_PRCM_SRC);
	if (rc) {
		dev_err(&state->i2c_client->dev,
                        "%s: unable to set McBSP clock source\n",
			__func__);
		omap_mcbsp_free(id);
		return rc;
	}

	memset(&regs, 0, sizeof(regs));

	/* McBSP register configuration - based on the I2S configuration
	 * used in the omap-mcbsp ALSA driver
	 */

	/* generic McBSP register settings */
	regs.spcr2 |= XINTM(3) | FREE;
	regs.xccr = DXENDLY(1) | XDMAEN | XDISABLE;

	/* 1-bit data delay for I2S */
	regs.xcr2 |= XDATDLY(1);

	/* McBSP master - set FS and bit clocks as outputs */
	regs.pcr0 |= FSXM | FSRM | CLKXM | CLKRM;

	/* sample rate generator drives the FS */
	regs.srgr2 |= FSGM;

	/* Normal BCLK + FS.
	 * FS active low. TX data driven on falling edge of bit clock
	 * and RX data sampled on rising edge of bit clock.
	 */
	regs.pcr0 |= FSXP | FSRP | CLKXP | CLKRP;

	regs.srgr1 |= CLKGDV(15);

	/* use dual-phase frames */
	regs.xcr2 |= XPHASE;

	/* set 1 word per McBSP frame for phase1 and phase2 */
	regs.xcr1 |= XFRLEN1(0);
	regs.xcr2 |= XFRLEN2(0);

	/* set word lengths for 16bit little endian */
	regs.xcr2 |= XWDLEN2(OMAP_MCBSP_WORD_16);
	regs.xcr1 |= XWDLEN1(OMAP_MCBSP_WORD_16);

	/* set FS period and length in terms of bit clock periods */
	regs.srgr2 |= FPER(framesize - 1);
	regs.srgr1 |= FWID((framesize >> 1) - 1);

	omap_mcbsp_config(id, &regs);

	omap_mcbsp_write_reg(id, OMAP_MCBSP_REG_DXR, 0);

	omap_mcbsp_set_tx_threshold(id, MCBSP_TX_THRESHOLD);

	rc = request_irq(omap_mcbsp_get_tx_irq(id),
			 tas5713_mcbsp_irq_handler,
			 0, "McBSP_i2s", state);
	if (rc) {
		dev_err(&state->i2c_client->dev, "%s: unable to request IRQ\n",
			__func__);
		omap_mcbsp_free(id);
		return rc;
	}

	/* enable interrupt on transmit underflow */
	omap_mcbsp_write_reg(id, OMAP_MCBSP_REG_IRQEN, XUNDFLEN);

	omap_mcbsp_start(id, 1, 0);

	return 0;
}

static void dma_tx_complete_callback(int lch, u16 ch_status, void *data);

/* caller must hold state->dma_req_lock */
static void launch_next_dma(struct tas5713_driver_state *state)
{
	struct i2s_omap_dma_request *req = list_first_entry(
	    &state->dma_req_queue, struct i2s_omap_dma_request, node);

	unsigned int id = state->pdata->mcbsp_id;

	omap_set_dma_transfer_params(state->dma_lch,
				     OMAP_DMA_DATA_TYPE_S16,
				     MCBSP_TX_THRESHOLD,
				     req->size / (MCBSP_TX_THRESHOLD * 2),
				     OMAP_DMA_SYNC_FRAME,
				     omap_mcbsp_dma_ch_params(id, 0),
				     0);

	omap_set_dma_dest_params(state->dma_lch,
				 0,
				 OMAP_DMA_AMODE_CONSTANT,
				 omap_mcbsp_dma_reg_params(id, 0),
				 0,
				 0);

	omap_set_dma_src_params(state->dma_lch,
				0,
				OMAP_DMA_AMODE_POST_INC,
				state->buf_phy[req->buf_num],
				0,
				0);

	omap_set_dma_callback(state->dma_lch, dma_tx_complete_callback, req);

	omap_start_dma(state->dma_lch);
}

static void dma_tx_complete_callback(int lch, u16 ch_status, void *data)
{
	struct i2s_omap_dma_request *req =
			(struct i2s_omap_dma_request *) data;
	struct tas5713_driver_state *state = req->state;
	unsigned long flags;

	spin_lock_irqsave(&state->dma_req_lock, flags);

	list_del_init(&req->node);
	complete(&state->comp[req->buf_num]);

	if (list_empty(&state->dma_req_queue)) {
		pr_debug("%s: Playback underflow\n", __func__);
		complete(&state->stop_completion);
	} else {
		launch_next_dma(state);
	}

	spin_unlock_irqrestore(&state->dma_req_lock, flags);
}

/* called with state->lock held */
static int start_playback(struct tas5713_driver_state *state,
			  struct i2s_omap_dma_request *req)
{
	unsigned long flags;
	bool need_dma_start;
	u64 enable_start;
	long long samples;

	spin_lock_irqsave(&state->dma_req_lock, flags);

	need_dma_start = list_empty(&state->dma_req_queue);
	list_add_tail(&req->node, &state->dma_req_queue);
	if (need_dma_start)
		launch_next_dma(state);

	enable_start = (*state->pdata->get_raw_counter)();

	samples = req->size / BYTES_PER_OUTPUT_SLOT;

	if (!i2s_fifo_enable(state, 1)) {
		u64 enable_stop = (*state->pdata->get_raw_counter)();
		state->dma_start_local_time =
				(enable_start >> 1) +
				(enable_stop >> 1) +
				(enable_start & enable_stop & 1);

		atomic64_set(&state->samples_queued_to_dma, samples);
	} else {
		atomic64_add(samples, &state->samples_queued_to_dma);
	}

	spin_unlock_irqrestore(&state->dma_req_lock, flags);

	return 0;
}

static bool stop_playback_if_necessary(struct tas5713_driver_state *state)
{
	unsigned long flags;
	spin_lock_irqsave(&state->dma_req_lock, flags);
	pr_debug("%s\n", __func__);
	if (!pending_buffer_requests(state)) {
		pr_debug("%s: no more data to play back\n", __func__);
		atomic64_set(&state->samples_queued_to_dma, 0);
		spin_unlock_irqrestore(&state->dma_req_lock, flags);
		allow_suspend(state);
		return true;
	}
	spin_unlock_irqrestore(&state->dma_req_lock, flags);

	return false;
}

static bool wait_till_stopped(struct tas5713_driver_state *state)
{
	int rc;
	pr_debug("%s: wait for completion\n", __func__);
	rc = wait_for_completion_timeout(
			&state->stop_completion, HZ);
	if (!rc)
		pr_err("%s: wait timed out", __func__);
	if (rc < 0)
		pr_err("%s: wait error %d\n", __func__, rc);
	allow_suspend(state);
	pr_debug("%s: done: %d\n", __func__, rc);
	return true;
}

static bool drain_fifo_and_disable(struct tas5713_driver_state *state)
{
	unsigned id = state->pdata->mcbsp_id;

	/* If the TX FIFO is enabled, wait for it to drain out completely.
	 * Assuming the FIFO was totally full, there would be 64 frames of audio
	 * (128 samples).  At 32KHz (our lowest frame rate), this would be 2mSec
	 * worth of audio.  Wait for 5mSec at most for this FIFO to drain before
	 * proceeding */
	if (is_i2s_fifo_enabled(state)) {
		u32 timeout = 0;
		u32 tmp;
		do {
			tmp = omap_mcbsp_read_reg(id, OMAP_MCBSP_REG_XBUFFSTAT);
			tmp &= 0xFF;
			usleep_range(100, 100);
		} while ((++timeout < 50) && (tmp != 0x80));
	}

	i2s_fifo_enable(state, 0);
	return true;
}

/* Ask for playback to stop.  The _nosync means that state->lock
 * has to be locked by the caller.
 */
static void request_stop_nosync(struct tas5713_driver_state *state)
{
	int i;
	unsigned long flags;

	pr_debug("%s\n", __func__);
	if (!state->stop) {
		state->stop = true;
		if (pending_buffer_requests(state))
			wait_till_stopped(state);
		for (i = 0; i < state->num_bufs; i++) {
			init_completion(&state->comp[i]);
			complete(&state->comp[i]);
		}
	}

	spin_lock_irqsave(&state->dma_req_lock, flags);

	if (!list_empty(&state->dma_req_queue))
		pr_err("%s: DMA not empty!\n", __func__);

	omap_stop_dma(state->dma_lch);
	INIT_LIST_HEAD(&state->dma_req_queue);

	spin_unlock_irqrestore(&state->dma_req_lock, flags);

	pr_debug("%s: done\n", __func__);
}

static ssize_t tas5713_write(struct file *file, const char __user *buf,
		size_t size, loff_t *off)
{
	ssize_t rc = 0;
	int out_buf;
	struct i2s_omap_dma_request *req;
	struct tas5713_driver_state *state = state_from_misc_out(file);

	mutex_lock(&state->lock);

	if (state->write_in_progress) {
		rc = -EBUSY;
		goto done;
	}
	state->write_in_progress = 1;

	if (!IS_ALIGNED(size, 4) || size < 4 || size > buf_size(state)) {
		pr_err("%s: invalid user size %d\n", __func__, size);
		rc = -EINVAL;
		goto done;
	}

	pr_debug("%s: write %d bytes\n", __func__, size);

	if (state->stop) {
		pr_debug("%s: playback has been cancelled\n", __func__);
		rc = -ECANCELED;
		goto done;
	}

	out_buf = (state->last_queued + 1) % state->num_bufs;

	/* Leave the state lock so that other ioctls (like master volume
	 * control) can happen while we are waiting for the next write.
	 */
	mutex_unlock(&state->lock);
	rc = wait_for_completion_interruptible_timeout(
	    &state->comp[out_buf], HZ);
	mutex_lock(&state->lock);

	/* Now that we are back in the state lock, make certain that we have not
	 * been asked to shut down while we were waiting.  Linux's file handle
	 * ref counting should make certain that our file has not been released
	 * out from under us yet.
	 */
	if (state->stop) {
		pr_debug("%s: playback has been cancelled\n", __func__);
		rc = -ECANCELED;
		goto done;
	}

	if (!rc) {
		pr_err("%s: timeout", __func__);
		rc = -ETIMEDOUT;
		goto done;
	} else if (rc < 0) {
		pr_err("%s: wait error %d", __func__, rc);
		goto done;
	}

	/* Fill the buffer and enqueue it. */
	pr_debug("%s: acquired buffer %d, copying data\n", __func__, out_buf);
	rc = copy_from_user(state->buffer[out_buf], buf, size);
	if (rc) {
		rc = -EFAULT;
		goto done;
	}

	prevent_suspend(state);

	req = &state->buf_req[out_buf];
	req->buf_num = out_buf;
	req->size = size;

	dma_sync_single_for_device(&state->i2c_client->dev,
				   state->buf_phy[out_buf],
				   buf_size(state),
				   DMA_TO_DEVICE);
	state->last_queued = out_buf;
	init_completion(&state->stop_completion);

	rc = start_playback(state, req);
	if (!rc)
		rc = size;
	else
		allow_suspend(state);

done:
	state->write_in_progress = 0;
	mutex_unlock(&state->lock);
	return rc;
}

static long tas5713_out_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	int rc = 0;
	struct tas5713_driver_state *state = state_from_misc_out_ctl(file);

	mutex_lock(&state->lock);

	switch (cmd) {
	case TAS5713_FLUSH: {
		if (pending_buffer_requests(state)) {
			pr_debug("%s: flushing\n", __func__);
			request_stop_nosync(state);
			pr_debug("%s: flushed\n", __func__);
		}
		if (stop_playback_if_necessary(state))
			pr_debug("%s: done (stopped)\n", __func__);

		drain_fifo_and_disable(state);
		state->stop = false;
	} break;

	case TAS5713_GET_NEXT_WRITE_TIMESTAMP: {
		struct tas5713_next_write_ts_resp resp;

		resp.samples_queued_since_dma_start =
			atomic64_read(&state->samples_queued_to_dma);

		resp.dma_start_time = state->dma_start_local_time;

		/* If no samples have been queued, then we have no DMA start
		 * time for this output channel.
		 */
		if ((!resp.samples_queued_since_dma_start) ||
		    (resp.samples_queued_since_dma_start >> 63)) {
			rc = -ENODEV;
			break;
		}

		if (copy_to_user((void __user *)arg, &resp, sizeof(resp)))
			rc = -EFAULT;
	} break;

	case TAS5713_GET_ERROR_REGISTER: {
		u8 err = 0x00;
		rc = i2c_smbus_read_byte_data(state->i2c_client, 0x02);

		if (rc < 0)
			break;

		err = (u8)rc;
		rc = 0;

		if (copy_to_user((void __user *)arg,
				&err, sizeof(err)))
			rc = -EFAULT;
	} break;

	case TAS5713_GET_MASTER_VOLUME: {
		u8 vol = state->last_master_vol_reg;

		if (copy_to_user((void __user *)arg,
				&vol, sizeof(vol)))
			rc = -EFAULT;
	} break;

	case TAS5713_SET_MASTER_VOLUME: {
		u8 vol;

		if (copy_from_user(&vol, (const void __user *)arg,
					sizeof(vol))) {
			rc = -EFAULT;
			break;
		}

		rc = i2c_smbus_write_byte_data(state->i2c_client, 0x07, vol);
		if (rc < 0)
			break;

		state->last_master_vol_reg = vol;

		rc = 0;

	} break;

	case TAS5713_DO_LOW_LEVEL_I2C: {
		struct tas5713_i2c_request req;
		int expected_rc;

		if (copy_from_user(&req, (const void __user *)arg,
					sizeof(req))) {
			rc = -EFAULT;
			break;
		}

		if (req.len > sizeof(req.data)) {
			rc = -EINVAL;
			break;
		}

		if (req.is_write_op) {
			u8 data[sizeof(req.data) + 1];
			data[0] = req.reg;
			memcpy(data + 1, req.data, req.len);
			rc = i2c_master_send(state->i2c_client,
					data, req.len + 1);
			expected_rc = req.len + 1;
		} else {
			rc = i2c_smbus_read_i2c_block_data(state->i2c_client,
					req.reg, req.len, req.data);
			expected_rc = req.len;
		}

		if (rc != expected_rc) {
			if (rc >= 0)
				rc = -EIO;
			break;
		} else
			rc = 0;

		if (copy_to_user((void __user *)arg,
				&req, sizeof(req)))
			rc = -EFAULT;
	} break;

	default:
		rc = -EINVAL;
	}

	mutex_unlock(&state->lock);
	return rc;
}

static int tas5713_open(struct inode *inode, struct file *file)
{
	int rc = 0;
	int i;
	struct tas5713_driver_state *state = state_from_misc_out(file);

	pr_debug("%s\n", __func__);

	mutex_lock(&state->lock);

	if (state->opened) {
		rc = -EBUSY;
		goto done;
	}

	state->opened = 1;
	state->stop = false;

	atomic64_set(&state->samples_queued_to_dma, 0);

	for (i = 0; i < I2S_MAX_NUM_BUFS; i++) {
		init_completion(&state->comp[i]);
		/* TX buf rest state is unqueued, complete. */
		complete(&state->comp[i]);
	}

done:
	mutex_unlock(&state->lock);
	return rc;
}

static int tas5713_release(struct inode *inode, struct file *file)
{
	struct tas5713_driver_state *state = state_from_misc_out(file);

	pr_debug("%s\n", __func__);

	mutex_lock(&state->lock);
	state->opened = 0;

	atomic64_set(&state->samples_queued_to_dma, 0);

	request_stop_nosync(state);
	if (stop_playback_if_necessary(state))
		pr_debug("%s: done (stopped)\n", __func__);
	drain_fifo_and_disable(state);
	allow_suspend(state);

	mutex_unlock(&state->lock);
	pr_debug("%s: done\n", __func__);

	return 0;
}

static const struct file_operations tas5713_out_fops = {
	.owner = THIS_MODULE,
	.open = tas5713_open,
	.release = tas5713_release,
	.write = tas5713_write,
};

static int tas5713_ctl_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int tas5713_ctl_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations tas5713_out_ctl_fops = {
	.owner = THIS_MODULE,
	.open = tas5713_ctl_open,
	.release = tas5713_ctl_release,
	.unlocked_ioctl = tas5713_out_ioctl,
};

static int init_stream_buffer(struct tas5713_driver_state *state, int num)
{
	int i, j;
	pr_debug("%s (num %d)\n", __func__,  num);

	for (i = 0; i < num; i++) {
		kfree(state->buffer[i]);
		state->buffer[i] = kmalloc(buf_size(state),
					   GFP_KERNEL | GFP_DMA);
		if (!state->buffer[i]) {
			pr_err("%s: could not allocate buffer\n", __func__);
			for (j = i - 1; j >= 0; j--) {
				kfree(state->buffer[j]);
				state->buffer[j] = 0;
			}
			return -ENOMEM;
		}
	}
	return 0;
}

static int setup_misc_device(struct miscdevice *misc,
			const struct file_operations  *fops,
			const char *fmt, ...)
{
	int rc = 0;
	va_list args;
	const int sz = 64;

	va_start(args, fmt);

	memset(misc, 0, sizeof(*misc));
	misc->minor = MISC_DYNAMIC_MINOR;
	misc->name  = kmalloc(sz, GFP_KERNEL);
	if (!misc->name) {
		rc = -ENOMEM;
		goto done;
	}

	vsnprintf((char *)misc->name, sz, fmt, args);
	misc->fops = fops;
	if (misc_register(misc)) {
		pr_err("%s: could not register %s\n", __func__, misc->name);
		kfree(misc->name);
		rc = -EIO;
		goto done;
	}

done:
	va_end(args);
	return rc;
}

static int setup_dma(struct tas5713_driver_state *state)
{
	int rc = 0;
	int i;
	int dma_channel;

	for (i = 0; i < state->num_bufs; i++) {
		state->buf_phy[i] = dma_map_single(
		    &state->i2c_client->dev, state->buffer[i],
		    1 << PCM_BUFFER_MAX_SIZE_ORDER,
		    DMA_TO_DEVICE);
		BUG_ON(!state->buf_phy[i]);
	}

	dma_channel = omap_mcbsp_dma_ch_params(state->pdata->mcbsp_id, 0);
	if (omap_request_dma(dma_channel, "McBSP TX",
			     dma_tx_complete_callback, NULL,
			     &state->dma_lch)) {
		rc = -EAGAIN;
		goto err_request_dma;
	}

	return rc;

err_request_dma:
	for (i = 0; i < state->num_bufs; i++) {
		dma_unmap_single(&state->i2c_client->dev, state->buf_phy[i],
				 1 << PCM_BUFFER_MAX_SIZE_ORDER,
				 DMA_TO_DEVICE);
		state->buf_phy[i] = 0;
	}

	return rc;
}

static void tear_down_dma(struct tas5713_driver_state *state)
{
	int i;

	for (i = 0; i < state->num_bufs; i++) {
		dma_unmap_single(&state->i2c_client->dev, state->buf_phy[i],
				 buf_size(state), DMA_TO_DEVICE);
		state->buf_phy[i] = 0;
	}

	omap_free_dma(state->dma_lch);
	state->dma_lch = -1;
}

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
	{ .size = 2, .data = "\x0E\xF1" },

	/* Clock Control Register: 44.1/48KHz sample clock,
	 * MCLK = 256xFs (defaults)
	 */
	{ .size = 2, .data = "\x00\x6C" },

	/* PWM Output Mux Register */
	{ .size = 5, .data = "\x25\x01\x02\x13\x45" },

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

static const int shutdown_wait_time_usec = (1000 + (13 * 12570));

static int power_up_tas5713(struct tas5713_driver_state *state)
{
	int rc, i, mcbsp_allocated = 0;
	struct tas5713_platform_data *pdata = state->pdata;
	unsigned mcbsp_id;

	BUG_ON(!pdata);
	BUG_ON(!pdata->mclk_out);

	mcbsp_id = pdata->mcbsp_id;

	/* Start by making absolutely certain that the reset and power down
	 * signals are asserted, and that the external level translator is
	 * enabled.
	 */
	gpio_set_value(pdata->pdn_gpio, 0);
	gpio_set_value(pdata->reset_gpio, 0);
	gpio_set_value(pdata->interface_en_gpio, 1);

	/* Turn on MCLK */
	rc = clk_enable(pdata->mclk_out);
	if (rc) {
		printk(ERR_LOG_PREFIX "Failed to enable MCLK (rc = %d)\n", rc);
		goto err_setup;
	}

	/* Now turn on the McBSP in order to generate the LRCLK signal. */
	rc = setup_mcbsp(state);
	if (rc) {
		printk(ERR_LOG_PREFIX "Failed to enable McBSP (rc = %d)\n", rc);
		goto err_setup;
	}

	mcbsp_allocated = 1;

	/* Give the LRCLK at least 100uSec to stabilize before de-asserting
	 * power down.
	 */
	usleep_range(100, 100);

	/* Deassert the power down signal and wait the specified 100uSec */
	gpio_set_value(pdata->pdn_gpio, 1);
	usleep_range(100, 100);

	/* Now deassert the reset signal and wait the specified 13.5mSec */
	gpio_set_value(pdata->reset_gpio, 1);
	usleep_range(13500, 13500);

	/* Select factory trimming for the interal OSC used to detect sample
	 * rates, then wait the spec'ed 50mSec before proceeding.
	 */
	rc = i2c_smbus_write_byte_data(state->i2c_client, 0x1B, 0x00);
	if (rc < 0) {
		printk(ERR_LOG_PREFIX "I2C failure (%d) while trying to trim"
				" oscillator\n", rc);
		goto err_setup;
	}
	usleep_range(50000, 50000);

	/* Now program in the default initialization */
	for (i = 0; i < ARRAY_SIZE(tas5713_init_sequence); ++i) {
		rc = i2c_master_send(state->i2c_client,
				     tas5713_init_sequence[i].data,
				     tas5713_init_sequence[i].size);

		if (rc < 0) {
			printk(ERR_LOG_PREFIX "I2C failure (%d) while trying to"
					" write register 0x%02x\n", rc,
					tas5713_init_sequence[i].data[0]);
			goto err_setup;
		} else if (0x07 == tas5713_init_sequence[i].data[0]) {
			/* If we just reset the master volume, make sure we
			 * record what we just set it to.
			 */
			state->last_master_vol_reg =
				tas5713_init_sequence[i].data[1];
		}
	}

	/* Exit shutdown and wait for the required amt of time after exiting
	 * shutdown.
	 */
	rc = i2c_smbus_write_byte_data(state->i2c_client, 0x05, 0x00);
	if (rc < 0) {
		printk(ERR_LOG_PREFIX "I2C failure (%d) while trying to exit"
				" shutdown\n", rc);
		goto err_setup;
	}
	usleep_range(shutdown_wait_time_usec, shutdown_wait_time_usec);

	return 0;

err_setup:
	clk_disable(pdata->mclk_out);

	/* Something went wrong?  Follow the power down sequence as given by the
	 * datasheet.  We don't whether or not communications with the TAS5713
	 * via I2C is possible right now, so we have to skip the shutdown phase
	 * and go directly to the sudden power loss sequence.
	 */
	gpio_set_value(pdata->pdn_gpio, 0);
	usleep_range(2000, 2000); /* spec'ed 2ms wait between PDN
				   * and Reset assert.
				   */
	gpio_set_value(pdata->reset_gpio, 0);

	/* Make sure the LRCLK is de-asserted. */
	if (mcbsp_allocated) {
		omap_mcbsp_stop(mcbsp_id, 1, 0);
		omap_mcbsp_free(mcbsp_id);
		free_irq(omap_mcbsp_get_tx_irq(mcbsp_id), state);
	}

	/* Shut of the external level translator */
	gpio_set_value(pdata->interface_en_gpio, 0);

	return rc;
}

static void power_down_tas5713(struct tas5713_driver_state *state)
{
	int rc;
	unsigned mcbsp_id = state->pdata->mcbsp_id;

	/* Enter shutdown and wait for the required amt of time after entering
	 * shutdown.
	 */
	rc = i2c_smbus_write_byte_data(state->i2c_client, 0x05, 0x40);
	if (rc < 0) {
		printk(ERR_LOG_PREFIX "I2C failure (%d) while trying to exit"
				" shutdown\n", rc);
		goto err_clean_shutdown_failed;
	}
	usleep_range(shutdown_wait_time_usec, shutdown_wait_time_usec);

	/* Assert power down, reset, and then shut off the LRCLK and MCLK.  We
	 * should not have to wait between power down and reset since we already
	 * cleanly entered the shutdown state via i2c commands.
	 */
	gpio_set_value(state->pdata->pdn_gpio, 0);
	goto common_exit;

err_clean_shutdown_failed:
	/* We failed to cleanly enter the shutdown state.  Follow the sudden
	 * power loss sequence (waiting 2mSec after asserting PDN before
	 * asserting Reset)
	 */
	gpio_set_value(state->pdata->pdn_gpio, 0);
	usleep_range(2000, 2500);

common_exit:
	gpio_set_value(state->pdata->reset_gpio, 0);

	/* Make sure the LR and MCLK clocks are now shut off. */
	omap_mcbsp_stop(mcbsp_id, 1, 0);
	clk_disable(state->pdata->mclk_out);

	/* Shut of the external level translator */
	gpio_set_value(state->pdata->interface_en_gpio, 0);
}

static int tas5713_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct tas5713_driver_state *state = NULL;
	int rc;
	int i;

	pr_info("%s:\n", __func__);

	state = kzalloc(sizeof(struct tas5713_driver_state), GFP_KERNEL);
	if (!state) {
		rc = -ENOMEM;
		goto err_state_alloc;
	}

	i2c_set_clientdata(client, state);
	state->i2c_client = client;
	state->pdata = client->dev.platform_data;

	if (state->pdata == NULL) {
		rc = -ENODEV;
		pr_err("%s: missing platform data\n", __func__);
		goto err_missing_platform_data;
	}
	rc = gpio_request_one(state->pdata->interface_en_gpio,
			      GPIOF_OUT_INIT_LOW,
			      "tas5713_interface_en");
	if (rc) {
		pr_err("%s: gpio_request_one(interface_en_gpio = %d) failed"
		       " with error %d\n", __func__,
		       state->pdata->interface_en_gpio, rc);
		goto err_request_interface_en_gpio;
	}
	rc = gpio_request_one(state->pdata->reset_gpio,
			      GPIOF_OUT_INIT_LOW,
			      "tas5713_reset");
	if (rc) {
		pr_err("%s: gpio_request_one(reset_gpio = %d) failed"
		       " with error %d\n", __func__,
		       state->pdata->reset_gpio, rc);
		goto err_request_reset_gpio;
	}
	rc = gpio_request_one(state->pdata->pdn_gpio,
			      GPIOF_OUT_INIT_LOW,
			      "tas5713_pdn");
	if (rc) {
		pr_err("%s: gpio_request_one(pdn_gpio = %d) failed"
		       " with error %d\n", __func__,
		       state->pdata->pdn_gpio, rc);
		goto err_request_pdn_gpio;
	}

	rc = setup_misc_device(&state->misc_out,
			       &tas5713_out_fops,
			       "audio%d_out",
			       id->driver_data);
	if (rc)
		goto err_misc_out;

	rc = setup_misc_device(&state->misc_out_ctl,
			       &tas5713_out_ctl_fops,
			       "audio%d_out_ctl",
			       id->driver_data);
	if (rc)
		goto err_misc_out_ctl;

	mutex_init(&state->lock);
	init_completion(&state->stop_completion);
	spin_lock_init(&state->dma_req_lock);
	state->num_bufs = I2S_DEFAULT_TX_NUM_BUFS;
	for (i = 0; i < I2S_MAX_NUM_BUFS; i++) {
		init_completion(&state->comp[i]);
		complete(&state->comp[i]);
		state->buffer[i] = 0;
		state->buf_phy[i] = 0;
		state->buf_req[i].state = state;
	}
	state->last_queued = 0;
	INIT_LIST_HEAD(&state->dma_req_queue);
	state->dma_lch = -1;

	rc = init_stream_buffer(state, state->num_bufs);
	if (rc)
		goto err_init_buffer;

	rc = setup_dma(state);
	if (rc)
		goto err_setup_dma;

	rc = power_up_tas5713(state);
	if (rc)
		goto err_power_up;

	return 0;

err_power_up:
	tear_down_dma(state);

err_setup_dma:
err_init_buffer:
	misc_deregister(&state->misc_out_ctl);
	kfree(state->misc_out_ctl.name);

err_misc_out_ctl:
	misc_deregister(&state->misc_out);
	kfree(state->misc_out.name);

err_misc_out:
	gpio_free(state->pdata->pdn_gpio);
err_request_pdn_gpio:
	gpio_free(state->pdata->reset_gpio);
err_request_reset_gpio:
	gpio_free(state->pdata->interface_en_gpio);
err_request_interface_en_gpio:
err_missing_platform_data:
	i2c_set_clientdata(client, NULL);
	kfree(state);
err_state_alloc:
	return rc;
}


static int __devexit tas5713_remove(struct i2c_client *client)
{
	struct tas5713_driver_state *state = i2c_get_clientdata(client);
	i2c_set_clientdata(client, NULL);

	if (state) {
		unsigned id = state->pdata->mcbsp_id;
		int i;

		power_down_tas5713(state);

		misc_deregister(&state->misc_out);
		kfree(state->misc_out.name);
		misc_deregister(&state->misc_out_ctl);
		kfree(state->misc_out_ctl.name);

		tear_down_dma(state);

		for (i = 0; i < state->num_bufs; i++)
			kfree(state->buffer[i]);

		omap_mcbsp_free(id);
		free_irq(omap_mcbsp_get_tx_irq(id), state);

		gpio_free(state->pdata->pdn_gpio);
		gpio_free(state->pdata->reset_gpio);
		gpio_free(state->pdata->interface_en_gpio);

		kfree(state);
	}

	return 0;
}

static struct i2c_device_id tas5713_idtable[] = {
	{ "tas5713", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tas5713_idtable);

static struct i2c_driver tas5713_driver = {
	.driver = {
		.name = "tas5713",
	},

	.id_table = tas5713_idtable,
	.probe = tas5713_probe,
	.remove = __devexit_p(tas5713_remove),

	/* TODO(johngro) implement these optional power management routines. */
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
