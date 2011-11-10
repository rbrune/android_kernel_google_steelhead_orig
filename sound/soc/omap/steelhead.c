/* sound/soc/omap/steelhead.c
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
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <sound/soc.h>

#include "../../../arch/arm/mach-omap2/board-steelhead.h"
#include "omap-mcbsp.h"

static struct platform_device *steelhead_spdif_device;
static struct platform_device *steelhead_tas5713_device;
static int dev_id = 0;

/*
 *	S/PDIF output
 */
static struct snd_soc_dai_link steelhead_spdif_dai[] = {
	{
		.name = "SPDIF",
		.stream_name = "SPDIF",
		.cpu_dai_name = "omap-mcasp-dai",
		.codec_name = "spdif-dit.0",
		.codec_dai_name = "dit-hifi",
		.platform_name = "omap-pcm-audio",
	},
};

static struct snd_soc_card snd_soc_steelhead_spdif = {
	.name = "SPDIF",
	.driver_name = "OMAP4",
	.long_name = "Steelhead SPDIF Card",
	.dai_link = steelhead_spdif_dai,
	.num_links = ARRAY_SIZE(steelhead_spdif_dai),
};

/*
 *	TAS5713 outputs
 */
struct steelhead_tas5713_drvdata {
	struct clk* mcbsp_clk;
};

static struct steelhead_tas5713_drvdata tas5713_drvdata;

static int steelhead_tas5713_startup(struct snd_pcm_substream* substream)
{
	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return -ENODEV;

	/* Turn on the level translator between OMAP and the TAS5713 */
	steelhead_set_tas5713_interface_en(1);

	return 0;
}

static void steelhead_tas5713_shutdown(struct snd_pcm_substream* substream)
{
	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return;

	/* Turn off the level translator between OMAP and the TAS5713 */
	steelhead_set_tas5713_interface_en(0);
}

static int steelhead_tas5713_hw_params(
		struct snd_pcm_substream* substream,
		struct snd_pcm_hw_params* params)
{
	struct snd_soc_pcm_runtime *rtd;
	unsigned long clk_rate;
	unsigned long tgt_rate;
	int clk_div;
	int ret;

	ret = 0;

	if ((substream->stream != SNDRV_PCM_STREAM_PLAYBACK) ||
		IS_ERR_OR_NULL(tas5713_drvdata.mcbsp_clk)) {
		return -ENODEV;
	}

	ret = clk_enable(tas5713_drvdata.mcbsp_clk);
	if (ret) {
		pr_err("failed to enable mcbsp clk\n");
		goto bailout;
	}

	clk_rate = clk_get_rate(tas5713_drvdata.mcbsp_clk);
	tgt_rate = params_rate(params) * 32;
	clk_div = clk_rate / tgt_rate;

	if ((clk_rate % tgt_rate) || (clk_div > 256)) {
		pr_err("Cannot produce a target rate of %d from a parent"
			" rate of %ld\n", params_rate(params), clk_rate);
		ret = -ENODEV;
		goto bailout;
	}

	rtd = substream->private_data;

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(rtd->cpu_dai,
			SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS);

	if (unlikely(ret < 0)) {
		pr_err("can't set tas5713 cpu dai format\n");
		goto bailout;
	}

	ret = snd_soc_dai_set_sysclk(rtd->cpu_dai,
			OMAP_MCBSP_SYSCLK_CLKS_FCLK,
			clk_rate,
			SND_SOC_CLOCK_IN);
	if (unlikely(ret < 0)) {
		pr_err("can't set tas5713 cpu dai sysclk\n");
		goto bailout;
	}

	ret = snd_soc_dai_set_clkdiv(rtd->cpu_dai, OMAP_MCBSP_CLKGDV, clk_div);
	if (unlikely(ret < 0)) {
		pr_err("can't set tas5713 cpu dai clkdiv\n");
		goto bailout;
	}

bailout:
	if (!IS_ERR_OR_NULL(tas5713_drvdata.mcbsp_clk) && ret)
		clk_disable(tas5713_drvdata.mcbsp_clk);

	return ret;
}

static int steelhead_tas5713_hw_free(struct snd_pcm_substream *substream) {
	if (!IS_ERR_OR_NULL(tas5713_drvdata.mcbsp_clk))
		clk_disable(tas5713_drvdata.mcbsp_clk);

	return 0;
}

static struct snd_soc_ops steelhead_tas5713_ops = {
	.startup = steelhead_tas5713_startup,
	.shutdown = steelhead_tas5713_shutdown,
	.hw_params = steelhead_tas5713_hw_params,
	.hw_free = steelhead_tas5713_hw_free,
};

static struct snd_soc_dai_link steelhead_tas5713_dai[] = {
	{
		.name = "TAS5713",
		.stream_name = "TAS5713",
		.cpu_dai_name = "omap-mcbsp-dai.1",	/* McBSP2 */
		.codec_name = "tas5713.4-001b",		/* I2C4 device 0x36 */
		.codec_dai_name = "tas5713-codec",
		.platform_name = "omap-pcm-audio",
		.ops = &steelhead_tas5713_ops,
	},
};

static struct snd_soc_card snd_soc_steelhead_tas5713 = {
	.name = "TAS5713",
	.driver_name = "OMAP4",
	.long_name = "Steelhead TAS5713 Card",
	.dai_link = steelhead_tas5713_dai,
	.num_links = ARRAY_SIZE(steelhead_tas5713_dai),
	.drvdata = &tas5713_drvdata,
};

static int __init make_audio_device(
		struct platform_device** dev,
		struct snd_soc_card* card_data)
{
	int ret;

	*dev = platform_device_alloc("soc-audio", dev_id);
	if (!(*dev)) {
		pr_err("Platform device allocation failed for "
				"Steelhead ASoC device \"%s\"\n",
				card_data->dai_link->name);
		return -ENOMEM;
	}

	platform_set_drvdata(*dev, card_data);

	ret = platform_device_add(*dev);
	if (ret)
		goto err;

	dev_id++;
	return 0;

err:
	pr_err("Unable to add platform device for Steelhead ASoC "
			"device \"%s\"", card_data->dai_link->name);
	platform_device_put(*dev);
	*dev = NULL;
	return ret;
}

static int __init steelhead_soc_spdif_init(void)
{
	return make_audio_device(&steelhead_spdif_device,
			&snd_soc_steelhead_spdif);
}

static int __init steelhead_soc_tas5713_init(void)
{
	int ret;
	struct clk *abe_24m_clk;

	abe_24m_clk = clk_get(NULL, "abe_24m_fclk");
	ret = IS_ERR_OR_NULL(abe_24m_clk);
	if (ret) {
		pr_err("failed to fetch abe_24m_clk\n");
		goto err;
	}

	tas5713_drvdata.mcbsp_clk = clk_get(NULL, "mcbsp2_sync_mux_ck");
	ret = IS_ERR_OR_NULL(tas5713_drvdata.mcbsp_clk);
	if (ret) {
		pr_err("failed to fetch mcbsp2_sync_mux_ck\n");
		goto err;
	}

	ret = clk_set_parent(tas5713_drvdata.mcbsp_clk,
			abe_24m_clk);

	if (ret) {
		pr_err(" failed to set reference clock for McBSP2 internal clk"
				" (res = %d)", ret);
		goto err;
	}

	ret = make_audio_device(
			&steelhead_tas5713_device,
			&snd_soc_steelhead_tas5713);

err:
	if (ret && !IS_ERR_OR_NULL(tas5713_drvdata.mcbsp_clk)) {
		clk_put(tas5713_drvdata.mcbsp_clk);
		tas5713_drvdata.mcbsp_clk = NULL;
	}

	if (!IS_ERR_OR_NULL(abe_24m_clk))
		clk_put(abe_24m_clk);

	/* The level translator enable signal was left enabled by the steelhead
	 * board file so that the TAS5713 codec driver could find (or not find)
	 * the device via I2C probe.  Whether or not we have successfully
	 * created an ASoC card representing the TAS5713 audio path, its time to
	 * turn the level translator back off.  Moving forward, it will be
	 * enabled/disabled dynamically by the machine driver as needed.
	 */
	steelhead_set_tas5713_interface_en(0);

	return ret;

}

static int __init steelhead_soc_init(void)
{
	int spdif_ret;
	int tas5713_ret;

	spdif_ret   = steelhead_soc_spdif_init();
	tas5713_ret = steelhead_soc_tas5713_init();

	if (spdif_ret && tas5713_ret) {
		pr_err("Failed to create any audio devices.");
		return spdif_ret;
	}

	return 0;

}
module_init(steelhead_soc_init);

static void __exit steelhead_soc_exit(void)
{
	if(steelhead_spdif_device) {
		platform_device_unregister(steelhead_spdif_device);
		steelhead_spdif_device = NULL;
	}

	if(steelhead_tas5713_device) {
		platform_device_unregister(steelhead_tas5713_device);
		steelhead_tas5713_device = NULL;

		if (!IS_ERR_OR_NULL(tas5713_drvdata.mcbsp_clk)) {
			clk_disable(tas5713_drvdata.mcbsp_clk);
			clk_put(tas5713_drvdata.mcbsp_clk);
			tas5713_drvdata.mcbsp_clk = NULL;
		}
	}
}
module_exit(steelhead_soc_exit);

MODULE_DESCRIPTION("ALSA SoC Steelhead");
MODULE_LICENSE("GPL");

