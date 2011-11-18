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


#include <linux/platform_device.h>
#include <sound/soc.h>

static struct platform_device *steelhead_snd_device;

static struct snd_soc_dai_link steelhead_dai[] = {
	{
		.name = "SPDIF",
		.stream_name = "SPDIF",
		.cpu_dai_name = "omap-mcasp-dai",
		.codec_name = "spdif-dit.0",
		.codec_dai_name = "dit-hifi",
		.platform_name = "omap-pcm-audio",
	},
};

static struct snd_soc_card snd_soc_steelhead = {
	.name = "Steelhead",
	.driver_name = "OMAP4",
	.long_name = "Steelhead OMAP4 Board",
	.dai_link = steelhead_dai,
	.num_links = ARRAY_SIZE(steelhead_dai),
};

static int __init steelhead_soc_init(void)
{
	int ret;
	printk(KERN_INFO "Steelhead ASoC init\n");

	steelhead_snd_device = platform_device_alloc("soc-audio", -1);
	if (!steelhead_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(steelhead_snd_device, &snd_soc_steelhead);

	ret = platform_device_add(steelhead_snd_device);
	if (ret)
		goto err;

	return 0;

err:
	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(steelhead_snd_device);
	return ret;
}
module_init(steelhead_soc_init);

static void __exit steelhead_soc_exit(void)
{
	platform_device_unregister(steelhead_snd_device);
}
module_exit(steelhead_soc_exit);

MODULE_DESCRIPTION("ALSA SoC Steelhead");
MODULE_LICENSE("GPL");

