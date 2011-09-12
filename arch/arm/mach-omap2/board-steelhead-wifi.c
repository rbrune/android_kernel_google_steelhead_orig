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
#include <linux/kernel.h>
#include <linux/if.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#include <plat/mmc.h>

#include "board-steelhead.h"
#include "hsmmc.h"
#include "mux.h"

#define GPIO_WLAN_PMENA			43
#define GPIO_WLAN_PMENA_MUX_NAME       "gpmc_a19.gpio_43"
#define GPIO_WLAN_IRQ			53
#define GPIO_WLAN_IRQ_MUX_NAME		"gpmc_ncs3.gpio_53"

static struct steelhead_gpio_reservation wlan_gpios[] = {
	{
		.gpio_name = "wlan_pmena",
		.gpio_id = GPIO_WLAN_PMENA,
		.mux_name = GPIO_WLAN_PMENA_MUX_NAME,
		.pin_mode = OMAP_PIN_OUTPUT,
		.init_state = GPIOF_OUT_INIT_LOW,
	},
	{
		.gpio_name = "wlan_irq",
		.gpio_id = GPIO_WLAN_IRQ,
		.mux_name = GPIO_WLAN_IRQ_MUX_NAME,
		.pin_mode = OMAP_PIN_INPUT,
		.init_state = GPIOF_IN,
	},
};

#define ATAG_STEELHEAD_MAC	0x57464d41
/* #define ATAG_STEELHEAD_MAC_DEBUG */

#define PREALLOC_WLAN_NUMBER_OF_SECTIONS	4
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS		160
#define PREALLOC_WLAN_SECTION_HEADER		24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 1024)

#define WLAN_SKB_BUF_NUM	16

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

typedef struct wifi_mem_prealloc_struct {
	void *mem_ptr;
	unsigned long size;
} wifi_mem_prealloc_t;

static wifi_mem_prealloc_t wifi_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER) }
};

static void *steelhead_wifi_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_NUMBER_OF_SECTIONS)
		return wlan_static_skb;
	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;
	if (wifi_mem_array[section].size < size)
		return NULL;
	return wifi_mem_array[section].mem_ptr;
}

int __init steelhead_init_wifi_mem(void)
{
	int i;

	for(i=0;( i < WLAN_SKB_BUF_NUM );i++) {
		if (i < (WLAN_SKB_BUF_NUM/2))
			wlan_static_skb[i] = dev_alloc_skb(4096);
		else
			wlan_static_skb[i] = dev_alloc_skb(8192);
	}
	for(i=0;( i < PREALLOC_WLAN_NUMBER_OF_SECTIONS );i++) {
		wifi_mem_array[i].mem_ptr = kmalloc(wifi_mem_array[i].size,
							GFP_KERNEL);
		if (wifi_mem_array[i].mem_ptr == NULL)
			return -ENOMEM;
	}
	return 0;
}

static struct resource steelhead_wifi_resources[] = {
	[0] = {
		.name		= "bcmdhd_wlan_irq",
		.start		= OMAP_GPIO_IRQ(GPIO_WLAN_IRQ),
		.end		= OMAP_GPIO_IRQ(GPIO_WLAN_IRQ),
		.flags          = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};

static int steelhead_wifi_cd = 0; /* WIFI virtual 'card detect' status */
static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

static int steelhead_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static unsigned int steelhead_wifi_status(struct device *dev)
{
	return steelhead_wifi_cd;
}

struct mmc_platform_data steelhead_wifi_data = {
	.ocr_mask		= MMC_VDD_165_195 | MMC_VDD_20_21,
	.built_in		= 1,
	.status			= steelhead_wifi_status,
	.card_present		= 0,
	.register_status_notify	= steelhead_wifi_status_register,
};

static int steelhead_wifi_set_carddetect(int val)
{
	steelhead_wifi_cd = val;
	if (wifi_status_cb) {
		wifi_status_cb(val, wifi_status_cb_devid);
	} else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
}

static int steelhead_wifi_power_state;

static int steelhead_wifi_power(int on)
{
	mdelay(100);
	gpio_set_value(GPIO_WLAN_PMENA, on);
	mdelay(200);

	steelhead_wifi_power_state = on;
	return 0;
}

static int steelhead_wifi_reset_state;

static int steelhead_wifi_reset(int on)
{
	steelhead_wifi_reset_state = on;
	return 0;
}

static int steelhead_mac_addr_initialized;
static unsigned char steelhead_mac_addr[IFHWADDRLEN];

static int __init steelhead_mac_addr_setup(char *str)
{
	char macstr[IFHWADDRLEN*3];
	char *macptr = macstr;
	char *token;
	int i = 0;

	if (!str)
		return 0;
	pr_info("wlan MAC = %s\n", str);
	if (strlen(str) >= sizeof(macstr))
		return 0;
	strcpy(macstr, str);

	/* on any errors, we just break and always return 1 to tell
	 * kernel param parsing that we handled the string so it
	 * doesn't waste time looking for another handler.
	 * When we validate the values when try to use them in
	 * steelhead_wifi_get_mac_addr().
	 */
	while ((token = strsep(&macptr, ":")) != NULL) {
		unsigned long val;
		int res;

		if (i >= IFHWADDRLEN) {
			pr_err("%s: mac address too long\n", __func__);
			goto err;
		}
		res = strict_strtoul(token, 0x10, &val);
		if (res < 0) {
			pr_err("%s: strtoul() error %d\n", __func__, res);
			goto err;
		}
		steelhead_mac_addr[i++] = (u8)val;
	}

	if (i != IFHWADDRLEN) {
		pr_err("%s: mac address too short\n", __func__);
		goto err;
	}
	steelhead_mac_addr_initialized = 1;
err:
	return 1;
}

__setup("androidboot.wifi_macaddr=", steelhead_mac_addr_setup);

static int steelhead_wifi_get_mac_addr(unsigned char *buf)
{
	uint rand_mac;

	if (!buf)
		return -EINVAL;

	if (!steelhead_mac_addr_initialized) {
		srandom32((uint)jiffies);
		rand_mac = random32();
		/* first 3 nibbles are for Google mac addresses */
		steelhead_mac_addr[0] = 0x00;
		steelhead_mac_addr[1] = 0x1A;
		steelhead_mac_addr[2] = 0x11;
		steelhead_mac_addr[3] = (unsigned char)rand_mac;
		steelhead_mac_addr[4] = (unsigned char)(rand_mac >> 8);
		steelhead_mac_addr[5] = (unsigned char)(rand_mac >> 16);
		steelhead_mac_addr_initialized = 1;
	}
	memcpy(buf, steelhead_mac_addr, IFHWADDRLEN);

	pr_info("steelhead_mac_addr = %x:%x:%x:%x:%x:%x\n",
		buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

	return 0;
}

/* Customized Locale table : OPTIONAL feature */
#define WLC_CNTRY_BUF_SZ	4
typedef struct cntry_locales_custom {
	char iso_abbrev[WLC_CNTRY_BUF_SZ];
	char custom_locale[WLC_CNTRY_BUF_SZ];
	int  custom_locale_rev;
} cntry_locales_custom_t;

static cntry_locales_custom_t steelhead_wifi_translate_custom_table[] = {
/* Table should be filled out based on custom platform regulatory requirement */
	{"",   "XY", 4},  /* universal */
	{"US", "US", 69}, /* input ISO "US" to : US regrev 69 */
	{"CA", "US", 69}, /* input ISO "CA" to : US regrev 69 */
	{"EU", "EU", 5},  /* European union countries */
	{"AT", "EU", 5},
	{"BE", "EU", 5},
	{"BG", "EU", 5},
	{"CY", "EU", 5},
	{"CZ", "EU", 5},
	{"DK", "EU", 5},
	{"EE", "EU", 5},
	{"FI", "EU", 5},
	{"FR", "EU", 5},
	{"DE", "EU", 5},
	{"GR", "EU", 5},
	{"HU", "EU", 5},
	{"IE", "EU", 5},
	{"IT", "EU", 5},
	{"LV", "EU", 5},
	{"LI", "EU", 5},
	{"LT", "EU", 5},
	{"LU", "EU", 5},
	{"MT", "EU", 5},
	{"NL", "EU", 5},
	{"PL", "EU", 5},
	{"PT", "EU", 5},
	{"RO", "EU", 5},
	{"SK", "EU", 5},
	{"SI", "EU", 5},
	{"ES", "EU", 5},
	{"SE", "EU", 5},
	{"GB", "EU", 5},  /* input ISO "GB" to : EU regrev 05 */
	{"IL", "IL", 0},
	{"CH", "CH", 0},
	{"TR", "TR", 0},
	{"NO", "NO", 0},
	{"KR", "XY", 3},
	{"AU", "XY", 3},
	{"CN", "XY", 3},  /* input ISO "CN" to : XY regrev 03 */
	{"TW", "XY", 3},
	{"AR", "XY", 3},
	{"MX", "XY", 3}
};

static void *steelhead_wifi_get_country_code(char *ccode)
{
	int size = ARRAY_SIZE(steelhead_wifi_translate_custom_table);
	int i;

	if (!ccode)
		return NULL;

	for (i = 0; i < size; i++)
		if (strcmp(ccode, steelhead_wifi_translate_custom_table[i].iso_abbrev) == 0)
			return &steelhead_wifi_translate_custom_table[i];
	return &steelhead_wifi_translate_custom_table[0];
}

static struct wifi_platform_data steelhead_wifi_control = {
	.set_power      = steelhead_wifi_power,
	.set_reset      = steelhead_wifi_reset,
	.set_carddetect = steelhead_wifi_set_carddetect,
	.mem_prealloc	= steelhead_wifi_mem_prealloc,
	.get_mac_addr	= steelhead_wifi_get_mac_addr,
	.get_country_code = steelhead_wifi_get_country_code,
};

static struct platform_device steelhead_wifi_device = {
        .name           = "bcmdhd_wlan",
        .id             = 1,
        .num_resources  = ARRAY_SIZE(steelhead_wifi_resources),
        .resource       = steelhead_wifi_resources,
        .dev            = {
                .platform_data = &steelhead_wifi_control,
        },
};

static void steelhead_init_wlan_hsmmc_mux(void) {
	/* WLAN SDIO: MMC5 CMD */
	omap_mux_init_signal("sdmmc5_cmd", OMAP_PIN_INPUT_PULLUP);
	/* WLAN SDIO: MMC5 CLK */
	omap_mux_init_signal("sdmmc5_clk", OMAP_PIN_INPUT_PULLUP);
	/* WLAN SDIO: MMC5 DAT[0-3] */
	omap_mux_init_signal("sdmmc5_dat0", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat1", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat2", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat3", OMAP_PIN_INPUT_PULLUP);
}
 
int __init steelhead_init_wlan(void)
{
	int status;

	/* Set up the pin mux for HSMMC 5 */
	steelhead_init_wlan_hsmmc_mux();

	/* Set up all of the GPIOs used to control the BCM4330 */
	status = steelhead_reserve_gpios(wlan_gpios,
					 ARRAY_SIZE(wlan_gpios),
					 "steelhead_wlan", true);
	if (status)
		return status;

	/* Preallocate the SKB memory to be used by the WiFi driver */
	steelhead_init_wifi_mem();

	/* Register the driver and we are done */
	return platform_device_register(&steelhead_wifi_device);
}
