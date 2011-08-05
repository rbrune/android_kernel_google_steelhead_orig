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
#include <linux/init.h>
#include <asm/gpio.h>

#include "board-steelhead.h"
#include "mux.h"

#define GPIO_BT_EN			46
#define GPIO_BT_EN_MUX_NAME             "gpmc_a22.gpio_46"
#define GPIO_BT_RST_N			52
#define GPIO_BT_RST_N_MUX_NAME          "gpmc_ncs2.gpio_52"
#define GPIO_BT_HOST_WAKE_BT		45
#define GPIO_BT_HOST_WAKE_BT_MUX_NAME   "gpmc_a21.gpio_45"
#define GPIO_BT_WAKEUP_HOST		47
#define GPIO_BT_WAKEUP_HOST_MUX_NAME    "gpmc_a23.gpio_47"

static struct steelhead_gpio_reservation bluetooth_gpios[] = {
	{
		.gpio_name = "bt_en",
		.gpio_id = GPIO_BT_EN,
		.mux_name = GPIO_BT_EN_MUX_NAME,
		.pin_mode = OMAP_PIN_OUTPUT,
		.init_state = GPIOF_OUT_INIT_LOW,
	},
	{
		.gpio_name = "bt_rst_n",
		.gpio_id = GPIO_BT_RST_N,
		.mux_name = GPIO_BT_RST_N_MUX_NAME,
		.pin_mode = OMAP_PIN_OUTPUT,
		.init_state = GPIOF_OUT_INIT_LOW,
	},
	{
		.gpio_name = "bt_host_wake_bt",
		.gpio_id = GPIO_BT_HOST_WAKE_BT,
		.mux_name = GPIO_BT_HOST_WAKE_BT_MUX_NAME,
		.pin_mode = OMAP_PIN_OUTPUT,
		.init_state = GPIOF_OUT_INIT_LOW,
	},
	{
		.gpio_name = "bt_wakeup_host",
		.gpio_id = GPIO_BT_WAKEUP_HOST,
		.mux_name = GPIO_BT_WAKEUP_HOST_MUX_NAME,
		.pin_mode = OMAP_PIN_INPUT,
		.init_state = GPIOF_IN,
	},
};

int __init steelhead_init_bluetooth(void)
{
	return steelhead_reserve_gpios(bluetooth_gpios,
			ARRAY_SIZE(bluetooth_gpios),
			"steelhead_bluetooth");
}
