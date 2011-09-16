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
#include <linux/platform_device.h>
#include <plat/serial.h>
#include <plat/omap-serial.h>

#include "board-steelhead.h"
#include "mux.h"

static struct omap_uart_port_info steelhead_uart2_info __initdata = {
	.use_dma	= 0,
	.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
	.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
	.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
	.auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
	.wake_peer	= bcm_bt_lpm_exit_lpm_locked,
	.rts_mux_driver_control = 1,
};

static struct omap_device_pad steelhead_bt_serial_pads[] __initdata = {
	{
		.name   = "uart2_cts.uart2_cts",
		.enable = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
	{
		.name   = "uart2_rts.uart2_rts",
		.enable = OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name   = "uart2_tx.uart2_tx",
		.enable = OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name   = "uart2_rx.uart2_rx",
		.enable = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
};

static struct platform_device bcm4330_bluetooth_device = {
	.name = "bcm4330_bluetooth",
	.id = -1,
};

static char *btaddr;
module_param(btaddr, charp, S_IRUGO);
MODULE_PARM_DESC(btaddr, "bluetooth address");

int __init steelhead_init_bluetooth(void)
{
	/* Setup the GPIOs we use to control the regulator enable, reset, and
	 * wakeup signals */
	omap_mux_init_signal("gpmc_a22.gpio_46", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("gpmc_ncs2.gpio_52", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("gpmc_a21.gpio_45", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("gpmc_a23.gpio_47",
			     OMAP_WAKEUP_EN | OMAP_PIN_INPUT);

	/* Add the serial port we will use to communicate with the BCM4330 */
	omap_serial_init_port_pads(1,
				   steelhead_bt_serial_pads,
				   ARRAY_SIZE(steelhead_bt_serial_pads),
				   &steelhead_uart2_info);

	/* Add the platform device used for BT power management */
	platform_device_register(&bcm4330_bluetooth_device);

	return 0;
}
