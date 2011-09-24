/* USB Host (EHCI) support for Samsung Tuna Board.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>

#include <plat/usb.h>

#include "board-steelhead.h"
#include "mux.h"

#define GPIO_ETHERNET_NENABLE	1
#define GPIO_ETHERNET_NRESET	62

static struct usbhs_omap_board_data usbhs_bdata __initconst = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset  = false,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

/* initially 3.3V not enabled and reset is asserted */
static struct gpio steelhead_ethernet_gpios[] __initdata = {
	{ GPIO_ETHERNET_NENABLE, GPIOF_OUT_INIT_HIGH, "ethernet_nenable"  },
	{ GPIO_ETHERNET_NRESET,	 GPIOF_OUT_INIT_LOW,  "ethernet_nreset" },
};

void __init omap4_steelhead_ehci_init(void)
{
	int ret = 0;
	struct clk *phy_ref_clk;

	omap_mux_init_gpio(GPIO_ETHERNET_NENABLE,
			   OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_HIGH);
	omap_mux_init_gpio(GPIO_ETHERNET_NRESET,
			   OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_HIGH);
	omap_mux_init_signal("usbb1_ulpitll_clk.usbb1_ulpiphy_clk",
			     OMAP_PIN_INPUT_PULLDOWN |
			     OMAP_PIN_OFF_INPUT_PULLDOWN);
	omap_mux_init_signal("usbb1_ulpitll_stp.usbb1_ulpiphy_stp",
			     OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW);
	omap_mux_init_signal("usbb1_ulpitll_dir.usbb1_ulpiphy_dir",
			     OMAP_PIN_INPUT | OMAP_PIN_OFF_INPUT_PULLDOWN);
	omap_mux_init_signal("usbb1_ulpitll_nxt.usbb1_ulpiphy_nxt",
			     OMAP_PIN_INPUT | OMAP_PIN_OFF_INPUT_PULLDOWN);
	omap_mux_init_signal("usbb1_ulpitll_dat0.usbb1_ulpiphy_dat0",
			     OMAP_PIN_INPUT | OMAP_PIN_OFF_INPUT_PULLDOWN);
	omap_mux_init_signal("usbb1_ulpitll_dat0.usbb1_ulpiphy_dat1",
			     OMAP_PIN_INPUT | OMAP_PIN_OFF_INPUT_PULLDOWN);
	omap_mux_init_signal("usbb1_ulpitll_dat0.usbb1_ulpiphy_dat2",
			     OMAP_PIN_INPUT | OMAP_PIN_OFF_INPUT_PULLDOWN);
	omap_mux_init_signal("usbb1_ulpitll_dat0.usbb1_ulpiphy_dat3",
			     OMAP_PIN_INPUT | OMAP_PIN_OFF_INPUT_PULLDOWN);
	omap_mux_init_signal("usbb1_ulpitll_dat0.usbb1_ulpiphy_dat4",
			     OMAP_PIN_INPUT | OMAP_PIN_OFF_INPUT_PULLDOWN);
	omap_mux_init_signal("usbb1_ulpitll_dat0.usbb1_ulpiphy_dat5",
			     OMAP_PIN_INPUT | OMAP_PIN_OFF_INPUT_PULLDOWN);
	omap_mux_init_signal("usbb1_ulpitll_dat0.usbb1_ulpiphy_dat6",
			     OMAP_PIN_INPUT | OMAP_PIN_OFF_INPUT_PULLDOWN);
	omap_mux_init_signal("usbb1_ulpitll_dat0.usbb1_ulpiphy_dat7",
			     OMAP_PIN_INPUT | OMAP_PIN_OFF_INPUT_PULLDOWN);

	/* Older boards used active high for enable */
	if (cpu_is_omap443x()) {
		steelhead_ethernet_gpios[0].flags = GPIOF_OUT_INIT_LOW;
	}

	ret = gpio_request_array(steelhead_ethernet_gpios,
				 ARRAY_SIZE(steelhead_ethernet_gpios));
	if (ret) {
		pr_err("omap: ehci: Cannot request gpios\n");
		return;
	}

	/* FREF_CLK3 provides the 38.4 MHz reference clock to the PHY */
	omap_mux_init_signal("fref_clk3_out", OMAP_PIN_OUTPUT | OMAP_MUX_MODE0);

	phy_ref_clk = clk_get(NULL, "auxclk3_ck");
	if (IS_ERR(phy_ref_clk)) {
		pr_err("omap: ehci: Cannot request auxclk3");
		return;
	}
	ret = clk_set_rate(phy_ref_clk, 38400000);
	if (ret < 0) {
		pr_err("omap: ehci: Cannot clk_set_rate  auxclk3 err %d", ret);
		goto err_clk_set_rate;
	}
	ret = clk_enable(phy_ref_clk);
	if (ret < 0) {
		pr_err("omap: ehci: Cannot clk_enable auxclk3 err %d", ret);
		goto err_clk_enable;
	}

	udelay(100);

	/* enable 3.3V power */
	if (cpu_is_omap443x()) {
		/* Older boards used active high for enable */
		gpio_set_value(GPIO_ETHERNET_NENABLE, 1);
	} else {
		gpio_set_value(GPIO_ETHERNET_NENABLE, 0);
	}

	/* datasheet says minimum reset assertion time is 1us */
	udelay(2);
	/* take out of reset */
	gpio_set_value(GPIO_ETHERNET_NRESET, 1);

	/* Everything went well with phy clock, pass it to ehci driver for
	 * low power managment now
	 */
	usbhs_bdata.transceiver_clk[0] = phy_ref_clk;

	usbhs_init(&usbhs_bdata);

	pr_info("usb:ehci initialized");
	return;

err_clk_set_rate:
err_clk_enable:
	clk_put(phy_ref_clk);
	return;
}
