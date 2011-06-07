/* Board support file for Google Steelhead Board.
 *
 * Copyright (C) 2011 Google, Inc.
 * Copyright (C) 2010 Texas Instruments
 *
 * Based on mach-omap2/board-tuna.c
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
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/usb/otg.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/wl12xx.h>

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <plat/display.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/panel-generic-dpi.h>
#include "timer-gp.h"

#include "hsmmc.h"
#include "control.h"
#include "mux.h"
#include "board-steelhead.h"

#include <linux/i2c.h>
#include <plat/i2c.h>
#include <linux/tas5713.h>
#include <linux/steelhead_avr.h>
#include <linux/aah_timesync.h>

#define GPIO_HUB_POWER		1
#define GPIO_HUB_NRESET		62
#define GPIO_WIFI_PMENA		43
#define GPIO_WIFI_IRQ		53

#define AVR_INT_GPIO_ID 40
#define TAS5713_RESET_GPIO_ID 42
#define TAS5713_PDN_GPIO_ID 44

/* wl127x BT, FM, GPS connectivity chip */
static int wl1271_gpios[] = {46, -1, -1};
static struct platform_device wl1271_device = {
	.name	= "kim",
	.id	= -1,
	.dev	= {
		.platform_data	= &wl1271_gpios,
	},
};

static struct gpio_led gpio_leds[] = {
	{
		.name			= "steelhead::status1",
		.default_trigger	= "heartbeat",
		.gpio			= 7,
	},
	{
		.name			= "steelhead::status2",
		.default_trigger	= "mmc0",
		.gpio			= 8,
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

static struct timesync_platform_data timesync_pdata = {
	.get_raw_counter = steelhead_get_raw_counter,
	.get_raw_counter_nominal_freq = steelhead_get_raw_counter_nominal_freq,
#ifdef CONFIG_AAH_TIMESYNC_DEBUG
	.register_timesync_event_handler = register_timesync_event_handler,
#endif
};

static struct platform_device timesync_device = {
	.name	= "timesync",
	.id	= -1,
	.dev	= {
		.platform_data	= &timesync_pdata,
	},
};

static struct platform_device *steelhead_devices[] __initdata = {
	&leds_gpio,
	&wl1271_device,
	&timesync_device,
};

static void __init steelhead_init_early(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
}

static const struct usbhs_omap_board_data usbhs_bdata __initconst = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset  = false,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

static void __init omap4_ehci_init(void)
{
	int ret;
	struct clk *phy_ref_clk;

	/* FREF_CLK3 provides the 19.2 MHz reference clock to the PHY */
	phy_ref_clk = clk_get(NULL, "auxclk3_ck");
	if (IS_ERR(phy_ref_clk)) {
		pr_err("Cannot request auxclk3\n");
		goto err_clk_get;
	}
	clk_set_rate(phy_ref_clk, 19200000);
	clk_enable(phy_ref_clk);

	/* disable the power to the usb hub prior to init */
	ret = gpio_request(GPIO_HUB_POWER, "hub_power");
	if (ret) {
		pr_err("Cannot request GPIO %d\n", GPIO_HUB_POWER);
		goto err_gpio_request_power;
	}
	gpio_export(GPIO_HUB_POWER, 0);
	gpio_direction_output(GPIO_HUB_POWER, 0);
	gpio_set_value(GPIO_HUB_POWER, 0);

	/* reset phy+hub */
	ret = gpio_request(GPIO_HUB_NRESET, "hub_nreset");
	if (ret) {
		pr_err("Cannot request GPIO %d\n", GPIO_HUB_NRESET);
		goto err_gpio_request_nreset;
	}
	gpio_export(GPIO_HUB_NRESET, 0);
	gpio_direction_output(GPIO_HUB_NRESET, 0);
	gpio_set_value(GPIO_HUB_NRESET, 0);
	gpio_set_value(GPIO_HUB_NRESET, 1);

	usbhs_init(&usbhs_bdata);

	/* enable power to hub */
	gpio_set_value(GPIO_HUB_POWER, 1);
	return;

err_gpio_request_nreset:
	gpio_free(GPIO_HUB_POWER);
err_gpio_request_power:
	clk_put(phy_ref_clk);
err_clk_get:
	pr_err("Unable to initialize EHCI power/reset\n");
	return;

}

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_UTMI,
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
	.mode			= MUSB_PERIPHERAL,
#else
	.mode			= MUSB_OTG,
#endif
	.power			= 100,
};

static struct twl4030_usb_data omap4_usbphy_data = {
	.phy_init	= omap4430_phy_init,
	.phy_exit	= omap4430_phy_exit,
	.phy_power	= omap4430_phy_power,
	.phy_set_clock	= omap4430_phy_set_clk,
	.phy_suspend	= omap4430_phy_suspend,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
	},
	{
		.name		= "wl1271",
		.mmc		= 5,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
		.nonremovable	= true,
	},
	{}	/* Terminator */
};

static struct regulator_consumer_supply steelhead_vmmc_supply[] = {
	{
		.supply = "vmmc",
		.dev_name = "omap_hsmmc.0",
	},
};

static struct regulator_consumer_supply steelhead_vmmc5_supply = {
	.supply = "vmmc",
	.dev_name = "omap_hsmmc.4",
};

static struct regulator_init_data steelhead_vmmc5 = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &steelhead_vmmc5_supply,
};

static struct fixed_voltage_config steelhead_vwlan = {
	.supply_name = "vwl1271",
	.microvolts = 1800000, /* 1.8V */
	.gpio = GPIO_WIFI_PMENA,
	.startup_delay = 70000, /* 70msec */
	.enable_high = 1,
	.enabled_at_boot = 0,
	.init_data = &steelhead_vmmc5,
};

static struct platform_device omap_vwlan_device = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev = {
		.platform_data = &steelhead_vwlan,
	},
};

struct wl12xx_platform_data steelhead_wlan_data  __initdata = {
	.irq = OMAP_GPIO_IRQ(GPIO_WIFI_IRQ),
	/* Steelhead ref clock is 38.4 MHz */
	.board_ref_clock = 2,
};

static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = container_of(dev,
				struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	if (!pdata) {
		dev_err(dev, "%s: NULL platform data\n", __func__);
		return -EINVAL;
	}
	/* Setting MMC1 Card detect Irq */
	if (pdev->id == 0) {
		ret = twl6030_mmc_card_detect_config();
		if (ret)
			dev_err(dev, "%s: Error card detect config(%d)\n",
				__func__, ret);
		else
			pdata->slots[0].card_detect = twl6030_mmc_card_detect;
	}
	return ret;
}

static __init void omap4_twl6030_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev) {
		pr_err("Failed omap4_twl6030_hsmmc_set_late_init\n");
		return;
	}
	pdata = dev->platform_data;

	pdata->init =	omap4_twl6030_hsmmc_late_init;
}

static int __init omap4_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	struct omap2_hsmmc_info *c;

	omap2_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++)
		omap4_twl6030_hsmmc_set_late_init(c->dev);

	return 0;
}

static struct regulator_init_data steelhead_vaux2 = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data steelhead_vaux3 = {
	.constraints = {
		.min_uV			= 1000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

/* VMMC1 for MMC1 card */
static struct regulator_init_data steelhead_vmmc = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = steelhead_vmmc_supply,
};

static struct regulator_init_data steelhead_vpp = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 2500000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data steelhead_vana = {
	.constraints = {
		.min_uV			= 2100000,
		.max_uV			= 2100000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data steelhead_vcxio = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data steelhead_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data steelhead_vusb = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 =	REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data steelhead_clk32kg = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
	},
};

static struct twl4030_platform_data steelhead_twldata = {
	.irq_base	= TWL6030_IRQ_BASE,
	.irq_end	= TWL6030_IRQ_END,

	/* Regulators */
	.vmmc		= &steelhead_vmmc,
	.vpp		= &steelhead_vpp,
	.vana		= &steelhead_vana,
	.vcxio		= &steelhead_vcxio,
	.vdac		= &steelhead_vdac,
	.vusb		= &steelhead_vusb,
	.vaux2		= &steelhead_vaux2,
	.vaux3		= &steelhead_vaux3,
	.clk32kg	= &steelhead_clk32kg,
	.usb		= &omap4_usbphy_data,
};

static struct i2c_board_info __initdata steelhead_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl6030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = OMAP44XX_IRQ_SYS_1N,
		.platform_data = &steelhead_twldata,
	},
};

static struct steelhead_avr_platform_data steelhead_avr_pdata = {
	/* Reset and Power Down GPIO configuration */
	.interrupt_gpio = AVR_INT_GPIO_ID,
};

static struct i2c_board_info __initdata steelhead_i2c_bus2[] = {
	{
		I2C_BOARD_INFO("steelhead-avr", (0x20)),
		.platform_data = &steelhead_avr_pdata,
	},
};

static struct tas5713_platform_data tas5713_pdata = {
	/* configure McBSP2 as an I2S transmitter */
	.mcbsp_id = OMAP_MCBSP2,

	/* Reset and Power Down GPIO configuration */
	.reset_gpio = TAS5713_RESET_GPIO_ID,
	.pdn_gpio = TAS5713_PDN_GPIO_ID,

	/* MCLK */
	.mclk_out = NULL,

	.get_raw_counter = steelhead_get_raw_counter,
	.get_raw_counter_nominal_freq = steelhead_get_raw_counter_nominal_freq,
};

static struct i2c_board_info __initdata steelhead_i2c_bus4[] = {
	{
		I2C_BOARD_INFO("tas5713", (0x36 >> 1)),
		.platform_data = &tas5713_pdata,
	},
};

static int __init steelhead_i2c_init(void)
{
	/*
	 * Phoenix Audio IC needs I2C1 to
	 * start with 400 KHz or less
	 */
	omap_register_i2c_bus(1, 400, steelhead_i2c1_boardinfo,
			      ARRAY_SIZE(steelhead_i2c1_boardinfo));
	omap_register_i2c_bus(2, 400, steelhead_i2c_bus2,
			      ARRAY_SIZE(steelhead_i2c_bus2));
	omap_register_i2c_bus(3, 400, NULL, 0);
	omap_register_i2c_bus(4, 400, steelhead_i2c_bus4,
			      ARRAY_SIZE(steelhead_i2c_bus4));

	return 0;
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/* WLAN IRQ - GPIO 53 */
	OMAP4_MUX(GPMC_NCS3, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	/* WLAN POWER ENABLE - GPIO 43 */
	OMAP4_MUX(GPMC_A19, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* WLAN SDIO: MMC5 CMD */
	OMAP4_MUX(SDMMC5_CMD, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* WLAN SDIO: MMC5 CLK */
	OMAP4_MUX(SDMMC5_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* WLAN SDIO: MMC5 DAT[0-3] */
	OMAP4_MUX(SDMMC5_DAT0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* gpio 0 - TFP410 PD */
	OMAP4_MUX(KPD_COL1, OMAP_PIN_OUTPUT | OMAP_MUX_MODE3),
	/* dispc2_data23 */
	OMAP4_MUX(USBB2_ULPITLL_STP, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data22 */
	OMAP4_MUX(USBB2_ULPITLL_DIR, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data21 */
	OMAP4_MUX(USBB2_ULPITLL_NXT, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data20 */
	OMAP4_MUX(USBB2_ULPITLL_DAT0, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data19 */
	OMAP4_MUX(USBB2_ULPITLL_DAT1, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data18 */
	OMAP4_MUX(USBB2_ULPITLL_DAT2, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data15 */
	OMAP4_MUX(USBB2_ULPITLL_DAT3, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data14 */
	OMAP4_MUX(USBB2_ULPITLL_DAT4, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data13 */
	OMAP4_MUX(USBB2_ULPITLL_DAT5, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data12 */
	OMAP4_MUX(USBB2_ULPITLL_DAT6, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data11 */
	OMAP4_MUX(USBB2_ULPITLL_DAT7, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data10 */
	OMAP4_MUX(DPM_EMU3, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data9 */
	OMAP4_MUX(DPM_EMU4, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data16 */
	OMAP4_MUX(DPM_EMU5, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data17 */
	OMAP4_MUX(DPM_EMU6, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_hsync */
	OMAP4_MUX(DPM_EMU7, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_pclk */
	OMAP4_MUX(DPM_EMU8, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_vsync */
	OMAP4_MUX(DPM_EMU9, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_de */
	OMAP4_MUX(DPM_EMU10, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data8 */
	OMAP4_MUX(DPM_EMU11, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data7 */
	OMAP4_MUX(DPM_EMU12, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data6 */
	OMAP4_MUX(DPM_EMU13, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data5 */
	OMAP4_MUX(DPM_EMU14, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data4 */
	OMAP4_MUX(DPM_EMU15, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
#ifdef CONFIG_AAH_TIMESYNC_DEBUG
	/* dispc2_data3 - set this pin to be DMTIMER8_PWM_EVT
	 * instead of DISPC2_DATA3
	 */
	OMAP4_MUX(DPM_EMU16, OMAP_PIN_INPUT | OMAP_MUX_MODE1),
#else
	OMAP4_MUX(DPM_EMU16, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
#endif
	/* dispc2_data2 */
	OMAP4_MUX(DPM_EMU17, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data1 */
	OMAP4_MUX(DPM_EMU18, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data0 */
	OMAP4_MUX(DPM_EMU19, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

static struct omap_device_pad serial2_pads[] __initdata = {
	OMAP_MUX_STATIC("uart2_cts.uart2_cts",
			 OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart2_rts.uart2_rts",
			 OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart2_rx.uart2_rx",
			 OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart2_tx.uart2_tx",
			 OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
};

static struct omap_device_pad serial3_pads[] __initdata = {
	OMAP_MUX_STATIC("uart3_cts_rctx.uart3_cts_rctx",
			 OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart3_rts_sd.uart3_rts_sd",
			 OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart3_rx_irrx.uart3_rx_irrx",
			 OMAP_PIN_INPUT | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart3_tx_irtx.uart3_tx_irtx",
			 OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
};

static struct omap_device_pad serial4_pads[] __initdata = {
	OMAP_MUX_STATIC("uart4_rx.uart4_rx",
			 OMAP_PIN_INPUT | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart4_tx.uart4_tx",
			 OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
};

static struct omap_board_data serial2_data = {
	.id             = 1,
	.pads           = serial2_pads,
	.pads_cnt       = ARRAY_SIZE(serial2_pads),
};

static struct omap_board_data serial3_data = {
	.id             = 2,
	.pads           = serial3_pads,
	.pads_cnt       = ARRAY_SIZE(serial3_pads),
};

static struct omap_board_data serial4_data = {
	.id             = 3,
	.pads           = serial4_pads,
	.pads_cnt       = ARRAY_SIZE(serial4_pads),
};

static inline void board_serial_init(void)
{
	struct omap_board_data bdata;
	bdata.flags     = 0;
	bdata.pads      = NULL;
	bdata.pads_cnt  = 0;
	bdata.id        = 0;
	/* pass dummy data for UART1 */
	omap_serial_init_port(&bdata);

	omap_serial_init_port(&serial2_data);
	omap_serial_init_port(&serial3_data);
	omap_serial_init_port(&serial4_data);
}
#else
#define board_mux	NULL

static inline void board_serial_init(void)
{
	omap_serial_init();
}
#endif

struct steelhead_gpio {
	unsigned gpio_id;
	const char *gpio_name;
	int pin_mode;
	int init_state;
};

static int steelhead_reserve_gpios(struct steelhead_gpio *data, int count,
				   const char *log_tag)
{
	int i;
	for (i = 0; i < count; ++i) {
		int status;
		struct steelhead_gpio *g = (data + i);
		char pin_name[32];

		snprintf(pin_name, sizeof(pin_name), "gpio_%d", g->gpio_id);
		omap_mux_init_signal(pin_name, g->pin_mode);

		/* Setup IRQ handler for the HDP input */
		status = gpio_request_one(
				g->gpio_id, g->init_state, g->gpio_name);

		if (status)
			pr_err("%s: failed to reserve gpio \"%s\" driver will"
					" not load. (status %d)\n",
					log_tag, g->gpio_name, status);
			return status;
	}

	return 0;
}

/******************************************************************************
 *                                                                            *
 *              TAS5713 Class-D Audio Amplifier Initialization                *
 *                                                                            *
 ******************************************************************************/

static const unsigned long tas5713_mclk_rate = 12288000;

static void steelhead_platform_init_audio(void)
{
	struct clk *m3x2_clk = NULL;
	struct clk *mclk_out = NULL;
	int res;
	unsigned long tgt_rate = (tas5713_mclk_rate * 5);

	/* Grab a hold of the GPIOs used to control the TAS5713 Reset and Power
	 * Down lines.  Configure them to be outputs, reserve them in the GPIO
	 * framwork, and set them to be driven low initially (hold the chip in
	 * reset)
	 */
	static struct steelhead_gpio tas5713_gpios[] = {
		{
			.gpio_id = TAS5713_RESET_GPIO_ID,
			.gpio_name = "tas5713_reset",
			.pin_mode = OMAP_PIN_OUTPUT,
			.init_state = GPIOF_OUT_INIT_LOW,
		},
		{
			.gpio_id = TAS5713_PDN_GPIO_ID,
			.gpio_name = "tas5713_pdn",
			.pin_mode = OMAP_PIN_OUTPUT,
			.init_state = GPIOF_OUT_INIT_LOW,
		},
	};
	if (steelhead_reserve_gpios(tas5713_gpios, ARRAY_SIZE(tas5713_gpios),
				"tas5713"))
		return;

	/* We use fref_clk1_out to drive MCLK on steelhead.  It should be
	 * configured to run from the M3 X2 output of DPLL_PER.  With a system
	 * clock of 38.4MHz, and default multiplier of 40 as the reference for
	 * the M3X2 divider, we should end up setting the M3 divider to 25 and
	 * the fref_clk1_out divider to 5 to generate a 12.288MHz MCLK which
	 * should be 256 * 48kHz.
	 */
	m3x2_clk = clk_get(NULL, "dpll_per_m3x2_ck");
	if (IS_ERR_OR_NULL(m3x2_clk)) {
		pr_err("tas5713: failed to dpll_per_m3x2_ck\n");
		goto err_clk_get_dpll;
	}
	mclk_out = clk_get(NULL, "auxclk1_ck");
	if (IS_ERR_OR_NULL(mclk_out)) {
		pr_err("tas5713: failed to auxclk1_ck\n");
		goto err_clk_get_aux;
	}

	res = clk_set_rate(m3x2_clk, tgt_rate);
	if (res < 0) {
		pr_err("tas5713: failed to set m3x2 clk"
			       "rate to %lu (res = %d).  "
			       "driver will not load.\n",
			       tgt_rate, res);
		goto err_set_clk;
	}

	res = clk_set_parent(mclk_out, m3x2_clk);
	if (res < 0) {
		pr_err("tas5713: failed to set reference clock"
				" for fref_clk1_out (res = %d)  "
			       "driver will not load.\n", res);
		goto err_set_clk;
	}

	res = clk_set_rate(mclk_out, tas5713_mclk_rate);
	if (res < 0) {
		pr_err("tas5713: failed to set mclk_out"
			       "rate to %lu (res = %d).  "
			       "driver will not load.\n",
			       tas5713_mclk_rate, res);
		goto err_set_clk;
	}

	/* Stash the clock and enable the pin. */
	tas5713_pdata.mclk_out = mclk_out;
	clk_put(m3x2_clk);
	omap_mux_init_signal("fref_clk1_out", OMAP_PIN_OUTPUT);
	return;

err_set_clk:
	clk_put(mclk_out);
err_clk_get_aux:
	clk_put(m3x2_clk);
err_clk_get_dpll:
	/* TODO: release gpios, but we're fatal anyway. */
	return;
}

/******************************************************************************
 *                                                                            *
 *           AVR Front Panel Controls and LED Initialization                  *
 *                                                                            *
 ******************************************************************************/

static void steelhead_platform_init_avr(void)
{
	/* Grab a hold of the GPIO used to control the AVR interrupt
	 * request line.  Configure it to be an input and reserve it in
	 * the GPIO framwork.
	 */
	static struct steelhead_gpio avr_gpios[] = {
		{
			.gpio_id = AVR_INT_GPIO_ID,
			.gpio_name = "avr_int",
			.pin_mode = OMAP_PIN_INPUT_PULLUP,
			.init_state = GPIOF_IN,
		},
	};

	if (steelhead_reserve_gpios(avr_gpios, ARRAY_SIZE(avr_gpios),
				"steelhead-avr"))
		return;

}

/* Display DVI */
#define STEELHEAD_DVI_TFP410_POWER_DOWN_GPIO	0

static int omap4_steelhead_enable_dvi(struct omap_dss_device *dssdev)
{
	gpio_set_value(dssdev->reset_gpio, 1);
	return 0;
}

static void omap4_steelhead_disable_dvi(struct omap_dss_device *dssdev)
{
	gpio_set_value(dssdev->reset_gpio, 0);
}

/* Using generic display panel */
static struct panel_generic_dpi_data omap4_dvi_panel = {
	.name			= "generic_720p",
	.platform_enable	= omap4_steelhead_enable_dvi,
	.platform_disable	= omap4_steelhead_disable_dvi,
};

struct omap_dss_device omap4_steelhead_dvi_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "dvi",
	.driver_name		= "generic_dpi_panel",
	.data			= &omap4_dvi_panel,
	.phy.dpi.data_lines	= 24,
	.reset_gpio		= STEELHEAD_DVI_TFP410_POWER_DOWN_GPIO,
	.channel		= OMAP_DSS_CHANNEL_LCD2,
};

int __init omap4_steelhead_dvi_init(void)
{
	int r;

	/* Requesting TFP410 DVI GPIO and disabling it, at bootup */
	r = gpio_request_one(omap4_steelhead_dvi_device.reset_gpio,
				GPIOF_OUT_INIT_LOW, "DVI PD");
	if (r)
		pr_err("Failed to get DVI powerdown GPIO\n");

	return r;
}

#if 0
static void omap4_steelhead_hdmi_mux_init(void)
{
	/* PAD0_HDMI_HPD_PAD1_HDMI_CEC */
	omap_mux_init_signal("hdmi_hpd",
			OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("hdmi_cec",
			OMAP_PIN_INPUT_PULLUP);
	/* PAD0_HDMI_DDC_SCL_PAD1_HDMI_DDC_SDA */
	omap_mux_init_signal("hdmi_ddc_scl",
			OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("hdmi_ddc_sda",
			OMAP_PIN_INPUT_PULLUP);
}

static int omap4_steelhead_panel_enable_hdmi(struct omap_dss_device *dssdev)
{
	int status;

	status = gpio_request_one(HDMI_GPIO_HPD, GPIOF_OUT_INIT_HIGH,
							"hdmi_gpio_hpd");
	if (status) {
		pr_err("Cannot request GPIO %d\n", HDMI_GPIO_HPD);
		return status;
	}
	status = gpio_request_one(HDMI_GPIO_LS_OE, GPIOF_OUT_INIT_HIGH,
							"hdmi_gpio_ls_oe");
	if (status) {
		pr_err("Cannot request GPIO %d\n", HDMI_GPIO_LS_OE);
		goto error1;
	}

	return 0;

error1:
	gpio_free(HDMI_GPIO_HPD);

	return status;
}

static void omap4_steelhead_panel_disable_hdmi(struct omap_dss_device *dssdev)
{
	gpio_free(HDMI_GPIO_LS_OE);
	gpio_free(HDMI_GPIO_HPD);
}

static struct omap_dss_device  omap4_steelhead_hdmi_device = {
	.name = "hdmi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_HDMI,
	.platform_enable = omap4_steelhead_panel_enable_hdmi,
	.platform_disable = omap4_steelhead_panel_disable_hdmi,
	.channel = OMAP_DSS_CHANNEL_DIGIT,
};
#endif

static struct omap_dss_device *omap4_steelhead_dss_devices[] = {
	&omap4_steelhead_dvi_device,
#if 0
	&omap4_steelhead_hdmi_device,
#endif
};

static struct omap_dss_board_info omap4_steelhead_dss_data = {
	.num_devices	= ARRAY_SIZE(omap4_steelhead_dss_devices),
	.devices	= omap4_steelhead_dss_devices,
	.default_device	= &omap4_steelhead_dvi_device,
};

void omap4_steelhead_display_init(void)
{
	int r;

	r = omap4_steelhead_dvi_init();
	if (r)
		pr_err("error initializing steelhead DVI\n");

#if 0
	omap4_steelhead_hdmi_mux_init();
#endif
	omap_display_init(&omap4_steelhead_dss_data);
}

static void __init steelhead_init(void)
{
	int package = OMAP_PACKAGE_CBS;

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;
	omap4_mux_init(board_mux, NULL, package);

	if (wl12xx_set_platform_data(&steelhead_wlan_data))
		pr_err("error setting wl12xx data\n");

	steelhead_platform_init_avr();
	steelhead_platform_init_audio();

	steelhead_i2c_init();
	platform_add_devices(steelhead_devices, ARRAY_SIZE(steelhead_devices));
	platform_device_register(&omap_vwlan_device);
	board_serial_init();
	omap4_twl6030_hsmmc_init(mmc);
	omap4_ehci_init();
	usb_musb_init(&musb_board_data);
	omap4_steelhead_android_usb_init();
	omap4_steelhead_display_init();
	steelhead_platform_init_counter();
}

static void __init steelhead_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}

MACHINE_START(STEELHEAD, "Steelhead")
	/* Maintainer: Google, Inc */
	.boot_params	= 0x80000100,
	.reserve	= omap_reserve,
	.map_io		= steelhead_map_io,
	.init_early	= steelhead_init_early,
	.init_irq	= gic_init_irq,
	.init_machine	= steelhead_init,
	.timer		= &omap_timer,
MACHINE_END


