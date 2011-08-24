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
#include <linux/ion.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/memblock.h>
#include <linux/moduleparam.h>
#include <linux/omap_ion.h>
#include <linux/usb/otg.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/reboot.h>

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <video/omapdss.h>
#include <linux/omapfb.h>
#include <video/omap-panel-generic-dpi.h>
#include <plat/vram.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/dma.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/remoteproc.h>
#include <mach/id.h>
#include "timer-gp.h"

#include "hsmmc.h"
#include "control.h"
#include "mux.h"
#include "board-steelhead.h"
#include "common-board-devices.h"
#include "pm.h"

#include <linux/i2c.h>
#include <plat/i2c.h>
#include <plat/mcasp.h>
#include <linux/tas5713.h>
#include <linux/steelhead_avr.h>
#include <linux/aah_localtime.h>
#include <sound/pcm.h>

#define GPIO_HUB_POWER		1
#define GPIO_HUB_NRESET		62

#if 0 /* TBD */
#define GPIO_NFC_IRQ 17
#define GPIO_NFC_FIRMWARE 172
#define GPIO_NFC_EN 173
#endif

#define STEELHEAD_RAMCONSOLE_START	(PLAT_PHYS_OFFSET + SZ_512M)
#define STEELHEAD_RAMCONSOLE_SIZE	SZ_2M

static struct resource ramconsole_resources[] = {
	{
		.flags  = IORESOURCE_MEM,
		.start	= STEELHEAD_RAMCONSOLE_START,
		.end	= (STEELHEAD_RAMCONSOLE_START +
			   STEELHEAD_RAMCONSOLE_SIZE - 1),
	},
};

static struct platform_device ramconsole_device = {
	.name           = "ram_console",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(ramconsole_resources),
	.resource       = ramconsole_resources,
};

int steelhead_hw_rev;

static bool enable_sr = true;
module_param(enable_sr, bool, S_IRUSR | S_IRGRP | S_IROTH);

#define HW_REV_0_GPIO_ID 182
#define HW_REV_1_GPIO_ID 101
#define HW_REV_2_GPIO_ID 171
static struct steelhead_gpio_reservation hwrev_gpios[] = {
	{
		.gpio_id = HW_REV_0_GPIO_ID,
		.gpio_name = "board_id_0",
		.mux_name = "fref_clk2_out.gpio_182",
		.pin_mode = OMAP_PIN_INPUT,
		.init_state = GPIOF_IN,
	},
	{
		.gpio_id = HW_REV_1_GPIO_ID,
		.gpio_name = "board_id_1",
		.mux_name = "gpmc_ncs4.gpio_101",
		.pin_mode = OMAP_PIN_INPUT,
		.init_state = GPIOF_IN,
	},
	{
		.gpio_id = HW_REV_2_GPIO_ID,
		.gpio_name = "board_id_2",
		.mux_name = "kpd_col3.gpio_171",
		.pin_mode = OMAP_PIN_INPUT,
		.init_state = GPIOF_IN,
	},
};

static const char const *omap4_steelhead_hw_name[] = {
	[STEELHEAD_REV_PRE_EVT] = "Steelhead pre-EVT",
};

static const char *omap4_steelhead_hw_rev_name(void)
{
	int num = ARRAY_SIZE(omap4_steelhead_hw_name);

	if (steelhead_hw_rev >= num ||
	    !omap4_steelhead_hw_name[steelhead_hw_rev])
		return "Steelhead unknown version";

	return omap4_steelhead_hw_name[steelhead_hw_rev];
}

static void __init omap4_steelhead_init_hw_rev(void)
{
	int ret;
	int i;

	/* initially an invalid value */
	steelhead_hw_rev = ARRAY_SIZE(omap4_steelhead_hw_name);

	/* mux init */
	ret = steelhead_reserve_gpios(hwrev_gpios, ARRAY_SIZE(hwrev_gpios),
				      "hw_rev");

	if (ret) {
		pr_err("unable to reserve gpios for hw rev\n");
		return;
	}

	for (i = 0; i < ARRAY_SIZE(hwrev_gpios); i++)
		steelhead_hw_rev |= gpio_get_value(hwrev_gpios[i].gpio_id) << i;

	pr_info("Steelhead HW revision: %02x (%s), cpu %s\n", steelhead_hw_rev,
		omap4_steelhead_hw_rev_name(),
		cpu_is_omap443x() ? "OMAP4430" : "OMAP4460");
}

#define OMAP_STEELHEAD_ION_HEAP_SECURE_INPUT_SIZE	(SZ_1M * 30)
#define OMAP_STEELHEAD_ION_HEAP_TILER_SIZE		SZ_128M
#define OMAP_STEELHEAD_ION_HEAP_LARGE_SURFACES_SIZE	SZ_32M
#define PHYS_ADDR_SMC_SIZE	(SZ_1M * 3)
#define PHYS_ADDR_SMC_MEM	(0x80000000 + SZ_1G - PHYS_ADDR_SMC_SIZE)
#define PHYS_ADDR_DUCATI_SIZE	(SZ_1M * 103)
#define PHYS_ADDR_DUCATI_MEM	(PHYS_ADDR_SMC_MEM - PHYS_ADDR_DUCATI_SIZE -\
				 OMAP_STEELHEAD_ION_HEAP_SECURE_INPUT_SIZE)

static struct ion_platform_data sh_ion_data = {
	.nr = 3,
	.heaps = {
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = OMAP_ION_HEAP_SECURE_INPUT,
			.name = "secure_input",
			.base = PHYS_ADDR_SMC_MEM -
				OMAP_STEELHEAD_ION_HEAP_SECURE_INPUT_SIZE,
			.size = OMAP_STEELHEAD_ION_HEAP_SECURE_INPUT_SIZE,
		},
		{	.type = OMAP_ION_HEAP_TYPE_TILER,
			.id = OMAP_ION_HEAP_TILER,
			.name = "tiler",
			.base = PHYS_ADDR_DUCATI_MEM -
					OMAP_STEELHEAD_ION_HEAP_TILER_SIZE,
			.size = OMAP_STEELHEAD_ION_HEAP_TILER_SIZE,
		},
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = OMAP_ION_HEAP_LARGE_SURFACES,
			.name = "large_surfaces",
			.base = 0x80000000 + SZ_512M + SZ_2M,
			.size = OMAP_STEELHEAD_ION_HEAP_LARGE_SURFACES_SIZE,
		},
	},
};

static struct platform_device steelhead_ion_device = {
	.name = "ion-omap4",
	.id = -1,
	.dev = {
		.platform_data = &sh_ion_data,
	},
};

static struct platform_device *steelhead_devices[] __initdata = {
	&ramconsole_device,
	&steelhead_ion_device,
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

static struct gpio steelhead_ehci_gpios[] __initdata = {
	{ GPIO_HUB_POWER,	GPIOF_OUT_INIT_LOW,  "hub_power"  },
	{ GPIO_HUB_NRESET,	GPIOF_OUT_INIT_LOW,  "hub_nreset" },
};

static void __init omap4_ehci_init(void)
{
	int ret;
	struct clk *phy_ref_clk;

	/* FREF_CLK3 provides the 38.4 MHz reference clock to the PHY */
	phy_ref_clk = clk_get(NULL, "auxclk3_ck");
	if (IS_ERR(phy_ref_clk)) {
		pr_err("Cannot request auxclk3\n");
		goto err_clk_get;
	}
	clk_set_rate(phy_ref_clk, 38400000);
	clk_enable(phy_ref_clk);

	/* disable the power to the usb hub prior to init and reset phy+hub */
	ret = gpio_request_array(steelhead_ehci_gpios,
				 ARRAY_SIZE(steelhead_ehci_gpios));
	if (ret) {
		pr_err("Unable to initialize EHCI power/reset\n");
		goto err_gpio_request_array;
	}
	gpio_export(GPIO_HUB_POWER, 0);
	gpio_export(GPIO_HUB_NRESET, 0);
	gpio_set_value(GPIO_HUB_NRESET, 1);

	usbhs_init(&usbhs_bdata);

	/* enable power to hub */
	gpio_set_value(GPIO_HUB_POWER, 1);
	return;

err_gpio_request_array:
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
		.nonremovable	= true,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.ocr_mask	= MMC_VDD_29_30	| MMC_VDD_30_31,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
	},
	{
		.name		= "omap_wlan",
		.mmc		= 5,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
		.ocr_mask	= MMC_VDD_27_28 |
				  MMC_VDD_28_29 |
				  MMC_VDD_29_30 |
				  MMC_VDD_30_31 |
				  MMC_VDD_31_32 |
				  MMC_VDD_32_33 |
				  MMC_VDD_33_34 |
				  MMC_VDD_34_35 |
				  MMC_VDD_35_36,
		.nonremovable	= false,
		.mmc_data	= &steelhead_wifi_data,
	},
	{}	/* Terminator */
};

static struct regulator_consumer_supply steelhead_vmmc_supply[] = {
	{
		.supply = "vmmc",
		.dev_name = "omap_hsmmc.0",
	},
};

static int __init omap4_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	omap2_hsmmc_init(controllers);
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
		.min_uV			= 3000000,
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

static struct regulator_consumer_supply steelhead_vcxio_supply[] = {
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi1"),
};

/* always on for Tungsten because it provides power to a number of CPU
 * inputs in VDDA_DPLL_CORE_AUDIO, VDDA_DPLL_MPU, VDDA_DPLL_IVA_PER,
 * VDDS_DV_BANK7, VDDA_USBA0OTG_1P8V as well as VDDA_DSI1, VDDA_DSI2,
 * VDDA_CSI21, VDDA_CSI22.
 */
static struct regulator_init_data steelhead_vcxio = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
	},
	.num_consumer_supplies	= ARRAY_SIZE(steelhead_vcxio_supply),
	.consumer_supplies	= steelhead_vcxio_supply,
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

/* clk32kg is a twl6030 32khz clock modeled as a regulator, used by wifi */
static struct regulator_init_data steelhead_clk32kg = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.always_on              = true,
	},
};

/* define a regulator so on 4460 this can be turned off because
 * it's not needed.
 * on 4430 however, it needs to be always on to provide vcore3 to the CPU.
 */
static struct regulator_init_data steelhead_vdd3 = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.always_on              = true,
	},
};

/* define a regulator so on 4460 this can be turned off because
 * it's not needed.
 * on 4430 however, it needs to be always on to provide vmem to the CPU.
 */
static struct regulator_init_data steelhead_vmem = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.always_on              = true,
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

	/* SMPS */
	.vdd3		= &steelhead_vdd3,
	.vmem		= &steelhead_vmem,
};

static struct omap_device_pad serial3_pads[] __initdata = {
	{
		.name   = "uart3_tx_irtx.uart3_tx_irtx",
		.enable = OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name   = "uart3_rx_irrx.uart3_rx_irrx",
		.flags  = OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable = OMAP_PIN_INPUT | OMAP_MUX_MODE0,
		.idle   = OMAP_PIN_INPUT | OMAP_MUX_MODE0,
	},
};

static inline void __init board_serial_init(void)
{
	omap_serial_init_port_pads(0, NULL, 0, NULL);
	/* port1 for bluetooth is done in board-steelhead-bluetooth.c */
	omap_serial_init_port_pads(2, serial3_pads, ARRAY_SIZE(serial3_pads),
			NULL);
	omap_serial_init_port_pads(3, NULL, 0, NULL);
}

int __init steelhead_reserve_gpios(struct steelhead_gpio_reservation *data,
				   int count,
				   const char *log_tag)
{
	int i;
	for (i = 0; i < count; ++i) {
		int status;
		struct steelhead_gpio_reservation *g = (data + i);

		status = omap_mux_init_signal(g->mux_name, g->pin_mode);
		if (status) {
			pr_err("%s: failed to set gpio pin mux for gpio \"%s\""
					" to mode %d.  (status %d)\n",
					log_tag, g->mux_name,
					g->pin_mode, status);
			return status;
		}

		status = gpio_request_one(
				g->gpio_id, g->init_state, g->gpio_name);

		if (status) {
			pr_err("%s: failed to reserve gpio \"%s\""
				       "(status %d)\n",
					log_tag, g->gpio_name, status);
			return status;
		}
	}

	return 0;
}

/******************************************************************************
 *                                                                            *
 *              TAS5713 Class-D Audio Amplifier Initialization                *
 *                                                                            *
 ******************************************************************************/

#define TAS5713_INTERFACE_EN_GPIO_ID 40
#define TAS5713_RESET_GPIO_ID 42
#define TAS5713_PDN_GPIO_ID 44

static struct tas5713_platform_data tas5713_pdata = {
	/* configure McBSP2 as an I2S transmitter */
	.mcbsp_id = OMAP_MCBSP2,

	/* Reset and Power Down GPIO configuration */
	.interface_en_gpio = TAS5713_INTERFACE_EN_GPIO_ID,
	.reset_gpio = TAS5713_RESET_GPIO_ID,
	.pdn_gpio = TAS5713_PDN_GPIO_ID,

	/* MCLK */
	.mclk_out = NULL,

	.get_raw_counter = steelhead_get_raw_counter,
	.get_raw_counter_nominal_freq = steelhead_get_raw_counter_nominal_freq,
};

static const unsigned long tas5713_mclk_rate = 12288000;

static void steelhead_platform_init_tas5713_audio(void)
{
	struct clk *m3x2_clk = NULL;
	struct clk *mclk_out = NULL;
	struct clk *mclk_src = NULL;
	struct clk *mcbsp_internal_clk =  NULL;
	struct clk *abe_24m_clk = NULL;
	int res;
	unsigned long tgt_rate = (tas5713_mclk_rate * 5);

	/* Grab a hold of the GPIOs used to control the TAS5713 Reset and Power
	 * Down lines.  Configure them to be outputs, reserve them in the GPIO
	 * framwork, and set them to be driven low initially (hold the chip in
	 * reset)
	 */
	static struct steelhead_gpio_reservation tas5713_gpios[] = {
		{
			.gpio_id = TAS5713_INTERFACE_EN_GPIO_ID,
			.gpio_name = "tas5713_interface_en",
			.mux_name = "gpmc_a16.gpio_40",
			.pin_mode = OMAP_PIN_OUTPUT,
			.init_state = GPIOF_OUT_INIT_LOW,
		},
		{
			.gpio_id = TAS5713_RESET_GPIO_ID,
			.gpio_name = "tas5713_reset",
			.mux_name = "gpmc_a18.gpio_42",
			.pin_mode = OMAP_PIN_OUTPUT,
			.init_state = GPIOF_OUT_INIT_LOW,
		},
		{
			.gpio_id = TAS5713_PDN_GPIO_ID,
			.gpio_name = "tas5713_pdn",
			.mux_name = "gpmc_a20.gpio_44",
			.pin_mode = OMAP_PIN_OUTPUT,
			.init_state = GPIOF_OUT_INIT_LOW,
		},
	};
	if (steelhead_reserve_gpios(tas5713_gpios, ARRAY_SIZE(tas5713_gpios),
				"tas5713"))
		return;

	/* Make sure that McBSP2's internal clock selection is set to the output
	 * of the ABE DPLL and not the PER DPLL.
	 */
	mcbsp_internal_clk = clk_get(NULL, "mcbsp2_sync_mux_ck");
	if (IS_ERR_OR_NULL(mcbsp_internal_clk)) {
		pr_err("tas5713: failed to fetch mcbsp2_sync_mux_ck\n");
		goto err;
	}

	abe_24m_clk = clk_get(NULL, "abe_24m_fclk");
	if (IS_ERR_OR_NULL(abe_24m_clk)) {
		pr_err("tas5713: failed to fetch abe_24m_clk\n");
		goto err;
	}

	res = clk_set_parent(mcbsp_internal_clk, abe_24m_clk);
	if (res < 0) {
		pr_err("tas5713: failed to set reference clock"
				" for McBSP2 internal clk (res = %d)  "
				"driver will not load.\n", res);
		goto err;
	}

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
		goto err;
	}

	mclk_out = clk_get(NULL, "auxclk1_ck");
	if (IS_ERR_OR_NULL(mclk_out)) {
		pr_err("tas5713: failed to auxclk1_ck\n");
		goto err;
	}

	res = clk_set_rate(m3x2_clk, tgt_rate);
	if (res < 0) {
		pr_err("tas5713: failed to set m3x2 clk"
			       "rate to %lu (res = %d).  "
			       "driver will not load.\n",
			       tgt_rate, res);
		goto err;
	}

	mclk_src = clk_get(NULL, "auxclk1_src_ck");
	if (IS_ERR_OR_NULL(mclk_out)) {
		pr_err("tas5713: failed to auxclk1_ck\n");
		goto err;
	}

	res = clk_set_parent(mclk_src, m3x2_clk);
	if (res < 0) {
		pr_err("tas5713: failed to set reference clock"
				" for fref_clk1_out (res = %d)  "
				"driver will not load.\n", res);
		goto err;
	}

	res = clk_set_rate(mclk_out, tas5713_mclk_rate);
	if (res < 0) {
		pr_err("tas5713: failed to set mclk_out"
			       "rate to %lu (res = %d).  "
			       "driver will not load.\n",
			       tas5713_mclk_rate, res);
		goto err;
	}

	/* Stash the clock and enable the pin. */
	tas5713_pdata.mclk_out = mclk_out;
	omap_mux_init_signal("fref_clk1_out", OMAP_PIN_OUTPUT);

err:
	if (!IS_ERR_OR_NULL(m3x2_clk))
		clk_put(m3x2_clk);
	if (!IS_ERR_OR_NULL(mclk_out))
		clk_put(mclk_out);
	if (!IS_ERR_OR_NULL(mclk_src))
		clk_put(mclk_src);
	if (!IS_ERR_OR_NULL(mcbsp_internal_clk))
		clk_put(mcbsp_internal_clk);
	if (!IS_ERR_OR_NULL(abe_24m_clk))
		clk_put(abe_24m_clk);
	/* TODO: release gpios, but we're fatal anyway. */
	return;
}

/******************************************************************************
 *                                                                            *
 *                     McASP S/PDIF Audio Initialization                      *
 *                                                                            *
 ******************************************************************************/

static struct omap_mcasp_platform_data steelhead_mcasp_pdata = {
	.get_raw_counter = steelhead_get_raw_counter,
};

static struct platform_device steelhead_mcasp_device = {
	.name		= "omap-mcasp-dai",
	.id		= 0,
	.dev	= {
		.platform_data	= &steelhead_mcasp_pdata,
	},
};

static struct platform_device steelhead_spdif_dit_device = {
	.name		= "spdif-dit",
	.id		= 0,
};

static void steelhead_platform_init_mcasp_audio(void)
{
	int res;

	res = omap_mux_init_signal("abe_mcbsp2_dr.abe_mcasp_axr",
				   OMAP_PIN_OUTPUT);
	if (res) {
		pr_err("omap-mcasp: failed to enable MCASP_AXR pin mux, S/PDIF"
				"will be unavailable. (res = %d)\n", res);
		return;
	}

	platform_device_register(&steelhead_spdif_dit_device);
	platform_device_register(&steelhead_mcasp_device);
	return;
};

/******************************************************************************
 *                                                                            *
 *           AVR Front Panel Controls and LED Initialization                  *
 *                                                                            *
 ******************************************************************************/

#define AVR_INT_GPIO_ID 49

static struct steelhead_avr_platform_data steelhead_avr_pdata = {
	/* Reset and Power Down GPIO configuration */
	.interrupt_gpio = AVR_INT_GPIO_ID,
};

static void steelhead_platform_init_avr(void)
{
	/* Grab a hold of the GPIO used to control the AVR interrupt
	 * request line.  Configure it to be an input and reserve it in
	 * the GPIO framwork.
	 */
	static struct steelhead_gpio_reservation avr_gpios[] = {
		{
			.gpio_id = AVR_INT_GPIO_ID,
			.gpio_name = "avr_int",
			.mux_name = "gpmc_a25.gpio_49",
			.pin_mode = OMAP_PIN_INPUT_PULLUP,
			.init_state = GPIOF_IN,
		},
	};

	if (steelhead_reserve_gpios(avr_gpios, ARRAY_SIZE(avr_gpios),
				"steelhead-avr"))
		return;

}

/******************************************************************************
 *                                                                            *
 *                            I2C Bus setup                                   *
 *                                                                            *
 ******************************************************************************/

static struct i2c_board_info __initdata steelhead_i2c_bus2[] = {
	{
		I2C_BOARD_INFO("steelhead-avr", (0x20)),
		.platform_data = &steelhead_avr_pdata,
	},
};

static struct i2c_board_info __initdata steelhead_i2c_bus4[] = {
	{
		I2C_BOARD_INFO("tas5713", (0x36 >> 1)),
		.platform_data = &tas5713_pdata,
	},
};

static int __init steelhead_i2c_init(void)
{
	omap_mux_init_signal("sys_nirq1", OMAP_PIN_INPUT);
	omap_mux_init_signal("i2c1_scl.i2c1_scl", OMAP_PIN_INPUT);
	omap_mux_init_signal("i2c1_sda.i2c1_sda", OMAP_PIN_INPUT);
	omap_mux_init_signal("i2c2_scl.i2c2_scl", OMAP_PIN_INPUT);
	omap_mux_init_signal("i2c2_sda.i2c2_sda", OMAP_PIN_INPUT);
	omap_mux_init_signal("i2c3_scl.i2c3_scl", OMAP_PIN_INPUT);
	omap_mux_init_signal("i2c3_sda.i2c3_sda", OMAP_PIN_INPUT);
	omap_mux_init_signal("i2c4_scl.i2c4_scl", OMAP_PIN_INPUT);
	omap_mux_init_signal("i2c4_sda.i2c4_sda", OMAP_PIN_INPUT);

	/*
	 * This will allow unused regulator to be shutdown. This flag
	 * should be set in the board file. Before regulators are registered.
	 */
	regulator_has_full_constraints();

	/* i2c1 - PMIC */
	omap4_pmic_init("twl6030", &steelhead_twldata);
	/* i2c2 - AVR */
	omap_register_i2c_bus(2, 400, steelhead_i2c_bus2,
			      ARRAY_SIZE(steelhead_i2c_bus2));
	/* i2c3 - NFC TBD */
	omap_register_i2c_bus(3, 400, NULL, 0);
	/* i2c4 - TAS5713 */
	omap_register_i2c_bus(4, 400, steelhead_i2c_bus4,
			      ARRAY_SIZE(steelhead_i2c_bus4));

	/*
	 * Drive MSECURE high for TWL6030 write access.
	 */
	omap_mux_init_signal("fref_clk0_out.gpio_wk6", OMAP_PIN_OUTPUT);
	gpio_request(6, "msecure");
	gpio_direction_output(6, 1);

	return 0;
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

static struct gpio panda_hdmi_gpios[] = {
	{ HDMI_GPIO_HPD,	GPIOF_OUT_INIT_HIGH, "hdmi_gpio_hpd"   },
	{ HDMI_GPIO_LS_OE,	GPIOF_OUT_INIT_HIGH, "hdmi_gpio_ls_oe" },
};

static int omap4_steelhead_panel_enable_hdmi(struct omap_dss_device *dssdev)
{
	int status;

	status = gpio_request_array(panda_hdmi_gpios,
				    ARRAY_SIZE(panda_hdmi_gpios));
	if (status)
		pr_err("Cannot request HDMI GPIOs\n");

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

#define PUBLIC_SAR_RAM_1_FREE_OFFSET 0xA0C

static int steelhead_reboot_notifier_handler(struct notifier_block *this,
					     unsigned long code, void *_cmd)
{
	void __iomem *sar_free_p = omap4_get_sar_ram_base();

	if (!sar_free_p)
		return notifier_from_errno(-ENOMEM);

	sar_free_p += PUBLIC_SAR_RAM_1_FREE_OFFSET;
	memset(sar_free_p, 0, 32);
	if (code == SYS_RESTART) {
		if (_cmd) {
			if (!strcmp(_cmd, "recovery"))
				strcpy(sar_free_p, "recovery");
			else if (!strcmp(_cmd, "bootloader"))
				strcpy(sar_free_p, "bootloader");
		}
	}

	return NOTIFY_DONE;
}

static struct notifier_block steelhead_reboot_notifier = {
	.notifier_call = steelhead_reboot_notifier_handler,
};

#if 0 /* TBD */
static void __init steelhead_nfc_init(void)
{
	gpio_request(GPIO_NFC_FIRMWARE, "nfc_firmware");
	gpio_direction_output(GPIO_NFC_FIRMWARE, 0);
	omap_mux_init_gpio(GPIO_NFC_FIRMWARE, OMAP_PIN_OUTPUT);

	gpio_request(GPIO_NFC_EN, "nfc_enable");
	gpio_direction_output(GPIO_NFC_EN, 1);
	omap_mux_init_gpio(GPIO_NFC_EN, OMAP_PIN_OUTPUT);

	gpio_request(GPIO_NFC_IRQ, "nfc_irq");
	gpio_direction_input(GPIO_NFC_IRQ);
	omap_mux_init_gpio(GPIO_NFC_IRQ, OMAP_PIN_INPUT_PULLUP);
}
#endif

static ssize_t steelhead_soc_family_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "OMAP%04x\n", GET_OMAP_TYPE);
}

static ssize_t steelhead_soc_revision_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "ES%d.%d\n", (GET_OMAP_REVISION() >> 4) & 0xf,
		       GET_OMAP_REVISION() & 0xf);
}

static ssize_t steelhead_soc_die_id_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	struct omap_die_id oid;
	omap_get_die_id(&oid);
	return sprintf(buf, "%08X-%08X-%08X-%08X\n", oid.id_3, oid.id_2,
			oid.id_1, oid.id_0);
}

static ssize_t steelhead_soc_prod_id_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	struct omap_die_id oid;
	omap_get_production_id(&oid);
	return sprintf(buf, "%08X-%08X\n", oid.id_1, oid.id_0);
}

static const char * const omap_types[] = {
	[OMAP2_DEVICE_TYPE_TEST]	= "TST",
	[OMAP2_DEVICE_TYPE_EMU]		= "EMU",
	[OMAP2_DEVICE_TYPE_SEC]		= "HS",
	[OMAP2_DEVICE_TYPE_GP]		= "GP",
	[OMAP2_DEVICE_TYPE_BAD]		= "BAD",
};

static ssize_t steelhead_soc_type_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", omap_types[omap_type()]);
}

#define STEELHEAD_ATTR_RO(_type, _name, _show) \
	struct kobj_attribute steelhead_##_type##_prop_attr_##_name = \
		__ATTR(_name, S_IRUGO, _show, NULL)

static STEELHEAD_ATTR_RO(soc, family, steelhead_soc_family_show);
static STEELHEAD_ATTR_RO(soc, revision, steelhead_soc_revision_show);
static STEELHEAD_ATTR_RO(soc, type, steelhead_soc_type_show);
static STEELHEAD_ATTR_RO(soc, die_id, steelhead_soc_die_id_show);
static STEELHEAD_ATTR_RO(soc, production_id, steelhead_soc_prod_id_show);

static struct attribute *steelhead_soc_prop_attrs[] = {
	&steelhead_soc_prop_attr_family.attr,
	&steelhead_soc_prop_attr_revision.attr,
	&steelhead_soc_prop_attr_type.attr,
	&steelhead_soc_prop_attr_die_id.attr,
	&steelhead_soc_prop_attr_production_id.attr,
	NULL,
};

static struct attribute_group steelhead_soc_prop_attr_group = {
	.attrs = steelhead_soc_prop_attrs,
};

static ssize_t steelhead_board_revision_show(struct kobject *kobj,
	 struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s (0x%02x)\n", omap4_steelhead_hw_rev_name(),
		steelhead_hw_rev);
}

static STEELHEAD_ATTR_RO(board, revision, steelhead_board_revision_show);
static struct attribute *steelhead_board_prop_attrs[] = {
	&steelhead_board_prop_attr_revision.attr,
	NULL,
};

static struct attribute_group steelhead_board_prop_attr_group = {
	.attrs = steelhead_board_prop_attrs,
};

static void __init omap4_steelhead_create_board_props(void)
{
	struct kobject *board_props_kobj;
	struct kobject *soc_kobj;
	int ret = 0;

	board_props_kobj = kobject_create_and_add("board_properties", NULL);
	if (!board_props_kobj)
		goto err_board_obj;

	soc_kobj = kobject_create_and_add("soc", board_props_kobj);
	if (!soc_kobj)
		goto err_soc_obj;

	ret = sysfs_create_group(board_props_kobj,
				 &steelhead_board_prop_attr_group);
	if (ret)
		goto err_board_sysfs_create;

	ret = sysfs_create_group(soc_kobj, &steelhead_soc_prop_attr_group);
	if (ret)
		goto err_soc_sysfs_create;

	return;

err_soc_sysfs_create:
	sysfs_remove_group(board_props_kobj, &steelhead_board_prop_attr_group);
err_board_sysfs_create:
	kobject_put(soc_kobj);
err_soc_obj:
	kobject_put(board_props_kobj);
err_board_obj:
	if (!board_props_kobj || !soc_kobj || ret)
		pr_err("failed to create board_properties\n");
}

static void __init steelhead_init(void)
{
	int package = OMAP_PACKAGE_CBS;

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;

	omap4_mux_init(NULL, NULL, package);

	omap4_steelhead_init_hw_rev();
	omap4_steelhead_emif_init();

	register_reboot_notifier(&steelhead_reboot_notifier);

	steelhead_platform_init_avr();
	steelhead_platform_init_tas5713_audio();
#if defined(CONFIG_SND_OMAP_SOC_MCASP)
	steelhead_platform_init_mcasp_audio();
#endif

	steelhead_i2c_init();
	platform_add_devices(steelhead_devices, ARRAY_SIZE(steelhead_devices));
	board_serial_init();
	omap4_twl6030_hsmmc_init(mmc);
	omap4_ehci_init();
	usb_musb_init(&musb_board_data);
	omap4_steelhead_create_board_props();
	omap4_steelhead_display_init();
	if (enable_sr)
		omap_enable_smartreflex_on_init();
	steelhead_init_wlan();
	steelhead_init_bluetooth();
}

static int __init steelhead_init_late(void)
{
	steelhead_platform_init_counter();
	return 0;
}

/*
 * This is needed because the platform counter is dependent on omap_dm_timers
 * which get initialized at device_init time
 */
late_initcall(steelhead_init_late);

static void __init steelhead_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}

static void __init steelhead_reserve(void)
{
	int i;
	int ret;

	/* do the static reservations first */
	memblock_remove(STEELHEAD_RAMCONSOLE_START, STEELHEAD_RAMCONSOLE_SIZE);

	memblock_remove(PHYS_ADDR_SMC_MEM, PHYS_ADDR_SMC_SIZE);
	memblock_remove(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE);

	for (i = 0; i < sh_ion_data.nr; i++) {
		if (sh_ion_data.heaps[i].type == ION_HEAP_TYPE_CARVEOUT ||
		    sh_ion_data.heaps[i].type == OMAP_ION_HEAP_TYPE_TILER) {
			ret = memblock_remove(sh_ion_data.heaps[i].base,
					      sh_ion_data.heaps[i].size);
			if (ret)
				pr_err("memblock remove of %x@%lx failed\n",
				       sh_ion_data.heaps[i].size,
				       sh_ion_data.heaps[i].base);
		}
	}
	/* ipu needs to recognize secure input buffer area as well */
	omap_ipu_set_static_mempool(PHYS_ADDR_DUCATI_MEM,
				    PHYS_ADDR_DUCATI_SIZE +
				    OMAP_STEELHEAD_ION_HEAP_SECURE_INPUT_SIZE);

	omap_reserve();
}

MACHINE_START(STEELHEAD, "Steelhead")
	/* Maintainer: Google, Inc */
	.boot_params	= 0x80000100,
	.reserve	= steelhead_reserve,
	.map_io		= steelhead_map_io,
	.init_early	= steelhead_init_early,
	.init_irq	= gic_init_irq,
	.init_machine	= steelhead_init,
	.timer		= &omap_timer,
MACHINE_END
