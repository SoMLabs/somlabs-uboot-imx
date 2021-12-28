// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2017-2019 A. Karas, SomLabs
 * Copyright (C) 2015-2016 Freescale Semiconductor, Inc.
 */

#include <init.h>
#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/global_data.h>
#include <asm/gpio.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/io.h>
#include <common.h>
#include <env.h>
#include <fsl_esdhc_imx.h>
#include <i2c.h>
#include <miiphy.h>
#include <linux/sizes.h>
#include <mmc.h>
#ifdef CONFIG_FEC_MXC
#include <netdev.h>
#endif
#include <splash.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)


#ifdef CONFIG_LDO_BYPASS_CHECK
void ldo_mode_set(int ldo_bypass)
{
	/* switch to ldo_bypass mode */
	if (ldo_bypass) {
		prep_anatop_bypass();
		set_anatop_bypass(1);
		finish_anatop_bypass();
		printf("switch to ldo_bypass mode!\n");
	}
}
#endif

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();

	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART1_RX_DATA__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

#ifdef CONFIG_FEC_MXC
static int setup_fec(void)
{
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int ret;

	/*
	 * Use 50M anatop loopback REF_CLK1 for ENET1,
	 * clear gpr1[13], set gpr1[17].
	 */
	clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC1_MASK,
			IOMUX_GPR1_FEC1_CLOCK_MUX1_SEL_MASK);

	ret = enable_fec_anatop_clock(0, ENET_50MHZ);
	if (ret)
		return ret;

	enable_enet_clk(1);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1f, 0x8190);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}
#endif

int board_mmc_get_env_dev(int devno)
{
	return devno;
}

int mmc_map_to_kernel_blk(int devno)
{
	return devno;
}

int board_early_init_f(void)
{
	setup_iomux_uart();

	return 0;
}

#ifdef CONFIG_SPLASH_SCREEN
static struct splash_location splash_locations[] = {
	{
		.name = "mmc",
		.storage = SPLASH_STORAGE_MMC,
		.flags = SPLASH_STORAGE_FS,
		.devpart = "1:1",
	},
	{
		.name = "nand",
		.storage = SPLASH_STORAGE_NAND,
		.flags = SPLASH_STORAGE_RAW,
		.offset = 0x300000,
	}
};

int splash_screen_prepare(void)
{
	int idx = (get_boot_device() == NAND_BOOT)?1:0;
	return splash_source_load(&splash_locations[idx], 1);
}
#endif

#ifdef CONFIG_DM_VIDEO

#define GPIO_LCD_RESET			IMX_GPIO_NR(3, 4)
#define GPIO_LCD_BRIGHTNESS		IMX_GPIO_NR(1, 15)

static int setup_lcd(void)
{
	enable_lcdif_clock(LCDIF1_BASE_ADDR, 1);

	/* Reset the LCD */
	gpio_request(GPIO_LCD_RESET, "lcd reset");
	gpio_direction_output(GPIO_LCD_RESET , 0);
	udelay(500);
	gpio_direction_output(GPIO_LCD_RESET , 1);

	/* Set Brightness to high */
	gpio_request(GPIO_LCD_BRIGHTNESS, "backlight");
	gpio_direction_output(GPIO_LCD_BRIGHTNESS , 1);

	return 0;
}

int board_cfb_skip(void)
{
	/* skip cfb init */
    return 1;
}

#endif

int board_init(void)
{
	/* Address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_SYS_I2C
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
#endif

#ifdef CONFIG_FEC_MXC
	setup_fec();
#endif

	return 0;
}

int board_late_init(void)
{
	const char* bootdev = NULL;
	const char* bootdevno = NULL;
	const char* fdt_suffix = NULL;

	if (is_cpu_type(MXC_CPU_MX6ULL))
		env_set("board", "visionsom-6ull");
	else
		env_set("board", "visionsom-6ul");

	switch (get_boot_device()) {
		case SD1_BOOT:
			bootdev = "sd";
			bootdevno = "0";
			break;
		case MMC1_BOOT:
			bootdev = "emmc";
			bootdevno = "0";
			fdt_suffix = "-emmc";
			break;
		case SD2_BOOT:
			bootdev = "sd";
			bootdevno = "1";
			break;
		case MMC2_BOOT:
			bootdev = "emmc";
			bootdevno = "1";
			fdt_suffix = "-emmc";
			break;
		case QSPI_BOOT:
			bootdev = "qspi";
			break;
		case NAND_BOOT:
			bootdev = "nand";
			fdt_suffix = "-nand";
			break;
		default:
			break;
	}
	if(bootdev) {
		env_set("bootdev", bootdev);
	}
	if(bootdevno) {
		env_set("bootdevno", bootdevno);
	}
	if(fdt_suffix) {
		env_set("fdt_suffix", fdt_suffix);
	}

	if(gd->ram_size < SZ_512M) {
		env_set("bootarg_cmasize", "cma=96M");
	}

#ifdef CONFIG_DM_VIDEO
	setup_lcd();
#endif

	set_wdog_reset((struct wdog_regs *)WDOG1_BASE_ADDR);

	return 0;
}

#ifdef CONFIG_DM_VIDEO
void board_preboot_os(void)
{
        gpio_set_value(GPIO_LCD_BRIGHTNESS, 0);
}
#endif

static int fdt_get_offset(void *fdt, const char *alias)
{
	const char *path = fdt_get_alias(fdt, alias);
	return fdt_path_offset(fdt, path);
}

static int fdt_enable_by_alias(void *fdt, const char *alias)
{
	int offset = fdt_get_offset(fdt, alias);
	return fdt_status_okay(fdt, offset);
}

/*
	This is called before relocation
	We need to detect boot mode and configure NAND/eMMC/SD variant here
	Nodes for not existing memories needs to be deleted from device tree.
	This allows us to have single bootloader build independant of boot mode
*/
int board_fix_fdt(void *fdt)
{
	int boot_dev = get_boot_device();

	if(boot_dev == NAND_BOOT) {
		// enable NAND interface
		fdt_enable_by_alias(fdt, "gpmi");
	} else {
		// enable USDHC2
		fdt_enable_by_alias(fdt, "mmc1");
	}

	return 0;
}
