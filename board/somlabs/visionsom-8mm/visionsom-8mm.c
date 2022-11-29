// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 Somlabs
 */

#include <common.h>
#include <malloc.h>
#include <errno.h>
#include <asm/io.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#include <asm/arch/imx8mm_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/arch/clock.h>
#include <spl.h>
#include <asm/mach-imx/dma.h>
#include <power/pmic.h>
#include <fdt_support.h>

#include <asm/mach-imx/boot_mode.h>

#include <usb.h>

#include "hw_config.h"

DECLARE_GLOBAL_DATA_PTR;

int board_phys_sdram_size(phys_size_t *size)
{
    *size = visionsom8mm_get_dram_size();
    return 0;
}

#ifndef CONFIG_SPL_BUILD
int board_early_init_f(void)
{

	return 0;
}
#endif

#if IS_ENABLED(CONFIG_FEC_MXC)
static int setup_fec(void)
{
    struct iomuxc_gpr_base_regs *gpr =
        (struct iomuxc_gpr_base_regs *)IOMUXC_GPR_BASE_ADDR;

    /* Use 125M anatop REF_CLK1 for ENET1, not from external */
    clrsetbits_le32(&gpr->gpr[1],
            IOMUXC_GPR_GPR1_GPR_ENET1_TX_CLK_SEL_MASK, 0);
    return set_clk_enet(ENET_125MHZ);
}

int board_phy_config(struct phy_device *phydev)
{
    if (phydev->drv->config)
        phydev->drv->config(phydev);
    return 0;
}
#endif

int board_usb_init(int index, enum usb_init_type init)
{
	return imx8m_usb_power(index, true);
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	return imx8m_usb_power(index, false);
}

int board_init(void)
{

	if (IS_ENABLED(CONFIG_FEC_MXC)) {
		setup_fec();
    }

    return 0;
}

static int cbadv_is_hdmi_selected(void)
{
#define MIPI_HDMI_SELECT IMX_GPIO_NR(4, 20)
    iomux_v3_cfg_t const mipi_hdmi_sel_pad = IMX8MM_PAD_SAI1_MCLK_GPIO4_IO20 | MUX_PAD_CTRL(NO_PAD_CTRL);

    imx_iomux_v3_setup_pad(mipi_hdmi_sel_pad);

    gpio_request(MIPI_HDMI_SELECT, "mipi_hdmi");
    gpio_direction_input(MIPI_HDMI_SELECT);
    return !gpio_get_value(MIPI_HDMI_SELECT);
}

static int cb_is_lvds_enabled(bool is_adv_cb)
{
    uint32_t lvds_select;

    if (is_adv_cb) {
        /* there is external pull-up on ADV board */
        imx_iomux_v3_setup_pad(IMX8MM_PAD_I2C3_SDA_GPIO5_IO19 | MUX_PAD_CTRL(NO_PAD_CTRL));
        lvds_select = IMX_GPIO_NR(5, 19);
    } else {
        imx_iomux_v3_setup_pad(IMX8MM_PAD_SPDIF_EXT_CLK_GPIO5_IO5 | MUX_PAD_CTRL(PAD_CTL_PE | PAD_CTL_PUE));
        lvds_select = IMX_GPIO_NR(5, 5);
    }

    gpio_request(lvds_select, "lvds_sel");
    gpio_direction_input(lvds_select);
    return !gpio_get_value(lvds_select);
}

enum display_type {
    dt_none,
    dt_mipi7_powertip,
    dt_mipi7_riverdi,
    dt_mipi10,
    dt_lvds,
    dt_hdmi,
    dt_reserved
};

int board_late_init(void)
{
    enum display_type display = dt_none;
    struct udevice *bus;
    struct udevice *i2c_dev = NULL;
    int ret;

    bool adv_carrier_board = false;

#ifdef CONFIG_ENV_IS_IN_MMC
    board_late_mmc_env_init();
#endif

    ret = uclass_get_device_by_seq(UCLASS_I2C, 1, &bus);
    if (ret) {
        printf("%s: Can't find bus\n", __func__);
        return -EINVAL;
    }

    /*
     *  check if there is RTC @0x51 on I2C2 bus - this allows us to
     *	  distinguish between STD and ADV carrier board
     */
    ret = dm_i2c_probe(bus, 0x51, 0, &i2c_dev);
    if(ret == 0) {
        adv_carrier_board = true;
        env_set("cb_type", "adv");
    } else {
        env_set("cb_type", "std");
    }

    /*
     * We have 5 display options supported for 2 kinds of carrier boards:
     * - no display
     * - MIPI 7 inch  720x1280, vertical   (PH720128T003-ZBC02) touch @ 0x38
     * - MIPI 10 inch 1280x800, horizontal (PH128800T004-ZFC18) touch @ 0x01
     * - LVDS 10 inch 1280x800, horizontal (RK101II01D-CT092A) - lvds converter @ 0x48
     * - HDMI - resolution detected by kernel with EDID - hdmi converter @ 0x48
     *
     * For std-cb we just check presence of i2c device with specific address and read gpio
     *   to distinguish between LVDS and HDMI mode (selected by jumper)
     *
     * For std-adv board we have to read GPIO4-20:
     *  - low state means HDMI/LVDS, another GPIO selects between them
     *  - high state means MIPI/no display mode - exact display is dected by scanning i2c bus
     */
    if(adv_carrier_board && cbadv_is_hdmi_selected()) {
        display = dt_hdmi;
    } else if(!adv_carrier_board && (dm_i2c_probe(bus, 0x48, 0, &i2c_dev) == 0)) {
        display = dt_hdmi;
    }

    if(display == dt_hdmi) {
        if(cb_is_lvds_enabled(adv_carrier_board)) {
            display = dt_lvds;
        }
    } else if(dm_i2c_probe(bus, 0x38, 0, &i2c_dev) == 0) {
        display = dt_mipi7_powertip;
        env_set("extra_args", "fbcon=rotate:1");
    } else if(dm_i2c_probe(bus, 0x41, 0, &i2c_dev) == 0) {
        display = dt_mipi7_riverdi;
    } else if(dm_i2c_probe(bus, 0x01, 0, &i2c_dev) == 0) {
        display = dt_mipi10;
    } else {
        display = dt_none;
    }

    const char* disp_type[] = {"", "-mipi7-powertip",  "-mipi7-riverdi", "-mipi10", "-lvds", "-hdmi"};
    const char* displays[]  = {"NONE", "MIPI 7\" POWERTIP", "MIPI 7\" RIVERDI", "MIPI 10\"", "LVDS", "HDMI"};

    env_set("cb_disp", disp_type[display]);
    printf("Carrier board type: [%s], display: [%s]\n", (adv_carrier_board)?"ADV":"STD", displays[display]);

    return 0;
}

int mmc_map_to_kernel_blk(int dev_no)
{
        return dev_no;
}

static int set_fdt_cma_size(void *fdt, int size) {
    int off = fdt_subnode_offset(fdt, 0, "reserved-memory");
    if(off < 0)
        return -1;

    off = fdt_subnode_offset(fdt, off, "linux,cma");
    if(off < 0)
        return -1;

    const uint32_t cma_value[2] = {cpu_to_fdt32(0), cpu_to_fdt32(size)};
    int ret = fdt_setprop(fdt, off, "size", cma_value, sizeof(cma_value));
    if(ret)
        return -1;

    return 0;
}
/*
    This is called before OS start
*/
int ft_board_setup(void *fdt, struct bd_info *bd)
{
    int boot_dev = get_boot_device();
    /*
     *  In case of eMMC boot switch to 8-bit bus and allow 1.8V signaling
     *  device tree is expected to be configured by default for SD card, but pinmux should be already
     *    set for 8-bit bus
     */
    if(boot_dev == MMC3_BOOT) {
        puts("Updating device tree for eMMC\n");
        const char *usdhc3_path = fdt_get_alias(fdt, "mmc2");
        int off = fdt_path_offset(fdt, usdhc3_path);
        if (off < 0) {
            printf("ERROR: Cannot find offset for usdhc3 controller (%d)!\n", off);
            return -1;	//TODO: check for error code
        }
        fdt_setprop_u32(fdt, off, "bus-width", 8);
        fdt_delprop(fdt, off, "no-1-8-v");
    }

    /* disable wifi/bt nodes if wifi is not present */
    if(!visionsom8mm_get_wifi_status()) {
        puts("Disabling WLAN/BT device tree nodes...\n");
        const char *path = fdt_get_alias(fdt, "mmc1");
        int off = fdt_path_offset(fdt, path);
        if (off) {
            fdt_status_disabled(fdt, off);
        } else {
            printf("WARNING: Cannot find offset for mmc1 (%d)!\n", off);
        }

        path = fdt_get_alias(fdt, "serial0");
        off = fdt_path_offset(fdt, path);
        if (off) {
            fdt_status_disabled(fdt, off);
        } else {
            printf("WARNING: Cannot find offset for serial0 (%d)!\n", off);
        }
    }

    /* In modules with 512MB RAM the Linux CMA should be decreased to 256MB */
    if(visionsom8mm_get_dram_size() == (512 * SZ_1M)) {
        puts("Setting Linux CMA size to 256MB\n");
        if(set_fdt_cma_size(fdt, 256 * SZ_1M) != 0)
            printf("WARNING: Cannot set new CMA value!\n");
    }

    const char* rev = visionsom8mm_get_hw_rev_str();
    fdt_setprop(fdt, 0, "somlabs,board-rev", rev, strlen(rev) + 1);

    return 0;
}

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY
int is_recovery_key_pressing(void)
{
	return 0; /*TODO*/
}
#endif /*CONFIG_ANDROID_RECOVERY*/
#endif /*CONFIG_FSL_FASTBOOT*/

#ifdef CONFIG_ANDROID_SUPPORT
bool is_power_key_pressed(void) {
	return (bool)(!!(readl(SNVS_HPSR) & (0x1 << 6)));
}
#endif
