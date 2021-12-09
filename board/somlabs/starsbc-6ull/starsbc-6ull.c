// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021 A. Karas, SomLabs
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
#include <fdt_support.h>
#include <linux/delay.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |               \
                       PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | \
                       PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST | PAD_CTL_HYS)

#ifdef CONFIG_LDO_BYPASS_CHECK
void ldo_mode_set(int ldo_bypass)
{
    /* switch to ldo_bypass mode */
    if (ldo_bypass)
    {
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

static iomux_v3_cfg_t const uart4_pads[] = {
    MX6_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_UART1_RX_DATA__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
    imx_iomux_v3_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));
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

int board_early_init_f(void)
{
    setup_iomux_uart();

    return 0;
}

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

#define GPIO_BT_ENABLE IMX_GPIO_NR(4, 17)
#define GPIO_BT_HOST_WAKE IMX_GPIO_NR(4, 19)

int board_with_wifi(void)
{
    int ret = 0;

    gpio_request(GPIO_BT_ENABLE, "bt_enable");
    gpio_request(GPIO_BT_HOST_WAKE, "bt_hwake");
    gpio_direction_input(GPIO_BT_HOST_WAKE);
    gpio_direction_output(GPIO_BT_ENABLE, 1);
    udelay(5000); // wait 5ms until signal is stable
    /* wlan/bt chip drives this pin low when BT PWR is enabled! */
    if (gpio_get_value(GPIO_BT_HOST_WAKE) == 0)
    {
        ret = 1;
    }
    gpio_direction_output(GPIO_BT_ENABLE, 0);
    gpio_free(GPIO_BT_ENABLE);
    gpio_free(GPIO_BT_HOST_WAKE);
    return ret;
}

#define GPIO_COMM_SHIELD_DETECT IMX_GPIO_NR(3, 17)
int detect_comm_shield(void)
{
   int ret = 0;

    gpio_request(GPIO_COMM_SHIELD_DETECT, "bt_enable");
    gpio_direction_input(GPIO_COMM_SHIELD_DETECT);
    /* COMM shield has 1k pull down resistor on GPIO 3.17 */
    if (gpio_get_value(GPIO_COMM_SHIELD_DETECT) == 0)
    {
        ret = 1;
    }
    gpio_free(GPIO_COMM_SHIELD_DETECT);
    return ret;
}


int board_late_init(void)
{
    if(detect_comm_shield())
    {
        env_set("fdt_suffix", "-shield-comm");
    }

    set_wdog_reset((struct wdog_regs *)WDOG1_BASE_ADDR);

    return 0;
}

static int set_fdt_cma_size(void *fdt, int size)
{
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
    if (boot_dev == MMC1_BOOT)
    {
        puts("Updating device tree for eMMC\n");
        const char *usdhc_path = fdt_get_alias(fdt, "mmc1");
        int off = fdt_path_offset(fdt, usdhc_path);
        if (off < 0)
        {
            printf("ERROR: Cannot find offset for usdhc controller (%d)!\n", off);
            return -1; // TODO: check for error code
        }
        fdt_setprop_u32(fdt, off, "bus-width", 8);
        fdt_delprop(fdt, off, "no-1-8-v");
        fdt_setprop(fdt, off, "non-removable", NULL, 0);
    }

    /* disable wifi/bt nodes if wifi is not present */
    if (!board_with_wifi())
    {
        puts("Disabling WLAN/BT device tree nodes...\n");
        const char *path = fdt_get_alias(fdt, "mmc0");
        int off = fdt_path_offset(fdt, path);
        if (off)
        {
            fdt_status_disabled(fdt, off);
        }
        else
        {
            printf("WARNING: Cannot find offset for mmc1 (%d)!\n", off);
        }

        path = fdt_get_alias(fdt, "serial4");
        off = fdt_path_offset(fdt, path);
        if (off)
        {
            fdt_status_disabled(fdt, off);
        }
        else
        {
            printf("WARNING: Cannot find offset for serial0 (%d)!\n", off);
        }
    }

    /* In modules with less than 512MB RAM the Linux CMA should be decreased to 96MB */
    if (gd->ram_size < SZ_512M)
    {
        puts("Setting Linux CMA size to 96MB\n");
        if (set_fdt_cma_size(fdt, 96 * SZ_1M) != 0)
            printf("WARNING: Cannot set new CMA value!\n");
    }

    return 0;
}
