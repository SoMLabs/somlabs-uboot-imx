// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018-2019 NXP
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

#include <asm/mach-imx/boot_mode.h>

#include <usb.h>
#include <sec_mipi_dsim.h>
#include <imx_mipi_dsi_bridge.h>
#include <mipi_dsi_panel.h>
#include <asm/mach-imx/video.h>

#include "hw_config.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)

static iomux_v3_cfg_t const uart_pads[] = {
    IMX8MM_PAD_UART4_RXD_UART4_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
    IMX8MM_PAD_UART4_TXD_UART4_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
    };

static iomux_v3_cfg_t const wdog_pads[] = {
    IMX8MM_PAD_GPIO1_IO02_WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

int board_early_init_f(void)
{
    struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

    imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

    set_wdog_reset(wdog);

    imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));

    init_uart_clk(3);

    return 0;
}

int dram_init(void)
{
    gd->ram_size = visionsom8mm_get_dram_size() - (rom_pointer[1]?rom_pointer[1]:0);
    return 0;
}

int dram_init_banksize(void)
{
    phys_size_t size_bank1 = visionsom8mm_get_dram_size();

#if CONFIG_NR_DRAM_BANKS == 2
    phys_size_t size_bank2 = 0;
    if (size_bank1 > (SZ_1G + SZ_2G)) {
        size_bank2 = size_bank1 - SZ_1G + SZ_2G;
        size_bank1 = SZ_1G + SZ_2G; // max 3GB in bank 1
    }
#else
    if (size_bank1 > (SZ_1G + SZ_2G)) {
        printf("WARNING: To support memory >3GB CONFIG_NR_DRAM_BANKS==2 must be set!!!");
    }
#endif
    gd->bd->bi_dram[0].start = PHYS_SDRAM;
    gd->bd->bi_dram[0].size = size_bank1 - (rom_pointer[1]?rom_pointer[1]:0);

#if CONFIG_NR_DRAM_BANKS == 2
    gd->bd->bi_dram[1].start = PHYS_SDRAM_2;
    gd->bd->bi_dram[1].size = size_bank2;
#endif

    return 0;
}

phys_size_t get_effective_memsize(void)
{
    return visionsom8mm_get_dram_size() - (rom_pointer[1]?rom_pointer[1]:0);
}

#ifdef CONFIG_FEC_MXC
#define FEC_RST_PAD IMX_GPIO_NR(1, 0)
static iomux_v3_cfg_t const fec1_rst_pads[] = {
    IMX8MM_PAD_GPIO1_IO00_GPIO1_IO0 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_fec(void)
{
    imx_iomux_v3_setup_multiple_pads(fec1_rst_pads, ARRAY_SIZE(fec1_rst_pads));

    gpio_request(FEC_RST_PAD, "fec1_rst");
    gpio_direction_output(FEC_RST_PAD, 0);
    udelay(500);
    gpio_direction_output(FEC_RST_PAD, 1);
}

static int setup_fec(void)
{
    struct iomuxc_gpr_base_regs *gpr =
        (struct iomuxc_gpr_base_regs *)IOMUXC_GPR_BASE_ADDR;

    setup_iomux_fec();

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

static iomux_v3_cfg_t const usb_vbus_pads[] = {
    IMX8MM_PAD_GPIO1_IO12_GPIO1_IO12 | MUX_PAD_CTRL(UART_PAD_CTRL),
    IMX8MM_PAD_GPIO1_IO14_GPIO1_IO14 | MUX_PAD_CTRL(UART_PAD_CTRL),
};

int board_usb_init(int index, enum usb_init_type init)
{
    int ret = 0;

    debug("board_usb_init %d, type %d\n", index, init);

    imx8m_usb_power(index, true);

    imx_iomux_v3_setup_multiple_pads(usb_vbus_pads, ARRAY_SIZE(usb_vbus_pads));
    if(index == 0) {
        gpio_request(IMX_GPIO_NR(1, 12), "OTG1 VBUS");
        gpio_direction_output(IMX_GPIO_NR(1, 12), 1);
    } else {
        gpio_request(IMX_GPIO_NR(1, 14), "OTG1 VBUS");
        gpio_direction_output(IMX_GPIO_NR(1, 14), 1);

    }


    return ret;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
    int ret = 0;

    debug("board_usb_cleanup %d, type %d\n", index, init);

    imx8m_usb_power(index, false);
    return ret;
}

int board_init(void)
{
#ifdef CONFIG_FEC_MXC
    setup_fec();
#endif

    return 0;
}

#ifdef CONFIG_VIDEO_MXS

#define ADV7535_MAIN 0x3d
#define ADV7535_DSI_CEC 0x3c

static const struct sec_mipi_dsim_plat_data imx8mm_mipi_dsim_plat_data = {
    .version	= 0x1060200,
    .max_data_lanes = 4,
    .max_data_rate  = 1500000000ULL,
    .reg_base = MIPI_DSI_BASE_ADDR,
    .gpr_base = CSI_BASE_ADDR + 0x8000,
};

static int adv7535_i2c_reg_write(struct udevice *dev, uint addr, uint mask, uint data)
{
    uint8_t valb;
    int err;

    if (mask != 0xff) {
        err = dm_i2c_read(dev, addr, &valb, 1);
        if (err)
            return err;

        valb &= ~mask;
        valb |= data;
    } else {
        valb = data;
    }

    err = dm_i2c_write(dev, addr, &valb, 1);
    return err;
}

static int adv7535_i2c_reg_read(struct udevice *dev, uint8_t addr, uint8_t *data)
{
    uint8_t valb;
    int err;

    err = dm_i2c_read(dev, addr, &valb, 1);
    if (err)
        return err;

    *data = (int)valb;
    return 0;
}

static void adv7535_init(void)
{
    struct udevice *bus, *main_dev, *cec_dev;
    int i2c_bus = 1;
    int ret;
    uint8_t val;

    ret = uclass_get_device_by_seq(UCLASS_I2C, i2c_bus, &bus);
    if (ret) {
        printf("%s: No bus %d\n", __func__, i2c_bus);
        return;
    }

    ret = dm_i2c_probe(bus, ADV7535_MAIN, 0, &main_dev);
    if (ret) {
        printf("%s: Can't find device id=0x%x, on bus %d\n",
            __func__, ADV7535_MAIN, i2c_bus);
        return;
    }

    ret = dm_i2c_probe(bus, ADV7535_DSI_CEC, 0, &cec_dev);
    if (ret) {
        printf("%s: Can't find device id=0x%x, on bus %d\n",
            __func__, ADV7535_MAIN, i2c_bus);
        return;
    }

    adv7535_i2c_reg_read(main_dev, 0x00, &val);
    debug("Chip revision: 0x%x (expected: 0x14)\n", val);
    adv7535_i2c_reg_read(cec_dev, 0x00, &val);
    debug("Chip ID MSB: 0x%x (expected: 0x75)\n", val);
    adv7535_i2c_reg_read(cec_dev, 0x01, &val);
    debug("Chip ID LSB: 0x%x (expected: 0x33)\n", val);

    /* Power */
    adv7535_i2c_reg_write(main_dev, 0x41, 0xff, 0x10);
    /* Initialisation (Fixed) Registers */
    adv7535_i2c_reg_write(main_dev, 0x16, 0xff, 0x20);
    adv7535_i2c_reg_write(main_dev, 0x9A, 0xff, 0xE0);
    adv7535_i2c_reg_write(main_dev, 0xBA, 0xff, 0x70);
    adv7535_i2c_reg_write(main_dev, 0xDE, 0xff, 0x82);
    adv7535_i2c_reg_write(main_dev, 0xE4, 0xff, 0x40);
    adv7535_i2c_reg_write(main_dev, 0xE5, 0xff, 0x80);
    adv7535_i2c_reg_write(cec_dev, 0x15, 0xff, 0xD0);
    adv7535_i2c_reg_write(cec_dev, 0x17, 0xff, 0xD0);
    adv7535_i2c_reg_write(cec_dev, 0x24, 0xff, 0x20);
    adv7535_i2c_reg_write(cec_dev, 0x57, 0xff, 0x11);
    /* 4 x DSI Lanes */
    adv7535_i2c_reg_write(cec_dev, 0x1C, 0xff, 0x40);

    /* DSI Pixel Clock Divider */
    adv7535_i2c_reg_write(cec_dev, 0x16, 0xff, 0x18);

    /* Enable Internal Timing Generator */
    adv7535_i2c_reg_write(cec_dev, 0x27, 0xff, 0xCB);
    /* 1920 x 1080p 60Hz */
    adv7535_i2c_reg_write(cec_dev, 0x28, 0xff, 0x89); /* total width */
    adv7535_i2c_reg_write(cec_dev, 0x29, 0xff, 0x80); /* total width */
    adv7535_i2c_reg_write(cec_dev, 0x2A, 0xff, 0x02); /* hsync */
    adv7535_i2c_reg_write(cec_dev, 0x2B, 0xff, 0xC0); /* hsync */
    adv7535_i2c_reg_write(cec_dev, 0x2C, 0xff, 0x05); /* hfp */
    adv7535_i2c_reg_write(cec_dev, 0x2D, 0xff, 0x80); /* hfp */
    adv7535_i2c_reg_write(cec_dev, 0x2E, 0xff, 0x09); /* hbp */
    adv7535_i2c_reg_write(cec_dev, 0x2F, 0xff, 0x40); /* hbp */

    adv7535_i2c_reg_write(cec_dev, 0x30, 0xff, 0x46); /* total height */
    adv7535_i2c_reg_write(cec_dev, 0x31, 0xff, 0x50); /* total height */
    adv7535_i2c_reg_write(cec_dev, 0x32, 0xff, 0x00); /* vsync */
    adv7535_i2c_reg_write(cec_dev, 0x33, 0xff, 0x50); /* vsync */
    adv7535_i2c_reg_write(cec_dev, 0x34, 0xff, 0x00); /* vfp */
    adv7535_i2c_reg_write(cec_dev, 0x35, 0xff, 0x40); /* vfp */
    adv7535_i2c_reg_write(cec_dev, 0x36, 0xff, 0x02); /* vbp */
    adv7535_i2c_reg_write(cec_dev, 0x37, 0xff, 0x40); /* vbp */

    /* Reset Internal Timing Generator */
    adv7535_i2c_reg_write(cec_dev, 0x27, 0xff, 0xCB);
    adv7535_i2c_reg_write(cec_dev, 0x27, 0xff, 0x8B);
    adv7535_i2c_reg_write(cec_dev, 0x27, 0xff, 0xCB);

    /* HDMI Output */
    adv7535_i2c_reg_write(main_dev, 0xAF, 0xff, 0x16);
    /* AVI Infoframe - RGB - 16-9 Aspect Ratio */
    adv7535_i2c_reg_write(main_dev, 0x55, 0xff, 0x02);
    adv7535_i2c_reg_write(main_dev, 0x56, 0xff, 0x0);

    /*  GC Packet Enable */
    adv7535_i2c_reg_write(main_dev, 0x40, 0xff, 0x0);
    /*  GC Colour Depth - 24 Bit */
    adv7535_i2c_reg_write(main_dev, 0x4C, 0xff, 0x0);
    /*  Down Dither Output Colour Depth - 8 Bit (default) */
    adv7535_i2c_reg_write(main_dev, 0x49, 0xff, 0x00);

    /* set low refresh 1080p30 */
    adv7535_i2c_reg_write(main_dev, 0x4A, 0xff, 0x80); /*should be 0x80 for 1080p60 and 0x8c for 1080p30*/

    /* HDMI Output Enable */
    adv7535_i2c_reg_write(cec_dev, 0xbe, 0xff, 0x3c);
    adv7535_i2c_reg_write(cec_dev, 0x03, 0xff, 0x89);
}

#define DISPLAY_MIX_SFT_RSTN_CSR		0x00
#define DISPLAY_MIX_CLK_EN_CSR		0x04

   /* 'DISP_MIX_SFT_RSTN_CSR' bit fields */
#define BUS_RSTN_BLK_SYNC_SFT_EN	BIT(6)

   /* 'DISP_MIX_CLK_EN_CSR' bit fields */
#define LCDIF_PIXEL_CLK_SFT_EN		BIT(7)
#define LCDIF_APB_CLK_SFT_EN		BIT(6)

void disp_mix_bus_rstn_reset(ulong gpr_base, bool reset)
{
    if (!reset)
        /* release reset */
        setbits_le32(gpr_base + DISPLAY_MIX_SFT_RSTN_CSR, BUS_RSTN_BLK_SYNC_SFT_EN);
    else
        /* hold reset */
        clrbits_le32(gpr_base + DISPLAY_MIX_SFT_RSTN_CSR, BUS_RSTN_BLK_SYNC_SFT_EN);
}

void disp_mix_lcdif_clks_enable(ulong gpr_base, bool enable)
{
    if (enable)
        /* enable lcdif clks */
        setbits_le32(gpr_base + DISPLAY_MIX_CLK_EN_CSR, LCDIF_PIXEL_CLK_SFT_EN | LCDIF_APB_CLK_SFT_EN);
    else
        /* disable lcdif clks */
        clrbits_le32(gpr_base + DISPLAY_MIX_CLK_EN_CSR, LCDIF_PIXEL_CLK_SFT_EN | LCDIF_APB_CLK_SFT_EN);
}

struct mipi_dsi_client_dev adv7535_dev = {
    .channel	= 0,
    .lanes = 4,
    .format  = MIPI_DSI_FMT_RGB888,
    .mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
              MIPI_DSI_MODE_EOT_PACKET | MIPI_DSI_MODE_VIDEO_HSE,
    .name = "ADV7535",
};

struct mipi_dsi_client_dev powertip_ph720128t003_dev = {
    .channel	= 0,
    .lanes = 2,
    .format  = MIPI_DSI_FMT_RGB888,
    .mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE | MIPI_DSI_MODE_LPM,
};

#define FSL_SIP_GPC			0xC2000000
#define FSL_SIP_CONFIG_GPC_PM_DOMAIN	0x3
#define DISPMIX				9
#define MIPI				10

static void do_enable_mipi2hdmi(struct display_info_t const *dev)
{
    gpio_request(IMX_GPIO_NR(4, 28), "DSI RESET");
    gpio_direction_output(IMX_GPIO_NR(4, 28), 1);

    /* ADV7353 initialization */
    adv7535_init();

    /* enable the dispmix & mipi phy power domain */
    call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, DISPMIX, true, 0);
    call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, MIPI, true, 0);

    /* Put lcdif out of reset */
    disp_mix_bus_rstn_reset(imx8mm_mipi_dsim_plat_data.gpr_base, false);
    disp_mix_lcdif_clks_enable(imx8mm_mipi_dsim_plat_data.gpr_base, true);

    /* Setup mipi dsim */
    sec_mipi_dsim_setup(&imx8mm_mipi_dsim_plat_data);
    imx_mipi_dsi_bridge_attach(&adv7535_dev); /* attach adv7535 device */
}

static void do_enable_mipi_powertip(struct display_info_t const *dev)
{
    gpio_request(IMX_GPIO_NR(4, 28), "DSI RESET");
    gpio_direction_output(IMX_GPIO_NR(4, 28), 0);
    mdelay(100);
    gpio_direction_output(IMX_GPIO_NR(4, 28), 1);

    /* enable the dispmix & mipi phy power domain */
    call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, DISPMIX, true, 0);
    call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, MIPI, true, 0);

    /* Put lcdif out of reset */
    disp_mix_bus_rstn_reset(imx8mm_mipi_dsim_plat_data.gpr_base, false);
    disp_mix_lcdif_clks_enable(imx8mm_mipi_dsim_plat_data.gpr_base, true);

    /* Setup mipi dsim */
    sec_mipi_dsim_setup(&imx8mm_mipi_dsim_plat_data);

    powertip_ph720128t003_dev.name = displays[1].mode.name;
    imx_mipi_dsi_bridge_attach(&powertip_ph720128t003_dev); /* attach rm67191 device */
}

void board_quiesce_devices(void)
{
    gpio_request(IMX_GPIO_NR(4, 28), "DSI RESET");
    gpio_direction_output(IMX_GPIO_NR(4, 28), 0);
}

struct display_info_t const displays[] = {{
    .bus = LCDIF_BASE_ADDR,
    .addr = 0,
    .pixfmt = 24,
    .detect = NULL,
    .enable	= do_enable_mipi2hdmi,
    .mode	= {
        .name			= "MIPI2HDMI",
        .refresh		= 60,
        .xres			= 1920,
        .yres			= 1080,
        .pixclock		= 6734, /* 148500000 */
        .left_margin	= 148,
        .right_margin	= 88,
        .upper_margin	= 36,
        .lower_margin	= 4,
        .hsync_len		= 44,
        .vsync_len		= 5,
        .sync			= FB_SYNC_EXT,
        .vmode			= FB_VMODE_NONINTERLACED

} }, {
    .bus = LCDIF_BASE_ADDR,
    .addr = 0,
    .pixfmt = 24,
    .detect = NULL,
    .enable	= do_enable_mipi_powertip,
    .mode	= {
        .name			= "POWERTIP_PH720128T003",
        .xres			= 720,
        .yres			= 1280,
        .pixclock		= 18518, /* 132000000 */
        .left_margin	= 20,
        .right_margin	= 20,
        .upper_margin	= 15,
        .lower_margin	= 10,
        .hsync_len		= 60,
        .vsync_len		= 2,
        .sync			= FB_SYNC_EXT,
        .vmode			= FB_VMODE_NONINTERLACED
} } };
size_t display_count = ARRAY_SIZE(displays);
#endif

static int cbadv_get_mipi_hdmi_selection(void)
{
#define MIPI_HDMI_SELECT IMX_GPIO_NR(4, 20)
    iomux_v3_cfg_t const mipi_hdmi_sel_pad = IMX8MM_PAD_SAI1_MCLK_GPIO4_IO20 | MUX_PAD_CTRL(NO_PAD_CTRL);

    imx_iomux_v3_setup_pad(mipi_hdmi_sel_pad);

    gpio_request(MIPI_HDMI_SELECT, "mipi_hdmi");
    gpio_direction_input(MIPI_HDMI_SELECT);
    return gpio_get_value(MIPI_HDMI_SELECT);
}

#define CB_STD		0x00
#define CB_ADV		0x01
#define CB_MIPI		0x00
#define CB_HDMI		0x02

enum cb_board_type
{
    cb_std_mipi = CB_STD | CB_MIPI,
    cb_std_hdmi = CB_STD | CB_HDMI,
    cb_adv_mipi = CB_ADV | CB_MIPI,
    cb_adv_hdmi = CB_ADV | CB_HDMI,
};

int board_late_init(void)
{
    struct udevice *bus;
    struct udevice *i2c_dev = NULL;
    int ret;

    enum cb_board_type carrier_board = cb_std_mipi;

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
        carrier_board |= CB_ADV;
    }

    /*
     * now check if there is MIPI/HDMI converter attached
     * to choose between MIPI display and HDMI output
     * We do this by checking if MIPI/HDMI bridge is present @0x48 on I2C2 bus
     *  for STD board
     * For ADV board we need to check jumper state at GPIO4-20 - low state means HDMI adapter,
     *  high means MIPI display is selected
     */
    if(carrier_board & CB_ADV ) {
        carrier_board |= cbadv_get_mipi_hdmi_selection()?CB_MIPI:CB_HDMI;
    } else {
        carrier_board |= (dm_i2c_probe(bus, 0x48, 0, &i2c_dev) == 0)?CB_HDMI:CB_MIPI;
    }

    switch(carrier_board) {
    case cb_std_mipi:
        env_set("fdt_file", "visionsom-8mm-cb-std.dtb");
        break;
    case cb_std_hdmi:
        env_set("fdt_file", "visionsom-8mm-cb-std-hdmi.dtb");
        break;
    case cb_adv_mipi:
        env_set("fdt_file", "visionsom-8mm-cb-adv.dtb");
        break;
    case cb_adv_hdmi:
        env_set("fdt_file", "visionsom-8mm-cb-adv-hdmi.dtb");
        break;
    };

    printf("Carrier board type: [%s] with [%s] display\n", (carrier_board & CB_ADV)?"ADV":"STD",
            (carrier_board & CB_HDMI)?"HDMI":"MIPI");

    return 0;
}

int mmc_map_to_kernel_blk(int dev_no)
{
        return dev_no;
}

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY
int is_recovery_key_pressing(void)
{
    return 0; /*TODO*/
}
#endif /*CONFIG_ANDROID_RECOVERY*/
#endif /*CONFIG_FSL_FASTBOOT*/

/*
    This is called before OS start
*/
int ft_board_setup(void *fdt, bd_t *bd)
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

    return 0;
}
