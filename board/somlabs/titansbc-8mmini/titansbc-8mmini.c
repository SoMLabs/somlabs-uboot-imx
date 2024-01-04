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

#include <imx_sip.h>
#include <linux/arm-smccc.h>

#include "hw_config.h"

DECLARE_GLOBAL_DATA_PTR;

int board_phys_sdram_size(phys_size_t *size)
{
    *size = titansbc8mmini_get_dram_size();
    return 0;
}

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

#define DISPMIX				9
#define MIPI				10

int board_init(void)
{
    struct arm_smccc_res res;

    if (IS_ENABLED(CONFIG_FEC_MXC)) {
        setup_fec();
    }

    arm_smccc_smc(IMX_SIP_GPC, IMX_SIP_GPC_PM_DOMAIN, DISPMIX, true, 0, 0, 0, 0, &res);
    arm_smccc_smc(IMX_SIP_GPC, IMX_SIP_GPC_PM_DOMAIN, MIPI, true, 0, 0, 0, 0, &res);

    return 0;
}

#ifdef CONFIG_ENV_IS_IN_MMC
static int check_mmc_autodetect(void)
{
   char *autodetect_str = env_get("mmcautodetect");

   if ((autodetect_str != NULL) &&
       (strcmp(autodetect_str, "yes") == 0)) {
       return 1;
   }

   return 0;
}

void board_late_mmc_env_init(void)
{
    char cmd[32];
    char mmcblk[32];
    u32 dev_no = mmc_get_env_dev();

    if (!check_mmc_autodetect())
        return;

    env_set_ulong("mmcdev", dev_no);

    /* Set mmcblk env */
    sprintf(mmcblk, "/dev/mmcblk%dp2 rootwait rw",
            mmc_map_to_kernel_blk(dev_no));
    env_set("mmcroot", mmcblk);

    sprintf(cmd, "mmc dev %d", dev_no);
    run_command(cmd, 0);
}
#endif

int board_late_init(void)
{
#ifdef CONFIG_ENV_IS_IN_MMC
    board_late_mmc_env_init();
#endif

    return 0;
}

int mmc_map_to_kernel_blk(int dev_no)
{
        return dev_no;
}

/*
    This is called before OS start
*/
int ft_board_setup(void *fdt, struct bd_info *bd)
{

    return 0;
}
