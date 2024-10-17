/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2024 Somlabs
 */

#ifndef __VISIONSOM_IMX93_H
#define __VISIONSOM_IMX93_H

#include <linux/sizes.h>
#include <linux/stringify.h>
#include <asm/arch/imx-regs.h>
#include "imx_env.h"

#define CFG_SYS_UBOOT_BASE	\
	(QSPI0_AMBA_BASE + CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR * 512)

#ifdef CONFIG_DISTRO_DEFAULTS
#define BOOT_TARGET_DEVICES(func) \
	func(MMC, mmc, 0) \

#include <config_distro_bootcmd.h>
#else
#define BOOTENV
#endif

#define CFG_SYS_INIT_RAM_ADDR        0x80000000
#define CFG_SYS_INIT_RAM_SIZE        0x200000

#define CFG_MMCROOT                  "/dev/mmcblk0p2"

#define CFG_SYS_SDRAM_BASE           0x80000000
#define PHYS_SDRAM                   0x80000000
#define PHYS_SDRAM_SIZE	             0x80000000 /* 2GB DDR */

/* Using ULP WDOG for reset */
#define WDOG_BASE_ADDR          WDG3_BASE_ADDR

#if defined(CONFIG_CMD_NET)
#define PHY_ANEG_TIMEOUT 20000
#endif

#endif	/* __VISIONSOM_IMX93_H */
