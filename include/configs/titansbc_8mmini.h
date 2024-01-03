/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2020 Somlabs
 */

#ifndef __TITANSBC_8MMINI_H
#define __TITANSBC_8MMINI_H

#include <linux/sizes.h>
#include <asm/arch/imx-regs.h>

#define CFG_SYS_UBOOT_BASE	\
	(QSPI0_AMBA_BASE + CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR * 512)

#ifdef CONFIG_SPL_BUILD

/* malloc f used before GD_FLG_FULL_MALLOC_INIT set */
#define CFG_MALLOC_F_ADDR		0x930000

#endif	/* CONFIG_SPL_BUILD */

/* ENET Config */
/* ENET1 */
#if defined(CONFIG_FEC_MXC)
#define PHY_ANEG_TIMEOUT		20000
#define CFG_FEC_MXC_PHYADDR		0
#define IMX_FEC_BASE			0x30BE0000
#endif	/* CONFIG_FEC_MXC */

#define CONFIG_BOOTCOMMAND \
	   "mmc dev ${mmcdev}; if mmc rescan; then " \
		   "if run loadbootscript; then " \
			   "run bootscript; " \
		   "else " \
			   "if run loadimage; then " \
				   "run mmcboot; " \
			   "else echo \"ERROR: Cannot load boot image!\"; " \
			   "fi; " \
		   "fi; " \
	   "else booti ${loadaddr} - ${fdt_addr}; fi"

/* Link Definitions */
#define CFG_SYS_INIT_RAM_ADDR        0x40000000
#define CFG_SYS_INIT_RAM_SIZE        0x200000
#define CFG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CFG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#define CFG_MMCROOT			"/dev/mmcblk2p2"  /* USDHC3 */

#define CFG_SYS_SDRAM_BASE           0x40000000
#define PHYS_SDRAM                      0x40000000
#define PHYS_SDRAM_SIZE			0x80000000 /* 2GB DDR */

#define CFG_MXC_UART_BASE		UART4_BASE_ADDR

/* USDHC */
#define CFG_SYS_FSL_USDHC_NUM		2
#define CFG_SYS_FSL_ESDHC_ADDR		0

#define CFG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)

#if defined(CONFIG_ANDROID_SUPPORT)
#include "imx8mm_evk_android.h"
#endif

#endif /*__TITANSBC_8MMINI_H */
