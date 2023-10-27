/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2019 NXP
 */

#ifndef __IMX8MP_EVK_H
#define __IMX8MP_EVK_H

#include <linux/sizes.h>
#include <linux/stringify.h>
#include <asm/arch/imx-regs.h>
#include "imx_env.h"

#define CFG_SYS_UBOOT_BASE	(QSPI0_AMBA_BASE + CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR * 512)

#if defined(CONFIG_CMD_NET)
#define CFG_FEC_XCV_TYPE             RGMII
#define CFG_FEC_MXC_PHYADDR          1
#define DWC_NET_PHYADDR			1
#define PHY_ANEG_TIMEOUT 20000
#endif

#ifdef CONFIG_DISTRO_DEFAULTS
#define BOOT_TARGET_DEVICES(func) \
       func(MMC, mmc, 1) \

#include <config_distro_bootcmd.h>
#else
#define BOOTENV
#endif

/* Link Definitions */
#define CFG_SYS_INIT_RAM_ADDR	0x40000000
#define CFG_SYS_INIT_RAM_SIZE	0x80000
#define CFG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CFG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#define CFG_MMCROOT                  "/dev/mmcblk2p2"  /* USDHC3 */


/* Totally 2GB DDR */
#define CFG_SYS_SDRAM_BASE		0x40000000
#define PHYS_SDRAM			0x40000000
#define PHYS_SDRAM_SIZE			0x40000000	/* 1 GB */
#define PHYS_SDRAM_2_SIZE		0

#define CFG_MXC_UART_BASE		UART4_BASE_ADDR

#define CFG_SYS_FSL_USDHC_NUM	3
#define CFG_SYS_FSL_ESDHC_ADDR	0

#define CFG_USB_MAX_CONTROLLER_COUNT         2
#define CFG_USBD_HS
#define CFG_USB_GADGET_VBUS_DRAW 2


#ifdef CONFIG_ANDROID_SUPPORT
#include "somlabs_spacesom_8mp_android.h"
#endif

#endif
