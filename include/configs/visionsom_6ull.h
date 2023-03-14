/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2017-2020 A. Karas, SomLabs
 * Copyright (C) 2015-2016 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the SoMlabs VisionSOM 6UL/6ULL board.
 */
#ifndef __VISIONSOM_6ULL_H
#define __VISIONSOM_6ULL_H

#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>
#include "mx6_common.h"
#include <asm/mach-imx/gpio.h>

/* SPL options */
#include "imx6_spl.h"

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(16 * SZ_1M)

#define CONFIG_BOOTCOMMAND \
	"run set_fdt_file; " \
	"run checkbootdev; " \
	"if run loadfdtcustom; then " \
		"echo Using custom dtb file: ${fdt_file_custom}; " \
	"else " \
		"echo Using dtb file: ${fdt_file}; run loadfdt; " \
	"fi; " \
	"if run loadbootscript; then " \
		"run bootscript; " \
	"else " \
		"if run loadimage; then " \
			"run setbootargs; " \
			"bootz ${loadaddr} - ${fdt_addr}; " \
		"fi; " \
	"fi"

/* Miscellaneous configurable options */

#define CONFIG_SYS_HZ                   1000

#define CONFIG_SERIAL_TAG

/* Physical Memory Map */
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* NAND stuff */
#ifdef CONFIG_NAND_MXS
#define CONFIG_SYS_MAX_NAND_DEVICE      1
#define CONFIG_SYS_NAND_BASE            0x40000000
#define CONFIG_SYS_NAND_5_ADDR_CYCLE
#define CONFIG_SYS_NAND_ONFI_DETECTION
#define CONFIG_SYS_NAND_USE_FLASH_BBT
#endif

/* USB Configs */
#ifdef CONFIG_CMD_USB
#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)
#endif

#ifdef CONFIG_CMD_NET
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR		0x1
#define CONFIG_FEC_XCV_TYPE		RMII
#define CONFIG_ETHPRIME			"eth0"
#endif

#endif /* __VISIONSOM_6ULL_H */
