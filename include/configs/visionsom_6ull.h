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

/* for version with fastboot we set bootdelay to 0 */
#ifdef CONFIG_FSL_FASTBOOT
#define CONFIG_MFG_ENV_SETTINGS \
	"bootdelay=0\0"
#else
#define CONFIG_MFG_ENV_SETTINGS
#endif

#define CONFIG_EXTRA_ENV_SETTINGS \
	CONFIG_MFG_ENV_SETTINGS \
	"bootm_size=0x10000000\0" \
	"console=ttymxc0\0" \
	"initrd_addr=0x86800000\0" \
	"fdt_addr=0x83000000\0" \
	"script=boot.scr\0" \
	"image=zImage\0" \
	"splashimage=0x80000000\0" \
	"splashfile=/boot/splash.bmp\0" \
	"setrootnand=setenv rootspec root=ubi0:rootfs ubi.mtd=ubi " \
		"rootfstype=ubifs "CONFIG_MTDPARTS_DEFAULT"\0" \
	"setbootscriptnand=setenv loadbootscript ubifsload " \
		"${loadaddr} /boot/${script};\0" \
	"setloadnand=setenv loadimage ubifsload ${loadaddr} /boot/${image}; " \
	            "setenv loadfdt ubifsload ${fdt_addr} /boot/${fdt_file};\0" \
	"mmcdev=1\0" \
	"mmcpart=1\0" \
	"mmcroot=/dev/mmcblk1p1 rootwait rw\0" \
	"setrootmmc=setenv rootspec root=${mmcroot}\0" \
	"setbootscriptmmc=setenv loadbootscript " \
		"load mmc ${mmcdev}:${mmcpart} " \
		"${loadaddr} /boot/${script};\0" \
	"setloadmmc=setenv loadimage load mmc ${mmcdev}:${mmcpart} " \
		"${loadaddr} /boot/${image}; " \
		"setenv loadfdt load mmc ${mmcdev}:${mmcpart} " \
		"${fdt_addr} /boot/${fdt_file};\0" \
	"setbootargs=setenv bootargs console=${console},${baudrate} " \
		"${rootspec}\0" \
	"execbootscript=echo Running bootscript...; source\0" \
	"setfdtfile=setenv fdt_file somlabs-${board}${fdt_suffix}.dtb\0" \
	"checkbootdev=if test ${bootdev} = nand; then " \
		"nand device 0; ubi part ubi; ubifsmount ubi0:rootfs; " \
		"run setbootscriptnand; " \
		"run setrootnand; " \
		"run setloadnand; " \
	"else " \
		"run setbootscriptmmc; " \
		"run setrootmmc; " \
		"run setloadmmc; " \
	"fi; " \

/* Miscellaneous configurable options */

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

//#define CONFIG_ENV_SIZE			SZ_8K

/* USB Configs */
#ifdef CONFIG_CMD_USB
#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)
#endif

<<<<<<< HEAD:include/configs/somlabs_visionsom_6ull.h
#ifdef CONFIG_CMD_NET
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR		0x1
#define CONFIG_FEC_XCV_TYPE		RMII
#define CONFIG_ETHPRIME			"eth0"
=======
#ifndef CONFIG_SPL_BUILD
#ifdef CONFIG_DM_VIDEO
#define CONFIG_VIDEO_MXS
#define CONFIG_VIDEO_LOGO
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_SPLASH_SOURCE
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_VIDEO_BMP_GZIP
#define CONFIG_SYS_VIDEO_LOGO_MAX_SIZE  (2 << 20)
>>>>>>> 574622302a... SOMLABS: add support for VisionSOM-6UL/ULL modules:include/configs/visionsom_6ull.h
#endif

#endif

#endif /* __VISIONSOM_6ULL_H */
