/*
 * Copyright (C) 2017-2019 A. Karas, SomLabs
 * Copyright (C) 2015-2016 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6UL 14x14 EVK board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __SOMLABS_VISIONSOM_6ULL_H
#define __SOMLABS_VISIONSOM_6ULL_H

#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>
#include "mx6_common.h"
#include <asm/mach-imx/gpio.h>

#ifdef CONFIG_SECURE_BOOT
#ifndef CONFIG_CSF_SIZE
#define CONFIG_CSF_SIZE 0x4000
#endif
#endif

#define PHYS_SDRAM_SIZE		SZ_512M
#define BOOTARGS_CMA_SIZE   ""

#undef CONFIG_LDO_BYPASS_CHECK

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(16 * SZ_1M)

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART1_BASE

/* MMC Configs */
#ifdef CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR	USDHC2_BASE_ADDR

#define CONFIG_SYS_FSL_USDHC_NUM	1
#endif /* CONFIG_FSL_USDHC */


#define MFG_NAND_PARTITION "mtdparts=gpmi-nand:3M(boot),1M(splash),-(ubi)"

#define CONFIG_CMD_READ
#define CONFIG_SERIAL_TAG

#define CONFIG_MFG_ENV_SETTINGS \
	"initrd_addr=0x86800000\0" \
	"initrd_high=0xffffffff\0" \
	"bootdelay=0\0" \
	"emmc_dev=1\0"\
	"emmc_ack=1\0"\
	"sd_dev=1\0" \

#define CONFIG_EXTRA_ENV_SETTINGS \
	CONFIG_MFG_ENV_SETTINGS \
	"fdt_addr=0x83000000\0" \
	"fdt_high=0xffffffff\0"	  \
	"console=ttymxc0\0" \
	"script=boot.scr\0" \
	"image=zImage\0" \
	"splashimage=0x80000000\0" \
	"setrootnand=setenv rootspec root=ubi0:rootfs ubi.mtd=ubi rootfstype=ubifs "MFG_NAND_PARTITION"\0" \
	"setbootscriptnand=setenv loadbootscript ubifsload ${loadaddr} /boot/${script};\0" \
	"setloadnand=setenv loadimage ubifsload ${loadaddr} /boot/${image}; " \
	            "setenv loadfdt ubifsload ${fdt_addr} /boot/${fdt_file};\0"\
	"mmcdev=0\0" \
	"mmcpart=1\0" \
	"mmcroot=/dev/mmcblk0p1 rootwait rw\0" \
	"setrootmmc=setenv rootspec root=${mmcroot}\0" \
	"setbootscriptmmc=setenv loadbootscript ext4load mmc ${mmcdev}:${mmcpart} ${loadaddr} /boot/${script};\0" \
	"setloadmmc=setenv loadimage ext4load mmc ${mmcdev}:${mmcpart} ${loadaddr} /boot/${image}; " \
	           "setenv loadfdt ext4load mmc ${mmcdev}:${mmcpart} ${fdt_sddr} /boot/${fdt_file};\0" \
	"setbootargs=setenv bootargs console=${console},${baudrate} " \
		BOOTARGS_CMA_SIZE \
		"${rootspec}\0" \
	"execbootscript=echo Running bootscript...; " \
		"source\0" \
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

#define CONFIG_BOOTCOMMAND \
		"run setfdtfile; " \
		"run checkbootdev; " \
		"run loadfdt;" \
		"if run loadbootscript; then " \
			"run bootscript; " \
		"else " \
			"if run loadimage; then " \
				"run setbootargs; " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
		   "fi; " \
	   "fi"

/* Miscellaneous configurable options */
#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 0x8000000)

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_SYS_HZ			1000

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#define CONFIG_IMX_THERMAL

#define CONFIG_IOMUX_LPSR


#ifdef CONFIG_FSL_QSPI
#define CONFIG_SYS_FSL_QSPI_AHB
#define CONFIG_SF_DEFAULT_BUS		0
#define CONFIG_SF_DEFAULT_CS		0
#define CONFIG_SF_DEFAULT_SPEED	40000000
#define CONFIG_SF_DEFAULT_MODE		SPI_MODE_0
#define FSL_QSPI_FLASH_NUM		1
#define FSL_QSPI_FLASH_SIZE		SZ_32M
#endif	/* CONFIG_FSL_QSPI */

#ifdef CONFIG_CMD_NAND
#define CONFIG_CMD_NAND_TRIMFFS

#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_SYS_NAND_BASE		0x40000000
#define CONFIG_SYS_NAND_5_ADDR_CYCLE
#define CONFIG_SYS_NAND_ONFI_DETECTION

/* Dynamic MTD partition support */
#define CONFIG_MTD_PARTITIONS
#define CONFIG_MTD_DEVICE
#define MTDIDS_DEFAULT            "nand0=gpmi-nand"
#define MTDPARTS_DEFAULT          MFG_NAND_PARTITION

#endif	/* CONFIG_CMD_NAND */

#define CONFIG_ENV_SIZE			SZ_8K

/* USB Configs */
#ifdef CONFIG_CMD_USB
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#endif	/* CONFIG_CMD_USB */

#ifdef CONFIG_FEC_MXC
#define CONFIG_MII
#define CONFIG_FEC_ENET_DEV		1

#if (CONFIG_FEC_ENET_DEV == 0)
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR          0x1
#define CONFIG_FEC_XCV_TYPE             RMII
#ifdef CONFIG_DM_ETH
#define CONFIG_ETHPRIME			"eth0"
#else	/* CONFIG_DM_ETH */
#define CONFIG_ETHPRIME			"FEC0"
#endif	/* CONFIG_DM_ETH */
#elif (CONFIG_FEC_ENET_DEV == 1)
#define IMX_FEC_BASE			ENET2_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR		0x2
#define CONFIG_FEC_XCV_TYPE		RMII
#ifdef CONFIG_DM_ETH
#define CONFIG_ETHPRIME			"eth1"
#else /* CONFIG_DM_ETH */
#define CONFIG_ETHPRIME			"FEC1"
#endif /* CONFIG_DM_ETH*/
#endif	/* CONFIG_FEC_ENET_DEV */
#define CONFIG_FEC_MXC_MDIO_BASE ENET_BASE_ADDR
#endif	/* CONFIG_FEC_MXC */

#ifdef CONFIG_VIDEO
#define CONFIG_VIDEO_MXS
#define CONFIG_VIDEO_LOGO
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_SPLASH_SOURCE
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IMX_VIDEO_SKIP
#endif	/* CONFIG_VIDEO */

#define CONFIG_MODULE_FUSE
#define CONFIG_OF_SYSTEM_SETUP

#endif
