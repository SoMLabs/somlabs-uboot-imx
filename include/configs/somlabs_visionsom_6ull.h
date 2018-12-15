/*
 * Copyright (C) 2018 M.Wolowik
 * Copyright (C) 2018 EMSYSLABS
 *
 * Configuration settings for the SomLabs Visionsom 6ULL
 *
 * SPDX-License-Identifier: GPL-2.0+
 */
#ifndef __SOMLABS_VISIONSOM_6ULL_H
#define __SOMLABS_VISIONSOM_6ULL_H

#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>
#include "mx6_common.h"
#include <asm/mach-imx/gpio.h>

#ifdef CONFIG_SECURE_BOOT
#ifndef CONFIG_CSF_SIZE
#define CONFIG_CSF_SIZE 				0x4000
#endif
#endif

#define PHYS_SDRAM_SIZE					SZ_512M

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN			(16 * SZ_1M)

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE			UART1_BASE

/* MMC Configs */
#ifdef CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR		USDHC2_BASE_ADDR
/* NAND pin conflicts with usdhc2 */
#ifdef CONFIG_SYS_USE_NAND
#define CONFIG_SYS_FSL_USDHC_NUM		1
#else
#define CONFIG_SYS_FSL_USDHC_NUM		2
#endif
#endif

/* Configure Ethernet */
#ifdef CONFIG_CMD_NET
#define CONFIG_CMD_MII
#define CONFIG_LIB_RAND
#define CONFIG_NET_RANDOM_ETHADDR

#define CONFIG_FEC_MXC
#define CONFIG_MII
#define CONFIG_FEC_ENET_DEV				0

#if (CONFIG_FEC_ENET_DEV == 0)
#define IMX_FEC_BASE					ENET_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR          0x1
#define CONFIG_FEC_XCV_TYPE             RMII
#elif (CONFIG_FEC_ENET_DEV == 1)
#define IMX_FEC_BASE					ENET2_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR			0x2
#define CONFIG_FEC_XCV_TYPE				RMII
#endif
#define CONFIG_ETHPRIME					"FEC0"

#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL
#endif

/* I2C configs */
#ifdef CONFIG_CMD_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1			/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2			/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_SPEED			100000
#endif

#define CONFIG_SYS_MMC_IMG_LOAD_PART	1

#define CONFIG_MFG_ENV_SETTINGS \
	"mfgtool_args=setenv bootargs console=${console},${baudrate} " \
	CONFIG_BOOTARGS_CMA_SIZE \
	"rdinit=/linuxrc " \
	"g_mass_storage.stall=0 g_mass_storage.removable=1 " \
	"g_mass_storage.file=/fat g_mass_storage.ro=1 " \
	"g_mass_storage.idVendor=0x066F g_mass_storage.idProduct=0x37FF "\
	"g_mass_storage.iSerialNumber=\"\" "\
	CONFIG_MTDPARTS_DEFAULT "\0" \
	"clk_ignore_unused "\
	"\0" \
	"initrd_addr=0x83800000\0" \
	"initrd_high=0xffffffff\0" \
	"bootcmd_mfg=run mfgtool_args;bootz ${loadaddr} ${initrd_addr} ${fdt_addr};\0" \

#if defined(CONFIG_SYS_BOOT_NAND)
#define CONFIG_EXTRA_ENV_SETTINGS \
	"videomode=video=ctfb:x:800,y:480,depth:24,pclk:30000,le:40,ri:40,up:29,lo:13,hs:48,vs:3,sync:s,vmode:0\0" \
	"fdt_file=somlabs-visionsom-6ull-nand.dtb\0" \
	"fdt_addr=0x83000000\0" \
	"fdt_high=0xffffffff\0"	  \
	"initrd_high=0xffffffff\0" \
	"boot_fdt=try\0" \
	"ip_dyn=yes\0" \
	"console=ttymxc0\0" \
	"bootargs=console=ttymxc0,115200 ubi.mtd=ubi "  \
	"root=ubi0:rootfs rw rootfstype=ubifs rootwait=1 "		     \
	"" \
	CONFIG_MTDPARTS_DEFAULT \
	"bootcmd=mtdparts default; ubi part ubi; ubifsmount ubi0:rootfs;" \
	"ubifsload ${loadaddr} /boot/zImage;" \
	"ubifsload ${fdt_addr} /boot/${fdt_file};" \
	"bootz ${loadaddr} - ${fdt_addr}\0"
#else
#define CONFIG_EXTRA_ENV_SETTINGS \
	"script=boot.scr\0" \
	"image=zImage\0" \
	"console=ttymxc0\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"fdt_file=somlabs-visionsom-6ull.dtb\0" \
	"fdt_addr=0x83000000\0" \
	"boot_fdt=try\0" \
	"ip_dyn=yes\0" \
	"videomode=video=ctfb:x:800,y:480,depth:24,pclk:30000,le:40,ri:40,up:29,lo:13,hs:48,vs:3,sync:s,vmode:0\0" \
	"mmcdev="__stringify(CONFIG_SYS_MMC_ENV_DEV)"\0" \
	"mmcpart=" __stringify(CONFIG_SYS_MMC_IMG_LOAD_PART) "\0" \
	"mmcroot=" CONFIG_MMCROOT " rootwait rw\0" \
	"mmcautodetect=yes\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} " \
	"root=${mmcroot}\0" \
	"loadbootscript=" \
	"ext4load mmc ${mmcdev}:${mmcpart} ${loadaddr} /boot/${script};\0" \
	"bootscript=echo Running bootscript from mmc ...; " \
	"source\0" \
	"loadimage=ext4load mmc ${mmcdev}:${mmcpart} ${loadaddr} /boot/${image}\0" \
	"loadfdt=ext4load mmc ${mmcdev}:${mmcpart} ${fdt_addr} /boot/${fdt_file}\0" \
	"mmcboot=echo Booting from mmc ...; " \
	"run mmcargs; " \
	"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
		"if run loadfdt; then " \
			"bootz ${loadaddr} - ${fdt_addr}; " \
		"else " \
			"if test ${boot_fdt} = try; then " \
				"bootz; " \
			"else " \
				"echo WARN: Cannot load the DT; " \
			"fi; " \
		"fi; " \
	"else " \
		"bootz; " \
	"fi;\0" \
	"netargs=setenv bootargs console=${console},${baudrate} " \
	"root=/dev/nfs " \
	"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
	"netboot=echo Booting from net ...; " \
	"run netargs; " \
	"if test ${ip_dyn} = yes; then " \
		"setenv get_cmd dhcp; " \
	"else " \
		"setenv get_cmd tftp; " \
	"fi; " \
	"${get_cmd} ${image}; " \
	"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
		"if ${get_cmd} ${fdt_addr} ${fdt_file}; then " \
			"bootz ${loadaddr} - ${fdt_addr}; " \
		"else " \
			"if test ${boot_fdt} = try; then " \
				"bootz; " \
			"else " \
				"echo WARN: Cannot load the DT; " \
			"fi; " \
		"fi; " \
	"else " \
		"bootz; " \
	"fi;\0"
#define CONFIG_BOOTCOMMAND \
	"mmc dev ${mmcdev};" \
	"mmc dev ${mmcdev}; if mmc rescan; then " \
	   "if run loadbootscript; then " \
		   "run bootscript; " \
	   "else " \
		   "if run loadimage; then " \
			   "run mmcboot; " \
		   "else run netboot; " \
		   "fi; " \
	   "fi; " \
	"else run netboot; fi"
#endif

/* Miscellaneous configurable options */
#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 0x8000000)

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_SYS_HZ				1000

/* Physical Memory Map */
#define PHYS_SDRAM					MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* Environment organization */
#if defined CONFIG_SOMLABS_VISIONSOM_6ULL_EMMC || defined CONFIG_SOMLABS_VISIONSOM_6ULL_SD
#define CONFIG_SYS_MMC_ENV_DEV		1	/* USDHC2 */
#define CONFIG_SYS_MMC_ENV_PART		0	/* user area */
#define CONFIG_MMCROOT				"/dev/mmcblk1p1"  /* USDHC2 */
#else
#undef CONFIG_MMC
#undef CONFIG_CMD_MMC
#undef CONFIG_GENERIC_MMC
#undef CONFIG_BOUNCE_BUFFER
#undef CONFIG_FSL_ESDHC
#undef CONFIG_FSL_USDHC
#undef CONFIG_SUPPORT_EMMC_BOOT
#endif

#define CONFIG_ENV_SIZE				SZ_8K
#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET			(12 * SZ_64K)
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_OFFSET			(768 * 1024)
#define CONFIG_ENV_SECT_SIZE		(64 * 1024)
#define CONFIG_ENV_SPI_BUS			CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS			CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE			CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#elif defined(CONFIG_ENV_IS_IN_NAND)
#undef CONFIG_ENV_SIZE
#define CONFIG_ENV_OFFSET			(60 << 20)
#define CONFIG_ENV_SECT_SIZE		(128 << 10)
#define CONFIG_ENV_SIZE				CONFIG_ENV_SECT_SIZE
#endif

#define CONFIG_IMX_THERMAL

#define CONFIG_IOMUX_LPSR

#define CONFIG_SOFT_SPI

#ifdef CONFIG_FSL_QSPI
#define CONFIG_SYS_FSL_QSPI_AHB
#define CONFIG_SF_DEFAULT_BUS		0
#define CONFIG_SF_DEFAULT_CS		0
#define CONFIG_SF_DEFAULT_SPEED		40000000
#define CONFIG_SF_DEFAULT_MODE		SPI_MODE_0
#define FSL_QSPI_FLASH_NUM			1
#define FSL_QSPI_FLASH_SIZE			SZ_32M
#endif


/* NAND stuff */
#if defined(CONFIG_SYS_USE_NAND)

/* #define CONFIG_NAND_MXS */
#define CONFIG_SYS_MAX_NAND_DEVICE	1
/* #define CONFIG_SYS_NAND_BASE		0x40000000 */
#define CONFIG_SYS_NAND_BASE		-1
#define CONFIG_SYS_NAND_5_ADDR_CYCLE
/* #define CONFIG_SYS_NAND_ONFI_DETECTION */
#define CONFIG_SYS_NAND_USE_FLASH_BBT

#endif

#ifdef CONFIG_VIDEO
#define CONFIG_VIDEO_MXS
#define CONFIG_VIDEO_LOGO
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_VIDEO_BMP_LOGO
#define MXS_LCDIF_BASE MX6UL_LCDIF1_BASE_ADDR
#endif

#endif