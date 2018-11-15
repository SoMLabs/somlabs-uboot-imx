# (C) Copyright 2018 M. Wolowik
#
# SPDX-License-Identifier:	GPL-2.0+
#

obj-y  := visionsom-6ull.o

ifeq ($(CONFIG_FSL_FASTBOOT),y)
KBUILD_CFLAGS   += -DCONFIG_FSL_FASTBOOT
endif

ifeq ($(CONFIG_SOMLABS_VISIONSOM_6ULL_SD),y)
KBUILD_CFLAGS   += -DCONFIG_SOMLABS_VISIONSOM_6ULL_SD
endif

ifeq ($(CONFIG_SYS_BOOT_NAND),y)
KBUILD_CFLAGS   += -DCONFIG_SYS_BOOT_NAND
endif

ifeq ($(CONFIG_BOOTARGS_CMA),y)
KBUILD_CFLAGS   += -DCONFIG_BOOTARGS_CMA
endif

ifeq ($(CONFIG_SYS_NO_FLASH),y)
KBUILD_CFLAGS   += -DCONFIG_SYS_NO_FLASH
endif

ifeq ($(CONFIG_DISPLAY_CPUINFO),y)
KBUILD_CFLAGS   += -DCONFIG_DISPLAY_CPUINFO
endif

ifeq ($(CONFIG_SYS_USE_NAND),y)
KBUILD_CFLAGS   += -DCONFIG_SYS_USE_NAND
endif

extra-$(CONFIG_USE_PLUGIN) :=  plugin.bin
$(obj)/plugin.bin: $(obj)/plugin.o
	$(OBJCOPY) -O binary --gap-fill 0xff $< $@
