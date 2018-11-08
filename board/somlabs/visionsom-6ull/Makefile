# (C) Copyright 2018 M. Wolowik
#
# SPDX-License-Identifier:	GPL-2.0+
#

obj-y  := visionsom-6ull.o
obj-y  += mmc.o

ifeq ($(CONFIG_FSL_FASTBOOT),y)
KBUILD_CFLAGS   += -DCONFIG_FSL_FASTBOOT
endif

ifeq ($(CONFIG_SOMLABS_VISIONSOM_6ULL_SD),y)
KBUILD_CFLAGS   += -DCONFIG_SOMLABS_VISIONSOM_6ULL_SD
endif

extra-$(CONFIG_USE_PLUGIN) :=  plugin.bin
$(obj)/plugin.bin: $(obj)/plugin.o
	$(OBJCOPY) -O binary --gap-fill 0xff $< $@
