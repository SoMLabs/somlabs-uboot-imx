# (C) Copyright 2018 M. Wolowik
#
# SPDX-License-Identifier:	GPL-2.0+
#

obj-y  := visionsom-6ull.o
obj-y  += mmc.o

extra-$(CONFIG_USE_PLUGIN) :=  plugin.bin
$(obj)/plugin.bin: $(obj)/plugin.o
	$(OBJCOPY) -O binary --gap-fill 0xff $< $@
