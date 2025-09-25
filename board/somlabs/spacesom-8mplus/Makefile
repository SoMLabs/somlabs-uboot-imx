#
# Copyright 2019 NXP
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += spacesom-8mplus.o
obj-y += hw_config.o

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
ifdef CONFIG_IMX8M_LPDDR4_FREQ0_3200MTS
obj-y += lpddr4_timing_ndm.o
else
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing.o
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing_4g.o
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing_mt53e1g32d2fw.o
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing_mt53e2g32d4de.o
endif
endif
