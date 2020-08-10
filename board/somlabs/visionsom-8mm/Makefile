#
# Copyright 2018 NXP
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += visionsom-8mm.o

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
obj-$(CONFIG_VISIOSOM_8MM_MEMORY_MT53D512M32D2DS) += lpddr4_timing_MT53D512M32D2DS.o
obj-$(CONFIG_VISIOSOM_8MM_MEMORY_K4F6E304HBMGCJ) += lpddr4_timing_K4F6E304HBMGCJ.o
obj-$(CONFIG_VISIOSOM_8MM_MEMORY_MT53B256M32D1DS) += lpddr4_timing_MT53B256M32D1DS.o
endif
