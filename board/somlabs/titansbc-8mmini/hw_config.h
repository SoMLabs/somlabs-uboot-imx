// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 Somlabs
 */

#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#include <asm/arch/ddr.h>
#include <linux/sizes.h>

#ifdef CONFIG_SPL_BUILD
struct dram_timing_info* titansbc8mmini_get_dram_timing(void);
#endif

const char* titansbc8mmini_get_dram_name(void);

phys_size_t titansbc8mmini_get_dram_size(void);

const char* titansbc8mmini_get_hw_rev_str(void);

#endif /* HW_CONFIG_H */
