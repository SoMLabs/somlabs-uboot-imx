// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 Somlabs
 */

#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#include <common.h>
#include <asm/arch/ddr.h>

#ifdef CONFIG_SPL_BUILD
struct dram_timing_info* visionsbc8mmini_get_dram_timing(void);
#endif

const char* visionsbc8mmini_get_dram_name(void);

phys_size_t visionsbc8mmini_get_dram_size(void);

const char* visionsbc8mmini_get_hw_rev_str(void);

#endif /* HW_CONFIG_H */
