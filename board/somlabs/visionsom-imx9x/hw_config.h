// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2025 Somlabs
 */

#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#include <common.h>
#include <asm/arch/ddr.h>

#ifdef CONFIG_SPL_BUILD
struct dram_timing_info* visionsomimx93_get_dram_timing(void);
#endif

const char* visionsomimx93_get_dram_name(void);

phys_size_t visionsomimx93_get_dram_size(void);

const char* visionsomimx93_get_hw_rev_str(void);

bool visionsomimx93_get_wifi_status(void);

#endif /* HW_CONFIG_H */
