// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2023 Somlabs
 */

#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#include <common.h>
#include <asm/arch/ddr.h>

#ifdef CONFIG_SPL_BUILD
struct dram_timing_info* spacesom8mp_get_dram_timing(void);
#endif

const char* spacesom8mp_get_dram_name(void);

phys_size_t spacesom8mp_get_dram_size(void);

const char* spacesom8mp_get_hw_rev_str(void);

bool spacesom8mp_get_wifi_status(void);

#endif /* HW_CONFIG_H */
