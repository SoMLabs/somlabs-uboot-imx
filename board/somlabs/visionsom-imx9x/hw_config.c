// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2025 Somlabs
 */

#include "hw_config.h"
#include <fuse.h>

#ifdef CONFIG_SPL_BUILD
#include "lpddr4_timing.h"
#endif

/*
   Definition of data structure held in OEM_SW_CFG fuse (bank 40, word 0)
*/
typedef struct {
    u8    hw_rev:4;
    u8    ddr_type:4;
    u8    wifi:1;
    u32   reserved:22;
    u8    valid:1;
} vsom_config_t;


/* Definition of data structure for DRAM parameters */
struct dram_params {
	char*                    name;		// memory name
	phys_size_t              size;		// in MB
#ifdef CONFIG_SPL_BUILD
	struct dram_timing_info* timing;
#endif
};

/* Helper macro to create DDR memory information entries */
#ifdef CONFIG_SPL_BUILD
#define MEM_ENTRY(name, size, cfg)\
    {name, size, cfg}
#else
#define MEM_ENTRY(name, size, cfg)\
    {name, size}
#endif

// TODO default timing change
const struct dram_params dram_data[] = {
    MEM_ENTRY("UNKNOWN",            0, NULL),
    MEM_ENTRY("DEFAULT",         1024, &dram_timing_mt53d512m16d1ds),
    MEM_ENTRY("MT53D512M16D1DS", 1024, &dram_timing_mt53d512m16d1ds),
    MEM_ENTRY("MT53E1G16D1ZW",   2048, &dram_timing_mt53e1g16d1zw),
};

/*
 *   Function to read configuration from fuses
 */
static vsom_config_t read_hw_config(void)
{
    union {
        vsom_config_t c;
        u32  value;
    } config;

    config.value = 0;

/*
 *  Hardware configuration is read from fuses.
 *  If fuses are not programmed we check if this is USB download boot and if so,
 *  default memory configuration is taken.
*/
    fuse_read(40, 0, &config.value);

    if (!config.c.valid && is_usb_boot()) {
        printf("Using default configuration...\n");
        config.value = 0x80000000;
    }

    debug("CFG: %08X, v: %c, wifi: %c, dram: %u, rev: %u\n",
          config.value, config.c.valid?'1':'0', config.c.wifi?'1':'0',
          config.c.ddr_type, config.c.hw_rev);

    return config.c;
}

/*
 *  Calculate index in DRAM info table based on HW information
 */
static u32 get_dram_info_index(void)
{
    u32 index = 0;

    vsom_config_t cfg = read_hw_config();

    if(cfg.valid && ((cfg.ddr_type + 1) < ARRAY_SIZE(dram_data)) && (dram_data[cfg.ddr_type + 1].size > 0)) {
        index = cfg.ddr_type + 1;
    } else {
        printf("ERROR: unknown memory type: %u (valid: %s)\n",
               cfg.ddr_type, cfg.valid? "true" : "false");
    }

    return index;
}

const char* visionsomimx93_get_dram_name(void)
{
    return dram_data[get_dram_info_index()].name;
}

/*
    return memory size in bytes
*/
phys_size_t visionsomimx93_get_dram_size(void)
{
    return dram_data[get_dram_info_index()].size * SZ_1M;
}

#ifdef CONFIG_SPL_BUILD
struct dram_timing_info* visionsomimx93_get_dram_timing(void)
{
    return dram_data[get_dram_info_index()].timing;
}
#endif

const char* visionsomimx93_get_hw_rev_str(void)
{
    vsom_config_t cfg = read_hw_config();

    if(cfg.valid) {
        switch(cfg.hw_rev) {
            case 0: return "1.0";
            case 1: return "1.1";
            default: break;
        }
    }
    // unknow revision/config data missing!
    return "x.x";
}

bool visionsomimx93_get_wifi_status(void)
{
    vsom_config_t cfg = read_hw_config();

    if(cfg.valid) {
        return cfg.wifi ? true : false;
    }
    return false;
}
