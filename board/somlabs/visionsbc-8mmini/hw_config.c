// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 Somlabs
 */

#include "hw_config.h"
#include <fuse.h>


/*
   Definition of data structure held in GP10 fuse (bank 14, word 0)
*/
typedef struct {
    u8    hw_rev:4;
    u8    ddr_type:4;
    u32   reserved:23;
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

#ifdef CONFIG_SPL_BUILD
extern struct dram_timing_info dram_timing_mt53d512m32d2ds;
extern struct dram_timing_info dram_timing_mt53b256m32d1ds;
extern struct dram_timing_info dram_timing_mt53d1024m32d4dt;
#endif

const struct dram_params dram_data[] = {
    MEM_ENTRY("UNKNOWN",            0, NULL),
	MEM_ENTRY("MT53D512M32D2DS", 2048, &dram_timing_mt53d512m32d2ds),
	MEM_ENTRY("MT53D512M32D2DS",  2048, &dram_timing_mt53d512m32d2ds),
	MEM_ENTRY("MT53B256M32D1DS", 1024, &dram_timing_mt53b256m32d1ds),
	MEM_ENTRY("MT53D1024M32D4DT", 4096, &dram_timing_mt53d1024m32d4dt),
};

/*
 *   Function to read configuration from fuses
 */
static vsom_config_t read_hw_config(void)
{
    union {
        vsom_config_t c;
        u32  value;
    }config;

    config.value = 0;

	/*
	 *  Hardware configuration is read from fuses.
	 *   If fuses are not programmed we check if this is USB download boot and if so,
	 *   memory configuration is taken from fixed location in OCRAM,
     *   when proper flag is detected.
	 *   This solution allows to use single bootloader build for boards programing.
	*/
	fuse_read(14, 0, &config.value);

    if (!config.c.valid && is_usb_boot()) {
        printf("Reading configuration from OCRAM...");
		// check last 2 words in OCRAM for DRAM config options
		u32* cfg = (u32*)0x93fffc;
		u32* flag = (u32*)0x93fff8;
		if (*flag == 0x55AABEEF) {
			config.value = *cfg;
		}
        printf("config value: %08X(flag:%08X)\n", *cfg, *flag);
	}

    debug("CFG: %08X, v: %c, dram: %u, rev: %u\n",
          config.value, config.c.valid?'1':'0',
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

    if(cfg.valid && ((cfg.ddr_type + 1) < ARRAY_SIZE(dram_data))) {
        index = cfg.ddr_type + 1;
    } else {
        printf("ERROR: unknown memory type: %u (valid: %s)\n",
               cfg.ddr_type, cfg.valid?"true":"false");
    }

    return index;
}

const char* visionsbc8mmini_get_dram_name(void)
{
    return dram_data[get_dram_info_index()].name;
}

/*
    return memory size in bytes
*/
phys_size_t visionsbc8mmini_get_dram_size(void)
{
    return dram_data[get_dram_info_index()].size * SZ_1M;
}

#ifdef CONFIG_SPL_BUILD
struct dram_timing_info* visionsbc8mmini_get_dram_timing(void)
{
    return dram_data[get_dram_info_index()].timing;
}
#endif

const char* visionsbc8mmini_get_hw_rev_str(void)
{
    vsom_config_t cfg = read_hw_config();

    if(cfg.valid) {
        switch(cfg.hw_rev) {
            case 0: return "1.0";
            case 1: return "1.1";
            case 2: return "1.2";
            default: break;
        }
    }
    // unknow revision/config data missing!
    return "x.x";
}
