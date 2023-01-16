// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2023 Somlabs
 */

#include "hw_config.h"
#include <fuse.h>

#ifdef CONFIG_SPL_BUILD
#include "lpddr4_timing.h"
#endif

/*
   Definition of data structure held in GP10 fuse (bank 14, word 0)
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

const struct dram_params dram_data[] = {
	MEM_ENTRY("UNKNOWN",            0, NULL),
	MEM_ENTRY("DEFAULT",         1024, &dram_timing),
	MEM_ENTRY("K4F8E3S4HBMFCJ",  1024, &dram_timing),
	MEM_ENTRY("MT53D512M32D2DS", 2048, &dram_timing),
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
	 *   default memory configuration is taken.
	*/
	fuse_read(14, 0, &config.value);

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
	u32 ddr_type;

	vsom_config_t cfg = read_hw_config();

	ddr_type = cfg.ddr_type + 1;

	if(cfg.valid && ((ddr_type) < ARRAY_SIZE(dram_data)) && (dram_data[ddr_type].size > 0)) {
		index = ddr_type;
	} else {
		printf("ERROR: unknown memory type: %u (valid: %s)\n",
			   cfg.ddr_type, cfg.valid?"true":"false");
	}

	return index;
}

const char* spacesom8mp_get_dram_name(void)
{
	return dram_data[get_dram_info_index()].name;
}

/*
	return memory size in bytes
*/
phys_size_t spacesom8mp_get_dram_size(void)
{
	return dram_data[get_dram_info_index()].size * SZ_1M;
}

#ifdef CONFIG_SPL_BUILD
struct dram_timing_info* spacesom8mp_get_dram_timing(void)
{
	return dram_data[get_dram_info_index()].timing;
}
#endif

const char* spacesom8mp_get_hw_rev_str(void)
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

bool spacesom8mp_get_wifi_status(void)
{
	vsom_config_t cfg = read_hw_config();

	if(cfg.valid) {
		return cfg.wifi?true:false;
	}
	return false;
}
