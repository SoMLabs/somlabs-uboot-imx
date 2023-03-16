/*
 * Copyright 2019 Arkadiusz Karas, arkadiusz.karas@somlabs.com
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */


/*
	This is a minimal implementation of NXP kobs-ng tool, it supports only imx6UL/ULL processors.
	Tested with 2G/2048 and 4G/4096B page size NAND flashes.

	It requiers software BCH and bit reverse libraries.
*/

#include <common.h>
#include <linux/mtd/mtd.h>
#include <linux/errno.h>
#include <command.h>
#include <console.h>
#include <watchdog.h>
#include <malloc.h>
#include <asm/byteorder.h>
#include <jffs2/jffs2.h>
#include <nand.h>
#include <command.h>
#include <asm/mach-imx/regs-bch.h>
#include <asm/io.h>
#include <linux/bch.h>
#include <linux/bitrev.h>

#ifdef CONFIG_NAND_MXS

struct FCB {
	uint32_t Checksum;
	uint32_t FingerPrint;				// 0x20424346 ('FCB ')
	uint32_t Version;					// 0x01000000
	struct {
		uint8_t data_setup;
		uint8_t data_hold;
		uint8_t address_setup;
		uint8_t dsample_time;
		uint8_t nand_timing_state;		// not used by ROM code
		uint8_t REA;					// not used by ROM code
		uint8_t RLOH;					// not used by ROM code
		uint8_t RHOH;					// not used by ROM code
	} m_NANDTiming;
	uint32_t PageDataSize;
	uint32_t TotalPageSize;				// data + oob
	uint32_t SectorsPerBlock;			// number of pages per block
	uint32_t NumberOfNANDs;				// not used by ROM code
	uint32_t TotalInternalDie;			// not used by ROM code
	uint32_t CellType;					// not used by ROM code
	uint32_t EccBlockNEccType;			// value from 0 to 20 indicating BCH errro correction level for Block BN
	uint32_t EccBlock0Size;				// size of block B0 used to configure BCH engine
	uint32_t EccBlockNSize;				// size of block BN used to configure BCH engine
	uint32_t EccBlock0EccType;			// value from 0 to 20 indicating BCH errro correction level for Block B0
	uint32_t MetadataBytes;				// number of metadata bytes
	uint32_t NumEccBlocksPerPage;		// number of ECC blocks BN not including B0
	uint32_t EccBlockNEccLevelSDK;		// not used by ROM code
	uint32_t EccBlock0SizeSDK;			// not used by ROM code
	uint32_t EccBlockNSizeSDK;			// not used by ROM code
	uint32_t EccBlock0EccLevelSDK;		// not used by ROM code
	uint32_t NumEccBlocksPerPageSDK;	// not used by ROM code
	uint32_t MetadataBytesSDK;			// not used by ROM code
	uint32_t EraseThreshold;			// not used by ROM code
	uint32_t BootPatch;                 // 0 for normal boot and 1 to load patch starting next to FCB.
	uint32_t PatchSectors;              // Size of patch in sectors.
	uint32_t Firmware1_startingPage;	// Page number address where the first copy of bootable firmware is located.
	uint32_t Firmware2_startingPage;	// Page number address where the second copy of bootable firmware is located.
	uint32_t PagesInFirmware1;			// Size of the first copy of firmware in pages.
	uint32_t PagesInFirmware2;			// Size of the second copy of firmware in pages.
	uint32_t DBBTSearchAreaStartAddress;// Page address for the bad block table search area.
	uint32_t BadBlockMarkerByte;		// 
	uint32_t BadBlockMarkerStartBit;	//
	uint32_t BBMarkerPhysicalOffset;	//
	uint32_t BCHType;					// 0 for BCH20 and 1 for BCH40.
	struct {
		uint32_t Timing2_ReadLatency;
		uint32_t Timing2_PreambleDelay;
		uint32_t Timing2_CEDelay;
		uint32_t Timing2_PostambleDelay;
		uint32_t Timing2_CmdAddPause;
		uint32_t Timing2_DataPause;
		uint32_t Speed;
		uint32_t Timing1_BusyTimeout;
	} TM;
	uint32_t DISBBSearch;
	uint32_t BBMark_spare_offset;
	uint32_t Onfi_sync_enable;
	uint32_t Onfi_sync_speed;
	uint32_t Onfi_syncNANDData[7];
	uint32_t DISBB_Search;
	uint8_t Reserved3[64];
};

struct DBBT {
	uint32_t reserved1;
	uint32_t FingerPrint;			// 0x44424254 ('DBBT')
	uint32_t Version;				// 0x00000001
	uint32_t reserved2;
	uint32_t DBBT_NUM_OF_PAGES;		// Size of the DBBT in pages
};


/* image header (IVT) structure definition */
struct IVT {
	uint32_t header;
	uint32_t entry;
	uint32_t reserved1;
	uint32_t dcd_addr;
	uint32_t boot_data;
	uint32_t self;
	uint32_t csf;
	uint32_t reserved2;
};

struct boot_data {
	uint32_t start;
	uint32_t length;
	uint32_t plugin_flag;
};

/*
    this function returns size of firmware stored at given address.
    If no valid firmware is found, funtion returns 0.
*/
static ulong get_firmware_size(ulong fw_img)
{
	ulong size = 0;
	struct IVT* ivt = (struct IVT*)fw_img;
	struct boot_data* bdata = (struct boot_data*)(fw_img + (ivt->boot_data - ivt->self));

	if(ivt->header != 0x402000D1) {
		printf("imxkobs: wrong header (%08X) - incorrect firmware image, exiting!\n", ivt->header);
		return 0;
	}

	printf("imxkobs: Image header:\n");
	printf("    header: %08X\n", ivt->header);		// 0x402000D1
	printf(" boot_data: %08X\n", ivt->boot_data);
	printf("  dcd_addr: %08X\n", ivt->dcd_addr);
	printf("      self: %08X\n", ivt->self);
	printf("       csf: %08X\n", ivt->csf);

	printf(" boot_data@ %08X\n", (u32)bdata);
	printf("     start: %08X\n", bdata->start);
	printf("    length: %08X\n", bdata->length);

	// length stored in file includes 1024 bytes padding needed for NAND (IVT ofset - see Reference Manula for i.mx6ULL)
	size = bdata->length - 1024;

	if(ivt->csf) {
		size += 0;	//TODO: add size used by CSF area in case of secure boot
	}

	return size;
}

/* parse NAND timings given from command line */
static void parse_nand_timings(struct FCB* fcb, const char* timings)
{
	//TODO: implement parameters parsing
}


static inline uint32_t mxs_nand_get_mark_offset(uint32_t page_data_size, uint32_t ecc_strength, uint32_t EccBlock0Size, uint32_t EccBlockNSize, uint32_t MetadataSize, uint32_t galois_field)
{
	uint32_t chunk_data_size_in_bits;
	uint32_t chunk_ecc_size_in_bits;
	uint32_t chunk_total_size_in_bits;
	uint32_t block_mark_chunk_number;
	uint32_t block_mark_chunk_bit_offset;
	uint32_t block_mark_bit_offset;

	chunk_data_size_in_bits = EccBlockNSize * 8;
	chunk_ecc_size_in_bits  = ecc_strength * galois_field;

	chunk_total_size_in_bits =
			chunk_data_size_in_bits + chunk_ecc_size_in_bits;

	/* Compute the bit offset of the block mark within the physical page. */
	block_mark_bit_offset = page_data_size * 8;

	if (EccBlock0Size == 0) /* ecc_for_meta */
		/* Subtract the metadata bits and ecc bits. */
		block_mark_bit_offset -= MetadataSize * 8
					+ chunk_ecc_size_in_bits;
	else
		/* Subtract the metadata bits. */
		block_mark_bit_offset -= MetadataSize * 8;

	/*
	 * Compute the chunk number (starting at zero) in which the block mark
	 * appears.
	 */
	block_mark_chunk_number =
			block_mark_bit_offset / chunk_total_size_in_bits;

	/*
	 * Compute the bit offset of the block mark within its chunk, and
	 * validate it.
	 */
	block_mark_chunk_bit_offset = block_mark_bit_offset -
			(block_mark_chunk_number * chunk_total_size_in_bits);

	if (block_mark_chunk_bit_offset > chunk_data_size_in_bits)
		return 1;

	/*
	 * Now that we know the chunk number in which the block mark appears,
	 * we can subtract all the ECC bits that appear before it.
	 */
	block_mark_bit_offset -=
		block_mark_chunk_number * chunk_ecc_size_in_bits;

	return block_mark_bit_offset;
}


static void prepare_fcb(struct FCB* fcb, struct mtd_info *mtd, const char* timings)
{
    // we read BCH Layout0 register to know desired NAND page layout
    // it is much easier than repeat all calculations, especially that some values are hardcoded in mxs_nand.c!
	// this assumesof course, that GPMI/BCH is already initialised!
    struct mxs_bch_regs *bch_regs = (struct mxs_bch_regs *)MXS_BCH_BASE;
    uint32_t tmp;

    memset(fcb, 0, mtd->writesize);         // clear memory content
    fcb->FingerPrint = 0x20424346;
    fcb->Version = 0x01000000;
    fcb->m_NANDTiming.data_setup = 80;
	fcb->m_NANDTiming.data_hold = 60;
	fcb->m_NANDTiming.address_setup = 25;
	fcb->m_NANDTiming.dsample_time = 6;
    if(timings){
        parse_nand_timings(fcb, timings);
    }
	fcb->PageDataSize =  mtd->writesize;
	fcb->TotalPageSize =  mtd->writesize + mtd->oobsize;		// data + oob
	fcb->SectorsPerBlock = mtd->erasesize/mtd->writesize;		// number of pages per block

    tmp = readl(&bch_regs->hw_bch_flash0layout0);
	fcb->EccBlock0Size = ((tmp & BCH_FLASHLAYOUT0_DATA0_SIZE_MASK) >> BCH_FLASHLAYOUT0_DATA0_SIZE_OFFSET)*4;// size of block B0 used to configure BCH engine
	fcb->EccBlock0EccType = (tmp & BCH_FLASHLAYOUT0_ECC0_MASK) >> BCH_FLASHLAYOUT0_ECC0_OFFSET;			// value from 0 to 20 indicating BCH errro correction level for Block B0
	fcb->MetadataBytes = (tmp & BCH_FLASHLAYOUT0_META_SIZE_MASK) >> BCH_FLASHLAYOUT0_META_SIZE_OFFSET;	// number of metadata bytes
	fcb->NumEccBlocksPerPage = (tmp & BCH_FLASHLAYOUT0_NBLOCKS_MASK) >> BCH_FLASHLAYOUT0_NBLOCKS_OFFSET;// number of ECC blocks BN not including B0
    fcb->BCHType = (tmp & BCH_FLASHLAYOUT0_GF13_0_GF14_1_MASK) >> BCH_FLASHLAYOUT0_GF13_0_GF14_1_OFFSET;     // GF13/GF14 

    tmp = readl(&bch_regs->hw_bch_flash0layout1);
	fcb->EccBlockNEccType = (tmp & BCH_FLASHLAYOUT1_ECCN_MASK) >> BCH_FLASHLAYOUT1_ECCN_OFFSET;
	fcb->EccBlockNSize = ((tmp & BCH_FLASHLAYOUT1_DATAN_SIZE_MASK) >> BCH_FLASHLAYOUT1_DATAN_SIZE_OFFSET)*4;// size of block BN used to configure BCH engine

    uint32_t bbm_bit_offset = mxs_nand_get_mark_offset(mtd->writesize, fcb->EccBlock0EccType*2, fcb->EccBlock0Size, fcb->EccBlockNSize, fcb->MetadataBytes, fcb->BCHType?14:13);

    fcb->BadBlockMarkerByte = bbm_bit_offset >> 3;
    fcb->BadBlockMarkerStartBit = bbm_bit_offset & 3;

    fcb->BBMarkerPhysicalOffset = mtd->writesize;

}


static uint32_t CalculateChecksum(const uint8_t* p, size_t size)
{
	uint32_t checksum = 0;
	
	while(size--) {
		checksum += *p++;
	}
	checksum ^= 0xFFFFFFFF;

	return checksum;
}

/**
 * Parses a string into a number.  The number stored at ptr is
 * potentially suffixed with K (for kilobytes, or 1024 bytes),
 * M (for megabytes, or 1048576 bytes), or G (for gigabytes, or
 * 1073741824).  If the number is suffixed with K, M, or G, then
 * the return value is the number multiplied by one kilobyte, one
 * megabyte, or one gigabyte, respectively.
 *
 * @param ptr where parse begins
 * @param retptr output pointer to next char after parse completes (output)
 * @return resulting unsigned int
 */
static u64 memsize_parse (const char *const ptr)
{
	char* eptr = NULL;
	u64 ret = simple_strtoull(ptr, &eptr, 0);

	switch (*eptr) {
		case 'G':
		case 'g':
			ret <<= 10;
		case 'M':
		case 'm':
			ret <<= 10;
		case 'K':
		case 'k':
			ret <<= 10;
		default:
			break;
	}

	return ret;
}

static int do_imxkobs(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
    loff_t off;
    ulong fw_addr;
    ulong fw_size;
    ulong max_parition_size;
    struct mtd_info *mtd;
    struct FCB* fcb;
    uint32_t* dbbt_data;
    ulong i;
    struct DBBT* dbbt;
    char* timings = NULL;
    ulong search_exponent = 1;
    int result = 1;
    u8* fwimg;
	u8* fcb_buffer;

    /* at least 2 arguments */
    if (argc < 3) {
        return CMD_RET_USAGE;
    }
    if (argc > 3) {
        timings = argv[4];
    }

    fw_addr = (ulong)simple_strtoul(argv[1], NULL, 16);
	max_parition_size = (ulong)memsize_parse(argv[2]);
	search_exponent = (ulong)memsize_parse(argv[3]);
    
    fw_size = get_firmware_size(fw_addr);

    if(fw_size == 0){
        printf("%s(): ERROR: Firmware image is not correct!\n", __func__);
        return 1;
    }
    fw_size += 1024;    // we need to add padding before image

    mtd = get_nand_dev_by_index(nand_curr_device);

    // calculate offsets
	ulong stride_size_in_bytes = mtd->erasesize;
    ulong search_area_size_in_strides = (1 << search_exponent);
	ulong search_area_size_in_bytes = search_area_size_in_strides * stride_size_in_bytes;     // (4 = 1 << SEARCH_EXPONENT (default equal 2))

	if ((search_area_size_in_bytes * 2) > max_parition_size) {
		printf( "mtd: boot partition size too small\n" 
			"\tsearch_area_size_in_bytes * 2 > boot partition size\n"
			"\t%#lx * 2 > %#lx\n",
			search_area_size_in_bytes, max_parition_size);
		return -1;
	}

	ulong search_area_size_in_pages = search_area_size_in_bytes / mtd->writesize;

	/*
	 * Figure out how large a boot stream the target MTD could possibly
	 * hold.
	 *
	 * The boot area will contain both search areas and two copies of the
	 * boot stream.
	 */
    ulong max_boot_stream_size_in_bytes = (max_parition_size - search_area_size_in_bytes * 2)/2;
    ulong boot_stream_size_in_pages = (fw_size + mtd->writesize - 1)/mtd->writesize;

    size_t fwimg_buff_size = (boot_stream_size_in_pages + 1)*mtd->writesize;

	/* Check if the boot stream will fit. */
	if (fw_size >= max_boot_stream_size_in_bytes) {
		printf("%s(): bootstream too large\n"
			"\tfw_size > max_boot_stream_size_in_bytes\n"
			"\t%#lx > %#lx\n", __func__, fw_size, max_boot_stream_size_in_bytes);
		return -1;
	}

    // allocate buffers
    fcb = malloc(mtd->writesize);           // allocate buffer for FCB block
	fcb_buffer = malloc(mtd->writesize + mtd->oobsize);	// write buffer fir FCB
    dbbt = malloc(mtd->writesize);           // allocate buffer for DDBT block
    dbbt_data = malloc(mtd->writesize);     // allocate buffer for DDBT data
    fwimg = malloc(fwimg_buff_size);        // buffer for final firmware image with padding

    // prepare discovered bad block table data - check only boot partition!
    memset(dbbt, 0, mtd->writesize);
    memset(dbbt_data, 0, mtd->writesize);
    dbbt->reserved1 = 0;
    dbbt->FingerPrint =  0x44424254;
    dbbt->Version = 0x00000001;
    dbbt->reserved2 = 0;
    dbbt->DBBT_NUM_OF_PAGES = 1;   // dbbt header

    dbbt_data[0] = 0;   // reserved
    dbbt_data[1] = 0;   // number of entries
    i = 0;
    for (off = search_area_size_in_bytes; off < max_parition_size; off += mtd->erasesize) {
        if (nand_block_isbad(mtd, off)){
            dbbt_data[2 + i] = ((ulong)off)/mtd->erasesize;
            if(++i > ((mtd->writesize/4)-2)) {
                goto error;
            }
        }
    }
    dbbt_data[1] = i;   // number of discovered bad blocks

    // copy Firmware image
    memset(fwimg, 0, 1024);  // padding
    memcpy(fwimg + 1024, (void*)fw_addr, fw_size - 1024);  // real image
    memset(fwimg + fw_size, 0, fwimg_buff_size - fw_size);  // guard page

    // prepare FCB
    prepare_fcb(fcb, mtd, timings);

    // find dbbt location
    fcb->DBBTSearchAreaStartAddress = search_area_size_in_pages;

    // find firmware location
    fcb->Firmware1_startingPage = (2 * search_area_size_in_bytes)/mtd->writesize;
    fcb->Firmware2_startingPage = ((2 * search_area_size_in_bytes) + max_boot_stream_size_in_bytes)/mtd->writesize;
    fcb->PagesInFirmware1 = boot_stream_size_in_pages;
    fcb->PagesInFirmware2 = boot_stream_size_in_pages;

	printf("%s(): max_boot_stream_size_in_bytes = %lu, boot_stream_size_in_bytes = %lu, boot_stream_size_in_pages = %lu\n",
			__func__, max_boot_stream_size_in_bytes, fw_size, boot_stream_size_in_pages);

	printf("%s(): fw#1 0x%08x - 0x%08lx (0x%08lx)\n"
		   "%s(): fw#2 0x%08x - 0x%08lx (0x%08lx)\n",
			__func__, fcb->Firmware1_startingPage*mtd->writesize,
            fcb->Firmware1_startingPage*mtd->writesize + max_boot_stream_size_in_bytes,
			fcb->Firmware1_startingPage*mtd->writesize + fw_size,
			__func__, fcb->Firmware2_startingPage*mtd->writesize,
			fcb->Firmware2_startingPage*mtd->writesize + max_boot_stream_size_in_bytes,
			fcb->Firmware2_startingPage*mtd->writesize + fw_size);

    // calculate checksum
	fcb->Checksum = CalculateChecksum(((uint8_t*)fcb) + 4, sizeof(*fcb) - 4);
	printf("%s(): fcb checksum: %08X\n", __func__, fcb->Checksum);

    // erase boot parition
    printf("%s(): erasing boot area...\n", __func__);
	nand_erase(mtd, 0, max_parition_size);

    // write firmware 1 image 
    printf("%s(): writing firmware image copy 1...\n", __func__);
    nand_write_skip_bad(mtd, fcb->Firmware1_startingPage*mtd->writesize, &fwimg_buff_size, NULL, 
                         max_boot_stream_size_in_bytes, fwimg, WITH_WR_VERIFY);

    // write firmware 2 image
    printf("%s(): writing firmware image copy 2...\n", __func__);
    nand_write_skip_bad(mtd, fcb->Firmware2_startingPage*mtd->writesize, &fwimg_buff_size, NULL, 
                         max_boot_stream_size_in_bytes, fwimg, WITH_WR_VERIFY);

    // write DBBT header
    printf("%s(): writing DBBT header...\n", __func__);
    {
        loff_t ofs = search_area_size_in_bytes;
        for(i = 0; i < search_area_size_in_strides; i++, ofs += stride_size_in_bytes){
            size_t wrsize = mtd->writesize;
            nand_write(mtd, ofs, &wrsize, (void*)dbbt);
        }
    }

    // write DBBT data
    printf("%s(): writing DBBT data...\n", __func__);
    if(dbbt_data[1] > 0) {      // write DDBT data only if there is any bad block found
		loff_t ofs = search_area_size_in_bytes;

		for (i = 0; i < search_area_size_in_strides; i++, ofs += stride_size_in_bytes) {
            size_t rwsize = mtd->writesize;
            printf("%s(): PUTTING down DBBT%ld BBTN%d @0x%llx (0x%x)\n", __func__, 
                            i, 0, ofs + 4 * mtd->writesize, mtd->writesize);
			
            int ret = nand_write(mtd, off +  4 * mtd->writesize , &rwsize, (void*)dbbt_data);

			if (ret) {
			 	printf("%s(): Failed to write BBTN @0x%llx (%d)\n", __func__, ofs, mtd->writesize);
			}
		}
	}
    // finally write FCB - with different BCH parameters - we calculate ECC using software BCH library 
	// and write FCB block in raw mode, with disabled hardware ECC module!
    printf("%s(): writing FCB...\n", __func__);
	{
		int i = 0;
		int j;
		struct bch_control* bch;

		// initialise fcb buffer
		memset(fcb_buffer, 0x00, mtd->writesize + mtd->oobsize);

		// create required layout for fcb data (metadata + data + ecc)
		// init BCH
		u32 ecc_bytes = (13*40)/8;	// (GF * ECC bits) / 8
		bch = init_bch(13, 40, 0);
		u8* src = (u8*)fcb;
		u8* dest = fcb_buffer + 32; // first 32 bytes are metadata (filled with 0's)
		for(i = 0; i < 8; i++) {	// loop for each ecc block
			u8 buffer[128];
			memcpy(dest, src, 128);
			dest += 128;
			for(j = 0; j < 128; j++) {	// we need to swap bits to make data words big-endiann
				buffer[j] = bitrev8(src[j]);
			}
			encode_bch(bch, buffer, 128, dest);
			/* reverse ecc bits */
			for (j = 0; j < ecc_bytes; j++) {
				dest[j] = bitrev8(dest[j]);
			}

			src += 128;
			dest += ecc_bytes;
		}
		free_bch(bch);
		// write FCB
		for (i = 0; i < (1 << search_exponent); i ++) {
			loff_t offset = i*search_area_size_in_bytes;
			mtd_oob_ops_t ops = {
				.datbuf = (u8 *)fcb_buffer,
				.oobbuf = (u8 *)fcb_buffer + mtd->writesize,
				.len = mtd->writesize,
				.ooblen = mtd->oobsize,
				.mode = MTD_OPS_RAW
			};
			printf("  writing fcb[%d] @ %08llX\n", i, offset);
			int ret = mtd_write_oob(mtd, offset, &ops);
			if (ret) {
				printf("%s: error at offset %llx, ret %d\n",
					__func__, (long long)off, ret);
			}
		}
	}
	printf("%s(): done\n", __func__);

    result = 0;

error:
    free(fcb);
	free(fcb_buffer);
    free(dbbt);
    free(dbbt_data);
    free(fwimg);
    return result;
}


U_BOOT_CMD(
        imxkobs, 5, 0, do_imxkobs,
        "imx-kobs tool",
        "fwaddr bootsize [search_exponent] [nand_timing]");

#endif
