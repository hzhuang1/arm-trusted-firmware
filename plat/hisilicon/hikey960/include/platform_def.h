/*
 * Copyright (c) 2016, ARM Limited and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __PLATFORM_DEF_H__
#define __PLATFORM_DEF_H__

#include <arch.h>
#include "../hikey960_def.h"

/*
 * Platform binary types for linking
 */
#define PLATFORM_LINKER_FORMAT          "elf64-littleaarch64"
#define PLATFORM_LINKER_ARCH            aarch64


/*
 * Generic platform constants
 */

/* Size of cacheable stacks */
#define PLATFORM_STACK_SIZE		0x800

#define FIRMWARE_WELCOME_STR		"Booting Trusted Firmware\n"

#define PLATFORM_CACHE_LINE_SIZE	64
#define PLATFORM_CLUSTER_COUNT		2
#define PLATFORM_CORE_COUNT_PER_CLUSTER	4
#define PLATFORM_CORE_COUNT		(PLATFORM_CLUSTER_COUNT *	\
					 PLATFORM_CORE_COUNT_PER_CLUSTER)
#define PLAT_MAX_PWR_LVL		MPIDR_AFFLVL2
#define PLAT_NUM_PWR_DOMAINS		(PLATFORM_CORE_COUNT + \
					 PLATFORM_CLUSTER_COUNT + 1)

#define PLAT_MAX_RET_STATE		1
#define PLAT_MAX_OFF_STATE		2

#define MAX_IO_DEVICES			3
#define MAX_IO_HANDLES			4
/* UFS RPMB and UFS User Data */
#define MAX_IO_BLOCK_DEVICES		2


/*
 * Platform memory map related constants
 */

/*
 * BL1 specific defines.
 */
#define BL1_RO_BASE			(0x1AC00000)
#define BL1_RO_LIMIT			(BL1_RO_BASE + 0x10000)
#define BL1_RW_BASE			(BL1_RO_LIMIT) 		/* 1AC1_0000 */
#define BL1_RW_SIZE			(0x00088000)
#define BL1_RW_LIMIT			(0x1AC98000)

#define NS_BL1U_BASE			(BL1_RW_BASE + 0x8000)	/* 1AC1_8000 */

/*
 * BL2 specific defines.
 */
#define BL2_BASE			(BL1_RW_BASE + 0x8000)	/* 1AC1_8000 */
#define BL2_LIMIT			(BL2_BASE + 0x40000) 	/* 1AC5_8000 */

#if 0
/*
 * BL2U specific defines.
 */
#define BL2U_BASE			BL2_BASE 		/* 1AC1_8000 */
#define BL2U_LIMIT			(BL2U_BASE + 0x20000) 	/* 1AC3_8000 */
#endif

/*
 * BL31 specific defines.
 */
#define BL31_BASE			BL2_LIMIT 		/* 1AC5_8000 */
#define BL31_LIMIT			(BL31_BASE + 0x40000) 	/* 1AC9_8000 */

/*
 * Platform specific page table and MMU setup constants
 */
#define ADDR_SPACE_SIZE			(1ull << 32)

#if IMAGE_BL1 || IMAGE_BL2U || IMAGE_BL2 || IMAGE_BL31
#define MAX_XLAT_TABLES			3
#endif

#define MAX_MMAP_REGIONS		16

#define HIKEY960_NS_IMAGE_OFFSET	(DDR_BASE + 0x1AC98000)

/*
 * Declarations and constants to access the mailboxes safely. Each mailbox is
 * aligned on the biggest cache line size in the platform. This is known only
 * to the platform as it might have a combination of integrated and external
 * caches. Such alignment ensures that two maiboxes do not sit on the same cache
 * line at any cache level. They could belong to different cpus/clusters &
 * get written while being protected by different locks causing corruption of
 * a valid mailbox address.
 */
#define CACHE_WRITEBACK_SHIFT   	6
#define CACHE_WRITEBACK_GRANULE 	(1 << CACHE_WRITEBACK_SHIFT)

#endif /* __PLATFORM_DEF_H__ */
