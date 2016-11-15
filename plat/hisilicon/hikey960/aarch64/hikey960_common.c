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

#include <arch_helpers.h>
#include <arm_gic.h>
#include <assert.h>
#include <bl_common.h>
#include <debug.h>
#include <mmio.h>
#include <platform.h>
#include <platform_def.h>
#include <xlat_tables.h>

#include "../hikey960_def.h"
#include "../hikey960_private.h"

#define MAP_DDR		MAP_REGION_FLAT(DDR_BASE,			\
					DDR_SIZE,			\
					MT_DEVICE | MT_RW | MT_NS)

#define MAP_BL 		MAP_REGION_FLAT(BL1_RO_BASE, 			\
					BL31_LIMIT - BL1_RO_BASE,	\
					MT_DEVICE | MT_RW | MT_NS)

#define MAP_FWU		MAP_REGION_FLAT(BL31_LIMIT,			\
					0x000F0000,			\
					MT_MEMORY | MT_RO | MT_SECURE | MT_EXECUTE)

#define MAP_DEVICE	MAP_REGION_FLAT(DEVICE_BASE,			\
					DEVICE_SIZE,			\
					MT_DEVICE | MT_RW | MT_SECURE)

/*
 * Table of regions for different BL stages to map using the MMU.
 * This doesn't include Trusted RAM as the 'mem_layout' argument passed to
 * hikey960_init_mmu_elx() will give the available subset of that,
 */
#if IMAGE_BL1 || IMAGE_BL2
static const mmap_region_t hikey960_mmap[] = {
	MAP_DDR,
	MAP_BL,
	MAP_DEVICE,
	{0}
};
#endif

#if IMAGE_BL2U
static const mmap_region_t hikey960_mmap[] = {
	MAP_DDR,
	MAP_FWU,
	MAP_BL,
	MAP_DEVICE,
	{0}
};
#endif

/*
 * Macro generating the code for the function setting up the pagetables as per
 * the platform memory map & initialize the mmu, for the given exception level
 */
#define HIKEY960_CONFIGURE_MMU_EL(_el)					\
	void hikey960_init_mmu_el##_el(unsigned long total_base,	\
				unsigned long total_size,		\
				unsigned long ro_start,			\
				unsigned long ro_limit,			\
				unsigned long coh_start,		\
				unsigned long coh_limit)		\
	{								\
	       mmap_add_region(total_base, total_base,			\
			       total_size,				\
			       MT_MEMORY | MT_RW | MT_SECURE);		\
	       mmap_add_region(ro_start, ro_start,			\
			       ro_limit - ro_start,			\
			       MT_MEMORY | MT_RO | MT_SECURE);		\
	       mmap_add_region(coh_start, coh_start,			\
			       coh_limit - coh_start,			\
			       MT_DEVICE | MT_RW | MT_SECURE);		\
	       mmap_add(hikey960_mmap);					\
	       init_xlat_tables();					\
									\
	       enable_mmu_el##_el(0);					\
	}

/* Define EL1 and EL3 variants of the function initialising the MMU */
HIKEY960_CONFIGURE_MMU_EL(1)
HIKEY960_CONFIGURE_MMU_EL(3)

unsigned long plat_get_ns_image_entrypoint(void)
{
	return HIKEY960_NS_IMAGE_OFFSET;
}

unsigned int plat_get_syscnt_freq2(void)
{
	return 1200000;
}
