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
#include <assert.h>
#include <bl_common.h>
#include <debug.h>
#include <delay_timer.h>
#include <errno.h>
#include <hi3660.h>
#include <mmio.h>
#include <string.h>

#define ADDR_CONVERT(addr)		((addr) < 0x40000 ? (addr) + 0xFFF30000 : (addr) + 0x40000000)

static void fw_data_init(void)
{
	unsigned long data_head_addr;
	unsigned int *data_addr;

	data_head_addr = mmio_read_32((uintptr_t) HISI_DATA_HEAD_BASE) + 0x14;
	data_addr = (unsigned int *) ADDR_CONVERT(data_head_addr);

	memcpy((void *)HISI_DATA0_BASE, (const void *)(unsigned long)ADDR_CONVERT(data_addr[0]), HISI_DATA0_SIZE);
	memcpy((void *)HISI_DATA1_BASE, (const void *)(unsigned long)ADDR_CONVERT(data_addr[1]), HISI_DATA1_SIZE);
}

int load_lpm3(void)
{
	INFO("start fw loading\n");

	fw_data_init();

	flush_dcache_range((uintptr_t)HISI_RESERVED_MEM_BASE,
			   HISI_RESERVED_MEM_SIZE);

	__asm__ volatile("sev");
	__asm__ volatile("sev");

	INFO("fw load success\n");

	return 0;
}
