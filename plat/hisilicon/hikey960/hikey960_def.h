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

#ifndef __HIKEY960_DEF_H__
#define __HIKEY960_DEF_H__

#include <common_def.h>
#include <tbbr_img_def.h>

/*
 * There're two DDR space region.
 * The first one ranges from 0x0000_0000 to 0xDFFF_FFFF.
 * The second one ranges from 0x0000_0001_0000_0000 to
 * 0x0000_0001_3FFF_FFFF.
 * At here, only the first 1GB DDR is mapped.
 */
#define DDR_BASE			0x0
#define DDR_SIZE			0xE0000000

#define DEVICE_BASE			0xE0000000
#define DEVICE_SIZE			0x20000000

/*
 * PL011 related constants
 */
#define PL011_UART5_BASE		0xFDF05000
#define PL011_BAUDRATE			115200
#define PL011_UART_CLK_IN_HZ		19200000

#define UFS_BASE			0
/* FIP partition */
#define HIKEY960_FIP_BASE		(UFS_BASE + 0x16c700000)
#define HIKEY960_FIP_MAX_SIZE		(6 << 30)

#define HIKEY960_UFS_DESC_BASE 		0x20000000
#define HIKEY960_UFS_DESC_SIZE 		0x00010000 	/* 64KB */
#define HIKEY960_UFS_DATA_BASE 		0x10000000
#define HIKEY960_UFS_DATA_SIZE 		0x0A000000 	/* 160MB */

#endif /* __HIKEY960_DEF_H__ */
