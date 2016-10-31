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

#ifndef __DW_UFS_H__
#define __DW_UFS_H__

#include <sys/types.h>

/* Bus Throtting */
#define BUSTHRTL 			0xC0
/* Outstanding OCP Requests */
#define OOCPR 				0xC4
/* Fatal Error Interrupt Enable */
#define FEIE 				0xC8
/* C-Port Direct Access Configuration register */
#define CDACFG 				0xD0
/* C-Port Direct Access Transmit 1 register */
#define CDATX1 				0xD4
/* C-Port Direct Access Transmit 2 register */
#define CDATX2 				0xD8
/* C-Port Direct Access Receive 1 register */
#define CDARX1 				0xDC
/* C-Port Direct Access Receive 2 register */
#define CDARX2 				0xE0
/* C-Port Direct Access Status register */
#define CDASTA 				0xE4
/* UPIU Loopback Configuration register */
#define LBMCFG 				0xF0
/* UPIU Loopback Status */
#define LBMSTA 				0xF4
/* Debug register */
#define DBG 				0xF8
/* HClk Divider register */
#define HCLKDIV 			0xFC

#define PA_HSSERIES 			0x156A
#define VS_MPHY_CFG_UPDT 		0xD085
#define VS_MPHY_DISABLE 		0xD0C1

typedef struct dw_ufs_params {
	uintptr_t 		reg_base;
	uintptr_t 		desc_base;
	size_t 			desc_size;
	unsigned long 		flags;
} dw_ufs_params_t;

int dw_ufs_init(dw_ufs_params_t *params);

#endif /* __DW_UFS_H__ */
