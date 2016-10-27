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

#ifndef __HI3660_MAILBOX_H__
#define __HI3660_MAILBOX_H__

#include <mailbox.h>
#include <stdint.h>

#define CPU_A53 			(1 << 0)
#define CPU_LPM3 			(1 << 3)
#define CPU_MASK 			(0xFF)

#define MBX_REG_BASE 			0xE896A000
#define MBX_MAX_CHANNELS 		29
#define MBX_MAX_DATA_LEN 		32

/* address space that ranges from 0x0000 to 0x0800 is used for channels */
#define MBX_SOURCE(ch) 			(MBX_REG_BASE + ((ch) * 0x40) + 0x00)
#define MBX_DSET(ch) 			(MBX_REG_BASE + ((ch) * 0x40) + 0x04)
#define MBX_DCLEAR(ch) 			(MBX_REG_BASE + ((ch) * 0x40) + 0x08)
#define MBX_DSTATUS(ch) 		(MBX_REG_BASE + ((ch) * 0x40) + 0x0C)
#define MBX_MODE(ch) 			(MBX_REG_BASE + ((ch) * 0x40) + 0x10)
#define MBX_MODE_STATE_STATUS_MASK 	(0xF << 4)
#define MBX_MODE_STATE_ACK 		(0x8 << 4)
#define MBX_MODE_STATE_DEST 		(0x4 << 4)
#define MBX_MODE_STATE_SOURCE 		(0x2 << 4)
#define MBX_MODE_STATE_IDLE 		(0x1 << 4)
#define MBX_MODE_AUTO_LINK 		(1 << 1)
#define MBX_MODE_AUTO_ANSWER 		(1 << 0)

#define MBX_IMASK(ch) 			(MBX_REG_BASE + ((ch) * 0x40) + 0x14)
#define MBX_ICLR(ch) 			(MBX_REG_BASE + ((ch) * 0x40) + 0x18)
#define MBX_SEND(ch) 			(MBX_REG_BASE + ((ch) * 0x40) + 0x1C)
#define MBX_DATA0(ch) 			(MBX_REG_BASE + ((ch) * 0x40) + 0x20)
#define MBX_DATA1(ch) 			(MBX_REG_BASE + ((ch) * 0x40) + 0x24)
#define MBX_DATA2(ch) 			(MBX_REG_BASE + ((ch) * 0x40) + 0x28)
#define MBX_DATA3(ch) 			(MBX_REG_BASE + ((ch) * 0x40) + 0x2C)
#define MBX_DATA4(ch) 			(MBX_REG_BASE + ((ch) * 0x40) + 0x30)
#define MBX_DATA5(ch) 			(MBX_REG_BASE + ((ch) * 0x40) + 0x34)
#define MBX_DATA6(ch) 			(MBX_REG_BASE + ((ch) * 0x40) + 0x38)
#define MBX_DATA7(ch) 			(MBX_REG_BASE + ((ch) * 0x40) + 0x3C)

/* id means CPU ID that ranges from 0 to 7 */
#define CPU_IMST(id) 			(MBX_REG_BASE + ((id) * 8) + 0x800)
#define CPU_IRST(id) 			(MBX_REG_BASE + ((id) * 8) + 0x804)

#define IPC_MBX_ACTIVE 			(MBX_REG_BASE + 0x900)
#define IPC_LOCK 			(MBX_REG_BASE + 0xA00)

#define MBX_IPC_LOCKED 			1
#define MBX_IPC_UNLOCKED 		0
#define MBX_IPC_UNLOCK_MAGIC 		0x1ACCE551

int hi3660_mbox_init(mbox_params_t *params);

#endif /* __HI3660_MAILBOX_H__ */
