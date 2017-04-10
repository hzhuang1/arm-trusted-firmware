/*
 * Copyright (c) 2017, Linaro Limited and Contributors. All rights reserved.
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
 * Neither the name of Linaro nor the names of its contributors may be used
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
#ifndef __HI3660_HKADC_H__
#define __HI3660_HKADC_H__

#define HKADC_SSI_REG_BASE			0xE82B8000

#define HKADC_DSP_START				(HKADC_SSI_REG_BASE + 0x000)
#define HKADC_WR_NUM				(HKADC_SSI_REG_BASE + 0x008)
#define HKADC_DSP_START_CLR			(HKADC_SSI_REG_BASE + 0x01C)
#define HKADC_WR01_DATA				(HKADC_SSI_REG_BASE + 0x020)

#define WR1_WRITE_MODE				(1 << 31)
#define WR1_READ_MODE				(0 << 31)
#define WR1_ADDR(x)				(((x) & 0x7F) << 24)
#define WR1_DATA(x)				(((x) & 0xFF) << 16)
#define WR0_WRITE_MODE				(1 << 15)
#define WR0_READ_MODE				(0 << 15)
#define WR0_ADDR(x)				(((x) & 0x7F) << 8)
#define WR0_DATA(x)				((x) & 0xFF)

#define HKADC_WR23_DATA				(HKADC_SSI_REG_BASE + 0x024)
#define HKADC_WR45_DATA				(HKADC_SSI_REG_BASE + 0x028)
#define HKADC_DELAY01				(HKADC_SSI_REG_BASE + 0x030)
#define HKADC_DELAY23				(HKADC_SSI_REG_BASE + 0x034)
#define HKADC_DELAY45				(HKADC_SSI_REG_BASE + 0x038)
#define HKADC_DSP_RD2_DATA			(HKADC_SSI_REG_BASE + 0x048)
#define HKADC_DSP_RD3_DATA			(HKADC_SSI_REG_BASE + 0x04C)

// HKADC Internal Registers
#define HKADC_CTRL_ADDR				0x00
#define HKADC_START_ADDR			0x01
#define HKADC_DATA1_ADDR			0x03   // high 8 bits
#define HKADC_DATA0_ADDR			0x04   // low 8 bits
#define HKADC_MODE_CFG				0x0A

#define HKADC_VALUE_HIGH			0x0FF0
#define HKADC_VALUE_LOW				0x000F
#define HKADC_VALID_VALUE			0x0FFF

#define HKADC_CHANNEL_MAX			15
#define HKADC_VREF_1V8				1800
#define HKADC_ACCURACY				0x0FFF

#define HKADC_WR01_VALUE			((HKADC_START_ADDR << 24) | \
						 (0x1 << 16))
#define HKADC_WR23_VALUE			((0x1 << 31) |		\
						 (HKADC_DATA0_ADDR << 24) | \
						 (1 << 15) |		\
						 (HKADC_DATA1_ADDR << 8))
#define HKADC_WR45_VALUE			(0x80)
#define HKADC_CHANNEL0_DELAY01_VALUE		((0x0700 << 16) | 0xFFFF)
#define HKADC_DELAY01_VALUE			((0x0700 << 16) | 0x0200)
#define HKADC_DELAY23_VALUE			((0x00C8 << 16) | 0x00C8)
#define START_DELAY_TIMEOUT			2000
#define HKADC_WR_NUM_VALUE			4

#endif /* __HI3660_HKADC_H__ */
