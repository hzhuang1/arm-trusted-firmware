/*
 * Copyright (c) 2016, Linaro Limited and Contributors. All rights reserved.
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
#ifndef __HI3660_H__
#define __HI3660_H__

#include <hi3660_crg.h>

#define LP_RAM_BASE 			0xFFF50000

#define SCTRL_REG_BASE 			0xFFF0A000

#define SCTRL_CONTROL_OFFSET 		0x000
#define SCTRL_CONTROL_SYS_MODE(x) 	(((x) & 0xf) << 3)
#define SCTRL_CONTROL_SYS_MODE_NORMAL 	((1 << 2) << 3)
#define SCTRL_CONTROL_SYS_MODE_SLOW 	((1 << 1) << 3)
#define SCTRL_CONTROL_SYS_MODE_MASK 	(0xf << 3)
#define SCTRL_CONTROL_MODE_CTRL_NORMAL 	(1 << 2)
#define SCTRL_CONTROL_MODE_CTRL_SLOW 	(1 << 1)
#define SCTRL_CONTROL_MODE_CTRL_MASK 	0x7

#define SCTRL_DEEPSLEEPED_OFFSET 	0x008
#define SCTRL_EFUSE_USB_MASK 		(1 << 30)
#define SCTRL_EFUSE_USB_PLL 		(1 << 30)
#define SCTRL_EFUSE_USB_ABB 		(0 << 30)
#define SCTRL_EFUSE_UFS_MASK 		(3 << 6)
#define SCTRL_EFUSE_UFS_PLL 		(1 << 6)
#define SCTRL_EFUSE_UFS_ABB 		(0 << 6)

#define SCTRL_SCISOEN_OFFSET 		0x040
#define SCTRL_SCISODIS_OFFSET 		0x044
#define SCISO_MMBUFISO 			(1 << 3)

#define SCTRL_SCPWREN_OFFSET 		0x060
#define SCPWREN_MMBUFPWREN 		(1 << 3)

#define SCTRL_PLL_CTRL0_OFFSET 		0x100
#define SCTRL_PLL0_POSTDIV2(x) 		(((x) & 0x7) << 23)
#define SCTRL_PLL0_POSTDIV1(x) 		(((x) & 0x7) << 20)
#define SCTRL_PLL0_FBDIV(x) 		(((x) & 0xfff) << 8)
#define SCTRL_PLL0_REFDIV(x) 		(((x) & 0x3f) << 2)
#define SCTRL_PLL0_EN 			(1 << 0)

#define SCTRL_PLL_CTRL1_OFFSET 		0x104
#define SCTRL_PLL0_CLK_NO_GATE 		(1 << 26)
#define SCTRL_PLL0_CFG_VLD 		(1 << 25)
#define SCTRL_PLL0_FRACDIV(x) 		((x) & 0xFFFFFF)

#define SCTRL_PLL_STAT_OFFSET 		0x10C
#define SCTRL_PLL0_STAT 		(1 << 0)

#define SCTRL_PERI_EN0_OFFSET 		0x160
#define SCTRL_PERI_DIS0_OFFSET 		0x164
#define SCTRL_PERI_STAT0_OFFSET 	0x168

#define SCTRL_SCPEREN1_OFFSET 		0x170
#define SCTRL_SCPERDIS1_OFFSET 		0x174
#define SCPEREN_GT_PCLK_MMBUFCFG 	(1 << 25)
#define SCPEREN_GT_PCLK_MMBUF 		(1 << 23)
#define SCPEREN_GT_ACLK_MMBUF 		(1 << 22)
#define SCPEREN_GT_CLK_NOC_AOBUS2MMBUF 	(1 << 6)

#define SCTRL_SCPERRSTEN1_OFFSET 	0x20C
#define SCTRL_SCPERRSTDIS1_OFFSET 	0x210
#define SCTRL_SCPERRSTSTAT1_OFFSET 	0x214
#define IP_RST_MMBUFCFG 		(1 << 12)
#define IP_RST_MMBUF 			(1 << 11)

#define SCTRL_SCPERRSTEN2_OFFSET 	0x218
#define SCTRL_SCPERRSTDIS2_OFFSET 	0x21C
#define SCTRL_SCPERRSTSTAT2_OFFSET 	0x220

#define SCTRL_SCCLKDIV2_OFFSET 		0x258
#define SEL_CLK_MMBUF_MASK 		(0x3 << 8)
#define SEL_CLK_MMBUF_PLL0 		(0x3 << 8)
#define SCCLKDIV2_GT_PCLK_MMBUF 	(1 << 7)

#define SCTRL_SCCLKDIV4_OFFSET 		0x260
#define GT_MMBUF_SYS 			(1 << 13)
#define GT_MMBUF_FLL 			(1 << 12)
#define GT_PLL_CLK_MMBUF 		(1 << 11)

#define SCTRL_SCINNERSTAT 		0x3A0
#define EMMC_UFS_SEL 			(1 << 15)

#define SCTRL_BAK_DATA0_OFFSET 		0x40C

#define SCTRL_LPMCU_CLKEN_OFFSET 	0x480
#define SCTRL_LPMCU_CLKDIS_OFFSET 	0x484
#define SCTRL_LPMCU_RSTEN_OFFSET 	0x500
#define SCTRL_LPMCU_RSTDIS_OFFSET 	0x504
#define DDRC_SOFT_BIT 			(1 << 6)
#define DDRC_CLK_BIT 			(1 << 5)

#define SCTRL_SCPEREN0_SEC_OFFSET 	0x900
#define SCTRL_SCPERDIS0_SEC_OFFSET 	0x904
#define MMBUF_SEC_CTRL_MASK 		(0xfff << 20)
#define MMBUF_SEC_CTRL(x) 		(((x) & 0xfff) << 20)

#define SCTRL_PERRSTEN1_SEC_OFFSET 	0xA50
#define SCTRL_PERRSTDIS1_SEC_OFFSET 	0xA54
#define SCTRL_PERRSTSTAT1_SEC_OFFSET 	0xA58
#define RST_ASP_SUBSYS_BIT 		(1 << 0)

#define SCTRL_PERRSTEN2_SEC_OFFSET 	0xB50
#define SCTRL_PERRSTDIS2_SEC_OFFSET 	0xB54
#define SCTRL_PERRSTSTAT2_SEC_OFFSET 	0xB58

#define SCTRL_HISEECLKDIV_OFFSET 	0xC28
#define SC_SEL_HISEE_PLL_MASK 		(1 << 4)
#define SC_SEL_HISEE_PLL0 		(1 << 4)
#define SC_SEL_HISEE_PLL2 		(0 << 4)
#define SC_DIV_HISEE_PLL_MASK 		(7 << 16)
#define SC_DIV_HISEE_PLL(x) 		((x) & 0x7)

#define PMC_REG_BASE 			0xFFF31000
#define PMC_PPLL1_CTRL0_OFFSET 		0x038
#define PMC_PPLL1_CTRL1_OFFSET 		0x03C
#define PMC_PPLL2_CTRL0_OFFSET 		0x040
#define PMC_PPLL2_CTRL1_OFFSET 		0x044
#define PMC_PPLL3_CTRL0_OFFSET 		0x048
#define PMC_PPLL3_CTRL1_OFFSET 		0x04C
#define PPLLx_LOCK 			(1 << 26)
#define PPLLx_WITHOUT_CLK_GATE 		(1 << 26)
#define PPLLx_CFG_VLD 			(1 << 25)
#define PPLLx_INT_MOD 			(1 << 24)
#define PPLLx_POSTDIV2_MASK 		(0x7 << 23)
#define PPLLx_POSTDIV2(x) 		(((x) & 0x7) << 23)
#define PPLLx_POSTDIV1_MASK 		(0x7 << 20)
#define PPLLx_POSTDIV1(x) 		(((x) & 0x7) << 20)
#define PPLLx_FRACDIV_MASK 		(0x00FFFFFF)
#define PPLLx_FRACDIV(x) 		((x) & 0x00FFFFFF)
#define PPLLx_FBDIV_MASK 		(0xfff << 8)
#define PPLLx_FBDIV(x) 			(((x) & 0xfff) << 8)
#define PPLLx_REFDIV_MASK 		(0x3f << 2)
#define PPLLx_REFDIV(x) 		(((x) & 0x3f) << 2)
#define PPLLx_BP 			(1 << 1)
#define PPLLx_EN 			(1 << 0)

#define PMC_DDRLP_CTRL_OFFSET 		0x30C
#define DDRC_CSYSREQ_CFG(x) 		((x) & 0xF)

#define PMC_NOC_POWER_IDLEREQ 		0x380
#define DDRPHY_BYPASS_MODE 		(1 << 0)


#define PMU_SSI0_REG_BASE 		0xFFF34000

#define PMU_SSI0_LDO8_CTRL0_OFFSET 	(0x68 << 2)
#define LDO8_CTRL0_EN_1_8V 		0x02

#define PMU_SSI0_CLK_TOP_CTRL7_OFFSET 	(0x10C << 2)
#define NP_XO_ABB_DIG 			(1 << 1)

#define LP_CONFIG_REG_BASE 		0xFFF3F000

#define IOMG_REG_BASE 			0xE896C000

#define IOMG_UART5_RX_OFFSET 		0x0BC
#define IOMG_UART5_TX_OFFSET 		0x0C0

#define TIMER9_REG_BASE 		0xE8A00000

#define PCTRL_REG_BASE 			0xE8A09000
#define PCTRL_PERI_CTRL24_OFFSET 	0x064

#define MMBUF_BASE 			0xEA800000

#define UART5_REG_BASE 			0xFDF05000

#define USB3OTG_REG_BASE 		0xFF100000

#define UFS_REG_BASE 			0xFF3B0000

#define UFS_SYS_REG_BASE 		0xFF3B1000

#endif  /* __HI3660_H__ */
