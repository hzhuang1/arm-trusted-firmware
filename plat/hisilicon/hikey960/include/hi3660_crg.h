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
#ifndef __HI3660_CRG_H__
#define __HI3660_CRG_H__

#define CRG_REG_BASE 			0xFFF35000

#define CRG_PEREN0_OFFSET 		0x000
#define CRG_PERDIS0_OFFSET 		0x004
#define CRG_PERSTAT0_OFFSET 		0x008
#define PEREN0_GT_CLK_AOMM 		(1 << 31)

#define CRG_PEREN1_OFFSET 		0x010
#define CRG_PERDIS1_OFFSET 		0x014
#define CRG_PERSTAT1_OFFSET 		0x018
#define CRG_PEREN2_OFFSET 		0x020
#define CRG_PERDIS2_OFFSET 		0x024
#define CRG_PERSTAT2_OFFSET 		0x028
#define PEREN2_HKADCSSI			(1 << 24)

#define CRG_PEREN4_OFFSET 		0x040
#define CRG_PERDIS4_OFFSET 		0x044
#define CRG_PERCLKEN4_OFFSET 		0x048
#define CRG_PERSTAT4_OFFSET 		0x04C
#define GT_ACLK_USB3OTG 		(1 << 1)
#define GT_CLK_USB3OTG_REF 		(1 << 0)

#define CRG_PEREN5_OFFSET 		0x050
#define CRG_PERDIS5_OFFSET 		0x054
#define CRG_PERSTAT5_OFFSET 		0x058
#define CRG_PERRSTEN0_OFFSET 		0x060
#define CRG_PERRSTDIS0_OFFSET 		0x064
#define CRG_PERRSTSTAT0_OFFSET 		0x068
#define CRG_PERRSTEN1_OFFSET 		0x06C
#define CRG_PERRSTDIS1_OFFSET 		0x070
#define CRG_PERRSTSTAT1_OFFSET 		0x074
#define CRG_PERRSTEN2_OFFSET 		0x078
#define CRG_PERRSTDIS2_OFFSET 		0x07C
#define CRG_PERRSTSTAT2_OFFSET 		0x080
#define PERRSTEN2_HKADCSSI		(1 << 24)

#define CRG_PERRSTEN3_OFFSET 		0x084
#define CRG_PERRSTDIS3_OFFSET 		0x088
#define CRG_PERRSTSTAT3_OFFSET 		0x08C
#define CRG_PERRSTEN4_OFFSET 		0x090
#define CRG_PERRSTDIS4_OFFSET 		0x094
#define CRG_PERRSTSTAT4_OFFSET 		0x098
#define IP_RST_USB3OTG_MUX 		(1 << 8)
#define IP_RST_USB3OTG_AHBIF 		(1 << 7)
#define IP_RST_USB3OTG_32K 		(1 << 6)
#define IP_RST_USB3OTG 			(1 << 5)
#define IP_RST_USB3OTGPHY_POR 		(1 << 3)

#define CRG_PERRSTEN5_OFFSET 		0x09C
#define CRG_PERRSTDIS5_OFFSET 		0x0A0
#define CRG_PERRSTSTAT5_OFFSET 		0x0A4

/* bit fields in CRG_PERI */
#define PERI_PCLK_PCTRL_BIT 		(1 << 31)
#define PERI_TIMER12_BIT 		(1 << 25)
#define PERI_TIMER11_BIT 		(1 << 24)
#define PERI_TIMER10_BIT 		(1 << 23)
#define PERI_TIMER9_BIT 		(1 << 22)
#define PERI_UART5_BIT 			(1 << 15)
#define PERI_UFS_BIT 			(1 << 12)
#define PERI_ARST_UFS_BIT		(1 << 7)
#define PERI_PPLL2_EN_CPU 		(1 << 3)
#define PERI_PWM_BIT 			(1 << 0)
#define PERI_DDRC_BIT 			(1 << 0)
#define PERI_DDRC_D_BIT 		(1 << 4)
#define PERI_DDRC_C_BIT 		(1 << 3)
#define PERI_DDRC_B_BIT 		(1 << 2)
#define PERI_DDRC_A_BIT 		(1 << 1)
#define PERI_DDRC_DMUX_BIT 		(1 << 0)

#define CRG_CLKDIV0_OFFSET 		0x0A0
#define SC_DIV_LPMCU_MASK 		((0x1F << 5) << 16)
#define SC_DIV_LPMCU(x) 		(((x) & 0x1F) << 5)

#define CRG_CLKDIV1_OFFSET 		0x0B0
#define SEL_LPMCU_PLL_MASK 		((1 << 1) << 16)
#define SEL_SYSBUS_MASK 		((1 << 0) << 16)
#define SEL_LPMCU_PLL1 			(1 << 1)
#define SEL_LPMCU_PLL0 			(0 << 1)
#define SEL_SYSBUS_PLL0 		(1 << 0)
#define SEL_SYSBUS_PLL1 		(0 << 0)

#define CRG_CLKDIV3_OFFSET 		0x0B4

#define CRG_CLKDIV12_OFFSET 		0x0D8
#define SC_DIV_A53HPM_MASK 		(0x7 << 13)
#define SC_DIV_A53HPM(x) 		(((x) & 0x7) << 13)

#define CRG_CLKDIV16_OFFSET 		0x0E8
#define DDRC_CLK_SW_REQ_CFG_MASK 	(0x3 << 12)
#define DDRC_CLK_SW_REQ_CFG(x) 		(((x) & 0x3) << 12)
#define SC_DIV_UFSPHY_CFG_MASK		(0x3 << 9)
#define SC_DIV_UFSPHY_CFG(x)		(((x) & 0x3) << 9)
#define DDRCPLL_SW 			(1 << 8)

#define CRG_CLKDIV17_OFFSET		0x0EC
#define SC_DIV_UFS_PERIBUS		(1 << 14)

#define CRG_CLKDIV20_OFFSET 		0x0F8
#define CLKDIV20_GT_CLK_AOMM 		(1 << 3)

#define CRG_CLKDIV22_OFFSET 		0x100
#define SEL_PLL_320M_MASK 		(1 << 16)
#define SEL_PLL2_320M 			(1 << 0)
#define SEL_PLL0_320M 			(0 << 0)

#define CRG_CLKDIV23_OFFSET 		0x104
#define PERI_DDRC_SW_BIT 		(1 << 13)
#define DIV_CLK_DDRSYS_MASK 		(0x3 << 10)
#define DIV_CLK_DDRSYS(x) 		(((x) & 0x3) << 10)
#define GET_DIV_CLK_DDRSYS(x) 		((x & DIV_CLK_DDRSYS_MASK) >> 10)
#define DIV_CLK_DDRCFG_MASK 		(0x6 << 5)
#define DIV_CLK_DDRCFG(x) 		(((x) & 0x6) << 5)
#define GET_DIV_CLK_DDRCFG(x) 		((x) & DIV_CLK_DDRCFG_MASK) >> 5)
#define DIV_CLK_DDRC_MASK 		0x1F
#define DIV_CLK_DDRC(x) 		((x) & DIV_CLK_DDRC_MASK)
#define GET_DIV_CLK_DDRC(x) 		((x) & DIV_CLK_DDRC_MASK)

#define CRG_CLKDIV25_OFFSET 		0x10C
#define DIV_SYSBUS_PLL_MASK 		(0xF << 16)
#define DIV_SYSBUS_PLL(x) 		((x) & 0xF)

#define CRG_PERI_CTRL2_OFFSET 		0x128
#define PERI_TIME_STAMP_CLK_MASK 	(0x7 << 28)
#define PERI_TIME_STAMP_CLK_DIV(x) 	(((x) & 0x7) << 22)

#define CRG_PEREN8_OFFSET 		0x430
#define CRG_PERDIS8_OFFSET 		0x434
#define CRG_PERSTAT8_OFFSET 		0x438
#define PERI_DMC_D_BIT 			(1 << 22)
#define PERI_DMC_C_BIT 			(1 << 21)
#define PERI_DMC_B_BIT 			(1 << 20)
#define PERI_DMC_A_BIT 			(1 << 19)
#define PERI_DMC_BIT 			(1 << 18)

#define CRG_PEREN11_OFFSET 		0x460
#define PPLL1_GATE_CPU 			(1 << 18)

#define CRG_PERSTAT11_OFFSET 		0x46C
#define PPLL3_EN_STAT 			(1 << 21)
#define PPLL2_EN_STAT 			(1 << 20)
#define PPLL1_EN_STAT 			(1 << 19)

#define CRG_GENERAL_SEC_RSTEN_OFFSET 	0xE20
#define CRG_GENERAL_SEC_RSTDIS_OFFSET 	0xE24
#define IP_RST_GPIO0_SEC 		(1 << 2)

#define CRG_GENERAL_SEC_CLKDIV0_OFFSET 	0xE90
#define SC_DIV_AO_HISE_MASK 		3
#define SC_DIV_AO_HISE(x) 		((x) & 0x3)

#endif 	/* __HI3660_CRG_H__ */
