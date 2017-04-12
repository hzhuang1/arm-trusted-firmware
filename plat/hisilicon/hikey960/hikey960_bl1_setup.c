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
#include <console.h>
#include <debug.h>
#include <delay_timer.h>
#include <dw_ufs.h>
#include <errno.h>
#include <gicv2.h>
#include <hi3660.h>
#include <mmio.h>
#include <generic_delay_timer.h>
#include <platform.h>
#include <platform_def.h>
#include <string.h>
#include <tbbr/tbbr_img_desc.h>
#include <ufs.h>

#include "../../bl1/bl1_private.h"
#include "hikey960_def.h"
#include "hikey960_private.h"

enum {
	BOOT_MODE_RECOVERY = 0,
	BOOT_MODE_NORMAL,
	BOOT_MODE_MASK = 1,
};

/*
 * Declarations of linker defined symbols which will help us find the layout
 * of trusted RAM
 */
extern unsigned long __COHERENT_RAM_START__;
extern unsigned long __COHERENT_RAM_END__;

/*
 * The next 2 constants identify the extents of the coherent memory region.
 * These addresses are used by the MMU setup code and therefore they must be
 * page-aligned.  It is the responsibility of the linker script to ensure that
 * __COHERENT_RAM_START__ and __COHERENT_RAM_END__ linker symbols refer to
 * page-aligned addresses.
 */
#define BL1_COHERENT_RAM_BASE (unsigned long)(&__COHERENT_RAM_START__)
#define BL1_COHERENT_RAM_LIMIT (unsigned long)(&__COHERENT_RAM_END__)

/* Data structure which holds the extents of the trusted RAM for BL1 */
static meminfo_t bl1_tzram_layout;

/******************************************************************************
 * On a GICv2 system, the Group 1 secure interrupts are treated as Group 0
 * interrupts.
 *****************************************************************************/
const unsigned int g0_interrupt_array[] = {
	IRQ_SEC_PHY_TIMER,
	IRQ_SEC_SGI_0
};

const gicv2_driver_data_t hikey960_gic_data = {
	.gicd_base = GICD_REG_BASE,
	.gicc_base = GICC_REG_BASE,
	.g0_interrupt_num = ARRAY_SIZE(g0_interrupt_array),
	.g0_interrupt_array = g0_interrupt_array,
};

meminfo_t *bl1_plat_sec_mem_layout(void)
{
	return &bl1_tzram_layout;
}

/*
 * Perform any BL1 specific platform actions.
 */
void bl1_early_platform_setup(void)
{
	const size_t bl1_size = BL1_RAM_LIMIT - BL1_RAM_BASE;
	unsigned int id, uart_base;

	generic_delay_timer_init();
	hikey960_read_boardid(&id);
	if (id == 5300)
		uart_base = PL011_UART5_BASE;
	else
		uart_base = PL011_UART6_BASE;
	/* Initialize the console to provide early debug support */
	console_init(uart_base, PL011_UART_CLK_IN_HZ, PL011_BAUDRATE);

	/* Allow BL1 to see the whole Trusted RAM */
	bl1_tzram_layout.total_base = BL1_RW_BASE;
	bl1_tzram_layout.total_size = BL1_RW_SIZE;

	/* Calculate how much RAM BL1 is using and how much remains free */
	bl1_tzram_layout.free_base = BL1_RW_BASE;
	bl1_tzram_layout.free_size = BL1_RW_SIZE;
	reserve_mem(&bl1_tzram_layout.free_base,
		    &bl1_tzram_layout.free_size,
		    BL1_RAM_BASE,
		    bl1_size);

	INFO("BL1: 0x%lx - 0x%lx [size = %lu]\n", BL1_RAM_BASE, BL1_RAM_LIMIT,
	     bl1_size);
}

/*
 * Perform the very early platform specific architecture setup here. At the
 * moment this only does basic initialization. Later architectural setup
 * (bl1_arch_setup()) does not do anything platform specific.
 */
void bl1_plat_arch_setup(void)
{
	hikey960_init_mmu_el3(bl1_tzram_layout.total_base,
			      bl1_tzram_layout.total_size,
			      BL1_RO_BASE,
			      BL1_RO_LIMIT,
			      BL1_COHERENT_RAM_BASE,
			      BL1_COHERENT_RAM_LIMIT);
}

static void hikey960_clk_init(void)
{
	//mmio_write_32(CRG_REG_BASE + CRG_CLKDIV3_OFFSET, 0xf0001000);

	/* change ldi0 sel to ppll2 */
	mmio_write_32(0xfff350b4, 0xf0002000);
	/* ldi0 20' */
	mmio_write_32(0xfff350bc, 0xfc004c00);
}

static void hikey960_pmu_init(void)
{
	/* clear np_xo_abb_dig_START bit in PMIC_CLK_TOP_CTRL7 register */
	mmio_clrbits_32(PMU_SSI0_REG_BASE + PMU_SSI0_CLK_TOP_CTRL7_OFFSET,
			NP_XO_ABB_DIG);
}

static void hikey960_enable_ppll3(void)
{
	/* enable ppll3 */
	mmio_write_32(0xfff31048, 0x4904305);
	mmio_write_32(0xfff3104c, 0x2300000);
	mmio_write_32(0xfff3104c, 0x6300000);
}

static void bus_idle_clear(unsigned int value)
{
	unsigned int pmc_value, pmc_value1, pmc_value2;
	int timeout = 100;

	pmc_value = value << 16;
	pmc_value &= ~value;
	mmio_write_32(0xfff31380, pmc_value);

	for (;;) {
		pmc_value1 = (unsigned int)mmio_read_32(0xfff31384);
		pmc_value2 = (unsigned int)mmio_read_32(0xfff31388);
		if (((pmc_value1 & value) == 0) && ((pmc_value2 & value) == 0))
			break;
		udelay(1);
		timeout--;
		if (timeout <= 0) {
			WARN("%s timeout\n", __func__);
			break;
		}
	}
}

static void set_vivobus_power_up(void)
{
	/* clk enable */
	mmio_write_32(0xfff350f8, 0x00020002);
	mmio_write_32(0xfff35000, 0x00001000);
}

static void set_dss_power_up(void)
{
	/* set edc0 133MHz = 1600MHz / 12 */
	mmio_write_32(0xfff350bc, 0x003f000b);
	/* mtcmos on */
	mmio_write_32(0xfff35150, 0x00000020);
	udelay(100);
	/* DISP CRG */
	mmio_write_32(0xfff35094, 0x00000010);
	/* clk enable */
	mmio_write_32(0xfff350f0, 0x01400140);
	mmio_write_32(0xfff35000, 0x00002000);
	mmio_write_32(0xfff35030, 0x0003b000);
	udelay(1);
	/* clk disable */
	mmio_write_32(0xfff35034, 0x0003b000);
	mmio_write_32(0xfff35004, 0x00002000);
	mmio_write_32(0xfff350f0, 0x01400000);
	udelay(1);
	/* iso disable */
	mmio_write_32(0xfff35148, 0x00000040);
	/* unreset */
	mmio_write_32(0xfff35094, 0x00000006);
	mmio_write_32(0xfff35088, 0x00000c00);
	/* clk enable */
	mmio_write_32(0xfff350f0, 0x01400140);
	mmio_write_32(0xfff35000, 0x00002000);
	mmio_write_32(0xfff35030, 0x0003b000);
	/* bus idle clear */
	bus_idle_clear(1 << 13);
	/* set edc0 400MHz for 2K 1600MHz / 4 */
	mmio_write_32(0xfff350bc, 0x003f0003);
}

static void set_vcodec_power_up(void)
{
	/* clk enable */
	mmio_write_32(0xfff350f8, 0x00040004);
	mmio_write_32(0xfff35000, 0x00000060);
	mmio_write_32(0xfff35020, 0x10000000);
	/* unreset */
	mmio_write_32(0xfff35064, 0x00000018);
	/* bus idle clear */
	bus_idle_clear(1 << 4);
}

static void set_vdec_power_up(void)
{
	/* mtcmos on */
	mmio_write_32(0xfff35150, 0x00000004);
	udelay(100);
	/* clk enable */
	mmio_write_32(0xfff350f0, 0x80008000);
	mmio_write_32(0xfff35020, 0x20080000);
	mmio_write_32(0xfff35030, 0x00000800);
	udelay(1);
	/* clk disable */
	mmio_write_32(0xfff35034, 0x00000800);
	mmio_write_32(0xfff35024, 0x20080000);
	mmio_write_32(0xfff350f0, 0x80000000);
	udelay(1);
	/* iso disable */
	mmio_write_32(0xfff35148, 0x00000004);
	/* unreset */
	mmio_write_32(0xfff35088, 0x00000200);
	/* clk enable */
	mmio_write_32(0xfff350f0, 0x80008000);
	mmio_write_32(0xfff35020, 0x20080000);
	mmio_write_32(0xfff35030, 0x00000800);
	/* bus idle clear */
	bus_idle_clear(1 << 10);
}

static void set_venc_power_up(void)
{
	/* set venc ppll3 */
	mmio_write_32(0xfff350c8, 0x18001000);
	/* set venc 258MHz, 1290MHz / 5 */
	mmio_write_32(0xfff350c8, 0x07c00100);
	/* mtcmos on */
	mmio_write_32(0xfff35150, 0x00000002);
	udelay(100);
	/* clk enable */
	mmio_write_32(0xfff350f4, 0x00010001);
	mmio_write_32(0xfff35020, 0x40000100);
	mmio_write_32(0xfff35030, 0x00000400);
	udelay(1);
	/* clk disable */
	mmio_write_32(0xfff35034, 0x00000400);
	mmio_write_32(0xfff35024, 0x40000100);
	mmio_write_32(0xfff350f4, 0x00010000);
	udelay(1);
	/* iso disable */
	/* unreset */
	mmio_write_32(0xfff35088, 0x00000100);
	/* clk enable */
	mmio_write_32(0xfff350f4, 0x00010001);
	mmio_write_32(0xfff35020, 0x40000100);
	mmio_write_32(0xfff35030, 0x00000400);
	/* bus idle clear */
	bus_idle_clear(1 << 11);
	/* set venc 645MHz, 1290MHz / 2 */
	mmio_write_32(0xfff350c8, 0x07c00040);
}

static void set_isp_power_up(void)
{
	/* mtcmos on */
	mmio_write_32(0xfff35150, 0x00000001);
	udelay(100);
	/* clk enable */
	mmio_write_32(0xfff350f0, 0x70007000);
	mmio_write_32(0xfff350f8, 0x00100010);
	mmio_write_32(0xfff35050, 0x01000010);
	mmio_write_32(0xfff35030, 0x0bf00000);
	udelay(1);
	/* clk disable */
	mmio_write_32(0xfff35054, 0x01000010);
	mmio_write_32(0xfff35034, 0x0bf00000);
	mmio_write_32(0xfff350f0, 0x70000000);
	mmio_write_32(0xfff350f8, 0x00100000);
	udelay(1);
	/* iso disable */
	mmio_write_32(0xfff35148, 0x00000001);
	/* unreset */
	mmio_write_32(0xfff35c84, 0x0000002f);
	/* clk enable */
	mmio_write_32(0xfff350f0, 0x70007000);
	mmio_write_32(0xfff350f8, 0x00100010);
	mmio_write_32(0xfff35050, 0x01000010);
	mmio_write_32(0xfff35030, 0x0bf00000);
	/* bus idle clear */
	bus_idle_clear(1 << 5);
	/* csi clk enable */
	mmio_write_32(0xfff35030, 0x00700000);
}

static void set_ivp_power_up(void)
{
	/* set ivp ppll0 */
	mmio_write_32(0xfff350a8, 0xc0000000);
	/* set ivp 267MHz, 1600MHz / 6 */
	mmio_write_32(0xfff350a8, 0x3c001400);
	/* mtcmos on */
	mmio_write_32(0xfff35150, 0x00200000);
	udelay(100);
	/* IVP CRG unreset */
	mmio_write_32(0xfff35c04, 0x00000001);
	/* clk enable */
	mmio_write_32(0xfff350f8, 0x02000200);
	mmio_write_32(0xfff35040, 0x000000a8);
	udelay(1);
	/* clk disable */
	mmio_write_32(0xfff35044, 0x000000a8);
	mmio_write_32(0xfff350f8, 0x02000000);
	udelay(1);
	/* iso disable */
	mmio_write_32(0xfff35148, 0x01000000);
	/* unreset */
	mmio_write_32(0xfff35c04, 0x00000002);
	/* clk enable */
	mmio_write_32(0xfff350f8, 0x02000200);
	mmio_write_32(0xfff35040, 0x000000a8);
	/* bus idle clear */
	bus_idle_clear(1 << 14);
	/* set ivp 533MHz, 1600MHz / 3 */
	mmio_write_32(0xfff350a8, 0x3c000800);
}

static void set_audio_power_up(void)
{
	unsigned int ret;
	int timeout = 100;
	/* mtcmos on */
	mmio_write_32(0xfff0a060, 0x00000001);
	udelay(100);
	/* clk enable */
	mmio_write_32(0xfff350f4, 0x80108010);
	mmio_write_32(0xfff0a258, 0x00010001);
	mmio_write_32(0xfff0a160, 0x0c000000);
	mmio_write_32(0xfff35000, 0x04000000);
	mmio_write_32(0xfff35050, 0x00000080);
	mmio_write_32(0xfff0a170, 0x0000000f);
	udelay(1);
	/* clk disable */
	mmio_write_32(0xfff0a174, 0x0000000f);
	mmio_write_32(0xfff0a164, 0x0c000000);
	mmio_write_32(0xfff35054, 0x00000080);
	mmio_write_32(0xfff35004, 0x04000000);
	mmio_write_32(0xfff0a258, 0x00010000);
	mmio_write_32(0xfff350f4, 0x80100000);
	udelay(1);
	/* iso disable */
	mmio_write_32(0xfff0a044, 0x00000001);
	udelay(1);
	/* unreset */
	mmio_write_32(0xfff0aa54, 0x00000001);
	mmio_write_32(0xfff0a204, 0x00000780);
	/* clk enable */
	mmio_write_32(0xfff350f4, 0x80108010);
	mmio_write_32(0xfff0a258, 0x00010001);
	mmio_write_32(0xfff0a160, 0x0c000000);
	mmio_write_32(0xfff35000, 0x04000000);
	mmio_write_32(0xfff35050, 0x00000080);
	mmio_write_32(0xfff0a170, 0x0000000f);
	/* bus idle clear */
	mmio_write_32(0xfff0a31c, 0x00040000);
	for (;;) {
		ret = mmio_read_32(0xfff0a378);
		if (((ret & (1 << 5)) == 0) && ((ret & (1 << 8)) == 0))
			break;
		udelay(1);
		timeout--;
		if (timeout <= 0) {
			WARN("%s timeout\n", __func__);
			break;
		}
	}
	mmio_write_32(0xe804e148, 0x00ff0000);
}

static void set_pcie_power_up(void)
{
	/* mtcmos on */
	mmio_write_32(0xfff0a060, 0x00000010);
	udelay(100);
	/* clk enable */
	mmio_write_32(0xfff0a268, 0x08000800);
	mmio_write_32(0xfff0a190, 0x00104000);
	mmio_write_32(0xfff35420, 0x000003a0);
	udelay(1);
	/* clk disable */
	mmio_write_32(0xfff0a194, 0x00104000);
	mmio_write_32(0xfff35424, 0x000003a0);
	mmio_write_32(0xfff0a268, 0x08000000);
	udelay(1);
	/* iso disable */
	mmio_write_32(0xfff0a044, 0x00000030);
	/* unreset */
	mmio_write_32(0xfff35088, 0x8c000000);
	/* clk enable */
	mmio_write_32(0xfff0a268, 0x08000800);
	mmio_write_32(0xfff0a190, 0x00104000);
	mmio_write_32(0xfff35420, 0x000003a0);
}

static void ispfunc_enable(void)
{
	/* enable ispfunc. Otherwise powerup isp_srt causes exception. */
	mmio_write_32(0xfff35000, 0x00000008);
	mmio_write_32(0xfff35460, 0xc004ffff);
	mmio_write_32(0xfff35030, 0x02000000);
	udelay(10000);
}

static void isps_control_clock(int flag)
{
	unsigned int ret;

	/* flag: 0 -- disable clock, 1 -- enable clock */
	if (flag) {
		ret = mmio_read_32(0xe8420364);
		ret |= 1;
		mmio_write_32(0xe8420364, ret);
	} else {
		ret = mmio_read_32(0xe8420364);
		ret &= ~1;
		mmio_write_32(0xe8420364, ret);
	}
}

static void set_isp_srt_power_up(void)
{
	unsigned int ret;

	ispfunc_enable();
	/* reset */
	mmio_write_32(0xe8420374, 0x00000001);
	mmio_write_32(0xe8420350, 0x00000000);
	mmio_write_32(0xe8420358, 0x00000000);
	/* mtcmos on */
	mmio_write_32(0xfff35150, 0x00400000);
	udelay(100);
	/* clk enable */
	isps_control_clock(1);
	udelay(1);
	isps_control_clock(0);
	udelay(1);
	/* iso disable */
	mmio_write_32(0xfff35148, 0x08000000);
	/* unreset */
	ret = mmio_read_32(0xe8420374);
	ret &= ~0x1;
	mmio_write_32(0xe8420374, ret);
	/* clk enable */
	isps_control_clock(1);
	/* enable clock gating for accessing csi registers */
	mmio_write_32(0xe8420010, ~0);
}

static void hikey960_regulator_enable(void)
{
	set_vivobus_power_up();
	hikey960_enable_ppll3();
	set_dss_power_up();
	set_vcodec_power_up();
	set_vdec_power_up();
	set_venc_power_up();
	set_isp_power_up();
	set_ivp_power_up();
	set_audio_power_up();
	set_pcie_power_up();
	set_isp_srt_power_up();
}

static void hikey960_ufs_reset(void)
{
	unsigned int data, mask;

	mmio_write_32(CRG_REG_BASE + CRG_PERRSTEN3_OFFSET, PERI_UFS_BIT);
	do {
		data = mmio_read_32(CRG_REG_BASE + CRG_PERRSTSTAT3_OFFSET);
	} while ((data & PERI_UFS_BIT) == 0);
	mmio_setbits_32(UFS_SYS_REG_BASE + UFS_SYS_PSW_POWER_CTRL_OFFSET,
			BIT_UFS_PSW_MTCMOS_EN);
	mdelay(1);
	mmio_setbits_32(UFS_SYS_REG_BASE + UFS_SYS_HC_LP_CTRL_OFFSET,
			BIT_SYSCTRL_PWR_READY);
	mmio_write_32(UFS_SYS_REG_BASE + UFS_SYS_UFS_DEVICE_RESET_CTRL_OFFSET,
		      MASK_UFS_DEVICE_RESET);
	mask = SC_DIV_UFS_PERIBUS << 16;
	mmio_write_32(CRG_REG_BASE + CRG_CLKDIV17_OFFSET, mask);
	mask = SC_DIV_UFSPHY_CFG_MASK << 16;
	data = SC_DIV_UFSPHY_CFG(3);
	mmio_write_32(CRG_REG_BASE + CRG_CLKDIV16_OFFSET, mask | data);
	data = mmio_read_32(UFS_SYS_REG_BASE + UFS_SYS_PHY_CLK_CTRL_OFFSET);
	data &= ~MASK_SYSCTRL_CFG_CLOCK_FREQ;
	data |= 0x39;
	mmio_write_32(UFS_SYS_REG_BASE + UFS_SYS_PHY_CLK_CTRL_OFFSET, data);
	mmio_clrbits_32(UFS_SYS_REG_BASE + UFS_SYS_PHY_CLK_CTRL_OFFSET,
			MASK_SYSCTRL_REF_CLOCK_SEL);

	mmio_setbits_32(UFS_SYS_REG_BASE + UFS_SYS_PSW_CLK_CTRL_OFFSET,
			BIT_SYSCTRL_PSW_CLK_EN);
	mmio_clrbits_32(UFS_SYS_REG_BASE + UFS_SYS_PSW_POWER_CTRL_OFFSET,
			BIT_UFS_PSW_ISO_CTRL);
	mmio_clrbits_32(UFS_SYS_REG_BASE + UFS_SYS_PHY_ISO_EN_OFFSET,
			BIT_UFS_PHY_ISO_CTRL);
	mmio_clrbits_32(UFS_SYS_REG_BASE + UFS_SYS_HC_LP_CTRL_OFFSET,
			BIT_SYSCTRL_LP_ISOL_EN);
	mmio_write_32(CRG_REG_BASE + CRG_PERRSTDIS3_OFFSET, PERI_ARST_UFS_BIT);
	mmio_setbits_32(UFS_SYS_REG_BASE + UFS_SYS_RESET_CTRL_EN_OFFSET,
			BIT_SYSCTRL_LP_RESET_N);
	mdelay(1);
	mmio_write_32(UFS_SYS_REG_BASE + UFS_SYS_UFS_DEVICE_RESET_CTRL_OFFSET,
		      MASK_UFS_DEVICE_RESET | BIT_UFS_DEVICE_RESET);
	mdelay(20);
	mmio_write_32(UFS_SYS_REG_BASE + UFS_SYS_UFS_DEVICE_RESET_CTRL_OFFSET,
		      0x03300330);

	mmio_write_32(CRG_REG_BASE + CRG_PERRSTDIS3_OFFSET, PERI_UFS_BIT);
	do {
		data = mmio_read_32(CRG_REG_BASE + CRG_PERRSTSTAT3_OFFSET);
	} while (data & PERI_UFS_BIT);
}

static void hikey960_ufs_init(void)
{
	dw_ufs_params_t ufs_params;

	memset(&ufs_params, 0, sizeof(ufs_params));
	ufs_params.reg_base = UFS_REG_BASE;
	ufs_params.desc_base = HIKEY960_UFS_DESC_BASE;
	ufs_params.desc_size = HIKEY960_UFS_DESC_SIZE;
	//ufs_params.flags = UFS_FLAGS_SKIPINIT;

	if ((ufs_params.flags & UFS_FLAGS_SKIPINIT) == 0)
		hikey960_ufs_reset();
	dw_ufs_init(&ufs_params);
}

static void hikey960_tzc_init(void)
{
	mmio_write_32(TZC_REG_BASE + TZC_EN0_OFFSET, ~0);
	mmio_write_32(TZC_REG_BASE + TZC_EN1_OFFSET, ~0);
	mmio_write_32(TZC_REG_BASE + TZC_EN2_OFFSET, ~0);
	mmio_write_32(TZC_REG_BASE + TZC_EN3_OFFSET, ~0);
	mmio_write_32(TZC_REG_BASE + TZC_EN4_OFFSET, ~0);
	mmio_write_32(TZC_REG_BASE + TZC_EN5_OFFSET, ~0);
	mmio_write_32(TZC_REG_BASE + TZC_EN6_OFFSET, ~0);
	mmio_write_32(TZC_REG_BASE + TZC_EN7_OFFSET, ~0);
	mmio_write_32(TZC_REG_BASE + TZC_EN8_OFFSET, ~0);
}

/*
 * Function which will perform any remaining platform-specific setup that can
 * occur after the MMU and data cache have been enabled.
 */
void bl1_platform_setup(void)
{
	hikey960_clk_init();
	hikey960_pmu_init();
	hikey960_regulator_enable();
	hikey960_tzc_init();
	hikey960_ufs_init();
	hikey960_io_setup();
}

/*
 * The following function checks if Firmware update is needed,
 * by checking if TOC in FIP image is valid or not.
 */
unsigned int bl1_plat_get_next_image_id(void)
{
	unsigned int mode, ret;

	mode = mmio_read_32(SCTRL_REG_BASE + SCTRL_BAK_DATA0_OFFSET);
	switch (mode & BOOT_MODE_MASK) {
	case BOOT_MODE_RECOVERY:
#if 0
		ret = BL2U_IMAGE_ID;
#else
		ret = NS_BL1U_IMAGE_ID;
#endif
		break;
	case BOOT_MODE_NORMAL:
		ret = BL2_IMAGE_ID;
		break;
	default:
		WARN("Invalid boot mode is found:%d\n", mode);
		panic();
	}
	return ret;
}

image_desc_t *bl1_plat_get_image_desc(unsigned int image_id)
{
	unsigned int index = 0;

	while (bl1_tbbr_image_descs[index].image_id != INVALID_IMAGE_ID) {
		if (bl1_tbbr_image_descs[index].image_id == image_id) {
			return &bl1_tbbr_image_descs[index];
		}
		index++;
	}

	return NULL;
}

void bl1_plat_set_ep_info(unsigned int image_id,
		entry_point_info_t *ep_info)
{
	unsigned int data = 0;

	if (image_id == BL2_IMAGE_ID)
		return;
	/* Initialize the GIC driver, cpu and distributor interfaces */
	gicv2_driver_init(&hikey960_gic_data);
	gicv2_distif_init();
	gicv2_pcpu_distif_init();
	gicv2_cpuif_enable();
	/* CNTFRQ is read-only in EL1 */
	write_cntfrq_el0(plat_get_syscnt_freq2());
	__asm__ volatile ("mrs	%0, cpacr_el1" : "=r"(data));
	do {
		data |= 3 << 20;
		__asm__ volatile ("msr	cpacr_el1, %0" : : "r"(data));
		__asm__ volatile ("mrs	%0, cpacr_el1" : "=r"(data));
	} while ((data & (3 << 20)) != (3 << 20));
	INFO("cpacr_el1:0x%x\n", data);

	ep_info->args.arg0 = 0xffff & read_mpidr();
	ep_info->spsr = SPSR_64(MODE_EL1, MODE_SP_ELX,
				DISABLE_ALL_EXCEPTIONS);
}
