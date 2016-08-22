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

#include <assert.h>
#include <bl_common.h>
#include <console.h>
#include <debug.h>
#include <dw_mmc.h>
#include <dw_usb.h>
#include <emmc.h>
#include <errno.h>
#include <fastboot/fastboot.h>
#include <hi6220.h>
#include <mmio.h>
#include <partition/partition.h>
#include <sp804_delay_timer.h>

#include "hikey_def.h"
#include "hikey_private.h"

/*
 * The next 2 constants identify the extents of the code & RO data region.
 * These addresses are used by the MMU setup code and therefore they must be
 * page-aligned.  It is the responsibility of the linker script to ensure that
 * __RO_START__ and __RO_END__ linker symbols refer to page-aligned addresses.
 */
#define BL2U_RO_BASE (unsigned long)(&__RO_START__)
#define BL2U_RO_LIMIT (unsigned long)(&__RO_END__)

/*
 * The next 2 constants identify the extents of the coherent memory region.
 * These addresses are used by the MMU setup code and therefore they must be
 * page-aligned.  It is the responsibility of the linker script to ensure that
 * __COHERENT_RAM_START__ and __COHERENT_RAM_END__ linker symbols refer to
 * page-aligned addresses.
 */
#define BL2U_COHERENT_RAM_BASE (unsigned long)(&__COHERENT_RAM_START__)
#define BL2U_COHERENT_RAM_LIMIT (unsigned long)(&__COHERENT_RAM_END__)

#define PHY_EYE_PATTERN			0x70533483

void bl2u_early_platform_setup(void)
{
	/* Initialize the console to provide early debug support */
	console_init(CONSOLE_BASE, PL011_UART_CLK_IN_HZ, PL011_BAUDRATE);
}

void bl2u_plat_arch_setup(void)
{
	hikey_init_mmu_el1(BL2U_RO_LIMIT,
			   BL31_LIMIT - BL2U_RO_LIMIT,
			   BL2U_RO_BASE,
			   BL2U_RO_LIMIT,
			   BL2U_COHERENT_RAM_BASE,
			   BL2U_COHERENT_RAM_LIMIT);
}

static void hikey_usb_phy_init(void)
{
	unsigned int data;

	/* enable USB clock */
	mmio_write_32(PERI_SC_PERIPH_CLKEN0, PERI_CLK0_USBOTG);
	do {
		data = mmio_read_32(PERI_SC_PERIPH_CLKSTAT0);
	} while ((data & PERI_CLK0_USBOTG) == 0);

	/* out of reset */
	mmio_write_32(PERI_SC_PERIPH_RSTDIS0,
		      PERI_RST0_USBOTG_BUS | PERI_RST0_POR_PICOPHY |
		      PERI_RST0_USBOTG | PERI_RST0_USBOTG_32K);
	do {
		data = mmio_read_32(PERI_SC_PERIPH_RSTSTAT0);
		data &= PERI_RST0_USBOTG_BUS | PERI_RST0_POR_PICOPHY |
			PERI_RST0_USBOTG | PERI_RST0_USBOTG_32K;
	} while (data);

	mmio_write_32(PERI_SC_PERIPH_CTRL8, PHY_EYE_PATTERN);

	/* configure USB PHY */
	data = mmio_read_32(PERI_SC_PERIPH_CTRL4);
	/* make PHY out of low power mode */
	data &= ~PERI_CTRL4_PICO_SIDDQ;
	/* detect VBUS by external circuit, switch D+ to 1.5KOhm pullup */
	data |= PERI_CTRL4_PICO_VBUSVLDEXTSEL | PERI_CTRL4_PICO_VBUSVLDEXT;
	data &= ~PERI_CTRL4_FPGA_EXT_PHY_SEL;
	/* select PHY */
	data &= ~PERI_CTRL4_OTG_PHY_SEL;
	mmio_write_32(PERI_SC_PERIPH_CTRL4, data);

	udelay(1000);

	data = mmio_read_32(PERI_SC_PERIPH_CTRL5);
	data &= ~PERI_CTRL5_PICOPHY_BC_MODE;
	mmio_write_32(PERI_SC_PERIPH_CTRL5, data);
    
	udelay(20000);
}

void bl2u_platform_setup(void)
{
	dw_mmc_params_t emmc_params;
	dw_usb_params_t usb_params;
	fastboot_params_t fb_params;
	char response[16];

	sp804_timer_init(SP804_TIMER0_BASE, 10, 192);

	memset(&emmc_params, 0, sizeof(dw_mmc_params_t));
	emmc_params.reg_base = DWMMC0_BASE;
	emmc_params.desc_base = HIKEY_MMC_DESC_BASE;
	emmc_params.desc_size = HIKEY_MMC_DESC_SIZE;
	emmc_params.clk_rate = 24 * 1000 * 1000;
	emmc_params.bus_width = EMMC_BUS_WIDTH_8;
	dw_mmc_init(&emmc_params);
	hikey_io_setup();
	partition_init(BL2U_IMAGE_ID);

	hikey_usb_phy_init();
	memset(&usb_params, 0, sizeof(dw_usb_params_t));
	usb_params.reg_base = DWUSB_BASE;
	usb_params.desc.base = HIKEY_USB_DESC_BASE;
	usb_params.desc.size = HIKEY_USB_DESC_SIZE;
	usb_params.buffer.base = HIKEY_USB_DATA_BASE;
	usb_params.buffer.size = HIKEY_USB_DATA_SIZE;
	usb_params.serialno = hikey_init_serialno();
	assert(usb_params.serialno != NULL);
	dw_usb_init(&usb_params);

	/* Initiailize the fastboot variable "max-download-size". */
	fb_params.base = HIKEY_MMC_DATA_BASE;
	fb_params.size = HIKEY_MMC_DATA_SIZE;
	fb_params.image_id = BL2U_IMAGE_ID;
	sprintf(response, "0x%x", HIKEY_MMC_DATA_SIZE);
	fastboot_set_var(FASTBOOT_VAR_MAX_DOWNLOAD_SIZE, response, NULL);
	fastboot_set_var(FASTBOOT_VAR_VERSION, "1.0", NULL);
	fastboot_set_var(FASTBOOT_VAR_PRODUCT, "hikey", NULL);
	fastboot_set_var(FASTBOOT_VAR_PARTITION_TYPE, "",
			 hikey_get_partition_type);
	fastboot_set_var(FASTBOOT_VAR_PARTITION_SIZE, "",
			 hikey_get_partition_size);
	fastboot_register_command(FASTBOOT_COMMAND_REBOOT, hikey_reboot);
	fastboot_register_command(FASTBOOT_COMMAND_OEM, hikey_oem);
	fastboot_register_command(FASTBOOT_COMMAND_FLASH, hikey_flash);
	fastboot_register_command(FASTBOOT_COMMAND_ERASE, hikey_erase);
	fastboot_run(&fb_params);
}
