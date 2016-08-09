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
#include <dwc2.h>
#include <emmc.h>
#include <errno.h>
#include <fastboot.h>
#include <hi6220.h>
#include <partition.h>
#include <pl061_gpio.h>
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

static void hikey_gpio_init(void)
{
	pl061_gpio_init();
	pl061_gpio_register(GPIO0_BASE, 0);
	pl061_gpio_register(GPIO1_BASE, 1);
	pl061_gpio_register(GPIO2_BASE, 2);
	pl061_gpio_register(GPIO3_BASE, 3);
	pl061_gpio_register(GPIO4_BASE, 4);
	pl061_gpio_register(GPIO5_BASE, 5);
	pl061_gpio_register(GPIO6_BASE, 6);
	pl061_gpio_register(GPIO7_BASE, 7);
	pl061_gpio_register(GPIO8_BASE, 8);
	pl061_gpio_register(GPIO9_BASE, 9);
	pl061_gpio_register(GPIO10_BASE, 10);
	pl061_gpio_register(GPIO11_BASE, 11);
	pl061_gpio_register(GPIO12_BASE, 12);
	pl061_gpio_register(GPIO13_BASE, 13);
	pl061_gpio_register(GPIO14_BASE, 14);
	pl061_gpio_register(GPIO15_BASE, 15);
	pl061_gpio_register(GPIO16_BASE, 16);
	pl061_gpio_register(GPIO17_BASE, 17);
	pl061_gpio_register(GPIO18_BASE, 18);
	pl061_gpio_register(GPIO19_BASE, 19);

	/* Power on indicator LED (USER_LED1). */
	gpio_set_direction(32, GPIO_DIR_OUT);	/* LED1 */
	gpio_set_value(32, GPIO_LEVEL_HIGH);
	gpio_set_direction(33, GPIO_DIR_OUT);	/* LED2 */
	gpio_set_value(33, GPIO_LEVEL_LOW);
	gpio_set_direction(34, GPIO_DIR_OUT);	/* LED3 */
	gpio_set_direction(35, GPIO_DIR_OUT);	/* LED4 */
}

void bl2u_platform_setup(void)
{
	dw_mmc_params_t dw_params;
	fastboot_params_t fb_params;
	partition_ops_t partition_ops;
	char response[16];

	sp804_timer_init(SP804_TIMER0_BASE, 10, 192);
	hikey_gpio_init();

	dw_params.reg_base = 0xf723d000;
	dw_params.desc_base = HIKEY_MMC_DESC_BASE;
	dw_params.desc_size = 1 << 20;
	dw_params.clk_rate = 24 * 1000 * 1000;
	dw_params.bus_width = EMMC_BUS_WIDTH_8;
	dw_params.cmd23 = 1;
	dw_mmc_init(&dw_params);
	hikey_io_setup();
	partition_ops.handler = hikey_partition_table_handler;
	partition_init(BL2U_IMAGE_ID, &partition_ops);

	hikey_init_serialno();
	fb_params.base = HIKEY_MMC_DATA_BASE;
	fb_params.size = HIKEY_MMC_DATA_SIZE;
	fb_params.image_id = BL2U_IMAGE_ID;
	dw_udc_init(&fb_params);
	/* Initiailize the fastboot variable "max-download-size". */
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
