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

#ifndef __FASTBOOT_H__
#define __FASTBOOT_H__

#include <ctype.h>
#include <errno.h>
#include <io_storage.h>
#include <partition.h>
#include <stdlib.h>
#include <string.h>
#include <types.h>

#define FASTBOOT_RESPONSE_LENGTH			64

#define FASTBOOT_COMMAND_BOOT				"boot"
#define FASTBOOT_COMMAND_CONTINUE			"continue"
#define FASTBOOT_COMMAND_DOWNLOAD			"download:"
#define FASTBOOT_COMMAND_ERASE				"erase:"
#define FASTBOOT_COMMAND_FLASH				"flash:"
#define FASTBOOT_COMMAND_GETVAR				"getvar:"
#define FASTBOOT_COMMAND_OEM				"oem "
#define FASTBOOT_COMMAND_REBOOT				"reboot"

#define FASTBOOT_VAR_MAX_DOWNLOAD_SIZE			"max-download-size"
#define FASTBOOT_VAR_PARTITION_SIZE			"partition-size:"
#define FASTBOOT_VAR_PARTITION_TYPE			"partition-type:"
#define FASTBOOT_VAR_PRODUCT				"product"
#define FASTBOOT_VAR_SERIALNO				"serialno"
#define FASTBOOT_VAR_VERSION				"version"

#define FASTBOOT_DATA_MASK				((128 << 20) - 1)

/*
 * base: address of download buffer in fastboot driver
 * size: size of download buffer in fastboot driver
 * image_id: id of io storage device
 */
typedef struct fastboot_params {
	uint64_t	base;
	uint64_t	size;
	unsigned int	image_id;
} fastboot_params_t;

typedef struct fastboot_ops {
	int		(*read)(size_t *size);
	int		(*write)(char *buf, size_t size);
} fastboot_ops_t;

int fastboot_run(fastboot_params_t *params);

int fastboot_device_is_attached(void);
void fastboot_device_handle_interrupts(void);
void fastboot_set_var(const char *name, const char *value,
		      int (*execute)(const char *arg, int left,
				     char *response));
void fastboot_register_command(const char *name,
			       int (*callback)(const char *arg));

extern void fb_download(char *cmdbuf);
extern void fb_flash(char *cmdbuf);
extern void fb_reboot(char *cmdbuf);
extern void fb_getvar(char *cmdbuf);
extern void rx_cmd(void);
extern void rx_data(void);
extern int fastboot_handle_command(void *buf, int length);
extern void fastboot_init(const fastboot_ops_t *ops_ptr);

extern void debug_rx_info(uintptr_t buf, unsigned long len);
extern int debug_rxdata_complete(void);
extern unsigned long rx_addr;
extern unsigned long rx_length;
extern int rx_data_complete;

#endif /* __FASTBOOT_H__ */
