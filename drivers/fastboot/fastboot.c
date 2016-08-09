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
#include <debug.h>
#include <fastboot.h>
#include <platform.h>

#pragma weak fastboot_device_is_attached
#pragma weak fastboot_device_handle_interrupts

#define FASTBOOT_MAX_COMMAND_LENGTH			4096

typedef enum {
	CMD_INDEX_BOOT,
	CMD_INDEX_CONTINUE,
	CMD_INDEX_DOWNLOAD,
	CMD_INDEX_ERASE,
	CMD_INDEX_FLASH,
	CMD_INDEX_GETVAR,
	CMD_INDEX_OEM,
	CMD_INDEX_REBOOT,
	CMD_INDEX_INVALID
} command_index;

typedef enum {
	VAR_INDEX_MAX_DOWNLOAD_SIZE,
	VAR_INDEX_PARTITION_SIZE,
	VAR_INDEX_PARTITION_TYPE,
	VAR_INDEX_PRODUCT,
	VAR_INDEX_SERIALNO,
	VAR_INDEX_VERSION,
	VAR_INDEX_INVALID
} var_index;

typedef enum {
	STATE_OFFLINE,
	STATE_COMMAND,
	STATE_COMPLETE,
	STATE_ERROR
} protocol_state;

typedef struct fastboot_handle {
	uintptr_t	io_dev_handle;
	protocol_state	state;
	int		(*read)(uintptr_t buf, size_t *size);
	int		(*write)(uintptr_t buf, size_t size);
} fastboot_handle_t;

struct fastboot_cmd {
	const char	*cmd;
	void		(*execute)(fastboot_handle_t *handle, char *arg);
	int		(*callback)(const char *arg);
};

struct fastboot_var {
	const char	*name;
	const char	*value;
	int		(*execute)(const char *arg, int left, char *response);
};

static void cmd_boot(fastboot_handle_t *handle, char *arg);
static void cmd_continue(fastboot_handle_t *handle, char *arg);
static void cmd_download(fastboot_handle_t *handle, char *arg);
static void cmd_erase(fastboot_handle_t *handle, char *arg);
static void cmd_flash(fastboot_handle_t *handle, char *arg);
static void cmd_getvar(fastboot_handle_t *handle, char *arg);
static void cmd_oem(fastboot_handle_t *handle, char *arg);
static void cmd_reboot(fastboot_handle_t *handle, char *arg);

static fastboot_params_t fb_params;
static fastboot_handle_t fb_handle;

static struct fastboot_cmd cmds[] = {
	[CMD_INDEX_BOOT] = {
		FASTBOOT_COMMAND_BOOT,
		cmd_boot
	},
	[CMD_INDEX_CONTINUE] = {
		FASTBOOT_COMMAND_CONTINUE,
		cmd_continue
	},
	[CMD_INDEX_DOWNLOAD] = {
		FASTBOOT_COMMAND_DOWNLOAD,
		cmd_download
	},
	[CMD_INDEX_ERASE] = {
		FASTBOOT_COMMAND_ERASE,
		cmd_erase
	},
	[CMD_INDEX_FLASH] = {
		FASTBOOT_COMMAND_FLASH,
		cmd_flash
	},
	[CMD_INDEX_GETVAR] = {
		FASTBOOT_COMMAND_GETVAR,
		cmd_getvar
	},
	[CMD_INDEX_OEM] = {
		FASTBOOT_COMMAND_OEM,
		cmd_oem
	},
	[CMD_INDEX_REBOOT] = {
		FASTBOOT_COMMAND_REBOOT,
		cmd_reboot
	}
};

static struct fastboot_var vars[] = {
	[VAR_INDEX_MAX_DOWNLOAD_SIZE] = {
		FASTBOOT_VAR_MAX_DOWNLOAD_SIZE
	},
	[VAR_INDEX_PARTITION_SIZE] = {
		FASTBOOT_VAR_PARTITION_SIZE
	},
	[VAR_INDEX_PARTITION_TYPE] = {
		FASTBOOT_VAR_PARTITION_TYPE
	},
	[VAR_INDEX_PRODUCT] = {
		FASTBOOT_VAR_PRODUCT
	},
	[VAR_INDEX_SERIALNO] = {
		FASTBOOT_VAR_SERIALNO
	},
	[VAR_INDEX_VERSION] = {
		FASTBOOT_VAR_VERSION
	}
};

unsigned long strtoul(const char *nptr, char **endptr, int base)
{
	unsigned long step, data;
	int i;

	if (base == 0)
		step = 10;
	else if ((base < 2) || (base > 36)) {
		VERBOSE("%s: invalid base %d\n", __func__, base);
		return 0;
	} else
		step = base;

	for (i = 0, data = 0; ; i++) {
		if (nptr[i] == '\0')
			break;
		else if (!isalpha(nptr[i]) && !isdigit(nptr[i])) {
			VERBOSE("%s: invalid string %s at %d [%x]\n",
				__func__, nptr, i, nptr[i]);
			return 0;
		} else {
			data *= step;
			if (isupper(nptr[i]))
				data += nptr[i] - 'A' + 10;
			else if (islower(nptr[i]))
				data += nptr[i] - 'a' + 10;
			else if (isdigit(nptr[i]))
				data += nptr[i] - '0';
		}
	}
	return data;
}

static void fastboot_ack(fastboot_handle_t *handle, const char *code,
			 const char *reason)
{
	char response[FASTBOOT_RESPONSE_LENGTH];

	if (handle->state == STATE_COMMAND) {
		if (reason == NULL)
			reason = "";
		snprintf(response, FASTBOOT_RESPONSE_LENGTH, "%s%s",
			 code, reason);
		handle->state = STATE_COMPLETE;
		handle->write((uintptr_t)response, strlen(response));
		rx_cmd();
	} else {
		assert(0);
	}
}

static void fastboot_okay(fastboot_handle_t *handle, const char *info)
{
	fastboot_ack(handle, "OKAY", info);
}

static void fastboot_fail(fastboot_handle_t *handle, const char *reason)
{
	fastboot_ack(handle, "FAIL", reason);
}

static void fastboot_data(fastboot_handle_t *handle, const char *reason)
{
	char response[FASTBOOT_RESPONSE_LENGTH];

	snprintf(response, FASTBOOT_RESPONSE_LENGTH, "DATA%s", reason);
	handle->write((uintptr_t)response, strlen(response));
}

static void fastboot_info(fastboot_handle_t *handle, const char *reason)
{
	char response[FASTBOOT_RESPONSE_LENGTH];

	snprintf(response, FASTBOOT_RESPONSE_LENGTH, "INFO%s", reason);
	handle->write((uintptr_t)response, strlen(response));
}

static void cmd_boot(fastboot_handle_t *handle, char *arg)
{
	fastboot_fail(handle, "not supported");
}

static void cmd_continue(fastboot_handle_t *handle, char *arg)
{
	fastboot_okay(handle, NULL);
	/* exit fastboot protocol */
	handle->state = STATE_OFFLINE;
}

static void cmd_download(fastboot_handle_t *handle, char *arg)
{
	unsigned long size;
	size_t bytes_read;
	char response[FASTBOOT_RESPONSE_LENGTH];
	int result;

	if (arg[0] == '\0') {
		fastboot_fail(handle, "invalid parameter");
		return;
	}
	size = strtoul(arg + 1, NULL, 16);		/* skip ':' */
	if (size > fb_params.size) {
		fastboot_fail(handle, "file is too large");
	} else {
		snprintf(response, FASTBOOT_RESPONSE_LENGTH, "%08x",
			 (uint32_t)size);
		assert(fb_params.size >= size);
		debug_rx_info(fb_params.base, size);
		fastboot_data(handle, response);
#if 0
		rx_data();
		for (;;) {
			fastboot_device_handle_interrupts();
			if (debug_rxdata_complete()) {
				fastboot_okay(handle, NULL);
				break;
			}
		}
#else
		result = handle->read(fb_params.base, &bytes_read);
		assert(result != 0);
		fastboot_okay(handle, NULL);
#endif
	}
}

static void cmd_erase(fastboot_handle_t *handle, char *arg)
{
	int result = -EINVAL;

	if (cmds[CMD_INDEX_ERASE].callback != NULL) {
		result = cmds[CMD_INDEX_ERASE].callback(arg);
	}
	if (result != 0) {
		fastboot_fail(handle, "invalid parameter");
	} else {
		fastboot_okay(handle, NULL);
	}
}

static void cmd_flash(fastboot_handle_t *handle, char *arg)
{
	uintptr_t dev_handle, image_handle, image_spec = 0;
	part_entry_t entry;
	char response[FASTBOOT_RESPONSE_LENGTH];
	size_t bytes_written;
	int result;

	if (cmds[CMD_INDEX_FLASH].callback != NULL) {
		result = cmds[CMD_INDEX_FLASH].callback(arg);
		if (result != 0) {
			goto exit;
		}
		fastboot_okay(handle, NULL);
		return;
	}
	/* If callback() doesn't exist. */
	result = get_partition_entry(arg, &entry);
	if (result != 0) {
		goto exit;
	}
	/* response status */
	snprintf(response, FASTBOOT_RESPONSE_LENGTH, "erasing flash");
	result = plat_get_image_source(fb_params.image_id, &dev_handle,
				       &image_spec);
	if (result != 0) {
		WARN("Failed to obtain reference to image id=%u (%i)\n",
		     fb_params.image_id, result);
		goto exit;
	}
	result = io_open(dev_handle, image_spec, &image_handle);
	if (result != 0) {
		WARN("Failed to access image id=%u (%i)\n",
		     fb_params.image_id, result);
		goto exit;
	}
	/* response status */
	snprintf(response, FASTBOOT_RESPONSE_LENGTH, "writing flash");
	fastboot_info(handle, response);

	result = io_seek(image_handle, IO_SEEK_SET, entry.start);
	if (result != 0) {
		WARN("Failed to seek (%i)\n", result);
		goto exit_io;
	}
	result = io_write(image_handle, fb_params.base, entry.length,
			  &bytes_written);
	if (result != 0) {
		goto exit_io;
	}
	io_close(image_handle);
	fastboot_okay(handle, NULL);
	return;
exit_io:
	io_close(image_handle);
exit:
	fastboot_fail(handle, "invalid parameter");
}

static void cmd_getvar(fastboot_handle_t *handle, char *arg)
{
	int i, num_vars, len, count;
	int left;
	char response[FASTBOOT_RESPONSE_LENGTH];
	char *var_name;

	left = FASTBOOT_RESPONSE_LENGTH;
	num_vars = sizeof(vars) / sizeof(struct fastboot_var);
	if (strcmp(arg, "all") == 0) {
		for (i = 0, count = 0; i < num_vars; i++) {
			len = strlen(vars[i].name);
			if (vars[i].name[len - 1] != ':') {
				len = snprintf(response + count, left,
					       "\n%s: %s", vars[i].name,
					       vars[i].value);
			} else {
				/*
				 * There's multiple sub-variables if ':'
				 * exists. Call *(execute)() for sub-variables.
				 */
				assert(vars[i].execute != NULL);
				len = vars[i].execute(NULL, left,
						      response + count);
			}
			if ((len < 0) || (left <= len)) {
				/* Don't add more characters into string. */
				response[count] = '\0';
				break;
			} else {
				count += len;
				left = left - len;
			}
		}
		if (count > 0) {
			sprintf(response + count, "\n");
			fastboot_okay(handle, response);
		} else {
			fastboot_okay(handle, NULL);
		}
		return;
	} else {
		for (i = 0; i < num_vars; i++) {
			if (memcmp(arg, vars[i].name, strlen(vars[i].name)))
				continue;
			if (vars[i].execute != NULL) {
				var_name = arg + strlen(vars[i].name);
				len = vars[i].execute(var_name, left,
						      response);
				if ((len < 0) || (left <= len)) {
					response[len] = '\0';
					fastboot_fail(handle, response);
				} else {
					fastboot_okay(handle, response);
				}
			} else {
				fastboot_okay(handle, vars[i].value);
			}
			return;
		}
		/* Can't find matched variable. */
		assert(0);
	}
}

static void cmd_oem(fastboot_handle_t *handle, char *arg)
{
	int result;

	assert((arg != NULL) && (cmds[CMD_INDEX_OEM].callback != NULL));
	result = cmds[CMD_INDEX_OEM].callback(arg);
	if (result == 0) {
		fastboot_okay(handle, NULL);
	} else {
		fastboot_fail(handle, "invalid parameter");
	}
}

static void cmd_reboot(fastboot_handle_t *handle, char *arg)
{
	fastboot_okay(handle, NULL);
	assert(cmds[CMD_INDEX_REBOOT].callback != 0);
	cmds[CMD_INDEX_REBOOT].callback(arg);
}

/* Send responses in fastboot protocol */
int fastboot_send(void)
{
	return 0;
}

int fastboot_handle_command(uintptr_t buf, int length)
{
	int i, num_cmds;

	NOTICE("#%s, %d\n", __func__, __LINE__);
	if (buf == 0) {
		WARN("%s, Invalid parameters.\n", __func__);
		return -EINVAL;
	}
	if (length == 0) {
		INFO("%s ignore\n", __func__);
		return 0;
	}
	if (length > FASTBOOT_MAX_COMMAND_LENGTH)
		length = FASTBOOT_MAX_COMMAND_LENGTH;
	*((char *)buf + length) = '\0';
	num_cmds = sizeof(cmds) / sizeof(struct fastboot_cmd);
	for (i = 0; i < num_cmds; i++) {
		if (memcmp((void *)buf, cmds[i].cmd, strlen(cmds[i].cmd)))
			continue;
		fb_handle.state = STATE_COMMAND;
		cmds[i].execute(&fb_handle,
				(void *)(buf + strlen(cmds[i].cmd)));
		if (fb_handle.state == STATE_COMMAND)
			fastboot_fail(&fb_handle, "unknown reason");
		return 0;
	}
	fastboot_fail(&fb_handle, "unknown command");
	return 0;
}

int fastboot_receive_data(void *buf, int length)
{
	if ((buf == NULL) || (length <= 0)) {
		WARN("%s, Invalid parameters.\n", __func__);
		return -EINVAL;
	}
	return 0;
}

void fastboot_set_var(const char *name, const char *value,
		      int (*execute)(const char *arg, int left, char *response))
{
	int i, num_vars;

	assert((name != NULL) && (value != NULL));
	num_vars = sizeof(vars) / sizeof(struct fastboot_var);
	for (i = 0; i < num_vars; i++) {
		if (memcmp(name, vars[i].name, strlen(vars[i].name)))
			continue;
		vars[i].value = value;
		vars[i].execute = execute;
		return;
	}
	/* set unexpected variable */
	assert(0);
}

void fastboot_register_command(const char *name,
			       int (*callback)(const char *arg))
{
	int i, num_cmds;

	assert((name != NULL) && (callback != NULL));
	num_cmds = sizeof(cmds) / sizeof(struct fastboot_cmd);
	for (i = 0; i < num_cmds; i++) {
		if (memcmp(name, cmds[i].cmd, strlen(cmds[i].cmd)))
			continue;
		cmds[i].callback = callback;
		return;
	}
	/* Can't find expected command. */
	assert(0);
}

static int fastboot_check_io(unsigned int image_id)
{
	uintptr_t dev_handle, image_handle, image_spec = 0;
	int result;

	result = plat_get_image_source(image_id, &dev_handle, &image_spec);
	if (result != 0) {
		WARN("Failed to obtain reference to image id=%u (%i)\n",
			image_id, result);
		return result;
	}
	result = io_open(dev_handle, image_spec, &image_handle);
	if (result != 0) {
		WARN("Failed to access image id=%u (%i)\n", image_id, result);
		return result;
	}
	io_close(image_handle);
	return result;
}

void fastboot_init(const fastboot_ops_t *ops_ptr)
{
	assert((ops_ptr != NULL) &&
	       (ops_ptr->read != NULL) &&
	       (ops_ptr->write != NULL));
	/* initialize fasboot handle */
	fb_handle.state = STATE_OFFLINE;
	fb_handle.read = ops_ptr->read;
	fb_handle.write = ops_ptr->write;
}

int fastboot_run(fastboot_params_t *params)
{
	size_t bytes_read;
	int result;
	int i = 0;

	assert((params != 0) &&
	       ((params->base & FASTBOOT_DATA_MASK) == 0) &&
	       ((params->size & FASTBOOT_DATA_MASK) == 0));
	memcpy(&fb_params, params, sizeof(fastboot_params_t));

	result = fastboot_check_io(params->image_id);
	if (result) {
		return result;
	}
	result = fastboot_device_is_attached();
	if (result) {
		return result;
	}
	fb_handle.state = STATE_COMMAND;

	while ((fb_handle.state != STATE_OFFLINE) &&
	       (fb_handle.state != STATE_ERROR)) {
#if 1
	NOTICE("#%s, %d\n", __func__, __LINE__);
		/* wait for command from host */
		result = fb_handle.read(fb_params.base, &bytes_read);
		if (i++ > 3)
			assert(0);
	NOTICE("#%s, %d\n", __func__, __LINE__);
		if (result == 0) {
			fastboot_handle_command(fb_params.base,
						bytes_read);
		} else {
			assert(0);
		}
#else
		fastboot_device_handle_interrupts();
#endif
	}
	(void)bytes_read;
	return 0;
}

//+++++++++++++++++++++++++++++++++++++++++++
//debug code in usb
void debug_rx_info(uintptr_t buf, unsigned long len)
{
       rx_addr = (unsigned long)buf;
       rx_length = len;
}

int debug_rxdata_complete(void)
{
	if (rx_data_complete)
		return 1;
	return 0;
}
