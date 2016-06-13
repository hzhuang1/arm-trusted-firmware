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
#include <errno.h>
#include <fastboot.h>

#if 0
static const fb_dev_info_t *fb_device;
static const fb_funcs_t *fb_funcs;

static int fb_fsm(fb_event_t *event)
{
	return 0;
}

int fb_run(void)
{
	struct fb_dev_funcs_t *funcs;
	fb_event_t event;
	int ret;

	while (1) {
		ret = funcs->wait_event(&event);
		if (ret != 0) {
			WARN("%s: failed to get event, error:%d\n",
			     __func__, ret);
			return ret;
		}
		ret = fb_fsm(&event);
		if (ret != 0) {
			WARN("%s: failed to handle fastboot event, error:%d\n",
			     __func__, ret);
			return ret;
		}
	}
	return 0;
}
#endif

#if 0
static fb_dev_info_t fb_device;

/* fastboot FSM */
int fb_run(uintptr_t handle)
{
	fb_event_t event;
	int result;

	assert((fb_device.funcs != NULL) &&
	       (fb_device.funcs->start != NULL) &&
	       (fb_device.funcs->stop != NULL) &&
	       (fb_device.funcs->send != NULL) &&
	       (fb_device.funcs->recv != NULL));
	result = fb_device.funcs->start(handle);
	assert(result == 0);
	while (1) {
		result = fb_device.funcs->wait_event(handle, &event);
		assert(result == 0);
	}
	(void)result;
	return 0;
}

int fb_register_device(const fb_funcs_t *funcs_ptr, void **dev_handle)
{
	int result = -EBUSY;

	assert(funcs_ptr != NULL);
	if (fb_device.funcs != NULL)
		return result;
	fb_device.funcs = funcs_ptr;
	*dev_handle = (void *)&fb_device;
	result = 0;
	return result;
}
#endif

#pragma weak fastboot_device_is_attached
#pragma weak fastboot_device_handle_interrupts

int fastboot_run(void)
{
	int result;

	result = fastboot_device_is_attached();
	if (result)
		return result;

	while (1) {
		fastboot_device_handle_interrupts();
	}
	return 0;
}
