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
#include <usb.h>

static fastboot_params_t region;
static const usb_ops_t *usb_ops;
static int usb_enum_done;

/*
 * incoming packet and output packet share the same buffer.
 */
int usb_handle_setup_packet(uintptr_t buf, size_t size)
{
	setup_packet *setup;
	struct usb_device_descriptor *descriptor;
	int result;

	assert(size > 0);
	setup = (setup_packet *)buf;
	switch (setup->request) {
	case USB_REQ_GET_STATUS:
		NOTICE("#get status\n");
		break;
	case USB_REQ_CLEAR_FEATURE:
		NOTICE("#clear feature\n");
		size = 0;
		break;
	case USB_REQ_SET_FEATURE:
		NOTICE("#set feature\n");
		break;
	case USB_REQ_SET_ADDRESS:
		NOTICE("#set address (%d)\n", setup->value);
		result = usb_ops->set_addr(setup->value);
		assert(result == 0);
		break;
	case USB_REQ_GET_DESCRIPTOR:
		NOTICE("#descr\n");
		size = sizeof(struct usb_device_descriptor);
		descriptor = (struct usb_device_descriptor *)buf;
		result = usb_ops->get_descriptor(setup, descriptor);
		assert((size <= 64) && (size > 0));
		assert(result == 0);
		break;
	case USB_REQ_GET_CONFIGURATION:
		NOTICE("#get configuration\n");
		break;
	case USB_REQ_SET_CONFIGURATION:
		NOTICE("#set configuration\n");
		break;
	case USB_REQ_GET_INTERFACE:
		NOTICE("#get interface\n");
		break;
	case USB_REQ_SET_INTERFACE:
		NOTICE("set interface\n");
		break;
	default:
		NOTICE("not matched. request:0x%x\n", setup->request);
		assert(0);
		break;
	}

	if (size == 0) {
		/* send empty packet */
		buf = 0;
	}
	usb_ops->submit_packet(0, buf, size);
	return 0;
}

int usb_handle_data_packet(uintptr_t buf, size_t size)
{
	return 0;
}

/*
 * input: buf
 * output: usb_intr & size
 */
int usb_wait_for_interrupt(uintptr_t buf, usb_interrupt_t *usb_intr,
			   size_t *size)
{
	int result;

	/* poll interrupt in dwc2 layer */
	assert((usb_intr != NULL) && (size != NULL));
	result = usb_ops->poll(usb_intr, size);
	NOTICE("#%s, %d, result:%d, type:%d\n", __func__, __LINE__, result, usb_intr->type);
	return result;
}

int usb_enum(void)
{
	usb_interrupt_t intr;
	size_t size;
	int result;

	usb_ops->init();
	usb_enum_done = 0;
	do {
		result = usb_wait_for_interrupt(region.base, &intr, &size);
		assert((result == 0) &&
		       (intr.type >= USB_INT_OUT_SETUP) &&
		       (intr.type < USB_INT_INVALID) &&
		       (region.size >= size));
		NOTICE("#%s, %d, interrupt type:%d, ep:%d, size:0x%lx\n",
			__func__, __LINE__, intr.type, intr.ep_idx, size);
		switch (intr.type) {
		case USB_INT_OUT_SETUP:
			result = usb_handle_setup_packet(region.base, size);
			break;
		case USB_INT_OUT_DATA:
			//result = usb_handle_data_packet(buf, *size);
			break;
		case USB_INT_IN:
			break;
		case USB_INT_ENUM_DONE:
			break;
		case USB_INT_RESET:
			break;
		default:
			assert(0);
		}
		NOTICE("#%s, %d\n", __func__, __LINE__);
	} while (usb_enum_done == 0);
	return result;
}

int usb_read(uintptr_t buf, size_t *size)
{
	int result;
	usb_interrupt_t usb_interrupt;

	assert(size != NULL);
	result = usb_wait_for_interrupt(buf, &usb_interrupt, size);
	assert((result == 0) &&
	       (usb_interrupt.type >= USB_INT_OUT_SETUP) &&
	       (usb_interrupt.type < USB_INT_INVALID));
	switch (usb_interrupt.type) {
	case USB_INT_OUT_SETUP:
		NOTICE("#%s, %d, size:%ld\n", __func__, __LINE__, *size);
		result = usb_handle_setup_packet(buf, *size);
		/* don't need fastboot protocol to handle it */
		*size = 0;
		break;
	case USB_INT_OUT_DATA:
		result = usb_handle_data_packet(buf, *size);
		break;
	case USB_INT_IN:
		/* return IN event to fastboot? */
		break;
	case USB_INT_ENUM_DONE:
		break;
	case USB_INT_RESET:
		break;
	default:
		assert(0);
	}
	return result;
}

int usb_write(uintptr_t buf, size_t size)
{
	return usb_ops->submit_packet(1, buf, size);
}

static fastboot_ops_t fb_ops = {
	.read	= usb_read,
	.write	= usb_write
};

void usb_init(const usb_ops_t *ops_ptr, fastboot_params_t *fb_params)
{
	assert((ops_ptr != NULL) &&			\
	       (fb_params != NULL) &&			\
	       (ops_ptr->get_descriptor != NULL) &&	\
	       (ops_ptr->init != NULL) &&		\
	       (ops_ptr->poll != NULL) &&		\
	       (ops_ptr->submit_packet != NULL));
	usb_ops = ops_ptr;
	memcpy(&region, fb_params, sizeof(fastboot_params_t));

	usb_enum();
	fastboot_init(&fb_ops);
}
