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
#include <assert.h>
#include <debug.h>
#include <fastboot.h>
#include <usb.h>
#include <mmio.h>

typedef struct usb_buffer {
	uintptr_t	rx_buf;
	size_t		rx_size;
	uintptr_t	tx_buf;
	size_t		tx_size;
} usb_buffer_t;

static usb_buffer_t region;
static const usb_ops_t *usb_ops;
static int usb_enum_done;

void dump_recv_buf(size_t size)
{
	uint32_t base;
	int i;

	base = region.rx_buf;
	INFO("dump rx size:%ld, [%x]\n", size, base);
	for (i = 0; i < size; i += 8) {
		INFO("[%x %x %x %x %x %x %x %x]\n",
			mmio_read_8(base + i),
			mmio_read_8(base + i + 1),
			mmio_read_8(base + i + 2),
			mmio_read_8(base + i + 3),
			mmio_read_8(base + i + 4),
			mmio_read_8(base + i + 5),
			mmio_read_8(base + i + 6),
			mmio_read_8(base + i + 7));
	}
	(void)base;
}

void dump_send_buf(size_t size)
{
	uint32_t base;
	int i;

	base = region.tx_buf;
	INFO("dump tx size:%ld, [%x]\n", size, base);
	for (i = 0; i < size; i += 8) {
		INFO("[%x %x %x %x %x %x %x %x]\n",
			mmio_read_8(base + i),
			mmio_read_8(base + i + 1),
			mmio_read_8(base + i + 2),
			mmio_read_8(base + i + 3),
			mmio_read_8(base + i + 4),
			mmio_read_8(base + i + 5),
			mmio_read_8(base + i + 6),
			mmio_read_8(base + i + 7));
	}
	(void)base;
}

int usb_recv_setup(uintptr_t buf, size_t size)
{
	int result;

	assert((size <= 64) && (size > 0));
	result = usb_ops->recv_setup(buf, size);
	assert(result == 0);
	return result;
}

int usb_setup_response(uintptr_t buf, size_t size)
{
	usb_ops->setup_response(buf, size);
	return 0;
}

/*
 * incoming packet and output packet share the same buffer.
 */
int usb_handle_setup_packet(uintptr_t buf)
{
	setup_packet *setup;
	size_t size = 0;
	int result;

	/* debug code */
	memset((void *)region.tx_size, 0, 0x1000);
	isb();
	dsb();

	setup = (setup_packet *)buf;
	switch (setup->request) {
	case USB_REQ_GET_STATUS:
		INFO("#get status\n");
		break;
	case USB_REQ_CLEAR_FEATURE:
		INFO("#clear feature\n");
		size = 0;
		break;
	case USB_REQ_SET_FEATURE:
		INFO("#set feature\n");
		break;
	case USB_REQ_SET_ADDRESS:
		INFO("#set address (%d)\n", setup->value);
		result = usb_ops->set_addr(setup->value);
		assert(result == 0);
		size = 0;
		break;
	case USB_REQ_GET_DESCRIPTOR:
		result = usb_ops->get_descriptor(setup, region.tx_buf, &size);
		INFO("for tx, size:0x%lx\n", size);
		dump_send_buf(size);
		//INFO("#descr, value:0x%x, index:0x%x, length:0x%x, size:0x%lx\n", setup->value, setup->index, setup->length, size);
		assert((result == 0) && (size < region.tx_size));
		//NOTICE("#%s, iSerialNum:%d\n", __func__, descriptor->iSerialNumber);
		break;
	case USB_REQ_GET_CONFIGURATION:
		INFO("#get configuration\n");
		break;
	case USB_REQ_SET_CONFIGURATION:
		INFO("#set configuration\n");
		break;
	case USB_REQ_GET_INTERFACE:
		INFO("#get interface\n");
		break;
	case USB_REQ_SET_INTERFACE:
		INFO("set interface\n");
		break;
	default:
		INFO("not matched. request:0x%x\n", setup->request);
		assert(0);
		break;
	}

	if (size == 0)
		buf = 0;
	usb_ops->setup_response(region.tx_buf, size);
	(void)result;
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
	/*
	NOTICE("#%s, %d, result:%d, ep:%d, type:%d\n",
		__func__, __LINE__, result, usb_intr->ep_idx, usb_intr->type);
		*/
	return result;
}

int usb_enum(void)
{
	usb_interrupt_t intr;
	size_t size;
	int result;

	usb_ops->init();
	usb_enum_done = 0;
	result = usb_recv_setup(region.rx_buf, 64);
	assert(result == 0);
	do {
		result = usb_wait_for_interrupt(region.rx_buf, &intr, &size);
		assert((result == 0) &&
		       (intr.type >= USB_INT_OUT_SETUP) &&
		       (intr.type < USB_INT_INVALID) &&
		       (region.rx_size >= size));
		switch (intr.type) {
		case USB_INT_OUT_SETUP:
			INFO("setup\n");
			//dump_recv_buf(size);
			result = usb_handle_setup_packet(region.rx_buf);
			assert(result == 0);
			result = usb_recv_setup(region.rx_buf, 64);
			assert(result == 0);
			break;
		case USB_INT_OUT_DATA:
			INFO("data\n");
			//dump_recv_buf(size);
			//result = usb_handle_data_packet(buf, *size);
			break;
		case USB_INT_IN:
			dump_send_buf(0x40);
			//dump_send_buf(size);
			break;
		case USB_INT_ENUM_DONE:
			usb_recv_setup(region.rx_buf, 64);
			break;
		case USB_INT_RESET:
			break;
		default:
			assert(0);
		}
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
		//INFO("#%s, %d, size:%ld\n", __func__, __LINE__, *size);
		result = usb_handle_setup_packet(buf);
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

#if 0
static fastboot_ops_t fb_ops = {
	.read	= usb_read,
	.write	= usb_write
};
#endif

void usb_init(const usb_ops_t *ops_ptr, fastboot_params_t *fb_params)
{
	assert((ops_ptr != NULL) &&			\
	       (ops_ptr->get_descriptor != NULL) &&	\
	       (ops_ptr->init != NULL) &&		\
	       (ops_ptr->poll != NULL) &&		\
	       (ops_ptr->setup_response != NULL) &&	\
	       (ops_ptr->submit_packet != NULL) &&	\
	       (fb_params != NULL) &&			\
	       (fb_params->size != 0));
	usb_ops = ops_ptr;
	region.rx_buf = fb_params->base;
	region.rx_size = fb_params->size / 2;
	region.tx_buf = fb_params->base + region.rx_size;
	region.tx_size = fb_params->size / 2;

	usb_enum();
	//fastboot_init(&fb_ops);
}
