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

#ifndef __USB_H__
#define __USB_H__

#include <fastboot.h>

typedef enum usb_interrupt_type {
	USB_INT_OUT_SETUP,
	USB_INT_OUT_DATA,
	USB_INT_IN,
	USB_INT_ENUM_DONE,
	USB_INT_RESET,
	USB_INT_INVALID
} usb_interrupt_type_t;

typedef struct usb_interrupt {
	usb_interrupt_type_t	type;
	int			ep_idx;
} usb_interrupt_t;

typedef struct {
	unsigned char		type;
	unsigned char		request;
	unsigned short		value;
	unsigned short		index;
	unsigned short		length;
} setup_packet;

/*
 * Standard requests, for the bRequest field of a SETUP packet.
 *
 * These are qualified by the bRequestType field, so that for example
 * TYPE_CLASS or TYPE_VENDOR specific feature flags could be retrieved
 * by a GET_STATUS request.
 */
#define USB_REQ_GET_STATUS              0x00
#define USB_REQ_CLEAR_FEATURE           0x01
#define USB_REQ_SET_FEATURE             0x03
#define USB_REQ_SET_ADDRESS             0x05
#define USB_REQ_GET_DESCRIPTOR          0x06
#define USB_REQ_SET_DESCRIPTOR          0x07
#define USB_REQ_GET_CONFIGURATION       0x08
#define USB_REQ_SET_CONFIGURATION       0x09
#define USB_REQ_GET_INTERFACE           0x0A
#define USB_REQ_SET_INTERFACE           0x0B
#define USB_REQ_SYNCH_FRAME             0x0C

/* USB_DT_DEVICE: Device descriptor */
struct usb_device_descriptor {
        unsigned char  bLength;
        unsigned char  bDescriptorType;

        unsigned short bcdUSB;
        unsigned char  bDeviceClass;
        unsigned char  bDeviceSubClass;
        unsigned char  bDeviceProtocol;
        unsigned char  bMaxPacketSize0;
        unsigned short idVendor;
        unsigned short idProduct;
        unsigned short bcdDevice;
        unsigned char  iManufacturer;
        unsigned char  iProduct;
        unsigned char  iSerialNumber;
        unsigned char  bNumConfigurations;
} __attribute__ ((packed));

typedef struct usb_ops {
	int	(*get_descriptor)(setup_packet *setup, struct usb_device_descriptor *descriptor);
	void	(*init)(void);
	int	(*poll)(usb_interrupt_t *usb_intr, size_t *size);
	int	(*recv_setup)(uintptr_t buf, size_t size);
	int	(*set_addr)(int addr);
	int	(*setup_response)(uintptr_t buf, size_t size);
	int	(*submit_packet)(int ep_idx, uintptr_t buf, size_t size);
} usb_ops_t;

void usb_init(const usb_ops_t *ops_ptr, fastboot_params_t *fb_params);

#endif	/* __USB_H__ */
