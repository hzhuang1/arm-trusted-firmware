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
#ifndef __MAILBOX_H__
#define __MAILBOX_H__

#include <stddef.h>
#include <stdint.h>

#define MAILBOX_ACK_TIMEOUT 				0x1000

#define MAILBOX_MAX_CHANNELS 				64

#define MAILBOX_DIR_TX 					1
#define MAILBOX_DIR_RX 					0

typedef struct mbox_ops {
	int 	(*request)(int chan, int direction);
	void 	(*free)(int chan);
	/* block operation */
	int 	(*send)(int chan, void *message, int len);
	int 	(*recv)(int chan, void *message, int *len);
} mbox_ops_t;

typedef struct mbox_params {
	unsigned int 		flags;
	int 			chans;
	uintptr_t 		tx_buf;
	uintptr_t 		rx_buf;
	int 			tx_size;
	int 			rx_size;
} mbox_params_t;

typedef struct mbox_info {
	mbox_ops_t 		*ops;
	uintptr_t 		tx_buf;
	uintptr_t 		rx_buf;
	int 			tx_size;
	int 			rx_size;
	unsigned long long 	bitmap; 	/* 64 channels */
	unsigned long long 	direction; 	/* 64 channels */
	int 			chans;
	unsigned int 		flags;
} mbox_info_t;

int mbox_send_message(int chan, void *message, int len);
int mbox_recv_message(int chan, void *message, int *len);
int mbox_request_channel(int chan, int direction);
void mbox_free_channel(int chan);
int mbox_init(mbox_ops_t *ops, mbox_params_t *params);

#endif /* __MAILBOX_H__ */
