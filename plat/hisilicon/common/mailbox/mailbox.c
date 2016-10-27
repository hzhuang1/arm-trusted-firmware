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
#include <delay_timer.h>
#include <errno.h>
#include <mailbox.h>

static mbox_info_t mbox_info;

/* return 1 if the channel is free */
static int check_free_chan(int chan)
{
	assert((mbox_info.ops != NULL) && (chan >= 0) &&
	       (chan < mbox_info.chans));

	if ((1 << chan) & mbox_info.bitmap)
		return 0;
	return 1;
}

static int get_direction(int chan)
{
	assert((mbox_info.ops != NULL) && (chan >= 0) &&
	       (chan < mbox_info.chans));

	if ((1 << chan) & mbox_info.direction)
		return MAILBOX_DIR_TX;
	return MAILBOX_DIR_RX;
}

int mbox_send_message(int chan, void *message, int len)
{
	int result;

	assert((mbox_info.ops != NULL) && (mbox_info.ops->request != NULL) &&
	       (chan >= 0) && (chan < mbox_info.chans) &&
	       (message != NULL) && (len >= 0));
	assert(get_direction(chan) == MAILBOX_DIR_TX);

	result = mbox_info.ops->send(chan, message, len);
	assert(result == 0);
	return 0;
}

int mbox_recv_message(int chan, void *message, int *len)
{
	int result;

	assert((mbox_info.ops != NULL) && (mbox_info.ops->request != NULL) &&
	       (chan >= 0) && (chan < mbox_info.chans) &&
	       (message != NULL) && (len != NULL));
	assert(get_direction(chan) == MAILBOX_DIR_RX);

	result = mbox_info.ops->recv(chan, message, len);
	assert(result == 0);
	(void)result;
	return 0;
}

int mbox_request_channel(int chan, int direction)
{
	int result;

	assert((mbox_info.ops != NULL) && (mbox_info.ops->request != NULL) &&
	       (chan >= 0) && (chan < mbox_info.chans) &&
	       ((direction == MAILBOX_DIR_TX) ||
		(direction == MAILBOX_DIR_RX)));

	if (check_free_chan(chan) == 0)
		return -EBUSY;

	result = mbox_info.ops->request(chan, direction);
	assert(result == 0);
	mbox_info.bitmap |= 1 << chan;
	if (direction == MAILBOX_DIR_TX)
		mbox_info.direction |= 1 << chan;
	(void)result;
	return 0;
}

void mbox_free_channel(int chan)
{
	assert((mbox_info.ops != NULL) && (mbox_info.ops->free != NULL) &&
	       (chan >= 0) && (chan < mbox_info.chans));

	assert(check_free_chan(chan) == 0);
	mbox_info.ops->free(chan);
	mbox_info.bitmap &= ~(1 << chan);
	mbox_info.direction &= ~(1 << chan);
}

int mbox_init(mbox_ops_t *ops, mbox_params_t *params)
{
	assert((ops != NULL) && (ops->request != NULL) &&
	       (ops->free != NULL) &&
	       (ops->send != NULL) &&
	       (ops->recv != NULL) &&
	       (params != NULL) && (params->chans > 0) &&
	       (params->chans <= MAILBOX_MAX_CHANNELS));

	mbox_info.ops = ops;
	mbox_info.bitmap = 0;
	mbox_info.direction = 0;
	mbox_info.chans = params->chans;
	mbox_info.flags = params->flags;
	return 0;
}
