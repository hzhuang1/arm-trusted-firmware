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
#include <hi3660_mailbox.h>
#include <mailbox.h>
#include <mmio.h>
#include <string.h>

typedef struct hi3660_chan {
	unsigned char 	src;
	unsigned char 	dst;
	unsigned char 	used;
} hi3660_chan_t;

static hi3660_chan_t chan_map[MBX_MAX_CHANNELS];

static void hi3660_mbox_check_state(int chan, unsigned int state)
{
	unsigned int data;

	data = mmio_read_32(MBX_MODE(chan));
	assert((data & (MBX_MODE_AUTO_ANSWER | MBX_MODE_AUTO_LINK)) == 0);

	data &= MBX_MODE_STATE_STATUS_MASK;
	assert(data == state);
	(void)state;
}

static int hi3660_mbox_send(int chan, void *message, int len)
{
	int i;
	unsigned int *buf;
	unsigned int data;

	assert((chan >= 0) && (chan < MBX_MAX_CHANNELS) &&
	       (message != NULL) && (len <= MBX_MAX_DATA_LEN));
	assert((chan_map[chan].used != 0) &&
	       (chan_map[chan].src != 0) &&
	       (chan_map[chan].dst != 0));

	buf = (unsigned int *)message;
	len = ((len + 3) >> 2); 	/* convert to word count */
	for (i = 0; i < len; i++) {
		mmio_write_32(MBX_DATA0(chan) + (i << 2), *(buf + i));
	}
	/* send out */
	mmio_write_32(MBX_SEND(chan), chan_map[chan].src);

	do {
		data = mmio_read_32(MBX_ICLR(chan));
	} while ((data & chan_map[chan].src) == 0);
	/* ack */
	mmio_write_32(MBX_ICLR(chan), chan_map[chan].src);
	return 0;
}

static int hi3660_mbox_recv(int chan, void *message, int *len)
{
	assert((chan >= 0) && (chan < MBX_MAX_CHANNELS) &&
	       (message != NULL) && (len != NULL));
	assert((chan_map[chan].used != 0) &&
	       (chan_map[chan].src != 0) &&
	       (chan_map[chan].dst != 0));
	unsigned int *buf, data;
	int i;

	/* wait IPC event */
	do {
		data = mmio_read_32(MBX_MODE(chan));
	} while ((data & MBX_MODE_STATE_STATUS_MASK) != MBX_MODE_STATE_DEST);
	/* wait to clear interrupt */
	do {
		data = mmio_read_32(MBX_ICLR(chan));
	} while (data == 0);
	do {
		mmio_write_32(MBX_ICLR(chan), chan_map[chan].dst);
		data = mmio_read_32(MBX_ICLR(chan));
	} while (data);

	/* read data from IPC */
	buf = (unsigned int *)message;
	for (i = 0; i < MBX_MAX_DATA_LEN; i += 4) {
		*(buf + (i >> 2)) = mmio_read_32(MBX_DATA0(chan) + i);
	}
	*len = MBX_MAX_DATA_LEN;
	/* ack */
	mmio_write_32(MBX_SEND(chan), chan_map[chan].dst);
	return 0;
}

static int hi3660_mbox_request(int chan, int direction)
{
	unsigned int data;
	unsigned int src, dst;

	assert((chan >= 0) && (chan < MBX_MAX_CHANNELS));

	if (direction == MAILBOX_DIR_TX) {
		src = CPU_A53;
		dst = CPU_LPM3;
	} else if (direction == MAILBOX_DIR_RX) {
		src = CPU_LPM3;
		dst = CPU_A53;
	} else
		assert(0);
	mmio_write_32(MBX_SOURCE(chan), src);
	data = mmio_read_32(MBX_SOURCE(chan));
	assert(data == src);
	
	/* mask all interrupts */
	mmio_write_32(MBX_IMASK(chan), CPU_MASK);
	/* unmask interrupt */
	mmio_write_32(MBX_IMASK(chan), ~(src | dst));

	/* set destination */
	mmio_write_32(MBX_DCLEAR(chan), (~dst) & CPU_MASK);
	mmio_write_32(MBX_DSET(chan), dst);
	data = mmio_read_32(MBX_DSTATUS(chan));
	assert((data & dst) != 0);

	/* clear auto link & auto answer */
	data = mmio_read_32(MBX_MODE(chan));
	data &= ~(MBX_MODE_AUTO_ANSWER | MBX_MODE_AUTO_LINK);
	mmio_write_32(MBX_MODE(chan), data);

	hi3660_mbox_check_state(chan, MBX_MODE_STATE_SOURCE);
	chan_map[chan].used = 1;
	chan_map[chan].src = src;
	chan_map[chan].dst = dst;
	return 0;
}

static void hi3660_mbox_free(int chan)
{
	assert((chan >= 0) && (chan < MBX_MAX_CHANNELS));
}

static mbox_ops_t hi3660_mbox_ops = {
	.send 		= hi3660_mbox_send,
	.recv 		= hi3660_mbox_recv,
	.request 	= hi3660_mbox_request,
	.free 		= hi3660_mbox_free,
};

int hi3660_mbox_init(mbox_params_t *params)
{
	int result;
	unsigned int data;

	assert(params != NULL);
	result = mbox_init(&hi3660_mbox_ops, params);
	assert(result == 0);
	memset(&chan_map, 0, sizeof(chan_map));

	/* unlock mailbox */
	data = mmio_read_32(IPC_LOCK);
	while (data == MBX_IPC_LOCKED) {
		mmio_write_32(IPC_LOCK, MBX_IPC_UNLOCK_MAGIC);
		data = mmio_read_32(IPC_LOCK);
	}
	(void)result;
	return 0;
}
