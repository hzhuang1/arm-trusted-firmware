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
#include <dw_ufs.h>
#include <mmio.h>
#include <stdint.h>
#include <string.h>
#include <ufs.h>

static int dwufs_phy_init(ufs_params_t *params)
{
	uic_cmd_t cmd;
	uintptr_t base;
	unsigned int tx0, tx1;
	int result;

	assert((params != NULL) && 		\
	       (params->reg_base != 0));

	base = params->reg_base;

	ufshc_dme_set(VS_MPHY_DISABLE, 0, 1);
	ufshc_dme_set(PA_HSSERIES, 0, 2);
	memset(&cmd, 0, sizeof(cmd));
	cmd.op = DME_SET;
	cmd.arg1 = 0x81140000;
	cmd.arg3 = 1;
	ufshc_send_uic_cmd(base, &cmd);
	memset(&cmd, 0, sizeof(cmd));
	cmd.op = DME_SET;
	cmd.arg1 = 0x81210000;
	cmd.arg3 = 0x2d;
	ufshc_send_uic_cmd(base, &cmd);
	memset(&cmd, 0, sizeof(cmd));
	cmd.op = DME_SET;
	cmd.arg1 = 0x81220000;
	cmd.arg3 = 0x1;
	ufshc_send_uic_cmd(base, &cmd);
	ufshc_dme_set(VS_MPHY_CFG_UPDT, 0, 1);

	memset(&cmd, 0, sizeof(cmd));
	cmd.op = DME_SET;
	cmd.arg1 = 0x800d0004;
	cmd.arg3 = 0x58;
	ufshc_send_uic_cmd(base, &cmd);
	memset(&cmd, 0, sizeof(cmd));
	cmd.op = DME_SET;
	cmd.arg1 = 0x800d0005;
	cmd.arg3 = 0x58;
	ufshc_send_uic_cmd(base, &cmd);
	memset(&cmd, 0, sizeof(cmd));
	cmd.op = DME_SET;
	cmd.arg1 = 0x800e0004;
	cmd.arg3 = 0xb;
	ufshc_send_uic_cmd(base, &cmd);
	memset(&cmd, 0, sizeof(cmd));
	cmd.op = DME_SET;
	cmd.arg1 = 0x800e0005;
	cmd.arg3 = 0xb;
	ufshc_send_uic_cmd(base, &cmd);
	memset(&cmd, 0, sizeof(cmd));
	cmd.op = DME_SET;
	cmd.arg1 = 0x80090004;
	cmd.arg3 = 0x1;
	ufshc_send_uic_cmd(base, &cmd);
	memset(&cmd, 0, sizeof(cmd));
	cmd.op = DME_SET;
	cmd.arg1 = 0x80090005;
	cmd.arg3 = 0x1;
	ufshc_send_uic_cmd(base, &cmd);
	ufshc_dme_set(VS_MPHY_CFG_UPDT, 0, 1);

	memset(&cmd, 0, sizeof(cmd));
	cmd.op = DME_SET;
	cmd.arg1 = 0x81130000;
	cmd.arg3 = 0x1;
	ufshc_send_uic_cmd(base, &cmd);
	ufshc_dme_set(VS_MPHY_CFG_UPDT, 0, 1);

	memset(&cmd, 0, sizeof(cmd));
	cmd.op = DME_SET;
	cmd.arg1 = 0x00950004;
	cmd.arg3 = 0x4a;
	ufshc_send_uic_cmd(base, &cmd);
	memset(&cmd, 0, sizeof(cmd));
	cmd.op = DME_SET;
	cmd.arg1 = 0x00950005;
	cmd.arg3 = 0x4a;
	ufshc_send_uic_cmd(base, &cmd);
	memset(&cmd, 0, sizeof(cmd));
	cmd.op = DME_SET;
	cmd.arg1 = 0x00940004;
	cmd.arg3 = 0x4a;
	ufshc_send_uic_cmd(base, &cmd);
	memset(&cmd, 0, sizeof(cmd));
	cmd.op = DME_SET;
	cmd.arg1 = 0x00940005;
	cmd.arg3 = 0x4a;
	ufshc_send_uic_cmd(base, &cmd);
	memset(&cmd, 0, sizeof(cmd));
	cmd.op = DME_SET;
	cmd.arg1 = 0x008f0004;
	cmd.arg3 = 0x7;
	ufshc_send_uic_cmd(base, &cmd);
	memset(&cmd, 0, sizeof(cmd));
	cmd.op = DME_SET;
	cmd.arg1 = 0x008f0005;
	cmd.arg3 = 0x7;
	ufshc_send_uic_cmd(base, &cmd);
	memset(&cmd, 0, sizeof(cmd));
	cmd.op = DME_SET;
	cmd.arg1 = 0x000f0000;
	cmd.arg3 = 0x5;
	ufshc_send_uic_cmd(base, &cmd);
	memset(&cmd, 0, sizeof(cmd));
	cmd.op = DME_SET;
	cmd.arg1 = 0x000f0001;
	cmd.arg3 = 0x5;
	ufshc_send_uic_cmd(base, &cmd);
	ufshc_dme_set(VS_MPHY_CFG_UPDT, 0, 1);

	/* FIXME: uic cmd read */

	memset(&cmd, 0, sizeof(cmd));
	cmd.op = DME_SET;
	cmd.arg1 = (VS_MPHY_DISABLE << 16);
	cmd.arg3 = 0x0;
	ufshc_send_uic_cmd(base, &cmd);

	do {
		result = ufshc_dme_get(0x41, 0, &tx0);
		assert(result == 0);
		result = ufshc_dme_get(0x41, 1, &tx1);
		assert(result == 0);
	} while ((tx0 != 1) || (tx1 != 1));

	mmio_write_32(base + HCLKDIV, 0xE4);
	mmio_clrbits_32(base + AHIT, 0x3FF);

	memset(&cmd, 0, sizeof(cmd));
	cmd.op = DME_SET;
	cmd.arg1 = 0x155e0000;
	ufshc_send_uic_cmd(base, &cmd);
	memset(&cmd, 0, sizeof(cmd));
	cmd.op = DME_SET;
	cmd.arg1 = 0xd0ab0000;
	ufshc_send_uic_cmd(base, &cmd);
	return 0;
}

static int dwufs_hc_init(ufs_params_t *params)
{
	assert((params != NULL) && 		\
	       (params->reg_base != 0));

	ufshc_dme_set(0x2044, 0, 0);
	ufshc_dme_set(0x2045, 0, 0);
	ufshc_dme_set(0x2040, 0, 9);
	return 0;
}

const ufs_ops_t dw_ufs_ops = {
	.phy_init 	= dwufs_phy_init,
	.hc_init 	= dwufs_hc_init,
};

int dw_ufs_init(dw_ufs_params_t *params)
{
	ufs_params_t ufs_params;

	memset(&ufs_params, 0, sizeof(ufs_params));
	ufs_params.reg_base = params->reg_base;
	ufs_params.desc_base = params->desc_base;
	ufs_params.desc_size = params->desc_size;
	ufs_params.flags = params->flags;
	ufs_init(&dw_ufs_ops, &ufs_params);
	return 0;
}
