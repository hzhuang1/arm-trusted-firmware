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
#include <debug.h>
#include <hisi_ipc.h>
#include <hisi_pwrc.h>
#include <hisi_sram_map.h>
#include <mmio.h>
#include <psci.h>

#include "hikey_def.h"

static uintptr_t hikey_sec_entrypoint;

/*******************************************************************************
 * Handler called when a power domain is about to be turned on. The
 * level and mpidr determine the affinity instance.
 ******************************************************************************/
static int hikey_pwr_domain_on(u_register_t mpidr)
{
	int cpu, cluster;
	int curr_cluster;
	unsigned int data;

	cluster = (mpidr & MPIDR_CLUSTER_MASK) >> MPIDR_AFF1_SHIFT;
	cpu = mpidr & MPIDR_CPU_MASK;
	curr_cluster = (read_mpidr() & MPIDR_CLUSTER_MASK) >> MPIDR_AFF1_SHIFT;
	if (cluster != curr_cluster) {
		hisi_ipc_cluster_on(cpu, cluster);
		NOTICE("#%s, %d, cluster:%d, curr_cluster:%d\n",
			__func__, __LINE__, cluster, curr_cluster);
	}
	hisi_pwrc_set_core_bx_addr(cpu, cluster, hikey_sec_entrypoint);
	hisi_ipc_cpu_on(cpu, cluster);
	/* Check power state of cluster */
	data = mmio_read_32(ACPU_CLUSTER_POWERDOWN_FLAGS_ADDR);
	NOTICE("#%s, %d, cluster:%d, cpu:%d, pd:0x%x\n",
		__func__, __LINE__, cluster, cpu, data);
	return 0;
}

static void hikey_pwr_domain_on_finish(const psci_power_state_t *target_state)
{
	NOTICE("#%s, %d\n", __func__, __LINE__);
}

/*******************************************************************************
 * Handler called to check the validity of the non secure entrypoint.
 ******************************************************************************/
static int hikey_validate_ns_entrypoint(uintptr_t entrypoint)
{
	/*
	 * Check if the non secure entrypoint lies within the non
	 * secure DRAM.
	 */
	if ((entrypoint > DDR_BASE) && (entrypoint < (DDR_BASE + DDR_SIZE)))
		return PSCI_E_SUCCESS;

	return PSCI_E_INVALID_ADDRESS;
}

/*******************************************************************************
 * Export the platform handlers to enable psci to invoke them
 ******************************************************************************/
static const plat_psci_ops_t hikey_psci_ops = {
	.cpu_standby			= NULL,
	.pwr_domain_on			= hikey_pwr_domain_on,
	.pwr_domain_off			= NULL,
	.pwr_domain_suspend		= NULL,
	.pwr_domain_on_finish		= hikey_pwr_domain_on_finish,
	.pwr_domain_suspend_finish	= NULL,
	.system_off			= NULL,
	.system_reset			= NULL,
	.validate_power_state		= NULL,
	.validate_ns_entrypoint		= hikey_validate_ns_entrypoint,
	.get_sys_suspend_power_state	= NULL,
};

/*******************************************************************************
 * Export the platform specific power ops and initialize Power Controller
 ******************************************************************************/
int plat_setup_psci_ops(uintptr_t sec_entrypoint,
			const plat_psci_ops_t **psci_ops)
{
	hikey_sec_entrypoint = sec_entrypoint;

	/*
	 * Initialize PSCI ops struct
	 */
	*psci_ops = &hikey_psci_ops;

	return 0;
}
