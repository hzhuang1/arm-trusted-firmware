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
#include <cci.h>
#include <debug.h>
#include <gicv2.h>
#include <hi3660.h>
#include <mmio.h>
#include <psci.h>

#include "hikey960_def.h"

static uintptr_t hikey960_sec_entrypoint;

/*******************************************************************************
 * Handler called when a power domain is about to be turned on. The
 * level and mpidr determine the affinity instance.
 ******************************************************************************/
static int hikey960_pwr_domain_on(u_register_t mpidr)
{
	return 0;
}

static void hikey960_pwr_domain_on_finish(const psci_power_state_t *target_state)
{
}

/*******************************************************************************
 * Handler called when a power domain is about to be turned off. The
 * target_state encodes the power state that each level should transition to.
 ******************************************************************************/
void hikey960_pwr_domain_off(const psci_power_state_t *target_state)
{
}

/*******************************************************************************
 * Handler to reboot the system.
 ******************************************************************************/
static void __dead2 hikey960_system_reset(void)
{
	panic();
}

/*******************************************************************************
 * Handler called to check the validity of the power state parameter.
 ******************************************************************************/
int hikey960_validate_power_state(unsigned int power_state,
			       psci_power_state_t *req_state)
{
	int pstate = psci_get_pstate_type(power_state);
	int pwr_lvl = psci_get_pstate_pwrlvl(power_state);
	int i;

	assert(req_state);

	if (pwr_lvl > PLAT_MAX_PWR_LVL)
		return PSCI_E_INVALID_PARAMS;

	/* Sanity check the requested state */
	if (pstate == PSTATE_TYPE_STANDBY) {
		/*
		 * It's possible to enter standby only on power level 0
		 * Ignore any other power level.
		 */
		if (pwr_lvl != MPIDR_AFFLVL0)
			return PSCI_E_INVALID_PARAMS;

		req_state->pwr_domain_state[MPIDR_AFFLVL0] =
					PLAT_MAX_RET_STATE;
	} else {
		for (i = MPIDR_AFFLVL0; i <= pwr_lvl; i++)
			req_state->pwr_domain_state[i] =
					PLAT_MAX_OFF_STATE;
	}

	/*
	 * We expect the 'state id' to be zero.
	 */
	if (psci_get_pstate_id(power_state))
		return PSCI_E_INVALID_PARAMS;

	return PSCI_E_SUCCESS;
}

/*******************************************************************************
 * Handler called to check the validity of the non secure entrypoint.
 ******************************************************************************/
static int hikey960_validate_ns_entrypoint(uintptr_t entrypoint)
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
static const plat_psci_ops_t hikey960_psci_ops = {
	.cpu_standby			= NULL,
	.pwr_domain_on			= hikey960_pwr_domain_on,
	.pwr_domain_on_finish		= hikey960_pwr_domain_on_finish,
	.pwr_domain_off			= hikey960_pwr_domain_off,
	.pwr_domain_suspend		= NULL,
	.pwr_domain_suspend_finish	= NULL,
	.system_off			= NULL,
	.system_reset			= hikey960_system_reset,
	.validate_power_state		= hikey960_validate_power_state,
	.validate_ns_entrypoint		= hikey960_validate_ns_entrypoint,
	.get_sys_suspend_power_state	= NULL,
};

/*******************************************************************************
 * Export the platform specific power ops and initialize Power Controller
 ******************************************************************************/
int plat_setup_psci_ops(uintptr_t sec_entrypoint,
			const plat_psci_ops_t **psci_ops)
{
	hikey960_sec_entrypoint = sec_entrypoint;

	/*
	 * Initialize PSCI ops struct
	 */
	*psci_ops = &hikey960_psci_ops;

	return 0;
}
