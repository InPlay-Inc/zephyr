/*
 * Copyright (c) 2023 Inplay Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "zephyr/arch/arm/irq.h"
#include "zephyr/irq.h"
#include <zephyr/kernel.h>
#include <zephyr/pm/pm.h>
#include <soc.h>
#include <zephyr/init.h>

#include <zephyr/logging/log.h>

#include <zephyr/drivers/timer/system_timer.h>
#include <cortex_m/exc.h>

#include <hal/hal_global.h>
#include <hal/hal_gpio.h>
#include <hal/hal_timer.h>

#define Aon_Tmr_Misc_IRQn 34

LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

extern void shutdown_and_resume(void);
extern bool in6xxe_ble_sleep(void);

/* Invoke Low Power/System Off specific Tasks */
void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE:

		hal_gpio_suspend();
		hal_global_suspend();

		shutdown_and_resume();

		hal_global_resume();				
		hal_gpio_resume();

		break;
	default:
		LOG_DBG("Unsupported power state %u", state);
		return;
	}
}

/* Handle SOC specific activity after Low Power Mode Exit */
void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE:
		/* Cortex-M core registers are lost when entering deep sleep, so recover them here */
		z_arm_exc_setup();
		arch_irq_unlock(0);
		/* These are PD1 registers, which should be recovered when wakeup */
		z_arm_irq_priority_set(Aon_Tmr_Misc_IRQn, 1, 0);
		irq_enable(Aon_Tmr_Misc_IRQn);
		aon_tmr_emit_int_mask_clear(AON_EMIT0_ID);
		//clk_rc_bypass_en();
		//hal_clk_calib_32k(5);
		break;
	default:
		LOG_DBG("Unsupported power state %u", state);
		break;
	}
}

#ifdef CONFIG_PM_POLICY_CUSTOM
const struct pm_state_info *pm_policy_next_state(uint8_t cpu, int32_t ticks)
{
	int64_t cyc = -1;
	uint8_t num_cpu_states;
	const struct pm_state_info *cpu_states;

#ifdef CONFIG_PM_NEED_ALL_DEVICES_IDLE
	if (pm_device_is_any_busy()) {
		return NULL;
	}
#endif

	if (!hal_clk_32k_ready()) {
		return NULL;
	}

#if CONFIG_BT_IN6XX
	if (!in6xxe_ble_sleep()) {
		return NULL;
	}
#endif

	num_cpu_states = pm_state_cpu_get_all(cpu, &cpu_states);

	for (int16_t i = (int16_t)num_cpu_states - 1; i >= 0; i--) {
		const struct pm_state_info *state = &cpu_states[i];
		uint32_t min_residency;

		min_residency = k_us_to_ticks_ceil32(state->min_residency_us);
		/*
		 * The tick interval for the system to enter sleep mode needs
		 * to be longer than or equal to the minimum residency.
		 */
		if (ticks >= min_residency) {
			return state;
		}
	}
	return NULL;
}
#endif

