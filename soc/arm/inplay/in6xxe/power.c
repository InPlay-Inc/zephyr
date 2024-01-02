/*
 * Copyright (c) 2023 Inplay Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "zephyr/irq.h"
#include <zephyr/kernel.h>
#include <zephyr/pm/pm.h>
#include <soc.h>
#include <zephyr/init.h>

#include <zephyr/logging/log.h>

#include <zephyr/drivers/timer/system_timer.h>

#include <hal/hal_global.h>
#include <hal/hal_gpio.h>
#include <hal/hal_timer.h>
#include <in_irq.h>

LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

extern void shutdown_and_resume(void);

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
		arch_irq_unlock(0);
		/* This PD1 registers, which should be recovered when wakeup */
		irq_enable(Aon_Tmr_Misc_IRQn);
		aon_tmr_emit_int_mask_clear(AON_EMIT0_ID);
		break;
	default:
		LOG_DBG("Unsupported power state %u", state);
		break;
	}
}

