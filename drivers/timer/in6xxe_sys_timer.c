/*
 * Copyright (c) 2023, Inplay Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT inplay_in6xxe_systimer

#include <zephyr/init.h>
#include <soc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys_clock.h>
#include <zephyr/sys/util.h>

#include <hal/hal_timer.h>
#include <in_irq.h>

#define COUNTER_MAX UINT32_MAX

#define CYC_PER_TICK (sys_clock_hw_cycles_per_sec()	\
		      / CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define MAX_TICKS ((k_ticks_t)(COUNTER_MAX / CYC_PER_TICK) - 1)
#define MAX_CYCLES (MAX_TICKS * CYC_PER_TICK)
#define MIN_DELAY 1

/* Timer count down, assume interval < COUNTER_MAX*/
#define COUNTER_DIFF(now, last) ((now) < (last) ? ((last) - (now)) : (COUNTER_MAX - (now) + (last)))
#define COUNTER_ADD(now, diff) ((now) > (diff) ? (now) - (diff) : COUNTER_MAX - (diff) + (now))

#define AON_TMR2_EMIT0_INT_STATUS   BIT(6)

/* Value of AON TIMER counter when the previous kernel tick was announced */
static atomic_t g_last_count;

/* Spinlock to sync between Compare ISR and update of Compare register */
static struct k_spinlock g_lock;

extern void delay_us(uint32_t);

static uint32_t aon_tmr2_get(void)
{
	aon_tmr2_snap_tick();
	return aon_tmr2_read_tick();
}

static void aon_tmr2_set_emit(int id, uint32_t tick)
{
	aon_tmr_emit_set_tick(id, tick);
}

static void aon_tmr2_isr(const void *arg)
{
	ARG_UNUSED(arg);

	uint32_t irq_status = aon_tmr_int_status();

	if (irq_status & AON_TMR2_EMIT0_INT_STATUS) {
		/* Clear PD1 interrupt */
		aon_tmr_int_clear(AON_TMR2_EMIT0_INT_STATUS);
		/* Clear emit signal */
		aon_tmr_emit_man_clr(AON_EMIT0_ID);

		k_spinlock_key_t key = k_spin_lock(&g_lock);

		uint32_t now = aon_tmr2_get();
		uint32_t dticks = (uint32_t)(COUNTER_DIFF(now, g_last_count) / CYC_PER_TICK);

		g_last_count = now;

		if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
			uint32_t next = COUNTER_ADD(g_last_count, CYC_PER_TICK);
			aon_tmr_emit_set_tick(AON_EMIT0_ID, next);
		}

		k_spin_unlock(&g_lock, key);
		sys_clock_announce(dticks);
	}
}

void sys_clock_set_timeout(int32_t ticks, bool idle)
{
	ARG_UNUSED(idle);

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return;
	}

	k_spinlock_key_t key = k_spin_lock(&g_lock);

	uint32_t now = aon_tmr2_get();
	uint32_t cyc = ticks * CYC_PER_TICK;

	aon_tmr2_set_emit(AON_EMIT0_ID, COUNTER_ADD(now, cyc));

	k_spin_unlock(&g_lock, key);
}

uint32_t sys_clock_elapsed(void)
{
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return 0;
	}

	k_spinlock_key_t key = k_spin_lock(&g_lock);
	uint32_t ret = COUNTER_DIFF(aon_tmr2_get(), g_last_count) / CYC_PER_TICK;

	k_spin_unlock(&g_lock, key);
	return 0;
}

uint32_t sys_clock_cycle_get_32(void)
{
	return aon_tmr2_get();
}

uint64_t sys_clock_cycle_get_64(void)
{
	return aon_tmr2_get();
}

static int sys_clock_driver_init(void)
{
	/* AON timer2 as idle timer */
	aon_tmr2_clk_en();

	aon_tmr2_irq_clr();
	aon_tmr2_reload_en();

	aon_tmr2_init_tick(COUNTER_MAX);

	// enable
	aon_tmr2_en();
	delay_us(150);

	aon_tmr_int_clk_en();
	aon_tmr_emit_clk_en();
	aon_tmr_emit_int_mask_clear(AON_EMIT0_ID);
	aon_tmr_emit_en(AON_EMIT0_ID);

	aon_tmr_emit_wup_en(AON_EMIT0_ID);

	/* AON timer misc interrupt should be enabled for emit interrupt */
	IRQ_CONNECT(Aon_Tmr_Misc_IRQn, 2, aon_tmr2_isr, NULL, 0);
	irq_enable(Aon_Tmr_Misc_IRQn);
	return 0;
}

SYS_INIT(sys_clock_driver_init, POST_KERNEL, 2);

