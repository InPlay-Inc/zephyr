/*
 * Copyright (c) 2023 InPlay Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/irq.h"
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>

#include <cmsis_core.h>

#include <hal/hal_global.h>

#define Calib_IRQn 36
#define Tmr2_IROn 28

#define LOG_LEVEL CONFIG_SOC_LOG_LEVEL
LOG_MODULE_REGISTER(soc);

extern void SystemInit(void);
extern void Calib_Handler(void *arg);
extern void TMR2_Handler(void *arg);

#ifdef CONFIG_TIMER_READS_ITS_FREQUENCY_AT_RUNTIME
extern int z_clock_hw_cycles_per_sec; 
#endif

static int in6xxe_pre_init(void)
{
    SystemInit();
	return 0;
}

SYS_INIT(in6xxe_pre_init, PRE_KERNEL_1, 0);

static int in6xxe_post_init(void)
{
	IRQ_CONNECT(Calib_IRQn, 2, Calib_Handler, NULL, 0);
	IRQ_CONNECT(Tmr2_IROn, 2, TMR2_Handler, NULL, 0);
	hal_global_post_init();

#ifdef CONFIG_TIMER_READS_ITS_FREQUENCY_AT_RUNTIME
	z_clock_hw_cycles_per_sec = hal_clk_32k_get();
#endif
	return 0;
}

SYS_INIT(in6xxe_post_init, POST_KERNEL, 0);

