/*
 * Copyright (c) 2023 Inplay Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "in_irq.h"
#include "zephyr/devicetree.h"
#include "zephyr/sys/__assert.h"
#include <sys/errno.h>
#define DT_DRV_COMPAT inplay_in6xxe_aon_tmr

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/gd32.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include <hal/hal_timer.h>
#include <hal/hal_clk.h>

#define AON_TMR_CTRL(base) (base + 0)
#define AON_TMR_CTRL_STS(base) (base + 4)
#define AON_TMR_INIT_VAL(base) (base + 0x28)
#define AON_TMR_INIT_STS(base) (base + 0x2C)
#define AON_TMR_READ_VAL(base) (base + 0xE8)

#define AON_TMR_MAX_VAL 0xffffffff

#define WAIT_AND_COMPARE(timeout, val1, val2) do {\
	int retry = timeout; \
	delay_us(150); \
	while (((val1) != (val2)) && retry--) { \
		clk_delay(1); \
	} \
	__ASSERT(retry > 0,"file:%s, line:%s\n", __FILE__, __LINE__); \
} while (0)

extern void delay_us(int);

struct in6xxe_aon_tmr_ch_data {
	counter_alarm_callback_t callback;
	void *user_data;
};

struct in6xxe_aon_tmr_data {
	counter_top_callback_t top_cb;
	void *top_user_data;
	struct in6xxe_aon_tmr_ch_data alarm[];
};

struct in6xxe_aon_tmr_config {
	struct counter_config_info counter_info;
	uint32_t base;   /* Control register base address */
	uint8_t index;
	bool is_64bit;
};

static const struct device *aon_tmr_devs[AON_TMR_MAX];

static void aon_tmr_isr(void *arg)
{
    uint32_t status = aon_tmr_int_status();
    uint32_t mask = aon_tmr_int_mask_status();
    aon_tmr_int_clear(status);
    status &= ~mask;

    for (uint32_t i = 0; i < AON_TMR_MAX; i++) {
        if ((status >> i) & 1) {
			const struct device *dev = aon_tmr_devs[i];
			const struct in6xxe_aon_tmr_config *config = dev->config;
			struct in6xxe_aon_tmr_data *data = dev->data;
			uint32_t ctl_reg = AON_TMR_CTRL(config->base);
			uint32_t sts_reg = AON_TMR_CTRL_STS(config->base);
			aon_tmr_irq_clr(ctl_reg, sts_reg);
			if (data->top_cb) {
				data->top_cb(dev, data->top_user_data);
			}
        }
    }
}

static int in6xxe_aon_tmr_start(const struct device *dev)
{
	const struct in6xxe_aon_tmr_config *config = dev->config;
	uint32_t ctl_reg = AON_TMR_CTRL(config->base);
	aon_tmr_en(ctl_reg);
	WAIT_AND_COMPARE(100, aon_tmr_en_sts(ctl_reg), 1);
	return 0;
}

static int in6xxe_aon_tmr_stop(const struct device *dev)
{
	const struct in6xxe_aon_tmr_config *config = dev->config;
	uint32_t ctl_reg = AON_TMR_CTRL(config->base);
	aon_tmr_dis(ctl_reg);
	WAIT_AND_COMPARE(100, aon_tmr_en_sts(ctl_reg), 0);
	return 0;
}

static int in6xxe_aon_tmr_get_value(const struct device *dev, uint32_t *ticks)
{
	uint64_t tick, init_tick;
	const struct in6xxe_aon_tmr_config *config = dev->config;
    if (config->index == AON_TMR3_ID) { /* AON timer 3, 64bits */
		int key = irq_lock();;
        aon_tmr3_snap_tick();
        tick = aon_tmr3_read_tick();
		init_tick = aon_tmr3_init_tick_sts();
		irq_unlock(key);
    } else {
		int key = irq_lock();;
        aon_tmr_snap_tick(config->index);
        tick = aon_tmr_read_tick(AON_TMR_READ_VAL(config->base));
		init_tick = aon_tmr_read_tick(AON_TMR_INIT_STS(config->base));
		irq_unlock(key);
    }
	if (tick <= init_tick) {
		*ticks = init_tick - tick;
	} else {
		*ticks = 0;
	}
	return 0;
}

static int in6xxe_aon_tmr_set_alarm(const struct device *dev, uint8_t chan_id,
				     const struct counter_alarm_cfg *alarm_cfg)
{
	const struct in6xxe_aon_tmr_config *config = dev->config;
	struct in6xxe_aon_tmr_data *data = dev->data;
	struct in6xxe_aon_tmr_ch_data *ch_data = &data->alarm[chan_id];
	if (config->index != AON_TMR2_ID) {
		return -ENOTSUP;
	}
	if (alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) {
		return -ENOTSUP;
	}
	ch_data->callback = alarm_cfg->callback;
	ch_data->user_data = alarm_cfg->user_data;

	// is timer disable?
	if (!aon_tmr2_dis_sts()) {
		aon_tmr2_dis();
		delay_us(150);
	}

	// write the tick count
	aon_tmr2_init_tick(alarm_cfg->ticks);

	// enable
	aon_tmr2_en();
	delay_us(150);

	//aon_tmr_int_clk_en();
	//aon_tmr_int_mask_clear(AON_TMR2_ID);

	return 0;
}

static int in6xxe_aon_tmr_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	const struct in6xxe_aon_tmr_config *config = dev->config;
	if (config->index != AON_TMR2_ID) {
		return -ENOTSUP;
	}

	aon_tmr2_dis();
	aon_tmr2_irq_clr();

	return 0;
}

static int in6xxe_aon_tmr_set_top_value(const struct device *dev,
					 const struct counter_top_cfg *cfg)
{
	const struct in6xxe_aon_tmr_config *config = dev->config;
	struct in6xxe_aon_tmr_data *data = dev->data;

	data->top_cb = cfg->callback;
	data->top_user_data = cfg->user_data;
	if (config->index == AON_TMR3_ID) {
		aon_tmr3_init_tick(cfg->ticks);
		WAIT_AND_COMPARE(100, aon_tmr3_init_tick_sts(), cfg->ticks);
	} else {
		aon_tmr_init_tick(AON_TMR_INIT_VAL(config->base), cfg->ticks);
		WAIT_AND_COMPARE(100, AON_TMR_INIT_STS(config->base), cfg->ticks);
	}
	if (cfg->callback != NULL) {
		aon_tmr_int_clk_en();
        aon_tmr_int_mask_clear(config->index);
	}
	return 0;
}

static uint32_t in6xxe_aon_tmr_get_pending_int(const struct device *dev)
{
	const struct in6xxe_aon_tmr_config *config = dev->config;
	return aon_tmr_int_status() & BIT(config->index);
}

static uint32_t in6xxe_aon_tmr_get_top_value(const struct device *dev)
{
	const struct in6xxe_aon_tmr_config *config = dev->config;
	if (config->index == AON_TMR3_ID) {
		return aon_tmr3_init_tick_sts();
	} else {
		return aon_tmr_init_tick_sts(AON_TMR_INIT_STS(config->base));
	}
}

static uint32_t in6xxe_aon_tmr_get_freq(const struct device *dev)
{
	return hal_clk_32k_get();
}

static const struct counter_driver_api counter_api = {
	.start = in6xxe_aon_tmr_start,
	.stop = in6xxe_aon_tmr_stop,
	.get_value = in6xxe_aon_tmr_get_value,
	.set_alarm = in6xxe_aon_tmr_set_alarm,
	.cancel_alarm = in6xxe_aon_tmr_cancel_alarm,
	.set_top_value = in6xxe_aon_tmr_set_top_value,
	.get_pending_int = in6xxe_aon_tmr_get_pending_int,
	.get_top_value = in6xxe_aon_tmr_get_top_value,
	.get_freq = in6xxe_aon_tmr_get_freq,
};

static int in6xxe_aon_timer_init(const struct device *dev)
{
	const struct in6xxe_aon_tmr_config *config = dev->config;

	IRQ_CONNECT(Aon_Tmr_Misc_IRQn, 2, aon_tmr_isr, NULL, 0);
	irq_enable(Aon_Tmr_Misc_IRQn);

	aon_tmr_devs[config->index] = dev;

	if (config->index == AON_TMR2_ID) {
		/* AON timer2 as idle timer */
		aon_tmr2_clk_en();
		aon_tmr_wup_en(AON_TMR2_ID);
		aon_tmr2_irq_clr();
		aon_tmr_int_mask_set(AON_TMR2_ID);
	} else {
		/* enable clock */
		aon_tmr_clk_en(config->index);
		delay_us(150);  
		aon_tmr_reload_en(config->index);
		aon_tmr_wup_en(config->index);
	}

	return 0;
}

#define IN6XXE_AON_TMR_DATA(n) \
	static struct in6xxe_aon_tmr_data_##n { \
		struct in6xxe_aon_tmr_data data; \
		struct in6xxe_aon_tmr_ch_data alarm[DT_INST_PROP(n, channel_num)]; \
	} aon_tmr_data_##n = {0}; \

#define IN6XXE_AON_TMR_INIT(n)   \
	IN6XXE_AON_TMR_DATA(n) \
	static const struct in6xxe_aon_tmr_config aon_tmr_config_##n = { \
		.counter_info = { \
			.max_top_value = AON_TMR_MAX_VAL, \
			.flags = 0, \
			.freq = 0, \
			.channels = DT_INST_PROP(n, channel_num), \
		}, \
		.base = DT_INST_REG_ADDR(n), \
		.index = DT_INST_PROP(n, index),      \
		.is_64bit = DT_INST_PROP(n, is_64bit), \
	};  \
	DEVICE_DT_INST_DEFINE(n, in6xxe_aon_timer_init, \
			NULL, &aon_tmr_data_##n, &aon_tmr_config_##n, POST_KERNEL, \
			CONFIG_COUNTER_INIT_PRIORITY, &counter_api);

DT_INST_FOREACH_STATUS_OKAY(IN6XXE_AON_TMR_INIT);

