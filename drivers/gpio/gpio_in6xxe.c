/*
 * Copyright (c) 2023 InPlay Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT inplay_in6xxe_gpio

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/clock_control.h>
//#include <zephyr/drivers/interrupt_controller/in6xxe_exti.h>
#include <zephyr/drivers/reset.h>


#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/irq.h>

#include <hal/hal_gpio.h>
#include <hal/hal_clk.h>

struct gpio_in6xxe_config {
	struct gpio_driver_config common;
	uint8_t port;
	uint8_t irq;
	uint8_t irq_priority;
	uint8_t ngpios;
};

struct gpio_in6xxe_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	sys_slist_t callbacks;
};

/**
 * @brief EXTI ISR callback.
 *
 * @param line EXTI line (equals to GPIO pin number).
 * @param arg GPIO port instance.
 */
static void gpio_in6xxe_isr(const void *arg)
{
	const struct device *dev = arg;
	struct gpio_in6xxe_data *data = dev->data;
	const struct gpio_in6xxe_config *config = dev->config;
	uint8_t ngpios = config->ngpios;

	uint32_t pins, status;
	if (config->port == 0) {
		status = gpio_port_0_int_status();
		gpio_port_0_int_clear_all();
	} else if (config->port == 1) {
		status = gpio_port_1_int_status();
		gpio_port_1_int_clear_all();
	} else if (config->port == 2) {
		status = gpio_port_2_int_status();
		gpio_port_2_int_clear_all();
	} else if (config->port == 3) {
		status = gpio_port_3_int_status();
		gpio_port_3_int_clear_all();
	} else if (config->port == 4) {
		status = gpio_port_4_int_status();
		gpio_port_4_int_clear_all();
	} else {
		return;
	}

	status &= GPIO_PORT_PIN_MASK_FROM_NGPIOS(ngpios<<1);
	pins = ((status >> ngpios) | (status & GPIO_PORT_PIN_MASK_FROM_NGPIOS(ngpios)));
	gpio_fire_callbacks(&data->callbacks, dev, pins);
}

/**
 * @brief Configure EXTI source selection register.
 *
 * @param port GPIO port instance.
 * @param pin GPIO pin number.
 *
 * @retval 0 on success.
 * @retval -EINVAL if pin is not valid.
 */
static int gpio_in6xxe_configure_extiss(const struct device *dev,
				      gpio_pin_t pin)
{
	return 0;
}

static inline int gpio_in6xxe_configure(const struct device *dev, gpio_pin_t pin,
				      gpio_flags_t flags)
{
	const struct gpio_in6xxe_config *const cfg = dev->config;
	int port = cfg->port;
	int res = 0;
	res = hal_gpio_pin_mux(port, pin, 0, 0);
	if (res)
		return -EINVAL;
	if (flags & GPIO_PULL_UP) {
		res = hal_gpio_pad_pd_pu(port, pin, 0, 1);
	} else if (flags & GPIO_PULL_DOWN) {
		res = hal_gpio_pad_pd_pu(port, pin, 1, 0);
	} else {
		res = hal_gpio_pad_pd_pu(port, pin, 0, 0);
	}

	if (flags & GPIO_OUTPUT) {
		res = hal_gpio_pad_oe_ie(port, pin, 1, 0);
		if (flags & GPIO_OUTPUT_HIGH) {
			res = hal_gpio_output(port, pin, 1, 1);
		} else if (flags & GPIO_OUTPUT_HIGH) {
			res = hal_gpio_output(port, pin, 0, 1);
		} else {
			res = hal_gpio_output(port, pin, 0, 1);
		}		
		
	}
	if (flags & GPIO_INPUT) {
		res = hal_gpio_pad_oe_ie(port, pin, 0, 1);
		res = hal_gpio_output(port, pin, 0, 0);
	}
	hal_gpio_sleep_pad_latch(port, pin, 1, 0);
	if (res)
		res = -EINVAL;
	return res;
}

static int gpio_in6xxe_port_get_raw(const struct device *dev, uint32_t *value)
{
	const struct gpio_in6xxe_config *config = dev->config;
	int port = config->port;
	for (int i = 0; i < 16; i++) {
		int lvl = hal_gpio_input_status(port, i);
		if (lvl)
			*value |= BIT(i);
		else
			*value &= ~BIT(i);
	}

	return 0;
}

static int gpio_in6xxe_port_set_masked_raw(const struct device *dev,
					 gpio_port_pins_t mask,
					 gpio_port_value_t value)
{
	const struct gpio_in6xxe_config *config = dev->config;
	int port = config->port;
	int res = -EINVAL;
	for (int i = 0; i < 16; i++) {
		if (mask & BIT(i)) {
			res = hal_gpio_output(port, i, (value>>i)&0x1, 1);		
		}
	}

	return 0;
}

static int gpio_in6xxe_port_set_bits_raw(const struct device *dev,
				       gpio_port_pins_t pins)
{
	const struct gpio_in6xxe_config *config = dev->config;
	int port = config->port;
	int res = -EINVAL;
	for (int i = 0; i < 16; i++) {
		if (pins & BIT(i)) {
			res = hal_gpio_output(port, i, 1, 1);		
		}
	}
	return res;
}

static int gpio_in6xxe_port_clear_bits_raw(const struct device *dev,
					 gpio_port_pins_t pins)
{
	const struct gpio_in6xxe_config *config = dev->config;
	int port = config->port;
	int res = -EINVAL;
	for (int i = 0; i < 16; i++) {
		if (pins & BIT(i)) {
			res = hal_gpio_output(port, i, 0, 1);
			
		}
	}
	return res;
}

static int gpio_in6xxe_port_toggle_bits(const struct device *dev,
				      gpio_port_pins_t pins)
{
	const struct gpio_in6xxe_config *config = dev->config;
	int port = config->port;
	int res = -EINVAL;
	for (int i = 0; i < 16; i++) {
		if (pins & BIT(i)) {
			int lvl = hal_gpio_output_status(port, i);
			lvl = !lvl;
			res = hal_gpio_output(port, i, lvl, 1);
			
		}
	}
	return res;
}

static int gpio_in6xxe_pin_interrupt_configure(const struct device *dev,
					     gpio_pin_t pin,
					     enum gpio_int_mode mode,
					     enum gpio_int_trig trig)
{
	const struct gpio_in6xxe_config *config = dev->config;
	irq_connect_dynamic(config->irq, config->irq_priority, gpio_in6xxe_isr, dev, 0);
	int res = 0;

	/* Enable GPIO interrupt clk */
	hal_clk_gpio_intr(config->port, 1);
	if (mode == GPIO_INT_MODE_DISABLED) {
		res = hal_gpio_ext_int_mask(config->port, pin, 0, 0, 0);
		irq_disable(config->irq);
	} else if (mode == GPIO_INT_MODE_EDGE) {
		res = hal_gpio_ext_int_unmask(config->port, pin, trig & GPIO_INT_TRIG_HIGH,
				trig & GPIO_INT_TRIG_LOW, 0);
		irq_enable(config->irq);
	} else {
		/* not supported */
	}

	return res == GPIO_ERR_NO_ERROR ? 0 : -EINVAL;
}

static int gpio_in6xxe_manage_callback(const struct device *dev,
				     struct gpio_callback *callback, bool set)
{
	struct gpio_in6xxe_data *data = dev->data;
	return gpio_manage_callback(&data->callbacks, callback, set);
}

static const struct gpio_driver_api gpio_in6xxe_api = {
	.pin_configure = gpio_in6xxe_configure,
	.port_get_raw = gpio_in6xxe_port_get_raw,
	.port_set_masked_raw = gpio_in6xxe_port_set_masked_raw,
	.port_set_bits_raw = gpio_in6xxe_port_set_bits_raw,
	.port_clear_bits_raw = gpio_in6xxe_port_clear_bits_raw,
	.port_toggle_bits = gpio_in6xxe_port_toggle_bits,
	.pin_interrupt_configure = gpio_in6xxe_pin_interrupt_configure,
	.manage_callback = gpio_in6xxe_manage_callback,
};

static int gpio_in6xxe_init(const struct device *port)
{
	const struct gpio_in6xxe_config *config = port->config;
	hal_gpio_init();
	return 0;
}

#define GPIO_INPLAY_DEVICE(id)						\
	static const struct gpio_in6xxe_config gpio_in6xxe_p##id##_config = {	\
		.common = {						\
			.port_pin_mask =				\
			GPIO_PORT_PIN_MASK_FROM_DT_INST(id),		\
		},							\
		.port = id,		\
		.irq = DT_IRQ_BY_IDX(DT_DRV_INST(id), 0, irq), \
		.irq_priority  = DT_IRQ_BY_IDX(DT_DRV_INST(id), 0, priority), \
		.ngpios = DT_PROP(DT_DRV_INST(id), ngpios), \
	};								\
									\
	static struct gpio_in6xxe_data gpio_in6xxe_p##id##_data;		\
									\
	DEVICE_DT_INST_DEFINE(id, gpio_in6xxe_init,			\
			 NULL,						\
			 &gpio_in6xxe_p##id##_data,			\
			 &gpio_in6xxe_p##id##_config,			\
			 PRE_KERNEL_1,					\
			 CONFIG_GPIO_INIT_PRIORITY,			\
			 &gpio_in6xxe_api);
DT_INST_FOREACH_STATUS_OKAY(GPIO_INPLAY_DEVICE)
