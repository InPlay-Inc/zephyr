/*
 * Copyright (c) 2023 InPlay Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//#include <zephyr/drivers/clock_control.h>

#include <zephyr/drivers/pinctrl.h>
#include "hal/hal_gpio.h"

static void pinctrl_configure_pin(const pinctrl_soc_pin_t *pin)
{
	uint32_t pin_cfg = *(uint32_t *)pin;
	hal_gpio_pin_cfg(pin_cfg);
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
			   uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		pinctrl_configure_pin(pins++);
	}

	return 0;
}
