/*
 * Copyright (c) 2021, Inplay Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT inplay_in6xxe_uart

#include <errno.h>

#include <zephyr/drivers/clock_control.h>
//#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>

#include "hal/hal_uart.h"
#include "in_irq.h"

void *uart_hdl = NULL;

static void uart_in6xx_poll_out(const struct device *dev, unsigned char out_char)
{
    if (uart_hdl) {
        hal_uart_putc(uart_hdl, out_char);
    }
}

static const struct uart_driver_api uart_in6xx_driver_api = {
    .poll_out = uart_in6xx_poll_out,
};

static int uart_in6xx_init(const struct device *dev)
{
	uart_init_t init = {0};
	init.baud_rate = 921600;
	init.data_len = 8;
	init.stop_bit = UART_STOP_1BIT;
	init.no_intr = 1;
	init.prio = IRQ_PRI_Normal;
	uart_hdl = hal_uart_open(1, &init);
    if (uart_hdl) {
        return 0;
    }
    return -1;
}

#define IN6XX_UART_INIT(n)                              \
	DEVICE_DT_INST_DEFINE(n, &uart_in6xx_init,			\
			      NULL,						            \
			      NULL,				                    \
			      NULL,                PRE_KERNEL_1,	\
			      CONFIG_SERIAL_INIT_PRIORITY,			\
			      &uart_in6xx_driver_api);              \

DT_INST_FOREACH_STATUS_OKAY(IN6XX_UART_INIT)

