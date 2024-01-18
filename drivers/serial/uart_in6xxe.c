/*
 * Copyright (c) 2023 InPlay Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT inplay_in6xxe_uart

#include <errno.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
#include <zephyr/pm/device.h>

#include "hal/hal_clk.h"
#include "hal/hal_uart.h"
#include "in_irq.h"
#include "in_compile.h"

struct in6xx_uart_config {
	uint32_t base;
	const struct pinctrl_dev_config *pcfg;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_config_func_t irq_config_func;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

struct in6xx_uart_data {
	uint32_t baud_rate;
	bool need_resume;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t user_cb;
	void *user_data;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

static int RAM_PM uart_in6xx_init(const struct device *dev);

static void RAM_PM uart_resume(const struct device *dev)
{
	struct in6xx_uart_data *data = dev->data;
	if (data->need_resume) {
		uart_in6xx_init(dev);
		data->need_resume = false;
	}
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_in6xx_isr(const struct device *dev)
{
	struct in6xx_uart_data *const data = dev->data;
	if (data->user_cb) {
		data->user_cb(dev, data->user_data);
	}
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static int uart_in6xx_poll_in(const struct device *dev, unsigned char *p_char)
{
	const struct in6xx_uart_config *cfg = dev->config;
	uart_rcvd_ready(cfg->base);
	*p_char = uart_read_data(cfg->base);
	return 0;
}

static void uart_in6xx_poll_out(const struct device *dev, unsigned char out_char)
{
	const struct in6xx_uart_config *cfg = dev->config;
	uart_resume(dev);
    uart_xmit_ready(cfg->base);
    uart_write_data(cfg->base, out_char);
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int uart_in6xx_fifo_fill(const struct device *dev, const uint8_t *tx_data,
		int len)
{
	int filled_len = 0;
	const struct in6xx_uart_config *cfg = dev->config;
	while ((uart_usr(cfg->base) & UART_USR_TFNF) && len--) {
		uart_write_data(cfg->base, *tx_data++);
		filled_len++;
	}
	return filled_len;
}

static int uart_in6xx_fifo_read(const struct device *dev, uint8_t *rx_data,
			 const int size)
{
	int read_len = 0;
	const struct in6xx_uart_config *cfg = dev->config;
	while ((uart_usr(cfg->base) & UART_USR_RFNE) && (read_len <= size)) {
		*rx_data++ = uart_read_data(cfg->base);
		read_len++;
	}
	return read_len;
}

static void uart_in6xx_irq_tx_enable(const struct device *dev)
{
	const struct in6xx_uart_config *cfg = dev->config;
	uart_intr_enable(cfg->base, UART_IER_ETBEI);
}

static void uart_in6xx_irq_tx_disable(const struct device *dev)
{
	const struct in6xx_uart_config *cfg = dev->config;
	uart_intr_disable(cfg->base, UART_IER_ETBEI);
}

static int uart_in6xx_irq_tx_ready(const struct device *dev)
{
	const struct in6xx_uart_config *cfg = dev->config;
	return uart_usr(cfg->base) & UART_USR_TFNF;
}

static void uart_in6xx_irq_rx_enable(const struct device *dev)
{
	const struct in6xx_uart_config *cfg = dev->config;
	uart_intr_enable(cfg->base, UART_IER_ERBFI);
}

static void uart_in6xx_irq_rx_disable(const struct device *dev)
{
	const struct in6xx_uart_config *cfg = dev->config;
	uart_intr_disable(cfg->base, UART_IER_ERBFI);
}

static int uart_in6xx_irq_tx_complete(const struct device *dev)
{
	const struct in6xx_uart_config *cfg = dev->config;
	return (uart_usr(cfg->base) & UART_USR_TFE)
		&&(uart_intr_enable_get(cfg->base) & UART_IER_ETBEI);
}

static int uart_in6xx_irq_rx_ready(const struct device *dev)
{
	const struct in6xx_uart_config *cfg = dev->config;
	return uart_usr(cfg->base) & UART_USR_RFNE;
}

static void uart_in6xx_irq_err_enable(const struct device *dev)
{
}

static void uart_in6xx_irq_err_disable(const struct device *dev)
{
}

static int uart_in6xx_irq_is_pending(const struct device *dev)
{
	const struct in6xx_uart_config *cfg = dev->config;
	return uart_intr_status(cfg->base) != UART_IT_ID_NONE;
}

static int uart_in6xx_irq_update(const struct device *dev)
{
	return 1;
}

static void uart_in6xx_irq_callback_set(const struct device *dev,
				 uart_irq_callback_user_data_t cb,
				 void *user_data)
{
	struct in6xx_uart_data *data = dev->data;
	data->user_data = user_data;
	data->user_cb = cb;
}
#endif

static const struct uart_driver_api uart_in6xx_driver_api = {
	.poll_in = uart_in6xx_poll_in,
    .poll_out = uart_in6xx_poll_out,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_in6xx_fifo_fill,
	.fifo_read = uart_in6xx_fifo_read,
	.irq_tx_enable = uart_in6xx_irq_tx_enable,
	.irq_tx_disable = uart_in6xx_irq_tx_disable,
	.irq_tx_ready = uart_in6xx_irq_tx_ready,
	.irq_rx_enable = uart_in6xx_irq_rx_enable,
	.irq_rx_disable = uart_in6xx_irq_rx_disable,
	.irq_rx_ready = uart_in6xx_irq_rx_ready,
	.irq_tx_complete = uart_in6xx_irq_tx_complete,
	.irq_err_enable = uart_in6xx_irq_err_enable,
	.irq_err_disable = uart_in6xx_irq_err_disable,
	.irq_is_pending = uart_in6xx_irq_is_pending,
	.irq_update = uart_in6xx_irq_update,
	.irq_callback_set = uart_in6xx_irq_callback_set,
#endif
};


static int RAM_PM uart_in6xx_init(const struct device *dev)
{
	int ret;
	const struct in6xx_uart_config *cfg = dev->config;
	struct in6xx_uart_data *data = dev->data;

    // enable uart clock
    hal_clk_uart_en(0, 1);
    hal_clk_uart_en(1, 1);

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

    /// Set baud rate
    uart_set_baud(cfg->base, hal_clk_d0_get(), data->baud_rate);

    /// Set stop bits, parity, even, data_len
    uart_set_lcr(cfg->base, UART_STOP_1BIT, 0, 0, 8);

    /// Set FIFO control
    uart_fcr(cfg->base, 1, 0, 0, 0, 1, 1);

    /// Set Flow control
 	uart_auto_fc(cfg->base, 0);
	uart_rts_off(cfg->base);

    /// Mask all interrupt
    uart_intr_disable(cfg->base, UART_IER_ALL);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	cfg->irq_config_func(dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

	return 0;
}

static int uart_in6xx_pm_action(const struct device *dev,
                                  enum pm_device_action action)
{
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
		{
			/* suspend the device */
			struct in6xx_uart_data *data = dev->data;
			data->need_resume = true;
		}
        break;
    case PM_DEVICE_ACTION_RESUME:
        /* resume the device */
        break;
    default:
        return -ENOTSUP;
    }

    return 0;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define IN6XX_UART_IRQ_HANDLER(n)										\
	static void RAM_PM uart_in6xx_config_func_##n(const struct device *dev)	\
	{																	\
		IRQ_CONNECT(DT_INST_IRQN(n),									\
			    DT_INST_IRQ(n, priority),								\
			    uart_in6xx_isr,                                         \
			    DEVICE_DT_INST_GET(n),									\
			    0);														\
		irq_enable(DT_INST_IRQN(n));									\
	}
#define IN6XX_UART_IRQ_HANDLER_FUNC_INIT(n)							    \
	.irq_config_func = uart_in6xx_config_func_##n
#else /* CONFIG_UART_INTERRUPT_DRIVEN */
#define IN6XX_UART_IRQ_HANDLER(n)
#define IN6XX_UART_IRQ_HANDLER_FUNC_INIT(n)
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#define IN6XX_UART_INIT(n)                              				\
	PM_DEVICE_DT_INST_DEFINE(n, uart_in6xx_pm_action);                  \
	PINCTRL_DT_INST_DEFINE(n);						                    \
	IN6XX_UART_IRQ_HANDLER(n)                                           \
	static struct in6xx_uart_data uart_in6xx_data_##n = {				\
		.baud_rate = DT_INST_PROP(n, current_speed),					\
		.need_resume = 0,                                               \
	};									                             	\
	static const struct in6xx_uart_config uart_in6xx_config_##n = {		\
		.base = DT_INST_REG_ADDR(n),									\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                      \
		IN6XX_UART_IRQ_HANDLER_FUNC_INIT(n)                             \
	};                                                  				\
	DEVICE_DT_INST_DEFINE(n, &uart_in6xx_init,							\
			      PM_DEVICE_DT_INST_GET(n), \
			      &uart_in6xx_data_##n,				                    \
			      &uart_in6xx_config_##n,                PRE_KERNEL_1,	\
			      CONFIG_SERIAL_INIT_PRIORITY,							\
			      &uart_in6xx_driver_api);              				\

DT_INST_FOREACH_STATUS_OKAY(IN6XX_UART_INIT)

