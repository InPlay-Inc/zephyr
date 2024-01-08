/* h4.c - H:4 UART based Bluetooth driver */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>

#include <zephyr/init.h>
//#include <zephyr/drivers/uart.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/drivers/bluetooth/hci_driver.h>
#include <zephyr/device.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>

#include "in_ble_api.h"
#include "ble_app.h"
#include "in_irq.h"
#include "hal/hal_trng.h"
#include <stdlib.h>

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_hci_driver_in6xx);
#include "common/bt_str.h"

#define PM_DEEP_SLEEP  2
#define MIN_SLEEP_TIME 5000 //us

#define HCI_NONE				0x00
#define HCI_CMD                 0x01
#define HCI_ACL                 0x02
#define HCI_SCO                 0x03
#define HCI_EVT                 0x04
#define HCI_ISO                 0x05

#define CNTL_RX_EVT 0x01
#define HOST_TX_EVT 0x02

typedef  void(*HCI_RX_CB)(void *arg, uint8_t status);

static struct k_thread in6xx_ble_thread_data;
static void *ble_hdl;
static uint32_t g_sleep_dur;
static K_KERNEL_STACK_DEFINE(in6xx_ble_thread_stack, 4096);

static K_SEM_DEFINE(ble_stack_sem, 0, 10);

#ifdef CONFIG_IN6XXE_HCI_TL
static K_KERNEL_STACK_DEFINE(host_tx_thread_stack, 2048);
static struct k_thread host_tx_thread_data;

static struct {
	uint8_t type;
	uint8_t have_type;
	struct net_buf *buf;
	struct k_fifo   fifo;
	struct k_fifo   rx_fifo;//host RX
	void(*cntl_rx_cb)(void *arg, uint8_t status);
	void *arg;
	uint8_t *cntl_rx_buf;
	uint16_t cntl_rx_len;
	struct k_sem sem;
} host_tx = {
	//.fifo = Z_FIFO_INITIALIZER(host_tx.fifo),
	.cntl_rx_cb = NULL,
};
#endif

extern int rwip_power_state(void *arg, uint32_t *duration);
extern void rwip_power_up(void *arg);
extern void rwip_power_down(void *arg, uint32_t duration);

/* Check if BLE can enter deep sleep */
bool in6xxe_ble_sleep(void)
{
	g_sleep_dur = 0xffffffff;
	int state = rwip_power_state(ble_hdl, &g_sleep_dur);
	return (state == PM_DEEP_SLEEP && g_sleep_dur >= MIN_SLEEP_TIME);
}

static void in6xx_ble_thread(void *p1, void *p2, void *p3)
{
	uint32_t seed = hal_trng_gen();
	srand(seed);

	while (true) {
		k_sem_take(&ble_stack_sem, K_FOREVER);
		in_ble_stack_handler(ble_hdl);
	}
}

static void ble_stack_signal(void)
{
    k_sem_give(&ble_stack_sem);
}

static bool is_hci_event_discardable(const uint8_t *evt_data)
{
	uint8_t evt_type = evt_data[0];

	switch (evt_type) {
#if defined(CONFIG_BT_BREDR)
	case BT_HCI_EVT_INQUIRY_RESULT_WITH_RSSI:
	case BT_HCI_EVT_EXTENDED_INQUIRY_RESULT:
		return true;
#endif
	case BT_HCI_EVT_LE_META_EVENT: {
		uint8_t subevt_type = evt_data[sizeof(struct bt_hci_evt_hdr)];

		switch (subevt_type) {
		case BT_HCI_EVT_LE_ADVERTISING_REPORT:
			return true;
		default:
			return false;
		}
	}
	default:
		return false;
	}
}

static struct net_buf *bt_in6xx_evt_recv(uint8_t *data, size_t remaining)
{
	bool discardable = false;
	struct bt_hci_evt_hdr hdr;
	struct net_buf *buf;
	size_t buf_tailroom;

	if (remaining < sizeof(hdr)) {
		LOG_ERR("Not enough data for event header");
		return NULL;
	}

	discardable = is_hci_event_discardable(data);

	memcpy((void *)&hdr, data, sizeof(hdr));
	data += sizeof(hdr);
	remaining -= sizeof(hdr);

	if (remaining != hdr.len) {
		LOG_ERR("Event payload length is not correct");
		return NULL;
	}
	LOG_DBG("len %u", hdr.len);

	buf = bt_buf_get_evt(hdr.evt, discardable, K_NO_WAIT);
	if (!buf) {
		if (discardable) {
			LOG_DBG("Discardable buffer pool full, ignoring event");
		} else {
			LOG_ERR("No available event buffers!");
		}
		return buf;
	}

	net_buf_add_mem(buf, &hdr, sizeof(hdr));

	buf_tailroom = net_buf_tailroom(buf);
	if (buf_tailroom < remaining) {
		LOG_ERR("Not enough space in buffer %zu/%zu", remaining, buf_tailroom);
		net_buf_unref(buf);
		return NULL;
	}

	net_buf_add_mem(buf, data, remaining);

	return buf;
}

static struct net_buf *bt_in6xx_acl_recv(uint8_t *data, size_t remaining)
{
	struct bt_hci_acl_hdr hdr;
	struct net_buf *buf;
	size_t buf_tailroom;

	if (remaining < sizeof(hdr)) {
		LOG_ERR("Not enough data for ACL header");
		return NULL;
	}

	buf = bt_buf_get_rx(BT_BUF_ACL_IN, K_NO_WAIT);
	if (buf) {
		memcpy((void *)&hdr, data, sizeof(hdr));
		data += sizeof(hdr);
		remaining -= sizeof(hdr);

		net_buf_add_mem(buf, &hdr, sizeof(hdr));
	} else {
		LOG_ERR("No available ACL buffers!");
		return NULL;
	}

	if (remaining != sys_le16_to_cpu(hdr.len)) {
		LOG_ERR("ACL payload length is not correct");
		net_buf_unref(buf);
		return NULL;
	}

	buf_tailroom = net_buf_tailroom(buf);
	if (buf_tailroom < remaining) {
		LOG_ERR("Not enough space in buffer %zu/%zu", remaining, buf_tailroom);
		net_buf_unref(buf);
		return NULL;
	}

	LOG_DBG("len %u", remaining);
	net_buf_add_mem(buf, data, remaining);

	return buf;
}

static void __attribute__((section("ISR"))) bt_in6xx_isr(const struct device *dev)
{
	ble_isr_cb(NULL);
}

#ifdef CONFIG_IN6XXE_HCI_TL
static void cntl_hci_read(void *hdl, uint8_t *buffer, uint16_t buffer_len, void(*callback)(void *arg, uint8_t status), void *arg)
{
	ARG_UNUSED(hdl);

	host_tx.cntl_rx_cb = callback;
	host_tx.arg = arg;
	host_tx.cntl_rx_buf = buffer;
	host_tx.cntl_rx_len = buffer_len;
	k_sem_give(&host_tx.sem);
	return;
}
static void host_tx_thread(void *p1, void *p2, void *p3)
{
	//struct net_buf *buf = host_tx.buf;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);


	
	while (1) {
		//LOG_DBG("wait event");
		k_sem_take(&host_tx.sem, K_FOREVER);

		if (!host_tx.buf) {
			host_tx.buf = net_buf_get(&host_tx.fifo, K_NO_WAIT);
			if (!host_tx.buf) {
				//LOG_DBG("TX no pending buffer!");
				continue;
			}
			//uint8_t pkt_indicator;

			//LOG_DBG("buf %p type %u len %u", host_tx.buf, bt_buf_get_type(host_tx.buf), host_tx.buf->len);

			switch (bt_buf_get_type(host_tx.buf)) {
				case BT_BUF_ACL_OUT:
					host_tx.type = HCI_ACL;
					host_tx.have_type = 1;
					break;
				case BT_BUF_CMD:
					host_tx.type = HCI_CMD;
					host_tx.have_type = 1;
					break;
				default:
					LOG_ERR("Unknown type %u", bt_buf_get_type(host_tx.buf));
					host_tx.type = HCI_NONE;
					net_buf_unref(host_tx.buf);
					host_tx.buf = NULL;
					host_tx.have_type = 0;
			}

			//net_buf_push_u8(host_tx.buf, pkt_indicator);
			LOG_HEXDUMP_DBG(host_tx.buf->data, host_tx.buf->len, "HOST TX:");
		}

		


		if (host_tx.cntl_rx_cb) {
			//LOG_DBG("buf len %d rx len %d", host_tx.buf->len, host_tx.cntl_rx_len);
			uint16_t len = host_tx.buf->len;
			if (host_tx.have_type) {
				len += 1;
			}
			if ((len >= host_tx.cntl_rx_len) && (host_tx.cntl_rx_buf != NULL) && (host_tx.cntl_rx_len != 0)) {
				if (host_tx.have_type) {
					host_tx.cntl_rx_buf[0] = host_tx.type;
					host_tx.cntl_rx_len--;
					host_tx.cntl_rx_buf++;
					host_tx.have_type = 0;
				}
				if (host_tx.cntl_rx_len > 0) {
					memcpy(host_tx.cntl_rx_buf, host_tx.buf->data, host_tx.cntl_rx_len);
					//LOG_HEXDUMP_DBG(host_tx.cntl_rx_buf, host_tx.cntl_rx_len, "HCI rx");
					net_buf_pull(host_tx.buf, host_tx.cntl_rx_len);
				}
				//LOG_HEXDUMP_DBG(host_tx.cntl_rx_buf, host_tx.cntl_rx_len, "HCI rx");


				HCI_RX_CB cb = host_tx.cntl_rx_cb;
				host_tx.cntl_rx_cb = NULL;
				host_tx.cntl_rx_buf = NULL;


				if (host_tx.buf->len == 0) {
					host_tx.type = HCI_NONE;
					net_buf_unref(host_tx.buf);
					host_tx.buf = NULL;
					host_tx.have_type = 0;
				}
				cb(host_tx.arg, 0);

				
			} else {
				//memcpy(host_tx.cntl_rx_buf, host_tx.buf->data, host_tx.buf->len);
				//net_buf_pull(host_tx.buf, host_tx.buf->len);
				printk("err cb\n");
				HCI_RX_CB cb = host_tx.cntl_rx_cb;
				host_tx.cntl_rx_cb = NULL;
				host_tx.cntl_rx_buf = NULL;
				cb(host_tx.arg, 1);
				//host_tx.cntl_rx_cb = NULL;
			
				host_tx.type = HCI_NONE;
				net_buf_unref(host_tx.buf);
				host_tx.buf = NULL;
				host_tx.have_type = 0;
			}
		} 
		
	}
	
}
static void cntl_hci_write(void *hdl, uint8_t *buffer, uint16_t buffer_len, void (*callback)(void *arg, uint8_t status), void *arg)
{
	uint8_t pkt_indicator;
	struct net_buf *buf = NULL;
	uint8_t status = 0;

	LOG_HEXDUMP_DBG(buffer, buffer_len, "uart WR:");
	if (!buffer || (buffer_len == 0)) {
		LOG_ERR("Invalid buffer");
		return;
	}

	pkt_indicator = buffer[0];
	size_t remaining = buffer_len - 1;
	uint8_t *data = &buffer[1];
	switch (pkt_indicator) {
		case HCI_EVT:
			buf = bt_in6xx_evt_recv(data, remaining);
			break;

		case HCI_ACL:
			buf = bt_in6xx_acl_recv(data, remaining);
			break;

		default:
			LOG_ERR("Unknown HCI type %u", pkt_indicator);
			status = 1;
	}

	if (buf) {
		//LOG_DBG("Calling bt_recv(%p)", buf);
		//LOG_HEXDUMP_DBG(buf->data, buf->len, "RECV buffer:");
		//uint16_t op = buf->data[3] | (buf->data[4] << 8);

		struct bt_hci_evt_hdr *hdr = (struct bt_hci_evt_hdr *)data;
		uint8_t evt_flags = bt_hci_evt_get_flags(hdr->evt);
		if (IS_ENABLED(CONFIG_BT_RECV_BLOCKING) && (evt_flags & BT_HCI_EVT_FLAG_RECV_PRIO)) {
			bt_recv_prio(buf);
		} else {
			bt_recv(buf);
			//net_buf_put(&host_tx.rx_fifo, buf);
		}
	}
	if (callback) {
		callback(arg, status);
	}
	return;
}
static int bt_in6xx_send(struct net_buf *buf)
{
	//LOG_DBG("buf %p type %u len %u fifo %x", buf, bt_buf_get_type(buf), buf->len,&host_tx.fifo);
	//uint16_t op = buf->data[0] | (buf->data[1] << 8);

	net_buf_put(&host_tx.fifo, buf);
	k_sem_give(&host_tx.sem);

	return 0;
}

#else
static int bt_in6xx_send(struct net_buf *buf)
{
    int err = 0;
    uint8_t pkt_indicator;

	LOG_DBG("buf %p type %u len %u", buf, bt_buf_get_type(buf), buf->len);

	switch (bt_buf_get_type(buf)) {
	case BT_BUF_ACL_OUT:
		pkt_indicator = HCI_ACL;
		break;
	case BT_BUF_CMD:
		pkt_indicator = HCI_CMD;
		break;
	default:
		LOG_ERR("Unknown type %u", bt_buf_get_type(buf));
		goto done;
	}
	net_buf_push_u8(buf, pkt_indicator);

	LOG_HEXDUMP_DBG(buf->data, buf->len, "Final HCI buffer:");

    if (0 != in_ble_vhci_host_send(ble_hdl, buf->data, buf->len)) {
        LOG_ERR("in6xx hci send err\n");
    }

done:
	net_buf_unref(buf);

    return err;
}

void in_ble_vhci_rx_cb(uint8_t type, uint8_t *data, uint16_t len)
{
	uint8_t pkt_indicator;
	struct net_buf *buf = NULL;
	size_t remaining = len;

	LOG_HEXDUMP_DBG(data, len, "host packet data:");

	pkt_indicator = type;

	switch (pkt_indicator) {
	case HCI_EVT:
		buf = bt_in6xx_evt_recv(data, remaining);
		break;

	case HCI_ACL:
		buf = bt_in6xx_acl_recv(data, remaining);
		break;

	default:
		LOG_ERR("Unknown HCI type %u", pkt_indicator);
        return;
	}

	if (buf) {
		LOG_DBG("Calling bt_recv(%p)", buf);

		bt_recv(buf);
	}
}
#endif

static int bt_in6xx_open(void)
{
	int ret;
	k_tid_t tid;

#ifdef CONFIG_IN6XXE_HCI_TL
	ret = k_sem_init(&host_tx.sem, 0, 16);
	if (ret) {
		LOG_ERR("Cannot initialize sem (err: %d)", ret);
	}
	k_fifo_init(&host_tx.fifo);
	k_fifo_init(&host_tx.rx_fifo);
	tid = k_thread_create(&host_tx_thread_data, host_tx_thread_stack,
			      K_KERNEL_STACK_SIZEOF(host_tx_thread_stack),
			      host_tx_thread, NULL, NULL, NULL,
			      K_PRIO_COOP(CONFIG_BT_RX_PRIO),
			      0, K_NO_WAIT);
    if (tid == NULL) {
        LOG_ERR("Err: host_tx_thread\n");
    } else {				  
		k_thread_name_set(tid, "in6xxe_host_tx_thread");
	}
#endif

	tid = k_thread_create(&in6xx_ble_thread_data, in6xx_ble_thread_stack,
			      K_KERNEL_STACK_SIZEOF(in6xx_ble_thread_stack),
			      in6xx_ble_thread, NULL, NULL, NULL,
			      K_PRIO_COOP(0), K_FP_REGS, K_NO_WAIT);
    if (tid == NULL) {
        LOG_ERR("Err: ble stack task create\n");
    } else {
		k_thread_name_set(tid, "in6xx_ble_thread");
	}

    ble_stack_cb_t ble_stack_cb;
    uint8_t bd_addr[6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0xdd};
    ble_stack_cb.stk_sig = ble_stack_signal;
    ble_stack_cb_set(&ble_stack_cb);
#ifdef CONFIG_IN6XXE_HCI_TL	
	ble_register_hci_cb(cntl_hci_write, cntl_hci_read);
#endif	
    ble_hdl = ble_stack_init(bd_addr);
	
    if (ble_hdl == NULL) {
		LOG_ERR("Err:ble_stack_init\n");
        return -1;
    }
#ifndef CONFIG_IN6XXE_HCI_TL
    in_ble_vhci_host_register_callback(in_ble_vhci_rx_cb);
#endif	
	IRQ_CONNECT(Ble_IRQn,
			    1,
			    bt_in6xx_isr,	
			    NULL,
			    0);
	irq_enable(Ble_IRQn);

	return 0;
}

static const struct bt_hci_driver drv = {
	.name           = "BT IN6XX",
	.open           = bt_in6xx_open,
	.send           = bt_in6xx_send,
	.bus            = BT_HCI_DRIVER_BUS_IPM,
#if defined(CONFIG_BT_DRIVER_QUIRK_NO_AUTO_DLE)
	.quirks         = BT_QUIRK_NO_AUTO_DLE,
#endif
};

static int in6xxe_hci_pm_action(const struct device *dev,
                                  enum pm_device_action action)
{
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        /* suspend the device */
		rwip_power_down(ble_hdl, g_sleep_dur);
        break;
    case PM_DEVICE_ACTION_RESUME:
        /* resume the device */
		rwip_power_up(ble_hdl);
        break;
    default:
        return -ENOTSUP;
    }

    return 0;
}

static int bt_in6xx_init(const struct device *dev)
{
	ARG_UNUSED(dev);

    bt_hci_driver_register(&drv);

    return 0;
}

PM_DEVICE_DEFINE(in6xxe_ble_pm, in6xxe_hci_pm_action);

DEVICE_DEFINE(in6xxe_hci, "in6xxe_ble", bt_in6xx_init,
		PM_DEVICE_GET(in6xxe_ble_pm), NULL, NULL,
		POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);

