/*
 * Copyright (c) 2023 Inplay Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/kernel.h"
#include "zephyr/sys_clock.h"
#include <zephyr/bluetooth/hci.h>

#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/drivers/bluetooth/hci_driver.h>

#include "in_ble_api.h"
#include "ble_app.h"
#include "in_irq.h"
#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_hci_driver_in6xx);

static struct k_thread in6xx_ble_thread_data;
static void *ble_hdl;
static K_KERNEL_STACK_DEFINE(in6xx_ble_thread_stack, 4096);

static K_SEM_DEFINE(ble_stack_sem, 0, 10);

#define HCI_CMD                 0x01
#define HCI_ACL                 0x02
#define HCI_SCO                 0x03
#define HCI_EVT                 0x04
#define HCI_ISO                 0x05

static void in6xx_ble_thread(void *p1, void *p2, void *p3)
{
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

static int bt_in6xx_open(void)
{
	k_tid_t tid;

	tid = k_thread_create(&in6xx_ble_thread_data, in6xx_ble_thread_stack,
			      K_KERNEL_STACK_SIZEOF(in6xx_ble_thread_stack),
			      in6xx_ble_thread, NULL, NULL, NULL,
			      K_PRIO_COOP(0), K_FP_REGS, K_NO_WAIT);
    if (tid != NULL) {
        printk("ble stack task created\n");
    }
	k_thread_name_set(tid, "in6xx_ble_thread");

    return 0;
}
static void __attribute__((section("ISR"))) bt_in6xx_isr(const struct device *dev)
{
	ble_isr_cb(NULL);
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

static int bt_in6xx_init(void)
{
    ble_stack_cb_t ble_stack_cb;
    uint8_t bd_addr[6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0xAA};
    ble_stack_cb.stk_sig = ble_stack_signal;
    ble_stack_cb_set(&ble_stack_cb);
    ble_hdl = ble_stack_init(bd_addr);
    if (ble_hdl == NULL) {
        return -1;
    }
    in_ble_vhci_host_register_callback(in_ble_vhci_rx_cb);
    bt_hci_driver_register(&drv);
	IRQ_CONNECT(Ble_IRQn,
			    1,
			    bt_in6xx_isr,	
			    NULL,
			    0);
	irq_enable(Ble_IRQn);	

    return 0;
}

SYS_INIT(bt_in6xx_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

