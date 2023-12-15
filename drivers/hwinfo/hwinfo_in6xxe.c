/*
 * Copyright (c) 2023 Inplay Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/hwinfo.h>
#include <string.h>

#include "hal/hal_efuse.h"

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	uint32_t uuid[3];
	hal_efuse_get_uuid(uuid);
	if (length >= sizeof(uuid)) {
		length = sizeof(uuid);
	}
	memcpy(buffer, uuid, length);
	return length;
}

