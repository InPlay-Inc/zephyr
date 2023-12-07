/*
 * Copyright (c) 2023 InPlay Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT inplay_in6xxe_flash_controller


#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>

#include <hal/hal_flash.h>
#define FLASH_SECTOR_SIZE 4096
#define FLASH_PAGE_SIZE 256
LOG_MODULE_REGISTER(flash_in6xxe, CONFIG_FLASH_LOG_LEVEL);

struct flash_in6xxe_data {
	struct k_sem mutex;
};

static struct flash_in6xxe_data flash_data;

static const struct flash_parameters flash_in6xxe_parameters = {
	.write_block_size = 4,
	.erase_value = 0xff,
};

static int flash_in6xxe_read(const struct device *dev, off_t offset,
			   void *data, size_t len)
{
	int res;
	if (len == 0U) {
		return 0;
	}

	res = hal_spi_flash_read(offset, data, len);
	if (res) {
		res = -EINVAL;
	}

	return res;
}

static int flash_in6xxe_write(const struct device *dev, off_t offset,
			    const void *data, size_t len)
{
	struct flash_in6xxe_data *dev_data = dev->data;
	int ret = 0;

	if (len == 0U) {
		return 0;
	}

	k_sem_take(&dev_data->mutex, K_FOREVER);

	ret = hal_spi_flash_prog_page(offset, data, len);

	k_sem_give(&dev_data->mutex);
	if (ret) {
		ret = -EINVAL;
	}
	return ret;
}

static int flash_in6xxe_erase(const struct device *dev, off_t offset, size_t size)
{
	struct flash_in6xxe_data *data = dev->data;
	int ret = 0;

	if (size == 0U) {
		return 0;
	}

	uint32_t nb = (size + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE;
	k_sem_take(&data->mutex, K_FOREVER);

	ret = hal_spi_flash_sector_erase(offset, nb);

	k_sem_give(&data->mutex);
	if (ret) {
		ret = -EINVAL;
	}
	return ret;
}

static const struct flash_parameters*
flash_in6xxe_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_in6xxe_parameters;
}
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static struct flash_pages_layout dev_layout;

static void flash_in6xxe_pages_layout(const struct device *dev,
				     const struct flash_pages_layout **layout,
				     size_t *layout_size)
{
	*layout = &dev_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */
static const struct flash_driver_api flash_in6xxe_driver_api = {
	.read = flash_in6xxe_read,
	.write = flash_in6xxe_write,
	.erase = flash_in6xxe_erase,
	.get_parameters = flash_in6xxe_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_in6xxe_pages_layout,
#endif
};

static int flash_in6xxe_init(const struct device *dev)
{
	struct flash_in6xxe_data *data = dev->data;

	k_sem_init(&data->mutex, 1, 1);
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	dev_layout.pages_count = SPI_FLASH_SIZE/SPI_FLASH_PAGE_SIZE;
	dev_layout.pages_size = SPI_FLASH_PAGE_SIZE;
#endif
	return 0;
}

DEVICE_DT_INST_DEFINE(0, flash_in6xxe_init, NULL,
		      &flash_data, NULL, POST_KERNEL,
		      CONFIG_FLASH_INIT_PRIORITY, &flash_in6xxe_driver_api);
