/* SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Martin Schröder <info@swedishembedded.com>
 *
 * Minimal read-only memory-mapped NOR flash driver for vemu boards.
 *
 * The flash content is file-backed and memory-mapped by vemu at the device's
 * reg base address. This driver provides read-only flash access to allow
 * MCUboot and Zephyr partition APIs to function.
 *
 * Write and erase operations return -ENOTSUP. OTA and DFU features require
 * a writable flash backend (extend this driver or use flash_simulator for
 * that use-case).
 */

#define DT_DRV_COMPAT vemu_mmio_flash

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <string.h>

struct mmio_flash_config {
	uintptr_t base;
	size_t size;
	struct flash_parameters params;
};

static int mmio_flash_read(const struct device *dev, off_t offset, void *data, size_t len)
{
	const struct mmio_flash_config *cfg = dev->config;

	if (offset < 0 || (size_t)offset + len > cfg->size) {
		return -EINVAL;
	}

	memcpy(data, (const void *)(cfg->base + (uintptr_t)offset), len);
	return 0;
}

static int mmio_flash_write(const struct device *dev, off_t offset, const void *data, size_t len)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(offset);
	ARG_UNUSED(data);
	ARG_UNUSED(len);
	return -ENOTSUP;
}

static int mmio_flash_erase(const struct device *dev, off_t offset, size_t size)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(offset);
	ARG_UNUSED(size);
	return -ENOTSUP;
}

static const struct flash_parameters *mmio_flash_get_parameters(const struct device *dev)
{
	const struct mmio_flash_config *cfg = dev->config;

	return &cfg->params;
}

#ifdef CONFIG_FLASH_PAGE_LAYOUT
static void mmio_flash_pages_layout(const struct device *dev,
				    const struct flash_pages_layout **layout,
				    size_t *layout_size)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(layout);
	ARG_UNUSED(layout_size);
}
#endif

static const struct flash_driver_api mmio_flash_api = {
	.read = mmio_flash_read,
	.write = mmio_flash_write,
	.erase = mmio_flash_erase,
	.get_parameters = mmio_flash_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = mmio_flash_pages_layout,
#endif
};

#define MMIO_FLASH_INIT(n)                                                                         \
	static const struct mmio_flash_config mmio_flash_config_##n = {                            \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.size = DT_INST_REG_SIZE(n),                                                       \
		.params = {                                                                        \
			.write_block_size = DT_INST_PROP_OR(n, write_block_size, 4),               \
			.erase_value = 0xFF,                                                       \
		},                                                                                 \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, NULL, NULL, NULL, &mmio_flash_config_##n, PRE_KERNEL_1,           \
			      CONFIG_FLASH_INIT_PRIORITY, &mmio_flash_api);

DT_INST_FOREACH_STATUS_OKAY(MMIO_FLASH_INIT)
