/*
 * Copyright (c) 2026 Martin Schröder <info@swedishembedded.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/arch/arm/mmu/arm_mmu.h>

static const struct arm_mmu_region mmu_regions[] = {
	/* Exception vectors at the aliased low address (also at 0xFFFF0000). */
	MMU_REGION_FLAT_ENTRY("vectors",
		0x00000000, 0x1000,
		MT_STRONGLY_ORDERED | MPERM_R | MPERM_X),

	/* GIC-400: distributor 0xFF1F1000, CPU interface 0xFF1F2000. */
	MMU_REGION_FLAT_ENTRY("gic",
		0xFF1F1000, 0x3000,
		MT_STRONGLY_ORDERED | MPERM_R | MPERM_W),

	/* UART2 console (0xFF4C0000, 4 KiB). */
	MMU_REGION_FLAT_ENTRY("uart2",
		0xFF4C0000, 0x1000,
		MT_DEVICE | MPERM_R | MPERM_W),

	/* CRU (clock/reset unit, 0xFF3A0000, 64 KiB). */
	MMU_REGION_FLAT_ENTRY("cru",
		0xFF3A0000, 0x10000,
		MT_DEVICE | MPERM_R | MPERM_W),
};

const struct arm_mmu_config mmu_config = {
	.num_regions = ARRAY_SIZE(mmu_regions),
	.mmu_regions = mmu_regions,
};
