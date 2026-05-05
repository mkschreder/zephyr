/*
 * Copyright 2026 Martin Schröder <info@swedishembedded.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ARMv8-M Security Attribution Unit (SAU) driver
 *
 * Programs SAU regions from DT nodes with compatible "arm,armv8m-sau-region"
 * at POST_KERNEL priority so SAU is active before any NS-world launch.
 *
 * Reference: ARM DDI 0553 §B3.4 (SAU programmer's model).
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/__assert.h>
#include <cmsis_core.h>

#define SAU_NODE DT_NODELABEL(sau)

/* Total number of SAU regions declared in DT. */
#define SAU_DT_REGION_COUNT \
	DT_CHILD_NUM_STATUS_OKAY(SAU_NODE)

/* Helper macros that expand over every child region node. */
#define SAU_BASE(n)  DT_REG_ADDR(n)
#define SAU_SIZE(n)  DT_REG_SIZE(n)
#define SAU_NSC(n)   DT_PROP(n, nsc)
#define SAU_END(n)   (SAU_BASE(n) + SAU_SIZE(n) - 1U)

/*
 * Compile-time validation (DDI 0553 §B3.4.4):
 * - RBAR.BADDR stores bits [31:5] → base address must be 32-byte aligned.
 * - RLAR.LADDR stores bits [31:5] → end address lower 5 bits must be 1
 *   (i.e. (base + size - 1) & 0x1F == 0x1F, so size is a multiple of 32).
 */
#define SAU_REGION_ASSERT(n)                                                  \
	BUILD_ASSERT((SAU_BASE(n) & 0x1FU) == 0U,                            \
		     "SAU region " DT_NODE_FULL_NAME(n)                       \
		     ": base address must be 32-byte aligned (DDI 0553 §B3.4.4)"); \
	BUILD_ASSERT((SAU_SIZE(n) & 0x1FU) == 0U,                            \
		     "SAU region " DT_NODE_FULL_NAME(n)                       \
		     ": size must be a multiple of 32 bytes (DDI 0553 §B3.4.4)"); \
	BUILD_ASSERT(SAU_SIZE(n) > 0U,                                        \
		     "SAU region " DT_NODE_FULL_NAME(n) ": size must not be zero");

DT_FOREACH_CHILD_STATUS_OKAY(SAU_NODE, SAU_REGION_ASSERT)

/**
 * @brief Program all SAU regions declared in the device tree.
 */
static int z_arm_sau_dt_init(void)
{
	uint32_t hw_regions = (SAU->TYPE & SAU_TYPE_SREGION_Msk) >> SAU_TYPE_SREGION_Pos;
	uint32_t dt_regions = SAU_DT_REGION_COUNT;

	BUILD_ASSERT(SAU_DT_REGION_COUNT <= 8U,
		     "SAU_DT_REGION_COUNT exceeds maximum of 8 regions");

	__ASSERT(dt_regions <= hw_regions,
		 "SAU: DT declares %u regions but hardware only has %u",
		 dt_regions, hw_regions);
	ARG_UNUSED(hw_regions);
	ARG_UNUSED(dt_regions);

	/* Disable SAU while programming to avoid partial attribute windows. */
	SAU->CTRL = 0U;

	uint32_t idx = 0U;

#define SAU_PROGRAM_REGION(n)                                                  \
	{                                                                      \
		uint32_t base = (uint32_t)SAU_BASE(n);                        \
		uint32_t end  = (uint32_t)SAU_END(n);                         \
		bool     nsc  = SAU_NSC(n);                                   \
		SAU->RNR  = idx;                                               \
		SAU->RBAR = base & SAU_RBAR_BADDR_Msk;                        \
		SAU->RLAR = (end & SAU_RLAR_LADDR_Msk)                        \
			  | (nsc ? SAU_RLAR_NSC_Msk : 0U)                     \
			  | SAU_RLAR_ENABLE_Msk;                               \
		idx++;                                                         \
	}

	DT_FOREACH_CHILD_STATUS_OKAY(SAU_NODE, SAU_PROGRAM_REGION)

#undef SAU_PROGRAM_REGION

	/* Enable the SAU. */
	SAU->CTRL = SAU_CTRL_ENABLE_Msk;

	/* Ensure SAU is active before any subsequent memory accesses. */
	__DSB();
	__ISB();

	return 0;
}

#if DT_NODE_HAS_STATUS(SAU_NODE, okay) && \
	defined(CONFIG_ARM_SECURE_FIRMWARE) && \
	defined(CONFIG_ARM_SAU_DT)
SYS_INIT(z_arm_sau_dt_init, POST_KERNEL, CONFIG_ARM_SAU_DT_INIT_PRIORITY);
#endif
