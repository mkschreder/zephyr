/*
 * Copyright 2026 Martin Schröder <info@swedishembedded.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ARMv8-M Non-Secure MPU configuration from Secure firmware.
 *
 * Provides z_arm_mpu_ns_configure() which programs the Non-Secure MPU
 * (MPU_NS) from Secure privileged state before launching the NS world.
 *
 * The NS world's MPU configuration is typically set up by the NS firmware
 * itself, but certain security policies require the Secure world to pre-
 * configure specific NS MPU regions (e.g. to prevent NS code from accessing
 * shared memory regions without SAU backing).
 *
 * Reference: ARM DDI 0553B §B3.4.5 (MPU_S vs MPU_NS),
 *            §D1.2.223 (MPU_NS registers accessible from Secure state).
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <cmsis_core.h>

#if defined(CONFIG_ARM_SECURE_FIRMWARE) && defined(CONFIG_ARM_MPU)

/**
 * @brief Descriptor for a single NS MPU region.
 */
struct arm_mpu_ns_region {
	uint32_t base; /**< Region base address (RBAR_NS value). */
	uint32_t limit; /**< Region limit/attr word (RLAR_NS value, with EN bit). */
};

/**
 * @brief Configure Non-Secure MPU regions from Secure state.
 *
 * Programs the NS MPU's RBAR_NS and RLAR_NS registers for each region
 * descriptor in @p regions.  The NS MPU is disabled, programmed, then
 * re-enabled in a single atomic sequence to avoid a window where NS
 * memory is accessible without protection.
 *
 * @param regions    Array of NS MPU region descriptors.
 * @param num_regions Number of regions to program (must not exceed
 *                   MPU_NS->TYPE.DREGION).
 *
 * @note This function must be called from Secure privileged mode.
 *       It has no effect if called from Non-Secure or unprivileged state.
 *
 * Reference: ARM DDI 0553B §B3.4.5.
 */
void z_arm_mpu_ns_configure(const struct arm_mpu_ns_region *regions,
			     uint32_t num_regions)
{
	__ASSERT(regions != NULL, "NS MPU regions pointer must not be NULL");

	uint32_t hw_regions = (MPU_NS->TYPE & MPU_TYPE_DREGION_Msk)
			      >> MPU_TYPE_DREGION_Pos;

	__ASSERT(num_regions <= hw_regions,
		 "NS MPU: requested %u regions but hardware has %u",
		 num_regions, hw_regions);
	ARG_UNUSED(hw_regions);

	/* Disable NS MPU before reprogramming. */
	MPU_NS->CTRL = 0U;
	__DSB();
	__ISB();

	for (uint32_t i = 0U; i < num_regions; i++) {
		MPU_NS->RNR  = i;
		MPU_NS->RBAR = regions[i].base;
		MPU_NS->RLAR = regions[i].limit;
	}

	/* Disable any remaining (previously configured) regions. */
	for (uint32_t i = num_regions; i < hw_regions; i++) {
		MPU_NS->RNR  = i;
		MPU_NS->RBAR = 0U;
		MPU_NS->RLAR = 0U; /* EN=0 */
	}

	/* Re-enable NS MPU with PRIVDEFENA=1 (privileged default map). */
	MPU_NS->CTRL = MPU_CTRL_ENABLE_Msk | MPU_CTRL_PRIVDEFENA_Msk;
	__DSB();
	__ISB();
}

#endif /* CONFIG_ARM_SECURE_FIRMWARE && CONFIG_ARM_MPU */
