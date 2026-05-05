/*
 * Copyright 2026 Martin Schröder <info@swedishembedded.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ARMv8-M TrustZone-M Non-Secure world launch handoff helpers.
 *
 * Provides the canonical, board-agnostic way for a Secure Zephyr image to
 * configure and start the Non-Secure world.  Replaces the ad-hoc per-SoC
 * sequences that wrote VTOR_NS, MSP_NS, and issued BXNS independently.
 *
 * Usage (Secure firmware):
 *
 * @code
 * #include <zephyr/arch/arm/cortex_m/tz_handoff.h>
 *
 * const struct arm_tz_handoff_descriptor ns_desc = {
 *     .vtor_ns  = CONFIG_NS_IMAGE_ROM_START,
 *     .msp_ns   = NS_STACK_TOP,  // first word of NS vector table
 *     .reset_ns = NS_RESET_VECTOR, // second word of NS vector table
 * };
 * arm_tz_launch_nonsecure(&ns_desc);
 * @endcode
 *
 * The helper reads VTOR_NS / MSP_NS from the descriptor rather than
 * deriving them from the NS vector table so that platforms with unusual
 * memory layouts (e.g. ROM-based NS image) can supply overrides.
 *
 * Reference: ARM DDI 0553B §C1.4.5 (BXNS / BLXNS),
 *            §D1.2.275 (VTOR_NS), §B3.17 (MSPLIM_NS).
 */

#ifndef ZEPHYR_INCLUDE_ARCH_ARM_CORTEX_M_TZ_HANDOFF_H_
#define ZEPHYR_INCLUDE_ARCH_ARM_CORTEX_M_TZ_HANDOFF_H_

#if defined(CONFIG_ARM_SECURE_FIRMWARE)

#include <zephyr/types.h>
#include <zephyr/sys/__assert.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Non-Secure world launch descriptor.
 *
 * All addresses must be physical (no virtual-memory mapping on M-profile).
 *
 * @note vtor_ns must be 128-byte aligned (VTOR_NS[6:0] are RES0, DDI 0553B
 *       §D1.2.275) and within the NS-attributed address space.
 */
struct arm_tz_handoff_descriptor {
	/** Base address of the Non-Secure vector table (VTOR_NS). */
	uintptr_t vtor_ns;

	/**
	 * Initial Non-Secure MSP value.  If zero, the helper reads the first
	 * word from the NS vector table at @p vtor_ns.
	 */
	uintptr_t msp_ns;

	/**
	 * Non-Secure reset vector (entry point address, Thumb bit set).
	 * If zero, the helper reads the second word from the NS vector table
	 * at @p vtor_ns.
	 */
	uintptr_t reset_ns;

	/**
	 * Initial Non-Secure PSPLIM value.  Set to 0 if the NS image manages
	 * its own PSP stack limit.
	 */
	uintptr_t psplim_ns;
};

/**
 * @brief Transfer control to the Non-Secure world.
 *
 * Performs the following steps in order (DDI 0553B §C1.4.5):
 *  1. Programs VTOR_NS with @p desc->vtor_ns.
 *  2. Programs MSP_NS with @p desc->msp_ns (or reads from vector table).
 *  3. Programs PSPLIM_NS with @p desc->psplim_ns.
 *  4. Calls the NS reset handler via BXNS (with bit[0]=1 for Thumb).
 *
 * @note This function does not return.  The NS world starts executing and
 *       can only re-enter Secure state through SG veneers.
 *
 * @param desc  Pointer to the handoff descriptor.  Must not be NULL.
 */
__attribute__((noreturn))
void arm_tz_launch_nonsecure(const struct arm_tz_handoff_descriptor *desc);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_ARM_SECURE_FIRMWARE */

#endif /* ZEPHYR_INCLUDE_ARCH_ARM_CORTEX_M_TZ_HANDOFF_H_ */
