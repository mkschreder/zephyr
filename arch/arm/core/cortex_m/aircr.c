/*
 * Copyright 2026 Martin Schröder <info@swedishembedded.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Safe AIRCR read-modify-write helper for ARMv8-M.
 *
 * All AIRCR writes in Zephyr must go through z_arm_aircr_set() so that
 * critical sticky bits (PRIS, SYSRESETREQS, BFHFNMINS) are never
 * accidentally cleared by a careless RMW that uses the wrong mask.
 *
 * Background:
 *   - Writing AIRCR without VECTKEY=0x05FA has no effect (write-key protection).
 *   - SYSRESETREQS (bit 3): once set by Secure firmware it should remain set.
 *   - PRIS (bit 14): once set to separate NS/S priorities it should remain set.
 *   - BFHFNMINS (bit 13): deliberately configurable via Kconfig.
 *
 * Reference: ARM DDI 0553B §D1.2.2 (AIRCR).
 */

#include <zephyr/arch/arm/arch.h>
#include <zephyr/kernel.h>
#include <cmsis_core.h>

/**
 * @brief Atomic read-modify-write of SCB->AIRCR preserving all other bits.
 *
 * @param mask  Bitmask of bits to modify (must not include VECTKEY bits).
 * @param value New values for the bits selected by @p mask.
 *
 * @note Interrupts need not be disabled; the VECTKEY write-key protection
 *       means only one writer can succeed per write transaction, and the
 *       M-profile AIRCR is not concurrently writable from both security
 *       states simultaneously while the PE holds the bus.
 */
void z_arm_aircr_set(uint32_t mask, uint32_t value)
{
	uint32_t reg = SCB->AIRCR;

	/*
	 * Mask out the VECTKEY field (which reads as VECTKEYSTAT = 0xFA05),
	 * then mask out the bits we are changing, and apply the new values
	 * together with the mandatory VECTKEY=0x05FA write token.
	 */
	reg &= ~(SCB_AIRCR_VECTKEY_Msk | mask);
	reg |= (0x05FAU << SCB_AIRCR_VECTKEY_Pos) | (value & mask);

	SCB->AIRCR = reg;

	/* Ensure the write completes before subsequent memory accesses. */
	__DSB();
	__ISB();
}
