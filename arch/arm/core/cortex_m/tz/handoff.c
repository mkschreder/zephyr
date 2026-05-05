/*
 * Copyright 2026 Martin Schröder <info@swedishembedded.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ARMv8-M TrustZone-M Non-Secure world launch helper.
 *
 * Implements arm_tz_launch_nonsecure() declared in tz_handoff.h.
 * Replaces ad-hoc per-SoC sequences that write VTOR_NS / MSP_NS manually.
 *
 * Reference: ARM DDI 0553B §C1.4.5 (BXNS), §D1.2.275 (VTOR_NS),
 *            §B3.17 (MSPLIM_NS), §D1.2.7 (AIRCR.SYSRESETREQS).
 */

#include <zephyr/arch/arm/cortex_m/tz_handoff.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <cmsis_core.h>

#if defined(CONFIG_ARM_SECURE_FIRMWARE)

/*
 * AIRCR write helper (pre-AIRCR helper, inline for handoff use).
 * Preserves VECTKEY and all RES0/sticky bits.
 */
static inline void handoff_aircr_set(uint32_t mask, uint32_t value)
{
	uint32_t reg = SCB->AIRCR;

	reg &= ~(SCB_AIRCR_VECTKEY_Msk | mask);
	reg |= (0x05FAU << SCB_AIRCR_VECTKEY_Pos) | (value & mask);
	SCB->AIRCR = reg;
	__DSB();
	__ISB();
}

__attribute__((noreturn))
void arm_tz_launch_nonsecure(const struct arm_tz_handoff_descriptor *desc)
{
	__ASSERT(desc != NULL, "tz_handoff: descriptor must not be NULL");
	__ASSERT((desc->vtor_ns & 0x3FU) == 0U,
		 "tz_handoff: VTOR_NS must be 64-byte aligned (DDI 0553B §D1.2.275)");

	/*
	 * Resolve initial MSP_NS and reset vector from the descriptor.
	 * If msp_ns or reset_ns are zero, fall back to reading from the
	 * NS vector table (standard Cortex-M vector table layout).
	 */
	const uint32_t *ns_vt = (const uint32_t *)desc->vtor_ns;
	uint32_t msp_ns   = desc->msp_ns   ? (uint32_t)desc->msp_ns   : ns_vt[0];
	uint32_t reset_ns = desc->reset_ns ? (uint32_t)desc->reset_ns : ns_vt[1];

	__ASSERT(reset_ns != 0U, "tz_handoff: NS reset vector is zero");

	/*
	 * Step 1: Set VTOR_NS (DDI 0553B §D1.2.275).
	 * Only the Secure side can write VTOR_NS.
	 */
	SCB_NS->VTOR = (uint32_t)desc->vtor_ns;
	__DSB();

	/*
	 * Step 2: Set MSP_NS (DDI 0553B §B3.17, MSPLIM_NS).
	 * MRS/MSR SYSm=0x18 = MSP_NS, SYSm=0x1A = MSPLIM_NS.
	 */
	__asm__ volatile(
		"mov r0, %0\n\t"
		".inst.w 0xF380881A\n\t"  /* MSR MSPLIM_NS, r0 (SYSm=0x1A) */
		: : "r"((uint32_t)desc->psplim_ns) : "r0");
	__asm__ volatile(
		"mov r0, %0\n\t"
		".inst.w 0xF3808818\n\t"  /* MSR MSP_NS, r0 (SYSm=0x18) */
		: : "r"(msp_ns) : "r0");
	__DSB();

	/*
	 * Step 3: Optionally set SYSRESETREQS so NS cannot reset Secure world.
	 * Only done if CONFIG_ARM_TZ_AIRCR_SYSRESETREQS is enabled (Phase 2).
	 * We always set it here as part of a safe handoff policy.
	 * DDI 0553B §D1.2.7 AIRCR.SYSRESETREQS (bit 3).
	 */
#if defined(CONFIG_ARM_TZ_AIRCR_SYSRESETREQS)
	handoff_aircr_set(SCB_AIRCR_SYSRESETREQS_Msk, SCB_AIRCR_SYSRESETREQS_Msk);
#endif

	/*
	 * Step 4: Branch to NS reset vector using BXNS (DDI 0553B §C1.4.5).
	 *
	 * BXNS Rn: in Secure state, branches to the address in Rn and
	 * transitions to Non-Secure state.  The LSB of Rn must be 1 (Thumb).
	 *
	 * We use an explicit `.inst.w 0x4718` (BXNS r3) because most
	 * toolchains do not support BXNS in inline assembly without the
	 * `__ARM_FEATURE_CMSE` macro.
	 *
	 * Encoding: BXNS Rn (T1):  1000 11 Rn[3:0] 0 0 0  (16-bit)
	 *   Rn = r3 → 0b0100_0111_0001_1000 = 0x4718
	 */
	register uint32_t _reset_ns __asm__("r3") = reset_ns | 1U; /* ensure Thumb bit */

	__asm__ volatile(
		".inst.n 0x4718\n\t"   /* BXNS r3 */
		: : "r"(_reset_ns) : "memory");

	/* Unreachable: BXNS does not return to Secure world. */
	CODE_UNREACHABLE;
}

#endif /* CONFIG_ARM_SECURE_FIRMWARE */
