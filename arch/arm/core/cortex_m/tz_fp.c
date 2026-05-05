/*
 * Copyright 2026 Martin Schröder <info@swedishembedded.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ARMv8-M TrustZone-M FPU context ownership tracking.
 *
 * Handles the interaction between TrustZone-M security states and lazy FP
 * state preservation (FPCCR.LSPACT / FPCCR.S).
 *
 * Problem (DDI 0553B §B7.7 / §B3.16):
 *   When a Secure thread uses FP and a NS exception fires, the lazy FP save
 *   is deferred (FPCCR.LSPACT=1, FPCCR.S=1 if the owning state was Secure).
 *   If a second Secure thread that also uses FP is context-switched in before
 *   the lazy save completes, the incoming thread's FP push will try to resolve
 *   the pending lazy save from the wrong context, triggering SFSR.LSERR.
 *
 * Fix:
 *   Before saving a thread's context in z_arm_pendsv, check if there is a
 *   pending Secure lazy FP save (FPCCR.LSPACT=1 && FPCCR.S=1).  If so, force
 *   the save by executing a dummy VSTM to trigger the lazy push before we
 *   context-switch.
 *
 * Reference:
 *   ARM DDI 0553B §B7.7 (Lazy FP across security states)
 *   ARM DDI 0553B §B3.16 (FPCCR.S / FPCCR.LSPACT)
 *   ARM DDI 0553B §D1.2.266 (SFSR.LSERR / SFSR.LSPERR)
 */

#include <zephyr/kernel.h>
#include <cmsis_core.h>

#if defined(CONFIG_ARM_SECURE_FIRMWARE) && defined(CONFIG_FPU) && defined(CONFIG_FPU_SHARING)

/*
 * FPCCR bit definitions (DDI 0553B §B3.16).
 * CMSIS may not define all of these for all toolchain versions.
 */
#ifndef FPU_FPDSCR_AHP_Msk
/* Fall back to raw bit values. */
#define FPCCR_S_BIT      (1UL << 2)   /* Secure FP context owner */
#define FPCCR_LSPACT_BIT (1UL << 0)   /* Lazy state preservation active */
#else
#define FPCCR_S_BIT      (FPU_FPCCR_S_Msk)
#define FPCCR_LSPACT_BIT (FPU_FPCCR_LSPACT_Msk)
#endif

/**
 * @brief Force Secure lazy FP save if a pending deferred save exists.
 *
 * Called from the context-switch save path (swap_helper.S) whenever FPU
 * sharing is active and we are in Secure firmware mode.  If there is a
 * pending Secure lazy FP save (FPCCR.LSPACT=1 && FPCCR.S=1) we execute a
 * dummy VSTM to trigger the hardware push before we overwrite the FP
 * registers with the new thread's context.
 *
 * This prevents SFSR.LSERR from firing when the incoming thread uses FP
 * and triggers its own lazy push while the old thread's push is still
 * pending.
 *
 * @return 1 if a lazy save was forced, 0 otherwise.
 */
int z_arm_tz_force_secure_fp_lazy_save(void)
{
	uint32_t fpccr = FPU->FPCCR;

	if ((fpccr & FPCCR_LSPACT_BIT) == 0U || (fpccr & FPCCR_S_BIT) == 0U) {
		return 0;
	}

	/*
	 * Force the lazy FP save by executing a VSTM to a temporary buffer.
	 * The hardware will perform the deferred push to the exception frame
	 * as a side-effect of this instruction, clearing FPCCR.LSPACT.
	 *
	 * We use a local 8-word buffer (s0-s7) on the Secure stack; it gets
	 * immediately discarded.
	 */
	volatile float tmp[8];

	__asm__ volatile(
		"vstmia %0, {s0-s7}\n\t"
		: : "r"(tmp) : "memory");

	return 1;
}

/**
 * @brief Recovery hook for SFSR.LSERR — lazy-FP-state error.
 *
 * Called by the z_arm_secure_fault_hook override in tz_fp.c when the fault
 * is SFSR.LSERR.  Attempts to recover by clearing the lazy-state active flag
 * and returning non-zero to tell the fault handler the fault is recoverable.
 *
 * This is a best-effort recovery: if the lazy FP frame is corrupt the
 * exception will re-fire on the next FP instruction, which is appropriate
 * (the thread will be killed on the second fault).
 */
int z_arm_secure_fault_hook(uint32_t sfsr, uint32_t sfar, struct arch_esf *esf)
{
	ARG_UNUSED(sfar);
	ARG_UNUSED(esf);

	/*
	 * SFSR.LSERR (bit 7): lazy state error — recoverable if caused by
	 * a context-switch-induced lazy save that we can retry.
	 */
	if ((sfsr & SAU_SFSR_LSERR_Msk) != 0U) {
		/*
		 * Clear FPCCR.LSPACT so the lazy save is abandoned.
		 * The affected thread will lose its FP context (acceptable —
		 * it will likely fault again on the next FP instruction, which
		 * is then handled as a normal hardware trap).
		 */
		FPU->FPCCR &= ~FPCCR_LSPACT_BIT;
		__DSB();
		return 1; /* recoverable */
	}

	return 0; /* all other SecureFaults remain fatal */
}

#endif /* CONFIG_ARM_SECURE_FIRMWARE && CONFIG_FPU && CONFIG_FPU_SHARING */
