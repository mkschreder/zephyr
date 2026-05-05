/*
 * Copyright 2026 Martin Schröder <info@swedishembedded.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ARMv8-M TrustZone-M security hardening initialization.
 *
 * Applies the Kconfig-controlled AIRCR bits, CCR_S.TRD, and calls the
 * weak debug-lockdown and lifecycle hooks.  Runs at PRE_KERNEL_1 so that
 * all hardening is in place before any thread or driver init.
 *
 * References:
 *   ARM DDI 0553B §D1.2.2  (AIRCR: BFHFNMINS, PRIS, SYSRESETREQS)
 *   ARM DDI 0553B §D1.2.13 (CCR_S.TRD)
 *   ARM DDI 0553B §H4.4.1  (DHCSR.S_SDE)
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <cmsis_core.h>
#include <zephyr/arch/arm/arch.h>
#include <zephyr/arch/arm/cortex_m/tz_lifecycle.h>

/* Forward declaration from exception.h (cannot include there due to cycles). */
void z_arm_aircr_set(uint32_t mask, uint32_t value);

/* -----------------------------------------------------------------------
 * Weak hooks — SoC-specific files may override these.
 * --------------------------------------------------------------------- */

/**
 * @brief Hook called to disable Secure halting debug.
 *
 * Override in SoC files to blow OTP fuses or clear DHCSR.S_SDE via DAP.
 * The default implementation is a no-op.
 *
 * Reference: ARM DDI 0553B §H4.4.1.
 */
__weak void z_arm_secure_debug_lockdown(void) {}

/**
 * @brief Lifecycle transition hook (weak default — no-op).
 *
 * SoC BSPs that support OTP lifecycle control override this symbol.
 * See include/zephyr/arch/arm/cortex_m/tz_lifecycle.h for level constants.
 *
 * @param level  Target ARM_TZ_LIFECYCLE_* level.
 * @return 0 on success, negative errno on error.
 */
__weak int z_arm_secure_lifecycle_advance(int level)
{
	ARG_UNUSED(level);
	return 0;
}

/**
 * @brief Query the current lifecycle level (weak default — returns UNKNOWN).
 *
 * SoC BSPs override this to read from the lifecycle controller.
 *
 * @return ARM_TZ_LIFECYCLE_UNKNOWN by default.
 */
__weak int z_arm_secure_lifecycle_current(void)
{
	return ARM_TZ_LIFECYCLE_UNKNOWN;
}

/* -----------------------------------------------------------------------
 * CCR_S.TRD — disable Secure Thread re-entry (DDI 0553B §D1.2.13).
 * --------------------------------------------------------------------- */
#if defined(CONFIG_ARM_TZ_CCR_S_TRD)
static void tz_set_ccr_s_trd(void)
{
	/*
	 * CCR_S (banked to Secure state) is accessed via SCB_S->CCR on
	 * CMSIS.  Write CCR_S[TRD]=1 (bit 2, DDI 0553B §D1.2.13).
	 *
	 * CMSIS may not expose SCB_NS->CCR for direct bit manipulation, so
	 * we use the _NS alias selectively.  The Secure CCR is reached via
	 * the normal SCB pointer when running in Secure state.
	 *
	 * Bit 2 in CCR_S is TRD: Thread Re-entry Disabled.
	 */
#if defined(SCB_CCR_TRD_Msk)
	SCB->CCR |= SCB_CCR_TRD_Msk;
#else
	/* TRD is bit 2 per DDI 0553B §D1.2.13 if CMSIS lacks the macro. */
	SCB->CCR |= (1U << 2);
#endif
	__DSB();
	__ISB();
}
#endif /* CONFIG_ARM_TZ_CCR_S_TRD */

/* -----------------------------------------------------------------------
 * Main hardening init function.
 * --------------------------------------------------------------------- */
static int z_arm_tz_hardening_init(void)
{
#if defined(CONFIG_ARM_TZ_AIRCR_PRIS)
	/*
	 * Separate NS and Secure interrupt priority ranges.
	 * DDI 0553B §B3.2.6: PRIS=1 maps NS priorities to the lower half.
	 */
	z_arm_aircr_set(SCB_AIRCR_PRIS_Msk, SCB_AIRCR_PRIS_Msk);
#endif

#if defined(CONFIG_ARM_TZ_AIRCR_SYSRESETREQS)
	/*
	 * Restrict system reset to Secure firmware only.
	 * DDI 0553B §D1.2.7: SYSRESETREQS=1 blocks NS SYSRESETREQ.
	 */
	z_arm_aircr_set(SCB_AIRCR_SYSRESETREQS_Msk, SCB_AIRCR_SYSRESETREQS_Msk);
#endif

#if defined(CONFIG_ARM_TZ_AIRCR_BFHFNMINS)
	/*
	 * Route BusFault / HardFault / NMI to Non-Secure state.
	 * DDI 0553B §D1.2.2: BFHFNMINS=1.
	 */
	z_arm_aircr_set(SCB_AIRCR_BFHFNMINS_Msk, SCB_AIRCR_BFHFNMINS_Msk);
#endif

#if defined(CONFIG_ARM_TZ_CCR_S_TRD)
	tz_set_ccr_s_trd();
#endif

#if defined(CONFIG_ARM_TZ_SECURE_DEBUG_DISABLE)
	z_arm_secure_debug_lockdown();
	(void)z_arm_secure_lifecycle_advance(ARM_TZ_LIFECYCLE_SECURED);
#endif

	return 0;
}

SYS_INIT(z_arm_tz_hardening_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
