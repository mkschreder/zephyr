/*
 * Copyright 2026 Martin Schröder <info@swedishembedded.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * ARMv8-M TrustZone-M test suite — entry point.
 *
 * Validates:
 *   - SAU register programming (CTRL, TYPE, RNR, RBAR, RLAR)
 *   - TT / TTA / TTT / TTAT instruction bitfield encoding per DDI 0553B C2.338
 *   - SFSR/SFAR accessibility and sticky-bit behaviour
 *   - BXNS/BLXNS INVTRAN SecureFault
 *   - NVIC_ITNS IRQ routing between Secure and Non-Secure NVICs
 *   - CMSE wrap-around defense in arm_cmse_addr_range_*
 *   - SFSR snapshot before clearing (p1-fault fix regression test)
 *   - Banked MSPLIM_S / MSPLIM_NS register independence
 *
 * References:
 *   ARM DDI 0553B.x  B3.4  (SAU programmer's model)
 *   ARM DDI 0553B.x  B3.13 (NVIC_ITNS)
 *   ARM DDI 0553B.x  C2.338 (TT result)
 *   ARM DDI 0553B.x  C2.24  (BXNS)
 *   ARM DDI 0553B.x  D1.2.266 (SFSR)
 */

#include <zephyr/ztest.h>
#include <tz_test.h>

volatile unsigned int tz_last_fault_reason = (unsigned int)-1;

/*
 * Fault handler: catch unexpected CPU exceptions during tests.
 * Without this, an unexpected fault halts the system and prevents
 * subsequent tests from running.
 */
void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	tz_last_fault_reason = reason;

#if defined(CONFIG_EXTRA_EXCEPTION_INFO)
	if (esf != NULL) {
		printk("  SFSR snapshot: 0x%08x  SFAR: 0x%08x\n",
		       esf->extra_info.secure_fault_status,
		       esf->extra_info.secure_fault_address);
	}
#endif
	/* Do NOT call k_fatal_halt in the test-override — let the thread die. */
	ARG_UNUSED(esf);
}

ZTEST_SUITE(arm_trustzone, NULL, NULL, NULL, NULL, NULL);
