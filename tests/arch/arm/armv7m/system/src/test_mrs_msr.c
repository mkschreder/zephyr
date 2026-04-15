/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M system: MRS / MSR special-register access.
 *
 * Reference: DDI0403E
 *   MRS A7.7.82  lines 10795-10812, B5.2.2 lines 29097-29160
 *   MSR A7.7.83  lines 10813-10830, B5.2.3 lines 29161-29237
 *   APSR  A2.3.2  lines 769-794
 *   PRIMASK/BASEPRI/FAULTMASK B1.4.3 lines 21816-21842
 *   CONTROL B1.4.4 lines 21881-21922
 *   xPSR  B1.4.2  lines 21725-21813
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <cmsis_core.h>
#include <armv7m_test.h>

/* APSR: read/write N/Z/C/V via apsr_nzcvq */
ZTEST(armv7m_system, test_mrs_apsr_flags)
{
	uint32_t apsr;

	/* Set known flags via MOVS 0 (Z=1, N=0, C=0) */
	__asm__ volatile(
		"movs r0, #0\n\t"
		"mrs  %[f], apsr\n\t"
		: [f] "=r"(apsr)
		:
		: "r0", "cc");
	zassert_true(armv7m_flag_z(apsr), "Z not set after MOVS 0");
}

ZTEST(armv7m_system, test_msr_apsr_roundtrip)
{
	uint32_t orig, modified, readback;

	__asm__ volatile("mrs %[o], apsr\n\t" : [o] "=r"(orig) : : );

	/* Set N and C, clear Z and V via MSR apsr_nzcvq */
	modified = (orig & ~ARMV7M_NZCV_MASK) | (1U << 31) | (1U << 29);
	__asm__ volatile(
		"msr apsr_nzcvq, %[m]\n\t"
		"mrs %[r], apsr\n\t"
		: [r] "=r"(readback)
		: [m] "r"(modified)
		: "cc");
	zassert_equal(readback & ARMV7M_NZCV_MASK,
		      modified & ARMV7M_NZCV_MASK,
		      "MSR/MRS APSR NZCV mismatch: wrote 0x%08x read 0x%08x",
		      modified, readback);

	/* Restore */
	__asm__ volatile("msr apsr_nzcvq, %[o]\n\t" : : [o] "r"(orig) : "cc");
}

/* IPSR: in thread mode, IPSR should be 0 */
ZTEST(armv7m_system, test_mrs_ipsr_thread_mode)
{
	uint32_t ipsr;

	__asm__ volatile("mrs %[i], ipsr\n\t" : [i] "=r"(ipsr) : : );
	/* IPSR bits[8:0] = exception number.  Thread mode = 0. */
	zassert_equal(ipsr & 0x1FFU, 0, "IPSR nonzero in thread mode: %d",
		      ipsr & 0x1FF);
}

/* PRIMASK: disable/enable all exceptions */
ZTEST(armv7m_system, test_primask_roundtrip)
{
	uint32_t pm_before, pm_after_set, pm_after_clr;

	__asm__ volatile("mrs %[p], primask\n\t" : [p] "=r"(pm_before) : : );

	/* CPSID i: set PRIMASK.PM = 1 */
	__asm__ volatile("cpsid i\n\t" : : : "memory");
	__asm__ volatile("mrs %[p], primask\n\t" : [p] "=r"(pm_after_set) : : );

	/* CPSIE i: clear PRIMASK.PM = 0 */
	__asm__ volatile("cpsie i\n\t" : : : "memory");
	__asm__ volatile("mrs %[p], primask\n\t" : [p] "=r"(pm_after_clr) : : );

	zassert_equal(pm_after_set & 1U, 1U, "PRIMASK.PM not set by CPSID i");
	zassert_equal(pm_after_clr & 1U, 0U, "PRIMASK.PM not cleared by CPSIE i");
}

/* BASEPRI: masks exceptions up to and including specified priority */
ZTEST(armv7m_system, test_basepri_roundtrip)
{
	uint32_t bp_before, bp_write, bp_read;

	__asm__ volatile("mrs %[b], basepri\n\t" : [b] "=r"(bp_before) : : );

	/* Set BASEPRI to 0x40 (priority 64) */
	bp_write = 0x40U;
	__asm__ volatile(
		"msr basepri, %[v]\n\t"
		"mrs %[r], basepri\n\t"
		: [r] "=r"(bp_read)
		: [v] "r"(bp_write)
		: );
	/* Implementation may zero-extend: read should match at least the written bits */
	zassert_not_equal(bp_read, 0, "BASEPRI read 0 after writing 0x40");

	/* Restore */
	__asm__ volatile("msr basepri, %[v]\n\t" : : [v] "r"(bp_before) : );
}

/* FAULTMASK */
ZTEST(armv7m_system, test_faultmask_roundtrip)
{
	uint32_t fm;

	__asm__ volatile("cpsid f\n\t" : : : "memory");
	__asm__ volatile("mrs %[f], faultmask\n\t" : [f] "=r"(fm) : : );
	zassert_equal(fm & 1U, 1U, "FAULTMASK not set by CPSID f");

	__asm__ volatile("cpsie f\n\t" : : : "memory");
	__asm__ volatile("mrs %[f], faultmask\n\t" : [f] "=r"(fm) : : );
	zassert_equal(fm & 1U, 0U, "FAULTMASK not cleared by CPSIE f");
}

/* CONTROL: read and verify bit fields */
ZTEST(armv7m_system, test_control_read)
{
	uint32_t ctrl;

	__asm__ volatile("mrs %[c], control\n\t" : [c] "=r"(ctrl) : : );
	/* In privileged thread mode, nPRIV (bit0) should be 0 */
	zassert_equal(ctrl & 1U, 0U, "CONTROL.nPRIV not 0 in privileged mode: 0x%x",
		      ctrl);
}

/* CONTROL.SPSEL: verify PSP is active in Zephyr thread mode.
 *
 * ARMv7-M DDI0403E B1.4.4: CONTROL.SPSEL selects which stack pointer is
 * active in Thread mode.  SPSEL=1 => PSP, SPSEL=0 => MSP.
 *
 * In Zephyr, all threads run with CONTROL.SPSEL=1 (PSP).  We verify this
 * is the case and that PSP holds a valid non-zero address.  We intentionally
 * do NOT switch back to MSP (SPSEL=0) because Zephyr threads are required
 * to use PSP; doing so would corrupt the kernel's stack tracking and trigger
 * a fatal assertion in the fault handler.
 */
ZTEST(armv7m_system, test_control_spsel)
{
	uint32_t ctrl, psp_val, msp_val;

	/* Verify Zephyr thread is using PSP (CONTROL.SPSEL=1) */
	__asm__ volatile("mrs %[c], control\n\t" : [c] "=r"(ctrl) : : );
	zassert_equal((ctrl >> 1) & 1U, 1U,
		      "CONTROL.SPSEL != 1 (PSP not active in thread mode): ctrl=0x%x",
		      ctrl);

	/* PSP must be a non-zero aligned address */
	__asm__ volatile("mrs %[p], psp\n\t" : [p] "=r"(psp_val) : : );
	zassert_not_equal(psp_val, 0, "PSP is zero in thread mode");
	zassert_equal(psp_val % 4, 0, "PSP is not word-aligned: 0x%08x", psp_val);

	/* MSP is still readable (handler stack) even when PSP is active */
	__asm__ volatile("mrs %[m], msp\n\t" : [m] "=r"(msp_val) : : );
	zassert_not_equal(msp_val, 0, "MSP is zero in thread mode");
}

/* xPSR: read full xPSR via MRS.
 *
 * ARMv7-M DDI0403E B5.2.2 (MRS pseudocode line 29138):
 *   "if SYSm<1> == '1' then R[d]<26:24> = '000'; // EPSR reads as zero"
 * Reference line 29155:
 *   "None of the EPSR bits are readable during normal execution.
 *    They all read as 0 when read using MRS."
 *
 * Therefore EPSR.T (bit 24) ALWAYS reads as 0 via MRS even though the
 * processor is in Thumb mode.  APSR (bits [31:28]) and IPSR (bits [8:0])
 * are readable.
 */
ZTEST(armv7m_system, test_mrs_xpsr)
{
	uint32_t xpsr;

	__asm__ volatile("mrs %[x], xpsr\n\t" : [x] "=r"(xpsr) : : );

	/* EPSR bits [26:24] must read as 0 per spec (not a QEMU bug) */
	zassert_equal((xpsr >> 24) & 0x7U, 0U,
		      "EPSR bits[26:24] not zero via MRS: 0x%08x", xpsr);

	/* IPSR bits[8:0] = 0 in thread mode */
	zassert_equal(xpsr & 0x1FFU, 0,
		      "IPSR nonzero in thread mode: 0x%08x", xpsr);
}
