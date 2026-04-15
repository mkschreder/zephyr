/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M ALU: Saturating instructions (DSP extension).
 *
 * Reference: DDI0403E
 *   SSAT   A7.7.153
 *   USAT   A7.7.197
 *   QADD   A7.7.102 lines 11959-12012
 *   QSUB   A7.7.105
 *   QDADD  A7.7.103
 *   QDSUB  A7.7.104
 *
 * These instructions are part of the DSP extension (present on Cortex-M4/M4F,
 * ARMv7E-M).  Guard with CONFIG_CPU_HAS_DSP.
 *
 * SSAT: saturate to signed N-bit range.
 * USAT: saturate to unsigned N-bit range.
 * Q flag set on saturation (A2.3.2 line 788).
 */

#include <zephyr/ztest.h>
#include <armv7m_test.h>

#if defined(__ARM_FEATURE_DSP)

/* SSAT */
ZTEST(armv7m_alu_flags, test_ssat_within_range)
{
	uint32_t r, apsr;

	/* SSAT #8 saturates to [-128, 127]; value 50 is within range */
	__asm__ volatile(
		"ssat %[r], #8, %[v]\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=r"(r), [f] "=r"(apsr)
		: [v] "r"(50)
		: "cc");
	zassert_equal((int32_t)r, 50, "SSAT in-range wrong");
	zassert_false(armv7m_flag_q(apsr), "Q spuriously set");
}

ZTEST(armv7m_alu_flags, test_ssat_clamp_positive)
{
	uint32_t r, apsr;

	/* SSAT #8: 200 > 127 => clamp to 127, Q set */
	__asm__ volatile(
		"mrs  %[f], apsr\n\t"  /* read current apsr */
		"bic  %[f], %[f], #0x08000000\n\t" /* clear Q */
		"msr  apsr_nzcvq, %[f]\n\t"
		"ssat %[r], #8, %[v]\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=&r"(apsr)
		: [v] "r"(200)
		: "cc");
	zassert_equal((int32_t)r, 127, "SSAT clamp positive wrong: %d",
		      (int32_t)r);
	zassert_true(armv7m_flag_q(apsr), "Q not set on saturation");
}

ZTEST(armv7m_alu_flags, test_ssat_clamp_negative)
{
	uint32_t r, apsr;

	/* SSAT #8: -200 < -128 => clamp to -128, Q set */
	__asm__ volatile(
		"mrs  %[f], apsr\n\t"
		"bic  %[f], %[f], #0x08000000\n\t"
		"msr  apsr_nzcvq, %[f]\n\t"
		"ssat %[r], #8, %[v]\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=&r"(apsr)
		: [v] "r"(-200)
		: "cc");
	zassert_equal((int32_t)r, -128, "SSAT clamp negative wrong: %d",
		      (int32_t)r);
	zassert_true(armv7m_flag_q(apsr), "Q not set");
}

/* USAT */
ZTEST(armv7m_alu_flags, test_usat_within_range)
{
	uint32_t r, apsr;

	/* USAT #8: 200 is within [0, 255] */
	__asm__ volatile(
		"usat %[r], #8, %[v]\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=r"(r), [f] "=r"(apsr)
		: [v] "r"(200)
		: "cc");
	zassert_equal(r, 200, "USAT in-range wrong");
	zassert_false(armv7m_flag_q(apsr), "Q spuriously set");
}

ZTEST(armv7m_alu_flags, test_usat_clamp_positive)
{
	uint32_t r, apsr;

	/* USAT #8: 300 > 255 => clamp to 255, Q set */
	__asm__ volatile(
		"mrs  %[f], apsr\n\t"
		"bic  %[f], %[f], #0x08000000\n\t"
		"msr  apsr_nzcvq, %[f]\n\t"
		"usat %[r], #8, %[v]\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=&r"(apsr)
		: [v] "r"(300)
		: "cc");
	zassert_equal(r, 255, "USAT clamp positive wrong: %d", r);
	zassert_true(armv7m_flag_q(apsr), "Q not set");
}

ZTEST(armv7m_alu_flags, test_usat_clamp_negative)
{
	uint32_t r, apsr;

	/* USAT #8: -1 < 0 => clamp to 0, Q set */
	__asm__ volatile(
		"mrs  %[f], apsr\n\t"
		"bic  %[f], %[f], #0x08000000\n\t"
		"msr  apsr_nzcvq, %[f]\n\t"
		"usat %[r], #8, %[v]\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=&r"(apsr)
		: [v] "r"(-1)
		: "cc");
	zassert_equal(r, 0, "USAT clamp negative wrong");
	zassert_true(armv7m_flag_q(apsr), "Q not set");
}

/* QADD: saturating signed add */
ZTEST(armv7m_alu_flags, test_qadd_no_sat)
{
	uint32_t r, apsr;

	__asm__ volatile(
		"mrs  %[f], apsr\n\t"
		"bic  %[f], %[f], #0x08000000\n\t"
		"msr  apsr_nzcvq, %[f]\n\t"
		"qadd %[r], %[a], %[b]\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=&r"(apsr)
		: [a] "r"(100), [b] "r"(200)
		: "cc");
	zassert_equal((int32_t)r, 300, "QADD no-sat wrong");
	zassert_false(armv7m_flag_q(apsr), "Q spuriously set");
}

ZTEST(armv7m_alu_flags, test_qadd_sat_positive)
{
	uint32_t r, apsr;

	/* 0x7FFFFFFF + 1 saturates to 0x7FFFFFFF */
	__asm__ volatile(
		"mrs  %[f], apsr\n\t"
		"bic  %[f], %[f], #0x08000000\n\t"
		"msr  apsr_nzcvq, %[f]\n\t"
		"qadd %[r], %[a], %[b]\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=&r"(apsr)
		: [a] "r"(0x7FFFFFFFU), [b] "r"(1)
		: "cc");
	zassert_equal((int32_t)r, 0x7FFFFFFF, "QADD sat positive wrong");
	zassert_true(armv7m_flag_q(apsr), "Q not set");
}

#else
ZTEST(armv7m_alu_flags, test_ssat_within_range)   { ztest_test_skip(); }
ZTEST(armv7m_alu_flags, test_ssat_clamp_positive) { ztest_test_skip(); }
ZTEST(armv7m_alu_flags, test_ssat_clamp_negative) { ztest_test_skip(); }
ZTEST(armv7m_alu_flags, test_usat_within_range)   { ztest_test_skip(); }
ZTEST(armv7m_alu_flags, test_usat_clamp_positive) { ztest_test_skip(); }
ZTEST(armv7m_alu_flags, test_usat_clamp_negative) { ztest_test_skip(); }
ZTEST(armv7m_alu_flags, test_qadd_no_sat)         { ztest_test_skip(); }
ZTEST(armv7m_alu_flags, test_qadd_sat_positive)   { ztest_test_skip(); }
#endif
