/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M ALU: Logic instructions AND / ORR / EOR / BIC / ORN / MVN.
 *
 * Reference: DDI0403E
 *   AND A7.7.8-9  lines 6111-6246
 *   ORR A7.7.91-92 lines 11205-11342
 *   EOR A7.7.35-36 lines 7592-7727
 *   BIC A7.7.15-16 lines 6587-6720
 *   ORN A7.7.89-90 lines 11089-11204
 *   MVN A7.7.85-86 lines 10903-11028
 *
 * Logic ops update N and Z from result.  C is updated from barrel
 * shifter carry-out when a shifted register operand is used; V is unchanged.
 */

#include <zephyr/ztest.h>
#include <armv7m_test.h>

/* AND */
ZTEST(armv7m_alu_flags, test_ands_all_ones)
{
	uint32_t r, apsr;

	__asm__ volatile(
		"movs %[r], #0xFF\n\t"
		"ands %[r], %[r], #0xFF\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		:
		: "cc");
	zassert_equal(r, 0xFF, "AND all-ones wrong");
	zassert_false(armv7m_flag_z(apsr), "Z spuriously set");
	zassert_false(armv7m_flag_n(apsr), "N spuriously set");
}

ZTEST(armv7m_alu_flags, test_ands_zero_result)
{
	uint32_t r, apsr;

	__asm__ volatile(
		"movs %[r], #0xAA\n\t"
		"ands %[r], %[r], #0x55\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		:
		: "cc");
	zassert_equal(r, 0, "AND zero result wrong");
	zassert_true(armv7m_flag_z(apsr), "Z not set");
}

ZTEST(armv7m_alu_flags, test_ands_negative_result)
{
	uint32_t a = 0xFF000000U, b = 0x80000000U, r, apsr;

	__asm__ volatile(
		"ands %[r], %[a], %[b]\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		: [a] "r"(a), [b] "r"(b)
		: "cc");
	zassert_equal(r, 0x80000000U, "AND result wrong");
	zassert_true(armv7m_flag_n(apsr), "N not set");
}

/* ORR */
ZTEST(armv7m_alu_flags, test_orrs_combine)
{
	uint32_t r, apsr;

	__asm__ volatile(
		"mov  %[r], #0xAA\n\t"
		"orrs %[r], %[r], #0x55\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		:
		: "cc");
	zassert_equal(r, 0xFF, "ORR combine wrong");
	zassert_false(armv7m_flag_z(apsr), "Z spuriously set");
}

ZTEST(armv7m_alu_flags, test_orrs_zero_stays_zero)
{
	uint32_t r, apsr;

	__asm__ volatile(
		"movs %[r], #0\n\t"
		"orrs %[r], %[r], #0\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		:
		: "cc");
	zassert_equal(r, 0, "ORR zero wrong");
	zassert_true(armv7m_flag_z(apsr), "Z not set");
}

/* EOR */
ZTEST(armv7m_alu_flags, test_eors_toggle)
{
	uint32_t r, apsr;

	__asm__ volatile(
		"mov  %[r], #0xFF\n\t"
		"eors %[r], %[r], #0x0F\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		:
		: "cc");
	zassert_equal(r, 0xF0, "EOR toggle wrong");
}

ZTEST(armv7m_alu_flags, test_eors_self_cancel)
{
	uint32_t a = 0xDEADBEEFU, r, apsr;

	__asm__ volatile(
		"eors %[r], %[a], %[a]\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		: [a] "r"(a)
		: "cc");
	zassert_equal(r, 0, "EOR self-cancel wrong");
	zassert_true(armv7m_flag_z(apsr), "Z not set");
}

/* BIC */
ZTEST(armv7m_alu_flags, test_bics_clear_bits)
{
	uint32_t r, apsr;

	__asm__ volatile(
		"mov  %[r], #0xFF\n\t"
		"bics %[r], %[r], #0x0F\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		:
		: "cc");
	zassert_equal(r, 0xF0, "BIC clear bits wrong");
}

ZTEST(armv7m_alu_flags, test_bics_clear_all)
{
	uint32_t a = 0xFFFFFFFFU, r, apsr;

	__asm__ volatile(
		"mvn  %[r], #0\n\t"          /* r = 0xFFFFFFFF */
		"bics %[r], %[a], %[r]\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		: [a] "r"(a)
		: "cc");
	zassert_equal(r, 0, "BIC clear all wrong");
	zassert_true(armv7m_flag_z(apsr), "Z not set");
}

/* ORN (Thumb-2 only, no 16-bit encoding) */
ZTEST(armv7m_alu_flags, test_orns_complement_mask)
{
	uint32_t r, mask;

	/* 0xFFFF0000 is not a valid Thumb-2 modified immediate; use a register */
	__asm__ volatile(
		"movw %[m], #0\n\t"
		"movt %[m], #0xFFFF\n\t"  /* m = 0xFFFF0000 */
		"movs %[r], #0\n\t"
		"orn  %[r], %[r], %[m]\n\t"
		: [r] "=&r"(r), [m] "=&r"(mask)
		:
		: "cc");
	/* 0 ORN 0xFFFF0000 = 0 OR NOT(0xFFFF0000) = 0 OR 0x0000FFFF = 0x0000FFFF */
	zassert_equal(r, 0x0000FFFFU, "ORN wrong: 0x%08x", r);
}

/* MVN */
ZTEST(armv7m_alu_flags, test_mvns_invert)
{
	uint32_t r, apsr;

	__asm__ volatile(
		"mvns %[r], #0\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		:
		: "cc");
	zassert_equal(r, 0xFFFFFFFFU, "MVN ~0 wrong");
	zassert_true(armv7m_flag_n(apsr), "N not set");
	zassert_false(armv7m_flag_z(apsr), "Z spuriously set");
}

ZTEST(armv7m_alu_flags, test_mvns_invert_ff)
{
	uint32_t a = 0xFFFFFFFFU, r, apsr;

	__asm__ volatile(
		"mvns %[r], %[a]\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		: [a] "r"(a)
		: "cc");
	zassert_equal(r, 0, "MVN 0xFFFFFFFF wrong");
	zassert_true(armv7m_flag_z(apsr), "Z not set");
}

/* Logic with shifted register operand: carry from shifter */
ZTEST(armv7m_alu_flags, test_ands_shift_carry)
{
	uint32_t r, apsr;
	uint32_t a = 0x80000000U;

	/* LSL by 1 shifts a bit into carry */
	__asm__ volatile(
		"ands %[r], %[a], %[a], lsl #1\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		: [a] "r"(a)
		: "cc");
	/* 0x80000000 LSL 1 = 0 (C=1), AND 0x80000000 = 0 */
	zassert_equal(r, 0, "AND+shift result wrong");
	zassert_true(armv7m_flag_c(apsr), "C (shifter carry) not set");
	zassert_true(armv7m_flag_z(apsr), "Z not set");
}
