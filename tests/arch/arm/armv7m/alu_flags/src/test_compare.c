/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M ALU: Compare instructions CMP / CMN / TST / TEQ.
 *
 * Reference: DDI0403E
 *   CMP A7.7.27-28  lines 7229-7372
 *   CMN A7.7.25-26  lines 7115-7228
 *   TST A7.7.193
 *   TEQ A7.7.186
 *
 * Compare instructions always update N/Z/C/V; they do not write a result.
 */

#include <zephyr/ztest.h>
#include <armv7m_test.h>

/* ---- CMP -------------------------------------------------------------- */

ZTEST(armv7m_alu_flags, test_cmp_equal)
{
	uint32_t apsr;
	uint32_t a = 0x12345678U;

	__asm__ volatile(
		"cmp %[a], %[a]\n\t"
		"mrs %[f], apsr\n\t"
		: [f] "=r"(apsr)
		: [a] "r"(a)
		: "cc");
	zassert_true(armv7m_flag_z(apsr), "Z not set for equal CMP");
	zassert_true(armv7m_flag_c(apsr), "C not set for equal CMP");
	zassert_false(armv7m_flag_n(apsr), "N spuriously set");
	zassert_false(armv7m_flag_v(apsr), "V spuriously set");
}

ZTEST(armv7m_alu_flags, test_cmp_less_unsigned)
{
	uint32_t apsr;

	__asm__ volatile(
		"cmp %[a], %[b]\n\t"
		"mrs %[f], apsr\n\t"
		: [f] "=r"(apsr)
		: [a] "r"(3U), [b] "r"(5U)
		: "cc");
	/* 3 < 5 unsigned: C=0 (borrow), N=1 */
	zassert_false(armv7m_flag_c(apsr), "C spuriously set for 3<5");
	zassert_false(armv7m_flag_z(apsr), "Z spuriously set");
	zassert_true(armv7m_flag_n(apsr), "N not set for 3<5");
}

ZTEST(armv7m_alu_flags, test_cmp_greater_unsigned)
{
	uint32_t apsr;

	__asm__ volatile(
		"cmp %[a], %[b]\n\t"
		"mrs %[f], apsr\n\t"
		: [f] "=r"(apsr)
		: [a] "r"(10U), [b] "r"(5U)
		: "cc");
	/* 10 > 5: C=1, N=0 */
	zassert_true(armv7m_flag_c(apsr), "C not set for 10>5");
	zassert_false(armv7m_flag_n(apsr), "N spuriously set");
	zassert_false(armv7m_flag_z(apsr), "Z spuriously set");
}

ZTEST(armv7m_alu_flags, test_cmp_signed_overflow)
{
	uint32_t apsr;

	/* 0x80000000 - 1 = 0x7FFFFFFF: signed overflow */
	__asm__ volatile(
		"cmp %[a], %[b]\n\t"
		"mrs %[f], apsr\n\t"
		: [f] "=r"(apsr)
		: [a] "r"(0x80000000U), [b] "r"(1U)
		: "cc");
	zassert_true(armv7m_flag_v(apsr), "V not set for signed overflow");
	zassert_true(armv7m_flag_c(apsr), "C not set");
}

ZTEST(armv7m_alu_flags, test_cmp_zero_minus_one)
{
	uint32_t apsr;

	__asm__ volatile(
		"cmp %[a], %[b]\n\t"
		"mrs %[f], apsr\n\t"
		: [f] "=r"(apsr)
		: [a] "r"(0U), [b] "r"(1U)
		: "cc");
	zassert_false(armv7m_flag_c(apsr), "C should be 0 (borrow)");
	zassert_true(armv7m_flag_n(apsr), "N not set");
}

/* CMP with immediate */
ZTEST(armv7m_alu_flags, test_cmp_imm_equal)
{
	uint32_t apsr;
	uint32_t a = 42;

	__asm__ volatile(
		"cmp %[a], #42\n\t"
		"mrs %[f], apsr\n\t"
		: [f] "=r"(apsr)
		: [a] "r"(a)
		: "cc");
	zassert_true(armv7m_flag_z(apsr), "Z not set");
}

/* ---- CMN -------------------------------------------------------------- */

ZTEST(armv7m_alu_flags, test_cmn_zero_plus_zero)
{
	uint32_t apsr;

	__asm__ volatile(
		"cmn %[a], #0\n\t"
		"mrs %[f], apsr\n\t"
		: [f] "=r"(apsr)
		: [a] "r"(0U)
		: "cc");
	zassert_true(armv7m_flag_z(apsr), "Z not set for CMN 0+0");
}

ZTEST(armv7m_alu_flags, test_cmn_overflow)
{
	uint32_t apsr;

	/* CMN 0x7FFFFFFF, 1: same as ADDS 0x7FFFFFFF + 1, V must be set */
	__asm__ volatile(
		"cmn %[a], %[b]\n\t"
		"mrs %[f], apsr\n\t"
		: [f] "=r"(apsr)
		: [a] "r"(0x7FFFFFFFU), [b] "r"(1U)
		: "cc");
	zassert_true(armv7m_flag_v(apsr), "V not set");
	zassert_false(armv7m_flag_c(apsr), "C spuriously set");
	zassert_true(armv7m_flag_n(apsr), "N not set");
}

/* ---- TST -------------------------------------------------------------- */

ZTEST(armv7m_alu_flags, test_tst_bit_set)
{
	uint32_t apsr;
	uint32_t a = 0x00000010U;

	__asm__ volatile(
		"tst %[a], #0x10\n\t"
		"mrs %[f], apsr\n\t"
		: [f] "=r"(apsr)
		: [a] "r"(a)
		: "cc");
	zassert_false(armv7m_flag_z(apsr), "Z spuriously set (bit is set)");
}

ZTEST(armv7m_alu_flags, test_tst_bit_clear)
{
	uint32_t apsr;
	uint32_t a = 0x00000010U;

	__asm__ volatile(
		"tst %[a], #0x20\n\t"
		"mrs %[f], apsr\n\t"
		: [f] "=r"(apsr)
		: [a] "r"(a)
		: "cc");
	zassert_true(armv7m_flag_z(apsr), "Z not set (bit is clear)");
}

ZTEST(armv7m_alu_flags, test_tst_high_bit)
{
	uint32_t apsr;
	uint32_t a = 0x80000001U;

	__asm__ volatile(
		"tst %[a], %[b]\n\t"
		"mrs %[f], apsr\n\t"
		: [f] "=r"(apsr)
		: [a] "r"(a), [b] "r"(0x80000000U)
		: "cc");
	zassert_true(armv7m_flag_n(apsr), "N not set for bit31");
	zassert_false(armv7m_flag_z(apsr), "Z spuriously set");
}

/* ---- TEQ -------------------------------------------------------------- */

ZTEST(armv7m_alu_flags, test_teq_equal)
{
	uint32_t apsr;
	uint32_t a = 0xCAFEBABEU;

	__asm__ volatile(
		"teq %[a], %[a]\n\t"
		"mrs %[f], apsr\n\t"
		: [f] "=r"(apsr)
		: [a] "r"(a)
		: "cc");
	zassert_true(armv7m_flag_z(apsr), "Z not set for equal TEQ");
}

ZTEST(armv7m_alu_flags, test_teq_different)
{
	uint32_t apsr;

	__asm__ volatile(
		"teq %[a], %[b]\n\t"
		"mrs %[f], apsr\n\t"
		: [f] "=r"(apsr)
		: [a] "r"(0xAAAAAAAAU), [b] "r"(0x55555555U)
		: "cc");
	zassert_false(armv7m_flag_z(apsr), "Z spuriously set");
	zassert_true(armv7m_flag_n(apsr), "N not set");
}
