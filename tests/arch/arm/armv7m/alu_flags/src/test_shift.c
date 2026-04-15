/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M ALU: Shift instructions LSL / LSR / ASR / ROR / RRX.
 *
 * Reference: DDI0403E
 *   LSL A7.7.68-69  lines 9792-9935
 *   LSR A7.7.70-71  lines 9936-10077
 *   ASR A7.7.10-11  lines 6247-6384
 *   ROR lines nearby A7.7.114-115
 *   RRX (special form of ROR) line range A7.7.116
 *
 * Key rules:
 *   Shift by 0: result unchanged, C unchanged (from previous), N/Z updated
 *   LSL by 32: result = 0, C = bit[0] of original
 *   LSR by 32: result = 0, C = bit[31] of original
 *   ASR by 32: result = sign-replicated, C = bit[31]
 *   RRX: shifts right 1 with C in at bit31, old bit0 out as C
 */

#include <zephyr/ztest.h>
#include <armv7m_test.h>

/* --- LSL --------------------------------------------------------------- */

ZTEST(armv7m_alu_flags, test_lsls_by_one)
{
	uint32_t r, apsr;

	__asm__ volatile(
		"movs %[r], #1\n\t"
		"lsls %[r], %[r], #1\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		:
		: "cc");
	zassert_equal(r, 2, "LSL 1 by 1 wrong");
	zassert_false(armv7m_flag_c(apsr), "C spuriously set");
}

ZTEST(armv7m_alu_flags, test_lsls_carry_out)
{
	uint32_t r, apsr;

	__asm__ volatile(
		"movs %[r], #0x80000000\n\t"  /* not valid imm; use mov+movt */
		: [r] "=&r"(r)
		:
		: "cc");

	/* Use movw/movt to load 0x80000000 */
	__asm__ volatile(
		"movw %[r], #0\n\t"
		"movt %[r], #0x8000\n\t"
		"lsls %[r], %[r], #1\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		:
		: "cc");
	zassert_equal(r, 0, "LSL 0x80000000 by 1 wrong");
	zassert_true(armv7m_flag_c(apsr), "C not set");
	zassert_true(armv7m_flag_z(apsr), "Z not set");
}

ZTEST(armv7m_alu_flags, test_lsls_by_zero)
{
	uint32_t r, apsr;
	uint32_t val = 0x5A5A5A5AU;

	/* Clear C first */
	__asm__ volatile(
		"movs %[r], #0\n\t"          /* clears C */
		"lsls %[r], %[v], #0\n\t"   /* shift by 0 */
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		: [v] "r"(val)
		: "cc");
	zassert_equal(r, val, "LSL by 0 changed value");
}

ZTEST(armv7m_alu_flags, test_lsls_reg_by_32)
{
	uint32_t r, apsr;
	uint32_t val = 0x00000001U; /* bit0 set, will become C after LSL 32 */

	__asm__ volatile(
		"lsls %[r], %[v], %[s]\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		: [v] "r"(val), [s] "r"(32)
		: "cc");
	zassert_equal(r, 0, "LSL reg by 32 result wrong");
	zassert_true(armv7m_flag_c(apsr), "C not set after LSL reg 32");
}

/* --- LSR --------------------------------------------------------------- */

ZTEST(armv7m_alu_flags, test_lsrs_by_one)
{
	uint32_t r, apsr;

	__asm__ volatile(
		"movs %[r], #2\n\t"
		"lsrs %[r], %[r], #1\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		:
		: "cc");
	zassert_equal(r, 1, "LSR 2 by 1 wrong");
	zassert_false(armv7m_flag_c(apsr), "C spuriously set");
}

ZTEST(armv7m_alu_flags, test_lsrs_carry_out)
{
	uint32_t r, apsr;

	__asm__ volatile(
		"movs %[r], #1\n\t"
		"lsrs %[r], %[r], #1\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		:
		: "cc");
	zassert_equal(r, 0, "LSR 1 by 1 result wrong");
	zassert_true(armv7m_flag_c(apsr), "C not set");
	zassert_true(armv7m_flag_z(apsr), "Z not set");
}

ZTEST(armv7m_alu_flags, test_lsrs_no_sign_extend)
{
	uint32_t r, apsr;
	uint32_t val = 0x80000000U;

	__asm__ volatile(
		"lsrs %[r], %[v], #1\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		: [v] "r"(val)
		: "cc");
	/* LSR does NOT sign-extend; bit31 becomes 0 */
	zassert_equal(r, 0x40000000U, "LSR 0x80000000 by 1 wrong");
	zassert_false(armv7m_flag_n(apsr), "N spuriously set (no sign ext)");
}

/* --- ASR --------------------------------------------------------------- */

ZTEST(armv7m_alu_flags, test_asrs_sign_extend)
{
	uint32_t r, apsr;
	uint32_t val = 0x80000000U;

	__asm__ volatile(
		"asrs %[r], %[v], #1\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		: [v] "r"(val)
		: "cc");
	/* ASR extends sign: 0x80000000 >> 1 = 0xC0000000 */
	zassert_equal(r, 0xC0000000U, "ASR sign-extend wrong");
	zassert_true(armv7m_flag_n(apsr), "N not set");
}

ZTEST(armv7m_alu_flags, test_asrs_all_ones_at_32)
{
	uint32_t r, apsr;
	uint32_t val = 0x80000000U;

	__asm__ volatile(
		"asrs %[r], %[v], %[s]\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		: [v] "r"(val), [s] "r"(32)
		: "cc");
	/* ASR by 32 on negative: result = 0xFFFFFFFF, C = bit31 = 1 */
	zassert_equal(r, 0xFFFFFFFFU, "ASR 32 neg wrong");
	zassert_true(armv7m_flag_c(apsr), "C not set");
	zassert_true(armv7m_flag_n(apsr), "N not set");
}

ZTEST(armv7m_alu_flags, test_asrs_positive_no_sign)
{
	uint32_t r, apsr;

	__asm__ volatile(
		"movs %[r], #0x7F\n\t"
		"asrs %[r], %[r], #4\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		:
		: "cc");
	zassert_equal(r, 7, "ASR positive wrong");
	zassert_false(armv7m_flag_n(apsr), "N spuriously set");
}

/* --- ROR --------------------------------------------------------------- */

ZTEST(armv7m_alu_flags, test_rors_by_one)
{
	uint32_t r, apsr;

	__asm__ volatile(
		"movs %[r], #1\n\t"
		"rors %[r], %[r], %[s]\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		: [s] "r"(1)
		: "cc");
	/* Rotate 0x00000001 right 1: 0x80000000, C=1 */
	zassert_equal(r, 0x80000000U, "ROR 1 by 1 wrong");
	zassert_true(armv7m_flag_c(apsr), "C not set");
	zassert_true(armv7m_flag_n(apsr), "N not set");
}

ZTEST(armv7m_alu_flags, test_rors_by_32_identity)
{
	uint32_t r, apsr;
	uint32_t val = 0x12345678U;

	__asm__ volatile(
		"rors %[r], %[v], %[s]\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		: [v] "r"(val), [s] "r"(32)
		: "cc");
	/* ROR by 32 = identity */
	zassert_equal(r, val, "ROR by 32 identity wrong");
}

/* --- RRX --------------------------------------------------------------- */

ZTEST(armv7m_alu_flags, test_rrx_carry_in)
{
	uint32_t r, apsr;

	/* Set C=1 via MOVS 0x80000000 LSL 1 */
	__asm__ volatile(
		"movw %[r], #0\n\t"
		"movt %[r], #0x8000\n\t"
		"lsls %[r], %[r], #1\n\t"   /* C=1, r=0 */
		"rrxs %[r], %[r]\n\t"        /* RRX: bit31=C=1, C=bit0=0 */
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		:
		: "cc");
	zassert_equal(r, 0x80000000U, "RRX carry-in wrong");
	zassert_false(armv7m_flag_c(apsr), "C not cleared");
	zassert_true(armv7m_flag_n(apsr), "N not set");
}

ZTEST(armv7m_alu_flags, test_rrx_carry_out)
{
	uint32_t r, apsr;

	/* Reliably clear C using ADDS 0+0 (carry out = 0).
	 * Then load 1 into r (MOVS without shift does NOT modify C).
	 * RRX 1 with C=0: result = (0<<31 | 1>>1) = 0, new C = old bit0 = 1. */
	__asm__ volatile(
		"movs %[r], #0\n\t"          /* r=0 */
		"adds %[r], %[r], #0\n\t"    /* C=0 (0+0 no carry) */
		"mov  %[r], #1\n\t"          /* r=1, C unchanged (MOV no flag update) */
		"rrxs %[r], %[r]\n\t"        /* RRX: result=0, C=1 (old bit0=1) */
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		:
		: "cc");
	zassert_equal(r, 0, "RRX carry-out result wrong: 0x%08x", r);
	zassert_true(armv7m_flag_c(apsr), "C not set after RRX 1");
}
