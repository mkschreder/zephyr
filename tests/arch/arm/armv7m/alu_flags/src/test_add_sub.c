/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M ALU torture: ADD / SUB / ADC / SBC / RSB family.
 *
 * Reference: DDI0403E
 *   ADD (imm)  A7.7.3  lines 5601-5716
 *   ADD (reg)  A7.7.4  lines 5717-5814
 *   ADC (imm)  A7.7.1  lines 5465-5520
 *   ADC (reg)  A7.7.2  lines 5521-5600
 *   SUB (imm)  A7.7.176
 *   SBC (imm/reg) A7.7.119-120
 *   RSB (imm/reg) A7.7.116-118
 *
 * APSR flags: N bit[31], Z bit[30], C bit[29], V bit[28] (lines 769-794)
 */

#include <zephyr/ztest.h>
#include <armv7m_test.h>

/* Helper: run ADDS, return result and APSR. */
static inline uint32_t adds_r_r(uint32_t a, uint32_t b, uint32_t *apsr_out)
{
	uint32_t result, apsr;

	__asm__ volatile(
		"adds %[r], %[a], %[b]\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(result), [f] "=r"(apsr)
		: [a] "r"(a), [b] "r"(b)
		: "cc");
	*apsr_out = apsr;
	return result;
}

static inline uint32_t subs_r_r(uint32_t a, uint32_t b, uint32_t *apsr_out)
{
	uint32_t result, apsr;

	__asm__ volatile(
		"subs %[r], %[a], %[b]\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(result), [f] "=r"(apsr)
		: [a] "r"(a), [b] "r"(b)
		: "cc");
	*apsr_out = apsr;
	return result;
}

/* ADC: result = a + b + carry_in */
static inline uint32_t adcs(uint32_t a, uint32_t b, uint32_t carry_in,
			     uint32_t *apsr_out)
{
	uint32_t result, apsr;

	__asm__ volatile(
		/* Set carry flag from carry_in */
		"lsrs %[ci], %[ci], #1\n\t" /* shift bit 0 into C */
		"adcs %[r], %[a], %[b]\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(result), [f] "=r"(apsr), [ci] "+r"(carry_in)
		: [a] "r"(a), [b] "r"(b)
		: "cc");
	*apsr_out = apsr;
	return result;
}

/* SBC: result = a - b - NOT(carry_in) */
static inline uint32_t sbcs(uint32_t a, uint32_t b, uint32_t carry_in,
			     uint32_t *apsr_out)
{
	uint32_t result, apsr;

	__asm__ volatile(
		"lsrs %[ci], %[ci], #1\n\t"
		"sbcs %[r], %[a], %[b]\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(result), [f] "=r"(apsr), [ci] "+r"(carry_in)
		: [a] "r"(a), [b] "r"(b)
		: "cc");
	*apsr_out = apsr;
	return result;
}

static inline uint32_t rsbs(uint32_t a, uint32_t *apsr_out)
{
	uint32_t result, apsr;

	__asm__ volatile(
		"rsbs %[r], %[a], #0\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(result), [f] "=r"(apsr)
		: [a] "r"(a)
		: "cc");
	*apsr_out = apsr;
	return result;
}

/* --- ADDS tests -------------------------------------------------------- */

ZTEST(armv7m_alu_flags, test_adds_zero_plus_zero)
{
	uint32_t apsr;
	uint32_t r = adds_r_r(0, 0, &apsr);

	zassert_equal(r, 0, "0+0 result wrong: 0x%08x", r);
	zassert_true(armv7m_flag_z(apsr), "Z not set");
	zassert_false(armv7m_flag_n(apsr), "N spuriously set");
	zassert_false(armv7m_flag_c(apsr), "C spuriously set");
	zassert_false(armv7m_flag_v(apsr), "V spuriously set");
}

ZTEST(armv7m_alu_flags, test_adds_zero_plus_one)
{
	uint32_t apsr;
	uint32_t r = adds_r_r(0, 1, &apsr);

	zassert_equal(r, 1, "0+1 result wrong");
	zassert_false(armv7m_flag_z(apsr), "Z spuriously set");
	zassert_false(armv7m_flag_n(apsr), "N spuriously set");
	zassert_false(armv7m_flag_c(apsr), "C spuriously set");
	zassert_false(armv7m_flag_v(apsr), "V spuriously set");
}

/* MAX + 1: carry + zero, no overflow (unsigned wraps, not signed) */
ZTEST(armv7m_alu_flags, test_adds_max_plus_one_carry_zero)
{
	uint32_t apsr;
	uint32_t r = adds_r_r(0xFFFFFFFFU, 1U, &apsr);

	zassert_equal(r, 0, "0xFFFFFFFF+1 result wrong: 0x%08x", r);
	zassert_true(armv7m_flag_z(apsr), "Z not set");
	zassert_true(armv7m_flag_c(apsr), "C not set");
	zassert_false(armv7m_flag_v(apsr), "V spuriously set");
	zassert_false(armv7m_flag_n(apsr), "N spuriously set");
}

/* 0x7FFFFFFF + 1: signed overflow, not unsigned carry */
ZTEST(armv7m_alu_flags, test_adds_signed_overflow)
{
	uint32_t apsr;
	uint32_t r = adds_r_r(0x7FFFFFFFU, 1U, &apsr);

	zassert_equal(r, 0x80000000U, "overflow result wrong: 0x%08x", r);
	zassert_false(armv7m_flag_z(apsr), "Z spuriously set");
	zassert_true(armv7m_flag_n(apsr), "N not set");
	zassert_false(armv7m_flag_c(apsr), "C spuriously set");
	zassert_true(armv7m_flag_v(apsr), "V not set");
}

/* 0x80000000 + 0x80000000: both carry and overflow */
ZTEST(armv7m_alu_flags, test_adds_carry_and_overflow)
{
	uint32_t apsr;
	uint32_t r = adds_r_r(0x80000000U, 0x80000000U, &apsr);

	zassert_equal(r, 0, "0x80000000+0x80000000 result wrong");
	zassert_true(armv7m_flag_z(apsr), "Z not set");
	zassert_true(armv7m_flag_c(apsr), "C not set");
	zassert_true(armv7m_flag_v(apsr), "V not set");
}

/* Negative result: N set */
ZTEST(armv7m_alu_flags, test_adds_negative_result)
{
	uint32_t apsr;
	uint32_t r = adds_r_r(0xFFFFFFFEU, 1U, &apsr);

	zassert_equal(r, 0xFFFFFFFFU, "result wrong");
	zassert_true(armv7m_flag_n(apsr), "N not set");
	zassert_false(armv7m_flag_z(apsr), "Z spuriously set");
	zassert_false(armv7m_flag_c(apsr), "C spuriously set");
	zassert_false(armv7m_flag_v(apsr), "V spuriously set");
}

ZTEST(armv7m_alu_flags, test_adds_imm_commutative)
{
	uint32_t apsr1, apsr2;
	uint32_t r1 = adds_r_r(0x12345678U, 0x9ABCDEF0U, &apsr1);
	uint32_t r2 = adds_r_r(0x9ABCDEF0U, 0x12345678U, &apsr2);

	zassert_equal(r1, r2, "ADD not commutative");
	zassert_equal(apsr1 & ARMV7M_NZCV_MASK, apsr2 & ARMV7M_NZCV_MASK,
		      "flags differ");
}

/* --- SUBS tests -------------------------------------------------------- */

ZTEST(armv7m_alu_flags, test_subs_zero_minus_zero)
{
	uint32_t apsr;
	uint32_t r = subs_r_r(0, 0, &apsr);

	zassert_equal(r, 0, "0-0 result wrong");
	zassert_true(armv7m_flag_z(apsr), "Z not set");
	zassert_true(armv7m_flag_c(apsr), "C (borrow=0) not set");
	zassert_false(armv7m_flag_n(apsr), "N spuriously set");
	zassert_false(armv7m_flag_v(apsr), "V spuriously set");
}

ZTEST(armv7m_alu_flags, test_subs_borrow)
{
	uint32_t apsr;
	uint32_t r = subs_r_r(0, 1, &apsr);

	zassert_equal(r, 0xFFFFFFFFU, "0-1 result wrong");
	zassert_true(armv7m_flag_n(apsr), "N not set");
	zassert_false(armv7m_flag_z(apsr), "Z spuriously set");
	zassert_false(armv7m_flag_c(apsr), "C (borrow=1) spuriously set");
	zassert_false(armv7m_flag_v(apsr), "V spuriously set");
}

/* 0x80000000 - 1: signed underflow+overflow */
ZTEST(armv7m_alu_flags, test_subs_signed_underflow)
{
	uint32_t apsr;
	uint32_t r = subs_r_r(0x80000000U, 1U, &apsr);

	zassert_equal(r, 0x7FFFFFFFU, "result wrong");
	zassert_true(armv7m_flag_v(apsr), "V not set");
	zassert_true(armv7m_flag_c(apsr), "C not set");
	zassert_false(armv7m_flag_n(apsr), "N spuriously set");
}

ZTEST(armv7m_alu_flags, test_subs_equal)
{
	uint32_t apsr;
	uint32_t r = subs_r_r(0xDEADBEEFU, 0xDEADBEEFU, &apsr);

	zassert_equal(r, 0, "equal sub result wrong");
	zassert_true(armv7m_flag_z(apsr), "Z not set");
	zassert_true(armv7m_flag_c(apsr), "C not set");
}

/* --- ADC tests --------------------------------------------------------- */

ZTEST(armv7m_alu_flags, test_adcs_no_carry)
{
	uint32_t apsr;
	/* carry_in = 0 (bit 0 = 0 -- shifted into C gives C=0) */
	uint32_t r = adcs(5, 3, 0, &apsr);

	zassert_equal(r, 8, "ADC no-carry wrong");
}

ZTEST(armv7m_alu_flags, test_adcs_with_carry)
{
	uint32_t apsr;
	/* carry_in = 1 (bit 0 = 1 -- shifted gives C=1) */
	uint32_t r = adcs(5, 3, 1, &apsr);

	/* 5 + 3 + 1 = 9 */
	zassert_equal(r, 9, "ADC with carry wrong: %d", r);
}

ZTEST(armv7m_alu_flags, test_adcs_max_carry_propagate)
{
	uint32_t apsr;
	/* 0xFFFFFFFF + 0 + C=1 = 0 with carry out */
	uint32_t r = adcs(0xFFFFFFFFU, 0U, 1, &apsr);

	zassert_equal(r, 0, "ADC max+C=1 result wrong");
	zassert_true(armv7m_flag_z(apsr), "Z not set");
	zassert_true(armv7m_flag_c(apsr), "C not set");
}

/* --- SBC tests --------------------------------------------------------- */

ZTEST(armv7m_alu_flags, test_sbcs_no_borrow)
{
	uint32_t apsr;
	/* C=1 means no borrow: result = a - b */
	uint32_t r = sbcs(10, 3, 1, &apsr);

	zassert_equal(r, 7, "SBC no-borrow wrong: %d", r);
}

ZTEST(armv7m_alu_flags, test_sbcs_with_borrow)
{
	uint32_t apsr;
	/* C=0 means borrow: result = a - b - 1 */
	uint32_t r = sbcs(10, 3, 0, &apsr);

	zassert_equal(r, 6, "SBC with borrow wrong: %d", r);
}

/* --- RSB tests --------------------------------------------------------- */

ZTEST(armv7m_alu_flags, test_rsbs_zero)
{
	uint32_t apsr;
	/* RSB 0, #0 = 0 */
	uint32_t r = rsbs(0, &apsr);

	zassert_equal(r, 0, "RSB 0 wrong");
	zassert_true(armv7m_flag_z(apsr), "Z not set");
	zassert_true(armv7m_flag_c(apsr), "C not set");
}

ZTEST(armv7m_alu_flags, test_rsbs_negate)
{
	uint32_t apsr;
	/* RSB 1, #0 = -1 = 0xFFFFFFFF */
	uint32_t r = rsbs(1, &apsr);

	zassert_equal(r, 0xFFFFFFFFU, "RSB 1 wrong");
	zassert_true(armv7m_flag_n(apsr), "N not set");
	zassert_false(armv7m_flag_c(apsr), "C spuriously set");
}

/* --- ADD immediate (wide encoding T3/T4) variants --------------------- */

ZTEST(armv7m_alu_flags, test_add_imm12)
{
	uint32_t result;

	__asm__ volatile(
		"movw %[r], #0\n\t"
		"addw %[r], %[r], #0xFFF\n\t" /* ADDW: imm12, no flag update */
		: [r] "=&r"(result)
		:
		: );
	zassert_equal(result, 0xFFF, "ADDW imm12 wrong: 0x%x", result);
}

/* ADD SP + imm (A7.7.5) */
ZTEST(armv7m_alu_flags, test_add_sp_plus_imm)
{
	uint32_t sp_val, result;

	__asm__ volatile(
		"mov %[s], sp\n\t"
		"add %[r], sp, #8\n\t"
		: [s] "=&r"(sp_val), [r] "=&r"(result)
		:
		: );
	zassert_equal(result, sp_val + 8, "ADD SP+8 wrong");
}
