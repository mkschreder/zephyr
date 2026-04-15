/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M ALU: Multiply and divide instructions.
 *
 * Reference: DDI0403E
 *   MUL    A7.7.84  lines 10831-10902
 *   MLA    A7.7.74  lines 10248-10303
 *   MLS    A7.7.75  lines 10304-10357
 *   SMULL  A7.7.149 lines 14624-14677
 *   UMULL  A7.7.204 lines 18047-18100
 *   SMLAL  A7.7.138 lines 13981-14034
 *   UMLAL  A7.7.203 lines 17993-18046
 *   SDIV   A7.7.127 lines 13389-13444
 *   UDIV   A7.7.195 lines 17573-17622
 */

#include <zephyr/ztest.h>
#include <armv7m_test.h>

/* MUL */
ZTEST(armv7m_alu_flags, test_mul_basic)
{
	uint32_t r;

	__asm__ volatile("mul %[r], %[a], %[b]\n\t"
			 : [r] "=r"(r)
			 : [a] "r"(6U), [b] "r"(7U)
			 : );
	zassert_equal(r, 42, "MUL 6x7 wrong");
}

ZTEST(armv7m_alu_flags, test_mul_zero)
{
	uint32_t r, apsr;

	/* Use mul then movs to check Z flag (MULS not available in Thumb-2 32-bit) */
	__asm__ volatile(
		"mul  %[r], %[a], %[b]\n\t"
		"movs %[r], %[r]\n\t"       /* MOVS to update Z */
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		: [a] "r"(0U), [b] "r"(0xFFFFFFFFU)
		: "cc");
	zassert_equal(r, 0, "MUL 0xFFFFFFFF*0 wrong");
	zassert_true(armv7m_flag_z(apsr), "Z not set");
}

ZTEST(armv7m_alu_flags, test_mul_wrap32)
{
	uint32_t r;

	/* 0x10000 * 0x10000 = 0x100000000 which truncates to 0 */
	__asm__ volatile("mul %[r], %[a], %[b]\n\t"
			 : [r] "=r"(r)
			 : [a] "r"(0x10000U), [b] "r"(0x10000U)
			 : );
	zassert_equal(r, 0, "MUL 64k*64k truncation wrong");
}

ZTEST(armv7m_alu_flags, test_mul_max_unsigned)
{
	uint32_t r;

	/* 0xFFFFFFFF * 0xFFFFFFFF low 32 = 0x00000001 */
	__asm__ volatile("mul %[r], %[a], %[b]\n\t"
			 : [r] "=r"(r)
			 : [a] "r"(0xFFFFFFFFU), [b] "r"(0xFFFFFFFFU)
			 : );
	zassert_equal(r, 1, "MUL 0xFFFFFFFF^2 low32 wrong: 0x%08x", r);
}

/* MLA: Rd = Ra + Rn*Rm */
ZTEST(armv7m_alu_flags, test_mla_accumulate)
{
	uint32_t r;

	__asm__ volatile("mla %[r], %[n], %[m], %[a]\n\t"
			 : [r] "=r"(r)
			 : [n] "r"(3U), [m] "r"(4U), [a] "r"(5U)
			 : );
	zassert_equal(r, 17, "MLA 5+3*4 wrong");
}

/* MLS: Rd = Ra - Rn*Rm */
ZTEST(armv7m_alu_flags, test_mls_subtract)
{
	uint32_t r;

	__asm__ volatile("mls %[r], %[n], %[m], %[a]\n\t"
			 : [r] "=r"(r)
			 : [n] "r"(3U), [m] "r"(4U), [a] "r"(100U)
			 : );
	zassert_equal(r, 88, "MLS 100-3*4 wrong");
}

/* UMULL: 64-bit unsigned */
ZTEST(armv7m_alu_flags, test_umull_basic)
{
	uint32_t hi, lo;

	__asm__ volatile("umull %[lo], %[hi], %[a], %[b]\n\t"
			 : [hi] "=r"(hi), [lo] "=r"(lo)
			 : [a] "r"(0x10000U), [b] "r"(0x10000U)
			 : );
	/* 0x10000 * 0x10000 = 0x100000000: hi=1, lo=0 */
	zassert_equal(hi, 1, "UMULL hi wrong");
	zassert_equal(lo, 0, "UMULL lo wrong");
}

ZTEST(armv7m_alu_flags, test_umull_max)
{
	uint32_t hi, lo;

	/* 0xFFFFFFFF * 0xFFFFFFFF = 0xFFFFFFFE00000001 */
	__asm__ volatile("umull %[lo], %[hi], %[a], %[b]\n\t"
			 : [hi] "=r"(hi), [lo] "=r"(lo)
			 : [a] "r"(0xFFFFFFFFU), [b] "r"(0xFFFFFFFFU)
			 : );
	zassert_equal(hi, 0xFFFFFFFEU, "UMULL max hi wrong: 0x%08x", hi);
	zassert_equal(lo, 0x00000001U, "UMULL max lo wrong: 0x%08x", lo);
}

/* SMULL: 64-bit signed */
ZTEST(armv7m_alu_flags, test_smull_negative)
{
	uint32_t hi, lo;

	/* -1 * -1 = 1 */
	__asm__ volatile("smull %[lo], %[hi], %[a], %[b]\n\t"
			 : [hi] "=r"(hi), [lo] "=r"(lo)
			 : [a] "r"(-1), [b] "r"(-1)
			 : );
	zassert_equal(hi, 0, "SMULL -1*-1 hi wrong");
	zassert_equal(lo, 1, "SMULL -1*-1 lo wrong");
}

ZTEST(armv7m_alu_flags, test_smull_signed_overflow)
{
	uint32_t hi, lo;

	/* 0x80000000 * 2 = 0xFFFFFFFF00000000 */
	__asm__ volatile("smull %[lo], %[hi], %[a], %[b]\n\t"
			 : [hi] "=r"(hi), [lo] "=r"(lo)
			 : [a] "r"(0x80000000U), [b] "r"(2U)
			 : );
	zassert_equal(hi, 0xFFFFFFFFU, "SMULL overflow hi wrong: 0x%08x", hi);
	zassert_equal(lo, 0, "SMULL overflow lo wrong");
}

/* UMLAL: lo:hi += a * b */
ZTEST(armv7m_alu_flags, test_umlal_accumulate)
{
	uint32_t hi = 0, lo = 5;

	__asm__ volatile("umlal %[lo], %[hi], %[a], %[b]\n\t"
			 : [hi] "+r"(hi), [lo] "+r"(lo)
			 : [a] "r"(3U), [b] "r"(4U)
			 : );
	/* 0 + 5 + 12 = 17 */
	zassert_equal(lo, 17, "UMLAL accumulate lo wrong");
	zassert_equal(hi, 0, "UMLAL accumulate hi wrong");
}

/* SMLAL */
ZTEST(armv7m_alu_flags, test_smlal_signed)
{
	uint32_t hi = 0xFFFFFFFFU, lo = 0xFFFFFFF0U;

	/* -16 + (-1 * 4) = -20 = 0xFFFFFFFFFFFFFFEC */
	__asm__ volatile("smlal %[lo], %[hi], %[a], %[b]\n\t"
			 : [hi] "+r"(hi), [lo] "+r"(lo)
			 : [a] "r"(-1), [b] "r"(4U)
			 : );
	zassert_equal(lo, 0xFFFFFFECU, "SMLAL lo wrong: 0x%08x", lo);
	zassert_equal(hi, 0xFFFFFFFFU, "SMLAL hi wrong: 0x%08x", hi);
}

/* SDIV */
ZTEST(armv7m_alu_flags, test_sdiv_positive)
{
	uint32_t r;

	__asm__ volatile("sdiv %[r], %[a], %[b]\n\t"
			 : [r] "=r"(r)
			 : [a] "r"(20), [b] "r"(3)
			 : );
	/* Truncates toward zero: 20/3 = 6 */
	zassert_equal(r, 6, "SDIV 20/3 wrong");
}

ZTEST(armv7m_alu_flags, test_sdiv_negative_truncate)
{
	uint32_t r;

	/* -7 / 2 = -3 (toward zero) */
	__asm__ volatile("sdiv %[r], %[a], %[b]\n\t"
			 : [r] "=r"(r)
			 : [a] "r"(-7), [b] "r"(2)
			 : );
	zassert_equal((int32_t)r, -3, "SDIV -7/2 wrong: %d", (int32_t)r);
}

ZTEST(armv7m_alu_flags, test_sdiv_exact)
{
	uint32_t r;

	__asm__ volatile("sdiv %[r], %[a], %[b]\n\t"
			 : [r] "=r"(r)
			 : [a] "r"(100), [b] "r"(5)
			 : );
	zassert_equal(r, 20, "SDIV 100/5 wrong");
}

/* UDIV */
ZTEST(armv7m_alu_flags, test_udiv_basic)
{
	uint32_t r;

	__asm__ volatile("udiv %[r], %[a], %[b]\n\t"
			 : [r] "=r"(r)
			 : [a] "r"(100U), [b] "r"(7U)
			 : );
	zassert_equal(r, 14, "UDIV 100/7 wrong");
}

ZTEST(armv7m_alu_flags, test_udiv_max)
{
	uint32_t r;

	__asm__ volatile("udiv %[r], %[a], %[b]\n\t"
			 : [r] "=r"(r)
			 : [a] "r"(0xFFFFFFFFU), [b] "r"(0xFFFFFFFFU)
			 : );
	zassert_equal(r, 1, "UDIV max/max wrong");
}
