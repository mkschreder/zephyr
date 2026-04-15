/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M: Condition codes and IT block semantics.
 *
 * Reference: DDI0403E
 *   IT block  A7.7.38  lines 7776-7870
 *   Condition codes A7.3 lines 5204-5311
 *
 * All 15 conditions (EQ NE CS CC MI PL VS VC HI LS GE LT GT LE) are
 * tested by crafting appropriate N/Z/C/V values via MOVS and CMP, then
 * using a conditional MOV inside an IT block to verify execution.
 *
 * Pattern: set up flags -> IT<cond> -> MOVcc to sentinel -> check sentinel.
 */

#include <zephyr/ztest.h>
#include <armv7m_test.h>

/* Helper: execute conditional instruction inside IT block.
 * Returns 1 if the conditional executed, 0 if it was skipped. */

/* EQ: Z=1 */
ZTEST(armv7m_alu_flags, test_cond_eq_taken)
{
	uint32_t sentinel = 0;
	uint32_t val = 5;

	__asm__ volatile(
		"cmp  %[v], %[v]\n\t"    /* sets Z=1 */
		"ite  eq\n\t"
		"moveq %[s], #1\n\t"
		"movne %[s], #0\n\t"
		: [s] "+r"(sentinel)
		: [v] "r"(val)
		: "cc");
	zassert_equal(sentinel, 1, "EQ not taken");
}

ZTEST(armv7m_alu_flags, test_cond_ne_taken)
{
	uint32_t sentinel = 0;

	__asm__ volatile(
		"cmp  %[a], %[b]\n\t"    /* 1 != 2, Z=0 */
		"ite  ne\n\t"
		"movne %[s], #1\n\t"
		"moveq %[s], #0\n\t"
		: [s] "+r"(sentinel)
		: [a] "r"(1U), [b] "r"(2U)
		: "cc");
	zassert_equal(sentinel, 1, "NE not taken");
}

/* CS/HS: C=1 */
ZTEST(armv7m_alu_flags, test_cond_cs_taken)
{
	uint32_t sentinel = 0;

	__asm__ volatile(
		"movs %[s], #0xFF\n\t"
		"adds %[s], %[s], #1\n\t"  /* 0xFF+1=0x100 in 8-bit; in 32-bit no C */
		/* use 0xFFFFFFFF + 1 for C=1 */
		"mvn  %[s], #0\n\t"        /* r = 0xFFFFFFFF */
		"adds %[s], %[s], #1\n\t"  /* C=1 */
		"ite  cs\n\t"
		"movcs %[s], #1\n\t"
		"movcc %[s], #0\n\t"
		: [s] "+r"(sentinel)
		:
		: "cc");
	zassert_equal(sentinel, 1, "CS not taken");
}

/* CC/LO: C=0 */
ZTEST(armv7m_alu_flags, test_cond_cc_taken)
{
	uint32_t sentinel = 0;

	__asm__ volatile(
		"cmp  %[a], %[b]\n\t"    /* 1 < 2 unsigned: C=0 */
		"ite  cc\n\t"
		"movcc %[s], #1\n\t"
		"movcs %[s], #0\n\t"
		: [s] "+r"(sentinel)
		: [a] "r"(1U), [b] "r"(2U)
		: "cc");
	zassert_equal(sentinel, 1, "CC not taken");
}

/* MI: N=1 */
ZTEST(armv7m_alu_flags, test_cond_mi_taken)
{
	uint32_t sentinel = 0;

	__asm__ volatile(
		"movs %[s], #0\n\t"
		"subs %[s], %[s], #1\n\t"  /* -1: N=1 */
		"ite  mi\n\t"
		"movmi %[s], #1\n\t"
		"movpl %[s], #0\n\t"
		: [s] "+r"(sentinel)
		:
		: "cc");
	zassert_equal(sentinel, 1, "MI not taken");
}

/* PL: N=0 */
ZTEST(armv7m_alu_flags, test_cond_pl_taken)
{
	uint32_t sentinel = 0;

	__asm__ volatile(
		"movs %[s], #1\n\t"        /* N=0 */
		"ite  pl\n\t"
		"movpl %[s], #1\n\t"
		"movmi %[s], #0\n\t"
		: [s] "+r"(sentinel)
		:
		: "cc");
	zassert_equal(sentinel, 1, "PL not taken");
}

/* VS: V=1 */
ZTEST(armv7m_alu_flags, test_cond_vs_taken)
{
	uint32_t sentinel = 0;

	__asm__ volatile(
		"movw %[s], #0\n\t"
		"movt %[s], #0x7FFF\n\t"   /* 0x7FFF0000 */
		"adds %[s], %[s], %[s]\n\t" /* 0x7FFF0000 * 2 = signed overflow V=1 */
		/* Use simpler: 0x7FFFFFFF + 1 -> V=1 */
		"movw %[s], #0xFFFF\n\t"
		"movt %[s], #0x7FFF\n\t"   /* 0x7FFFFFFF */
		"adds %[s], %[s], #1\n\t"  /* V=1 */
		"ite  vs\n\t"
		"movvs %[s], #1\n\t"
		"movvc %[s], #0\n\t"
		: [s] "+r"(sentinel)
		:
		: "cc");
	zassert_equal(sentinel, 1, "VS not taken");
}

/* VC: V=0 */
ZTEST(armv7m_alu_flags, test_cond_vc_taken)
{
	uint32_t sentinel = 0;

	__asm__ volatile(
		"movs %[s], #1\n\t"        /* V=0 (no overflow) */
		"ite  vc\n\t"
		"movvc %[s], #1\n\t"
		"movvs %[s], #0\n\t"
		: [s] "+r"(sentinel)
		:
		: "cc");
	zassert_equal(sentinel, 1, "VC not taken");
}

/* HI: C=1 and Z=0 (unsigned higher) */
ZTEST(armv7m_alu_flags, test_cond_hi_taken)
{
	uint32_t sentinel = 0;

	__asm__ volatile(
		"cmp  %[a], %[b]\n\t"    /* 10 > 5: C=1, Z=0 */
		"ite  hi\n\t"
		"movhi %[s], #1\n\t"
		"movls %[s], #0\n\t"
		: [s] "+r"(sentinel)
		: [a] "r"(10U), [b] "r"(5U)
		: "cc");
	zassert_equal(sentinel, 1, "HI not taken");
}

/* LS: C=0 or Z=1 (unsigned lower or same) */
ZTEST(armv7m_alu_flags, test_cond_ls_taken)
{
	uint32_t sentinel = 0;

	__asm__ volatile(
		"cmp  %[a], %[b]\n\t"    /* 3 < 10: C=0, Z=0 => LS */
		"ite  ls\n\t"
		"movls %[s], #1\n\t"
		"movhi %[s], #0\n\t"
		: [s] "+r"(sentinel)
		: [a] "r"(3U), [b] "r"(10U)
		: "cc");
	zassert_equal(sentinel, 1, "LS not taken");
}

/* GE: N=V */
ZTEST(armv7m_alu_flags, test_cond_ge_taken)
{
	uint32_t sentinel = 0;

	__asm__ volatile(
		"cmp  %[a], %[b]\n\t"    /* 5 >= 3 signed: N=0,V=0 => GE */
		"ite  ge\n\t"
		"movge %[s], #1\n\t"
		"movlt %[s], #0\n\t"
		: [s] "+r"(sentinel)
		: [a] "r"(5), [b] "r"(3)
		: "cc");
	zassert_equal(sentinel, 1, "GE not taken");
}

/* LT: N!=V */
ZTEST(armv7m_alu_flags, test_cond_lt_taken)
{
	uint32_t sentinel = 0;

	__asm__ volatile(
		"cmp  %[a], %[b]\n\t"    /* -1 < 0: N=1, V=0 => LT */
		"ite  lt\n\t"
		"movlt %[s], #1\n\t"
		"movge %[s], #0\n\t"
		: [s] "+r"(sentinel)
		: [a] "r"(-1), [b] "r"(0)
		: "cc");
	zassert_equal(sentinel, 1, "LT not taken");
}

/* GT: Z=0 and N=V */
ZTEST(armv7m_alu_flags, test_cond_gt_taken)
{
	uint32_t sentinel = 0;

	__asm__ volatile(
		"cmp  %[a], %[b]\n\t"    /* 5 > 3: Z=0, N=0, V=0 => GT */
		"ite  gt\n\t"
		"movgt %[s], #1\n\t"
		"movle %[s], #0\n\t"
		: [s] "+r"(sentinel)
		: [a] "r"(5), [b] "r"(3)
		: "cc");
	zassert_equal(sentinel, 1, "GT not taken");
}

/* LE: Z=1 or N!=V */
ZTEST(armv7m_alu_flags, test_cond_le_taken)
{
	uint32_t sentinel = 0;

	__asm__ volatile(
		"cmp  %[a], %[b]\n\t"    /* 3 <= 5: N=1, V=0 => LE (N!=V) */
		"ite  le\n\t"
		"movle %[s], #1\n\t"
		"movgt %[s], #0\n\t"
		: [s] "+r"(sentinel)
		: [a] "r"(3), [b] "r"(5)
		: "cc");
	zassert_equal(sentinel, 1, "LE not taken");
}

/* IT block with 4 instructions (ITTTT) */
ZTEST(armv7m_alu_flags, test_it_block_4_then)
{
	uint32_t a = 0, b = 0, c = 0, d = 0;

	__asm__ volatile(
		"movs %[x], #0\n\t"   /* Z=1 */
		"itttt eq\n\t"
		"moveq %[a], #1\n\t"
		"moveq %[b], #2\n\t"
		"moveq %[c], #3\n\t"
		"moveq %[d], #4\n\t"
		: [a] "+r"(a), [b] "+r"(b), [c] "+r"(c), [d] "+r"(d),
		  [x] "=&r"((uint32_t){0})
		:
		: "cc");
	zassert_equal(a, 1, "ITTTT a wrong");
	zassert_equal(b, 2, "ITTTT b wrong");
	zassert_equal(c, 3, "ITTTT c wrong");
	zassert_equal(d, 4, "ITTTT d wrong");
}

/* IT block: else branch taken */
ZTEST(armv7m_alu_flags, test_it_block_ite_else)
{
	uint32_t r = 99;

	__asm__ volatile(
		"cmp  %[a], %[b]\n\t"  /* 1 != 2, NE=true, EQ=false */
		"ite  eq\n\t"
		"moveq %[r], #0\n\t"   /* not taken */
		"movne %[r], #1\n\t"   /* taken */
		: [r] "+r"(r)
		: [a] "r"(1U), [b] "r"(2U)
		: "cc");
	zassert_equal(r, 1, "ITE else branch wrong");
}

/* Verify that a skipped instruction does not modify destination */
ZTEST(armv7m_alu_flags, test_it_skip_preserves_reg)
{
	uint32_t r = 0xDEADBEEFU;

	__asm__ volatile(
		"cmp  %[a], %[b]\n\t"  /* 1 != 2, Z=0 */
		"it   eq\n\t"
		"moveq %[r], #0\n\t"   /* should be skipped */
		: [r] "+r"(r)
		: [a] "r"(1U), [b] "r"(2U)
		: "cc");
	zassert_equal(r, 0xDEADBEEFU, "IT skip did not preserve register");
}
