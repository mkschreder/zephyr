/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M branch instructions.
 *
 * Reference: DDI0403E
 *   B          A7.7.12  lines 6385-6484
 *   BL         A7.7.18  lines 6765-6812
 *   BLX (reg)  A7.7.19  lines 6813-6856
 *   BX         A7.7.20  lines 6857-6898
 *   CBNZ/CBZ   A7.7.21  lines 6899-6948
 */

#include <zephyr/ztest.h>
#include <armv7m_test.h>

/* B: forward unconditional branch */
ZTEST(armv7m_branch_it, test_b_forward)
{
	uint32_t r = 0;

	__asm__ volatile(
		"b    1f\n\t"
		"movs %[r], #0xFF\n\t"   /* should be skipped */
		"1:\n\t"
		"movs %[r], #1\n\t"
		: [r] "+r"(r)
		:
		: "cc");
	zassert_equal(r, 1, "B forward did not branch");
}

/* B: backward branch (loop) */
ZTEST(armv7m_branch_it, test_b_backward_loop)
{
	uint32_t count = 0;

	__asm__ volatile(
		"movs %[c], #0\n\t"
		"1:\n\t"
		"adds %[c], %[c], #1\n\t"
		"cmp  %[c], #5\n\t"
		"bne  1b\n\t"
		: [c] "+r"(count)
		:
		: "cc");
	zassert_equal(count, 5, "B backward loop wrong: %d", count);
}

/* BL: saves return address in LR */
static volatile uint32_t bl_test_lr;

__attribute__((noinline)) static void bl_target(void)
{
	/* Capture LR to verify BL set it correctly */
	__asm__ volatile("mov %0, lr\n\t" : "=r"(bl_test_lr) : : );
}

ZTEST(armv7m_branch_it, test_bl_saves_lr)
{
	uint32_t pc_after_bl;

	__asm__ volatile(
		"bl   %[fn]\n\t"
		"mov  %[pc], pc\n\t"
		: [pc] "=r"(pc_after_bl)
		: [fn] "X"(bl_target)
		: "lr", "memory");
	/* LR saved by BL points to instruction after BL (Thumb bit set) */
	zassert_not_equal(bl_test_lr, 0, "BL did not set LR");
	/* LR must have Thumb bit set (bit0=1) */
	zassert_true(bl_test_lr & 1, "BL LR Thumb bit not set");
}

/* BX: branch to register (Thumb stay) */
ZTEST(armv7m_branch_it, test_bx_lr_returns)
{
	uint32_t result = 0;

	__asm__ volatile(
		"push {lr}\n\t"
		"bl   1f\n\t"
		"b    2f\n\t"
		"1:\n\t"
		"mov  %[r], #42\n\t"
		"bx   lr\n\t"
		"2:\n\t"
		"pop  {lr}\n\t"
		: [r] "+r"(result)
		:
		: "memory");
	zassert_equal(result, 42, "BX LR return wrong");
}

/* BLX (register): call via register */
ZTEST(armv7m_branch_it, test_blx_reg)
{
	uint32_t result = 0;
	uintptr_t fn_addr = (uintptr_t)bl_target | 1U; /* Thumb bit */

	bl_test_lr = 0;
	__asm__ volatile(
		"blx %[fn]\n\t"
		: : [fn] "r"(fn_addr) : "lr", "memory");
	/* bl_target sets bl_test_lr */
	zassert_not_equal(bl_test_lr, 0, "BLX reg did not call target");
}

/* CBZ: branch if zero */
ZTEST(armv7m_branch_it, test_cbz_taken)
{
	uint32_t r = 99;
	uint32_t zero = 0;

	__asm__ volatile(
		"cbz  %[z], 1f\n\t"
		"movs %[r], #0\n\t"   /* not taken */
		"b    2f\n\t"
		"1:\n\t"
		"movs %[r], #1\n\t"   /* taken */
		"2:\n\t"
		: [r] "+r"(r)
		: [z] "r"(zero)
		: "cc");
	zassert_equal(r, 1, "CBZ taken wrong");
}

ZTEST(armv7m_branch_it, test_cbz_not_taken)
{
	uint32_t r = 99;
	uint32_t nonzero = 5;

	__asm__ volatile(
		"cbz  %[nz], 1f\n\t"
		"movs %[r], #0\n\t"   /* taken (not branch) */
		"b    2f\n\t"
		"1:\n\t"
		"movs %[r], #1\n\t"
		"2:\n\t"
		: [r] "+r"(r)
		: [nz] "r"(nonzero)
		: "cc");
	zassert_equal(r, 0, "CBZ not-taken wrong: %d", r);
}

/* CBNZ: branch if nonzero */
ZTEST(armv7m_branch_it, test_cbnz_taken)
{
	uint32_t r = 0;
	uint32_t nonzero = 42;

	__asm__ volatile(
		"cbnz %[nz], 1f\n\t"
		"b    2f\n\t"
		"1:\n\t"
		"movs %[r], #1\n\t"
		"2:\n\t"
		: [r] "+r"(r)
		: [nz] "r"(nonzero)
		: "cc");
	zassert_equal(r, 1, "CBNZ taken wrong");
}

ZTEST(armv7m_branch_it, test_cbnz_not_taken)
{
	uint32_t r = 0;
	uint32_t zero_val = 0;

	__asm__ volatile(
		"cbnz %[z], 1f\n\t"
		"movs %[r], #2\n\t"   /* should execute */
		"b    2f\n\t"
		"1:\n\t"
		"movs %[r], #1\n\t"
		"2:\n\t"
		: [r] "+r"(r)
		: [z] "r"(zero_val)
		: "cc");
	zassert_equal(r, 2, "CBNZ not-taken wrong");
}

/* Conditional B: all conditions sampled */
ZTEST(armv7m_branch_it, test_bne_loop)
{
	uint32_t i;

	__asm__ volatile(
		"movs %[i], #10\n\t"
		"1:\n\t"
		"subs %[i], %[i], #1\n\t"
		"bne  1b\n\t"
		: [i] "+r"(i)
		:
		: "cc");
	zassert_equal(i, 0, "BNE loop wrong: %d", i);
}

ZTEST(armv7m_branch_it, test_beq_branch)
{
	uint32_t r = 0;

	__asm__ volatile(
		"cmp  %[a], %[b]\n\t"
		"beq  1f\n\t"
		"movs %[r], #99\n\t"
		"b    2f\n\t"
		"1:\n\t"
		"movs %[r], #7\n\t"
		"2:\n\t"
		: [r] "+r"(r)
		: [a] "r"(5U), [b] "r"(5U)
		: "cc");
	zassert_equal(r, 7, "BEQ wrong");
}

/* TBB: table branch byte */
ZTEST(armv7m_branch_it, test_tbb)
{
	uint32_t idx = 2, result = 0;

	__asm__ volatile(
		"adr  r1, 1f\n\t"
		"tbb  [r1, %[i]]\n\t"
		/* jump table: 3 entries, each is (offset/2) from TBB+4 */
		"1:\n\t"
		".byte 4\n\t"  /* idx=0: jump over 2 instrs */
		".byte 2\n\t"  /* idx=1: jump over 1 instr */
		".byte 0\n\t"  /* idx=2: jump right to next instr */
		".align 2\n\t"
		"nop\n\t"
		"nop\n\t"
		"movs %[r], #3\n\t"   /* idx=2 lands here */
		"b    2f\n\t"
		"movs %[r], #2\n\t"   /* idx=1 */
		"b    2f\n\t"
		"movs %[r], #1\n\t"   /* idx=0 */
		"2:\n\t"
		: [r] "+r"(result)
		: [i] "r"(idx)
		: "r1", "cc");
	zassert_equal(result, 3, "TBB idx=2 wrong: %d", result);
}
