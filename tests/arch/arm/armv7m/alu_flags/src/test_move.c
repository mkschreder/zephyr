/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M ALU: MOV / MOVT / ADR / MOVW.
 *
 * Reference: DDI0403E
 *   MOV (imm)     A7.7.76  lines 10358-10443
 *   MOV (reg)     A7.7.77  lines 10444-10544
 *   MOV (shifted) A7.7.78  lines 10545-10584
 *   MOVT          A7.7.79  lines 10585-10630
 *   ADR           A7.7.7   lines 6003-6110
 */

#include <zephyr/ztest.h>
#include <armv7m_test.h>

ZTEST(armv7m_alu_flags, test_mov_imm8)
{
	uint32_t r;

	__asm__ volatile("movs %[r], #0xAB\n\t" : [r] "=r"(r) : : "cc");
	zassert_equal(r, 0xABU, "MOV imm8 wrong: 0x%x", r);
}

ZTEST(armv7m_alu_flags, test_mov_imm16)
{
	uint32_t r;

	__asm__ volatile("movw %[r], #0x1234\n\t" : [r] "=r"(r) : : );
	zassert_equal(r, 0x1234U, "MOVW imm16 wrong");
}

ZTEST(armv7m_alu_flags, test_mov_imm_sets_n)
{
	uint32_t r, apsr;

	/* modified immediate: 0x80000000 */
	__asm__ volatile(
		"mov  %[r], #0x80000000\n\t"
		"movs %[r], %[r]\n\t"       /* MOVS to actually set flags */
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		:
		: "cc");
	zassert_equal(r, 0x80000000U, "MOV 0x80000000 wrong");
	zassert_true(armv7m_flag_n(apsr), "N not set");
}

ZTEST(armv7m_alu_flags, test_movt_upper_halfword)
{
	uint32_t r;

	__asm__ volatile(
		"movw %[r], #0x5678\n\t"
		"movt %[r], #0x1234\n\t"
		: [r] "=&r"(r)
		:
		: );
	zassert_equal(r, 0x12345678U, "MOVT upper halfword wrong: 0x%08x", r);
}

ZTEST(armv7m_alu_flags, test_movt_preserves_lower)
{
	uint32_t r;

	__asm__ volatile(
		"movw %[r], #0xBEEF\n\t"
		"movt %[r], #0xDEAD\n\t"
		: [r] "=&r"(r)
		:
		: );
	zassert_equal(r, 0xDEADBEEFU, "MOVT preserve lower wrong");
}

ZTEST(armv7m_alu_flags, test_movt_zero_upper)
{
	uint32_t r;

	__asm__ volatile(
		"movw %[r], #0xFFFF\n\t"
		"movt %[r], #0\n\t"
		: [r] "=&r"(r)
		:
		: );
	zassert_equal(r, 0x0000FFFFU, "MOVT zero upper wrong");
}

ZTEST(armv7m_alu_flags, test_mov_reg_copy)
{
	uint32_t src = 0xCAFEBABEU, dst;

	__asm__ volatile(
		"mov %[d], %[s]\n\t"
		: [d] "=r"(dst)
		: [s] "r"(src)
		: );
	zassert_equal(dst, src, "MOV reg copy wrong");
}

ZTEST(armv7m_alu_flags, test_movs_zero_sets_z)
{
	uint32_t r, apsr;

	__asm__ volatile(
		"movs %[r], #0\n\t"
		"mrs  %[f], apsr\n\t"
		: [r] "=&r"(r), [f] "=r"(apsr)
		:
		: "cc");
	zassert_equal(r, 0, "MOVS 0 wrong");
	zassert_true(armv7m_flag_z(apsr), "Z not set");
	zassert_false(armv7m_flag_n(apsr), "N spuriously set");
}

/* MOV with shifted register operand */
ZTEST(armv7m_alu_flags, test_mov_shifted_reg)
{
	uint32_t r;
	uint32_t src = 3U;

	__asm__ volatile(
		"mov %[r], %[s], lsl #2\n\t"
		: [r] "=r"(r)
		: [s] "r"(src)
		: );
	zassert_equal(r, 12, "MOV shifted reg wrong");
}

/* ADR: loads PC-relative address */
ZTEST(armv7m_alu_flags, test_adr_forward)
{
	uint32_t addr1, addr2;

	__asm__ volatile(
		"adr %[a], 1f\n\t"
		"nop\n\t"
		"nop\n\t"
		"1:\n\t"
		"adr %[b], 1b\n\t"
		: [a] "=r"(addr1), [b] "=r"(addr2)
		:
		: );
	/* addr1 should equal addr2 since they both point to label 1 */
	zassert_equal(addr1, addr2, "ADR forward/back mismatch: %x %x",
		      addr1, addr2);
}
