/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M ALU: Bitfield and miscellaneous data-processing.
 *
 * Reference: DDI0403E
 *   BFC   A7.7.13  lines 6485-6534
 *   BFI   A7.7.14  lines 6535-6586
 *   SBFX  A7.7.124
 *   UBFX  A7.7.196
 *   CLZ   A7.7.24  lines 7065-7114
 *   RBIT  A7.7.112 lines 12485-12532
 *   REV   A7.7.113
 *   REV16 A7.7.114
 *   REVSH A7.7.115
 */

#include <zephyr/ztest.h>
#include <armv7m_test.h>

/* BFC: bit field clear */
ZTEST(armv7m_alu_flags, test_bfc_middle_bits)
{
	uint32_t r = 0xFFFFFFFFU;

	__asm__ volatile("bfc %[r], #8, #8\n\t" : [r] "+r"(r) : : );
	/* Clear bits [15:8]: result = 0xFFFF00FF */
	zassert_equal(r, 0xFFFF00FFU, "BFC [15:8] wrong: 0x%08x", r);
}

ZTEST(armv7m_alu_flags, test_bfc_all_bits)
{
	uint32_t r = 0xFFFFFFFFU;

	__asm__ volatile("bfc %[r], #0, #32\n\t" : [r] "+r"(r) : : );
	zassert_equal(r, 0, "BFC all 32 bits wrong");
}

ZTEST(armv7m_alu_flags, test_bfc_single_bit)
{
	uint32_t r = 0xFFFFFFFFU;

	__asm__ volatile("bfc %[r], #15, #1\n\t" : [r] "+r"(r) : : );
	zassert_equal(r, 0xFFFF7FFFU, "BFC single bit[15] wrong: 0x%08x", r);
}

/* BFI: bit field insert */
ZTEST(armv7m_alu_flags, test_bfi_insert)
{
	uint32_t dst = 0, src = 0xABU;

	__asm__ volatile("bfi %[d], %[s], #8, #8\n\t"
			 : [d] "+r"(dst)
			 : [s] "r"(src)
			 : );
	/* Insert bits[7:0] of src into dst[15:8] */
	zassert_equal(dst, 0xAB00U, "BFI insert wrong: 0x%08x", dst);
}

ZTEST(armv7m_alu_flags, test_bfi_preserve_surrounding)
{
	uint32_t dst = 0xFFFFFFFFU, src = 0x55U;

	__asm__ volatile("bfi %[d], %[s], #4, #8\n\t"
			 : [d] "+r"(dst)
			 : [s] "r"(src)
			 : );
	/* Bits [11:4] become 0x55; surrounding bits stay 1 */
	zassert_equal(dst, 0xFFFFF55FU, "BFI preserve wrong: 0x%08x", dst);
}

/* SBFX: signed bit field extract */
ZTEST(armv7m_alu_flags, test_sbfx_positive)
{
	uint32_t r;
	uint32_t src = 0x00000040U; /* bit 6 set */

	__asm__ volatile("sbfx %[r], %[s], #5, #4\n\t"
			 : [r] "=r"(r)
			 : [s] "r"(src)
			 : );
	/* Extract bits [8:5]: 0b0010 = 2, sign bit = 0 => +2 */
	zassert_equal((int32_t)r, 2, "SBFX positive wrong: %d", (int32_t)r);
}

ZTEST(armv7m_alu_flags, test_sbfx_negative)
{
	uint32_t r;
	uint32_t src = 0x00000080U; /* bit 7 set */

	__asm__ volatile("sbfx %[r], %[s], #5, #4\n\t"
			 : [r] "=r"(r)
			 : [s] "r"(src)
			 : );
	/* Extract bits [8:5]: 0b0100 = 4, but width=4 sign bit (bit3) is 0 => +4 */
	/* Actually: bits[8:5] of 0x80 = (0x80>>5)&0xF = 4, sign bit = 0 */
	zassert_equal((int32_t)r, 4, "SBFX bits [8:5] of 0x80 wrong: %d",
		      (int32_t)r);
}

ZTEST(armv7m_alu_flags, test_sbfx_sign_extension)
{
	uint32_t r;
	uint32_t src = 0x00000010U; /* bit 4 set */

	/* Extract bits [4:1] (lsb=1, width=4): value = 0b1000 = 8 */
	/* Sign bit (bit3 of extracted) = 1 => sign extend to -8 */
	__asm__ volatile("sbfx %[r], %[s], #1, #4\n\t"
			 : [r] "=r"(r)
			 : [s] "r"(src)
			 : );
	zassert_equal((int32_t)r, -8, "SBFX sign-ext wrong: %d", (int32_t)r);
}

/* UBFX: unsigned bit field extract */
ZTEST(armv7m_alu_flags, test_ubfx_extract)
{
	uint32_t r;
	uint32_t src = 0x12345678U;

	__asm__ volatile("ubfx %[r], %[s], #8, #8\n\t"
			 : [r] "=r"(r)
			 : [s] "r"(src)
			 : );
	/* Extract bits[15:8]: 0x56 */
	zassert_equal(r, 0x56U, "UBFX bits[15:8] wrong: 0x%02x", r);
}

ZTEST(armv7m_alu_flags, test_ubfx_no_sign_extend)
{
	uint32_t r;
	/* src = 0x0FF0: bits[11:4] = 0b1111_1111 = 0xFF
	 * UBFX #4, #8: extract bits[4+8-1:4] = bits[11:4]
	 * Result = (0x0FF0 >> 4) & 0xFF = 0xFF, no sign extension */
	__asm__ volatile("ubfx %[r], %[s], #4, #8\n\t"
			 : [r] "=r"(r)
			 : [s] "r"(0x0FF0U)
			 : );
	zassert_equal(r, 0xFFU, "UBFX no-sign wrong: 0x%x", r);
}

/* CLZ: count leading zeros */
ZTEST(armv7m_alu_flags, test_clz_zero)
{
	uint32_t r;

	__asm__ volatile("clz %[r], %[v]\n\t"
			 : [r] "=r"(r)
			 : [v] "r"(0U)
			 : );
	zassert_equal(r, 32, "CLZ 0 wrong");
}

ZTEST(armv7m_alu_flags, test_clz_all_ones)
{
	uint32_t r;

	__asm__ volatile("clz %[r], %[v]\n\t"
			 : [r] "=r"(r)
			 : [v] "r"(0xFFFFFFFFU)
			 : );
	zassert_equal(r, 0, "CLZ 0xFFFFFFFF wrong");
}

ZTEST(armv7m_alu_flags, test_clz_bit31)
{
	uint32_t r;

	__asm__ volatile("clz %[r], %[v]\n\t"
			 : [r] "=r"(r)
			 : [v] "r"(0x80000000U)
			 : );
	zassert_equal(r, 0, "CLZ bit31 wrong");
}

ZTEST(armv7m_alu_flags, test_clz_bit0)
{
	uint32_t r;

	__asm__ volatile("clz %[r], %[v]\n\t"
			 : [r] "=r"(r)
			 : [v] "r"(1U)
			 : );
	zassert_equal(r, 31, "CLZ bit0 wrong");
}

ZTEST(armv7m_alu_flags, test_clz_various)
{
	uint32_t r;

	__asm__ volatile("clz %[r], %[v]\n\t"
			 : [r] "=r"(r)
			 : [v] "r"(0x00001000U)
			 : );
	zassert_equal(r, 19, "CLZ 0x1000 wrong");
}

/* RBIT: reverse bits */
ZTEST(armv7m_alu_flags, test_rbit_bit0_to_bit31)
{
	uint32_t r;

	__asm__ volatile("rbit %[r], %[v]\n\t"
			 : [r] "=r"(r)
			 : [v] "r"(1U)
			 : );
	zassert_equal(r, 0x80000000U, "RBIT bit0->bit31 wrong");
}

ZTEST(armv7m_alu_flags, test_rbit_roundtrip)
{
	uint32_t r1, r2;
	uint32_t val = 0x12345678U;

	__asm__ volatile(
		"rbit %[r1], %[v]\n\t"
		"rbit %[r2], %[r1]\n\t"
		: [r1] "=&r"(r1), [r2] "=&r"(r2)
		: [v] "r"(val)
		: );
	zassert_equal(r2, val, "RBIT roundtrip wrong");
}

ZTEST(armv7m_alu_flags, test_rbit_palindrome)
{
	uint32_t r;

	__asm__ volatile("rbit %[r], %[v]\n\t"
			 : [r] "=r"(r)
			 : [v] "r"(0xA5A5A5A5U)
			 : );
	/* 0xA5A5A5A5 reversed bit-by-bit = 0xA5A5A5A5 */
	zassert_equal(r, 0xA5A5A5A5U, "RBIT palindrome wrong: 0x%08x", r);
}

/* REV: byte-reverse word */
ZTEST(armv7m_alu_flags, test_rev_word)
{
	uint32_t r;

	__asm__ volatile("rev %[r], %[v]\n\t"
			 : [r] "=r"(r)
			 : [v] "r"(0x12345678U)
			 : );
	zassert_equal(r, 0x78563412U, "REV word wrong: 0x%08x", r);
}

/* REV16: byte-reverse each halfword */
ZTEST(armv7m_alu_flags, test_rev16_halfwords)
{
	uint32_t r;

	__asm__ volatile("rev16 %[r], %[v]\n\t"
			 : [r] "=r"(r)
			 : [v] "r"(0x12345678U)
			 : );
	/* High half: 0x1234 -> 0x3412; low half: 0x5678 -> 0x7856 */
	zassert_equal(r, 0x34127856U, "REV16 wrong: 0x%08x", r);
}

/* REVSH: byte-reverse signed halfword.
 * Algorithm: result = SignExtend(Rm[7:0] : Rm[15:8], 32)
 * i.e., swap bytes of the bottom halfword, then sign-extend from bit15.
 */
ZTEST(armv7m_alu_flags, test_revsh_sign_extend)
{
	uint32_t r;

	/* Input 0x00000080: Rm[7:0]=0x80, Rm[15:8]=0x00
	 * Reversed halfword: 0x8000, bit15=1 => sign-ext => 0xFFFF8000 */
	__asm__ volatile("revsh %[r], %[v]\n\t"
			 : [r] "=r"(r)
			 : [v] "r"(0x00000080U)
			 : );
	zassert_equal(r, 0xFFFF8000U, "REVSH sign-extend wrong: 0x%08x", r);
}

ZTEST(armv7m_alu_flags, test_revsh_positive)
{
	uint32_t r;

	/* Input 0x00001234: Rm[7:0]=0x34, Rm[15:8]=0x12
	 * Reversed halfword: 0x3412, bit15=0 => no sign ext */
	__asm__ volatile("revsh %[r], %[v]\n\t"
			 : [r] "=r"(r)
			 : [v] "r"(0x00001234U)
			 : );
	zassert_equal(r, 0x00003412U, "REVSH positive wrong: 0x%08x", r);
}
