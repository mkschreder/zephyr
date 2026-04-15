/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M load/store: LDR / STR (word).
 *
 * Reference: DDI0403E
 *   LDR (immediate) A7.7.43 lines 8145-8250
 *   LDR (literal)   A7.7.44 lines 8251-8340
 *   LDR (register)  A7.7.45 lines 8341-8412
 *   STR equivalents A7.7.161-163
 *   Addressing modes A4.6.5 lines 3566-3593, A7.5 lines 5393-5431
 */

#include <zephyr/ztest.h>
#include <armv7m_test.h>
#include <stdint.h>

static uint32_t mem_word[8];

ZTEST(armv7m_load_store, test_str_ldr_roundtrip)
{
	uint32_t stored = 0xDEADBEEFU, loaded;
	uint32_t *addr = &mem_word[0];

	__asm__ volatile(
		"str %[s], [%[a]]\n\t"
		"ldr %[l], [%[a]]\n\t"
		: [l] "=r"(loaded)
		: [s] "r"(stored), [a] "r"(addr)
		: "memory");
	zassert_equal(loaded, stored, "LDR/STR roundtrip wrong");
}

ZTEST(armv7m_load_store, test_str_ldr_imm_offset)
{
	uint32_t val = 0x12345678U, loaded;
	uint32_t *base = mem_word;

	mem_word[2] = 0;
	__asm__ volatile(
		"str %[v], [%[b], #8]\n\t"   /* offset = 2 words = 8 bytes */
		"ldr %[l], [%[b], #8]\n\t"
		: [l] "=r"(loaded)
		: [v] "r"(val), [b] "r"(base)
		: "memory");
	zassert_equal(loaded, val, "STR/LDR imm offset wrong");
}

ZTEST(armv7m_load_store, test_str_ldr_negative_offset)
{
	uint32_t val = 0xABCDEF01U, loaded;
	uint32_t *base = &mem_word[4];

	__asm__ volatile(
		"str %[v], [%[b], #-8]\n\t"  /* base[-2] */
		"ldr %[l], [%[b], #-8]\n\t"
		: [l] "=r"(loaded)
		: [v] "r"(val), [b] "r"(base)
		: "memory");
	zassert_equal(loaded, val, "STR/LDR negative offset wrong");
}

/* Pre-indexed: STR Rt, [Rn, #imm]! -- writes to Rn+imm, updates Rn */
ZTEST(armv7m_load_store, test_str_pre_indexed_writeback)
{
	uint32_t val = 0x99999999U, loaded;
	uint32_t *ptr = mem_word;
	uint32_t *result_ptr;

	__asm__ volatile(
		"str  %[v], [%[p], #4]!\n\t"
		"mov  %[rp], %[p]\n\t"
		: [p] "+r"(ptr), [rp] "=r"(result_ptr)
		: [v] "r"(val)
		: "memory");
	/* ptr should now point to mem_word[1] */
	zassert_equal(result_ptr, &mem_word[1], "pre-index writeback wrong");
	loaded = mem_word[1];
	zassert_equal(loaded, val, "pre-index store wrong");
}

/* Post-indexed: LDR Rt, [Rn], #imm -- loads from Rn, then Rn += imm */
ZTEST(armv7m_load_store, test_ldr_post_indexed_writeback)
{
	mem_word[0] = 0x77777777U;
	uint32_t loaded;
	uint32_t *ptr = mem_word;

	__asm__ volatile(
		"ldr %[l], [%[p]], #4\n\t"
		: [l] "=r"(loaded), [p] "+r"(ptr)
		:
		: "memory");
	zassert_equal(loaded, 0x77777777U, "post-index load wrong");
	zassert_equal(ptr, &mem_word[1], "post-index writeback wrong");
}

/* Register offset: LDR Rt, [Rn, Rm, LSL #shift] */
ZTEST(armv7m_load_store, test_ldr_reg_offset_shift)
{
	uint32_t base_val = 0xFEEDFACEU;
	uint32_t loaded;
	uint32_t *base = mem_word;
	uint32_t idx = 3;

	mem_word[3] = base_val;
	__asm__ volatile(
		"ldr %[l], [%[b], %[i], lsl #2]\n\t"
		: [l] "=r"(loaded)
		: [b] "r"(base), [i] "r"(idx)
		: "memory");
	zassert_equal(loaded, base_val, "LDR reg+shift wrong");
}

/* PC-relative literal load */
ZTEST(armv7m_load_store, test_ldr_literal)
{
	uint32_t r;

	__asm__ volatile(
		"ldr %[r], =0xCAFEBABE\n\t"
		: [r] "=r"(r)
		:
		: );
	zassert_equal(r, 0xCAFEBABEU, "LDR literal wrong: 0x%08x", r);
}

/* Multiple STR then LDR to verify memory independence */
ZTEST(armv7m_load_store, test_str_multiple_independent)
{
	uint32_t a = 0x11111111U, b = 0x22222222U, c = 0x33333333U;
	uint32_t la, lb, lc;
	uint32_t *p = mem_word;

	__asm__ volatile(
		"str %[a], [%[p]]\n\t"
		"str %[b], [%[p], #4]\n\t"
		"str %[c], [%[p], #8]\n\t"
		"ldr %[la], [%[p]]\n\t"
		"ldr %[lb], [%[p], #4]\n\t"
		"ldr %[lc], [%[p], #8]\n\t"
		: [la] "=r"(la), [lb] "=r"(lb), [lc] "=r"(lc)
		: [a] "r"(a), [b] "r"(b), [c] "r"(c), [p] "r"(p)
		: "memory");
	zassert_equal(la, a, "STR[0] wrong");
	zassert_equal(lb, b, "STR[1] wrong");
	zassert_equal(lc, c, "STR[2] wrong");
}
