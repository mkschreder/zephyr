/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M load/store: LDRD / STRD (dual-register).
 *
 * Reference: DDI0403E
 *   LDRD (immediate) A7.7.50 lines 8625-8702
 *   LDRD (literal)   A7.7.51 lines 8703-8770
 *   STRD             A7.7.166 lines 15667-15742
 */

#include <zephyr/ztest.h>
#include <armv7m_test.h>
#include <stdint.h>
#include <string.h>

static uint32_t mem_dual[8];

ZTEST(armv7m_load_store, test_strd_ldrd_roundtrip)
{
	uint32_t hi = 0xDEADBEEFU, lo = 0xCAFEBABEU;
	uint32_t lhi, llo;
	uint32_t *p = mem_dual;

	__asm__ volatile(
		"strd %[lo], %[hi], [%[p]]\n\t"
		"ldrd %[llo], %[lhi], [%[p]]\n\t"
		: [llo] "=r"(llo), [lhi] "=r"(lhi)
		: [lo] "r"(lo), [hi] "r"(hi), [p] "r"(p)
		: "memory");
	zassert_equal(llo, lo, "STRD/LDRD lo wrong");
	zassert_equal(lhi, hi, "STRD/LDRD hi wrong");
}

ZTEST(armv7m_load_store, test_strd_ldrd_offset)
{
	uint32_t a = 0x11223344U, b = 0x55667788U;
	uint32_t la, lb;
	uint32_t *p = mem_dual;

	memset(mem_dual, 0, sizeof(mem_dual));
	__asm__ volatile(
		"strd %[a], %[b], [%[p], #8]\n\t"  /* store at p+8 */
		"ldrd %[la], %[lb], [%[p], #8]\n\t"
		: [la] "=r"(la), [lb] "=r"(lb)
		: [a] "r"(a), [b] "r"(b), [p] "r"(p)
		: "memory");
	zassert_equal(la, a, "STRD/LDRD offset a wrong");
	zassert_equal(lb, b, "STRD/LDRD offset b wrong");
	/* Verify adjacent slots not corrupted */
	zassert_equal(mem_dual[0], 0, "STRD corrupted [0]");
	zassert_equal(mem_dual[1], 0, "STRD corrupted [1]");
}

ZTEST(armv7m_load_store, test_strd_pre_indexed)
{
	uint32_t a = 0xAAAAAAAAU, b = 0xBBBBBBBBU;
	uint32_t la, lb;
	uint32_t *p = mem_dual;

	__asm__ volatile(
		"strd %[a], %[b], [%[p], #4]!\n\t"
		"ldrd %[la], %[lb], [%[p]]\n\t"
		: [la] "=r"(la), [lb] "=r"(lb), [p] "+r"(p)
		: [a] "r"(a), [b] "r"(b)
		: "memory");
	zassert_equal(la, a, "STRD pre-index a wrong");
	zassert_equal(lb, b, "STRD pre-index b wrong");
	zassert_equal(p, &mem_dual[1], "STRD pre-index ptr wrong");
}

ZTEST(armv7m_load_store, test_ldrd_post_indexed)
{
	mem_dual[0] = 0x12345678U;
	mem_dual[1] = 0x9ABCDEF0U;
	uint32_t lo, hi;
	uint32_t *p = mem_dual;

	__asm__ volatile(
		"ldrd %[lo], %[hi], [%[p]], #8\n\t"
		: [lo] "=r"(lo), [hi] "=r"(hi), [p] "+r"(p)
		:
		: "memory");
	zassert_equal(lo, 0x12345678U, "LDRD post lo wrong");
	zassert_equal(hi, 0x9ABCDEF0U, "LDRD post hi wrong");
	zassert_equal(p, &mem_dual[2], "LDRD post ptr wrong");
}
