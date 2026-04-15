/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M load/store: unaligned access behavior.
 *
 * ARMv7-M supports unaligned word and halfword loads/stores unless
 * CCR.UNALIGN_TRP is set (which triggers UsageFault).
 *
 * By default (CCR.UNALIGN_TRP=0) these succeed transparently.
 */

#include <zephyr/ztest.h>
#include <armv7m_test.h>
#include <stdint.h>
#include <string.h>

/* We use a byte buffer to build an unaligned access target */
static uint8_t buf[16] __attribute__((aligned(8)));

ZTEST(armv7m_load_store, test_ldr_unaligned_1)
{
	uint32_t expected = 0x04030201U;
	uint32_t loaded;

	buf[0] = 0; buf[1] = 1; buf[2] = 2; buf[3] = 3; buf[4] = 4;
	/* Read word from buf+1 (unaligned) */
	__asm__ volatile(
		"ldr %[l], [%[a], #1]\n\t"
		: [l] "=r"(loaded)
		: [a] "r"(buf)
		: "memory");
	zassert_equal(loaded, expected, "LDR unaligned+1 wrong: 0x%08x", loaded);
}

ZTEST(armv7m_load_store, test_ldr_unaligned_2)
{
	uint32_t expected = 0x05040302U;
	uint32_t loaded;

	buf[0] = 0; buf[1] = 1; buf[2] = 2; buf[3] = 3;
	buf[4] = 4; buf[5] = 5;
	__asm__ volatile(
		"ldr %[l], [%[a], #2]\n\t"
		: [l] "=r"(loaded)
		: [a] "r"(buf)
		: "memory");
	zassert_equal(loaded, expected, "LDR unaligned+2 wrong: 0x%08x", loaded);
}

ZTEST(armv7m_load_store, test_ldr_unaligned_3)
{
	uint32_t expected = 0x06050403U;
	uint32_t loaded;

	buf[0] = 0; buf[1] = 1; buf[2] = 2; buf[3] = 3;
	buf[4] = 4; buf[5] = 5; buf[6] = 6;
	__asm__ volatile(
		"ldr %[l], [%[a], #3]\n\t"
		: [l] "=r"(loaded)
		: [a] "r"(buf)
		: "memory");
	zassert_equal(loaded, expected, "LDR unaligned+3 wrong: 0x%08x", loaded);
}

ZTEST(armv7m_load_store, test_ldrh_unaligned_1)
{
	uint32_t expected = 0x0201U;
	uint32_t loaded;

	buf[0] = 0; buf[1] = 1; buf[2] = 2;
	__asm__ volatile(
		"ldrh %[l], [%[a], #1]\n\t"
		: [l] "=r"(loaded)
		: [a] "r"(buf)
		: "memory");
	zassert_equal(loaded, expected, "LDRH unaligned+1 wrong: 0x%04x", loaded);
}

ZTEST(armv7m_load_store, test_str_unaligned_roundtrip)
{
	uint32_t val = 0xDEADBEEFU;
	uint32_t loaded;

	memset(buf, 0, sizeof(buf));
	/* Store to unaligned addr +1 */
	__asm__ volatile(
		"str  %[v], [%[a], #1]\n\t"
		"ldr  %[l], [%[a], #1]\n\t"
		: [l] "=r"(loaded)
		: [v] "r"(val), [a] "r"(buf)
		: "memory");
	zassert_equal(loaded, val, "STR/LDR unaligned roundtrip wrong");
}
