/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M load/store: byte and halfword operations.
 *
 * Reference: DDI0403E
 *   LDRB  A7.7.46-48 lines 8413-8572
 *   LDRH  A7.7.55-57 lines 8936-9161
 *   LDRSB A7.7.59-61 lines 9212-9421
 *   LDRSH A7.7.63-65 lines 9472-9687
 *   STRB / STRH equivalents
 */

#include <zephyr/ztest.h>
#include <armv7m_test.h>
#include <stdint.h>
#include <string.h>

static uint8_t mem_byte[16];

/* STRB / LDRB: zero-extend */
ZTEST(armv7m_load_store, test_strb_ldrb_roundtrip)
{
	uint32_t val = 0xABU, loaded;
	uint8_t *addr = mem_byte;

	__asm__ volatile(
		"strb %[v], [%[a]]\n\t"
		"ldrb %[l], [%[a]]\n\t"
		: [l] "=r"(loaded)
		: [v] "r"(val), [a] "r"(addr)
		: "memory");
	zassert_equal(loaded, 0xABU, "STRB/LDRB roundtrip wrong");
}

ZTEST(armv7m_load_store, test_ldrb_zero_extend)
{
	uint32_t loaded;

	mem_byte[0] = 0xFFU;
	__asm__ volatile(
		"ldrb %[l], [%[a]]\n\t"
		: [l] "=r"(loaded)
		: [a] "r"(mem_byte)
		: "memory");
	/* LDRB must zero-extend: upper 24 bits are 0 */
	zassert_equal(loaded, 0xFFU, "LDRB zero-extend wrong: 0x%08x", loaded);
}

ZTEST(armv7m_load_store, test_strb_only_writes_byte)
{
	/* Fill with known pattern, then write one byte, check others intact */
	memset(mem_byte, 0x55U, sizeof(mem_byte));
	uint32_t val = 0xAAU;

	__asm__ volatile(
		"strb %[v], [%[a], #4]\n\t"
		: : [v] "r"(val), [a] "r"(mem_byte) : "memory");
	zassert_equal(mem_byte[4], 0xAAU, "STRB byte[4] wrong");
	zassert_equal(mem_byte[3], 0x55U, "STRB corrupted adjacent byte[3]");
	zassert_equal(mem_byte[5], 0x55U, "STRB corrupted adjacent byte[5]");
}

/* STRH / LDRH: zero-extend */
ZTEST(armv7m_load_store, test_strh_ldrh_roundtrip)
{
	uint16_t *addr = (uint16_t *)(mem_byte);
	uint32_t val = 0xBEEFU, loaded;

	__asm__ volatile(
		"strh %[v], [%[a]]\n\t"
		"ldrh %[l], [%[a]]\n\t"
		: [l] "=r"(loaded)
		: [v] "r"(val), [a] "r"(addr)
		: "memory");
	zassert_equal(loaded, 0xBEEFU, "STRH/LDRH roundtrip wrong");
}

ZTEST(armv7m_load_store, test_ldrh_zero_extend)
{
	uint16_t *addr = (uint16_t *)(mem_byte);
	uint32_t loaded;

	*addr = 0x8000U;
	__asm__ volatile(
		"ldrh %[l], [%[a]]\n\t"
		: [l] "=r"(loaded)
		: [a] "r"(addr)
		: "memory");
	/* LDRH zero-extends: bit15 does NOT sign-extend */
	zassert_equal(loaded, 0x8000U, "LDRH zero-extend wrong: 0x%08x", loaded);
}

/* LDRSB: sign-extend */
ZTEST(armv7m_load_store, test_ldrsb_sign_extend_negative)
{
	uint32_t loaded;

	mem_byte[0] = 0x80U; /* bit7 = 1 => negative */
	__asm__ volatile(
		"ldrsb %[l], [%[a]]\n\t"
		: [l] "=r"(loaded)
		: [a] "r"(mem_byte)
		: "memory");
	zassert_equal(loaded, 0xFFFFFF80U, "LDRSB sign-ext negative wrong: 0x%08x",
		      loaded);
}

ZTEST(armv7m_load_store, test_ldrsb_sign_extend_positive)
{
	uint32_t loaded;

	mem_byte[0] = 0x7FU; /* bit7 = 0 => positive */
	__asm__ volatile(
		"ldrsb %[l], [%[a]]\n\t"
		: [l] "=r"(loaded)
		: [a] "r"(mem_byte)
		: "memory");
	zassert_equal(loaded, 0x7FU, "LDRSB sign-ext positive wrong");
}

ZTEST(armv7m_load_store, test_ldrsb_boundary)
{
	uint32_t loaded;

	mem_byte[0] = 0x00U;
	__asm__ volatile("ldrsb %[l], [%[a]]\n\t"
			 : [l] "=r"(loaded) : [a] "r"(mem_byte) : "memory");
	zassert_equal(loaded, 0, "LDRSB 0x00 wrong");

	mem_byte[0] = 0xFFU;
	__asm__ volatile("ldrsb %[l], [%[a]]\n\t"
			 : [l] "=r"(loaded) : [a] "r"(mem_byte) : "memory");
	zassert_equal(loaded, 0xFFFFFFFFU, "LDRSB 0xFF wrong");
}

/* LDRSH: sign-extend */
ZTEST(armv7m_load_store, test_ldrsh_sign_extend_negative)
{
	uint16_t *addr = (uint16_t *)(mem_byte);
	uint32_t loaded;

	*addr = 0x8000U; /* bit15 = 1 */
	__asm__ volatile(
		"ldrsh %[l], [%[a]]\n\t"
		: [l] "=r"(loaded)
		: [a] "r"(addr)
		: "memory");
	zassert_equal(loaded, 0xFFFF8000U, "LDRSH sign-ext negative wrong: 0x%08x",
		      loaded);
}

ZTEST(armv7m_load_store, test_ldrsh_sign_extend_positive)
{
	uint16_t *addr = (uint16_t *)(mem_byte);
	uint32_t loaded;

	*addr = 0x7FFFU;
	__asm__ volatile(
		"ldrsh %[l], [%[a]]\n\t"
		: [l] "=r"(loaded)
		: [a] "r"(addr)
		: "memory");
	zassert_equal(loaded, 0x7FFFU, "LDRSH sign-ext positive wrong");
}

/* STRB with register offset */
ZTEST(armv7m_load_store, test_strb_ldrb_reg_offset)
{
	uint32_t idx = 7, val = 0x42U, loaded;

	__asm__ volatile(
		"strb %[v], [%[a], %[i]]\n\t"
		"ldrb %[l], [%[a], %[i]]\n\t"
		: [l] "=r"(loaded)
		: [v] "r"(val), [a] "r"(mem_byte), [i] "r"(idx)
		: "memory");
	zassert_equal(loaded, val, "STRB/LDRB reg offset wrong");
}

/* STRH with pre-index writeback */
ZTEST(armv7m_load_store, test_strh_pre_index)
{
	uint16_t *ptr = (uint16_t *)(mem_byte);
	uint32_t val = 0x1234U, loaded;

	__asm__ volatile(
		"strh %[v], [%[p], #2]!\n\t"
		"ldrh %[l], [%[p]]\n\t"
		: [l] "=r"(loaded), [p] "+r"(ptr)
		: [v] "r"(val)
		: "memory");
	zassert_equal(loaded, val, "STRH pre-index wrong");
	zassert_equal(ptr, (uint16_t *)(mem_byte + 2), "STRH pre-index ptr wrong");
}
