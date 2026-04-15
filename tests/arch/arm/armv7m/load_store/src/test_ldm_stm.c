/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M load/store: LDM / STM / PUSH / POP.
 *
 * Reference: DDI0403E
 *   LDM/LDMIA A7.7.41 lines 8005-8084
 *   LDMDB     A7.7.42 lines 8085-8144
 *   STM/STMIA A7.7.159 lines 15145-15214
 *   STMDB     A7.7.160 lines 15215-15272
 *   POP       A7.7.99  lines 11738-11833
 *   PUSH      A7.7.101 lines 11879-11958
 *
 * NOTE: STM/LDM inline asm is written using fixed hardware registers
 * (explicit register variables) to avoid undefined behavior from GCC
 * assigning registers in non-sequential order to reglist operands.
 */

#include <zephyr/ztest.h>
#include <armv7m_test.h>
#include <stdint.h>
#include <string.h>

static uint32_t stm_buf[16];

/* STMIA/LDMIA roundtrip using explicit r4-r6 */
ZTEST(armv7m_load_store, test_stmia_ldmia_roundtrip)
{
	const uint32_t va = 0x11111111U, vb = 0x22222222U, vc = 0x33333333U;
	uint32_t la, lb, lc;
	uint32_t *p = stm_buf;

	/* Use explicit registers to guarantee sorted order */
	register uint32_t ra __asm__("r4") = va;
	register uint32_t rb __asm__("r5") = vb;
	register uint32_t rc __asm__("r6") = vc;
	register uint32_t rla __asm__("r7");
	register uint32_t rlb __asm__("r8");
	register uint32_t rlc __asm__("r9");

	__asm__ volatile(
		"stmia %[p], {r4, r5, r6}\n\t"
		"ldmia %[p], {r7, r8, r9}\n\t"
		: "=r"(rla), "=r"(rlb), "=r"(rlc)
		: "r"(ra), "r"(rb), "r"(rc), [p] "r"(p)
		: "memory");
	la = rla; lb = rlb; lc = rlc;

	zassert_equal(la, va, "STMIA/LDMIA a wrong: 0x%08x", la);
	zassert_equal(lb, vb, "STMIA/LDMIA b wrong: 0x%08x", lb);
	zassert_equal(lc, vc, "STMIA/LDMIA c wrong: 0x%08x", lc);
}

/* STMIA with writeback */
ZTEST(armv7m_load_store, test_stmia_writeback)
{
	const uint32_t va = 0xAAAAAAAAU, vb = 0xBBBBBBBBU;
	uint32_t *p = stm_buf;

	register uint32_t ra __asm__("r4") = va;
	register uint32_t rb __asm__("r5") = vb;

	__asm__ volatile(
		"stmia %[p]!, {r4, r5}\n\t"
		: [p] "+r"(p)
		: "r"(ra), "r"(rb)
		: "memory");

	zassert_equal(p, &stm_buf[2], "STMIA writeback ptr wrong: %p", p);
	zassert_equal(stm_buf[0], va, "STMIA[0] wrong: 0x%08x", stm_buf[0]);
	zassert_equal(stm_buf[1], vb, "STMIA[1] wrong: 0x%08x", stm_buf[1]);
}

/* LDMIA with writeback */
ZTEST(armv7m_load_store, test_ldmia_writeback)
{
	stm_buf[0] = 0xDEAD0001U;
	stm_buf[1] = 0xDEAD0002U;
	stm_buf[2] = 0xDEAD0003U;
	uint32_t *p = stm_buf;

	register uint32_t r0 __asm__("r4");
	register uint32_t r1 __asm__("r5");
	register uint32_t r2 __asm__("r6");

	__asm__ volatile(
		"ldmia %[p]!, {r4, r5, r6}\n\t"
		: "=r"(r0), "=r"(r1), "=r"(r2), [p] "+r"(p)
		:
		: "memory");

	zassert_equal(r0, 0xDEAD0001U, "LDMIA r4 wrong: 0x%08x", r0);
	zassert_equal(r1, 0xDEAD0002U, "LDMIA r5 wrong: 0x%08x", r1);
	zassert_equal(r2, 0xDEAD0003U, "LDMIA r6 wrong: 0x%08x", r2);
	zassert_equal(p, &stm_buf[3], "LDMIA writeback ptr wrong");
}

/* STMDB: decrement before */
ZTEST(armv7m_load_store, test_stmdb_ldmia)
{
	const uint32_t va = 0xCCCCCCCCU, vb = 0xDDDDDDDDU;
	/* Point p to stm_buf[4] so STMDB can go back */
	uint32_t *p = &stm_buf[4];

	register uint32_t ra __asm__("r4") = va;
	register uint32_t rb __asm__("r5") = vb;
	register uint32_t rla __asm__("r6");
	register uint32_t rlb __asm__("r7");

	__asm__ volatile(
		"stmdb %[p]!, {r4, r5}\n\t"
		"ldmia %[p],  {r6, r7}\n\t"
		: [p] "+r"(p), "=r"(rla), "=r"(rlb)
		: "r"(ra), "r"(rb)
		: "memory");

	zassert_equal(p, &stm_buf[2], "STMDB ptr wrong");
	zassert_equal(rla, va, "STMDB a wrong: 0x%08x", rla);
	zassert_equal(rlb, vb, "STMDB b wrong: 0x%08x", rlb);
}

/* Single-register STM/LDM */
ZTEST(armv7m_load_store, test_stm_single_reg)
{
	const uint32_t val = 0x5A5A5A5AU;
	uint32_t *p = stm_buf;

	register uint32_t rv __asm__("r4") = val;
	register uint32_t rl __asm__("r5");

	__asm__ volatile(
		"stmia %[p], {r4}\n\t"
		"ldmia %[p], {r5}\n\t"
		: "=r"(rl)
		: "r"(rv), [p] "r"(p)
		: "memory");
	zassert_equal(rl, val, "STM single reg wrong: 0x%08x", rl);
}

/* PUSH / POP test */
ZTEST(armv7m_load_store, test_push_pop_roundtrip)
{
	const uint32_t va = 0x12345678U, vb = 0xABCDEF01U;
	uint32_t la, lb;

	register uint32_t ra __asm__("r4") = va;
	register uint32_t rb __asm__("r5") = vb;
	register uint32_t rla __asm__("r4");
	register uint32_t rlb __asm__("r5");

	__asm__ volatile(
		"push {r4, r5}\n\t"
		"pop  {r4, r5}\n\t"
		: "=r"(rla), "=r"(rlb)
		: "r"(ra), "r"(rb)
		: "memory");
	la = rla; lb = rlb;
	zassert_equal(la, va, "PUSH/POP a wrong: 0x%08x", la);
	zassert_equal(lb, vb, "PUSH/POP b wrong: 0x%08x", lb);
}

/* PUSH LR, POP PC pattern (via BL) */
ZTEST(armv7m_load_store, test_push_lr_pop_preserves_sp)
{
	uint32_t sp_before, sp_after;

	__asm__ volatile(
		"mov %[sb], sp\n\t"
		"push {r4, lr}\n\t"
		"nop\n\t"
		"pop  {r4, lr}\n\t"
		"mov %[sa], sp\n\t"
		: [sb] "=&r"(sp_before), [sa] "=&r"(sp_after)
		:
		: "r4", "memory");
	zassert_equal(sp_before, sp_after, "SP not restored after push/pop");
}
