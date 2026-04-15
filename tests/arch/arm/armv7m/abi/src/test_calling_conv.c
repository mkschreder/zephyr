/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M ABI: AAPCS calling convention.
 *
 * AAPCS: first 4 integer args in r0-r3, remainder on stack.
 * Return value in r0 (32-bit) or r0:r1 (64-bit).
 * Callee-saved: r4-r11, SP.
 * Caller-saved: r0-r3, r12, LR.
 */

#include <zephyr/ztest.h>
#include <armv7m_test.h>
#include <stdarg.h>

/* Prevent inlining so the call convention is actually exercised */
__attribute__((noinline))
static uint32_t sum4(uint32_t a, uint32_t b, uint32_t c, uint32_t d)
{
	return a + b + c + d;
}

__attribute__((noinline))
static uint32_t sum5(uint32_t a, uint32_t b, uint32_t c, uint32_t d, uint32_t e)
{
	return a + b + c + d + e;
}

__attribute__((noinline))
static uint32_t sum8(uint32_t a, uint32_t b, uint32_t c, uint32_t d,
		     uint32_t e, uint32_t f, uint32_t g, uint32_t h)
{
	return a + b + c + d + e + f + g + h;
}

ZTEST(armv7m_abi, test_args_in_regs_r0_r3)
{
	uint32_t r = sum4(1, 2, 3, 4);

	zassert_equal(r, 10, "4-arg sum wrong: %d", r);
}

ZTEST(armv7m_abi, test_arg5_on_stack)
{
	uint32_t r = sum5(1, 2, 3, 4, 5);

	zassert_equal(r, 15, "5-arg sum wrong: %d", r);
}

ZTEST(armv7m_abi, test_arg8_on_stack)
{
	uint32_t r = sum8(1, 2, 3, 4, 5, 6, 7, 8);

	zassert_equal(r, 36, "8-arg sum wrong: %d", r);
}

/* Return value: 64-bit in r0:r1 */
__attribute__((noinline))
static uint64_t mul64(uint32_t a, uint32_t b)
{
	return (uint64_t)a * b;
}

ZTEST(armv7m_abi, test_return_64bit)
{
	uint64_t r = mul64(0x10000U, 0x10000U);

	zassert_equal(r, 0x100000000ULL, "64-bit return wrong: %llu", r);
}

/* Callee-saved registers r4-r7 must survive a function call.
 *
 * AAPCS: r4-r11 are callee-saved.  The called function (sum4) must preserve
 * them.  We use a dedicated assembly helper (test_callee_saved.S) that loads
 * known sentinels into r4-r7, calls fn(), then stores post-call values to
 * out-pointers.  This avoids GCC inline-asm register allocation surprises.
 */
void test_check_callee_saved_asm(uint32_t *r4_out, uint32_t *r5_out,
				 uint32_t *r6_out, uint32_t *r7_out,
				 void (*fn)(void));

static void callee_target(void)
{
	(void)sum4(1, 2, 3, 4);
}

ZTEST(armv7m_abi, test_callee_saved_preserved)
{
	uint32_t r4_after, r5_after, r6_after, r7_after;

	test_check_callee_saved_asm(&r4_after, &r5_after, &r6_after, &r7_after,
				    callee_target);

	zassert_equal(r4_after, 0x0000AA01U,
		      "r4 not callee-saved: 0x%08x", r4_after);
	zassert_equal(r5_after, 0x0000BB02U,
		      "r5 not callee-saved: 0x%08x", r5_after);
	zassert_equal(r6_after, 0x0000CC03U,
		      "r6 not callee-saved: 0x%08x", r6_after);
	zassert_equal(r7_after, 0x0000DD04U,
		      "r7 not callee-saved: 0x%08x", r7_after);
}

/* Helper for SP alignment test (must be file-scope, not nested) */
__attribute__((noinline))
static uint32_t get_sp_at_call(void)
{
	uint32_t sp;

	__asm__ volatile("mov %0, sp\n\t" : "=r"(sp) : : );
	return sp;
}

/* Stack alignment: SP must be 8-byte aligned at call site */
ZTEST(armv7m_abi, test_sp_alignment_at_call)
{
	uint32_t sp_at_call = get_sp_at_call();

	zassert_equal(sp_at_call % 8, 0,
		      "SP not 8-byte aligned at call: 0x%08x", sp_at_call);
}

/* Struct argument passing: small struct in register(s) */
struct small_struct {
	uint16_t x;
	uint16_t y;
};

__attribute__((noinline))
static uint32_t add_small_struct(struct small_struct s)
{
	return s.x + s.y;
}

ZTEST(armv7m_abi, test_small_struct_in_reg)
{
	struct small_struct s = {.x = 100, .y = 200};
	uint32_t r = add_small_struct(s);

	zassert_equal(r, 300, "small struct arg wrong");
}

/* Large struct: passed via hidden pointer */
struct large_struct {
	uint32_t a, b, c, d, e;
};

__attribute__((noinline))
static uint32_t sum_large(struct large_struct s)
{
	return s.a + s.b + s.c + s.d + s.e;
}

ZTEST(armv7m_abi, test_large_struct_pass)
{
	struct large_struct s = {1, 2, 3, 4, 5};
	uint32_t r = sum_large(s);

	zassert_equal(r, 15, "large struct arg wrong");
}

/* Struct return: large struct via hidden output pointer */
__attribute__((noinline))
static struct large_struct make_large(uint32_t base)
{
	return (struct large_struct){base, base+1, base+2, base+3, base+4};
}

ZTEST(armv7m_abi, test_struct_return)
{
	struct large_struct s = make_large(10);

	zassert_equal(s.a, 10, "struct return a wrong");
	zassert_equal(s.b, 11, "struct return b wrong");
	zassert_equal(s.c, 12, "struct return c wrong");
	zassert_equal(s.d, 13, "struct return d wrong");
	zassert_equal(s.e, 14, "struct return e wrong");
}
