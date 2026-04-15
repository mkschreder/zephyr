/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M ABI: varargs and 64-bit arithmetic.
 */

#include <zephyr/ztest.h>
#include <armv7m_test.h>
#include <stdarg.h>
#include <stdint.h>

__attribute__((noinline))
static int sum_n(int n, ...)
{
	va_list ap;
	int total = 0;

	va_start(ap, n);
	for (int i = 0; i < n; i++) {
		total += va_arg(ap, int);
	}
	va_end(ap);
	return total;
}

ZTEST(armv7m_abi, test_varargs_ints)
{
	zassert_equal(sum_n(3, 10, 20, 30), 60, "varargs int sum wrong");
	zassert_equal(sum_n(5, 1, 2, 3, 4, 5), 15, "varargs 5 ints wrong");
}

__attribute__((noinline))
static int64_t sum_ll(int n, ...)
{
	va_list ap;
	int64_t total = 0;

	va_start(ap, n);
	for (int i = 0; i < n; i++) {
		total += va_arg(ap, long long);
	}
	va_end(ap);
	return total;
}

ZTEST(armv7m_abi, test_varargs_longlong)
{
	int64_t r = sum_ll(2, (long long)0x100000000LL, (long long)1LL);

	zassert_equal(r, 0x100000001LL, "varargs long long wrong");
}

/* int64_t arithmetic */
ZTEST(armv7m_abi, test_int64_add)
{
	volatile int64_t a = 0x7FFFFFFFFFFFFFFFLL;
	volatile int64_t b = 1LL;
	int64_t r = a + b;

	/* Wraps to INT64_MIN */
	zassert_equal(r, (int64_t)0x8000000000000000LL, "int64 add overflow wrong");
}

ZTEST(armv7m_abi, test_int64_sub)
{
	volatile int64_t a = (int64_t)0x8000000000000000LL;
	volatile int64_t b = 1LL;
	int64_t r = a - b;

	zassert_equal(r, 0x7FFFFFFFFFFFFFFFLL, "int64 sub wrong");
}

ZTEST(armv7m_abi, test_int64_mul)
{
	volatile int64_t a = 0x100000000LL;
	volatile int64_t b = 0x100000000LL;
	int64_t r = a * b;

	/* 2^32 * 2^32 = 2^64 which wraps to 0 in 64-bit */
	zassert_equal(r, 0LL, "int64 mul overflow wrong");
}

ZTEST(armv7m_abi, test_int64_div)
{
	volatile int64_t a = -1000000LL;
	volatile int64_t b = 3LL;
	int64_t r = a / b;

	zassert_equal(r, -333333LL, "int64 div wrong: %lld", r);
}

ZTEST(armv7m_abi, test_int64_shl)
{
	volatile int64_t a = 1LL;
	int64_t r = a << 40;

	zassert_equal(r, 0x10000000000LL, "int64 shl wrong");
}

ZTEST(armv7m_abi, test_int64_shr_signed)
{
	volatile int64_t a = (int64_t)0x8000000000000000LL;
	int64_t r = a >> 63;

	/* Arithmetic shift: all ones */
	zassert_equal(r, -1LL, "int64 shr signed wrong");
}

ZTEST(armv7m_abi, test_uint64_shr_unsigned)
{
	volatile uint64_t a = 0x8000000000000000ULL;
	uint64_t r = a >> 63;

	zassert_equal(r, 1ULL, "uint64 shr unsigned wrong");
}

/* Mixed type varargs (int + pointer + long long) */
__attribute__((noinline))
static int check_mixed(int count, ...)
{
	va_list ap;
	int ok = 1;

	va_start(ap, count);
	int i = va_arg(ap, int);
	void *p = va_arg(ap, void *);
	long long ll = va_arg(ap, long long);
	va_end(ap);

	if (i != 42) {
		ok = 0;
	}
	if (p != (void *)0x1000) {
		ok = 0;
	}
	if (ll != 0x123456789ABCLL) {
		ok = 0;
	}
	ARG_UNUSED(count);
	return ok;
}

ZTEST(armv7m_abi, test_varargs_mixed_types)
{
	int r = check_mixed(3, (int)42, (void *)0x1000, (long long)0x123456789ABCLL);

	zassert_equal(r, 1, "Mixed varargs wrong");
}
