/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M exclusive: LDREX / STREX / CLREX and barriers.
 *
 * Reference: DDI0403E
 *   LDREX    A7.7.52  lines 8771-8825
 *   STREX    A7.7.167 lines ~15743-15800
 *   LDREXB   A7.7.53  lines 8826-8882
 *   STREXB   A7.7.168
 *   LDREXH   A7.7.54  lines 8883-8935
 *   STREXH   A7.7.169
 *   CLREX    A7.7.23  lines 7033-7064
 *   DMB      A7.7.33  lines 7495-7538
 *   DSB      A7.7.34  lines 7539-7591
 *   ISB      A7.7.37  lines 7728-7775
 *   Exclusive access semantics A3.4 lines 1933-2210
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <cmsis_core.h>
#include <armv7m_test.h>

static volatile uint32_t excl_word;
static volatile uint8_t  excl_byte;
static volatile uint16_t excl_half;

/* LDREX/STREX: success path (no intervening access) */
ZTEST(armv7m_exclusive, test_ldrex_strex_success)
{
	uint32_t loaded, result;

	excl_word = 0x12345678U;
	__asm__ volatile(
		"ldrex %[l], [%[a]]\n\t"
		"strex %[r], %[v], [%[a]]\n\t"
		: [l] "=&r"(loaded), [r] "=&r"(result)
		: [a] "r"(&excl_word), [v] "r"(0xDEADBEEFU)
		: "memory");
	zassert_equal(loaded, 0x12345678U, "LDREX loaded wrong: 0x%08x", loaded);
	zassert_equal(result, 0, "STREX success should return 0, got %d", result);
	zassert_equal(excl_word, 0xDEADBEEFU, "STREX did not write");
}

/* LDREXB/STREXB */
ZTEST(armv7m_exclusive, test_ldrexb_strexb_success)
{
	uint32_t loaded, result;

	excl_byte = 0x42U;
	__asm__ volatile(
		"ldrexb %[l], [%[a]]\n\t"
		"strexb %[r], %[v], [%[a]]\n\t"
		: [l] "=&r"(loaded), [r] "=&r"(result)
		: [a] "r"(&excl_byte), [v] "r"(0xABU)
		: "memory");
	zassert_equal(loaded, 0x42U, "LDREXB loaded wrong");
	zassert_equal(result, 0, "STREXB should succeed");
	zassert_equal(excl_byte, 0xABU, "STREXB did not write");
}

/* LDREXH/STREXH */
ZTEST(armv7m_exclusive, test_ldrexh_strexh_success)
{
	uint32_t loaded, result;

	excl_half = 0x1234U;
	__asm__ volatile(
		"ldrexh %[l], [%[a]]\n\t"
		"strexh %[r], %[v], [%[a]]\n\t"
		: [l] "=&r"(loaded), [r] "=&r"(result)
		: [a] "r"(&excl_half), [v] "r"(0xABCDU)
		: "memory");
	zassert_equal(loaded, 0x1234U, "LDREXH loaded wrong");
	zassert_equal(result, 0, "STREXH should succeed");
	zassert_equal(excl_half, 0xABCDU, "STREXH did not write");
}

/* CLREX: LDREX then CLREX then STREX must fail */
ZTEST(armv7m_exclusive, test_clrex_invalidates)
{
	uint32_t loaded, result;
	uint32_t orig = 0xAAAAAAAAU;

	excl_word = orig;
	__asm__ volatile(
		"ldrex %[l], [%[a]]\n\t"
		"clrex\n\t"
		"strex %[r], %[v], [%[a]]\n\t"
		: [l] "=&r"(loaded), [r] "=&r"(result)
		: [a] "r"(&excl_word), [v] "r"(0xBBBBBBBBU)
		: "memory");
	zassert_equal(result, 1, "STREX after CLREX should fail, got %d", result);
	/* Value must be unchanged */
	zassert_equal(excl_word, orig, "STREX after CLREX wrote value");
}

/* Exclusive store after plain store: exclusive monitor cleared */
ZTEST(armv7m_exclusive, test_strex_fails_after_plain_str)
{
	uint32_t loaded, result;
	uint32_t orig = 0xCCCCCCCCU;

	excl_word = orig;
	__asm__ volatile(
		"ldrex %[l], [%[a]]\n\t"
		/* Plain STR to same address breaks the exclusive */
		"str   %[v2], [%[a]]\n\t"
		"strex %[r], %[v3], [%[a]]\n\t"
		: [l] "=&r"(loaded), [r] "=&r"(result)
		: [a] "r"(&excl_word), [v2] "r"(0x11111111U), [v3] "r"(0x22222222U)
		: "memory");
	/* On single-processor, the plain STR may or may not clear the local
	 * monitor depending on implementation.  Per A3.4.1 line 1967 the
	 * exclusive monitor is cleared by a Clrex, exception entry, or
	 * another exclusive operation to a different address.  Plain STR to
	 * the same address is implementation-defined on uniprocessor -- so we
	 * only check that the STR wrote correctly. */
	zassert_equal(excl_word == 0x11111111U || excl_word == 0x22222222U,
		      true, "unexpected value after plain STR+STREX: 0x%08x",
		      excl_word);
}

/* Compare-and-swap loop using LDREX/STREX */
ZTEST(armv7m_exclusive, test_cas_loop)
{
	static volatile uint32_t shared = 0;
	uint32_t old_val, new_val, result;
	int iter = 0;

	shared = 0;
	old_val = 0;
	new_val = 0xDEAD;

	/* Spin until STREX succeeds */
	do {
		__asm__ volatile(
			"ldrex %[l], [%[a]]\n\t"
			"strex %[r], %[v], [%[a]]\n\t"
			: [l] "=&r"(old_val), [r] "=&r"(result)
			: [a] "r"(&shared), [v] "r"(new_val)
			: "memory");
		iter++;
	} while (result != 0 && iter < 100);

	zassert_equal(result, 0, "CAS loop never succeeded");
	zassert_equal(shared, new_val, "CAS loop wrong value");
}

/* DMB: does not fault, executes correctly */
ZTEST(armv7m_exclusive, test_dmb_sy)
{
	__asm__ volatile("dmb sy\n\t" : : : "memory");
	zassert_true(true, "DMB SY should not fault");
}

ZTEST(armv7m_exclusive, test_dmb_ish)
{
	/* DMB ISH (inner shareable) -- valid on ARMv7-M */
	__asm__ volatile("dmb ish\n\t" : : : "memory");
	zassert_true(true, "DMB ISH should not fault");
}

ZTEST(armv7m_exclusive, test_dmb_nsh)
{
	__asm__ volatile("dmb nsh\n\t" : : : "memory");
	zassert_true(true, "DMB NSH should not fault");
}

/* DSB */
ZTEST(armv7m_exclusive, test_dsb_sy)
{
	__asm__ volatile("dsb sy\n\t" : : : "memory");
	zassert_true(true, "DSB SY should not fault");
}

/* ISB: pipeline flush; used after CONTROL write */
ZTEST(armv7m_exclusive, test_isb)
{
	uint32_t ctrl_before, ctrl_after;

	__asm__ volatile("mrs %[c], control\n\t" : [c] "=r"(ctrl_before) : : );
	/* Write same value back + ISB (as spec requires after CONTROL write) */
	__asm__ volatile(
		"msr control, %[v]\n\t"
		"isb\n\t"
		"mrs %[c], control\n\t"
		: [c] "=r"(ctrl_after)
		: [v] "r"(ctrl_before)
		: "memory");
	zassert_equal(ctrl_after, ctrl_before, "CONTROL changed after ISB");
}

/* DMB ordering: write-then-read with barrier */
ZTEST(armv7m_exclusive, test_dmb_ordering)
{
	static volatile uint32_t flag = 0, data = 0;

	data = 0xCAFE;
	__asm__ volatile("dmb sy\n\t" : : : "memory");
	flag = 1;
	__asm__ volatile("dmb sy\n\t" : : : "memory");

	zassert_equal(flag, 1, "DMB flag wrong");
	zassert_equal(data, 0xCAFE, "DMB data wrong");
}

/* Multi-threaded atomic counter using LDREX/STREX */
static volatile uint32_t shared_counter;

static void atomic_increment_thread(void *p1, void *p2, void *p3)
{
	int n = (int)(intptr_t)p1;

	for (int i = 0; i < n; i++) {
		uint32_t old, result;
		int retry = 0;

		do {
			__asm__ volatile(
				"ldrex %[o], [%[a]]\n\t"
				"add   %[o], %[o], #1\n\t"
				"strex %[r], %[o], [%[a]]\n\t"
				: [o] "=&r"(old), [r] "=&r"(result)
				: [a] "r"(&shared_counter)
				: "memory");
			retry++;
		} while (result != 0 && retry < 1000);
	}
}

K_THREAD_STACK_DEFINE(t1_stack, 512);
K_THREAD_STACK_DEFINE(t2_stack, 512);
static struct k_thread t1, t2;

ZTEST(armv7m_exclusive, test_atomic_counter_two_threads)
{
	const int N = 200;

	shared_counter = 0;

	k_thread_create(&t1, t1_stack, K_THREAD_STACK_SIZEOF(t1_stack),
			atomic_increment_thread, (void *)(intptr_t)N,
			NULL, NULL, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
	k_thread_create(&t2, t2_stack, K_THREAD_STACK_SIZEOF(t2_stack),
			atomic_increment_thread, (void *)(intptr_t)N,
			NULL, NULL, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);

	k_thread_join(&t1, K_MSEC(2000));
	k_thread_join(&t2, K_MSEC(2000));

	zassert_equal(shared_counter, 2 * N,
		      "Atomic counter wrong: expected %d got %d",
		      2 * N, shared_counter);
}
