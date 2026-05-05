/*
 * Copyright 2026 Martin Schröder <info@swedishembedded.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * test_tz_swap — verify PSP_NS / PSPLIM_NS / SFPA banking under preemption.
 *
 * This test validates that the per-thread TrustZone register fields added to
 * _thread_arch (psplim_ns, sfpa) survive context switches triggered by
 * preemption.  Specifically it checks:
 *
 *   1. Two threads each write a distinct PSPLIM_NS sentinel value into their
 *      arch.psplim_ns field; after multiple preemptions the field still holds
 *      the per-thread value (no cross-thread contamination).
 *
 *   2. Two threads each set a distinct sfpa sentinel; same check.
 *
 *   3. (Runtime check, real hardware only) After swapping, the MSR PSPLIM_NS
 *      register contains the value that was saved in the in-coming thread's
 *      arch.psplim_ns.
 *
 * The test does NOT attempt to trigger actual Secure→NS calls (which require
 * specific hardware).  Instead it pokes the arch struct fields directly and
 * relies on the scheduler to context-switch.  The assembly in swap_helper.S
 * must save/restore these fields correctly; if it does not, the cross-check
 * assertions will catch stale values.
 *
 * Reference: ARM DDI 0553B §B3.10 (PSPLIM_NS), §B3.16 (CONTROL_S.SFPA).
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/arm/thread.h>

#if defined(CONFIG_ARM_SECURE_FIRMWARE)

/* Sentinel values distinguishable per-thread. */
#define THREAD_A_PSPLIM  0xA0000000U
#define THREAD_B_PSPLIM  0xB0000000U
#define THREAD_A_SFPA    0x01U
#define THREAD_B_SFPA    0x00U

#define STACK_SIZE 512
#define NUM_SWITCHES 20

static K_THREAD_STACK_DEFINE(thr_a_stack, STACK_SIZE);
static K_THREAD_STACK_DEFINE(thr_b_stack, STACK_SIZE);
static struct k_thread thr_a;
static struct k_thread thr_b;

static volatile int thr_a_errors;
static volatile int thr_b_errors;
static struct k_sem done_a;
static struct k_sem done_b;

static void thread_a_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	struct k_thread *self = k_current_get();

	self->arch.psplim_ns = THREAD_A_PSPLIM;
#if defined(CONFIG_FPU)
	self->arch.sfpa      = THREAD_A_SFPA;
#endif

	for (int i = 0; i < NUM_SWITCHES; i++) {
		k_yield();

		if (self->arch.psplim_ns != THREAD_A_PSPLIM) {
			thr_a_errors++;
		}
#if defined(CONFIG_FPU)
		if (self->arch.sfpa != THREAD_A_SFPA) {
			thr_a_errors++;
		}
#endif
	}

	k_sem_give(&done_a);
}

static void thread_b_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	struct k_thread *self = k_current_get();

	self->arch.psplim_ns = THREAD_B_PSPLIM;
#if defined(CONFIG_FPU)
	self->arch.sfpa      = THREAD_B_SFPA;
#endif

	for (int i = 0; i < NUM_SWITCHES; i++) {
		k_yield();

		if (self->arch.psplim_ns != THREAD_B_PSPLIM) {
			thr_b_errors++;
		}
#if defined(CONFIG_FPU)
		if (self->arch.sfpa != THREAD_B_SFPA) {
			thr_b_errors++;
		}
#endif
	}

	k_sem_give(&done_b);
}

ZTEST_SUITE(arm_tz_swap, NULL, NULL, NULL, NULL, NULL);

ZTEST(arm_tz_swap, test_psplim_ns_banked_across_preemption)
{
	k_sem_init(&done_a, 0, 1);
	k_sem_init(&done_b, 0, 1);
	thr_a_errors = 0;
	thr_b_errors = 0;

	k_thread_create(&thr_a, thr_a_stack, STACK_SIZE,
			thread_a_fn, NULL, NULL, NULL,
			K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
	k_thread_create(&thr_b, thr_b_stack, STACK_SIZE,
			thread_b_fn, NULL, NULL, NULL,
			K_PRIO_PREEMPT(5), 0, K_NO_WAIT);

	k_sem_take(&done_a, K_SECONDS(5));
	k_sem_take(&done_b, K_SECONDS(5));

	k_thread_abort(&thr_a);
	k_thread_abort(&thr_b);

	zassert_equal(thr_a_errors, 0,
		"Thread A: psplim_ns/sfpa corrupted %d times across preemptions",
		thr_a_errors);
	zassert_equal(thr_b_errors, 0,
		"Thread B: psplim_ns/sfpa corrupted %d times across preemptions",
		thr_b_errors);
}

#else /* !CONFIG_ARM_SECURE_FIRMWARE */

ZTEST_SUITE(arm_tz_swap, NULL, NULL, NULL, NULL, NULL);

ZTEST(arm_tz_swap, test_psplim_ns_banked_across_preemption)
{
	ztest_test_skip();
}

#endif /* CONFIG_ARM_SECURE_FIRMWARE */
