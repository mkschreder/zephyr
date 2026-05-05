/*
 * Copyright 2026 Martin Schröder <info@swedishembedded.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * trustzone_ns — minimal ARMv8-M Non-Secure world reference application.
 *
 * Demonstrates:
 *   1. Calling Secure entry functions via the SG veneer library
 *      (libentryveneers.a produced by the Secure build).
 *   2. Thread-safe NS entry calls using TZ_THREAD_SAFE_NONSECURE_ENTRY_FUNC
 *      (or the mutex variant when CONFIG_ARM_TZ_ENTRY_MUTEX=y).
 *   3. GPR-scrubbed wrapper using Z_ARM_TZ_NS_ENTRY.
 *
 * Build using `west secure-build` which orchestrates building the Secure
 * image first, extracts its veneer library, and then builds this NS image.
 *
 * Reference: ARM DDI 0553B §C1.4 (NS call rules).
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>

/* Declarations of Secure entry functions (implemented in trustzone_secure). */
extern uint32_t secure_entry_xor(uint32_t a, uint32_t b);
extern int      secure_entry_write_byte(uint8_t *buf, uint8_t val);

/* -----------------------------------------------------------------------
 * Thread-safe wrapper for secure_entry_xor.
 *
 * When CONFIG_ARM_TZ_ENTRY_MUTEX=y this uses a per-function mutex.
 * Otherwise it falls back to k_sched_lock.
 * ----------------------------------------------------------------------- */
TZ_THREAD_SAFE_NONSECURE_ENTRY_FUNC_MUTEX(
	secure_xor_safe, uint32_t, secure_entry_xor,
	uint32_t a, uint32_t b)

/* -----------------------------------------------------------------------
 * Worker thread — exercises the Secure entry function concurrently.
 * ----------------------------------------------------------------------- */
#define STACK_SIZE  1024
#define NUM_THREADS 4

static K_THREAD_STACK_ARRAY_DEFINE(stacks, NUM_THREADS, STACK_SIZE);
static struct k_thread threads[NUM_THREADS];

static void worker(void *p1, void *p2, void *p3)
{
	int id = (int)(intptr_t)p1;
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (int i = 0; i < 10; i++) {
		uint32_t a = (uint32_t)(id * 100 + i);
		uint32_t b = (uint32_t)(id * 7 + 3);
		uint32_t result = secure_xor_safe(a, b);

		if (result != (a ^ b)) {
			printk("ERROR: thread %d iter %d: "
			       "expected 0x%08X got 0x%08X\n",
			       id, i, a ^ b, result);
		}
	}
}

int main(void)
{
	printk("TrustZone NS sample: Non-Secure world running\n");

	/* Demonstrate a direct (un-wrapped) call on the main thread. */
	uint32_t val = secure_entry_xor(0xDEAD, 0xBEEF);

	printk("secure_entry_xor(0xDEAD, 0xBEEF) = 0x%08X "
	       "(expect 0x%08X)\n", val, 0xDEADU ^ 0xBEEFU);

	/* Launch workers to exercise concurrent NS→Secure calls. */
	for (int i = 0; i < NUM_THREADS; i++) {
		k_thread_create(&threads[i], stacks[i], STACK_SIZE,
				worker, (void *)(intptr_t)i, NULL, NULL,
				K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
	}

	for (int i = 0; i < NUM_THREADS; i++) {
		k_thread_join(&threads[i], K_SECONDS(10));
	}

	printk("TrustZone NS sample: done\n");
	return 0;
}
