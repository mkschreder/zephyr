/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M torture test harness -- C implementation.
 */

#include <armv7m_test.h>
#include <zephyr/sys/printk.h>
#include <string.h>

/* -----------------------------------------------------------------------
 * Fault expectation state
 * --------------------------------------------------------------------- */

volatile unsigned int armv7m_expected_reason = (unsigned int)-1;
volatile bool armv7m_fault_fired;

void armv7m_fault_handler_record(unsigned int reason, const struct arch_esf *esf)
{
	ARG_UNUSED(esf);

	if (armv7m_expected_reason == (unsigned int)-1) {
		printk("armv7m_test: unexpected fault reason=%u\n", reason);
		k_fatal_halt(reason);
	}
	if (reason != armv7m_expected_reason) {
		printk("armv7m_test: wrong fault reason got=%u expected=%u\n",
		       reason, armv7m_expected_reason);
		k_fatal_halt(reason);
	}

	armv7m_fault_fired = true;
	armv7m_expected_reason = (unsigned int)-1;
}

void armv7m_expect_fault(unsigned int expected_reason, void (*fn)(void))
{
	armv7m_fault_fired = false;
	armv7m_expected_reason = expected_reason;

	fn();

	/* Allow the fault handler to run if we came back immediately */
	k_yield();

	zassert_true(armv7m_fault_fired,
		     "expected fault reason=%u did not fire", expected_reason);
	armv7m_fault_fired = false;
}

/* -----------------------------------------------------------------------
 * Memory helpers
 * --------------------------------------------------------------------- */

void armv7m_fill_pattern(void *buf, size_t len, uint32_t seed)
{
	uint8_t *p = (uint8_t *)buf;
	uint32_t state = seed;

	for (size_t i = 0; i < len; i++) {
		/* Linear congruential: multiplier 1664525, addend 1013904223 */
		state = state * 1664525U + 1013904223U;
		p[i] = (uint8_t)(state >> 24);
	}
}

uint32_t armv7m_hash32(const void *buf, size_t len)
{
	const uint8_t *p = (const uint8_t *)buf;
	/* FNV-1a 32-bit */
	uint32_t h = 2166136261U;

	for (size_t i = 0; i < len; i++) {
		h ^= p[i];
		h *= 16777619U;
	}
	return h;
}
