/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M torture test shared harness.
 *
 * Provides:
 *   - Register/flag capture (via assembly, avoiding compiler interference)
 *   - APSR flag accessors
 *   - Deterministic memory pattern fill and hash
 *   - Fault-expectation wrapper compatible with Zephyr's k_sys_fatal_error_handler
 */

#ifndef ARMV7M_TEST_H
#define ARMV7M_TEST_H

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------
 * Register signature.
 * armv7m_capture_sig() is implemented in armv7m_test_asm.S to avoid any
 * compiler-generated instructions between the code under test and the
 * capture point.
 * --------------------------------------------------------------------- */
struct armv7m_sig {
	uint32_t r[13]; /* r0-r12 */
	uint32_t sp;    /* r13 (current SP, read via mov not MRS) */
	uint32_t lr;    /* r14 */
	uint32_t pc;    /* r15, holds return address at time of capture */
	uint32_t xpsr;  /* full xPSR via MRS */
};

/**
 * @brief Capture current register state into @p sig.
 *
 * Uses inline assembly to snapshot r0-r12, SP, LR, PC (return addr),
 * and APSR without any intervening compiler-generated instructions.
 * Call this *immediately* after the sequence under test.
 */
void armv7m_capture_sig(struct armv7m_sig *sig);

/* -----------------------------------------------------------------------
 * APSR helpers.  All take the raw 32-bit APSR value returned by MRS.
 * APSR layout (DDI0403E A2.3.2, lines 769-794):
 *   bit 31 = N, bit 30 = Z, bit 29 = C, bit 28 = V, bit 27 = Q
 *   bits [19:16] = GE[3:0]
 * --------------------------------------------------------------------- */

static inline uint32_t armv7m_read_apsr(void)
{
	uint32_t v;

	__asm__ volatile("mrs %0, apsr" : "=r"(v));
	return v;
}

static inline bool armv7m_flag_n(uint32_t apsr)
{
	return (apsr >> 31) & 1U;
}

static inline bool armv7m_flag_z(uint32_t apsr)
{
	return (apsr >> 30) & 1U;
}

static inline bool armv7m_flag_c(uint32_t apsr)
{
	return (apsr >> 29) & 1U;
}

static inline bool armv7m_flag_v(uint32_t apsr)
{
	return (apsr >> 28) & 1U;
}

static inline bool armv7m_flag_q(uint32_t apsr)
{
	return (apsr >> 27) & 1U;
}

static inline uint8_t armv7m_flag_ge(uint32_t apsr)
{
	return (apsr >> 16) & 0xFU;
}

/* Mask of N/Z/C/V bits together */
#define ARMV7M_NZCV_MASK  0xF0000000U
/* Mask of N/Z/C/V/Q bits */
#define ARMV7M_NZCVQ_MASK 0xF8000000U

/* -----------------------------------------------------------------------
 * Memory helpers
 * --------------------------------------------------------------------- */

/**
 * @brief Fill @p buf with a deterministic pattern derived from @p seed.
 *
 * Uses a linear congruential generator so the pattern is reproducible
 * and independent of the target state.
 */
void armv7m_fill_pattern(void *buf, size_t len, uint32_t seed);

/**
 * @brief Compute a simple 32-bit hash over @p buf.
 *
 * Algorithm: FNV-1a 32-bit.  Suitable for comparing memory regions
 * between a reference and the device under test.
 */
uint32_t armv7m_hash32(const void *buf, size_t len);

/* -----------------------------------------------------------------------
 * Fault expectation.
 *
 * Usage:
 *   armv7m_expect_fault(K_ERR_CPU_EXCEPTION, my_fault_fn);
 *
 * The helper:
 *   1. Sets the global expected_reason so k_sys_fatal_error_handler
 *      records success rather than aborting.
 *   2. Calls fn().
 *   3. Asserts that a fault actually fired (via a flag the handler sets).
 *
 * The test application must define k_sys_fatal_error_handler and call
 * armv7m_fault_handler_check() from within it.  A convenience macro is
 * provided below.
 * --------------------------------------------------------------------- */
extern volatile unsigned int armv7m_expected_reason;
extern volatile bool armv7m_fault_fired;

/**
 * @brief Record that the fault handler was called with the right reason.
 *
 * Call this at the top of k_sys_fatal_error_handler:
 *
 *   void k_sys_fatal_error_handler(unsigned int reason,
 *                                   const struct arch_esf *esf)
 *   {
 *       ARMV7M_FAULT_HANDLER_BODY(reason, esf);
 *   }
 */
void armv7m_fault_handler_record(unsigned int reason, const struct arch_esf *esf);

/**
 * @brief Run fn() and assert that it triggers the expected fault.
 */
void armv7m_expect_fault(unsigned int expected_reason, void (*fn)(void));

/**
 * @brief Convenience macro for k_sys_fatal_error_handler body.
 */
#define ARMV7M_FAULT_HANDLER_BODY(reason, esf) \
	armv7m_fault_handler_record((reason), (esf))

#ifdef __cplusplus
}
#endif

#endif /* ARMV7M_TEST_H */
