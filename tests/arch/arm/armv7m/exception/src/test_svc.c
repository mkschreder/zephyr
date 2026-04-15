/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M exception: SVC (supervisor call).
 *
 * Reference: DDI0403E
 *   SVC A7.7.178 lines 16527-16580
 *   Exception entry B1.5.6 lines 22321-22371
 *   Exception return B1.5.8 lines 22486-22581
 *
 * Tests:
 *   - SVC via irq_offload: verifies SVCall exception is taken and returns
 *   - Register state (callee-saved) preserved across SVC entry/return
 *   - UDF triggers UsageFault / HardFault
 *
 * NOTE: In Zephyr, SVCall is owned by the kernel for system calls and
 * irq_offload.  PendSV is owned for context switching.  SysTick is owned
 * for the system tick.  These exceptions cannot be installed via
 * IRQ_CONNECT; instead we use the Zephyr-provided irq_offload API which
 * exercises the SVCall path, and we rely on Zephyr's scheduler for PendSV
 * and on k_uptime_get() for SysTick verification.
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/irq_offload.h>
#include <cmsis_core.h>
#include <armv7m_test.h>

/* ------------------------------------------------------------------
 * test_svc_fires: irq_offload exercises the SVCall path.
 * The offloaded function runs in handler mode (ISR context).
 * ------------------------------------------------------------------ */
static volatile bool svc_offload_ran;

static void offload_fn(const void *arg)
{
	ARG_UNUSED(arg);
	svc_offload_ran = true;
	/* Verify we're in handler mode (IPSR != 0) */
	uint32_t ipsr;

	__asm__ volatile("mrs %0, ipsr\n\t" : "=r"(ipsr));
	/* On ARM, when in an exception handler, IPSR != 0 */
	svc_offload_ran = (ipsr != 0);
}

ZTEST(armv7m_exception, test_svc_fires)
{
	svc_offload_ran = false;
	irq_offload(offload_fn, NULL);
	zassert_true(svc_offload_ran,
		     "irq_offload/SVCall did not run handler in handler mode");
}

/* ------------------------------------------------------------------
 * test_svc_preserves_regs: callee-saved regs (r4/r5) survive SVC.
 *
 * ARMv7-M DDI0403E B1.5.6: exception entry saves r0-r3,r12,lr,pc,xpsr.
 * Callee-saved regs (r4-r11) are NOT saved by hardware; the software
 * handler is responsible for preserving them per AAPCS.
 *
 * We use irq_offload (which uses SVC internally) to trigger an SVCall
 * and verify that r4/r5 are intact on return.
 * ------------------------------------------------------------------ */
static volatile uint32_t svc_reg_r4_check;
static volatile uint32_t svc_reg_r5_check;

static void offload_read_regs(const void *arg)
{
	ARG_UNUSED(arg);
	uint32_t r4v, r5v;

	__asm__ volatile(
		"mov %[a], r4\n\t"
		"mov %[b], r5\n\t"
		: [a] "=r"(r4v), [b] "=r"(r5v));
	svc_reg_r4_check = r4v;
	svc_reg_r5_check = r5v;
}

ZTEST(armv7m_exception, test_svc_preserves_regs)
{
	register uint32_t r4_v __asm__("r4") = 0xDEAD0001U;
	register uint32_t r5_v __asm__("r5") = 0xDEAD0002U;

	__asm__ volatile("" : : "r"(r4_v), "r"(r5_v));
	irq_offload(offload_read_regs, NULL);

	uint32_t r4_after, r5_after;

	__asm__ volatile(
		"mov %[a], r4\n\t"
		"mov %[b], r5\n\t"
		: [a] "=r"(r4_after), [b] "=r"(r5_after));

	zassert_equal(r4_after, 0xDEAD0001U,
		      "r4 corrupted across SVCall: 0x%08x", r4_after);
	zassert_equal(r5_after, 0xDEAD0002U,
		      "r5 corrupted across SVCall: 0x%08x", r5_after);
}

/* ------------------------------------------------------------------
 * test_udf_triggers_fault: UDF #imm16 triggers UsageFault / HardFault.
 * ARMv7-M DDI0403E A7.7.191 (UNDEFINED) / B1.5.14.
 * ------------------------------------------------------------------ */
ZTEST(armv7m_exception, test_udf_triggers_fault)
{
	armv7m_expected_reason = K_ERR_CPU_EXCEPTION;
	armv7m_fault_fired = false;

	__asm__ volatile("udf #0\n\t" : : : "memory");

	zassert_true(armv7m_fault_fired, "UDF did not fault");
	armv7m_fault_fired = false;
}
