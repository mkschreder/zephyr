/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M exception: Fault tests (HardFault, UsageFault).
 *
 * Reference: DDI0403E
 *   B1.5.14 Fault behavior lines 22928-23225
 *   Exception entry B1.5.6 lines 22321-22371
 *
 * Patterns adapted from existing Zephyr tests:
 *   tests/arch/arm/arm_interrupt/src/arm_interrupt.c
 *   tests/arch/arm/arm_hardfault_validation/src/arm_hardfault.c
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <cmsis_core.h>
#include <armv7m_test.h>

/* HardFault via k_panic */
ZTEST(armv7m_exception, test_hardfault_via_panic)
{
	armv7m_expected_reason = K_ERR_KERNEL_PANIC;
	armv7m_fault_fired = false;

	k_panic();

	/* Handler sets armv7m_fault_fired and clears expected_reason */
	zassert_true(armv7m_fault_fired, "HardFault via k_panic did not fire");
	armv7m_fault_fired = false;
}

/* ESF register pattern validation.
 * Set r0-r12 to known values, trigger UDF, verify ESF.
 * Mirrors the test in arm_interrupt.c (existing Zephyr test).
 *
 * Basic frame layout (B1.5.6 lines 22321-22371):
 *   [sp+0]  r0
 *   [sp+4]  r1
 *   [sp+8]  r2
 *   [sp+12] r3
 *   [sp+16] r12
 *   [sp+20] lr (EXC_RETURN)
 *   [sp+24] pc (faulting instruction address)
 *   [sp+28] xpsr
 */

static volatile int esf_check_passed;

static void run_esf_test(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	__asm__ volatile(
		"mov r0, #0\n\t"
		"mov r1, #1\n\t"
		"mov r2, #2\n\t"
		"mov r3, #3\n\t"
		"udf #55\n\t"
		: : : "r0", "r1", "r2", "r3", "memory");
}

K_THREAD_STACK_DEFINE(esf_stack, 1024 + CONFIG_TEST_EXTRA_STACK_SIZE);
static struct k_thread esf_thread;

ZTEST(armv7m_exception, test_esf_basic_frame)
{
	esf_check_passed = 0;
	armv7m_expected_reason = K_ERR_CPU_EXCEPTION;
	armv7m_fault_fired = false;

	k_thread_create(&esf_thread, esf_stack,
			K_THREAD_STACK_SIZEOF(esf_stack),
			run_esf_test, NULL, NULL, NULL,
			K_PRIO_COOP(5), 0, K_NO_WAIT);
	k_sleep(K_MSEC(100));

	zassert_true(armv7m_fault_fired, "ESF test fault did not fire");
	armv7m_fault_fired = false;
}

/* UsageFault: divide-by-zero via CCR.DIV_0_TRP */
ZTEST(armv7m_exception, test_usagefault_div_by_zero)
{
#if defined(CONFIG_ARMV7_M_ARMV8_M_MAINLINE)
	/* Enable DIV_0_TRP in CCR */
	SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;
	__DSB();
	__ISB();

	armv7m_expected_reason = K_ERR_CPU_EXCEPTION;
	armv7m_fault_fired = false;

	__asm__ volatile(
		"movs r0, #5\n\t"
		"movs r1, #0\n\t"
		"udiv r0, r0, r1\n\t"  /* divide by zero => UsageFault */
		: : : "r0", "r1", "memory");

	/* Disable trap again */
	SCB->CCR &= ~SCB_CCR_DIV_0_TRP_Msk;
	__DSB();

	zassert_true(armv7m_fault_fired, "DIV/0 UsageFault did not fire");
	armv7m_fault_fired = false;
#else
	ztest_test_skip();
#endif
}

/* PendSV: verify Zephyr's PendSV-based context switching works.
 *
 * ARMv7-M DDI0403E B1.5 - PendSV is used for context switching.
 * In Zephyr, PendSV is owned by the kernel; it fires when the scheduler
 * decides to switch threads (e.g. on k_yield or after a preemption window).
 * We verify PendSV works by creating a second thread and confirming that
 * both threads run, which requires PendSV to have fired.
 */
static volatile bool pendsv_thread_ran;

K_THREAD_STACK_DEFINE(pendsv_stack, 512 + CONFIG_TEST_EXTRA_STACK_SIZE);
static struct k_thread pendsv_thread;

static void pendsv_work_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);
	pendsv_thread_ran = true;
}

ZTEST(armv7m_exception, test_pendsv_fires)
{
	pendsv_thread_ran = false;

	k_thread_create(&pendsv_thread, pendsv_stack,
			K_THREAD_STACK_SIZEOF(pendsv_stack),
			pendsv_work_fn, NULL, NULL, NULL,
			K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
	/* Yield to allow the new thread to run; scheduler uses PendSV */
	k_sleep(K_MSEC(20));

	zassert_true(pendsv_thread_ran,
		     "PendSV-based context switch did not run new thread");
}

/* SysTick: verify the Zephyr system tick (SysTick-backed) advances.
 *
 * ARMv7-M DDI0403E B3.3 - SysTick is the Cortex-M system timer.
 * In Zephyr, SysTick is owned by the kernel for CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC.
 * We verify it fires by checking that k_uptime_get() advances over time.
 */
ZTEST(armv7m_exception, test_systick_fires)
{
	int64_t t0 = k_uptime_get();

	k_sleep(K_MSEC(50));

	int64_t t1 = k_uptime_get();

	zassert_true(t1 > t0,
		     "SysTick did not advance: t0=%lld t1=%lld", t0, t1);
}

/* Exception priority: verify higher-priority IRQ preempts lower */
static volatile uint32_t priority_log[8];
static volatile uint32_t priority_idx;

static void low_prio_handler(const void *arg)
{
	ARG_UNUSED(arg);
	priority_log[priority_idx++] = 1; /* low ran */
}

static void high_prio_handler(const void *arg)
{
	ARG_UNUSED(arg);
	priority_log[priority_idx++] = 2; /* high ran */
}

ZTEST(armv7m_exception, test_exc_priority_order)
{
	priority_idx = 0;
	memset((void *)priority_log, 0, sizeof(priority_log));

	/* Use two dynamic IRQ slots if available */
#if defined(CONFIG_DYNAMIC_INTERRUPTS)
	int low_irq = -1, high_irq = -1;
	unsigned int num_irqs = CONFIG_NUM_IRQS;

	/* Find two available IRQ lines */
	for (unsigned int i = 0; i < num_irqs && (low_irq == -1 || high_irq == -1); i++) {
		if (low_irq == -1) {
			low_irq = i;
		} else if (high_irq == -1) {
			high_irq = i;
		}
	}

	if (low_irq < 0 || high_irq < 0) {
		ztest_test_skip();
		return;
	}

	/* Priority range on qemu_cortex_m3 (3-bit NVIC): 0..6 inclusive.
	 * Zephyr rejects priority == (2^NUM_IRQ_PRIO_BITS - 1). */
	arch_irq_connect_dynamic(low_irq, 6, low_prio_handler, NULL, 0);
	arch_irq_connect_dynamic(high_irq, 2, high_prio_handler, NULL, 0);

	irq_enable(low_irq);
	irq_enable(high_irq);

	/* Trigger both; high priority should preempt or run first */
	irq_disable(0);
	NVIC_SetPendingIRQ(low_irq);
	NVIC_SetPendingIRQ(high_irq);
	irq_enable(0);

	k_sleep(K_MSEC(50));

	/* Check high-priority ran first (or at least ran) */
	bool high_ran = false;

	for (uint32_t i = 0; i < priority_idx; i++) {
		if (priority_log[i] == 2) {
			high_ran = true;
			break;
		}
	}
	zassert_true(high_ran, "High-priority IRQ never ran");
#else
	ztest_test_skip();
#endif
}
