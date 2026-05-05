/*
 * Copyright (c) 2026 Zephyr Project contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief NVIC_ITNS programming from device tree security attributes
 *
 * For each interrupt declared with `security = "non-secure"` in the device
 * tree this module sets the corresponding NVIC->ITNS bit at boot so that
 * the interrupt is routed to the Non-Secure NVIC view.
 *
 * The programming happens in PRE_KERNEL_2 (before drivers that may enable
 * interrupts), ensuring all IRQ targets are correct before any NS code runs.
 *
 * Reference: ARM DDI 0553 §B3.13 (NVIC_ITNS).
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/devicetree.h>
#include <cmsis_core.h>

#if defined(CONFIG_ARM_SECURE_FIRMWARE) && defined(CONFIG_ARM_NVIC_ITNS_DT)

/*
 * DT iteration macro: for every IRQ specifier that carries
 * security = "non-secure", set the ITNS bit.
 *
 * The DT binding models the security attribute as a property on the
 * interrupt controller node or on individual IRQ specifiers.  We use
 * the generated list produced by CONFIG_ARM_NVIC_ITNS_DT_IRQ_LIST which
 * is a comma-separated list of absolute IRQ numbers to route NS.
 *
 * If the list is empty this file compiles to a no-op.
 */

/* CONFIG_ARM_NVIC_ITNS_DT_NONSECURE_IRQS is a space-separated list of
 * absolute IRQ numbers generated from DT "interrupts" + "security" cells.
 * It is set via Kconfig string when CONFIG_ARM_NVIC_ITNS_DT=y.
 *
 * The canonical way to generate this list is via a DT chosen node or via
 * an IRQ controller binding that exposes a `secure` cell.  Until a DT
 * binding standardises it here we implement the programming hook and allow
 * board/SoC files to set the Kconfig string.
 */

/**
 * @brief Set a single IRQ to target Non-Secure state.
 *
 * @param irq Absolute IRQ number (0-based, external interrupts only).
 */
static inline void nvic_itns_set_ns(unsigned int irq)
{
	__ASSERT(irq < (sizeof(NVIC->ITNS) * 8U),
		 "IRQ %u out of NVIC_ITNS range", irq);
	NVIC->ITNS[irq / 32U] |= BIT(irq % 32U);
}

/**
 * @brief Program NVIC_ITNS from the DT-derived NS IRQ list.
 */
static int z_arm_nvic_itns_dt_init(void)
{
	/*
	 * Iterate over the compile-time list of NS IRQs.
	 *
	 * ARM_NVIC_ITNS_IRQ_LIST is defined by CMakeLists.txt as a raw,
	 * unquoted token list (e.g. 5, 12, 23) derived from the Kconfig
	 * string CONFIG_ARM_NVIC_ITNS_DT_NONSECURE_IRQS.  CMake only emits
	 * the definition when the string is non-empty, so the block below
	 * compiles away completely when no IRQs are configured.
	 *
	 * (CONFIG_ARM_NVIC_ITNS_DT_NONSECURE_IRQS itself is a quoted string
	 *  and therefore cannot be used as a C integer array initialiser.)
	 */
#ifdef ARM_NVIC_ITNS_IRQ_LIST
	static const uint16_t ns_irqs[] = {
		ARM_NVIC_ITNS_IRQ_LIST
	};
	for (size_t i = 0U; i < ARRAY_SIZE(ns_irqs); i++) {
		nvic_itns_set_ns(ns_irqs[i]);
	}
	__DSB();
	__ISB();
#endif /* ARM_NVIC_ITNS_IRQ_LIST */
	return 0;
}

SYS_INIT(z_arm_nvic_itns_dt_init, PRE_KERNEL_2, CONFIG_ARM_NVIC_ITNS_DT_INIT_PRIORITY);

#endif /* CONFIG_ARM_SECURE_FIRMWARE && CONFIG_ARM_NVIC_ITNS_DT */
