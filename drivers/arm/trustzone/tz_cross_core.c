/*
 * Copyright 2026 Martin Schröder <info@swedishembedded.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Skeleton driver for the arm,trustzone-cross-core DT binding.
 *
 * Provides:
 *   - Initialization hook for cross-core security coordination hardware.
 *   - Weak callback z_arm_tz_cross_core_notify_ns_reset() that SoC BSPs
 *     override to implement vendor-specific cross-core signalling.
 *
 * On nRF5340: the Application Core SPU controls which resources are
 * accessible from the Network Core.  This driver's init hook would configure
 * the SPU and enable the IPC peripheral for cross-core messaging.
 *
 * On STM32H7 dual-core: the HSEM (Hardware Semaphore) and IPCC peripherals
 * serve the same role.
 *
 * Extend this skeleton by implementing the weak hooks and adding SoC-specific
 * DTS overlays that instantiate arm,trustzone-cross-core nodes.
 *
 * Reference: ARM DDI 0553B §A1.3.2 (multi-core TrustZone).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/sys/printk.h>

#define DT_DRV_COMPAT arm_trustzone_cross_core

/**
 * @brief Notify the Non-Secure core of a security event (weak hook).
 *
 * SoC BSPs override this to implement the actual cross-core notification
 * mechanism (e.g. IPCC on STM32H7, IPC on nRF5340, SEMA42 on i.MX8).
 *
 * @param event_code  Vendor-defined event code (0 = NS reset requested).
 */
__weak void z_arm_tz_cross_core_notify_ns(uint32_t event_code)
{
	ARG_UNUSED(event_code);
	/* Default: no-op.  Override in SoC-specific code. */
}

/**
 * @brief Called when the Secure world receives a reset request from NS core.
 *
 * Override this weak hook to implement policy (e.g. validate then grant, or
 * deny and log the attempt).
 *
 * @return 0 to allow the NS reset, -EPERM to deny it.
 */
__weak int z_arm_tz_cross_core_ns_reset_request(void)
{
	return 0; /* Allow by default. */
}

static int tz_cross_core_init(void)
{
	/*
	 * Nothing to do in the skeleton.  SoC-specific init (mailbox setup,
	 * shared-memory mapping, SPU configuration) goes here when the
	 * concrete implementation is added.
	 */
	return 0;
}

SYS_INIT(tz_cross_core_init, PRE_KERNEL_1, CONFIG_ARM_TZ_CROSS_CORE_INIT_PRIORITY);
