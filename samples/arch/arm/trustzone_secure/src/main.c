/*
 * Copyright 2026 Martin Schröder <info@swedishembedded.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * trustzone_secure — minimal ARMv8-M Secure world reference application.
 *
 * Demonstrates:
 *   1. SAU region programming from Devicetree (CONFIG_ARM_SAU_DT).
 *   2. A Secure entry function with NS-caller enforcement.
 *   3. NS world launch via arm_tz_launch_nonsecure().
 *
 * Board-specific SAU regions are declared in the board's DTS overlay.  See
 * boards/arm/mps2_an521/mps2_an521_cpu0_s.overlay for an example.
 *
 * Reference: ARM DDI 0553B §C1.4 (cmse_nonsecure_entry); §B3.4 (SAU).
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <arm_cmse.h>

#include <zephyr/arch/arm/cortex_m/tz_handoff.h>
#include <zephyr/arch/arm/cortex_m/cmse.h>
#include <cmsis_core.h>

/* -----------------------------------------------------------------------
 * Secure entry function — callable from Non-Secure state only.
 * Placed in the NSC region by the linker (arm,armv8m-sau-region with nsc).
 * ----------------------------------------------------------------------- */

/**
 * @brief Return the XOR of two uint32_t values.
 *
 * Trivial example to show parameter validation via cmse_check_address_range.
 */
uint32_t __attribute__((cmse_nonsecure_entry))
secure_entry_xor(uint32_t a, uint32_t b)
{
	Z_ARM_TZ_REQUIRE_NS_CALLER();
	return a ^ b;
}

/**
 * @brief Write a byte to a caller-supplied Non-Secure buffer.
 *
 * Validates that @p buf is actually readable/writable from Non-Secure state
 * before touching it.
 */
int __attribute__((cmse_nonsecure_entry))
secure_entry_write_byte(uint8_t *buf, uint8_t val)
{
	Z_ARM_TZ_REQUIRE_NS_CALLER();

	if (!cmse_check_address_range(buf, sizeof(*buf),
				      CMSE_NONSECURE | CMSE_MPU_READWRITE)) {
		return -EPERM;
	}

	*buf = val;
	return 0;
}

/* -----------------------------------------------------------------------
 * Non-Secure world bootstrap parameters.
 *
 * In a real system these come from a linker-exported symbol or a fixed
 * partition descriptor agreed upon between the Secure and NS images.
 * Here we use Kconfig symbols set by the board/west secure-build.
 * ----------------------------------------------------------------------- */

#ifndef CONFIG_ARM_NS_IMAGE_VTOR
#define CONFIG_ARM_NS_IMAGE_VTOR 0x00200000U
#endif

#ifndef CONFIG_ARM_NS_MSP_INITIAL
#define CONFIG_ARM_NS_MSP_INITIAL 0x20040000U
#endif

/* -----------------------------------------------------------------------
 * Application entry point.
 * ----------------------------------------------------------------------- */

int main(void)
{
	printk("TrustZone Secure sample: Secure world booting\n");

	printk("SAU configured from DT\n");

	const struct arm_tz_handoff_descriptor ns_desc = {
		.vtor_ns    = CONFIG_ARM_NS_IMAGE_VTOR,
		.msp_ns     = CONFIG_ARM_NS_MSP_INITIAL,
		.reset_ns   = *((uint32_t *)(CONFIG_ARM_NS_IMAGE_VTOR + 4U)),
		.psplim_ns  = 0U,
	};

	printk("Launching Non-Secure world at 0x%08X\n", ns_desc.vtor_ns);

	arm_tz_launch_nonsecure(&ns_desc);

	/* arm_tz_launch_nonsecure() is noreturn. */
	CODE_UNREACHABLE;
	return 0;
}
