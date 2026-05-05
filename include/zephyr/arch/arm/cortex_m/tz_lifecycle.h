/*
 * Copyright 2026 Martin Schröder <info@swedishembedded.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ARMv8-M TrustZone-M device lifecycle management API.
 *
 * Many SoCs implement a device lifecycle state machine controlled by OTP
 * fuses, eFuses, or a lifecycle controller peripheral.  This header provides:
 *
 *   1. Symbolic lifecycle level constants.
 *   2. A declaration of the weak hook z_arm_secure_lifecycle_advance().
 *   3. A helper to read the current lifecycle state (SoC-specific).
 *
 * SoC BSPs override z_arm_secure_lifecycle_advance() to transition hardware
 * into the target lifecycle state by programming the appropriate registers or
 * fuses.
 *
 * Lifecycle transitions are irreversible; the implementation must verify that
 * the requested target level is reachable from the current state and reject
 * (by returning -EPERM) any invalid transition.
 *
 * Reference: ARM PSA Attestation API; ARM Platform Security Model 1.1 §3.5.
 */

#ifndef ZEPHYR_INCLUDE_ARCH_ARM_CORTEX_M_TZ_LIFECYCLE_H_
#define ZEPHYR_INCLUDE_ARCH_ARM_CORTEX_M_TZ_LIFECYCLE_H_

#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup tz_lifecycle TrustZone-M Device Lifecycle
 * @{
 */

/**
 * @name Device lifecycle levels.
 *
 * Aligned with PSA Lifecycle states (PSA Certified §3.5):
 *
 *   UNKNOWN         — Reset state; lifecycle controller not initialised.
 *   ASSEMBLY        — Device manufactured; RoT not yet provisioned.
 *   PSA_ROT_PROV    — PSA Root of Trust provisioned; development keys set.
 *   SECURED         — All keys provisioned; Secure debug disabled.
 *   NON_PSA_ROT     — Non-PSA RoT provisioned (platform-specific).
 *   RECOVERABLE_PSA — Secure world entered recoverable debug mode.
 *   DECOMMISSIONED  — End-of-life; all secrets erased.
 *
 * The numeric values are not standardised — SoC BSPs may define their own
 * encoding as long as they are monotonically increasing (no downgrade).
 * @{
 */
#define ARM_TZ_LIFECYCLE_UNKNOWN            0
#define ARM_TZ_LIFECYCLE_ASSEMBLY           1
#define ARM_TZ_LIFECYCLE_PSA_ROT_PROV       2
#define ARM_TZ_LIFECYCLE_SECURED            3
#define ARM_TZ_LIFECYCLE_NON_PSA_ROT        4
#define ARM_TZ_LIFECYCLE_RECOVERABLE_PSA    5
#define ARM_TZ_LIFECYCLE_DECOMMISSIONED     6
/** @} */

/**
 * @brief Advance the device lifecycle to the requested level (weak hook).
 *
 * This function is called by z_arm_tz_hardening_init() when
 * CONFIG_ARM_TZ_SECURE_DEBUG_DISABLE=y.  The default (weak) implementation
 * is a no-op and returns 0.
 *
 * SoC BSPs that support OTP-based lifecycle management override this symbol.
 * The implementation must:
 *   - Validate that the requested @p level is reachable from the current
 *     lifecycle state.
 *   - Transition the hardware (blow fuses, write lifecycle controller
 *     registers, etc.).
 *   - Return 0 on success or a negative errno on failure.
 *
 * This function MUST NOT be called after the first call to
 * arm_tz_launch_nonsecure() — lifecycle transitions are a Secure-only
 * operation that must complete before NS firmware starts.
 *
 * @param level  Target lifecycle level.  One of ARM_TZ_LIFECYCLE_*.
 * @return       0 on success, -EPERM if the transition is not allowed,
 *               negative errno for other errors.
 */
int z_arm_secure_lifecycle_advance(int level);

/**
 * @brief Query the current device lifecycle level (weak hook).
 *
 * The default implementation returns ARM_TZ_LIFECYCLE_UNKNOWN.  SoC BSPs
 * override this to read from the lifecycle controller or OTP shadow
 * registers.
 *
 * @return Current lifecycle level (ARM_TZ_LIFECYCLE_*), or a negative errno
 *         if the lifecycle controller cannot be read.
 */
int z_arm_secure_lifecycle_current(void);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ARCH_ARM_CORTEX_M_TZ_LIFECYCLE_H_ */
