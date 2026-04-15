/*
 * SPDX-License-Identifier: Apache-2.0
 * ARMv7-M exception torture suite entry point.
 *
 * We provide the k_sys_fatal_error_handler here so the harness
 * expectation mechanism works across all exception test files.
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <armv7m_test.h>

ZTEST_SUITE(armv7m_exception, NULL, NULL, NULL, NULL, NULL);

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	ARMV7M_FAULT_HANDLER_BODY(reason, esf);
}
