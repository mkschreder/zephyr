/*
 * Copyright 2026 Martin Schröder <info@swedishembedded.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shared harness for the arm_trustzone test suite.
 */

#ifndef ZEPHYR_TESTS_ARCH_ARM_ARM_TRUSTZONE_TZ_TEST_H_
#define ZEPHYR_TESTS_ARCH_ARM_ARM_TRUSTZONE_TZ_TEST_H_

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <cmsis_core.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * TT (Test Target) instruction helpers.
 * DDI 0553B §C2.338-340.
 *
 * TT r0, r1 — T1 encoding: 0xE841_F000
 */
static inline uint32_t __attribute__((always_inline)) tz_tt(uint32_t addr)
{
	uint32_t result;

	__asm__ volatile(
		"mov r1, %1\n\t"
		".inst.w 0xE841F000\n\t"   /* TT r0, r1 */
		"mov %0, r0\n\t"
		: "=r"(result) : "r"(addr) : "r0", "r1");
	return result;
}

/* TT_RESP bitfield layout (DDI 0553B §D1.2.266) */
#define TZ_TT_S_BIT       (1U << 22) /* address is Secure */
#define TZ_TT_NSR_BIT     (1U << 20) /* NS world can read */
#define TZ_TT_NSRW_BIT    (1U << 21) /* NS world can read-write */
#define TZ_TT_RW_BIT      (1U << 19) /* address is read-write */
#define TZ_TT_R_BIT       (1U << 18) /* address is readable */
#define TZ_TT_SRVALID_BIT (1U << 17) /* SAU region matched */
#define TZ_TT_MRVALID_BIT (1U << 16) /* MPU region matched */
#define TZ_TT_SREGION(x)  (((x) >> 8) & 0xFFU)
#define TZ_TT_MREGION(x)  (((x) >> 0) & 0xFFU)

/* SAU MMIO registers (DDI 0553B §B3.4) */
#define TZ_SAU_BASE   (0xE000EDD0UL)
#define TZ_SAU_CTRL   (*(volatile uint32_t *)(TZ_SAU_BASE + 0x00))
#define TZ_SAU_TYPE   (*(volatile uint32_t *)(TZ_SAU_BASE + 0x04))
#define TZ_SAU_RNR    (*(volatile uint32_t *)(TZ_SAU_BASE + 0x08))
#define TZ_SAU_RBAR   (*(volatile uint32_t *)(TZ_SAU_BASE + 0x0C))
#define TZ_SAU_RLAR   (*(volatile uint32_t *)(TZ_SAU_BASE + 0x10))
#define TZ_SAU_SFSR   (*(volatile uint32_t *)(TZ_SAU_BASE + 0x14))
#define TZ_SAU_SFAR   (*(volatile uint32_t *)(TZ_SAU_BASE + 0x18))

#define TZ_SAU_CTRL_ENABLE  (1U << 0)
#define TZ_SAU_CTRL_ALLNS   (1U << 1)
#define TZ_SAU_RLAR_ENABLE  (1U << 0)
#define TZ_SAU_RLAR_NSC     (1U << 1)

/* SFSR bits (DDI 0553B §D1.2.266) */
#define TZ_SFSR_INVEP_BIT    (1U << 0)
#define TZ_SFSR_INVIS_BIT    (1U << 1)
#define TZ_SFSR_INVER_BIT    (1U << 2)
#define TZ_SFSR_AUVIOL_BIT   (1U << 3)
#define TZ_SFSR_INVTRAN_BIT  (1U << 4)
#define TZ_SFSR_LSPERR_BIT   (1U << 5)
#define TZ_SFSR_SFARVALID    (1U << 6)
#define TZ_SFSR_LSERR_BIT    (1U << 7)

/* AIRCR (DDI 0553B §D1.2.2) */
#define TZ_AIRCR_BASE       (0xE000ED0CUL)
#define TZ_AIRCR            (*(volatile uint32_t *)TZ_AIRCR_BASE)
#define TZ_AIRCR_VECTKEY    (0x05FAU << 16)
#define TZ_AIRCR_BFHFNMINS  (1U << 13)

/* Last fault reason captured by k_sys_fatal_error_handler override. */
extern volatile unsigned int tz_last_fault_reason;

/* Save/restore SAU state across tests. */
static inline uint32_t tz_sau_save(void) { return TZ_SAU_CTRL; }
static inline void tz_sau_restore(uint32_t saved)
{
	TZ_SAU_CTRL = 0U;
	TZ_SAU_CTRL = saved;
}

/** Program SAU region n: [base, limit] inclusive, 32-byte aligned. */
static inline void tz_sau_set_region(uint32_t n, uint32_t base, uint32_t limit, bool nsc)
{
	TZ_SAU_RNR  = n;
	TZ_SAU_RBAR = base & ~0x1FU;
	TZ_SAU_RLAR = (limit & ~0x1FU)
		    | (nsc ? TZ_SAU_RLAR_NSC : 0U)
		    | TZ_SAU_RLAR_ENABLE;
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_TESTS_ARCH_ARM_ARM_TRUSTZONE_TZ_TEST_H_ */
