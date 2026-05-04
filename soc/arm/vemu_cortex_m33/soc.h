/*
 * Copyright (c) 2026 Martin Schröder <info@swedishembedded.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC_H_
#define _SOC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* IRQ numbers for vemu Cortex-M33 virtual SoC */
typedef enum IRQn {
  NonMaskableInt_IRQn   = -14,
  HardFault_IRQn        = -13,
  MemoryManagement_IRQn = -12,
  BusFault_IRQn         = -11,
  UsageFault_IRQn       = -10,
  SecureFault_IRQn      =  -9,
  SVCall_IRQn           =  -5,
  DebugMonitor_IRQn     =  -4,
  PendSV_IRQn           =  -2,
  SysTick_IRQn          =  -1,
  /* External interrupts */
  IRQ0_IRQn             =   0,
  IRQ1_IRQn             =   1,
  IRQ2_IRQn             =   2,
  IRQ3_IRQn             =   3,
} IRQn_Type;

/* Cortex-M33 processor configuration */
#define __CM33_REV              0x0000U  /* Core revision r0p0 */
#define __SAUREGION_PRESENT     1U       /* SAU regions present */
#define __MPU_PRESENT           1U       /* MPU present */
#define __VTOR_PRESENT          1U       /* VTOR present */
#define __NVIC_PRIO_BITS        3U       /* Number of Bits used for Priority Levels */
#define __Vendor_SysTickConfig  0U       /* Standard SysTick config */
#define __FPU_PRESENT           0U       /* No FPU */
#define __DSP_PRESENT           0U       /* No DSP extension */

#ifdef __cplusplus
}
#endif

#include <core_cm33.h>

#ifndef _ASMLANGUAGE
#include <zephyr/sys/util.h>
#endif

#endif /* _SOC_H_ */
