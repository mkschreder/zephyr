/*
 * Copyright (c) 2019 Microchip Technology Inc.
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <pm/pm.h>
#include <soc.h>
#include <init.h>

#include <stm32f4xx_hal.h>
#include <stm32f4xx_ll_utils.h>
#include <stm32f4xx_ll_bus.h>
#include <stm32f4xx_ll_cortex.h>
#include <stm32f4xx_ll_pwr.h>
#include <stm32f4xx_ll_rcc.h>
#include <stm32f4xx_ll_system.h>
#include <clock_control/clock_stm32_ll_common.h>

#include <logging/log.h>

LOG_MODULE_DECLARE(os, CONFIG_SOC_LOG_LEVEL);

void stm32_pwr_deepsleep_enter_mainregu(){
	/* ensure the proper wake-up system clock */
	//LL_RCC_SetClkAfterWakeFromStop(RCC_STOP_WAKEUPCLOCK_SELECTED);
	/* enter STOP0 mode */
	//HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON,PWR_SLEEPENTRY_WFE);

}

void stm32_pwr_deepsleep_enter_lpregu(){
	/* ensure the proper wake-up system clock */
	//LL_RCC_SetClkAfterWakeFromStop(RCC_STOP_WAKEUPCLOCK_SELECTED);
	//LL_SYSTICK_DisableIT();

	//LL_SYSTICK_EnableIT();
}

void stm32_pwr_standby_enter(){

}

/*
 * Called from pm_system_suspend(int32_t ticks) in subsys/power.c
 * For deep sleep pm_system_suspend has executed all the driver
 * power management call backs.
 */
void pm_power_state_set(struct pm_state_info info)
{
	//LL_PWR_MODE_STOP_MAINREGU /*!< Enter Stop mode when the CPU enters deepsleep */
	//LL_PWR_MODE_STOP_LPREGU /*!< Enter Stop mode (with low power Regulator ON) when the CPU enters deepsleep */
	//LL_PWR_MODE_STOP_MAINREGU_UNDERDRIVE /*!< Enter Stop mode (with main Regulator in under-drive mode) when the CPU enters deepsleep */
	//LL_PWR_MODE_STOP_LPREGU_UNDERDRIVE /*!< Enter Stop mode (with low power Regulator in under-drive mode) when the CPU enters deepsleep */
	//LL_PWR_MODE_STOP_MAINREGU_DEEPSLEEP /*!< Enter Stop mode (with main Regulator in Deep Sleep mode) when the CPU enters deepsleep */
	//LL_PWR_MODE_STOP_LPREGU_DEEPSLEEP /*!< Enter Stop mode (with low power Regulator in Deep Sleep mode) when the CPU enters deepsleep */
	//LL_PWR_MODE_STANDBY /*!< Enter Standby mode when the CPU enters deepsleep */
	HAL_DBGMCU_EnableDBGSleepMode();
	HAL_DBGMCU_EnableDBGStopMode();
	HAL_DBGMCU_EnableDBGStandbyMode();

	if (info.state == PM_STATE_SUSPEND_TO_IDLE) {
		LL_PWR_SetPowerMode(LL_PWR_MODE_STOP_LPREGU);
		LL_LPM_EnableSleep();
	} else if (info.state == PM_STATE_SUSPEND_TO_RAM) {
		switch (info.substate_id) {
			case 1: /* this corresponds to the STOP_MAINREGU mode: */
				LL_PWR_ClearFlag_WU();
				LL_PWR_SetPowerMode(LL_PWR_MODE_STOP_MAINREGU);
				LL_LPM_EnableDeepSleep();
				break;
			case 2: /* this corresponds to the LPREGU mode: */
				LL_PWR_ClearFlag_WU();
				LL_PWR_SetPowerMode(LL_PWR_MODE_STOP_LPREGU);
				LL_LPM_EnableDeepSleep();
				break;
			default:
				LOG_DBG("Unsupported power state substate-id %u",
					info.substate_id);
				break;
		}
	} else if(info.state == PM_STATE_STANDBY){
		LL_PWR_SetPowerMode(LL_PWR_MODE_STANDBY);
		LL_LPM_EnableDeepSleep();
	} else {
		//LOG_DBG("Unsupported power state %u", info.state);
	}

	// enter idle state and wait for interrupt
	k_cpu_idle();
}

void pm_power_state_exit_post_ops(struct pm_state_info info)
{
	if (info.state == PM_STATE_SUSPEND_TO_IDLE) {
		switch (info.substate_id) {
		case 1:	/* STOP0 */
			__fallthrough;
		case 2:	/* STOP1 */
			//HAL_ResumeTick();
			//LL_SYSTICK_EnableIT();
			__fallthrough;
		case 3:	/* STOP2 */
			LL_LPM_DisableSleepOnExit();
			LL_LPM_EnableSleep();
			stm32_clock_control_init(NULL);
			break;
		default:
			//LOG_DBG("Unsupported power substate-id %u", info.substate_id);
			break;
		}
	} else if (info.state == PM_STATE_SUSPEND_TO_RAM) {
		//LOG_DBG("Restoring clocks\n");
		stm32_clock_control_init(NULL);
	} else {
		//LOG_DBG("Unsupported power substate-id %u", info.state);
	}

	irq_unlock(0);
}
