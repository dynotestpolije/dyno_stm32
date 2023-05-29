/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "main.h"
/* Private includes ----------------------------------------------------------*/

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern SPI_HandleTypeDef hspi_max6675;
extern TIM_HandleTypeDef htim_encoder;
extern TIM_HandleTypeDef htim_send_timer;
extern TIM_HandleTypeDef htim_rpm;

#ifdef WITH_PHASE_Z
extern TIM_HandleTypeDef htim_phase_z;
#endif

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void) {
    while (1) {
    }
}
/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void) {
    while (1) {
        /* USER CODE BEGIN W1_HardFault_IRQn 0 */
        /* USER CODE END W1_HardFault_IRQn 0 */
    }
}
/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void) {
    while (1) {
    }
}
/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void) {
    while (1) {
    }
}
/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void) {
    while (1) {
    }
}
/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void) {}
/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void) {}
/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void) {}
/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void) { HAL_IncTick(); }

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles TIM2 global interrupt.
 */
void TIM2_IRQHandler(void) { HAL_TIM_IRQHandler(&htim_encoder); }
/**
 * @brief This function handles TIM3 global interrupt.
 */
void TIM3_IRQHandler(void) { HAL_TIM_IRQHandler(&htim_send_timer); }
/**
 * @brief This function handles TIM4 global interrupt.
 */
void TIM4_IRQHandler(void) { HAL_TIM_IRQHandler(&htim_rpm); }
/**
 * @brief This function handles SPI1 global interrupt.
 */
void SPI1_IRQHandler(void) { HAL_SPI_IRQHandler(&hspi_max6675); }

#ifdef WITH_PHASE_Z
/**
 * @brief This function handles TIM5 global interrupt.
 */
void TIM5_IRQHandler(void) { HAL_TIM_IRQHandler(&htim_phase_z); }
#endif
/**
 * @brief This function handles USB On The Go FS global interrupt.
 */
void OTG_FS_IRQHandler(void) { HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS); }
