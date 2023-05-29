/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 *
 * Copyright (c) 2023 Rizal Achmad Pahlevi <echo 'cml6YWwuYWhtYWRwQGdtYWlsLmNvbQo=' | base64 -d>
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#include <stm32f4xx_hal.h>
#ifdef __cplusplus
extern "C" {
#endif

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
/* Private defines -----------------------------------------------------------*/
#define ENCODER_CH1_Pin             GPIO_PIN_0
#define ENCODER_CH1_GPIO_Port       GPIOA
#define ENCODER_CH2_Pin             GPIO_PIN_1
#define ENCODER_CH2_GPIO_Port       GPIOA
#define ENCODER_CH3_Pin             GPIO_PIN_2
#define ENCODER_CH3_GPIO_Port       GPIOA
#define LED_INDICATOR_1_Pin         GPIO_PIN_3
#define LED_INDICATOR_1_GPIO_Port   GPIOA
#define LED_INDICATOR_2_Pin         GPIO_PIN_4
#define LED_INDICATOR_2_GPIO_Port   GPIOA
#define MAX6675_SCK_Pin             GPIO_PIN_5
#define MAX6675_SCK_GPIO_Port       GPIOA
#define MAX6675_MISO_Pin            GPIO_PIN_6
#define MAX6675_MISO_GPIO_Port      GPIOA
#define MAX6675_CS_Pin              GPIO_PIN_0
#define MAX6675_CS_GPIO_Port        GPIOB
#define USBD_DM_Pin                 GPIO_PIN_11
#define USBD_DM_GPIO_Port           GPIOA
#define USBD_DP_Pin                 GPIO_PIN_12
#define USBD_DP_GPIO_Port           GPIOA
#define RPM_INPUT_CAPTURE_Pin       GPIO_PIN_9
#define RPM_INPUT_CAPTURE_GPIO_Port GPIOB

#define SET_PIN(PORT, PIN)          ((PORT)->BSRR = (PIN))
#define RESET_PIN(PORT, PIN)        ((PORT)->BSRR = (uint32_t)(PIN) << 16U)

#define DYNO_STARTED                0x0
#define DYNO_STOPPED                0x1
#define SENDBIT_DISABLE             0x0
#define SENDBIT_ENABLE              0x1

#define MAX6675_NOT_CONNECTED       0x0
#define MAX6675_CONNECTED           0x1

#define TIM_CHANNEL_ENC             TIM_CHANNEL_ALL
#define TIM_CHANNEL_RPM             TIM_CHANNEL_4

#ifdef WITH_PHASE_Z
#define TIM_CHANNEL_PHASE_Z TIM_CHANNEL_3
#endif

#define CH_RPM CCR4

#ifdef WITH_PHASE_Z
#define CH_PHASE_Z CCR3
#endif

#define GET_COUNTER(HTIM)   ((HTIM).Instance->CNT)
#define RESET_COUNTER(HTIM) ((HTIM).Instance->CNT = 0)

#define HANDLE_HAL_STATUS__(CALL)                                                                  \
    if (CALL != HAL_OK) {                                                                          \
        Error_Handler();                                                                           \
    }

void MX_Start(void);
void MX_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
