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

/* USER CODE BEGIN Private defines */
#ifndef MAX_PULSE_RER_ROTATION
#define MAX_PULSE_RER_ROTATION 360
#endif

#define DYNO_STARTED          0x0
#define DYNO_STOPPED          0x1
#define SENDBIT_DISABLE       0x0
#define SENDBIT_ENABLE        0x1
#define MAX6675_NOT_CONNECTED 0x0
#define MAX6675_CONNECTED     0x1

typedef enum {
    USBCMD_NOOP = 0x0,
    USBCMD_START = 0x1,
    USBCMD_STOP = 0x2,
    USBCMD_RESTART = 0x3,
    USBCMD_SHUTDOWN = 0x4
} DYNO_USBCmd;
// pub time: u32, // in ms
// pub pulse_encoder_max: u32,
// pub pulse_encoder: u32,
// pub pulse_rpm: u32,
// pub temperature: f32,
typedef struct {
    uint32_t time;
    uint32_t period;
    uint32_t pulse_encoder_max;
    uint32_t pulse_enc_raw;
    uint32_t pulse_enc;
    uint32_t pulse_enc_z;
    uint32_t pulse_rpm;
    float temperature;
} DataDyno;

#define DYNO_SIZE_DATA sizeof(DataDyno)
#define DATA_DYNO_INIT                                                                             \
    {                                                                                              \
        .time = 0,                                                                                 \
        .period = 0,                                                                               \
        .pulse_encoder_max = MAX_PULSE_RER_ROTATION,                                               \
        .pulse_enc = 0,                                                                            \
        .pulse_enc_z = 0,                                                                          \
        .pulse_rpm = 0,                                                                            \
        .temperature = 0.0,                                                                        \
    };

#define TIM_CHANNEL_ENC TIM_CHANNEL_ALL
#define TIM_CHANNEL_RPM TIM_CHANNEL_4

#ifdef WITH_PHASE_Z
#define TIM_CHANNEL_PHASE_Z TIM_CHANNEL_3
#endif

#define CH_RPM CCR4
#ifdef WITH_PHASE_Z
#define CH_PHASE_Z CCR3
#endif

#define GET_COUNTER(HTIM)          (HTIM.Instance->CNT)
#define GET_COUNTER_CH(HTIM, CH)   (HTIM.Instance->CH)
#define RESET_COUNTER(HTIM)        (HTIM.Instance->CNT = 0)
#define RESET_COUNTER_CH(HTIM, CH) (HTIM.Instance->CH = 0)

#define HANDLE_HAL_STATUS__(CALL)                                                                  \
    if (CALL == HAL_ERROR) {                                                                       \
        Error_Handler();                                                                           \
    }

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
