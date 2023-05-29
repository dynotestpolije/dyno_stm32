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
#ifndef __DYNO_H
#define __DYNO_H
#include <stdint.h>

#ifndef MAX_PPR_ENCODER
#define MAX_PPR_ENCODER 360 // default max pulse
#endif

#ifndef MAX_RPM_ENCODER
#define MAX_RPM_ENCODER 6000
#endif

#ifndef PERIOD_SEND_DATA_MS
#define PERIOD_SEND_DATA_MS 200
#endif
// #define PERIOD_SEND_DATA_HZ (1000/200)
#ifndef PRESCALAR_ENCODER_PULSE
#define PRESCALAR_ENCODER_PULSE 4
#endif

#define _PRESCALAR_SD       2000
#define _HCLK_CPU           96000000

#define PERIOD_SEND_DATA_HZ (1000 / PERIOD_SEND_DATA_MS)
#define PRESCALAR_SEND_DATA (_PRESCALAR_SD - 1)
#define PERIOD_SEND_DATA    (_HCLK_CPU / (PERIOD_SEND_DATA_HZ * _PRESCALAR_SD)) - 1

#define MAX_RPS_ENCODER     ((MAX_RPM_ENCODER / 60) / PERIOD_SEND_DATA_HZ)
#define MAX_PULSE_ENCODER   (MAX_RPS_ENCODER * MAX_PPR_ENCODER)

// const uint32_t __t = MAX_PULSE_ENCODER;

typedef struct {
    uint32_t period;
    uint32_t pulse_enc_max;
    uint32_t pulse_enc_raw;
    uint32_t pulse_enc;
    uint32_t pulse_enc_z;
    uint32_t pulse_rpm;
    float temperature;
} DataDyno;

#define DYNO_SIZE_DATA sizeof(DataDyno)
#endif

#ifdef DYNO_IMPLEMENTATION
inline static DataDyno datadyno_create() {
    return (DataDyno){
        .period = 0,
        .pulse_enc_max = MAX_PPR_ENCODER,
        .pulse_enc_raw = 0,
        .pulse_enc = 0,
        .pulse_enc_z = 0,
        .pulse_rpm = 0,
        .temperature = 0.0,
    };
}
inline static void datadyno_reset(DataDyno *d) {
    d->period = 0;
    d->pulse_enc_max = MAX_PPR_ENCODER;
    d->pulse_enc_raw = 0;
    d->pulse_enc = 0;
    d->pulse_enc_z = 0;
    d->pulse_rpm = 0;
    d->temperature = 0.0;
}
#endif
