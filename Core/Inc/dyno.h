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

#ifndef MAX_PULSE_RER_ROTATION
#define MAX_PULSE_RER_ROTATION 360 // default max pulse
#endif

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
#endif

#ifdef DYNO_IMPLEMENTATION
inline static DataDyno datadyno_create_static() {
    return (DataDyno){
        .time = 0,
        .period = 0,
        .pulse_encoder_max = MAX_PULSE_RER_ROTATION,
        .pulse_enc = 0,
        .pulse_enc_z = 0,
        .pulse_rpm = 0,
        .temperature = 0.0,
    };
}
inline static void datadyno_reset(DataDyno *d) {
    d->time = 0;
    d->period = 0;
    d->pulse_encoder_max = MAX_PULSE_RER_ROTATION;
    d->pulse_enc = 0;
    d->pulse_enc_z = 0;
    d->pulse_rpm = 0;
    d->temperature = 0.0;
}
#endif
