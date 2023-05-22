/**
 ******************************************************************************
 * @file           : usb_device.h
 * @version        : v1.0_Cube
 * @brief          : Header for usb_device.c file.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_DEVICE__H__
#define __USB_DEVICE__H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_def.h"

extern USBD_HandleTypeDef hUsbDeviceFS;
extern USBD_CDC_LineCodingTypeDef USBD_LC;
/** USB Device initialization function. */
void MX_USB_DEVICE_Init(void);

#endif /* __USB_DEVICE__H__ */
