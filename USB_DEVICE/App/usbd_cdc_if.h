/**
 ******************************************************************************
 * @file           : usbd_cdc_if.h
 * @version        : v1.0_Cube
 * @brief          : Header for usbd_cdc_if.c file.
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
#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"

#define BAUD(BUF) (uint32_t)(BUF[0] | (BUF[1] << 8) | (BUF[2] << 16) | (BUF[3] << 24))

/** @defgroup USBD_CDC_IF_Exported_Defines USBD_CDC_IF_Exported_Defines
 * @brief Defines.
 * @{
 */
/* Define size for the receive and transmit buffer over CDC */
#define APP_RX_DATA_SIZE 1024
#define APP_TX_DATA_SIZE (8 * 1024)

/** CDC Interface callback. */
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;
extern volatile uint8_t v_start_bit;

uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

#define CDC_IS_USB_OPENED(hUsbFs) (((hUsbFs)->dev_state == USBD_STATE_CONFIGURED) && (v_start_bit == DYNO_STARTED))
#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H__ */
