/**
 **************************************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 **************************************************************************************************
 *
 * Copyright (c) 2023 Rizal Achmad Pahlevi <echo 'cml6YWwuYWhtYWRwQGdtYWlsLmNvbQo=' | base64 -d>
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 **************************************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include <main.h>
#include <usb_device.h>
#include <usbd_cdc_if.h>

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi_max6675;

TIM_HandleTypeDef htim_encoder;
TIM_HandleTypeDef htim_send_timer;
TIM_HandleTypeDef htim_rpm;

#ifdef WITH_PHASE_Z
TIM_HandleTypeDef htim_phase_z;
#endif

volatile uint8_t v_usb_cmd_bit = USBCMD_NOOP;
volatile uint8_t v_usb_send_bit = SENDBIT_DISABLE;
uint8_t start_bit = DYNO_STARTED;

#define BUFFER_DATA_SIZE DYNO_SIZE_DATA + 1
static uint8_t BUFFER_DATA[BUFFER_DATA_SIZE] = {[DYNO_SIZE_DATA] = '\n'};
static DataDyno dyno = DATA_DYNO_INIT;

static uint32_t last_time = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIMEncoder_Init(void);
static void MX_TIMRpm_Init(void);
static void MX_TIMSend_Init(void);
static void MX_SPIMAX6675_Init(void);
#ifdef WITH_PHASE_Z
static void MX_TIMPhaseZ_Init(void);
#endif
static void MX_NVIC_Init(void);
static void MX_NVIC_DeInit(void);

static void MX_Start(void);
static void MX_Stop(void);

static HAL_StatusTypeDef MAX6675_Temp(float *data_temp);

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();
    /* Configure the system clock */
    SystemClock_Config();
    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_TIMEncoder_Init();
    MX_TIMRpm_Init();
    MX_TIMSend_Init();
    MX_SPIMAX6675_Init();
#ifdef WITH_PHASE_Z
    MX_TIMPhaseZ_Init();
#endif
    MX_USB_DEVICE_Init();

    /* Infinite loop */
    while (1) {
        if (v_usb_send_bit == SENDBIT_ENABLE && start_bit == DYNO_STARTED) {
            uint32_t now = HAL_GetTick();

            dyno.pulse_enc_raw = GET_COUNTER(htim_encoder);
            dyno.pulse_rpm = GET_COUNTER_CH(htim_rpm, CH_RPM);
#ifdef WITH_PHASE_Z
            dyno.pulse_enc_z = GET_COUNTER_CH(htim_phase_z, CH_PHASE_Z);
#endif
            dyno.pulse_enc = dyno.pulse_enc_raw * 4;
            dyno.period = now - last_time;
            dyno.time = now;
            last_time = now;
            MAX6675_Temp(&dyno.temperature);

            memcpy(&BUFFER_DATA[0], &dyno, DYNO_SIZE_DATA);
            CDC_Transmit_FS(BUFFER_DATA, BUFFER_DATA_SIZE);

            // reset the counter
            RESET_COUNTER(htim_encoder);
            RESET_COUNTER_CH(htim_rpm, CH_RPM);
#ifdef WITH_PHASE_Z
            RESET_COUNTER_CH(htim_phase_z, CH_PHASE_Z);
#endif
            // reset sendbit
            v_usb_send_bit = SENDBIT_DISABLE;
            // set low led indicator 2
            GPIOA->BSRR = LED_INDICATOR_2_Pin;
        }

        if (v_usb_cmd_bit != USBCMD_NOOP) {
            switch (v_usb_cmd_bit) {
            case USBCMD_START:
                MX_Start();
                break;
            case USBCMD_STOP:
                MX_Stop();
                break;
            case USBCMD_RESTART:
                HAL_NVIC_SystemReset();
                break;
            default:
                break;
            }
            v_usb_cmd_bit = USBCMD_NOOP;
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        // set high led indicator 2
        GPIOA->BSRR = (uint32_t)LED_INDICATOR_2_Pin << 16U;
        v_usb_send_bit = SENDBIT_ENABLE;
    }
}

static void MX_Start(void) {
    MX_NVIC_Init();

    if (HAL_OK != HAL_TIM_Encoder_Start(&htim_encoder, TIM_CHANNEL_ENC)) {
        // assert_param() Error_Send("Failed to start `HAL_TIM_Encoder_Start`");
    }
    HAL_TIM_Base_Start(&htim_send_timer);
    HAL_TIM_IC_Start(&htim_rpm, TIM_CHANNEL_RPM);

#ifdef WITH_PHASE_Z
    HAL_TIM_IC_Start(&htim_phase_z, TIM_CHANNEL_PHASE_Z);
#endif

    start_bit = DYNO_STARTED;
    last_time = HAL_GetTick();
    HAL_GPIO_WritePin(GPIOA, LED_INDICATOR_1_Pin, GPIO_PIN_SET);
}

static void MX_Stop(void) {
    MX_NVIC_DeInit();
    HAL_TIM_Encoder_Stop(&htim_encoder, TIM_CHANNEL_ENC);
    HAL_TIM_Base_Stop(&htim_send_timer);
    HAL_TIM_IC_Stop(&htim_rpm, TIM_CHANNEL_RPM);

#ifdef WITH_PHASE_Z
    HAL_TIM_IC_Stop(&htim_phase_z, TIM_CHANNEL_PHASE_Z);
#endif

    // reset the data
    dyno = (DataDyno)DATA_DYNO_INIT;

    start_bit = DYNO_STOPPED;
    HAL_GPIO_WritePin(GPIOA, LED_INDICATOR_1_Pin, GPIO_PIN_RESET);
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void) {
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    /* TIM3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    /* TIM4_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    /* TIM5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
    /* SPI1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
}

static void MX_NVIC_DeInit(void) {
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
    HAL_NVIC_DisableIRQ(TIM5_IRQn);
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPIMAX6675_Init(void) {
    /* SPI1 parameter configuration*/
    hspi_max6675.Instance = SPI1;
    hspi_max6675.Init.Mode = SPI_MODE_MASTER;
    hspi_max6675.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
    hspi_max6675.Init.DataSize = SPI_DATASIZE_16BIT;
    hspi_max6675.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi_max6675.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi_max6675.Init.NSS = SPI_NSS_SOFT;
    hspi_max6675.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi_max6675.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi_max6675.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi_max6675.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi_max6675.Init.CRCPolynomial = 9;
    if (HAL_SPI_Init(&hspi_max6675) != HAL_OK) {
        Error_Handler();
    }
}

static HAL_StatusTypeDef MAX6675_Temp(float *data_temp) {
    static uint8_t data_RX[2] = {0};
    HAL_StatusTypeDef ret = HAL_OK;

    // Waits for Chip Ready(according to Datasheet, the max time for conversion is 220ms)
    MAX6675_CS_GPIO_Port->BSRR = MAX6675_CS_Pin; // reset
    ret = HAL_SPI_Receive(&hspi_max6675, data_RX, 1, 220);
    MAX6675_CS_GPIO_Port->BSRR = (uint32_t)MAX6675_CS_Pin << 16U; // set

    // convert [uint8_t; 2] to  uint16_t value
    uint16_t raw = (uint16_t)(((uint16_t)data_RX[0] << 8) | data_RX[1]);
    // State of Connecting
    if (((raw >> 2) & 0x01) == 0 && ret == HAL_OK) {
        *data_temp = (raw >> 3) * 0.25;
        return HAL_OK;
    }
    return ret;
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIMEncoder_Init(void) {

    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim_encoder.Instance = TIM2;
    htim_encoder.Init.Prescaler = 0;
    htim_encoder.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim_encoder.Init.Period = 4294967295;
    htim_encoder.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim_encoder.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV4;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV4;
    sConfig.IC2Filter = 0;
    if (HAL_TIM_Encoder_Init(&htim_encoder, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim_encoder, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIMSend_Init(void) {

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim_send_timer.Instance = TIM3;
    htim_send_timer.Init.Prescaler = 9599;
    htim_send_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim_send_timer.Init.Period = 1999;
    htim_send_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim_send_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim_send_timer) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim_send_timer, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim_send_timer, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIMRpm_Init(void) {

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};

    htim_rpm.Instance = TIM4;
    htim_rpm.Init.Prescaler = 0;
    htim_rpm.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim_rpm.Init.Period = 65535;
    htim_rpm.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim_rpm.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_IC_Init(&htim_rpm) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim_rpm, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim_rpm, &sConfigIC, TIM_CHANNEL_RPM) != HAL_OK) {
        Error_Handler();
    }
}

#ifdef WITH_PHASE_Z
/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIMPhaseZ_Init(void) {

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};

    htim_phase_z.Instance = TIM5;
    htim_phase_z.Init.Prescaler = 0;
    htim_phase_z.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim_phase_z.Init.Period = 4294967295;
    htim_phase_z.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim_phase_z.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_IC_Init(&htim_phase_z) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim_phase_z, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim_phase_z, &sConfigIC, TIM_CHANNEL_PHASE_Z) != HAL_OK) {
        Error_Handler();
    }
}
#endif

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, LED_INDICATOR_1_Pin | LED_INDICATOR_2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(MAX6675_CS_GPIO_Port, MAX6675_CS_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : PC13 PC14 PC15 */
    GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : LED_INDICATOR_1_Pin LED_INDICATOR_2_Pin */
    GPIO_InitStruct.Pin = LED_INDICATOR_1_Pin | LED_INDICATOR_2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : MAX6675_CS_Pin */
    GPIO_InitStruct.Pin = MAX6675_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MAX6675_CS_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PA7 PA8 PA9 PA10
                             PA13 PA14 PA15 */
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_13 |
                          GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PB1 PB2 PB10 PB12
                             PB13 PB14 PB15 PB3
                             PB4 PB5 PB6 PB7
                             PB8 */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_12 | GPIO_PIN_13 |
                          GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 |
                          GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 192;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    __disable_irq();
    while (1) {
    }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number, ex:
     * printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
