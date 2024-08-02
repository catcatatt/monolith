/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
extern uint8_t rtc[25];

#ifdef ENABLE_SERIAL
extern uint32_t serial_flag;
extern ring_buffer_t SERIAL_BUFFER;
extern uint8_t SERIAL_BUFFER_ARR[1 << 14];

uint8_t payload[sizeof(LOG) + 2] = { 0x05, 0x12, };
#endif

#ifdef ENABLE_MONITOR_GPS
extern uint32_t gps_flag;
extern uint8_t gps_data[1 << 7];
#endif

// RTC sync rx event
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    // checksum verification
    uint32_t checksum = 0;

    for (int i = 0; i < 19; i++) {
      checksum += rtc[i];
    }

    checksum &= 0xff;

    if (rtc[19] == (uint8_t)checksum) {
      // transmit ACK; tx complete interrupt will be ignored
      HAL_UART_Transmit_IT(UART_DEBUG, (uint8_t *)"ACK", 3);

      RTC_FIX(RTC_UART);

      // re-enable RTC fix message
      HAL_UART_Receive_IT(UART_DEBUG, rtc, 20);
    }
  }
}

// serial tx event
#ifdef ENABLE_SERIAL
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
//  if (huart->Instance == USART6) {
//    if (ring_buffer_is_empty(&SERIAL_BUFFER)) {
//      serial_flag &= ~(1 << SERIAL_BUFFER_REMAIN);
//      serial_flag &= ~(1 << SERIAL_BUFFER_TRANSMIT);
//    }
//    else {
//      ring_buffer_dequeue_arr(&SERIAL_BUFFER, (char *)(payload + 2), sizeof(LOG));
//      HAL_UART_Transmit_IT(UART_SERIAL, payload, sizeof(LOG) + 2);
//    }
//  }
//
//  return;
//}

int SERIAL_SETUP(void) {
  ring_buffer_init(&SERIAL_BUFFER, (char *)SERIAL_BUFFER_ARR, sizeof(SERIAL_BUFFER_ARR));

  return SYS_OK;
}

//void SERIAL_TRANSMIT_LOG(void) {
//  if (serial_flag & (1 << SERIAL_BUFFER_REMAIN) && !(serial_flag & (1 << SERIAL_BUFFER_TRANSMIT))) {
//    serial_flag |= 1 << SERIAL_BUFFER_TRANSMIT;
//
//    ring_buffer_dequeue_arr(&SERIAL_BUFFER, (char *)(payload + 2), sizeof(LOG));
//    HAL_UART_Transmit_IT(UART_SERIAL, payload, sizeof(LOG) + 2);
//  }
//}
#endif
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART3 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}
/* USART6 init function */



void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = USART_DEBUG_TX_Pin|USART_DEBUG_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = USART_GPS_TX_Pin|USART_GPS_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART3 DMA Init */
    /* USART3_RX Init */
    hdma_usart2_rx.Instance = DMA1_Stream1;
    hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, USART_DEBUG_TX_Pin|USART_DEBUG_RX_Pin);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PD5    ------> USART2_TX
    PD6     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOD, USART_GPS_TX_Pin|USART_GPS_RX_Pin);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
 
}

/* USER CODE BEGIN 1 */
#ifdef ENABLE_MONITOR_GPS
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart->Instance == USART2) {
    gps_flag = true; // mark GPS updated

    // re-enable DMA to receive UART data until line idle
    HAL_UARTEx_ReceiveToIdle_DMA(UART_GPS, gps_data, 1 << 7);

    // disable Half Transfer interrupt
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
  }
}

int GPS_SETUP(void) {
  // set module power full
  const uint8_t GPS_PMS_FULL[] = { 0xB5, 0x62, 0x06, 0x86, 0x00, 0x00, 0x8C, 0xAA };

  HAL_UART_Transmit(UART_GPS, GPS_PMS_FULL, sizeof(GPS_PMS_FULL), 100);
  HAL_Delay(50); // wait enough time rather than check ACK message

  // disable unnecessary NMEA messages
  const uint8_t GPS_DISABLE_NMEA_GxGGA[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24 };
  const uint8_t GPS_DISABLE_NMEA_GxGLL[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B };
  const uint8_t GPS_DISABLE_NMEA_GxGSA[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32 };
  const uint8_t GPS_DISABLE_NMEA_GxGSV[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39 };
  const uint8_t GPS_DISABLE_NMEA_GxVTG[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47 };

  HAL_UART_Transmit(UART_GPS, GPS_DISABLE_NMEA_GxGGA, sizeof(GPS_DISABLE_NMEA_GxGGA), 100);
  HAL_Delay(50);
  HAL_UART_Transmit(UART_GPS, GPS_DISABLE_NMEA_GxGLL, sizeof(GPS_DISABLE_NMEA_GxGLL), 100);
  HAL_Delay(50);
  HAL_UART_Transmit(UART_GPS, GPS_DISABLE_NMEA_GxGSA, sizeof(GPS_DISABLE_NMEA_GxGSA), 100);
  HAL_Delay(50);
  HAL_UART_Transmit(UART_GPS, GPS_DISABLE_NMEA_GxGSV, sizeof(GPS_DISABLE_NMEA_GxGSV), 100);
  HAL_Delay(50);
  HAL_UART_Transmit(UART_GPS, GPS_DISABLE_NMEA_GxVTG, sizeof(GPS_DISABLE_NMEA_GxVTG), 100);
  HAL_Delay(50);

  // disable GPTXT messages
  const uint8_t GPS_DISABLE_NMEA_INFO[] = { 0xB5, 0x62, 0x06, 0x02, 0x0A, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87, 0x9A, 0x77 };

  HAL_UART_Transmit(UART_GPS, GPS_DISABLE_NMEA_INFO, sizeof(GPS_DISABLE_NMEA_INFO), 100);
  HAL_Delay(50);

  // set target module refresh rate to 10Hz
  // reference: u-blox 7 Receiver Description Including Protocol Specification V14, 35.14 CFG-RATE
  const uint8_t GPS_RATE_10HZ[] = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12 };

  HAL_UART_Transmit(UART_GPS, GPS_RATE_10HZ, sizeof(GPS_RATE_10HZ), 100);
  HAL_Delay(50);

  // set target module baud rate to 115200bps
  // reference: u-blox 7 Receiver Description Including Protocol Specification V14, 35.13 CFG-PRT
  const uint8_t GPS_BAUD_115200[] = { 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x7E };

  HAL_UART_Transmit(UART_GPS, GPS_BAUD_115200, sizeof(GPS_BAUD_115200), 100);
  HAL_Delay(50);

  // match our baud rate to 115200bps
  USART2->BRR = HAL_RCC_GetPCLK1Freq() / 115200;

  // receive UART data until line idle
  HAL_UARTEx_ReceiveToIdle_DMA(UART_GPS, gps_data, 1 << 7);

  // disable Half Transfer interrupt
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

  return SYS_OK;
}
#endif
/* USER CODE END 1 */
