/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AIN0_Pin GPIO_PIN_0
#define AIN0_GPIO_Port GPIOC
#define AIN1_Pin GPIO_PIN_1
#define AIN1_GPIO_Port GPIOC
#define AIN2_Pin GPIO_PIN_2
#define AIN2_GPIO_Port GPIOC
#define AIN3_Pin GPIO_PIN_3
#define AIN3_GPIO_Port GPIOC
#define IC0_Pin GPIO_PIN_0
#define IC0_GPIO_Port GPIOA
#define IC1_Pin GPIO_PIN_1
#define IC1_GPIO_Port GPIOA
#define IC2_Pin GPIO_PIN_2
#define IC2_GPIO_Port GPIOA
#define IC3_Pin GPIO_PIN_3
#define IC3_GPIO_Port GPIOA
#define LED_ONBOARD_0_Pin GPIO_PIN_6
#define LED_ONBOARD_0_GPIO_Port GPIOA
#define LED_ONBOARD_1_Pin GPIO_PIN_7
#define LED_ONBOARD_1_GPIO_Port GPIOA
#define LED_CUSTOM_0_Pin GPIO_PIN_9
#define LED_CUSTOM_0_GPIO_Port GPIOE
#define LED_CUSTOM_1_Pin GPIO_PIN_10
#define LED_CUSTOM_1_GPIO_Port GPIOE
#define LED_ERR_Pin GPIO_PIN_11
#define LED_ERR_GPIO_Port GPIOE
#define LED_HEARTBEAT_Pin GPIO_PIN_12
#define LED_HEARTBEAT_GPIO_Port GPIOE
#define LED_SD_Pin GPIO_PIN_13
#define LED_SD_GPIO_Port GPIOE
#define LED_CAN_Pin GPIO_PIN_14
#define LED_CAN_GPIO_Port GPIOE
#define LED_ESP_Pin GPIO_PIN_15
#define LED_ESP_GPIO_Port GPIOE
#define I2C_LOG_SCL_Pin GPIO_PIN_10
#define I2C_LOG_SCL_GPIO_Port GPIOB
#define I2C_LOG_SDA_Pin GPIO_PIN_11
#define I2C_LOG_SDA_GPIO_Port GPIOB
#define SDIO_DETECT_Pin GPIO_PIN_12
#define SDIO_DETECT_GPIO_Port GPIOB
#define DIN0_Pin GPIO_PIN_8
#define DIN0_GPIO_Port GPIOD
#define DIN1_Pin GPIO_PIN_9
#define DIN1_GPIO_Port GPIOD
#define DIN2_Pin GPIO_PIN_10
#define DIN2_GPIO_Port GPIOD
#define DIN3_Pin GPIO_PIN_11
#define DIN3_GPIO_Port GPIOD
#define DIN4_Pin GPIO_PIN_12
#define DIN4_GPIO_Port GPIOD
#define DIN5_Pin GPIO_PIN_13
#define DIN5_GPIO_Port GPIOD
#define DIN6_Pin GPIO_PIN_14
#define DIN6_GPIO_Port GPIOD
#define DIN7_Pin GPIO_PIN_15
#define DIN7_GPIO_Port GPIOD
#define USART_LOG_TX_Pin GPIO_PIN_6
#define USART_LOG_TX_GPIO_Port GPIOC
#define USART_LOG_RX_Pin GPIO_PIN_7
#define USART_LOG_RX_GPIO_Port GPIOC
#define I2C_ACC_SDA_Pin GPIO_PIN_9
#define I2C_ACC_SDA_GPIO_Port GPIOC
#define I2C_ACC_SCL_Pin GPIO_PIN_8
#define I2C_ACC_SCL_GPIO_Port GPIOA
#define USART_DEBUG_TX_Pin GPIO_PIN_9
#define USART_DEBUG_TX_GPIO_Port GPIOA
#define USART_DEBUG_RX_Pin GPIO_PIN_10
#define USART_DEBUG_RX_GPIO_Port GPIOA
#define USART_GPS_TX_Pin GPIO_PIN_10
#define USART_GPS_TX_GPIO_Port GPIOC
#define USART_GPS_RX_Pin GPIO_PIN_11
#define USART_GPS_RX_GPIO_Port GPIOC
#define USART_ESP_TX_Pin GPIO_PIN_5
#define USART_ESP_TX_GPIO_Port GPIOD
#define USART_ESP_RX_Pin GPIO_PIN_6
#define USART_ESP_RX_GPIO_Port GPIOD
#define I2C_ESP_SCL_Pin GPIO_PIN_6
#define I2C_ESP_SCL_GPIO_Port GPIOB
#define I2C_ESP_SDA_Pin GPIO_PIN_7
#define I2C_ESP_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
