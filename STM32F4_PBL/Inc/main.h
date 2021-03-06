/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BRIDGE_B1_Pin GPIO_PIN_3
#define BRIDGE_B1_GPIO_Port GPIOF
#define BRIDGE_A2_Pin GPIO_PIN_5
#define BRIDGE_A2_GPIO_Port GPIOF
#define BRIDGE_A1_Pin GPIO_PIN_10
#define BRIDGE_A1_GPIO_Port GPIOF
#define BRIDGE_B2_Pin GPIO_PIN_0
#define BRIDGE_B2_GPIO_Port GPIOC
#define UART_MLIDAR_S_TX_Pin GPIO_PIN_10
#define UART_MLIDAR_S_TX_GPIO_Port GPIOB
#define UART_MLIDAR_S_RX_Pin GPIO_PIN_11
#define UART_MLIDAR_S_RX_GPIO_Port GPIOB
#define RED_LED_Pin GPIO_PIN_14
#define RED_LED_GPIO_Port GPIOB
#define UART_RFID_TX_Pin GPIO_PIN_10
#define UART_RFID_TX_GPIO_Port GPIOC
#define UART_RFID_RX_Pin GPIO_PIN_11
#define UART_RFID_RX_GPIO_Port GPIOC
#define ULTRASOUND_TX_Pin GPIO_PIN_12
#define ULTRASOUND_TX_GPIO_Port GPIOC
#define ULTRASOUND_RX_Pin GPIO_PIN_2
#define ULTRASOUND_RX_GPIO_Port GPIOD
#define UART_GPS_TX_Pin GPIO_PIN_5
#define UART_GPS_TX_GPIO_Port GPIOD
#define UART_GPS_RX_Pin GPIO_PIN_6
#define UART_GPS_RX_GPIO_Port GPIOD
#define GPIO_ELECTROMAGNET_Pin GPIO_PIN_7
#define GPIO_ELECTROMAGNET_GPIO_Port GPIOD
#define UART_MLIDAR_RX_Pin GPIO_PIN_9
#define UART_MLIDAR_RX_GPIO_Port GPIOG
#define UART_MLIDAR_TX_Pin GPIO_PIN_14
#define UART_MLIDAR_TX_GPIO_Port GPIOG
#define BLUE_LED_Pin GPIO_PIN_7
#define BLUE_LED_GPIO_Port GPIOB
#define SPI1_SS_Pin GPIO_PIN_1
#define SPI1_SS_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
