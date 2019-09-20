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
#define ENCODER_L_Pin GPIO_PIN_12
#define ENCODER_L_GPIO_Port GPIOF
#define ENCODER_L_EXTI_IRQn EXTI15_10_IRQn
#define ENCODER_R_Pin GPIO_PIN_13
#define ENCODER_R_GPIO_Port GPIOF
#define ENCODER_R_EXTI_IRQn EXTI15_10_IRQn
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
