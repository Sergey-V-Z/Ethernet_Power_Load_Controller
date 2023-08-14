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
#include "stm32f0xx_hal.h"

#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_crs.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_gpio.h"

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
void IN_GPIO_Init(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define A3_LED_Pin LL_GPIO_PIN_0
#define A3_LED_GPIO_Port GPIOF
#define EN3_Pin LL_GPIO_PIN_1
#define EN3_GPIO_Port GPIOF
#define EN2_Pin LL_GPIO_PIN_4
#define EN2_GPIO_Port GPIOA
#define EN1_Pin LL_GPIO_PIN_5
#define EN1_GPIO_Port GPIOA
#define A2_PWM_CH1_Pin LL_GPIO_PIN_6
#define A2_PWM_CH1_GPIO_Port GPIOA
#define A1_PWM_CH2_Pin LL_GPIO_PIN_7
#define A1_PWM_CH2_GPIO_Port GPIOA
#define A0_PWM_CH4_Pin LL_GPIO_PIN_1
#define A0_PWM_CH4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
