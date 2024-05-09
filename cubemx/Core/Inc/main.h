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
#include "stm32f1xx_hal.h"

#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"

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
#define ENC_SW_Pin LL_GPIO_PIN_13
#define ENC_SW_GPIO_Port GPIOC
#define BTN_1_Pin LL_GPIO_PIN_14
#define BTN_1_GPIO_Port GPIOC
#define BTN_1_EXTI_IRQn EXTI15_10_IRQn
#define BTN_2_Pin LL_GPIO_PIN_15
#define BTN_2_GPIO_Port GPIOC
#define BTN_2_EXTI_IRQn EXTI15_10_IRQn
#define CH1_HEATER_Pin LL_GPIO_PIN_0
#define CH1_HEATER_GPIO_Port GPIOA
#define CH2_HEATER_Pin LL_GPIO_PIN_1
#define CH2_HEATER_GPIO_Port GPIOA
#define CH1_THERMO_Pin LL_GPIO_PIN_2
#define CH1_THERMO_GPIO_Port GPIOA
#define CH1_CURRENT_Pin LL_GPIO_PIN_3
#define CH1_CURRENT_GPIO_Port GPIOA
#define CH2_THERMO_Pin LL_GPIO_PIN_4
#define CH2_THERMO_GPIO_Port GPIOA
#define CH2_CURRENT_Pin LL_GPIO_PIN_5
#define CH2_CURRENT_GPIO_Port GPIOA
#define CH1_TILT_Pin LL_GPIO_PIN_6
#define CH1_TILT_GPIO_Port GPIOA
#define CH2_TILT_Pin LL_GPIO_PIN_7
#define CH2_TILT_GPIO_Port GPIOA
#define LCD_CS1_Pin LL_GPIO_PIN_0
#define LCD_CS1_GPIO_Port GPIOB
#define LCD_CS2_Pin LL_GPIO_PIN_1
#define LCD_CS2_GPIO_Port GPIOB
#define LCD_RST_Pin LL_GPIO_PIN_2
#define LCD_RST_GPIO_Port GPIOB
#define LCD_D2_Pin LL_GPIO_PIN_10
#define LCD_D2_GPIO_Port GPIOB
#define LCD_D3_Pin LL_GPIO_PIN_11
#define LCD_D3_GPIO_Port GPIOB
#define LCD_D4_Pin LL_GPIO_PIN_12
#define LCD_D4_GPIO_Port GPIOB
#define LCD_D5_Pin LL_GPIO_PIN_13
#define LCD_D5_GPIO_Port GPIOB
#define LCD_D6_Pin LL_GPIO_PIN_14
#define LCD_D6_GPIO_Port GPIOB
#define LCD_D7_Pin LL_GPIO_PIN_15
#define LCD_D7_GPIO_Port GPIOB
#define LCD_E_Pin LL_GPIO_PIN_8
#define LCD_E_GPIO_Port GPIOA
#define LCD_RW_Pin LL_GPIO_PIN_9
#define LCD_RW_GPIO_Port GPIOA
#define LCD_DI_Pin LL_GPIO_PIN_10
#define LCD_DI_GPIO_Port GPIOA
#define BUZZER_Pin LL_GPIO_PIN_12
#define BUZZER_GPIO_Port GPIOA
#define LCD_D0_Pin LL_GPIO_PIN_8
#define LCD_D0_GPIO_Port GPIOB
#define LCD_D1_Pin LL_GPIO_PIN_9
#define LCD_D1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
