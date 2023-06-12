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
#define D5_Pin GPIO_PIN_2
#define D5_GPIO_Port GPIOE
#define EN_Pin GPIO_PIN_3
#define EN_GPIO_Port GPIOE
#define D6_Pin GPIO_PIN_4
#define D6_GPIO_Port GPIOE
#define LCD_BTN_Pin GPIO_PIN_5
#define LCD_BTN_GPIO_Port GPIOE
#define LCD_BTN_EXTI_IRQn EXTI9_5_IRQn
#define D7_Pin GPIO_PIN_6
#define D7_GPIO_Port GPIOE
#define Choice_1_Pin GPIO_PIN_0
#define Choice_1_GPIO_Port GPIOC
#define Choice_1_EXTI_IRQn EXTI0_IRQn
#define Choice_2_Pin GPIO_PIN_1
#define Choice_2_GPIO_Port GPIOC
#define Choice_2_EXTI_IRQn EXTI1_IRQn
#define Choice_3_Pin GPIO_PIN_2
#define Choice_3_GPIO_Port GPIOC
#define Choice_3_EXTI_IRQn EXTI2_IRQn
#define Buzzer_Pin GPIO_PIN_2
#define Buzzer_GPIO_Port GPIOA
#define Heater_Pin GPIO_PIN_4
#define Heater_GPIO_Port GPIOA
#define test_led_Pin GPIO_PIN_13
#define test_led_GPIO_Port GPIOD
#define D4_Pin GPIO_PIN_0
#define D4_GPIO_Port GPIOE
#define RS_Pin GPIO_PIN_1
#define RS_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
