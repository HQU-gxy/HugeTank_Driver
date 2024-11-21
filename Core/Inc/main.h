/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOA
#define BTN_Pin GPIO_PIN_4
#define BTN_GPIO_Port GPIOA
#define GYRO_CS_Pin GPIO_PIN_0
#define GYRO_CS_GPIO_Port GPIOB
#define ACC_CS_Pin GPIO_PIN_1
#define ACC_CS_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_12
#define LCD_CS_GPIO_Port GPIOB
#define LCD_BL_Pin GPIO_PIN_6
#define LCD_BL_GPIO_Port GPIOC
#define LCD_DC_Pin GPIO_PIN_8
#define LCD_DC_GPIO_Port GPIOA
#define ESC2_ALM_Pin GPIO_PIN_9
#define ESC2_ALM_GPIO_Port GPIOA
#define ESC2_EN_Pin GPIO_PIN_10
#define ESC2_EN_GPIO_Port GPIOA
#define ESC2_DIR_Pin GPIO_PIN_11
#define ESC2_DIR_GPIO_Port GPIOA
#define ESC2_FG_Pin GPIO_PIN_12
#define ESC2_FG_GPIO_Port GPIOA
#define ESC2_PWM_Pin GPIO_PIN_15
#define ESC2_PWM_GPIO_Port GPIOA
#define ESC1_ALM_Pin GPIO_PIN_10
#define ESC1_ALM_GPIO_Port GPIOC
#define ESC1_EN_Pin GPIO_PIN_11
#define ESC1_EN_GPIO_Port GPIOC
#define ESC1_DIR_Pin GPIO_PIN_3
#define ESC1_DIR_GPIO_Port GPIOB
#define ESC1_FG_Pin GPIO_PIN_4
#define ESC1_FG_GPIO_Port GPIOB
#define ESC1_PWM_Pin GPIO_PIN_5
#define ESC1_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
void app_main();
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
