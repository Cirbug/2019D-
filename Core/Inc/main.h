/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define Key0_Pin GPIO_PIN_4
#define Key0_GPIO_Port GPIOE
#define Key0_EXTI_IRQn EXTI4_IRQn
#define Key_up_Pin GPIO_PIN_0
#define Key_up_GPIO_Port GPIOA
#define Key_up_EXTI_IRQn EXTI0_IRQn
#define ADC1_Us_Pin GPIO_PIN_2
#define ADC1_Us_GPIO_Port GPIOA
#define ADC1_Ui_Pin GPIO_PIN_3
#define ADC1_Ui_GPIO_Port GPIOA
#define Back_Light_Pin GPIO_PIN_15
#define Back_Light_GPIO_Port GPIOB
#define AD9954_UPD_Pin GPIO_PIN_6
#define AD9954_UPD_GPIO_Port GPIOG
#define AD9954_SDO_Pin GPIO_PIN_8
#define AD9954_SDO_GPIO_Port GPIOG
#define AD9954_CS_Pin GPIO_PIN_6
#define AD9954_CS_GPIO_Port GPIOC
#define AD9954_PS1_Pin GPIO_PIN_8
#define AD9954_PS1_GPIO_Port GPIOC
#define AD9954_IOSY_Pin GPIO_PIN_12
#define AD9954_IOSY_GPIO_Port GPIOA
#define AD9954_OSK_Pin GPIO_PIN_11
#define AD9954_OSK_GPIO_Port GPIOC
#define AD9954_PS0_Pin GPIO_PIN_12
#define AD9954_PS0_GPIO_Port GPIOC
#define AD9954_SCLK_Pin GPIO_PIN_6
#define AD9954_SCLK_GPIO_Port GPIOD
#define trigger2_Pin GPIO_PIN_13
#define trigger2_GPIO_Port GPIOG
#define AD9954_SDIO_Pin GPIO_PIN_15
#define AD9954_SDIO_GPIO_Port GPIOG
#define AD9954_PWR_Pin GPIO_PIN_3
#define AD9954_PWR_GPIO_Port GPIOB
#define trigger1_Pin GPIO_PIN_4
#define trigger1_GPIO_Port GPIOB
#define AD9954_RES_Pin GPIO_PIN_6
#define AD9954_RES_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
