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
#define K3_Pin GPIO_PIN_3
#define K3_GPIO_Port GPIOE
#define K4_Pin GPIO_PIN_4
#define K4_GPIO_Port GPIOE
#define ADC4_Pin GPIO_PIN_0
#define ADC4_GPIO_Port GPIOA
#define ADC1_Pin GPIO_PIN_2
#define ADC1_GPIO_Port GPIOA
#define ADC2_Pin GPIO_PIN_3
#define ADC2_GPIO_Port GPIOA
#define ADC3_Pin GPIO_PIN_4
#define ADC3_GPIO_Port GPIOA
#define ADC5_Pin GPIO_PIN_5
#define ADC5_GPIO_Port GPIOC
#define K1_Pin GPIO_PIN_3
#define K1_GPIO_Port GPIOD
#define K2_Pin GPIO_PIN_4
#define K2_GPIO_Port GPIOD
#define OLED_GPIO1_Pin GPIO_PIN_8
#define OLED_GPIO1_GPIO_Port GPIOB
#define OLED_GPIO2_Pin GPIO_PIN_9
#define OLED_GPIO2_GPIO_Port GPIOB
#define OLED_GPIO3_Pin GPIO_PIN_0
#define OLED_GPIO3_GPIO_Port GPIOE
#define OLED_GPIO4_Pin GPIO_PIN_1
#define OLED_GPIO4_GPIO_Port GPIOE

#define DIR1_Pin GPIO_PIN_7
#define DIR1_GPIO_Port GPIOE
#define DIR2_Pin GPIO_PIN_8
#define DIR2_GPIO_Port GPIOE
#define DIR3_Pin GPIO_PIN_10
#define DIR3_GPIO_Port GPIOE
#define DIR4_Pin GPIO_PIN_12
#define DIR4_GPIO_Port GPIOE
#define DIR5_Pin GPIO_PIN_6
#define DIR5_GPIO_Port GPIOA
#define DIR6_Pin GPIO_PIN_8
#define DIR6_GPIO_Port GPIOA
#define DIR7_Pin GPIO_PIN_15
#define DIR7_GPIO_Port GPIOE

/* SPI display control lines. */
#define LCD_RST_Pin OLED_GPIO1_Pin
#define LCD_RST_GPIO_Port OLED_GPIO1_GPIO_Port
#define LCD_DC_Pin OLED_GPIO2_Pin
#define LCD_DC_GPIO_Port OLED_GPIO2_GPIO_Port
#define LCD_CS_Pin OLED_GPIO3_Pin
#define LCD_CS_GPIO_Port OLED_GPIO3_GPIO_Port
#define LCD_BLK_Pin OLED_GPIO4_Pin
#define LCD_BLK_GPIO_Port OLED_GPIO4_GPIO_Port

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
