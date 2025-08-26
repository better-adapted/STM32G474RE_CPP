/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

#include "stm32g4xx_nucleo.h"

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
#define LASER_0_TTL_Pin GPIO_PIN_14
#define LASER_0_TTL_GPIO_Port GPIOC
#define LASER_1_TTL_Pin GPIO_PIN_15
#define LASER_1_TTL_GPIO_Port GPIOC
#define LASER_2_TTL_Pin GPIO_PIN_3
#define LASER_2_TTL_GPIO_Port GPIOC
#define LASER_3_TTL_Pin GPIO_PIN_0
#define LASER_3_TTL_GPIO_Port GPIOA
#define LASER_4_TTL_Pin GPIO_PIN_5
#define LASER_4_TTL_GPIO_Port GPIOA
#define LASER_5_TTL_Pin GPIO_PIN_6
#define LASER_5_TTL_GPIO_Port GPIOA
#define LASER_6_TTL_Pin GPIO_PIN_7
#define LASER_6_TTL_GPIO_Port GPIOA
#define LASER_7_TTL_Pin GPIO_PIN_4
#define LASER_7_TTL_GPIO_Port GPIOC
#define DIL_SWT_0_Pin GPIO_PIN_5
#define DIL_SWT_0_GPIO_Port GPIOC
#define DIL_SWT_1_Pin GPIO_PIN_0
#define DIL_SWT_1_GPIO_Port GPIOB
#define DIL_SWT_2_Pin GPIO_PIN_2
#define DIL_SWT_2_GPIO_Port GPIOB
#define DIL_SWT_3_Pin GPIO_PIN_10
#define DIL_SWT_3_GPIO_Port GPIOB
#define SENSORS_CLK_Pin GPIO_PIN_9
#define SENSORS_CLK_GPIO_Port GPIOA
#define SENSORS_ST_Pin GPIO_PIN_10
#define SENSORS_ST_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define CPU_BOOT0_Pin GPIO_PIN_8
#define CPU_BOOT0_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
