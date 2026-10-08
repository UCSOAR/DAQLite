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
#include "stm32h7xx_hal.h"

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
#define BARO07_CS_Pin GPIO_PIN_13
#define BARO07_CS_GPIO_Port GPIOC
#define MAG_CS_Pin GPIO_PIN_2
#define MAG_CS_GPIO_Port GPIOA
#define BARO11_CS_Pin GPIO_PIN_3
#define BARO11_CS_GPIO_Port GPIOA
#define IMU32_INT_Pin GPIO_PIN_4
#define IMU32_INT_GPIO_Port GPIOA
#define IMU32_CS_Pin GPIO_PIN_5
#define IMU32_CS_GPIO_Port GPIOC
#define SPI_FLASH_CS_Pin GPIO_PIN_2
#define SPI_FLASH_CS_GPIO_Port GPIOB
#define Flash_Reset_Pin GPIO_PIN_14
#define Flash_Reset_GPIO_Port GPIOD
#define GPS_CS_Pin GPIO_PIN_9
#define GPS_CS_GPIO_Port GPIOA
#define IMU16_INT_Pin GPIO_PIN_15
#define IMU16_INT_GPIO_Port GPIOA
#define IMU16_CS_Pin GPIO_PIN_0
#define IMU16_CS_GPIO_Port GPIOD
#define BARO07_CSB4_Pin GPIO_PIN_4
#define BARO07_CSB4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
