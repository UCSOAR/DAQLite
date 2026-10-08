/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.h
  * @brief   This file contains the common defines and functions prototypes for
  *          the user_diskio driver.
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
#ifndef __USER_DISKIO_H
#define __USER_DISKIO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* USER CODE BEGIN 0 */

/* Includes ------------------------------------------------------------------*/
#include "mx66xx.hpp"
#include "mx66xx_qspi.hpp"
#include "mx66xxConf.hpp"

 /* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define SECTOR_SIZE 4096     // FatFS sector size, matches flash erase granularity
#define SECTOR_COUNT 8192    // 32MB usable (4096 * 8192 = 33,554,432 bytes)
#define FLASH_START_ADDR 0x0 // Start address on flash


 /* Exported functions ------------------------------------------------------- */
extern Diskio_drvTypeDef  USER_Driver;

/* USER CODE END 0 */

#ifdef __cplusplus
}
#endif

#endif /* __USER_DISKIO_H */
