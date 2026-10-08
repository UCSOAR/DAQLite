/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   This file includes a diskio driver skeleton to be completed by the user.
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

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/*
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future.
 * Kept to ensure backward compatibility with previous CubeMx versions when
 * migrating projects.
 * User code previously added there should be copied in the new user sections before
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ff_gen_drv.h"
#include "mx66xx.hpp"
#include "user_diskio.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
	  if (pdrv != 0)
	  {
	    return STA_NOINIT;
	  }

	  // Initialize flash memory
	  // Flash is ready to use - no special initialization needed
	  // The MX66 driver handles SPI configuration

	  // Mark disk as initialized and ready
	  Stat = 0; // No errors, disk is ready
	  return Stat;
  /* USER CODE END INIT */
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
	  if (pdrv != 0)
	  {
	    return STA_NOINIT;
	  }

	  return Stat;
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */
	  if (pdrv != 0 || !buff)
	  {
	    return RES_PARERR;
	  }

	  if (Stat & STA_NOINIT)
	  {
	    return RES_NOTRDY;
	  }

	  // Check sector boundaries
	  if ((sector + count) > SECTOR_COUNT)
	  {
	    return RES_PARERR;
	  }

	  // Calculate flash address and read data
	  uint32_t flash_addr = FLASH_START_ADDR + (sector * SECTOR_SIZE);
	  uint32_t total_size = count * SECTOR_SIZE;

	  // Read directly from flash
	  MX66xx_FastReadBytes(buff, flash_addr, total_size);

	  return RES_OK;
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{
  /* USER CODE BEGIN WRITE */
  /* USER CODE HERE */
	  if (pdrv != 0 || !buff)
	  {
	    return RES_PARERR;
	  }

	  if (Stat & STA_NOINIT)
	  {
	    return RES_NOTRDY;
	  }

	  // Check sector boundaries
	  if ((sector + count) > SECTOR_COUNT)
	  {
	    return RES_PARERR;
	  }

	  // Calculate flash address
	  uint32_t flash_addr = FLASH_START_ADDR + (sector * SECTOR_SIZE);
	  uint32_t total_size = count * SECTOR_SIZE;
	  uint32_t flash_sector = flash_addr / FS_SECTOR_SIZE;  // Which 4KB sector
	  uint16_t sector_offset = flash_addr % FS_SECTOR_SIZE; // Offset within sector

	  //  // Erase all affected 4KB sectors first
	  //  uint32_t end_flash_addr = flash_addr + total_size;
	  //  uint32_t end_flash_sector = (end_flash_addr - 1) / FS_SECTOR_SIZE;
	  //  for (uint32_t s = flash_sector; s <= end_flash_sector; s++)
	  //  {
	  //    MX66xx_EraseSector(s);
	  //  }

	  // Write data to flash (page-based so addresses are explicit and can cross sectors)
	  uint32_t write_addr = flash_addr;
	  uint32_t bytes_left = total_size;
	  uint32_t buffer_index = 0;

	  while (bytes_left > 0)
	  {
	    uint32_t page = write_addr / FS_PAGE_SIZE;
	    uint32_t offset_in_page = write_addr % FS_PAGE_SIZE;
	    uint32_t chunk = FS_PAGE_SIZE - offset_in_page;
	    if (chunk > bytes_left)
	    {
	      chunk = bytes_left;
	    }

	    MX66xx_WritePage((uint8_t *)&buff[buffer_index], page, offset_in_page, chunk);

	    write_addr += chunk;
	    buffer_index += chunk;
	    bytes_left -= chunk;
	  }

	  return RES_OK;
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
	DRESULT res = RES_ERROR;

	  if (pdrv != 0)
	  {
	    return RES_PARERR;
	  }

	  if (Stat & STA_NOINIT)
	  {
	    return RES_NOTRDY;
	  }

	  switch (cmd)
	  {
	  case CTRL_SYNC:
	    // For flash, sync is instant (write operations complete immediately)
	    res = RES_OK;
	    break;

	  case GET_SECTOR_COUNT:
	    *(DWORD *)buff = SECTOR_COUNT;
	    res = RES_OK;
	    break;

	  case GET_SECTOR_SIZE:
	    *(WORD *)buff = SECTOR_SIZE;
	    res = RES_OK;
	    break;

	  case GET_BLOCK_SIZE:
	    // Return the flash erase block size in sectors (4KB sector / 512-byte sector = 8 sectors)
	    *(DWORD *)buff = (FS_SECTOR_SIZE / SECTOR_SIZE);
	    res = RES_OK;
	    break;

	  default:
	    res = RES_PARERR;
	    break;
	  }

	  return res;
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */

