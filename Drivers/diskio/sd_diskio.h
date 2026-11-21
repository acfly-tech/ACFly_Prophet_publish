/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    sd_diskio.h
 * @brief   Header for sd_diskio.c module
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Note: code generation based on sd_diskio_dma_rtos_template.h */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SD_DISKIO_H
#define __SD_DISKIO_H

/* USER CODE BEGIN firstSection */
/* can be used to modify / undefine following code or add new definitions */
/* USER CODE END firstSection */

/* Includes ------------------------------------------------------------------*/
#include "bsp_driver_sd.h"
#include "diskio.h"
#include "ff_gen_drv.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

extern const Diskio_drvTypeDef SD_Driver;

/* USER CODE BEGIN lastSection */
/* can be used to modify / undefine previous code or add new definitions */
/* USER CODE END lastSection */

/* Definitions of physical drive number for each drive */
#define DEV_SD_INT (1U) /* Example: Map SD Disk to physical drive 4 */
#define DEV_SD_USR (2U) /* Example: Map SD Disk to physical drive 5 */
#define DEV_SD_EXT (3U) /* Example: Map SD Disk to physical drive 6 */
#define DEV_SD (0U)     /* Example: Map SD Disk to physical drive 3 */

#define INTERNAL_SD_SIZE 128 * 1024 * 1024
#define USER_SD_SIZE 512 * 1024 * 1024

DSTATUS SD_initialize(BYTE pdrv);
DSTATUS SD_status(BYTE pdrv);
DRESULT SD_erase(BYTE pdrv, DWORD sector, UINT count);
DRESULT SD_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
DRESULT SD_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
DRESULT SD_ioctl(BYTE pdrv, BYTE cmd, void *buff);

#endif /* __SD_DISKIO_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
