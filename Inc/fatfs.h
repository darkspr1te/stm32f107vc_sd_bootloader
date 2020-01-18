/**
  ******************************************************************************
  * @file   fatfs.h
  * @brief  Header for fatfs applications
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __fatfs_H
#define __fatfs_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "ff.h"
#include "ff_gen_drv.h"
//#include "user_diskio.h" /* defines USER_Driver as external */
#include "spisd_diskio.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

//extern uint8_t retUSER; /* Return value for USER */
//extern char USERPath[4]; /* USER logical drive path */
//extern FATFS USERFatFS; /* File system object for USER logical drive */
//extern FIL USERFile; /* File object for USER */

extern char SPIFL_Path[4];	/* SPI Flash logical drive path */
extern char USBH_Path[4];	/* USB stick logical drive path */

extern char SPISD_Path[4];	/* SPI SD card logical drive path */

extern SPI_HandleTypeDef hspi1;

typedef enum {
	SPI_SDCARD = 0,
	SPI_FLASH
} dselect_t;

void deviceSelect(dselect_t device);
void deviceDeselect();


void MX_FATFS_Init(void);
FRESULT transferFile(const TCHAR *source, const TCHAR *dest, uint8_t overwrite);
/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */
#ifdef __cplusplus
}
#endif
#endif /*__fatfs_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
