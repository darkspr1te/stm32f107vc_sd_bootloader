/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
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

#include "fatfs.h"
#include "boot_conf.h"
//uint8_t retUSER;    /* Return value for USER */
//char USERPath[4];   /* USER logical drive path */
//FATFS USERFatFS;    /* File system object for USER logical drive */
//FIL USERFile;       /* File object for USER */

//extern char SPIFL_Path[4];	/* SPI Flash logical drive path */
//extern char USBH_Path[4];	/* USB stick logical drive path */
//extern char SPISD_Path[4];	/* SPI SD card logical drive path */

//extern SPI_HandleTypeDef hspi1;

char SPISD_Path[4];     /* USER logical drive path */
char SPIFL_Path[4];     /* SPI Flash logical drive path */
char USBH_Path[4];      /* USB stick logical drive path */

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */    
void deviceSelect(dselect_t device)  {

	if (device == SPI_SDCARD) {
		HAL_GPIO_WritePin(FLASH_nCS_GPIO_Port, FLASH_nCS_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SDCARD_nCS_GPIO_Port, SDCARD_nCS_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(SDCARD_nCS_GPIO_Port, SDCARD_nCS_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(FLASH_nCS_GPIO_Port, FLASH_nCS_Pin, GPIO_PIN_RESET);
	}
}

void deviceDeselect() {
	HAL_GPIO_WritePin(SDCARD_nCS_GPIO_Port, SDCARD_nCS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FLASH_nCS_GPIO_Port, FLASH_nCS_Pin, GPIO_PIN_SET);
}


FRESULT transferFile(const TCHAR *source, const TCHAR *dest, uint8_t overwrite) {

	FIL		*pSourceFile = NULL;	// >4k
	FIL		*pDestFile = NULL;		// >4k
	BYTE	*pBuffer = NULL;		// _MAX_SS = 4k

	FRESULT res = FR_OK;

	if (NULL != (pSourceFile = (FIL *)pvPortMalloc(sizeof(FIL))) &&
			NULL != (pDestFile = (FIL *)pvPortMalloc(sizeof(FIL))) &&
			NULL != (pBuffer = (BYTE *)pvPortMalloc(_MIN_SS))) {

		if (FR_OK == (res = f_open(pSourceFile, source, FA_READ))) {
			if (FR_OK == (res = f_open(pDestFile, dest,	FA_WRITE
									| (overwrite ? FA_CREATE_ALWAYS : FA_CREATE_NEW)))) {

				size_t bytes = (size_t) -1, wbytes;
				do {
					if (FR_OK
							== (res = f_read(pSourceFile, pBuffer, _MIN_SS,
									&bytes)) && bytes) {
						res = f_write(pDestFile, pBuffer, bytes, &wbytes);
					}
				} while (FR_OK == res && bytes && wbytes);

				f_close(pDestFile);
			}

			f_close(pSourceFile);
		}

	} else {
		res = FR_NOT_ENOUGH_CORE;
	}

	if (pBuffer) vPortFree(pBuffer);
	if (pDestFile) vPortFree(pDestFile);
	if (pSourceFile) vPortFree(pSourceFile);

	return res;
}
void MX_FATFS_Init(void) 
{
  /*## FatFS: Link the USER driver ###########################*/
  //retUSER = FATFS_LinkDriver(&USER_Driver, USERPath);

   // FATFS_LinkDriver(&SPIFLASH_Driver, SPIFL_Path);	// 0:/
    FATFS_LinkDriver(&SPISD_Driver, SPISD_Path);    // 1:/
  //  FATFS_LinkDriver(&USBH_Driver, USBH_Path);		// 2:/

  /* USER CODE BEGIN Init */
  /* additional user code for init */     
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC 
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  return 0;
  /* USER CODE END get_fattime */  
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
