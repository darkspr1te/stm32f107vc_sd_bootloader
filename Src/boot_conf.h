
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOOT_CONF_H
#define __BOOT_CONF_H

#include "stm32f1xx_hal.h"
#include "fatfs.h"

#define MKS_TFT

#if defined(STM32F107xC) && defined(MKS_TFT)
/**
 * Makerbase MKS-TFT32
 */
#define SPEAKER_Pin             GPIO_PIN_2
#define SPEAKER_GPIO_Port       GPIOA
#define SDCARD_nCS_Pin          GPIO_PIN_11
#define SDCARD_nCS_GPIO_Port    GPIOD
#define FLASH_nCS_Pin           GPIO_PIN_9
#define FLASH_nCS_GPIO_Port     GPIOB
#define FIRMWARE_FILENAME       "mks.bin"
//Vector address to load firmware and boot 
#define MAIN_PR_OFFSET 0x7000
//printed via UART1 during debug builds
#define HARDWARE                "MKS TFT 2.8 V1.4 Board"  

#endif

/**
 *  if you dont want to rename firmware update file after flashing 
 *  == Caution ==
 *  Bootloader will write firmware to flash each boot if file not rename after update
 *  define only for testing purpose, remove define in release builds 
 */
//#define DONT_RENAME



//Common Defines 
#define LOCATION                "0:/"
#define FIRMWARE      LOCATION FIRMWARE_FILENAME
#define OLD                     "old"
#define RENAME_FILE        LOCATION OLD FIRMWARE_FILENAME
extern FATFS sdFileSystem;		// 0:/


#if !defined(MAIN_PR_OFFSET)
#define MAIN_PR_OFFSET 0x8000
#endif
#if !defined(FIRMWARE)
#define FIRMWARE                "0:/firmware.bin"
#endif 


typedef enum
{
	FLASH_RESULT_OK = 0,
	FLASH_RESULT_FILE_ERROR,
	FLASH_RESULT_FLASH_ERROR,
	FLASH_FILE_NOT_EXISTS,
	FLASH_FILE_CANNOT_OPEN,
	FLASH_FILE_INVALID_HASH,
	FLASH_FILE_TOO_BIG
} FlashResult;

FlashResult flash(const char *fname);
extern const uint32_t *mcuFirstPageAddr;

typedef void (*Callable)();



#endif /* __BOOT_CONF_H */

