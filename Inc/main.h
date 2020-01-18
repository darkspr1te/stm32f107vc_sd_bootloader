/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#define SOFTWARE_VERSION "1.0"



void Error_Handler(void);


/*

experimental defines/code for app valid detect 


#define NONE 0x00
#define APP 0x01
#define BL 0x02

#define HEAD_BASE_ADDR 0x0803F800
#define UPDATE_HEAD_ADDR 0x0803F000
#define APP_HEAD_ADDR 0x0803F800
#define BL_HEAD_ADDR 0x0803FC00
#define BANK_ADDR 	   0x0801B000
#define BL_BASE_ADDR 	0x08000000
#define BL_BANK_ADDR 	   0x08037000

struct head{
	uint8_t magic;
	uint8_t type;
	uint16_t depends;
	uint16_t ver;
	uint8_t time[6];
	uint32_t len;
	uint32_t addr;
	uint8_t crc[150];
	uint8_t reserve[30];
} ;

 struct head *app_head = (struct head *)APP_HEAD_ADDR ;
 
 
 if(app_head->magic == 'W' && app_head->type == APP){
		go_to(app_head->addr);
		

*/
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

