/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "boot_conf.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#if defined(STM32F107xC)

SPI_HandleTypeDef hspi1;	// SD-card
TIM_HandleTypeDef htim2;	// Buzzer


char SPISD_Path[4];     /* USER logical drive path */

//static void MX_SPI1_Init(void);
//static void MX_TIM2_Init(void);
static void ShortBeep();

#elif defined(STM32F103xE)
# include "bsp_driver_sd.h"

SD_HandleTypeDef hsd;
HAL_SD_CardInfoTypedef SDCardInfo;
char SD_Path[4];        /* SD logical drive path */

static void MX_SDIO_SD_Init(void);
static inline void ShortBeep() {}
#endif

FATFS sdFileSystem;		// 0:/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

void osDelay(__IO uint32_t Delay)
{
    HAL_Delay(Delay);
}

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

inline void moveVectorTable(uint32_t Offset)
{
    // __disable_irq();
    SCB->VTOR = FLASH_BASE | Offset;
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
 
  HAL_Init();


  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_FATFS_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  #ifdef DEBUG
  printf("\n\rBooting\n\r");
  #endif
  MX_FATFS_Init();
  ShortBeep();
#ifdef DEBUG
  printf("Mounting Filesystem...\n\r");
#endif
  unsigned char result;
  result = f_mount(&sdFileSystem, SPISD_Path, 1);

  if (result == FR_OK)
  {
    #ifdef DEBUG
	  printf("SUCCESS! opensd\n\r");
    #endif
  } else {
    #ifdef DEBUG
	  printf("FATFS FAILED CODE : %d\r\n", (int)result);
    #endif
  }
  if (FR_OK == f_mount(&sdFileSystem, SPISD_Path, 1) && FR_OK == flash(FIRMWARE))
  //if (FR_OK == f_mount(&sdFileSystem, SPISD_Path, 1) && FR_OK == flash("0:/mks.bin"))
  {
      f_mount(NULL, SPISD_Path, 1);
      ShortBeep();
      HAL_SPI_MspDeInit(&hspi1);
      HAL_TIM_Base_MspDeInit(&htim2);

      __HAL_RCC_GPIOA_CLK_DISABLE();
      __HAL_RCC_GPIOB_CLK_DISABLE();
      __HAL_RCC_GPIOC_CLK_DISABLE();
      __HAL_RCC_GPIOD_CLK_DISABLE();
      __HAL_RCC_GPIOE_CLK_DISABLE();

      HAL_DeInit();

      // Disabling SysTick interrupt
      SysTick->CTRL = 0;
      moveVectorTable(MAIN_PR_OFFSET);
      // Setting initial value to stack pointer
      __set_MSP(*mcuFirstPageAddr);
      // booting really

      Callable resetHandler = (Callable) (*(mcuFirstPageAddr + 1) );
      resetHandler();
  }
 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    ShortBeep();
    
    #ifdef DEBUG
    printf("app failed to boot at %#010x\n\r",MAIN_PR_OFFSET);
    printf("we should not be here!\n\r");
    printf("no app to boot too!\n\r");
    #endif
    HAL_Delay(5000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the Systick interrupt time 
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* SPI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(SPI1_IRQn);
}

/* USER CODE BEGIN 4 */
void ShortBeep()
{
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_Delay(40);
	HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_3);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
