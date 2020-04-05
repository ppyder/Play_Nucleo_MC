/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "UserKey.h" 
#include "UserLED.h" 
#include "AD_Monitor.h"
#include "TinyDelay.h"
#include "AS5048a.h"
#include "Time_Reference.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//bool AS5048a_ReadWrite_DMA(pAS5048a_t pDev, uint16_t *pTxBuffer, volatile uint16_t *pRxBuffer);
//bool AS5048a_ReadWrite_Block(pAS5048a_t pDevice, uint16_t *pTxBuffer, volatile uint16_t *pRxBuffer);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define SYS_TICK_FREQUENCY 2000
uint32_t Temp = 0;
uint8_t Temp_ch = '\0';
uint32_t DelayCnt = 0;
uint32_t i = 0;
uint16_t TempCMD = AS5048A_CMD_READANGLE; 
uint16_t TempRead = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    const char Str[] = "Hello!";
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_SPI3_Init();
  MX_TIM6_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  
    //延时100ms等待系统电源稳定
    HAL_Delay(100);
  
    /* Reconfigure the SysTick interrupt to fire every 500 us. */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/SYS_TICK_FREQUENCY);

    //获取系统频率
    Temp = HAL_RCC_GetSysClockFreq();
        
    //开启串口接收(测试用)
    HAL_UART_Receive_IT(&huart2, &Temp_ch, 1);
    
    //启动定时器时基
    HAL_TIM_Base_Start(&htim2);
    
    //初始化用户按键
    UserKeyInit();
    
    //初始化用户LED
    UserLED_Init();
    
    //AD监视器初始化
    AD_MonitorInit();
    
    //获取ADC频率
    Temp = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_ADC1);
    
    //初始化延时
    TinyDelayInit(&htim6, 72);
    
    //初始化工作时间基准
    TimeRef_Init(&htim16, 144);
    
    //开始计时
    TimeRef_Start();
        
    //AS5048a初始化
    AS5048a_Init(&AS5048a, 15000, GPIOA, GPIO_PIN_4, &hspi3, 49478, 5);
    
    //测试发送字节
    HAL_UART_Transmit_IT(&huart2, (uint8_t*)Str, sizeof(Str));
    
    Temp = sizeof(AS5048a_AGCR_t);
        
//    for(i = 0; i < 1000000; i++)
//    {
//        while(AS5048A_STATE_BUSY == AS5048a.State);
//        AS5048a_ReadWrite_Block(&AS5048a, &TempCMD, &TempRead);
//    }
    
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      //按键事件监视与响应
      Key_EventScan();
      
      //LED任务执行
      LED_TaskLoop();
            
      //转换监视的ADC值
      RegularDataDeal();
      
      //更新总秒数
      TimeRef_GetTotal();
            
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM16;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_PLLCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
