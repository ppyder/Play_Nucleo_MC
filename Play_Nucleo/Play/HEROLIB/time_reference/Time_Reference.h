/**
 ******************************************************************************
 * @file    TimeReference.h
 * @brief   包含了时间基准的操作和类型声明，本文件不建议用户修改.
 ******************************************************************************
 *
 * Copyright (C) HITwh Excellent Robot Organization(HERO). 2015-2020. All rights reserved.
 *
 * @version 1.0 示例版本
 * @author  杨亦凡
 * @contact 17863107058(手机)   942041771(qq)
 * @date    2020/04/03
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TIME_REFERENCE_H
#define TIME_REFERENCE_H

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>

/* 在此包含所用芯片的头文件 */
#include "stm32f3xx.h"
/* USER CODE END Includes */


/**
  * @brief 通用时间结构体类型定义
  */
typedef struct
{
    uint32_t Ticks;     // tick计数值
    int64_t  CarryNum;  // tick溢出进位值
}GlobalTime_t, *pGlobalTime;


//寄存器数据范围
#define TIMEREF_CNT_MAX   (65536)   // 使得CNT溢出产生中断需要向其输入的tick数

//使定时器产生中断的总tick数，每过这么多tick，中断中CarryNum涨一。
#define TIMEREF_TOTALCNT  (TIMEREF_CNT_MAX)  

/* Exported functions --------------------------------------------------------*/
bool  TimeRef_Init(TIM_HandleTypeDef* htim, uint32_t Freq_MHz);
bool  TimeRef_Start(void);
bool  TimeRef_Clear(void);

float TimeRef_GetTotal(void);
float TimeRef_GetTotal_FromGlobalTime(pGlobalTime pTime);

bool  TimeRef_GetPreciseTime(pGlobalTime pTime);
float TimeRef_TimeMinus(pGlobalTime pLeft, pGlobalTime pRight);

void  TimeReference_Callback(void);

#endif /* TIME_REFERENCE_H */

/************************ (C) COPYRIGHT HITwh Excellent Robot Organization(HERO). *****END OF FILE****/
