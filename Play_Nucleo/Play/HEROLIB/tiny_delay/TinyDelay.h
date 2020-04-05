/**
 ******************************************************************************
 * @file    TinyDelay.h
 * @brief   包含了短时延时的操作和类型声明，本文件不建议用户修改.
 ******************************************************************************
 *
 * Copyright (C) HITwh Excellent Robot Organization(HERO). 2015-2020. All rights reserved.
 *
 * @version 1.0 示例版本
 * @author  杨亦凡
 * @contact 17863107058(手机)   942041771(qq)
 * @date    2020/03/31
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TINYDELAY_H
#define TINYDELAY_H

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>

/* 在此包含所用芯片的头文件 */
#include "stm32f3xx.h"
/* USER CODE END Includes */

/**
  * @brief 延时组件状态枚举定义
  */
typedef enum
{
    DELAY_STATE_RESET,  // 未初始化的状态
    DELAY_STATE_READY,  // 已初始化的可用状态
    DELAY_STATE_BUSY,   // 标明正在计时
}DelayState_t;


/**
  * @brief 延时组件结构体类型定义
  */
typedef struct
{
    TIM_HandleTypeDef     *htim;        // 定时器句柄
    
    float                 DelayTime;    // 延时时间长度，单位us
    volatile DelayState_t State;        // 延时状态
    
}TinyDelay_t, *pTinyDelay_t;


/* Exported functions --------------------------------------------------------*/
bool TinyDelayInit(TIM_HandleTypeDef *htim, uint32_t Freq_MHz);
bool Tiny_Delay(uint16_t Time);
void Tiny_Delay_Abort(void);
void Tiny_DelayCallback(void);

#endif /* TINYDELAY_H */

/************************ (C) COPYRIGHT HITwh Excellent Robot Organization(HERO). *****END OF FILE****/
