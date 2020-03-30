/**
 ******************************************************************************
 * @file    LED.h
 * @brief   包含了LED控制相关的操作和类型声明，本文件不建议用户修改.
 ******************************************************************************
 *
 * Copyright (C) HITwh Excellent Robot Organization(HERO). 2015-2020. All rights reserved.
 *
 * @version 1.0 示例版本
 * @author  杨亦凡
 * @contact 17863107058(手机)   942041771(qq)
 * @date    2020/03/23
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LED_H
#define LED_H

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>

/* 在此包含所用芯片的头文件 */
#include "stm32f3xx.h"
/* USER CODE END Includes */


/**
  * @brief 根据芯片型号选择处理方法
  * @note 除不能使用位带操作IO的芯片外，尽量使用位带方式对普通IO进行读写。
  */
#if defined(STM32F1)
#define USING_BITBAND
#include "io_bit_band_F10x.h"

#elif defined(STM32F3)
// Do nothing, F3 series do not support bit-band operation.

#elif defined(STM32F4)
#define USING_BITBAND
#include "io_bit_band_F4xx.h"
#endif /* STM32Fx */


/**
  * @brief LED任务类型枚举定义
  * @note  说明：此处定义了几类LED任务，当启用“任务”功能时，它将被周期性地执行。
  *        任何任务在执行完成前都不允许被打断，除非它被显式地终止。另外，任务彼此互斥。  
  */
typedef enum
{
    LED_NOTASK = 0x00,      // 无任务
    LED_TASK_BREATHE,       // 呼吸灯(PWM驱动的灯特有)
    LED_TASK_TWINKLE,       // 闪烁灯
    LED_TASKNum
}LEDTask_t;

/**
  * @brief LED状态枚举定义
  */
typedef enum
{
    LED_STATE_ON,      
    LED_STATE_OFF, 
}LEDState_t;

/**
  * @brief LED类型枚举定义
  */
typedef enum
{
    LED_TYPE_ONOFF,      // IO控制的仅支持开关的LED
    LED_TYPE_Tunable,    // PWM控制的可调亮度的LED
}LEDType_t;

/**
  * @brief LED工作错误枚举定义
  */
enum LEDErrorCode
{
    LED_NOERROR = 0,          // 未出错
    LED_ERROR_ParamInvalid,   // 初始化函数输入参数非法
    LED_ERROR_IOTurning,      // IO方式的LED还妄图调亮度？（狗头）
};


/**
  * @brief LED组件结构定义
  */
typedef struct
{
    LEDType_t       Type;          // LED类型
    LEDState_t      State;         // LED状态(亮灭)
                                  
    /* IO_LED */                  
    GPIO_TypeDef*   Port;          // 引脚端口
    uint16_t        Pin;           // 引脚位置
    uint16_t        PinNum;        // 引脚号
    GPIO_PinState   ONPolarity;    // 使LED亮起时的电平状态
    GPIO_PinState   OFFPolarity;   // 使LED熄灭时的电平状态
#ifdef USING_BITBAND              
    BIT_BAND_t      LED_Write;     // IO写位带
#endif                            
                                  
    /* PWM_LED */                 
    TIM_HandleTypeDef* htim;       // 时基句柄
    uint32_t           Channel;    // 输出通道
    float              Level;      // 亮度等级 (0 - 99)
                                  
    /* LED_Task */                
    bool        isTasking;         // 用于标记任务状态的互斥量
    LEDTask_t   TaskType;          // 任务类型
    uint32_t    Period;            // 变化周期 (ms为单位)
    uint32_t    Times;             // 变化次数 (0 - UINT32_MAX, 0代表无限次)
                                  
    uint32_t    TimeCnt;           // 计时器 (ms为单位)
    uint32_t    PeriodCnt;         // 周期计数器 (次)    
}LED_t, *pLED_t;


/* Exported functions --------------------------------------------------------*/
void LED_ON_OFF_Init(pLED_t pLED, GPIO_TypeDef* Port, uint16_t Pin, GPIO_PinState ONPolarity);
void LED_Tunable_Init(pLED_t pLED, TIM_HandleTypeDef *htim, uint32_t Channel, float Level);

void LED_TurnON(pLED_t pLED);
void LED_TurnOFF(pLED_t pLED);
void LED_ON_OFF_Toggle(pLED_t pLED);

void LED_Adjustting(pLED_t pLED, float Level);

void LED_Task_Start(pLED_t pLED, LEDTask_t Task, uint32_t Period, uint32_t Times);
void LED_Task_Abort(pLED_t pLED);
void LED_Task_AdjPeriod(pLED_t pLED, uint32_t NewPeriod);
void LED_TaskLoop(void);

#endif /* LED_H */

/************************ (C) COPYRIGHT HITwh Excellent Robot Organization(HERO). *****END OF FILE****/
