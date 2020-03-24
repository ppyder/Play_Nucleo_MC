/**
 ******************************************************************************
 * @file    Key.h
 * @brief   包含了按键相关的操作和类型声明，本文件不建议用户修改.
 ******************************************************************************
 *
 * Copyright (C) HITwh Excellent Robot Organization(HERO). 2015-2020. All rights reserved.
 *
 * @version 1.0 示例版本
 * @author  杨亦凡
 * @contact 17863107058(手机)   942041771(qq)
 * @date    2020/03/19
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef KEY_H
#define KEY_H

/* Includes ------------------------------------------------------------------*/
#include "filter.h"

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
  * @brief 按键检测方式枚举定义
  */
typedef enum 
{
    KEY_MODE_INT,   // 外部中断方式
    KEY_MODE_SCAN   // 轮询扫描方式
}KeyInputMode_t;

/**
  * @brief 按键控制方式枚举定义
  * @note  模式介绍
  *         点动控制：按键按下有效，松手无效，即松即停。
  *         长动控制：按键按下的意义在于切换状态。
  */
typedef enum 
{
    KEY_TYPE_INCHING,    // 点动控制
    KEY_TYPE_SWITCH      // 长动控制
}KeyType_t;

/**
  * @brief 按键事件枚举定义
  * @note 可以在初始化时标记关心的事件，当事件发生时，会产生相应的捕获。
  */
typedef enum 
{
    KEY_NOEVENT = 0X00,         // 没有按键事件
    KEY_EVENT_DOWN    = 0x01,   // 按键已被按下（配合点动模式）
    KEY_EVENT_UP      = 0x02,   // 按键已经弹起（点动）
    KEY_EVENT_PRESS   = 0x04,   // 按键被按过一次（长动）
}KeyEvents_t;

/**
  * @brief 按键工作错误枚举定义
  */
enum KeyErrorCode
{
    KEY_NOERROR = 0,          // 未出错
    KEY_ERROR_ParamInvalid,   // 初始化函数输入参数非法
    KEY_ERROR_PortInvalid,    // GPIO端口非法
};


/**
  * @brief 按键类型 结构定义
  */
typedef struct Key_Typedef
{
    KeyType_t       Type;               // 按键类型
    KeyInputMode_t  Mode;               // 按键检测方式
    
    GPIO_TypeDef*   Port;               // 引脚端口
    uint16_t        Pin;                // 引脚位置
    uint16_t        PinNum;             // 引脚号
    GPIO_PinState   DownPolarity;       // 按键按下时的电平状态
    
#ifdef USING_BITBAND
    BIT_BAND_t      Key_Pin_Read;       // IO读位带
#endif    
    KeyEvents_t     Target_Events;      // 关心的事件类型
    KeyEvents_t     Events;             // 按键事件状态，调用处理函数后自动清空
    void (*pDealFunc)(KeyEvents_t);     // 捕获发生后调用的处理方法
    
    /** 扫描方式相关的参数 **/
    StateFilter_t   KeyFillter;         // 按键状态滤波器
    
    /** 中断方式相关的参数 **/
    EXTI_HandleTypeDef hexti;           // 按键中断线句柄
    
}Key_t, *pKey_t;


/* Exported functions --------------------------------------------------------*/
void Key_Scan_Init( pKey_t pKey, void (*pDealFunc)(KeyEvents_t Events),
                    KeyType_t Type, uint32_t CheckPeriod, uint32_t MaskBits,
                    GPIO_TypeDef* Port, uint16_t Pin, GPIO_PinState DownPolarity );
void Key_INT_Init( pKey_t pKey, void (*pDealFunc)(KeyEvents_t Events),
                   KeyType_t Type, uint32_t Line,
                   GPIO_TypeDef* Port, uint16_t Pin, GPIO_PinState DownPolarity);

void Key_Scan(void);
void Key_Int(uint16_t GPIO_Pin);

#endif /* KEY_H */

/************************ (C) COPYRIGHT HITwh Excellent Robot Organization(HERO). *****END OF FILE****/
