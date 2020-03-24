/**
 ******************************************************************************
 * @file    LED_User.h
 * @brief   包含了用户LED相关的数据操作接口，本文件允许用户自行修改.
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
#ifndef LED_USER_H
#define LED_USER_H

/* Includes ------------------------------------------------------------------*/
#include "LED.h"

/**
  * @brief 可用按键序列枚举
  * @note  说明：此序列中的枚举由用户指定，从0开始按顺序递增。
  */
enum UserLEDs
{
    Board_LED = 0,     // Nucleo板上的绿LED
    Dev_LED,           // 功率板上的红LED
    LEDNum             // 标记用户按键的总个数
};

/**
  * @brief 用户LED实例序列
  * @note  说明：访问时可通过枚举 @UserLEDs 作为下标进行快速访问。
  */
extern LED_t UserLEDs[LEDNum];

#endif /* KEY_USER_H */

/************************ (C) COPYRIGHT HITwh Excellent Robot Organization(HERO). *****END OF FILE****/
