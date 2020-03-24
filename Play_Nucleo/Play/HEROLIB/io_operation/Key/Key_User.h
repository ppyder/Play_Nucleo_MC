/**
 ******************************************************************************
 * @file    Key_User.h
 * @brief   包含了用户按键相关的数据操作接口，本文件允许用户自行修改.
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
#ifndef KEY_USER_H
#define KEY_USER_H

/* Includes ------------------------------------------------------------------*/
#include "Key.h"

/**
  * @brief 可用按键序列枚举
  * @note  说明：此序列中的枚举由用户指定，从0开始按顺序递增。
  */
enum UserKeys
{
    Key1 = 0,
    KeyNum  // 标记用户按键的总个数
};

/**
  * @brief 用户按键实例序列
  * @note  说明：访问时可通过枚举 @UserKeys 作为下标进行快速访问。
  */
extern Key_t UserKeys[KeyNum];

#endif /* KEY_USER_H */

/************************ (C) COPYRIGHT HITwh Excellent Robot Organization(HERO). *****END OF FILE****/
