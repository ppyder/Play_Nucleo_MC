/**
 ******************************************************************************
 * @file    LED_User.h
 * @brief   �������û�LED��ص����ݲ����ӿڣ����ļ������û������޸�.
 ******************************************************************************
 *
 * Copyright (C) HITwh Excellent Robot Organization(HERO). 2015-2020. All rights reserved.
 *
 * @version 1.0 ʾ���汾
 * @author  ���ෲ
 * @contact 17863107058(�ֻ�)   942041771(qq)
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
  * @brief ���ð�������ö��
  * @note  ˵�����������е�ö�����û�ָ������0��ʼ��˳�������
  */
enum UserLEDs
{
    Board_LED = 0,     // Nucleo���ϵ���LED
    Dev_LED,           // ���ʰ��ϵĺ�LED
    LEDNum             // ����û��������ܸ���
};

/**
  * @brief �û�LEDʵ������
  * @note  ˵��������ʱ��ͨ��ö�� @UserLEDs ��Ϊ�±���п��ٷ��ʡ�
  */
extern LED_t UserLEDs[LEDNum];

#endif /* KEY_USER_H */

/************************ (C) COPYRIGHT HITwh Excellent Robot Organization(HERO). *****END OF FILE****/
