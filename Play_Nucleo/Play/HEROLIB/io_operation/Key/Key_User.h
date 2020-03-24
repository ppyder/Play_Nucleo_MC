/**
 ******************************************************************************
 * @file    Key_User.h
 * @brief   �������û�������ص����ݲ����ӿڣ����ļ������û������޸�.
 ******************************************************************************
 *
 * Copyright (C) HITwh Excellent Robot Organization(HERO). 2015-2020. All rights reserved.
 *
 * @version 1.0 ʾ���汾
 * @author  ���ෲ
 * @contact 17863107058(�ֻ�)   942041771(qq)
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
  * @brief ���ð�������ö��
  * @note  ˵�����������е�ö�����û�ָ������0��ʼ��˳�������
  */
enum UserKeys
{
    Key1 = 0,
    KeyNum  // ����û��������ܸ���
};

/**
  * @brief �û�����ʵ������
  * @note  ˵��������ʱ��ͨ��ö�� @UserKeys ��Ϊ�±���п��ٷ��ʡ�
  */
extern Key_t UserKeys[KeyNum];

#endif /* KEY_USER_H */

/************************ (C) COPYRIGHT HITwh Excellent Robot Organization(HERO). *****END OF FILE****/
