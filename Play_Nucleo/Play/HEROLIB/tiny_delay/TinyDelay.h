/**
 ******************************************************************************
 * @file    TinyDelay.h
 * @brief   �����˶�ʱ��ʱ�Ĳ������������������ļ��������û��޸�.
 ******************************************************************************
 *
 * Copyright (C) HITwh Excellent Robot Organization(HERO). 2015-2020. All rights reserved.
 *
 * @version 1.0 ʾ���汾
 * @author  ���ෲ
 * @contact 17863107058(�ֻ�)   942041771(qq)
 * @date    2020/03/31
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TINYDELAY_H
#define TINYDELAY_H

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>

/* �ڴ˰�������оƬ��ͷ�ļ� */
#include "stm32f3xx.h"
/* USER CODE END Includes */

/**
  * @brief ��ʱ���״̬ö�ٶ���
  */
typedef enum
{
    DELAY_STATE_RESET,  // δ��ʼ����״̬
    DELAY_STATE_READY,  // �ѳ�ʼ���Ŀ���״̬
    DELAY_STATE_BUSY,   // �������ڼ�ʱ
}DelayState_t;


/**
  * @brief ��ʱ����ṹ�����Ͷ���
  */
typedef struct
{
    TIM_HandleTypeDef     *htim;        // ��ʱ�����
    
    float                 DelayTime;    // ��ʱʱ�䳤�ȣ���λus
    volatile DelayState_t State;        // ��ʱ״̬
    
}TinyDelay_t, *pTinyDelay_t;


/* Exported functions --------------------------------------------------------*/
bool TinyDelayInit(TIM_HandleTypeDef *htim, uint32_t Freq_MHz);
bool Tiny_Delay(uint16_t Time);
void Tiny_Delay_Abort(void);
void Tiny_DelayCallback(void);

#endif /* TINYDELAY_H */

/************************ (C) COPYRIGHT HITwh Excellent Robot Organization(HERO). *****END OF FILE****/
