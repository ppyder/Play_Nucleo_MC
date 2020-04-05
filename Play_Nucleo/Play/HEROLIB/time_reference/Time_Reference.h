/**
 ******************************************************************************
 * @file    TimeReference.h
 * @brief   ������ʱ���׼�Ĳ������������������ļ��������û��޸�.
 ******************************************************************************
 *
 * Copyright (C) HITwh Excellent Robot Organization(HERO). 2015-2020. All rights reserved.
 *
 * @version 1.0 ʾ���汾
 * @author  ���ෲ
 * @contact 17863107058(�ֻ�)   942041771(qq)
 * @date    2020/04/03
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TIME_REFERENCE_H
#define TIME_REFERENCE_H

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>

/* �ڴ˰�������оƬ��ͷ�ļ� */
#include "stm32f3xx.h"
/* USER CODE END Includes */


/**
  * @brief ͨ��ʱ��ṹ�����Ͷ���
  */
typedef struct
{
    uint32_t Ticks;     // tick����ֵ
    int64_t  CarryNum;  // tick�����λֵ
}GlobalTime_t, *pGlobalTime;


//�Ĵ������ݷ�Χ
#define TIMEREF_CNT_MAX   (65536)   // ʹ��CNT��������ж���Ҫ���������tick��

//ʹ��ʱ�������жϵ���tick����ÿ����ô��tick���ж���CarryNum��һ��
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
