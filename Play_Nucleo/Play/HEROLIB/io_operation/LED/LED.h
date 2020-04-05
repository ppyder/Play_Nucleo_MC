/**
 ******************************************************************************
 * @file    LED.h
 * @brief   ������LED������صĲ������������������ļ��������û��޸�.
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
#ifndef LED_H
#define LED_H

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>

/* �ڴ˰�������оƬ��ͷ�ļ� */
#include "stm32f3xx.h"
/* USER CODE END Includes */


/**
  * @brief ����оƬ�ͺ�ѡ������
  * @note ������ʹ��λ������IO��оƬ�⣬����ʹ��λ����ʽ����ͨIO���ж�д��
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
  * @brief LED��������ö�ٶ���
  * @note  ˵�����˴������˼���LED���񣬵����á����񡱹���ʱ�������������Ե�ִ�С�
  *        �κ�������ִ�����ǰ����������ϣ�����������ʽ����ֹ�����⣬����˴˻��⡣  
  */
typedef enum
{
    LED_NOTASK = 0x00,      // ������
    LED_TASK_BREATHE,       // ������(PWM�����ĵ�����)
    LED_TASK_TWINKLE,       // ��˸��
    LED_TASKNum
}LEDTask_t;

/**
  * @brief LED״̬ö�ٶ���
  */
typedef enum
{
    LED_STATE_ON,      
    LED_STATE_OFF, 
}LEDState_t;

/**
  * @brief LED����ö�ٶ���
  */
typedef enum
{
    LED_TYPE_ONOFF,      // IO���ƵĽ�֧�ֿ��ص�LED
    LED_TYPE_Tunable,    // PWM���ƵĿɵ����ȵ�LED
}LEDType_t;

/**
  * @brief LED��������ö�ٶ���
  */
enum LEDErrorCode
{
    LED_NOERROR = 0,          // δ����
    LED_ERROR_ParamInvalid,   // ��ʼ��������������Ƿ�
    LED_ERROR_IOTurning,      // IO��ʽ��LED����ͼ�����ȣ�����ͷ��
};


/**
  * @brief LED����ṹ����
  */
typedef struct
{
    LEDType_t       Type;          // LED����
    LEDState_t      State;         // LED״̬(����)
                                  
    /* IO_LED */                  
    GPIO_TypeDef*   Port;          // ���Ŷ˿�
    uint16_t        Pin;           // ����λ��
    uint16_t        PinNum;        // ���ź�
    GPIO_PinState   ONPolarity;    // ʹLED����ʱ�ĵ�ƽ״̬
    GPIO_PinState   OFFPolarity;   // ʹLEDϨ��ʱ�ĵ�ƽ״̬
#ifdef USING_BITBAND              
    BIT_BAND_t      LED_Write;     // IOдλ��
#endif                            
                                  
    /* PWM_LED */                 
    TIM_HandleTypeDef* htim;       // ʱ�����
    uint32_t           Channel;    // ���ͨ��
    float              Level;      // ���ȵȼ� (0 - 99)
                                  
    /* LED_Task */                
    bool        isTasking;         // ���ڱ������״̬�Ļ�����
    LEDTask_t   TaskType;          // ��������
    uint32_t    Period;            // �仯���� (msΪ��λ)
    uint32_t    Times;             // �仯���� (0 - UINT32_MAX, 0�������޴�)
                                  
    uint32_t    TimeCnt;           // ��ʱ�� (msΪ��λ)
    uint32_t    PeriodCnt;         // ���ڼ����� (��)    
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
void LED_TaskUpdate(void);
void LED_TaskLoop(void);

#endif /* LED_H */

/************************ (C) COPYRIGHT HITwh Excellent Robot Organization(HERO). *****END OF FILE****/
