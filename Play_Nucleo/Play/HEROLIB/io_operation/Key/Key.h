/**
 ******************************************************************************
 * @file    Key.h
 * @brief   �����˰�����صĲ������������������ļ��������û��޸�.
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
#ifndef KEY_H
#define KEY_H

/* Includes ------------------------------------------------------------------*/
#include "filter.h"

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
  * @brief ������ⷽʽö�ٶ���
  */
typedef enum 
{
    KEY_MODE_INT,   // �ⲿ�жϷ�ʽ
    KEY_MODE_SCAN   // ��ѯɨ�跽ʽ
}KeyInputMode_t;

/**
  * @brief �������Ʒ�ʽö�ٶ���
  * @note  ģʽ����
  *         �㶯���ƣ�����������Ч��������Ч�����ɼ�ͣ��
  *         �������ƣ��������µ����������л�״̬��
  */
typedef enum 
{
    KEY_TYPE_INCHING,    // �㶯����
    KEY_TYPE_SWITCH      // ��������
}KeyType_t;

/**
  * @brief �����¼�ö�ٶ���
  * @note �����ڳ�ʼ��ʱ��ǹ��ĵ��¼������¼�����ʱ���������Ӧ�Ĳ���
  */
typedef enum 
{
    KEY_NOEVENT = 0X00,         // û�а����¼�
    KEY_EVENT_DOWN    = 0x01,   // �����ѱ����£���ϵ㶯ģʽ��
    KEY_EVENT_UP      = 0x02,   // �����Ѿ����𣨵㶯��
    KEY_EVENT_PRESS   = 0x04,   // ����������һ�Σ�������
}KeyEvents_t;

/**
  * @brief ������������ö�ٶ���
  */
enum KeyErrorCode
{
    KEY_NOERROR = 0,          // δ����
    KEY_ERROR_ParamInvalid,   // ��ʼ��������������Ƿ�
    KEY_ERROR_PortInvalid,    // GPIO�˿ڷǷ�
};


/**
  * @brief �������� �ṹ����
  */
typedef struct Key_Typedef
{
    KeyType_t       Type;               // ��������
    KeyInputMode_t  Mode;               // ������ⷽʽ
    
    GPIO_TypeDef*   Port;               // ���Ŷ˿�
    uint16_t        Pin;                // ����λ��
    uint16_t        PinNum;             // ���ź�
    GPIO_PinState   DownPolarity;       // ��������ʱ�ĵ�ƽ״̬
    
#ifdef USING_BITBAND
    BIT_BAND_t      Key_Pin_Read;       // IO��λ��
#endif    
    KeyEvents_t     Target_Events;      // ���ĵ��¼�����
    KeyEvents_t     Events;             // �����¼�״̬�����ô��������Զ����
    void (*pDealFunc)(KeyEvents_t);     // ����������õĴ�����
    
    /** ɨ�跽ʽ��صĲ��� **/
    StateFilter_t   KeyFillter;         // ����״̬�˲���
    
    /** �жϷ�ʽ��صĲ��� **/
    EXTI_HandleTypeDef hexti;           // �����ж��߾��
    
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
void Key_EventScan(void);

#endif /* KEY_H */

/************************ (C) COPYRIGHT HITwh Excellent Robot Organization(HERO). *****END OF FILE****/
