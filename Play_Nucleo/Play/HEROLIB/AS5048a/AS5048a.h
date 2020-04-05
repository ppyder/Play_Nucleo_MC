/**
 ******************************************************************************
 * @file    AS5048a.h
 * @brief   ������AS5048a������Ĳ������������������ļ��������û��޸�.
 ******************************************************************************
 *
 * Copyright (C) HITwh Excellent Robot Organization(HERO). 2015-2020. All rights reserved.
 *
 * @version 1.0 ʾ���汾
 * @author  ���ෲ
 * @contact 17863107058(�ֻ�)   942041771(qq)
 * @date    2020/04/05
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef AS5048A_H
#define AS5048A_H

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "filter.h"
#include "Time_Reference.h"

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
  * @brief ���Կ���
  * @note ���ô˺�������ò��Ժ�����
  */
#define AS5048A_DEBUG

/* Exported macro ------------------------------------------------------------*/
/**
  * @brief AS5048a�����ٶȼ��㲻�ɿ���������
  * @note  �����ɿ������� @InvalidCnt ֵ���ڴ�ֵʱ��
  *       ��Ϊ�ϸ���Ч���������ٶ��Ѳ����ţ������� @isReliable��־��
  */
#define AS5048A_MAX_SPEED_INVALID_CNT (5)

/**
  * @brief AS5048a����֡ ��������
  */
#define AS5048A_PAR_SET    (0x01<<15) // żУ��λ��һ
#define AS5048A_PAR_RESET  (0x00<<15) // żУ��λ����
#define AS5048A_WRITE      (0x00<<14) // ָ��λ��дָ��
#define AS5048A_READ       (0x01<<14) // ָ��λ����ָ��

/**
  * @brief AS5048a����֡ ��������
  */
#define AS5048A_HOST_ERROR (0x01<<14) // �����־��һ��������һ�δ����������������ݴ��ڴ���
#define AS5048A_DATAMASK   (0x3FFF)   // ����֡�к�14λ�������ݣ�ǰ��λ�ǹ�����Ϣ

/**
  * @brief AS5048a�Ĵ�����ַ
  */
#define AS5048A_REG_NOP     0x0000   // ����Ĵ��������ڶ�ȡNOPָ�ֻ����
#define AS5048A_REG_EF      0x0001   // �����־�Ĵ��������ڶ�ȡ�����־��ֻ������ȡ�������
#define AS5048A_REG_PC      0x0003   // ��̿��ƼĴ��������ڽ��й����Կ��Ʋ������ɶ�д��
#define AS5048A_REG_ZPH     0x0016   // ��λ�߼Ĵ��������ڴ洢�Ƕ���λ�߰�λ���ɶ�д��
#define AS5048A_REG_ZPL     0x0017   // ��λ�ͼĴ��������ڴ洢�Ƕ���λ�Ͱ�λ���ɶ�д��
#define AS5048A_REG_AGC     0x3FFD   // ��Ϻ��Զ�����Ĵ�����������Ϲ����������ṩ���档ֻ����
#define AS5048A_REG_MAG     0x3FFE   // �ų���Ϣ�Ĵ��������ڴ洢�ų�ǿ����Ϣ��ֻ����
#define AS5048A_REG_ANGLE   0x3FFF   // �Ƕ���Ϣ�Ĵ��������ڴ洢�Ƕ���Ϣ��ֻ����

/**
  * @brief AS5048a���Ĵ�����ָ� AS5048A_CMD_READ
  */
#define AS5048A_CMD_READNOP   (AS5048A_PAR_SET   | AS5048A_READ | AS5048A_REG_NOP)   // 0xC000
#define AS5048A_CMD_READEF 	  (AS5048A_PAR_RESET | AS5048A_READ | AS5048A_REG_EF)    // 0x4001
#define AS5048A_CMD_READPC 	  (AS5048A_PAR_SET   | AS5048A_READ | AS5048A_REG_PC)    // 0xC003
#define AS5048A_CMD_READZPH   (AS5048A_PAR_RESET | AS5048A_READ | AS5048A_REG_ZPH)   // 0x4016
#define AS5048A_CMD_READZPL   (AS5048A_PAR_SET   | AS5048A_READ | AS5048A_REG_ZPL)   // 0xC017
#define AS5048A_CMD_READAGC   (AS5048A_PAR_RESET | AS5048A_READ | AS5048A_REG_AGC)   // 0x7FFD
#define AS5048A_CMD_READMAG   (AS5048A_PAR_RESET | AS5048A_READ | AS5048A_REG_MAG)   // 0x7FFE
#define AS5048A_CMD_READANGLE (AS5048A_PAR_SET   | AS5048A_READ | AS5048A_REG_ANGLE) // 0xFFFF

/**
  * @brief AS5048a�����꣺�ж��Ƿ��ڴ���״̬
  */
#define IS_AS5048A_ERROR(__STATE__) ((__STATE__) == AS5048A_STATE_SPI_ERROR || \
                                     (__STATE__) == AS5048A_STATE_CMD_ERROR || \
                                     (__STATE__) == AS5048A_STATE_ZP_ERROR  || \
                                     (__STATE__) == AS5048A_STATE_DIG_ERROR)
                                     
/* Exported types ------------------------------------------------------------*/
/**
  * @brief AS5048a����״̬ö�����Ͷ���
  */
typedef enum
{
    /* ����״̬ */
    AS5048A_STATE_RESET,        // δ��ʼ��
    AS5048A_STATE_READY,        // ׼����������ɣ�����
    AS5048A_STATE_BUSY,         // ����ͨ�Ź�����
    
    /* ����״̬����������״̬���������ֹͣ��������ֵ��С�Ĵ����뽫������ֵ����Ĵ����� */
    AS5048A_STATE_SPI_ERROR,    /* SPI�ײ㷢������һ����оƬ���裨��DMA��SPI�ȣ����������⡣
                                   �����ô���ʱ�����ڹ������� @AS5048A_HAL_ErrorCode�в鿴�ײ�����Ĵ����롣*/
    AS5048A_STATE_CMD_ERROR,    /* ���Ĵ���ʱ��������(Command Error)��
                                   ���磬����֡�д����־�������ֵĴ������࣬��������͵��ӻ���֡�����⡣   */
    AS5048A_STATE_DIG_ERROR,    /* ������������Ҫ��(Diagnose Error)��
                                   ���ܵ�����У��ų����ϸ�оƬδ�ϵ�(ƫ�ò���ʧ��)��CORDIC�㷨���       */
    AS5048A_STATE_ZP_ERROR,     /* ��λ��ʼ��ʧ��(Zero Position Error)��
                                   ����û�гɹ��ذ��ϵ�ʱ��ת��λ����Ϊ��λ��*/
}AS5048a_State_t;


/**
  * @brief REG - Error Flag �����־�Ĵ��� -- 0x0001
  */
typedef union
{
    uint16_t Value;                // ����֡д��ֵ
    struct
    {
        unsigned Framing   : 1;    // ֡����
        unsigned CMD       : 1;    // ��Чָ��
        unsigned Parity    : 1;    // ��żУ�����
        unsigned Unused    : 13;   // δʹ������
    }bit_area;
    
}AS5048a_EFR_t;

/**
  * @brief REG - Programming Control ��̿��ƼĴ��� -- 0x0003
  */
typedef union
{
    uint16_t Value;                // ����֡д��ֵ
    struct
    {
        unsigned PE        : 1;    // ���ʹ��λ��ֻ����������OTP�Ĵ����ſɱ�д��
        unsigned Unused1   : 2;    // δʹ������
        unsigned Burn      : 1;    // ��¼(�����ڲ����Զ���̳���)
        unsigned Unused2   : 2;    // δʹ������
        unsigned Verify    : 1;    // У��(��OTP���ݼ��ص������޸ĵ���ֵ�Ƚ���������ڲ��Ĵ�����)
        unsigned Unused3   : 7;    // δʹ������
    }bit_area;
    
}AS5048a_PCR_t;

/**
  * @brief REG - OTP: Zero Position High ��λ���ݸ߼Ĵ��� -- 0x0016
  * @note  OTP(One Time Programmable) ��һ�α�̼Ĵ�����
  *        ��8λ��Ч��
  */
typedef union
{
    uint16_t Value;                // ����֡д��ֵ
    struct
    {
        unsigned Data     : 8;     // ��λ�õĸ߰�λ����
        unsigned Unused   : 8;     // δʹ������
    }bit_area;
    
}AS5048a_ZPHR_t;

/**
  * @brief REG - OTP: Zero Position Low ��λ���ݵͼĴ��� -- 0x0017
  * @note  OTP(One Time Programmable) ��һ�α�̼Ĵ�����
  *        ��6λ��Ч��
  */
typedef union
{
    uint16_t Value;                // ����֡д��ֵ
    struct
    {
        unsigned Data     : 6;     // ��λ�õĵ���λ����
        unsigned Unused   : 10;    // δʹ������
    }bit_area;
    
}AS5048a_ZPLR_t;

/**
  * @brief REG - Diagnostics & Automatic Gain Control ��Ϻ��Զ�������ƼĴ��� -- 0x3FFD
  * @note  ��COMP_H/COMP_Lλ��һʱ������ȥ�鿴Magnitude�Ĵ����Բ��������
  */
typedef union
{
    uint16_t Value;                // ����֡д��ֵ
    struct
    {
        unsigned AutoGrain : 8;    // �Զ���������ֵ(0��ʾǿ�ų���255��ʾ���ų�)
        unsigned OCF       : 1;    /* (Offset Compensation Finished)����λ��һ��ʾƫ�Ʋ����㷨�Ѿ���ɡ�
                                      ��λһ���ϵ��ͻ���һ��*/
        unsigned COF       : 1;    /* (Cordic Overflow)����λ��һ��ʾ�ڲ���CORDIC�㷨�������������
                                      ��λ��һʱ���ǶȺʹų��������ݲ����ã�PWM����������ϴε���Ч���״̬��*/
        unsigned Comp_L    : 1;    // �ų���ǿ��־λ��AutoGrainΪ0ʱ��λ��һ��
        unsigned Comp_H    : 1;    // �ų�������־λ��AutoGrainΪ255ʱ��λ��һ��
        unsigned Unused    : 4;    // δʹ������
    }bit_area;
    
}AS5048a_AGCR_t;

/**
  * @brief REG - Magnitude �ų����ݼĴ��� -- 0x3FFE
  */ 
typedef union
{
    uint16_t Value;                // ����֡д��ֵ
    struct
    {
        unsigned Data     : 14;    // �ų�����
        unsigned Unused   : 2;     // δʹ������
    }bit_area;
    
}AS5048a_MAGR_t;

/**
  * @brief REG - Angle     �Ƕ����ݼĴ��� -- 0x3FFF
  */ 
typedef union
{
    uint16_t Value;                // ����֡д��ֵ
    struct
    {
        unsigned Data     : 14;    // �Ƕ�����
        unsigned Unused   : 2;     // δʹ������
    }bit_area;
    
}AS5048a_ANGR_t;

/**
  * @brief AS5048a���ݽṹ����
  */
typedef struct
{
    /* Ӳ���ײ���� */
    SPI_HandleTypeDef *hspi;
    
    GPIO_TypeDef*   NSSPort;        // ���Ŷ˿�
    uint16_t        NSSPin;         // ����λ��
    uint16_t        NSSPinNum;      // ���ź�
#ifdef USING_BITBAND              
    BIT_BAND_t      NSS_Write;      // IOдλ��
#endif
    /* ����ӿڲ��� */
    bool            isDebuging;     // ����Ƿ������״̬�����״̬�²�������÷�����ģʽ�µ��κκ�����
    bool            isSpeedReliable;/* ����ٶ������Ƿ�ɿ������������ٶ����ݲ��ɿ���������λ���㡣
                                       ����λ��һʱ��Speed�н��������µĿɿ����ݡ�
                                       ���ٶ����ݲ��ɿ�ʱ������ȡʵʱλ�á����������µĻ�׼�Ƕȡ�
                                       ��ͨ�����˱�־���������״̬���м�ء�                     */
    float           BaseAngle;      // ��׼�Ƕȣ��ǴӴ����������ĽǶ�ֵ���Ƕ��ơ�
    float           RealAngle;      // ʵʱ�Ƕȣ����ٶȺͻ�׼�Ƕȹ��Ƴ����ĵ��ýӿڼ���ʱ��˲ʱ��λ�ã��Ƕ��ơ�
    float           RawSpeed;       // ԭʼת��
    float           Speed;          // ת�٣������ζ�ȡλ���Լ���ʱ���������������λRPM��ӦΪ��ֵ��
    float           MaxRPM;         // Ԥ�����ת�٣����ڹ����ٶ����ݿɿ��Ժͽ�λ����Ȧ���䡣
    AS5048a_EFR_t   ErrorFlags;     // Error Flag�Ĵ���ֵ��
    AS5048a_AGCR_t  AGCR;           // AGC�Ĵ���ֵ
    AS5048a_MAGR_t  MAGR;           // Magnet�Ĵ���ֵ
    
    /* ͨ�Ų��� */
    uint32_t        FrameTime;      // һ��ͨ�����ںķѵ�ʱ�䣬��λns (�뾭ʵ���ã���ͬϵͳ��Ƶ��ʱ�䲻ͬ)��
    AS5048a_State_t State;          // ���AS508����״̬�������������ͻ����־��
    
    /* ���ڶ�ȡ��ز��� */
    uint16_t        LastCMD;        // �ϴ�ִ�еĶ�ȡָ������Ƕ��Ĵ���������������ǣ���
    uint16_t        PeriodErrorCnt; // ���ڶ�ȡ������������ͨ�Ź��ϼ�������
    uint16_t        ErrorMax;       // ���� ��������ͨ�Ŵ��� ����������
    uint16_t        TempRxBuffer;   // ����DMA�������ʱ���ݻ�������
    bool            isDataInvalid;  /* ����ϴ�ͨ���е������Ƿ���ţ����ظ�֡�еĴ����־��һʱ����λ��һ��
                                       ���´�ͨ���У���������������־��ʹ��ͨ�Żָ������� */
    GlobalTime_t    ReadMoment;     // ��¼�ϴζ�ȡλ�õ�ʱ�̡�
    
    SlidAveFilter_t SpeedFilter;    // ����ƽ��ֵ�˲�������ٶ����������������
}AS5048a_t, *pAS5048a_t;

/* Exported variable --------------------------------------------------------*/
extern AS5048a_t AS5048a;

/* Exported functions --------------------------------------------------------*/
bool AS5048a_Init( pAS5048a_t pDev, float MaxRPM, 
                   GPIO_TypeDef* NSSPort, uint16_t NSS_Pin, 
                   SPI_HandleTypeDef* hspi, uint16_t FrameTime, uint16_t MaxErrorTimes );
bool AS5048a_DiagnoseConditions(pAS5048a_t pDev);

void AS5048a_UpdateAngleSpeed_DMA(pAS5048a_t pDev);
void AS5048a_DMARx_Callback(pAS5048a_t pDev);

bool AS5048a_GetSpeed(pAS5048a_t pDev, float *pSpeed);
bool AS5048a_GetRealTimeAngle(pAS5048a_t pDev, float *pAngle);

#ifdef AS5048A_DEBUG
void AS5048a_Test(void);
#endif /* AS5048A_DEBUG */

#endif
