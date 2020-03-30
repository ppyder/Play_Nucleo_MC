
#ifndef AS5048A_H
#define AS5048A_H

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
  * @brief AS5048���ݽṹ����
  */
typedef struct
{
    SPI_HandleTypeDef *hspi;
    
    GPIO_TypeDef*   Port;          // ���Ŷ˿�
    uint16_t        Pin;           // ����λ��
    uint16_t        PinNum;        // ���ź�
#ifdef USING_BITBAND              
    BIT_BAND_t      NSS_Write;     // IOдλ��
#endif   
    
}AS5048_t, *pAS5048_t;


//������ָ���д��־λΪ��14λ
#define AS5048_WRITE     (0x00<<14)
#define AS5048_READ      (0x01<<14)

//У��λ
#define AS5048_PAR_SET   (0x01<<15) // żУ��λ��һ
#define AS5048_PAR_RESET (0x00<<15) // żУ��λ����


//�Ĵ�����ַ����
#define AS5048_REG_NOP     0x0000   // ����Ĵ��������ڶ�ȡNOPָ�ֻ����
#define AS5048_REG_EF      0x0001   // �����־�Ĵ��������ڶ�ȡ�����־��ֻ������ȡ�������
#define AS5048_REG_PC      0x0003   // ��̿��ƼĴ��������ڽ��й����Կ��Ʋ������ɶ�д��
#define AS5048_REG_ZPH     0x0016   // ��λ�߼Ĵ��������ڴ洢�Ƕ���λ�߰�λ���ɶ�д��
#define AS5048_REG_ZPL     0x0017   // ��λ�ͼĴ��������ڴ洢�Ƕ���λ�Ͱ�λ���ɶ�д��
#define AS5048_REG_AGC     0x3FFD   // ��Ϻ��Զ�����Ĵ�����������Ϲ����������ṩ���档ֻ����
#define AS5048_REG_MAG     0x3FFE   // �ų���Ϣ�Ĵ��������ڴ洢�ų�ǿ����Ϣ��ֻ����
#define AS5048_REG_ANGLE   0x3FFF   // �Ƕ���Ϣ�Ĵ��������ڴ洢�Ƕ���Ϣ��ֻ����

/**
  * @brief REG - Error Flag �����־�Ĵ��� -- 0x0001
  */
#define AS5048_CMD_READEF 	      (AS5048_PAR_RESET | AS5048_READ | AS5048_REG_EF)
#define AS5048_EF_PARITY_ERROR 	  (0X01<<2) // ��żУ�����
#define AS5048_EF_COMMAND_INVALID (0X01<<1) // ָ��Ƿ�
#define AS5048_EF_FRAMING_ERROR   (0X01<<0) // ֡����

/**
  * @brief REG - Programming Control ��̿��ƼĴ��� -- 0x0003
  */
#define AS5048_CMD_READPC 	  (AS5048_PAR_SET | AS5048_READ | AS5048_REG_PC)
#define AS5048_PC_VERIFY 	  (0X01<<6)  // У�飨��OTP���ݼ��ص������޸ĵ���ֵ�Ƚ���������ڲ��Ĵ����У�
#define AS5048_PC_BURN 	      (0X01<<3)  // ��¼�������ڲ����Զ���̳���
#define AS5048_PC_PRO_EN 	  (0X01<<0)  // ���ʹ��λ��ֻ����������OTP�Ĵ����ſɱ�д��

/**
  * @brief REG - OTP: Zero Position High ��λ���ݸ߼Ĵ��� -- 0x0016
  * @note  OTP(One Time Programmable) ��һ�α�̼Ĵ�����
  *        ��8λ��Ч��
  */
#define AS5048_CMD_READZPH 	  (AS5048_PAR_RESET | AS5048_READ | AS5048_REG_ZPH)
#define AS5048_ZPH_MASK 	  (0xFF)     // ����������

/**
  * @brief REG - OTP: Zero Position Low ��λ���ݵͼĴ��� -- 0x0017
  * @note  OTP(One Time Programmable) ��һ�α�̼Ĵ�����
  *        ��6λ��Ч��
  */
#define AS5048_CMD_READZPL 	  (AS5048_PAR_SET | AS5048_READ | AS5048_REG_ZPL)
#define AS5048_ZERO_L_MASK 	  (0x3F)     // ����������

/**
  * @brief REG - Diagnostics & Automatic Gain Control ��Ϻ��Զ�������ƼĴ��� -- 0x3FFD
  * @note  ��COMP_H/COMP_Lλ��һʱ������ȥ�鿴Magnitude�Ĵ����Բ��������
  */
#define AS5048_CMD_READAGC 	  (AS5048_PAR_RESET | AS5048_READ | AS5048_REG_AGC)
#define AS5048_AGC_COMP_H 	  (0x01<<11) // �Ƚϣ�ǿ�����ų���ǿʱ��λ��һ��
#define AS5048_AGC_COMP_L 	  (0x01<<10) // �Ƚϣ��������ų�����ʱ��λ��һ��
#define AS5048_AGC_COF   	  (0x01<<9)  /* (Cordic Overflow)����λ��һ��ʾ�ڲ���CORDIC�㷨�г�����Χ�Ĵ���
                                            ��λ��һʱ���ǶȺʹų��������ݲ����ã�PWM����������ϴε���Ч���״̬��*/
#define AS5048_AGC_OCF   	  (0x01<<8)  /* (Offset Compensation Finished)����λ��һ��ʾƫ�Ʋ����㷨�Ѿ���ɡ�
                                            ��λһ���ϵ��ͻ���һ��*/
#define AS5048_AGC_DATAMASK   (0x7F)     // ��������������(�����ݣ�0��ʾǿ�ų���255��ʾ���ų�)

/**
  * @brief REG - Magnitude �ų����ݼĴ��� -- 0x3FFE
  * @brief REG - Angle     �Ƕ����ݼĴ��� -- 0x3FFF
  */ 
#define AS5048_CMD_READMAG 	  (AS5048_PAR_RESET | AS5048_READ | AS5048_REG_MAG)
#define AS5048_CMD_READANGLE  (AS5048_PAR_SET   | AS5048_READ | AS5048_REG_ANGLE)
#define AS5048_DATAMASK       (0x3FFF)   // ����֡�к�14λ�������ݣ�ǰ��λ�ǹ�����Ϣ

void AS5048a_Test(void);

#endif
