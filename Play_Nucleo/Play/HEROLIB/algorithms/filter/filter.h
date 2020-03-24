/**
 ******************************************************************************
 * @file    filter.h
 * @brief   ���������˲��������Ͷ���ͺ�������.
 *              + ����״̬�˲�������ض��������
 *                  ++ ����״̬�˲��������ص�ö�����Ͷ���
 *                  ++ ����״̬�˲��������ݽṹ����
 *                  ++ ����״̬�˲����Ĳ�����������
 *              + ���������˲�������ض��������
 *                  ++ ���������˲��������ݽṹ����
 *                  ++ ���������˲����Ĳ�����������
 ******************************************************************************
 *
 * Copyright (C) HITwh Excellent Robot Organization(HERO). 2015-2018. All rights reserved.
 *
 * @version 1.0 ʾ���汾
 * @author  ���ෲ
 * @contact 17863107058(�ֻ�)   942041771(qq)
 * @date    2018/10/10
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FILTER_H
#define FILTER_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/** @addtogroup HeroLibrary HERO�ŶӴ����
  * @{
  */

/** @addtogroup Low_Level_Library Low Level Library(�ײ��)
  * @{
  */

/** @addtogroup Universal_Software_Support Universal(���ͨ��֧��)
  * @{
  */
  
/** @addtogroup Universal_filter fillter�˲���
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup Filter_Exported_Types Filter��������
  * @{
  */

/**
  * @brief �˲�������״̬ ö�ٶ���
  */
typedef enum 
{
    NO_JUMPING,     //!< ������
    RISING_EDGE,    //!< ����������
    FALLING_EDGE,   //!< �½�������
    
}JumpingState;

/**
  * @brief һ�׵�ͨ�����˲�����ѡ���� ö�ٶ���
  */
enum FillterParams
{
    Fill_3Hz,     //3Hz��ֹ
    Fill_Num
};   

/**
  * @brief ״̬�˲��� �ṹ����
  */
typedef struct
{
    uint32_t        CheckPeriod;    //!< ������ڣ���λΪus��
    
    uint32_t        History;	    //!< ����¼����ʷֵ
    
    uint8_t         MaskBits;       //!< �˲�λ��
    
	bool            State;		    //!< �˲���ȷ�ϵ�״̬
    
    JumpingState    Jumping;        //!< ������������״̬
		
}StateFilter_t, *pStateFilter;

/**
  * @brief һ�׵�ͨ�����˲��� �ṹ����
  */
typedef struct 
{
    double num[2];         //!< Z�任���˲������崫���ķ�������
    double den[2];         //!< Z�任���˲������崫���ķ�ĸ����
    
    double LastInput;      //!< ��¼�ϴ��˲���������ֵ
    double LastOutput;     //!< ��¼�ϴ��˲��������
    
}FillterParam_t;
/**
  * @}
  */

/**
  * @brief ����ƽ��ֵ�˲� �ṹ����
  */
typedef struct 
{
    uint32_t DataSize;     //!< ���ݴ��ڴ�С
    float*   pDataBuffer;  //!< ѭ���������ݻ�������ַ���䳤��Ӧ�봰�ڴ�С��ȣ�
    
    uint32_t DataIndex;    //!< ѭ�����������±�
    float    Sum;          //!< �����
    
}SlidAveFilter_t;
/**
  * @}
  */


/* Exported variable --------------------------------------------------------*/
/** @addtogroup Filter_Variable Filter��������
  * @{
  */

extern FillterParam_t Fill_Params[Fill_Num];
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup Filter_Functions Filter������������
  * @{
  */
bool StateFilterInit(pStateFilter pFilter, uint32_t CheckPeriod, uint32_t MaskBits);
bool StateFilter(StateFilter_t *pFilter, bool NowState);
JumpingState GetJumpingState(StateFilter_t *pFilter);

double DataFillter(double Value, FillterParam_t *Params);

void SlidingAveFilterInit(SlidAveFilter_t *pFilter, float *pBuffer, uint32_t BufferSize);
float SlidingAveFilter(SlidAveFilter_t *pFilter, float NewData);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* FILTER_H */

/************************ (C) COPYRIGHT HITwh Excellent Robot Organization(HERO). *****END OF FILE****/
