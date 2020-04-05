/**
 ******************************************************************************
 * @file    TinyDelay.c
 * @brief   �м�㣺����һ��ר�õĶ�ʱ��ʱ���ṩ��׼�Ķ�ʱ�ӳ�.
 * @version 1.0 ʾ���汾
 * @author  ���ෲ
 * @contact 17863107058(�ֻ�)   942041771(qq)
 * @date    2020/03/31
 *
 ******************************************************************************
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (C) HITwh Excellent Robot Organization(HERO). 2015-2020.</center></h2>
 *
 ******************************************************************************
 */
 
/* Includes ------------------------------------------------------------------*/
#include "TinyDelay.h"

/* Public variables --------------------------------------------------------*/
TinyDelay_t TinyDelay = 
{
    .DelayTime = 0,
    .State     = DELAY_STATE_RESET,
};

/**
 * @brief	  ��ʼ��С��ʱϵͳ
 * @param[in] htim     - ����ʵ����ʱ�Ķ�ʱ�����
 * @param[in] Freq_MHz - ���붨ʱ���ġ�δ��Ԥ��Ƶ������ʱ��Ƶ��
 * @retval    ��ʼ���Ƿ�ɹ����ɹ�Ϊtrue.
 * @note      ��ʼ�����Ϊ����ʱ��ʱ��������ÿ0.5usһ����
 *            ����������Ƿ��򾭹�Ԥ��Ƶ�޷����2Mʱ��ʱ����false��
 */
bool TinyDelayInit(TIM_HandleTypeDef *htim, uint32_t Freq_MHz)
{  
    TinyDelay.htim = htim;
    TinyDelay.DelayTime = 0;
    
    //��鶨ʱ���Ƿ����
    if(NULL == htim || HAL_TIM_STATE_READY != htim->State) { return false; }
    
    //����Ƿ���ʹ�������ź�Ƶ��Ϊ2M
    if(0 != (Freq_MHz % 2)) { return false; }
    
    //������Ԥ��Ƶ���ӣ�ʹ�������ʱ���ź�Ƶ��Ϊ2M
    __HAL_TIM_SET_PRESCALER(htim, (Freq_MHz/2 - 1));
    
    TinyDelay.State = DELAY_STATE_READY;
    
    return true;
}

/**
 * @brief	    ��ȷ��С��ʱ����
 * @param[in]	Time   - ��ʱʱ�䣬��λΪ0.5us�����磺Time = 2������ʱ1us��
 * @retval	    ��ʱ�Ƿ�ɹ����ɹ�Ϊtrue.
 * @note        1. ֧�ֵ���ʱ��Χ��0.5us - 32768us(32.768ms)
 *              2. ��һ����ʱ����ǰ��������ʼ��һ����ʱ�����˷����ǡ��������á��ġ�
 */
bool Tiny_Delay(uint16_t Time)
{
    //����ʱ��������
    if(HAL_TIM_STATE_READY != TinyDelay.htim->State || 
       DELAY_STATE_READY != TinyDelay.State) { return false; }
    
    //��¼��ʱʱ��
    TinyDelay.DelayTime = Time / 2.0;
       
    //���ö�ʱ������ֵ
    __HAL_TIM_SET_AUTORELOAD(TinyDelay.htim, Time);
    
    //���״̬
    TinyDelay.State = DELAY_STATE_BUSY;
       
    //������ʱ
    HAL_TIM_Base_Start_IT(TinyDelay.htim);
       
    //�ȴ���ʱ����
    while(DELAY_STATE_BUSY == TinyDelay.State);
    
    return true;
}

/**
 * @brief	��ֹ��ʱ
 * @param	None.
 * @retval	None.
 * @note    1. ������ǰ������ʱ��
 *          2. ��ʹ�����ȼ����ߵ��ж�ʵ�ִ˹��ܡ�
 */
void Tiny_Delay_Abort(void)
{
    HAL_TIM_Base_Stop_IT(TinyDelay.htim);
}

/**
 * @brief	��ʱ�ص�������������ʱ������־
 * @param	None.
 * @retval	None.
 * @note    1. �˺���������Ŀ�궨ʱ����ʱ�ص������б����á�
 */
void Tiny_DelayCallback(void)
{
    TinyDelay.State = DELAY_STATE_READY;
}

/************************ (C) COPYRIGHT HITwh Excellent Robot Organization(HERO). *****END OF FILE****/
