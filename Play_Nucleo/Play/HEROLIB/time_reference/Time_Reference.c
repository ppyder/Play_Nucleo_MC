/**
 ******************************************************************************
 * @file    TimeReference.c
 * @brief   �м�㣺����һ��ר�õĶ�ʱ��ʱ���ṩ��׼��ʱ��ο�.
 *          ����֧�ּ����ٶȡ�ʵʱλ�ã��ṩns��ʱ���׼(оƬ�����ṩ����󾫶�)��
 * @version 1.0 ʾ���汾
 * @author  ���ෲ
 * @contact 17863107058(�ֻ�)   942041771(qq)
 * @date    2020/04/03
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
#include "Time_Reference.h"

/**
  * @brief ʱ���׼����ṹ�����Ͷ���
  */
typedef struct
{
    TIM_HandleTypeDef* htim;        // ��ʱ�����
    
    int64_t            CarryNum;    // ��ʱ���������λ(�ڶ�ʱ�ж�������)
    volatile uint16_t* pCNT;        // Ŀ�궨ʱ����CNT�Ĵ�����ַ 
    
    float              Tick_T;      // ����ʱ��tick������(һ��tick�೤ʱ��)
    float              Toatal_s;    // ���á���ȡʱ�䡱�����Դ˱������и��£���λs(��)
    
    
}TimeRef_t, *pTimeRef_t;

/* Public variables --------------------------------------------------------*/
TimeRef_t TimeRef = // ʱ���ṹ��
{
    .htim = NULL,
    .CarryNum = 0,
    .pCNT = NULL,
    .Tick_T = 1,
    .Toatal_s = 0,
};

/**
 * @brief	    ��ʼ��ʱ���׼
 * @param[in]	htim     - ����ʵ�ֹ��ܵĶ�ʱ�����
 * @param[in]	Freq_MHz - ���붨ʱ���ġ�δ��Ԥ��Ƶ������ʱ��Ƶ��
 * @retval	    ��ʼ���Ƿ�ɹ����ɹ�Ϊtrue.
 * @note        1. ��ʼ�����Ϊ����ʱ�������Ƶ�����ϼ���������ʱ�������̴�����ʱ�жϡ�
 *              2. ��ʹ�õĶ�ʱ���ڱ���������ڼ䲻�������á�
 */
bool TimeRef_Init(TIM_HandleTypeDef* htim, uint32_t Freq_MHz)
{
    //��ȫ���
    if( NULL == htim || HAL_TIM_STATE_READY != htim->State  || 
        !IS_TIM_REPETITION_COUNTER_INSTANCE(htim->Instance) || __HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) 
    { return false; }
    
    //����������
    TimeRef.htim = htim;
    
    //��¼��Ҫ�õ��ļĴ����ĵ�ַ
    TimeRef.pCNT = (volatile uint16_t*)&(htim->Instance->CNT);
    
    //����һ��ʱ��tick��ȥ�೤ʱ��
    TimeRef.Tick_T = (1.0 / Freq_MHz) * 1e-6;
    
    //����URSλ��ʹ�ö�ʱ�������жϽ��ɼ���������¼���������ģʽ���������ʱ�������жϣ�
    htim->Instance->CR1 |= TIM_CR1_URS;
    
    //������Ԥ��Ƶ���ӣ�ʹ�������ʱ���ź�Ƶ�����
    __HAL_TIM_SET_PRESCALER(htim, 0);
    
    //����ARRֵ
    __HAL_TIM_SET_AUTORELOAD(htim, TIMEREF_CNT_MAX - 1);
        
    //����CNTֵ
    __HAL_TIM_SET_COUNTER(htim, 0);
    
    return true;
}

/**
 * @brief	    ��ʼ��ʱ
 * @param   	None.
 * @retval	    ������ʱ�Ƿ�ɹ����ɹ�Ϊtrue.
 * @note        1. ���ü�ʱǰ��λ������
 */
bool TimeRef_Start(void)
{
    if(NULL == TimeRef.htim) { return false; }
    
    //��λ����
    TimeRef_Clear();
    
    //������ʱ
    return (HAL_OK == HAL_TIM_Base_Start_IT(TimeRef.htim));
}

/**
 * @brief	    ��λ������(���¿�ʼ����)
 * @param   	None.
 * @retval	    ��λ�Ƿ�ɹ����ɹ�Ϊtrue.
 */
bool TimeRef_Clear(void)
{  
    if(NULL == TimeRef.htim) { return false; }
        
    //��ս�λ����
    TimeRef.CarryNum = 0;
    
    //����һ�������¼�������Ĵ���
    HAL_TIM_GenerateEvent(TimeRef.htim, TIM_EVENTSOURCE_UPDATE);
    
    return true;
}

/**
 * @brief	    ��ĳͨ��ʱ��ṹ�л�ȡʱ��
 * @param[out] 	pTime   - ���������ݽ�����ŵĽṹ���ַ.
 * @retval	    �����Ƿ�ɹ����ɹ�Ϊtrue
 */
bool TimeRef_GetPreciseTime(pGlobalTime pTime)
{
    if(NULL == pTime) { return false; }

    //��λֵ
    pTime->CarryNum = TimeRef.CarryNum;
    
    //tick��
    pTime->Ticks = (*TimeRef.pCNT);                       
    
    return true;
}

/**
 * @brief	    ����ʵʱʱ��
 * @param   	None.
 * @retval	    ʵʱʱ�䣬��λΪ�롣
 * @note        1. �������������ڱ�����ĳ�Ա @Total_s�У���λΪ�롣
 */
float TimeRef_GetTotal(void)
{
    if(NULL == TimeRef.htim) { return 0; }
    
    //�ѵݼ��ļĴ���ֵ���������ŵ�ʱ��
    uint32_t TempCNT = (*TimeRef.pCNT);
    
    //����Ӽ�ʱ��ʼ���ŵ���ʱ��(ע�⣺ʹ��64λ����ʱ��Ҫע����ʽ�ڵ�����ת������ΪĬ����ʹ��int32�����)
    TimeRef.Toatal_s = TimeRef.Tick_T * 
                       (TimeRef.CarryNum * (int64_t)TIMEREF_TOTALCNT + (int64_t)TempCNT);
    
    return TimeRef.Toatal_s;
}

/**
 * @brief	    ��ĳͨ��ʱ��ṹ�л�ȡʱ��
 * @param[in]  	pTime  - ��ת���ṹ���ַ.
 * @retval	    ת���������λΪ�롣
 */
float TimeRef_GetTotal_FromGlobalTime(pGlobalTime pTime)
{
    float result = 0;
    
    if(NULL == pTime) { return 0; }
    
    //����ʱ��
    result = TimeRef.Tick_T * (pTime->CarryNum * (int64_t)TIMEREF_TOTALCNT + (int64_t)pTime->Ticks);
    
    return result;
}

/**
 * @brief	    ��ȷ��ʱ�����
 * @param[in]  	pLeft  -  ���ı�����.
 * @param[in]  	pRight  - ���ļ���.
 * @retval	    ת���������λΪ�롣
 */
float TimeRef_TimeMinus(pGlobalTime pLeft, pGlobalTime pRight)
{
    int64_t Total_ticks = 0;
    
    //��ȫ���
    if(NULL == pLeft || NULL == pRight) { return 0; }
    
    Total_ticks = (pLeft->CarryNum - pRight->CarryNum) * (int64_t)TIMEREF_TOTALCNT + 
                  (int64_t)pLeft->Ticks - (int64_t)pRight->Ticks;
    
    
    return (Total_ticks * TimeRef.Tick_T );
}

/**
 * @brief	    �ص�������
 * @param   	None.
 * @retval	    �����Ƿ�ɹ����ɹ�Ϊtrue.
 * @note        1. �����������ڶ�Ӧ�Ķ�ʱ���ж��б�����Եص��á�
 *              2. ������ԡ��ĺ����ǣ�Ӧ����������������ʱ���ж���Ӧ����Ϊ��ʱ�ص�����������ͬһ����
 */
void TimeReference_Callback(void)
{
    //��λ����ֵ����
    TimeRef.CarryNum++;
    return;
}

/************************ (C) COPYRIGHT HITwh Excellent Robot Organization(HERO). *****END OF FILE****/
