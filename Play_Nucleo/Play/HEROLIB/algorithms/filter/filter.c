/**
 ******************************************************************************
 * @file    filter.c
 * @brief   ����ײ㣺�˲�����صĻ��������ͷ���.
 *              + �ṩ��״̬�˲�����صķ���
 *                  ++ ״̬�˲�����ʼ��
 *                  ++ �˲�����״̬�Ļ�ȡ����
 *                  ++ ��λ�˲��ĺ���
 *              + �ṩ�������˲�����صķ���
 *                  ++ һ�׵�ͨ�˲���
 *                  ++ ����ƽ��ֵ�˲�����ʼ��
 *                  ++ ����ƽ��ֵ�˲���
 * @version 1.0 ʾ���汾
 * @author  ���ෲ
 * @contact 17863107058(�ֻ�)   942041771(qq)
 * @date    2018/10/18
 *
 ******************************************************************************
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (C) HITwh Excellent Robot Organization(HERO). 2015-2018.</center></h2>
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "filter.h"
#include <string.h>     //ʹ�����е�NULL��

/* Private constants --------------------------------------------------------*/
/**
  * @brief �������� �˲����� ���ھ����˲�λ��
  */
static const uint32_t FILTER_MASK_CODE[33] = 
{
    0X00,
    0x01,       0x03,       0x07,       0x0F,
    0x1F,       0x3F,       0x7F,       0xFF,
    0x1FF,      0x3FF,      0x7FF,      0xFFF,
    0x1FFF,     0x3FFF,     0x7FFF,     0xFFFF,
    0x1FFFF,    0x3FFFF,    0x7FFFF,    0xFFFFF,
    0x1FFFFF,   0x3FFFFF,   0x7FFFFF,   0xFFFFFF,
    0x1FFFFFF,  0x3FFFFFF,  0x7FFFFFF,  0xFFFFFFF,
    0x1FFFFFFF, 0x3FFFFFFF, 0x7FFFFFFF, 0xFFFFFFFF
};

/**
 * @brief	    ״̬�˲�����ʼ��
 * @param[out]  pFilter     ָ�����ʼ���ṹ���ָ��
 * @param[in]   CheckPeriod ������� 
 * @param[in]   MaskBits    �˲�λ��
 * @retval	    ��ʼ�������bool����ʧ�ܷ���false.
 */
bool StateFilterInit(pStateFilter pFilter, uint32_t CheckPeriod, uint32_t MaskBits)
{
    //��ȫ���
    if(NULL == pFilter) { return false; }
    
    pFilter->CheckPeriod = CheckPeriod; //���ɨ������
    pFilter->MaskBits    = MaskBits;    //�趨�˲�����
    pFilter->History     = 0x00;        //�����ʷֵ
    pFilter->Jumping     = NO_JUMPING;  //�������״̬
    pFilter->State       = false;       //Ĭ�ϳ�ʼ״̬
    
    return true;
}

/**
 * @brief	    �����˲�����
 * @param[out]  pFilter     ָ�򱾴β����˲����˲����ṹ��
 * @param[in]   NowState    ��ǰ���˲��������Ķ����˲̬����״̬
 * @retval	    ��ǰ��״̬��bool������Ч״̬Ϊfalse.
 */
bool StateFilter(StateFilter_t *pFilter, bool NowState)
{
    uint32_t temp = 0;
    
    if(pFilter == NULL)
        return false;
    
    NowState &= FILTER_MASK_CODE[1];    //ȡĩλ�����Ƿ������Σ��������С
    
    pFilter->History <<= 1;
    pFilter->History |= NowState;       //������ʷ
    
    //ȡ���µ� �˲�λ�� λ
    temp = (pFilter->History) & FILTER_MASK_CODE[pFilter->MaskBits];
    
    if(temp == FILTER_MASK_CODE[pFilter->MaskBits])    //����ע��λ����1��˵��������ν���ߵ�ƽ״̬��
    {
        if(pFilter->State != true)
        {
            pFilter->Jumping = RISING_EDGE; //��Ǵ���������
            pFilter->State = true;
        }
    }
    else if(temp == FILTER_MASK_CODE[0])               //���롰�͵�ƽ״̬��
    {
        if(pFilter->State != false)
        {
            pFilter->Jumping = FALLING_EDGE; //��Ǵ����½���
            pFilter->State = false;
        }
    }
    
    return pFilter->State;
}

/**
 * @brief	    ��ȡ����״̬
 * @param[in]   pFilter     ָ��������ȡ״̬���˲����ṹ��
 * @note        ˵����״̬�˲��������߼�״̬���˲����ã������ش�����߼�0��Ϊ���߼�1��
 * @retval	    ����״̬��JumpingState��,��Ч״̬ΪNO_JUMPING.
 */
JumpingState GetJumpingState(StateFilter_t *pFilter)
{
    JumpingState result = NO_JUMPING;
    
    if(pFilter == NULL)
        return NO_JUMPING;
    
    result = pFilter->Jumping;
    
    pFilter->Jumping = NO_JUMPING;      //����һ�α�ʹ֮ʧЧ����ֹ�ظ�������Ӧ����
    
    return result;
}

//Z�任���ӡ���ĸ������
//ע������������Ƶ��Ϊ1000Hz�������ʹ��˫���Ա任����������ġ�
FillterParam_t Fill_Params[] = 
{
    {{0.0861, 0.0861}, {1.0000, -0.8277}, 0, 0}, //3Hz��ֹ
};

/**
 * @brief	    һ�׵�ͨ�����˲�����ע�����Ƶ��
 * @param[in]   Value     ��ǰ����ֵ
 * @param[in]   pParams   ָ���Ӧ���˲��������ṹ��
 * @retval	    �˲�����������
 */
double DataFillter(double Value, FillterParam_t *pParams)
{
    double result;
    
    result = (1 / pParams->den[0]) 
            * (pParams->num[0] * Value + pParams->num[1] * pParams->LastInput 
                - pParams->den[1] * pParams->LastOutput);
    
    //��¼��ʷֵ
    pParams->LastInput = Value;
    pParams->LastOutput = result;
    
    return result;
}   

/**
 * @brief	    ����ƽ��ֵ�˲��ṹ���ʼ��
 * @param[out]  pFilter    ����ʼ���˲����ĵ�ַ
 * @param[in]   pBuffer    ���ݽ��ջ�������ַ
 * @param[in]   BufferSize ����������
 * @param[in]   InitValue  ��ʼ��ֵ
 * @retval	    None.
 */
void SlidingAveFilterInit(SlidAveFilter_t *pFilter, float *pBuffer, uint32_t BufferSize, float InitValue)
{
    if(NULL == pFilter || NULL == pBuffer) { return; }
    
    pFilter->pDataBuffer = pBuffer;
    pFilter->DataSize = BufferSize;
    
    pFilter->Sum = 0;
    pFilter->DataIndex = 0;
    
    SetSlidingAveBuffer(pFilter, InitValue);
    
    return;
}

/**
 * @brief	    ����ƽ��ֵ�˲�
 * @param[in]   pFilter    ��Ӧ�˲����ĵ�ַ
 * @param[in]   NewData    �²�������
 * @retval	    �˲�������ֵ.
 * @note    ����ѭ������ʵ������ ����ƽ��ֵ�˲�
 *          Ĭ���䱾��������ģ���˶�����ĩ��ַ��ȣ���������Ϊָ��ǰһλ���������Ϊ��ָ���һλ
 */
float SlidingAveFilter(SlidAveFilter_t *pFilter, float NewData)
{
    float result = 0;
    
    if(NULL == pFilter || NULL == pFilter->pDataBuffer) { return 0; }
    
    //���������ȥ����ɵ�����
    pFilter->Sum -= pFilter->pDataBuffer[pFilter->DataIndex];
    
    //�������ݱ����ڶ�����
    pFilter->pDataBuffer[pFilter->DataIndex] = NewData;
    
    //�������ݼ����������
    pFilter->Sum += NewData;
    
    //���ƽ��ֵ�˲����
    result = pFilter->Sum / pFilter->DataSize;
    
    //����ѭ��ָ��
    pFilter->DataIndex++;

    //ѭ��ָ��
    if(pFilter->DataIndex >= pFilter->DataSize)
    {
        pFilter->DataIndex = 0;
    }
    
    return result;
}

/**
 * @brief	    ���û���ƽ��ֵ�˲������ݻ�����
 * @param[in]   pFilter    ��Ӧ�˲����ĵ�ַ
 * @param[in]   Value      �趨ֵ
 * @retval	    �˲�������ֵ.
 * @note    ����������ÿһ�������趨ΪԤ��ֵ����ʹ�ۼ�����Ϊ���ǵĺ͡�
 */
void SetSlidingAveBuffer(SlidAveFilter_t *pFilter, float Value)
{
    if(NULL == pFilter || NULL == pFilter->pDataBuffer) { return; }
    
    //�����ۼ���
    pFilter->Sum = 0;
    
    //���¸�ֵ
    for(int i = 0; i < pFilter->DataSize; i++)
    {
        pFilter->pDataBuffer[i] = Value;
        pFilter->Sum += Value; 
    }
    
    return;
}

/************************ (C) COPYRIGHT HITwh Excellent Robot Organization(HERO). *****END OF FILE****/
