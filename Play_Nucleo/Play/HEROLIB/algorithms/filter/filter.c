/**
 ******************************************************************************
 * @file    filter.c
 * @brief   软件底层：滤波器相关的基本操作和方法.
 *              + 提供了状态滤波器相关的方法
 *                  ++ 状态滤波器初始化
 *                  ++ 滤波跳变状态的获取函数
 *                  ++ 按位滤波的函数
 *              + 提供了数据滤波器相关的方法
 *                  ++ 一阶低通滤波器
 *                  ++ 滑动平均值滤波器初始化
 *                  ++ 滑动平均值滤波器
 * @version 1.0 示例版本
 * @author  杨亦凡
 * @contact 17863107058(手机)   942041771(qq)
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
#include <string.h>     //使用其中的NULL宏

/* Private constants --------------------------------------------------------*/
/**
  * @brief 常量数表 滤波掩码 用于决定滤波位数
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
 * @brief	    状态滤波器初始化
 * @param[out]  pFilter     指向待初始化结构体的指针
 * @param[in]   CheckPeriod 检测周期 
 * @param[in]   MaskBits    滤波位数
 * @retval	    初始化结果（bool），失败返回false.
 */
bool StateFilterInit(pStateFilter pFilter, uint32_t CheckPeriod, uint32_t MaskBits)
{
    //安全检查
    if(NULL == pFilter) { return false; }
    
    pFilter->CheckPeriod = CheckPeriod; //标记扫描周期
    pFilter->MaskBits    = MaskBits;    //设定滤波窗口
    pFilter->History     = 0x00;        //清空历史值
    pFilter->Jumping     = NO_JUMPING;  //清空跳变状态
    pFilter->State       = false;       //默认初始状态
    
    return true;
}

/**
 * @brief	    输入滤波函数
 * @param[out]  pFilter     指向本次参与滤波的滤波器结构体
 * @param[in]   NowState    当前该滤波器所关心对象的瞬态采样状态
 * @retval	    当前的状态（bool），无效状态为false.
 */
bool StateFilter(StateFilter_t *pFilter, bool NowState)
{
    uint32_t temp = 0;
    
    if(pFilter == NULL)
        return false;
    
    NowState &= FILTER_MASK_CODE[1];    //取末位，将非法输入的危害降到最小
    
    pFilter->History <<= 1;
    pFilter->History |= NowState;       //更新历史
    
    //取最新的 滤波位数 位
    temp = (pFilter->History) & FILTER_MASK_CODE[pFilter->MaskBits];
    
    if(temp == FILTER_MASK_CODE[pFilter->MaskBits])    //被关注的位都是1，说明进入所谓“高电平状态”
    {
        if(pFilter->State != true)
        {
            pFilter->Jumping = RISING_EDGE; //标记触发上升沿
            pFilter->State = true;
        }
    }
    else if(temp == FILTER_MASK_CODE[0])               //进入“低电平状态”
    {
        if(pFilter->State != false)
        {
            pFilter->Jumping = FALLING_EDGE; //标记触发下降沿
            pFilter->State = false;
        }
    }
    
    return pFilter->State;
}

/**
 * @brief	    获取跳变状态
 * @param[in]   pFilter     指向期望获取状态的滤波器结构体
 * @note        说明：状态滤波器仅起逻辑状态的滤波作用，上升沿代表从逻辑0变为了逻辑1。
 * @retval	    跳变状态（JumpingState）,无效状态为NO_JUMPING.
 */
JumpingState GetJumpingState(StateFilter_t *pFilter)
{
    JumpingState result = NO_JUMPING;
    
    if(pFilter == NULL)
        return NO_JUMPING;
    
    result = pFilter->Jumping;
    
    pFilter->Jumping = NO_JUMPING;      //读走一次便使之失效，防止重复触发相应操作
    
    return result;
}

//Z变换分子、分母向量，
//注意这是在运算频率为1000Hz的情况下使用双线性变换法计算出来的。
FillterParam_t Fill_Params[] = 
{
    {{0.0861, 0.0861}, {1.0000, -0.8277}, 0, 0}, //3Hz截止
};

/**
 * @brief	    一阶低通数据滤波器，注意调用频率
 * @param[in]   Value     当前采样值
 * @param[in]   pParams   指向对应的滤波器参量结构体
 * @retval	    滤波后的输出数据
 */
double DataFillter(double Value, FillterParam_t *pParams)
{
    double result;
    
    result = (1 / pParams->den[0]) 
            * (pParams->num[0] * Value + pParams->num[1] * pParams->LastInput 
                - pParams->den[1] * pParams->LastOutput);
    
    //记录历史值
    pParams->LastInput = Value;
    pParams->LastOutput = result;
    
    return result;
}   

/**
 * @brief	    滑动平均值滤波结构体初始化
 * @param[out]  pFilter    待初始化滤波器的地址
 * @param[in]   pBuffer    数据接收缓冲区地址
 * @param[in]   BufferSize 缓冲区长度
 * @retval	    None.
 */
void SlidingAveFilterInit(SlidAveFilter_t *pFilter, float *pBuffer, uint32_t BufferSize)
{
    pFilter->pDataBuffer = pBuffer;
    pFilter->DataSize = BufferSize;
    
    pFilter->Sum = 0;
    pFilter->DataIndex = 0;
}

/**
 * @brief	    滑动平均值滤波结构体初始化
 * @param[in]   pFilter    对应滤波器的地址
 * @param[in]   NewData    新采样数据
 * @retval	    滤波后的输出值.
 * @note    采样循环队列实现数据 滑动平均值滤波
 *          默认其本身就是满的，因此队列首末地址相等，最新数据为指针前一位，最旧数据为此指针后一位
 */
float SlidingAveFilter(SlidAveFilter_t *pFilter, float NewData)
{
    float result = 0;
    
    //从求和器中去除最旧的数据
    pFilter->Sum -= pFilter->pDataBuffer[pFilter->DataIndex];
    
    //将新数据保存在队列中
    pFilter->pDataBuffer[pFilter->DataIndex] = NewData;
    
    //将新数据加入求和器中
    pFilter->Sum += NewData;
    
    //获得平均值滤波结果
    result = pFilter->Sum / pFilter->DataSize;
    
    //自增循环指针
    pFilter->DataIndex++;

    //循环指针
    if(pFilter->DataIndex >= pFilter->DataSize)
    {
        pFilter->DataIndex = 0;
    }
    
    return result;
}

/************************ (C) COPYRIGHT HITwh Excellent Robot Organization(HERO). *****END OF FILE****/
