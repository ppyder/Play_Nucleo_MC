/**
 ******************************************************************************
 * @file    TinyDelay.c
 * @brief   中间层：基于一个专用的定时器时基提供精准的短时延迟.
 * @version 1.0 示例版本
 * @author  杨亦凡
 * @contact 17863107058(手机)   942041771(qq)
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
 * @brief	  初始化小延时系统
 * @param[in] htim     - 用于实现延时的定时器句柄
 * @param[in] Freq_MHz - 输入定时器的、未经预分频的外设时钟频率
 * @retval    初始化是否成功，成功为true.
 * @note      初始化结果为：定时器时基计数器每0.5us一增。
 *            当输入参数非法或经过预分频无法获得2M时钟时返回false。
 */
bool TinyDelayInit(TIM_HandleTypeDef *htim, uint32_t Freq_MHz)
{  
    TinyDelay.htim = htim;
    TinyDelay.DelayTime = 0;
    
    //检查定时器是否可用
    if(NULL == htim || HAL_TIM_STATE_READY != htim->State) { return false; }
    
    //检查是否能使得输入信号频率为2M
    if(0 != (Freq_MHz % 2)) { return false; }
    
    //更改其预分频因子，使得输入的时钟信号频率为2M
    __HAL_TIM_SET_PRESCALER(htim, (Freq_MHz/2 - 1));
    
    TinyDelay.State = DELAY_STATE_READY;
    
    return true;
}

/**
 * @brief	    精确的小延时函数
 * @param[in]	Time   - 延时时间，单位为0.5us。例如：Time = 2，即延时1us。
 * @retval	    延时是否成功，成功为true.
 * @note        1. 支持的延时范围：0.5us - 32768us(32.768ms)
 *              2. 当一次延时结束前，不允许开始新一次延时。即此方法是“不可重用”的。
 */
bool Tiny_Delay(uint16_t Time)
{
    //此延时不可重用
    if(HAL_TIM_STATE_READY != TinyDelay.htim->State || 
       DELAY_STATE_READY != TinyDelay.State) { return false; }
    
    //记录延时时长
    TinyDelay.DelayTime = Time / 2.0;
       
    //设置定时器计数值
    __HAL_TIM_SET_AUTORELOAD(TinyDelay.htim, Time);
    
    //标记状态
    TinyDelay.State = DELAY_STATE_BUSY;
       
    //开启定时
    HAL_TIM_Base_Start_IT(TinyDelay.htim);
       
    //等待延时结束
    while(DELAY_STATE_BUSY == TinyDelay.State);
    
    return true;
}

/**
 * @brief	终止延时
 * @param	None.
 * @retval	None.
 * @note    1. 用于提前结束延时。
 *          2. 可使用优先级更高的中断实现此功能。
 */
void Tiny_Delay_Abort(void)
{
    HAL_TIM_Base_Stop_IT(TinyDelay.htim);
}

/**
 * @brief	延时回调函数，更改延时阻塞标志
 * @param	None.
 * @retval	None.
 * @note    1. 此函数期望在目标定时器定时回调函数中被调用。
 */
void Tiny_DelayCallback(void)
{
    TinyDelay.State = DELAY_STATE_READY;
}

/************************ (C) COPYRIGHT HITwh Excellent Robot Organization(HERO). *****END OF FILE****/
