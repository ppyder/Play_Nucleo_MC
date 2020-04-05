/**
 ******************************************************************************
 * @file    TimeReference.c
 * @brief   中间层：基于一个专用的定时器时基提供精准的时间参考.
 *          用于支持计算速度、实时位置，提供ns级时间基准(芯片所能提供的最大精度)。
 * @version 1.0 示例版本
 * @author  杨亦凡
 * @contact 17863107058(手机)   942041771(qq)
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
  * @brief 时间基准组件结构体类型定义
  */
typedef struct
{
    TIM_HandleTypeDef* htim;        // 定时器句柄
    
    int64_t            CarryNum;    // 定时器的溢出进位(于定时中断中自增)
    volatile uint16_t* pCNT;        // 目标定时器的CNT寄存器地址 
    
    float              Tick_T;      // 输入时钟tick的周期(一个tick多长时间)
    float              Toatal_s;    // 调用“获取时间”函数对此变量进行更新，单位s(秒)
    
    
}TimeRef_t, *pTimeRef_t;

/* Public variables --------------------------------------------------------*/
TimeRef_t TimeRef = // 时基结构体
{
    .htim = NULL,
    .CarryNum = 0,
    .pCNT = NULL,
    .Tick_T = 1,
    .Toatal_s = 0,
};

/**
 * @brief	    初始化时间基准
 * @param[in]	htim     - 用于实现功能的定时器句柄
 * @param[in]	Freq_MHz - 输入定时器的、未经预分频的外设时钟频率
 * @retval	    初始化是否成功，成功为true.
 * @note        1. 初始化结果为：定时器以最大频率向上计数，并以时间满量程触发定时中断。
 *              2. 被使用的定时器在本组件调用期间不可作他用。
 */
bool TimeRef_Init(TIM_HandleTypeDef* htim, uint32_t Freq_MHz)
{
    //安全检查
    if( NULL == htim || HAL_TIM_STATE_READY != htim->State  || 
        !IS_TIM_REPETITION_COUNTER_INSTANCE(htim->Instance) || __HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) 
    { return false; }
    
    //保存外设句柄
    TimeRef.htim = htim;
    
    //记录所要用到的寄存器的地址
    TimeRef.pCNT = (volatile uint16_t*)&(htim->Instance->CNT);
    
    //计算一个时钟tick过去多长时间
    TimeRef.Tick_T = (1.0 / Freq_MHz) * 1e-6;
    
    //设置URS位，使得定时器更新中断仅由计数器溢出事件触发（从模式和软件更新时不触发中断）
    htim->Instance->CR1 |= TIM_CR1_URS;
    
    //更改其预分频因子，使得输入的时钟信号频率最大
    __HAL_TIM_SET_PRESCALER(htim, 0);
    
    //设置ARR值
    __HAL_TIM_SET_AUTORELOAD(htim, TIMEREF_CNT_MAX - 1);
        
    //设置CNT值
    __HAL_TIM_SET_COUNTER(htim, 0);
    
    return true;
}

/**
 * @brief	    开始计时
 * @param   	None.
 * @retval	    启动计时是否成功，成功为true.
 * @note        1. 启用计时前复位计数器
 */
bool TimeRef_Start(void)
{
    if(NULL == TimeRef.htim) { return false; }
    
    //复位计数
    TimeRef_Clear();
    
    //开启定时
    return (HAL_OK == HAL_TIM_Base_Start_IT(TimeRef.htim));
}

/**
 * @brief	    复位计数器(重新开始计数)
 * @param   	None.
 * @retval	    复位是否成功，成功为true.
 */
bool TimeRef_Clear(void)
{  
    if(NULL == TimeRef.htim) { return false; }
        
    //清空进位数据
    TimeRef.CarryNum = 0;
    
    //产生一个更新事件，清零寄存器
    HAL_TIM_GenerateEvent(TimeRef.htim, TIM_EVENTSOURCE_UPDATE);
    
    return true;
}

/**
 * @brief	    从某通用时间结构中获取时间
 * @param[out] 	pTime   - 读出的数据将被存放的结构体地址.
 * @retval	    读出是否成功，成功为true
 */
bool TimeRef_GetPreciseTime(pGlobalTime pTime)
{
    if(NULL == pTime) { return false; }

    //进位值
    pTime->CarryNum = TimeRef.CarryNum;
    
    //tick数
    pTime->Ticks = (*TimeRef.pCNT);                       
    
    return true;
}

/**
 * @brief	    更新实时时间
 * @param   	None.
 * @retval	    实时时间，单位为秒。
 * @note        1. 计算结果被更新在本组件的成员 @Total_s中，单位为秒。
 */
float TimeRef_GetTotal(void)
{
    if(NULL == TimeRef.htim) { return 0; }
    
    //已递减的寄存器值，表征流逝的时间
    uint32_t TempCNT = (*TimeRef.pCNT);
    
    //计算从计时开始流逝的总时间(注意：使用64位数据时，要注意表达式内的类型转换。因为默认是使用int32计算的)
    TimeRef.Toatal_s = TimeRef.Tick_T * 
                       (TimeRef.CarryNum * (int64_t)TIMEREF_TOTALCNT + (int64_t)TempCNT);
    
    return TimeRef.Toatal_s;
}

/**
 * @brief	    从某通用时间结构中获取时间
 * @param[in]  	pTime  - 待转换结构体地址.
 * @retval	    转换结果，单位为秒。
 */
float TimeRef_GetTotal_FromGlobalTime(pGlobalTime pTime)
{
    float result = 0;
    
    if(NULL == pTime) { return 0; }
    
    //换算时间
    result = TimeRef.Tick_T * (pTime->CarryNum * (int64_t)TIMEREF_TOTALCNT + (int64_t)pTime->Ticks);
    
    return result;
}

/**
 * @brief	    精确的时间求差
 * @param[in]  	pLeft  -  求差的被减数.
 * @param[in]  	pRight  - 求差的减数.
 * @retval	    转换结果，单位为秒。
 */
float TimeRef_TimeMinus(pGlobalTime pLeft, pGlobalTime pRight)
{
    int64_t Total_ticks = 0;
    
    //安全检查
    if(NULL == pLeft || NULL == pRight) { return 0; }
    
    Total_ticks = (pLeft->CarryNum - pRight->CarryNum) * (int64_t)TIMEREF_TOTALCNT + 
                  (int64_t)pLeft->Ticks - (int64_t)pRight->Ticks;
    
    
    return (Total_ticks * TimeRef.Tick_T );
}

/**
 * @brief	    回调函数，
 * @param   	None.
 * @retval	    运算是否成功，成功为true.
 * @note        1. 期望本函数在对应的定时器中断中被有针对地调用。
 *              2. “有针对”的含义是，应至少区分于其他定时器中断响应，因为定时回调函数往往是同一个。
 */
void TimeReference_Callback(void)
{
    //进位计数值自增
    TimeRef.CarryNum++;
    return;
}

/************************ (C) COPYRIGHT HITwh Excellent Robot Organization(HERO). *****END OF FILE****/
