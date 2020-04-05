/**
 ******************************************************************************
 * @file    filter.h
 * @brief   包含关于滤波器的类型定义和函数声明.
 *              + 包含状态滤波器的相关定义的声明
 *                  ++ 包含状态滤波器跳变沿的枚举类型定义
 *                  ++ 包含状态滤波器的数据结构定义
 *                  ++ 包含状态滤波器的操作函数声明
 *              + 包含数据滤波器的相关定义的声明
 *                  ++ 包含数据滤波器的数据结构定义
 *                  ++ 包含数据滤波器的操作函数声明
 ******************************************************************************
 *
 * Copyright (C) HITwh Excellent Robot Organization(HERO). 2015-2018. All rights reserved.
 *
 * @version 1.0 示例版本
 * @author  杨亦凡
 * @contact 17863107058(手机)   942041771(qq)
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

/** @addtogroup HeroLibrary HERO团队代码库
  * @{
  */

/** @addtogroup Low_Level_Library Low Level Library(底层库)
  * @{
  */

/** @addtogroup Universal_Software_Support Universal(软件通用支持)
  * @{
  */
  
/** @addtogroup Universal_filter fillter滤波器
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup Filter_Exported_Types Filter数据类型
  * @{
  */

/**
  * @brief 滤波器跳变状态 枚举定义
  */
typedef enum 
{
    NO_JUMPING,     //!< 无跳变
    RISING_EDGE,    //!< 上升沿跳变
    FALLING_EDGE,   //!< 下降沿跳变
    
}JumpingState;

/**
  * @brief 一阶低通数据滤波器可选参数 枚举定义
  */
enum FillterParams
{
    Fill_3Hz,     //3Hz截止
    Fill_Num
};   

/**
  * @brief 状态滤波器 结构定义
  */
typedef struct
{
    uint32_t        CheckPeriod;    //!< 检测周期（单位为us）
    
    uint32_t        History;	    //!< 所记录的历史值
    
    uint8_t         MaskBits;       //!< 滤波位数
    
	bool            State;		    //!< 滤波后确认的状态
    
    JumpingState    Jumping;        //!< 所监测出的跳变状态
		
}StateFilter_t, *pStateFilter;

/**
  * @brief 一阶低通数据滤波器 结构定义
  */
typedef struct 
{
    double num[2];         //!< Z变换后滤波器脉冲传函的分子向量
    double den[2];         //!< Z变换后滤波器脉冲传函的分母向量
    
    double LastInput;      //!< 记录上次滤波器的输入值
    double LastOutput;     //!< 记录上次滤波器的输出
    
}FillterParam_t;
/**
  * @}
  */

/**
  * @brief 滑动平均值滤波 结构定义
  */
typedef struct 
{
    uint32_t DataSize;     //!< 数据窗口大小
    float*   pDataBuffer;  //!< 循环队列数据缓冲区地址（其长度应与窗口大小相等）
    
    uint32_t DataIndex;    //!< 循环队列数组下标
    float    Sum;          //!< 求和器
    
}SlidAveFilter_t;
/**
  * @}
  */


/* Exported variable --------------------------------------------------------*/
/** @addtogroup Filter_Variable Filter公共变量
  * @{
  */

extern FillterParam_t Fill_Params[Fill_Num];
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup Filter_Functions Filter基本操作函数
  * @{
  */
bool StateFilterInit(pStateFilter pFilter, uint32_t CheckPeriod, uint32_t MaskBits);
bool StateFilter(StateFilter_t *pFilter, bool NowState);
JumpingState GetJumpingState(StateFilter_t *pFilter);

double DataFillter(double Value, FillterParam_t *Params);

void SlidingAveFilterInit(SlidAveFilter_t *pFilter, float *pBuffer, uint32_t BufferSize, float InitValue);
float SlidingAveFilter(SlidAveFilter_t *pFilter, float NewData);
void SetSlidingAveBuffer(SlidAveFilter_t *pFilter, float Value);
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
