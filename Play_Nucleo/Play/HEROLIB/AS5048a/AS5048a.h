/**
 ******************************************************************************
 * @file    AS5048a.h
 * @brief   包含了AS5048a驱动库的操作和类型声明，本文件不建议用户修改.
 ******************************************************************************
 *
 * Copyright (C) HITwh Excellent Robot Organization(HERO). 2015-2020. All rights reserved.
 *
 * @version 1.0 示例版本
 * @author  杨亦凡
 * @contact 17863107058(手机)   942041771(qq)
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

/* 在此包含所用芯片的头文件 */
#include "stm32f3xx.h"
/* USER CODE END Includes */

/**
  * @brief 根据芯片型号选择处理方法
  * @note 除不能使用位带操作IO的芯片外，尽量使用位带方式对普通IO进行读写。
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
  * @brief 调试开关
  * @note 启用此宏可以启用测试函数。
  */
#define AS5048A_DEBUG

/* Exported macro ------------------------------------------------------------*/
/**
  * @brief AS5048a允许速度计算不可靠的最大次数
  * @note  当不可靠计数器 @InvalidCnt 值大于此值时，
  *       认为上个有效测算所得速度已不可信，将置零 @isReliable标志。
  */
#define AS5048A_MAX_SPEED_INVALID_CNT (5)

/**
  * @brief AS5048a发送帧 数据掩码
  */
#define AS5048A_PAR_SET    (0x01<<15) // 偶校验位置一
#define AS5048A_PAR_RESET  (0x00<<15) // 偶校验位置零
#define AS5048A_WRITE      (0x00<<14) // 指令位：写指令
#define AS5048A_READ       (0x01<<14) // 指令位：读指令

/**
  * @brief AS5048a返回帧 数据掩码
  */
#define AS5048A_HOST_ERROR (0x01<<14) // 错误标志置一表明，上一次从主机传输来的数据存在错误。
#define AS5048A_DATAMASK   (0x3FFF)   // 数据帧中后14位才是数据，前两位是功能信息

/**
  * @brief AS5048a寄存器地址
  */
#define AS5048A_REG_NOP     0x0000   // 虚拟寄存器，用于读取NOP指令。只读。
#define AS5048A_REG_EF      0x0001   // 错误标志寄存器，用于读取错误标志。只读，读取后清除。
#define AS5048A_REG_PC      0x0003   // 编程控制寄存器，用于进行功能性控制操作。可读写。
#define AS5048A_REG_ZPH     0x0016   // 零位高寄存器，用于存储角度零位高八位。可读写。
#define AS5048A_REG_ZPL     0x0017   // 零位低寄存器，用于存储角度零位低八位。可读写。
#define AS5048A_REG_AGC     0x3FFD   // 诊断和自动增益寄存器，用于诊断工作环境和提供增益。只读。
#define AS5048A_REG_MAG     0x3FFE   // 磁场信息寄存器，用于存储磁场强度信息。只读。
#define AS5048A_REG_ANGLE   0x3FFF   // 角度信息寄存器，用于存储角度信息。只读。

/**
  * @brief AS5048a读寄存器的指令集 AS5048A_CMD_READ
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
  * @brief AS5048a操作宏：判断是否处于错误状态
  */
#define IS_AS5048A_ERROR(__STATE__) ((__STATE__) == AS5048A_STATE_SPI_ERROR || \
                                     (__STATE__) == AS5048A_STATE_CMD_ERROR || \
                                     (__STATE__) == AS5048A_STATE_ZP_ERROR  || \
                                     (__STATE__) == AS5048A_STATE_DIG_ERROR)
                                     
/* Exported types ------------------------------------------------------------*/
/**
  * @brief AS5048a工作状态枚举类型定义
  */
typedef enum
{
    /* 基本状态 */
    AS5048A_STATE_RESET,        // 未初始化
    AS5048A_STATE_READY,        // 准备工作已完成，待命
    AS5048A_STATE_BUSY,         // 正在通信过程中
    
    /* 错误状态：处于以下状态将导致组件停止工作，数值更小的错误码将覆盖数值更大的错误码 */
    AS5048A_STATE_SPI_ERROR,    /* SPI底层发生错误，一般是芯片外设（如DMA、SPI等）出现了问题。
                                   发生该错误时，可在公共变量 @AS5048A_HAL_ErrorCode中查看底层操作的错误码。*/
    AS5048A_STATE_CMD_ERROR,    /* 读寄存器时发生错误(Command Error)。
                                   例如，返回帧中错误标志连续出现的次数过多，这表明发送到从机的帧有问题。   */
    AS5048A_STATE_DIG_ERROR,    /* 工作条件不合要求(Diagnose Error)。
                                   可能的情况有：磁场不合格；芯片未上电(偏置补偿失败)；CORDIC算法溢出       */
    AS5048A_STATE_ZP_ERROR,     /* 零位初始化失败(Zero Position Error)。
                                   即，没有成功地把上电时的转子位置作为零位。*/
}AS5048a_State_t;


/**
  * @brief REG - Error Flag 错误标志寄存器 -- 0x0001
  */
typedef union
{
    uint16_t Value;                // 返回帧写入值
    struct
    {
        unsigned Framing   : 1;    // 帧错误
        unsigned CMD       : 1;    // 无效指令
        unsigned Parity    : 1;    // 奇偶校验错误
        unsigned Unused    : 13;   // 未使用区域
    }bit_area;
    
}AS5048a_EFR_t;

/**
  * @brief REG - Programming Control 编程控制寄存器 -- 0x0003
  */
typedef union
{
    uint16_t Value;                // 返回帧写入值
    struct
    {
        unsigned PE        : 1;    // 编程使能位，只有设置它后，OTP寄存器才可被写入
        unsigned Unused1   : 2;    // 未使用区域
        unsigned Burn      : 1;    // 烧录(启动内部的自动编程程序)
        unsigned Unused2   : 2;    // 未使用区域
        unsigned Verify    : 1;    // 校验(将OTP数据加载到具有修改的阈值比较器级别的内部寄存器中)
        unsigned Unused3   : 7;    // 未使用区域
    }bit_area;
    
}AS5048a_PCR_t;

/**
  * @brief REG - OTP: Zero Position High 零位数据高寄存器 -- 0x0016
  * @note  OTP(One Time Programmable) 即一次编程寄存器。
  *        低8位有效。
  */
typedef union
{
    uint16_t Value;                // 返回帧写入值
    struct
    {
        unsigned Data     : 8;     // 零位置的高八位数据
        unsigned Unused   : 8;     // 未使用区域
    }bit_area;
    
}AS5048a_ZPHR_t;

/**
  * @brief REG - OTP: Zero Position Low 零位数据低寄存器 -- 0x0017
  * @note  OTP(One Time Programmable) 即一次编程寄存器。
  *        低6位有效。
  */
typedef union
{
    uint16_t Value;                // 返回帧写入值
    struct
    {
        unsigned Data     : 6;     // 零位置的低六位数据
        unsigned Unused   : 10;    // 未使用区域
    }bit_area;
    
}AS5048a_ZPLR_t;

/**
  * @brief REG - Diagnostics & Automatic Gain Control 诊断和自动增益控制寄存器 -- 0x3FFD
  * @note  当COMP_H/COMP_L位置一时，建议去查看Magnitude寄存器以查明情况。
  */
typedef union
{
    uint16_t Value;                // 返回帧写入值
    struct
    {
        unsigned AutoGrain : 8;    // 自动补偿增益值(0表示强磁场，255表示弱磁场)
        unsigned OCF       : 1;    /* (Offset Compensation Finished)，此位置一表示偏移补偿算法已经完成。
                                      此位一般上电后就会置一。*/
        unsigned COF       : 1;    /* (Cordic Overflow)，此位置一表示内部的CORDIC算法产生了溢出错误。
                                      此位置一时，角度和磁场测量数据不可用，PWM输出保持在上次的有效输出状态。*/
        unsigned Comp_L    : 1;    // 磁场过强标志位（AutoGrain为0时此位置一）
        unsigned Comp_H    : 1;    // 磁场过弱标志位（AutoGrain为255时此位置一）
        unsigned Unused    : 4;    // 未使用区域
    }bit_area;
    
}AS5048a_AGCR_t;

/**
  * @brief REG - Magnitude 磁场数据寄存器 -- 0x3FFE
  */ 
typedef union
{
    uint16_t Value;                // 返回帧写入值
    struct
    {
        unsigned Data     : 14;    // 磁场数据
        unsigned Unused   : 2;     // 未使用区域
    }bit_area;
    
}AS5048a_MAGR_t;

/**
  * @brief REG - Angle     角度数据寄存器 -- 0x3FFF
  */ 
typedef union
{
    uint16_t Value;                // 返回帧写入值
    struct
    {
        unsigned Data     : 14;    // 角度数据
        unsigned Unused   : 2;     // 未使用区域
    }bit_area;
    
}AS5048a_ANGR_t;

/**
  * @brief AS5048a数据结构定义
  */
typedef struct
{
    /* 硬件底层参数 */
    SPI_HandleTypeDef *hspi;
    
    GPIO_TypeDef*   NSSPort;        // 引脚端口
    uint16_t        NSSPin;         // 引脚位置
    uint16_t        NSSPinNum;      // 引脚号
#ifdef USING_BITBAND              
    BIT_BAND_t      NSS_Write;      // IO写位带
#endif
    /* 组件接口参数 */
    bool            isDebuging;     // 标记是否处于诊断状态，诊断状态下不允许调用非阻塞模式下的任何函数。
    bool            isSpeedReliable;/* 标记速度数据是否可靠。连续两次速度数据不可靠，即将此位置零。
                                       当此位置一时，Speed中将保存最新的可靠数据。
                                       当速度数据不可靠时，“获取实时位置”将返回最新的基准角度。
                                       可通过检查此标志对组件工作状态进行监控。                     */
    float           BaseAngle;      // 基准角度，是从传感器读来的角度值，角度制。
    float           RealAngle;      // 实时角度，由速度和基准角度估计出来的调用接口计算时的瞬时角位置，角度制。
    float           RawSpeed;       // 原始转速
    float           Speed;          // 转速，由两次读取位置以及其时间间隔计算得来，单位RPM，应为正值。
    float           MaxRPM;         // 预期最大转速，用于估计速度数据可靠性和角位置整圈跳变。
    AS5048a_EFR_t   ErrorFlags;     // Error Flag寄存器值。
    AS5048a_AGCR_t  AGCR;           // AGC寄存器值
    AS5048a_MAGR_t  MAGR;           // Magnet寄存器值
    
    /* 通信参数 */
    uint32_t        FrameTime;      // 一个通信周期耗费的时间，单位ns (须经实测获得，不同系统主频下时间不同)。
    AS5048a_State_t State;          // 标记AS508工作状态，被用作保护和互斥标志。
    
    /* 周期读取相关参数 */
    uint16_t        LastCMD;        // 上次执行的读取指令（区分是读寄存器还是清除错误标记）。
    uint16_t        PeriodErrorCnt; // 周期读取中连续遇到的通信故障计数器。
    uint16_t        ErrorMax;       // 允许 连续出现通信错误 的最大次数。
    uint16_t        TempRxBuffer;   // 用于DMA传输的临时数据缓冲区。
    bool            isDataInvalid;  /* 标记上次通信中的数据是否可信，当回复帧中的错误标志置一时，此位置一。
                                       在下次通信中，将发送清除错误标志以使得通信恢复正常。 */
    GlobalTime_t    ReadMoment;     // 记录上次读取位置的时刻。
    
    SlidAveFilter_t SpeedFilter;    // 滑动平均值滤波，针对速度数据跳动的情况。
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
