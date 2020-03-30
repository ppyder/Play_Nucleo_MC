
#ifndef AS5048A_H
#define AS5048A_H

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
  * @brief AS5048数据结构定义
  */
typedef struct
{
    SPI_HandleTypeDef *hspi;
    
    GPIO_TypeDef*   Port;          // 引脚端口
    uint16_t        Pin;           // 引脚位置
    uint16_t        PinNum;        // 引脚号
#ifdef USING_BITBAND              
    BIT_BAND_t      NSS_Write;     // IO写位带
#endif   
    
}AS5048_t, *pAS5048_t;


//功能性指令，读写标志位为第14位
#define AS5048_WRITE     (0x00<<14)
#define AS5048_READ      (0x01<<14)

//校验位
#define AS5048_PAR_SET   (0x01<<15) // 偶校验位置一
#define AS5048_PAR_RESET (0x00<<15) // 偶校验位置零


//寄存器地址集合
#define AS5048_REG_NOP     0x0000   // 虚拟寄存器，用于读取NOP指令。只读。
#define AS5048_REG_EF      0x0001   // 错误标志寄存器，用于读取错误标志。只读，读取后清除。
#define AS5048_REG_PC      0x0003   // 编程控制寄存器，用于进行功能性控制操作。可读写。
#define AS5048_REG_ZPH     0x0016   // 零位高寄存器，用于存储角度零位高八位。可读写。
#define AS5048_REG_ZPL     0x0017   // 零位低寄存器，用于存储角度零位低八位。可读写。
#define AS5048_REG_AGC     0x3FFD   // 诊断和自动增益寄存器，用于诊断工作环境和提供增益。只读。
#define AS5048_REG_MAG     0x3FFE   // 磁场信息寄存器，用于存储磁场强度信息。只读。
#define AS5048_REG_ANGLE   0x3FFF   // 角度信息寄存器，用于存储角度信息。只读。

/**
  * @brief REG - Error Flag 错误标志寄存器 -- 0x0001
  */
#define AS5048_CMD_READEF 	      (AS5048_PAR_RESET | AS5048_READ | AS5048_REG_EF)
#define AS5048_EF_PARITY_ERROR 	  (0X01<<2) // 奇偶校验错误
#define AS5048_EF_COMMAND_INVALID (0X01<<1) // 指令非法
#define AS5048_EF_FRAMING_ERROR   (0X01<<0) // 帧错误

/**
  * @brief REG - Programming Control 编程控制寄存器 -- 0x0003
  */
#define AS5048_CMD_READPC 	  (AS5048_PAR_SET | AS5048_READ | AS5048_REG_PC)
#define AS5048_PC_VERIFY 	  (0X01<<6)  // 校验（将OTP数据加载到具有修改的阈值比较器级别的内部寄存器中）
#define AS5048_PC_BURN 	      (0X01<<3)  // 烧录（启动内部的自动编程程序）
#define AS5048_PC_PRO_EN 	  (0X01<<0)  // 编程使能位，只有设置它后，OTP寄存器才可被写入

/**
  * @brief REG - OTP: Zero Position High 零位数据高寄存器 -- 0x0016
  * @note  OTP(One Time Programmable) 即一次编程寄存器。
  *        低8位有效。
  */
#define AS5048_CMD_READZPH 	  (AS5048_PAR_RESET | AS5048_READ | AS5048_REG_ZPH)
#define AS5048_ZPH_MASK 	  (0xFF)     // 数据域掩码

/**
  * @brief REG - OTP: Zero Position Low 零位数据低寄存器 -- 0x0017
  * @note  OTP(One Time Programmable) 即一次编程寄存器。
  *        低6位有效。
  */
#define AS5048_CMD_READZPL 	  (AS5048_PAR_SET | AS5048_READ | AS5048_REG_ZPL)
#define AS5048_ZERO_L_MASK 	  (0x3F)     // 数据域掩码

/**
  * @brief REG - Diagnostics & Automatic Gain Control 诊断和自动增益控制寄存器 -- 0x3FFD
  * @note  当COMP_H/COMP_L位置一时，建议去查看Magnitude寄存器以查明情况。
  */
#define AS5048_CMD_READAGC 	  (AS5048_PAR_RESET | AS5048_READ | AS5048_REG_AGC)
#define AS5048_AGC_COMP_H 	  (0x01<<11) // 比较：强。若磁场过强时此位置一。
#define AS5048_AGC_COMP_L 	  (0x01<<10) // 比较：弱。若磁场过弱时此位置一。
#define AS5048_AGC_COF   	  (0x01<<9)  /* (Cordic Overflow)，此位置一表示内部的CORDIC算法有超出范围的错误。
                                            此位置一时，角度和磁场测量数据不可用，PWM输出保持在上次的有效输出状态。*/
#define AS5048_AGC_OCF   	  (0x01<<8)  /* (Offset Compensation Finished)，此位置一表示偏移补偿算法已经完成。
                                            此位一般上电后就会置一。*/
#define AS5048_AGC_DATAMASK   (0x7F)     // 增益数据域掩码(此数据：0表示强磁场，255表示弱磁场)

/**
  * @brief REG - Magnitude 磁场数据寄存器 -- 0x3FFE
  * @brief REG - Angle     角度数据寄存器 -- 0x3FFF
  */ 
#define AS5048_CMD_READMAG 	  (AS5048_PAR_RESET | AS5048_READ | AS5048_REG_MAG)
#define AS5048_CMD_READANGLE  (AS5048_PAR_SET   | AS5048_READ | AS5048_REG_ANGLE)
#define AS5048_DATAMASK       (0x3FFF)   // 数据帧中后14位才是数据，前两位是功能信息

void AS5048a_Test(void);

#endif
