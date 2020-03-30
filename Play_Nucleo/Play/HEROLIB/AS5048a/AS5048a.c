#include "AS5048a.h"

#include "spi.h"

uint16_t TestRxBuffer = 0;
uint16_t TestTxBuffer = 0;//(uint16_t)(11 << 14); // NOP指令

static inline void AS5048_NSS_SET(pAS5048_t pDevice);
static inline void AS5048_NSS_RESET(pAS5048_t pDevice);
static uint32_t PinBit2PinNum(uint32_t PinBit);

//初始化
void AS5048_Init(void)
{
    return;
}


uint16_t TempValue[2];

//Test发送接收
void AS5048a_Test(void)
{
    TestTxBuffer = AS5048_CMD_READEF;//AS5048_CMD_READAGC;
    
    // 发送度读寄存器指令
    HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&TestTxBuffer, (uint8_t*)&TestRxBuffer, 1, 100);
    
    HAL_Delay(50);
    
    // 再次发送，此时返回含错误标志的寄存器值，同时寄存器值被清零
    HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&TestTxBuffer, (uint8_t*)TempValue, 1, 100);
    
    HAL_Delay(50);
    
    //TestTxBuffer = 0;
    
    // 发送NOP指令
    HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&TestTxBuffer, (uint8_t*)(TempValue + 1), 1, 100);
    
    return;
}

//置高NSS电平
static inline void AS5048_NSS_SET(pAS5048_t pDevice) 
{
#if defined(USING_BITBAND)  
    pDevice->NSS_Write = GPIO_PIN_SET; 
#else 
    HAL_GPIO_WritePin(pDevice->Port, pDevice->Pin, GPIO_PIN_SET); 
#endif  
}

//置低NSS电平
static inline void AS5048_NSS_RESET(pAS5048_t pDevice) 
{
#if defined(USING_BITBAND)  
    pDevice->NSS_Write = GPIO_PIN_RESET; 
#else 
    HAL_GPIO_WritePin(pDevice->Port, pDevice->Pin, GPIO_PIN_RESET); 
#endif  
}

//获得偶校验位，该位在第15位。
//偶校验位的目的在于使得整个位序列中“1”的个数为偶数。
uint16_t inline AS5048a_Parity_EVEN(uint16_t Num)
{
    //取低15位
    uint16_t Temp = Num & (~(1 << 15));
    
    /* 算法的核心思想是将“1”集中和对折（折叠）到最低位。 以异或抵消都是1的位（偶数个1等于没有1），
       最终剩下的最后一位若是1，则代表有奇数个“1”。 */
    
    /* 示例：(为简化表示，高位不写了。我们仅关心折叠对消后的低位结果)
              原数：0b 0101 1110 1110 1110 
        第一次异或：0b           0101 1110  => 0b 1011 0000
        第二次异或：(Temp >> 4) =              0b      1011  => 0b 1011
        第三次异或：(Temp >> 2) =                               0b   10  => 0b 01
        第四次异或：(Temp >> 1) =                                           0b  0  => 0b 1
        得到结果：  末位为1，表明原数中有奇数个“1”，为将“1”个数补为偶数个，偶校验位应为“1”。
    */
    
    Temp ^= Temp >> 8;  
    Temp ^= Temp >> 4;
    Temp ^= Temp >> 2;
    Temp ^= Temp >> 1;
    
    //奇偶校验位在第15位
    //“&1”的目的是取最低位结果，将其作为偶校验位数值
    return (Temp & 1) << 15;
}

/**
 * @brief	    确定一个八位二进制数中最高非零位的位置（最低位标号为0，从右到左依次递增，最高位为7）
 * @param[in]	Number - 待检测的四位二进制数
 * @retval	    位置标号(uint8_t)
 */
static uint8_t CheckHighest1In8Bits(uint8_t Number)
{
	int8_t cnt = 0;
	uint8_t result = 0;
	
	for(cnt = (8 - 1); cnt >= 0; cnt--)
	{
		if((Number >> cnt) == 1)
		{
			result = cnt;			//从高位往低位，捕获第一个不是零的位
			break;
		}
	}
	
	return result;
}

/**
 * @brief	    将引脚位表达转换为引脚数字编号
 * @param[in]	PinBit	待转换的参量
 * @retval	    位置标号(uint8_t)
 * @note	    ①内部调用
 *			    ②官方为位表达, 非数字编号（数字编号方便阅读） @ref GPIO_pins_define
 */
static uint32_t PinBit2PinNum(uint32_t PinBit)
{
	uint32_t result = 0;
	
	PinBit &= 0xFFFF;	//取后十六位
	
	if((PinBit >> 8) != 0)
	{
		result = CheckHighest1In8Bits((uint8_t)(PinBit >> 8)) + 8;
	}
	else
	{
		result = CheckHighest1In8Bits((uint8_t)(PinBit & 0xFF));
	}
	
	return result;
}
