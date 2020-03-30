#include "AS5048a.h"

#include "spi.h"

uint16_t TestRxBuffer = 0;
uint16_t TestTxBuffer = 0;//(uint16_t)(11 << 14); // NOPָ��

static inline void AS5048_NSS_SET(pAS5048_t pDevice);
static inline void AS5048_NSS_RESET(pAS5048_t pDevice);
static uint32_t PinBit2PinNum(uint32_t PinBit);

//��ʼ��
void AS5048_Init(void)
{
    return;
}


uint16_t TempValue[2];

//Test���ͽ���
void AS5048a_Test(void)
{
    TestTxBuffer = AS5048_CMD_READEF;//AS5048_CMD_READAGC;
    
    // ���Ͷȶ��Ĵ���ָ��
    HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&TestTxBuffer, (uint8_t*)&TestRxBuffer, 1, 100);
    
    HAL_Delay(50);
    
    // �ٴη��ͣ���ʱ���غ������־�ļĴ���ֵ��ͬʱ�Ĵ���ֵ������
    HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&TestTxBuffer, (uint8_t*)TempValue, 1, 100);
    
    HAL_Delay(50);
    
    //TestTxBuffer = 0;
    
    // ����NOPָ��
    HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&TestTxBuffer, (uint8_t*)(TempValue + 1), 1, 100);
    
    return;
}

//�ø�NSS��ƽ
static inline void AS5048_NSS_SET(pAS5048_t pDevice) 
{
#if defined(USING_BITBAND)  
    pDevice->NSS_Write = GPIO_PIN_SET; 
#else 
    HAL_GPIO_WritePin(pDevice->Port, pDevice->Pin, GPIO_PIN_SET); 
#endif  
}

//�õ�NSS��ƽ
static inline void AS5048_NSS_RESET(pAS5048_t pDevice) 
{
#if defined(USING_BITBAND)  
    pDevice->NSS_Write = GPIO_PIN_RESET; 
#else 
    HAL_GPIO_WritePin(pDevice->Port, pDevice->Pin, GPIO_PIN_RESET); 
#endif  
}

//���żУ��λ����λ�ڵ�15λ��
//żУ��λ��Ŀ������ʹ������λ�����С�1���ĸ���Ϊż����
uint16_t inline AS5048a_Parity_EVEN(uint16_t Num)
{
    //ȡ��15λ
    uint16_t Temp = Num & (~(1 << 15));
    
    /* �㷨�ĺ���˼���ǽ���1�����кͶ��ۣ��۵��������λ�� ������������1��λ��ż����1����û��1����
       ����ʣ�µ����һλ����1�����������������1���� */
    
    /* ʾ����(Ϊ�򻯱�ʾ����λ��д�ˡ����ǽ������۵�������ĵ�λ���)
              ԭ����0b 0101 1110 1110 1110 
        ��һ�����0b           0101 1110  => 0b 1011 0000
        �ڶ������(Temp >> 4) =              0b      1011  => 0b 1011
        ���������(Temp >> 2) =                               0b   10  => 0b 01
        ���Ĵ����(Temp >> 1) =                                           0b  0  => 0b 1
        �õ������  ĩλΪ1������ԭ��������������1����Ϊ����1��������Ϊż������żУ��λӦΪ��1����
    */
    
    Temp ^= Temp >> 8;  
    Temp ^= Temp >> 4;
    Temp ^= Temp >> 2;
    Temp ^= Temp >> 1;
    
    //��żУ��λ�ڵ�15λ
    //��&1����Ŀ����ȡ���λ�����������ΪżУ��λ��ֵ
    return (Temp & 1) << 15;
}

/**
 * @brief	    ȷ��һ����λ������������߷���λ��λ�ã����λ���Ϊ0�����ҵ������ε��������λΪ7��
 * @param[in]	Number - ��������λ��������
 * @retval	    λ�ñ��(uint8_t)
 */
static uint8_t CheckHighest1In8Bits(uint8_t Number)
{
	int8_t cnt = 0;
	uint8_t result = 0;
	
	for(cnt = (8 - 1); cnt >= 0; cnt--)
	{
		if((Number >> cnt) == 1)
		{
			result = cnt;			//�Ӹ�λ����λ�������һ���������λ
			break;
		}
	}
	
	return result;
}

/**
 * @brief	    ������λ���ת��Ϊ�������ֱ��
 * @param[in]	PinBit	��ת���Ĳ���
 * @retval	    λ�ñ��(uint8_t)
 * @note	    ���ڲ�����
 *			    �ڹٷ�Ϊλ���, �����ֱ�ţ����ֱ�ŷ����Ķ��� @ref GPIO_pins_define
 */
static uint32_t PinBit2PinNum(uint32_t PinBit)
{
	uint32_t result = 0;
	
	PinBit &= 0xFFFF;	//ȡ��ʮ��λ
	
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
