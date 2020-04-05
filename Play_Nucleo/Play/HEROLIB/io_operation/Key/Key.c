/**
 ******************************************************************************
 * @file    Key.c
 * @brief   中间层：按键相关的基本操作和方法.
 * @version 1.0 示例版本
 * @author  杨亦凡
 * @contact 17863107058(手机)   942041771(qq)
 * @date    2020/03/20
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
#include "Key.h"
#include "Key_User.h"

/* Public variables --------------------------------------------------------*/
//用户按键序列
Key_t UserKeys[KeyNum];

/* Private macros -----------------------------------------------------------*/
/**
  * @brief Key扫描周期，单位ms
  */
#define KEY_LOOP_PERIOD 1
/**
  * @brief 按键内部计时器清零周期(单位ms, 最大值为UINT32_MAX)
  */
#define KEY_SCAN_PERIOD (1000 * KEY_LOOP_PERIOD)

/* Private functions --------------------------------------------------------*/
static int32_t Is_Key_INT(uint16_t GPIO_Pin);
static uint32_t PinBit2PinNum(uint32_t PinBit);
static void KeyErrorHandler(enum KeyErrorCode Code);
static uint32_t GPIOPort2EXTIPort(GPIO_TypeDef* GPIOPort);

/* Private variables --------------------------------------------------------*/
bool isEventOccured = false;    // 事件发生标志，用于减少无谓的遍历操作

/**
 * @brief	    按扫描方式初始化按键
 * @param[out]	pKey          - 指向被初始化结构体的指针
 * @param[in]	pDealFunc     - 指向函数处理方法的函数指针
 * @param[in]	Port          - 按键GPIO
 * @param[in]	Pin           - 按键引脚，取值 GPIO_PIN_x (where x can be (0..15))
 * @param[in]	DownPolarity  - 按键被按下时的电平状态
 * @param[in]	Type          - 按键类型，取值 @KeyType_t
 * @param[in]	CheckPeriod   - 扫描周期，单位为 KEY_LOOP_PERIOD ms，取值范围:[1, KEY_SCAN_PERIOD)。
 * @param[in]	MaskBits      - 滤波位数，指示连续检测到多少次同一状态后确认状态，取值范围:[1, 32]。
 * @retval	    None.
 * @note        参数CheckPeriod和MaskBits仅在检测模式为“轮询扫描模式”下有效。
 *              此两者共同确定了滤波窗口，即“某电平维持多久后确认状态”。
 */
void Key_Scan_Init( pKey_t pKey, void (*pDealFunc)(KeyEvents_t Events),
                    KeyType_t Type, uint32_t CheckPeriod, uint32_t MaskBits,
                    GPIO_TypeDef* Port, uint16_t Pin, GPIO_PinState DownPolarity )
{
    //安全检测
    if( NULL == pKey      || !IS_GPIO_ALL_INSTANCE(Port) ||  
        !IS_GPIO_PIN(Pin) || !IS_GPIO_PIN_ACTION(DownPolarity) )
    { 
        KeyErrorHandler(KEY_ERROR_ParamInvalid);
        return; 
    }
    
    //记录按键硬件信息
    pKey->Port    = Port;
    pKey->Pin     = Pin;
    pKey->PinNum  = PinBit2PinNum(Pin);
    pKey->DownPolarity = DownPolarity;
    
#ifdef USING_BITBAND    
    //初始化位带
    pKey->Key_Pin_Read = &BIT_ADDR((uint32_t)Port + IDR_Addr_Offset, pKey->PinNum);
#endif
    
    //记录按键功能信息
    pKey->Type = Type;
    pKey->Mode = KEY_MODE_SCAN;
    
    //根据类型进行事件选择
    switch(pKey->Type)
    {
        case KEY_TYPE_INCHING:   //点动控制，关注事件KEY_EVENT_DOWN和KEY_EVENT_UP
            
            pKey->Target_Events = (KeyEvents_t)(KEY_EVENT_DOWN | KEY_EVENT_UP);
            break;
        
        case KEY_TYPE_SWITCH:    //长动控制，关注事件KEY_EVENT_PRESS
            
            pKey->Target_Events = (KeyEvents_t)KEY_EVENT_PRESS;
            break;
        
        default:break;
    }
    
    //处理函数初始化
    pKey->pDealFunc = pDealFunc;
    
    //初始化状态滤波器
    StateFilterInit(&pKey->KeyFillter, CheckPeriod * KEY_LOOP_PERIOD, MaskBits);
}

/**
 * @brief	    按外部中断方式初始化按键
 * @param[out]	pKey          - 指向被初始化结构体的指针
 * @param[in]	pDealFunc     - 指向函数处理方法的函数指针
 * @param[in]	Port          - 按键GPIO
 * @param[in]	Pin           - 按键引脚，取值 GPIO_PIN_x (where x can be (0..15))
 * @param[in]	DownPolarity  - 按键被按下时的电平状态
 * @param[in]	Type          - 按键类型，取值 @KeyType_t
 * @param[in]	Line          - 中断线，取值 EXTI_LINE_x (where x can be (0..15))
 * @retval	    None.
 */
void Key_INT_Init( pKey_t pKey, void (*pDealFunc)(KeyEvents_t Events),
                   KeyType_t Type, uint32_t Line,
                   GPIO_TypeDef* Port, uint16_t Pin, GPIO_PinState DownPolarity)
{
    EXTI_ConfigTypeDef Config;
    
    //安全检测
    if( NULL == pKey      || !IS_GPIO_ALL_INSTANCE(Port) ||  
        !IS_GPIO_PIN(Pin) || !IS_GPIO_PIN_ACTION(DownPolarity) )
    { 
        KeyErrorHandler(KEY_ERROR_ParamInvalid);
        return; 
    }
    
    //记录按键硬件信息
    pKey->Port    = Port;
    pKey->Pin     = Pin;
    pKey->PinNum  = PinBit2PinNum(Pin);
    pKey->DownPolarity = DownPolarity;
    
    Config.Line = Line;
    Config.Mode = EXTI_MODE_INTERRUPT;
    Config.GPIOSel = GPIOPort2EXTIPort(Port);
    
#ifdef USING_BITBAND    
    //初始化位带
    Key_Pin_Read = &BIT_ADDR((uint32_t)Key_GPIO_Port + IDR_Addr_Offset, GPIO_Pin_Num);
#endif
    
    //记录按键功能信息
    pKey->Type = Type;
    pKey->Mode = KEY_MODE_INT;
    
    //根据类型进行事件选择
    switch(pKey->Type)
    {
        case KEY_TYPE_INCHING:   //点动控制，关注事件KEY_EVENT_DOWN和KEY_EVENT_UP
            
            Config.Trigger = EXTI_TRIGGER_RISING_FALLING;
            pKey->Target_Events = (KeyEvents_t)(KEY_EVENT_DOWN | KEY_EVENT_UP);
            break;
        
        case KEY_TYPE_SWITCH:    //长动控制，关注事件KEY_EVENT_PRESS
            
            Config.Trigger = (GPIO_PIN_RESET == DownPolarity ? EXTI_TRIGGER_FALLING : EXTI_TRIGGER_RISING);
            pKey->Target_Events = (KeyEvents_t)KEY_EVENT_PRESS;
            break;
        
        default:break;
    }
    
    //初始化中断线
    HAL_EXTI_SetConfigLine(&pKey->hexti, &Config);
    
    //处理函数初始化
    pKey->pDealFunc = pDealFunc;
}

/**
 * @brief	扫描按键状态
 * @param	None.
 * @retval	None
 * @note    此函数期望被周期性地调用，用来实现Key的动态检测和滤波。
 *          调用周期为 KEY_LOOP_PERIOD ，单位为ms。
 */
void Key_Scan(void)
{
    static uint32_t TimeCnt = 0;    //时间计数器，定时清零
    GPIO_PinState State;
    
    //累加计时器值
    TimeCnt += KEY_LOOP_PERIOD;
    
    //遍历所有按键
    for(uint32_t i = 0; i < KeyNum; i++)
    {
        //被选中且到了所指定的扫描时间
        if(KEY_MODE_SCAN == UserKeys[i].Mode && (0 == TimeCnt % UserKeys[i].KeyFillter.CheckPeriod))
        {
#ifdef USING_BITBAND
            TempState = StateFilter(&UserKeys[i].KeyFillter, (*Key_Pin_Read));
#else
            State = HAL_GPIO_ReadPin(UserKeys[i].Port, UserKeys[i].Pin);
            StateFilter(&UserKeys[i].KeyFillter, (State == UserKeys[i].DownPolarity));
#endif
        }
        else { continue; }
        
        //事件检测
        switch(GetJumpingState(&UserKeys[i].KeyFillter))
        {
            case RISING_EDGE:   //上升沿，标志用户按键按下
                UserKeys[i].Events = (KeyEvents_t)(UserKeys[i].Target_Events & (KEY_EVENT_DOWN | KEY_EVENT_PRESS));
                break;
                
            case FALLING_EDGE:  //下降沿，标志用户按键弹起
                UserKeys[i].Events = (KeyEvents_t)(UserKeys[i].Target_Events & KEY_EVENT_UP);
                break;
            
            case NO_JUMPING:break;
            default:break;
        }
        
        //标记事件是否发生了(或逻辑)
        isEventOccured = isEventOccured || (KEY_NOEVENT != UserKeys[i].Events);
    }
    
    if(TimeCnt > (KEY_SCAN_PERIOD - 1))
    {
        TimeCnt = 0;
    }
    
    return;
}

/**
 * @brief	    扫描按键状态
 * @param[in]	GPIO_Pin - 引发中断的引脚号
 * @retval	    None
 * @note        1. 此函数期望被在外部中断的回调函数中调用。
 *              2. 进入这个函数说明一定产生了中断，
 *              但无法判断是上升还是下降沿，因此还需状态读取以判断。
 */
void Key_Int(uint16_t GPIO_Pin)
{
    int32_t KeyIndex = 0;
    GPIO_PinState State;
    
    //获取组件下标
    KeyIndex = Is_Key_INT(GPIO_Pin);
    
    //若为有效下标
    if(KeyIndex > -1)
    {
#ifdef USING_BITBAND
        State = (*Key_Pin_Read);
#else
        State = HAL_GPIO_ReadPin(UserKeys[KeyIndex].Port, UserKeys[KeyIndex].Pin);
#endif  
        //这里上升下降沿并非按实际电平定义，是与扫描模式统一，按照“按键有效”的逻辑状态定义的。
        if(State == UserKeys[KeyIndex].DownPolarity)
        {
            //按键被按下了，说明逻辑上升沿发生了
            UserKeys[KeyIndex].Events = (KeyEvents_t)(UserKeys[KeyIndex].Target_Events & (KEY_EVENT_DOWN | KEY_EVENT_PRESS));
        }
        else
        {
            //按键被释放了，说明逻辑下降沿发生了
            UserKeys[KeyIndex].Events = (KeyEvents_t)(UserKeys[KeyIndex].Target_Events & KEY_EVENT_UP);
        }
        
        //标记事件是否发生了(或逻辑)
        isEventOccured = isEventOccured || (KEY_NOEVENT != UserKeys[KeyIndex].Events);
    }
    else { return; }
    
    return;
}

/**
 * @brief	执行按键事件响应
 * @param	None.
 * @retval	None
 * @note    此函数期望被在主循环中调用，用来执行已被标记的Key事件响应。
 */
void Key_EventScan(void)
{
    //有按键事件发生才做处理
    if(isEventOccured)
    {
        //一次遍历后，所有的按键事件都将被执行。
        // 放在前面的目的是，若后续函数中有较多延时操作，
        // 那么新的按键事件请求将不会被掩盖。
        isEventOccured = false;
        
        //遍历所有按键
        for(uint32_t i = 0; i < KeyNum; i++)
        {       
            //按照事件调用处理函数
            if(KEY_NOEVENT != UserKeys[i].Events)
            {
                UserKeys[i].pDealFunc(UserKeys[i].Events);
                UserKeys[i].Events = KEY_NOEVENT;
            }
        }
    }
    else { return; }
    
    return;
}

/**
 * @brief	    本文件执行中的错误处理
 * @param[in]	ErrorCode - 错误码
 * @retval	    None.
 */
enum KeyErrorCode KeyErrorCode = KEY_NOERROR;
static void KeyErrorHandler(enum KeyErrorCode Code)
{
    KeyErrorCode = Code;
    
    //程序进入此处，请检查变量KeyErrorCode中的错误码以定位出错原因。
    while(1);
}

/**
 * @brief	    检测是否是本组件引脚产生的中断
 * @param[in]	GPIO_Pin - GPIO引脚号
 * @retval	    若是，则返回组件下标，若不是，则返回-1.
 */
static int32_t Is_Key_INT(uint16_t GPIO_Pin)
{
    int32_t result = -1;
    
    //遍历查询
    for(int i = 0; i < KeyNum; i++)
    {
        //找到后直接结束循环
        if(GPIO_Pin == UserKeys[i].Pin)
        {
            result = i;
            break;
        }
    }
    
    return result;
}

/**
 * @brief	    将GPIOPort转化为EXTIPort参数
 * @param[in]	GPIOPort - GPIO端口
 * @retval	    EXTIPort参数
 */
static uint32_t GPIOPort2EXTIPort(GPIO_TypeDef* GPIOPort)
{
    uint32_t result = 0;
    
    switch((uint32_t)GPIOPort)
    {
        case GPIOA_BASE:
            result = EXTI_GPIOA;
            break;
        case GPIOB_BASE:
            result = EXTI_GPIOB;
            break;
        case GPIOC_BASE:
            result = EXTI_GPIOC;
            break;
        case GPIOD_BASE:
            result = EXTI_GPIOD;
            break;
#if defined(GPIOE)
        case GPIOE_BASE:
            result = EXTI_GPIOE;
            break;
#endif /* GPIOE */
        case GPIOF_BASE:
            result = EXTI_GPIOF;
            break;
#if defined(GPIOG)
        case GPIOG_BASE:
            result = EXTI_GPIOG;
            break;
#endif /* GPIOG */
#if defined(GPIOH)
        case GPIOH_BASE:
            result = EXTI_GPIOH;
            break;
#endif /* GPIOH */ 
#if defined(GPIOI)
        case GPIOI_BASE:
            result = EXTI_GPIOI;
            break;
#endif /* GPIOI */
        
        default:  KeyErrorHandler(KEY_ERROR_PortInvalid);  break;
    }
    
    return result;
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

/************************ (C) COPYRIGHT HITwh Excellent Robot Organization(HERO). *****END OF FILE****/
