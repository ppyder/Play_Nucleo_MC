/**
 ******************************************************************************
 * @file    Key.c
 * @brief   �м�㣺������صĻ��������ͷ���.
 * @version 1.0 ʾ���汾
 * @author  ���ෲ
 * @contact 17863107058(�ֻ�)   942041771(qq)
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
//�û���������
Key_t UserKeys[KeyNum];

/* Private macros -----------------------------------------------------------*/
/**
  * @brief Keyɨ�����ڣ���λms
  */
#define KEY_LOOP_PERIOD 1
/**
  * @brief �����ڲ���ʱ����������(��λms, ���ֵΪUINT32_MAX)
  */
#define KEY_SCAN_PERIOD (1000 * KEY_LOOP_PERIOD)

/* Private functions --------------------------------------------------------*/
static int32_t Is_Key_INT(uint16_t GPIO_Pin);
static uint32_t PinBit2PinNum(uint32_t PinBit);
static void KeyErrorHandler(enum KeyErrorCode Code);
static uint32_t GPIOPort2EXTIPort(GPIO_TypeDef* GPIOPort);

/**
 * @brief	    ��ɨ�跽ʽ��ʼ������
 * @param[out]	pKey          - ָ�򱻳�ʼ���ṹ���ָ��
 * @param[in]	pDealFunc     - ָ�����������ĺ���ָ��
 * @param[in]	Port          - ����GPIO
 * @param[in]	Pin           - �������ţ�ȡֵ GPIO_PIN_x (where x can be (0..15))
 * @param[in]	DownPolarity  - ����������ʱ�ĵ�ƽ״̬
 * @param[in]	Type          - �������ͣ�ȡֵ @KeyType_t
 * @param[in]	CheckPeriod   - ɨ�����ڣ���λΪ KEY_LOOP_PERIOD ms��ȡֵ��Χ:[1, KEY_SCAN_PERIOD)��
 * @param[in]	MaskBits      - �˲�λ����ָʾ������⵽���ٴ�ͬһ״̬��ȷ��״̬��ȡֵ��Χ:[1, 32]��
 * @retval	    None.
 * @note        ����CheckPeriod��MaskBits���ڼ��ģʽΪ����ѯɨ��ģʽ������Ч��
 *              �����߹�ͬȷ�����˲����ڣ�����ĳ��ƽά�ֶ�ú�ȷ��״̬����
 */
void Key_Scan_Init( pKey_t pKey, void (*pDealFunc)(KeyEvents_t Events),
                    KeyType_t Type, uint32_t CheckPeriod, uint32_t MaskBits,
                    GPIO_TypeDef* Port, uint16_t Pin, GPIO_PinState DownPolarity )
{
    //��ȫ���
    if( NULL == pKey      || !IS_GPIO_ALL_INSTANCE(Port) ||  
        !IS_GPIO_PIN(Pin) || !IS_GPIO_PIN_ACTION(DownPolarity) )
    { 
        KeyErrorHandler(KEY_ERROR_ParamInvalid);
        return; 
    }
    
    //��¼����Ӳ����Ϣ
    pKey->Port    = Port;
    pKey->Pin     = Pin;
    pKey->PinNum  = PinBit2PinNum(Pin);
    pKey->DownPolarity = DownPolarity;
    
#ifdef USING_BITBAND    
    //��ʼ��λ��
    pKey->Key_Pin_Read = &BIT_ADDR((uint32_t)Key_GPIO_Port + IDR_Addr_Offset, GPIO_Pin_Num);
#endif
    
    //��¼����������Ϣ
    pKey->Type = Type;
    pKey->Mode = KEY_MODE_SCAN;
    
    //�������ͽ����¼�ѡ��
    switch(pKey->Type)
    {
        case KEY_TYPE_INCHING:   //�㶯���ƣ���ע�¼�KEY_EVENT_DOWN��KEY_EVENT_UP
            
            pKey->Target_Events = (KeyEvents_t)(KEY_EVENT_DOWN | KEY_EVENT_UP);
            break;
        
        case KEY_TYPE_SWITCH:    //�������ƣ���ע�¼�KEY_EVENT_PRESS
            
            pKey->Target_Events = (KeyEvents_t)KEY_EVENT_PRESS;
            break;
        
        default:break;
    }
    
    //��������ʼ��
    pKey->pDealFunc = pDealFunc;
    
    //��ʼ��״̬�˲���
    StateFilterInit(&pKey->KeyFillter, CheckPeriod * KEY_LOOP_PERIOD, MaskBits);
}

/**
 * @brief	    ���ⲿ�жϷ�ʽ��ʼ������
 * @param[out]	pKey          - ָ�򱻳�ʼ���ṹ���ָ��
 * @param[in]	pDealFunc     - ָ�����������ĺ���ָ��
 * @param[in]	Port          - ����GPIO
 * @param[in]	Pin           - �������ţ�ȡֵ GPIO_PIN_x (where x can be (0..15))
 * @param[in]	DownPolarity  - ����������ʱ�ĵ�ƽ״̬
 * @param[in]	Type          - �������ͣ�ȡֵ @KeyType_t
 * @param[in]	Line          - �ж��ߣ�ȡֵ EXTI_LINE_x (where x can be (0..15))
 * @retval	    None.
 */
void Key_INT_Init( pKey_t pKey, void (*pDealFunc)(KeyEvents_t Events),
                   KeyType_t Type, uint32_t Line,
                   GPIO_TypeDef* Port, uint16_t Pin, GPIO_PinState DownPolarity)
{
    EXTI_ConfigTypeDef Config;
    
    //��ȫ���
    if( NULL == pKey      || !IS_GPIO_ALL_INSTANCE(Port) ||  
        !IS_GPIO_PIN(Pin) || !IS_GPIO_PIN_ACTION(DownPolarity) )
    { 
        KeyErrorHandler(KEY_ERROR_ParamInvalid);
        return; 
    }
    
    //��¼����Ӳ����Ϣ
    pKey->Port    = Port;
    pKey->Pin     = Pin;
    pKey->PinNum  = PinBit2PinNum(Pin);
    pKey->DownPolarity = DownPolarity;
    
    Config.Line = Line;
    Config.Mode = EXTI_MODE_INTERRUPT;
    Config.GPIOSel = GPIOPort2EXTIPort(Port);
    
#ifdef USING_BITBAND    
    //��ʼ��λ��
    Key_Pin_Read = &BIT_ADDR((uint32_t)Key_GPIO_Port + IDR_Addr_Offset, GPIO_Pin_Num);
#endif
    
    //��¼����������Ϣ
    pKey->Type = Type;
    pKey->Mode = KEY_MODE_INT;
    
    //�������ͽ����¼�ѡ��
    switch(pKey->Type)
    {
        case KEY_TYPE_INCHING:   //�㶯���ƣ���ע�¼�KEY_EVENT_DOWN��KEY_EVENT_UP
            
            Config.Trigger = EXTI_TRIGGER_RISING_FALLING;
            pKey->Target_Events = (KeyEvents_t)(KEY_EVENT_DOWN | KEY_EVENT_UP);
            break;
        
        case KEY_TYPE_SWITCH:    //�������ƣ���ע�¼�KEY_EVENT_PRESS
            
            Config.Trigger = (GPIO_PIN_RESET == DownPolarity ? EXTI_TRIGGER_FALLING : EXTI_TRIGGER_RISING);
            pKey->Target_Events = (KeyEvents_t)KEY_EVENT_PRESS;
            break;
        
        default:break;
    }
    
    //��ʼ���ж���
    HAL_EXTI_SetConfigLine(&pKey->hexti, &Config);
    
    //��������ʼ��
    pKey->pDealFunc = pDealFunc;
}

/**
 * @brief	ɨ�谴��״̬
 * @param	None.
 * @retval	None
 * @note    �˺��������������Եص��ã�����ʵ��Key�Ķ�̬�����˲���
 *          ��������Ϊ KEY_LOOP_PERIOD ����λΪms��
 */
void Key_Scan(void)
{
    static uint32_t TimeCnt = 0;    //ʱ�����������ʱ����
    GPIO_PinState State;
    
    //�ۼӼ�ʱ��ֵ
    TimeCnt += KEY_LOOP_PERIOD;
    
    //�������а���
    for(uint32_t i = 0; i < KeyNum; i++)
    {
        //��ѡ���ҵ�����ָ����ɨ��ʱ��
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
        
        //�¼����
        switch(GetJumpingState(&UserKeys[i].KeyFillter))
        {
            case RISING_EDGE:   //�����أ���־�û���������
                UserKeys[i].Events = (KeyEvents_t)(UserKeys[i].Target_Events & (KEY_EVENT_DOWN | KEY_EVENT_PRESS));
                break;
                
            case FALLING_EDGE:  //�½��أ���־�û���������
                UserKeys[i].Events = (KeyEvents_t)(UserKeys[i].Target_Events & KEY_EVENT_UP);
                break;
            
            case NO_JUMPING:break;
            default:break;
        }
        
        //�����¼����ô�����
        if(KEY_NOEVENT != UserKeys[i].Events)
        {
            UserKeys[i].pDealFunc(UserKeys[i].Events);
            UserKeys[i].Events = KEY_NOEVENT;
        }
    }
    
    if(TimeCnt > (KEY_SCAN_PERIOD - 1))
    {
        TimeCnt = 0;
    }
    
    return;
}

/**
 * @brief	    ɨ�谴��״̬
 * @param[in]	GPIO_Pin - �����жϵ����ź�
 * @retval	    None
 * @note        1. �˺������������ⲿ�жϵĻص������е��á�
 *              2. �����������˵��һ���������жϣ�
 *              ���޷��ж������������½��أ���˻���״̬��ȡ���жϡ�
 */
void Key_Int(uint16_t GPIO_Pin)
{
    int32_t KeyIndex = 0;
    GPIO_PinState State;
    
    //��ȡ����±�
    KeyIndex = Is_Key_INT(GPIO_Pin);
    
    //��Ϊ��Ч�±�
    if(KeyIndex > -1)
    {
#ifdef USING_BITBAND
        State = (*Key_Pin_Read);
#else
        State = HAL_GPIO_ReadPin(UserKeys[KeyIndex].Port, UserKeys[KeyIndex].Pin);
#endif  
        //���������½��ز��ǰ�ʵ�ʵ�ƽ���壬����ɨ��ģʽͳһ�����ա�������Ч�����߼�״̬����ġ�
        if(State == UserKeys[KeyIndex].DownPolarity)
        {
            //�����������ˣ�˵���߼������ط�����
            UserKeys[KeyIndex].Events = (KeyEvents_t)(UserKeys[KeyIndex].Target_Events & (KEY_EVENT_DOWN | KEY_EVENT_PRESS));
        }
        else
        {
            //�������ͷ��ˣ�˵���߼��½��ط�����
            UserKeys[KeyIndex].Events = (KeyEvents_t)(UserKeys[KeyIndex].Target_Events & KEY_EVENT_UP);
        }
        
        //�����¼����ô�����
        if(KEY_NOEVENT != UserKeys[KeyIndex].Events)
        {
            UserKeys[KeyIndex].pDealFunc(UserKeys[KeyIndex].Events);
            UserKeys[KeyIndex].Events = KEY_NOEVENT;
        }
    }
    else { return; }
    
    return;
}

/**
 * @brief	    ���ļ�ִ���еĴ�����
 * @param[in]	ErrorCode - ������
 * @retval	    None.
 */
enum KeyErrorCode KeyErrorCode = KEY_NOERROR;
static void KeyErrorHandler(enum KeyErrorCode Code)
{
    KeyErrorCode = Code;
    
    //�������˴����������KeyErrorCode�еĴ������Զ�λ����ԭ��
    while(1);
}

/**
 * @brief	    ����Ƿ��Ǳ�������Ų������ж�
 * @param[in]	GPIO_Pin - GPIO���ź�
 * @retval	    ���ǣ��򷵻�����±꣬�����ǣ��򷵻�-1.
 */
static int32_t Is_Key_INT(uint16_t GPIO_Pin)
{
    int32_t result = -1;
    
    //������ѯ
    for(int i = 0; i < KeyNum; i++)
    {
        //�ҵ���ֱ�ӽ���ѭ��
        if(GPIO_Pin == UserKeys[i].Pin)
        {
            result = i;
            break;
        }
    }
    
    return result;
}

/**
 * @brief	    ��GPIOPortת��ΪEXTIPort����
 * @param[in]	GPIOPort - GPIO�˿�
 * @retval	    EXTIPort����
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

/************************ (C) COPYRIGHT HITwh Excellent Robot Organization(HERO). *****END OF FILE****/
