/**
 ******************************************************************************
 * @file    LED.c
 * @brief   �м�㣺LED��صĻ��������ͷ���.
 * @version 1.0 ʾ���汾
 * @author  ���ෲ
 * @contact 17863107058(�ֻ�)   942041771(qq)
 * @date    2020/03/23
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
#include "LED.h"
#include "LED_User.h"

/* Public variables ---------------------------------------------------------*/
//�û���������
LED_t UserLEDs[LEDNum];

/* Private macros -----------------------------------------------------------*/
#define LEVEL_MAX       100.0f  // ���ȵȼ����ֵ�趨Ϊ100
#define ZOOM_FACTOR     0.01f   // (��Ϊ�涨���ȵȼ���0-100����������ǳ���100��������0.01)
#define LED_LOOP_PERIOD 1.0f    // LED�������ڣ���λms

/* Private functions --------------------------------------------------------*/
static uint32_t PinBit2PinNum(uint32_t PinBit);
static void LEDErrorHandler(enum LEDErrorCode Code);
static inline void BreatheTask(pLED_t pLED);
static inline void TwinkleTask(pLED_t pLED);

/* Private variables --------------------------------------------------------*/
void (*pTaskFunc[LED_TASKNum])(pLED_t) = // LED������ָ�����飬˳������ö��˳��
{
    NULL,
    BreatheTask,
    TwinkleTask,
};
bool isTimerUpdated = false;            // ��Ǽ�ʱ���Ƿ�����ˣ���ֹLED��������ѭ���б�����ص���

/**
 * @brief	    ��IO��ʽ��ʼ��LED
 * @param[out]	pLED          - ָ�򱻳�ʼ���ṹ���ָ��
 * @param[in]	Port          - LED GPIO
 * @param[in]	Pin           - LED ���ţ�ȡֵ GPIO_PIN_x (where x can be (0..15))
 * @param[in]	ONPolarity    - LED ����ʱ�ĵ�ƽ״̬
 * @retval	    None.
 */
void LED_ON_OFF_Init(pLED_t pLED, GPIO_TypeDef* Port, uint16_t Pin, GPIO_PinState ONPolarity)
{
    //��ȫ���
    if( NULL == pLED      || !IS_GPIO_ALL_INSTANCE(Port) ||  
        !IS_GPIO_PIN(Pin) || !IS_GPIO_PIN_ACTION(ONPolarity) )
    { 
        LEDErrorHandler(LED_ERROR_ParamInvalid);
        return; 
    }
    
    //��¼����Ӳ����Ϣ
    pLED->Port    = Port;
    pLED->Pin     = Pin;
    pLED->PinNum  = PinBit2PinNum(Pin);
    
    if(GPIO_PIN_RESET == ONPolarity)
    {
        pLED->ONPolarity = GPIO_PIN_RESET;
        pLED->OFFPolarity = GPIO_PIN_SET;
    }
    else
    {
        pLED->ONPolarity = GPIO_PIN_SET;
        pLED->OFFPolarity = GPIO_PIN_RESET;
    }
    
    //���LED����ΪIO����
    pLED->Type = LED_TYPE_ONOFF;
    
#ifdef USING_BITBAND    
    //��ʼ��λ��
    pLED->LED_Write = &BIT_ADDR((uint32_t)Port + ODR_Addr_Offset, pLED->PinNum);
#endif
    
    //��ʼ��״̬
    pLED->isTasking = false;
    
    //��ʼ��Ϊ�ر�״̬
    LED_TurnOFF(pLED);
    
    return;
}

/**
 * @brief	    ���ɵ����ȵķ�ʽ��ʼ��LED
 * @param[out]	pLED          - ָ�򱻳�ʼ���ṹ���ָ��
 * @param[in]	htim          - ָ��LED��������ʱ����ָ��
 * @param[in]	Channel       - PWM���ͨ����ȡֵ TIM_CHANNEL_x (where x can be (0..4))
 * @param[in]	Level         - LED ����ʱ�����ȵȼ���ȡֵ [0, LEVEL_MAX]
 * @retval	    None.
 */
void LED_Tunable_Init(pLED_t pLED, TIM_HandleTypeDef *htim, uint32_t Channel, float Level)
{
    uint16_t ARR_Value;     //��ȡ��ʱ������
    uint16_t CCR_Value = 0; //�������õıȽ�ֵ
    
    //��ȫ���
    if(NULL == pLED || NULL == htim || !IS_TIM_CHANNELS(Channel))
    { 
        LEDErrorHandler(LED_ERROR_ParamInvalid);
        return; 
    }
    
    //��¼����Ӳ����Ϣ
    pLED->htim    = htim;
    pLED->Channel = Channel;
    pLED->Level   = Level;
        
    //���LED����ΪPWM����
    pLED->Type = LED_TYPE_Tunable;
    
    //��ȡ����
    ARR_Value = __HAL_TIM_GET_AUTORELOAD(htim);
    
    //��ռ�ձ�ӳ��CCR�Ĵ���ֵ
    CCR_Value = (uint32_t)(Level * ARR_Value * ZOOM_FACTOR);
    
    //����CCRֵ
    __HAL_TIM_SET_COMPARE(htim, Channel, CCR_Value);
    
    //������Ӧ���豸(���Ѿ���ʼ���Ͳ����ٳ�ʼ��)
    if(HAL_TIM_STATE_RESET == htim->State)
    {
        HAL_TIM_Base_Start(htim);
    }
    
    //��ʼ��״̬
    pLED->isTasking = false;
    
    //��ʼ��Ϊ�ر�״̬
    LED_TurnOFF(pLED);
    
    return;
}

/**
 * @brief	    ��LED
 * @param[in]	pLED    - ָ�򱻳�ʼ���ṹ���ָ��
 * @retval	    None.
 * @note        ����״̬�µ�����Ч��
 */
void LED_TurnON(pLED_t pLED)
{
    //����״̬�²����������޸�״̬
    if(pLED->isTasking) { return; }
    
    //���ദ��
    switch(pLED->Type)
    {
        case LED_TYPE_ONOFF:
#ifdef  USING_BITBAND
            pLED->LED_Write = pLED->ONPolarity;
#else
            HAL_GPIO_WritePin(pLED->Port, pLED->Pin, pLED->ONPolarity);
#endif        
            break;
        
        case LED_TYPE_Tunable:
            
            HAL_TIM_PWM_Start(pLED->htim, pLED->Channel);
            break;
        
        default:break;
    }
    
    //���״̬
    pLED->State = LED_STATE_ON;
    
    return;
}

/**
 * @brief	    �ر�LED
 * @param[in]	pLED    - ָ�򱻳�ʼ���ṹ���ָ��
 * @retval	    None.
 * @note        ����״̬�µ�����Ч��
 */
void LED_TurnOFF(pLED_t pLED)
{
    //����״̬�²����������޸�״̬
    if(pLED->isTasking) { return; }
    
    //���ദ��
    switch(pLED->Type)
    {
        case LED_TYPE_ONOFF:
#ifdef  USING_BITBAND
            pLED->LED_Write = pLED->OFFPolarity;
#else
            HAL_GPIO_WritePin(pLED->Port, pLED->Pin, pLED->OFFPolarity);
#endif     
            break;
        
        case LED_TYPE_Tunable:
            
            HAL_TIM_PWM_Stop(pLED->htim, pLED->Channel);
            break;
        
        default:break;
    }
    
    //���״̬
    pLED->State = LED_STATE_OFF;
    
    return;
}

/**
 * @brief	    ��תLED����״̬
 * @param[in]	pLED    - ָ�򱻳�ʼ���ṹ���ָ��
 * @retval	    None.
 * @note        ����״̬�µ�����Ч��
 */
void LED_ON_OFF_Toggle(pLED_t pLED)
{
    //����״̬�²����������޸�״̬
    if(pLED->isTasking) { return; }
    
    if(LED_STATE_ON != pLED->State)
    {
        LED_TurnON(pLED);
    }
    else
    {
        LED_TurnOFF(pLED);
    }
    
    return;
}

/**
 * @brief	    ����LED����
 * @param[in]	pLED    - ָ�򱻳�ʼ���ṹ���ָ��
 * @param[in]	Level   - ���������ȵȼ���ȡֵ��Χ [0, 100]��
 * @retval	    None.
 * @note        1. ����״̬�µ�����Ч��
 *              2. ���ô˺������������ȱ�����Ӱ�쿪��״̬��
 *              3. ��LED_TYPE_Tunable���͵�LED֧�ִ˹��ܣ���������������������
 */
void LED_Adjustting(pLED_t pLED, float Level)
{
    uint32_t ARR_Value;     //��ȡ��ʱ������
    uint32_t CCR_Value = 0; //�������õıȽ�ֵ
    
    //����״̬�²����������޸�״̬
    if(pLED->isTasking) { return; }
    
    //������
    if(LED_TYPE_ONOFF == pLED->Type)
    {
        LEDErrorHandler(LED_ERROR_IOTurning);
    }

    //�޷�
    Level = (Level < 0 ? 0 : \
            (Level > LEVEL_MAX ? LEVEL_MAX : Level));
    
    //��ȡ����
    ARR_Value = __HAL_TIM_GET_AUTORELOAD(pLED->htim);
    
    //��ռ�ձ�ӳ��CCR�Ĵ���ֵ
    CCR_Value = (uint16_t)(Level * ARR_Value * ZOOM_FACTOR);
    
    //����CCRֵ
    __HAL_TIM_SET_COMPARE(pLED->htim, pLED->Channel, CCR_Value);
    
    //����Level
    pLED->Level = Level;
    
    return;
}

/**
 * @brief	    ����һ��LED����
 * @param[in]	pLED    - ָ�򱻳�ʼ���ṹ���ָ��
 * @param[in]	Task    - �������ͣ�ȡֵ @LEDTask_t
 * @param[in]	Period  - �������ڣ�ָ���һ�������仯���������ʱ�䣬��λms
 * @param[in]	Times   - ����ִ�д�����һ�����������Ϊһ�Σ���ֵ��0��Ϊ���޴�
 * @retval	    None.
 * @note        ����״̬�µ�����Ч��
 */
void LED_Task_Start(pLED_t pLED, LEDTask_t Task, uint32_t Period, uint32_t Times)
{
    //����״̬�²������ظ�����
    if(pLED->isTasking) { return; }
    
    //��¼ѡ��ֵ
    pLED->TaskType = Task;
    pLED->Period   = Period;
    pLED->Times    = Times;
    
    //�����ƵĶ������ͼ��
    if(LED_TASK_BREATHE == Task)
    {
        //���ͼ��
        if(LED_TYPE_ONOFF == pLED->Type)
        {
            LEDErrorHandler(LED_ERROR_IOTurning);
        }
        
        //���CCR
        LED_Adjustting(pLED, 0);
    }
    
    //��չ���ֵ
    pLED->PeriodCnt = 0;
    pLED->TimeCnt   = 0;
    
    pLED->isTasking = true;
    
    //�����豸�����ദ��
    switch(pLED->Type)
    {
        case LED_TYPE_ONOFF:
#ifdef  USING_BITBAND
            pLED->LED_Write = pLED->ONPolarity;
#else
            HAL_GPIO_WritePin(pLED->Port, pLED->Pin, pLED->ONPolarity);
#endif        
            break;
        
        case LED_TYPE_Tunable:
            
            HAL_TIM_PWM_Start(pLED->htim, pLED->Channel);
            break;
        
        default:break;
    }
    
    //���״̬
    pLED->State = LED_STATE_ON;
    
    return;
}

/**
 * @brief	    ��ֹLED����
 * @param[in]	pLED    - ָ�򱻳�ʼ���ṹ���ָ��
 * @retval	    None.
 * @note        ����Ψһ���Խ������񡢴����񻥳����İ취��
 */
void LED_Task_Abort(pLED_t pLED)
{
    //���������
    pLED->isTasking = false;
    
    //�ر��豸
    LED_TurnOFF(pLED);
    
    return;
}

/**
 * @brief	    ��������������е�LED�仯���ڣ�Ƶ�ʣ�
 * @param[in]	pLED      - ָ�򱻳�ʼ���ṹ���ָ��
 * @param[in]	NewPeriod - ���±�ָ���ı仯���ڣ���λms
 * @retval	    None.
 * @note        ����������ִ�й�������Ч�����������޴��������޸���˸�����Ƶ�ʵĳ��ϡ�
 *              ���磺���¶Ȼ��ʣ�����ͨ��LED�仯Ƶ�ʱ仯����ӳ��仯��
 */
void LED_Task_AdjPeriod(pLED_t pLED, uint32_t NewPeriod)
{
    //�趨������
    pLED->Period = NewPeriod;
    
    return;
}

/**
 * @brief	LED������״̬����
 * @param	None.
 * @retval	None.
 * @note    �˺��������������Եص��ã�����ʵ��LED״̬�Ķ�̬���������LED����
 *          ��������Ϊ LED_LOOP_PERIOD ����λΪms��
 */
void LED_TaskUpdate(void)
{
    //�������LED����
    for(int i = 0; i < LEDNum; i++)
    {
        if(UserLEDs[i].isTasking)
        {           
            //��ʱ����
            UserLEDs[i].TimeCnt += (uint32_t)LED_LOOP_PERIOD;
            
            if(UserLEDs[i].TimeCnt >= UserLEDs[i].Period)
            {
                //�ƴ�����
                UserLEDs[i].PeriodCnt++;
                
                //��ʱ������
                UserLEDs[i].TimeCnt = 0;
                
                //����������(TimesΪ0����Ϊ����ֹ����)
                if((UserLEDs[i].Times > 0) && (UserLEDs[i].PeriodCnt >= UserLEDs[i].Times))
                {
                    LED_Task_Abort(&UserLEDs[i]);
                }
            }
        }
    }
    
    isTimerUpdated = true;
    
    return;
}

/**
 * @brief	LED����ִ��
 * @param	None.
 * @retval	None.
 * @note    �˺�������������ѭ���е��á�
 */
void LED_TaskLoop(void)
{
    //��ʱ�����ݸ��º���ִ������
    if(isTimerUpdated)
    {
        //�������LED����
        for(int i = 0; i < LEDNum; i++)
        {
            if(UserLEDs[i].isTasking)
            {
                //������ô�����
                pTaskFunc[UserLEDs[i].TaskType](&UserLEDs[i]);
            }
        }
    }
    return;
}

/**
 * @brief	    ���ٺ���������
 * @param[in]	pLED    - ָ�򱻳�ʼ���ṹ���ָ��
 * @retval	    None.
 */
static inline void BreatheTask(pLED_t pLED)
{
    uint32_t ARR_Value;        //��ȡ��ʱ������
    uint32_t CCR_Value = 0;    //�������õıȽ�ֵ
    
    float HalfPeriod = pLED->Period / 2.0;
    float delta = pLED->TimeCnt - HalfPeriod;
        
    //�������ȵȼ������ǲ����Ķ���ӳ�䷽ʽʵ�ֺ��������Ա仯���ɣ�
    pLED->Level = LEVEL_MAX * (delta < 0 ? -delta : delta) / HalfPeriod;
    
    //�޷�
    pLED->Level = (pLED->Level < 0 ? 0 : \
                  (pLED->Level > LEVEL_MAX ? LEVEL_MAX : pLED->Level));
    
    //��ȡ����
    ARR_Value = __HAL_TIM_GET_AUTORELOAD(pLED->htim);
    
    //��ռ�ձ�ӳ��CCR�Ĵ���ֵ
    CCR_Value = (uint16_t)(pLED->Level * ARR_Value * ZOOM_FACTOR);
    
    //����CCRֵ
    __HAL_TIM_SET_COMPARE(pLED->htim, pLED->Channel, CCR_Value);
    
    return;
}

/**
 * @brief	    ��˸������
 * @param[in]	pLED    - ָ�򱻳�ʼ���ṹ���ָ��
 * @retval	    None.
 */
static inline void TwinkleTask(pLED_t pLED)
{
    static bool TimeFlag = false; //false���������ڵķ�ת��δִ��
                                  //true ����ǰ�����ڵķ�ת��δִ��
    bool isCouldToggle = false;
    
    //�����������,һ��һ����
    isCouldToggle = (!TimeFlag && pLED->TimeCnt >= (pLED->Period / 2)) || 
                    (TimeFlag && pLED->TimeCnt < (pLED->Period / 2));
    
    //�ж��Ƿ�ɷ�ת
    if(isCouldToggle)
    {
        if(LED_STATE_ON != pLED->State)
        {
            //���ദ��
            switch(pLED->Type)
            {
                case LED_TYPE_ONOFF:
#ifdef  USING_BITBAND
                    pLED->LED_Write = pLED->ONPolarity;
#else
                    HAL_GPIO_WritePin(pLED->Port, pLED->Pin, pLED->ONPolarity);
#endif        
                    break;
                
                case LED_TYPE_Tunable:
                    
                    HAL_TIM_PWM_Start(pLED->htim, pLED->Channel);
                    break;
                
                default:break;
        }
        
        //���״̬
        pLED->State = LED_STATE_ON;
        }
        else
        {
            switch(pLED->Type)
            {
                case LED_TYPE_ONOFF:
#ifdef  USING_BITBAND
                    pLED->LED_Write = pLED->OFFPolarity;
#else
                    HAL_GPIO_WritePin(pLED->Port, pLED->Pin, pLED->OFFPolarity);
#endif     
                    break;
                
                case LED_TYPE_Tunable:
                    
                    HAL_TIM_PWM_Stop(pLED->htim, pLED->Channel);
                    break;
                
                default:break;
            }
            
            //���״̬
            pLED->State = LED_STATE_OFF;
        }
        
        //��ת��־
        TimeFlag = !TimeFlag;
    }
    
    return;
}

/**
 * @brief	    ���ļ�ִ���еĴ�����
 * @param[in]	ErrorCode - ������
 * @retval	    None.
 */
enum LEDErrorCode LEDErrorCode = LED_NOERROR;
static void LEDErrorHandler(enum LEDErrorCode Code)
{
    LEDErrorCode = Code;
    
    //�������˴����������LEDErrorCode�еĴ������Զ�λ����ԭ��
    while(1);
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
