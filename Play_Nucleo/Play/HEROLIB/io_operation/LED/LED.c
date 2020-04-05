/**
 ******************************************************************************
 * @file    LED.c
 * @brief   中间层：LED相关的基本操作和方法.
 * @version 1.0 示例版本
 * @author  杨亦凡
 * @contact 17863107058(手机)   942041771(qq)
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
//用户按键序列
LED_t UserLEDs[LEDNum];

/* Private macros -----------------------------------------------------------*/
#define LEVEL_MAX       100.0f  // 亮度等级最大值设定为100
#define ZOOM_FACTOR     0.01f   // (因为规定亮度等级是0-100，因此这里是除以100，即乘以0.01)
#define LED_LOOP_PERIOD 1.0f    // LED控制周期，单位ms

/* Private functions --------------------------------------------------------*/
static uint32_t PinBit2PinNum(uint32_t PinBit);
static void LEDErrorHandler(enum LEDErrorCode Code);
static inline void BreatheTask(pLED_t pLED);
static inline void TwinkleTask(pLED_t pLED);

/* Private variables --------------------------------------------------------*/
void (*pTaskFunc[LED_TASKNum])(pLED_t) = // LED任务函数指针数组，顺序依从枚举顺序
{
    NULL,
    BreatheTask,
    TwinkleTask,
};
bool isTimerUpdated = false;            // 标记计时器是否更新了，防止LED任务在主循环中被过多地调用

/**
 * @brief	    按IO方式初始化LED
 * @param[out]	pLED          - 指向被初始化结构体的指针
 * @param[in]	Port          - LED GPIO
 * @param[in]	Pin           - LED 引脚，取值 GPIO_PIN_x (where x can be (0..15))
 * @param[in]	ONPolarity    - LED 亮起时的电平状态
 * @retval	    None.
 */
void LED_ON_OFF_Init(pLED_t pLED, GPIO_TypeDef* Port, uint16_t Pin, GPIO_PinState ONPolarity)
{
    //安全检测
    if( NULL == pLED      || !IS_GPIO_ALL_INSTANCE(Port) ||  
        !IS_GPIO_PIN(Pin) || !IS_GPIO_PIN_ACTION(ONPolarity) )
    { 
        LEDErrorHandler(LED_ERROR_ParamInvalid);
        return; 
    }
    
    //记录按键硬件信息
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
    
    //标记LED种类为IO类型
    pLED->Type = LED_TYPE_ONOFF;
    
#ifdef USING_BITBAND    
    //初始化位带
    pLED->LED_Write = &BIT_ADDR((uint32_t)Port + ODR_Addr_Offset, pLED->PinNum);
#endif
    
    //初始化状态
    pLED->isTasking = false;
    
    //初始化为关闭状态
    LED_TurnOFF(pLED);
    
    return;
}

/**
 * @brief	    按可调亮度的方式初始化LED
 * @param[out]	pLED          - 指向被初始化结构体的指针
 * @param[in]	htim          - 指向LED所依赖的时基的指针
 * @param[in]	Channel       - PWM输出通道，取值 TIM_CHANNEL_x (where x can be (0..4))
 * @param[in]	Level         - LED 亮起时的亮度等级，取值 [0, LEVEL_MAX]
 * @retval	    None.
 */
void LED_Tunable_Init(pLED_t pLED, TIM_HandleTypeDef *htim, uint32_t Channel, float Level)
{
    uint16_t ARR_Value;     //获取的时基周期
    uint16_t CCR_Value = 0; //计算所得的比较值
    
    //安全检测
    if(NULL == pLED || NULL == htim || !IS_TIM_CHANNELS(Channel))
    { 
        LEDErrorHandler(LED_ERROR_ParamInvalid);
        return; 
    }
    
    //记录按键硬件信息
    pLED->htim    = htim;
    pLED->Channel = Channel;
    pLED->Level   = Level;
        
    //标记LED种类为PWM类型
    pLED->Type = LED_TYPE_Tunable;
    
    //获取周期
    ARR_Value = __HAL_TIM_GET_AUTORELOAD(htim);
    
    //按占空比映射CCR寄存器值
    CCR_Value = (uint32_t)(Level * ARR_Value * ZOOM_FACTOR);
    
    //设置CCR值
    __HAL_TIM_SET_COMPARE(htim, Channel, CCR_Value);
    
    //开启相应的设备(若已经初始化就不必再初始化)
    if(HAL_TIM_STATE_RESET == htim->State)
    {
        HAL_TIM_Base_Start(htim);
    }
    
    //初始化状态
    pLED->isTasking = false;
    
    //初始化为关闭状态
    LED_TurnOFF(pLED);
    
    return;
}

/**
 * @brief	    打开LED
 * @param[in]	pLED    - 指向被初始化结构体的指针
 * @retval	    None.
 * @note        任务状态下调用无效。
 */
void LED_TurnON(pLED_t pLED)
{
    //任务状态下不允许另外修改状态
    if(pLED->isTasking) { return; }
    
    //分类处理
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
    
    //标记状态
    pLED->State = LED_STATE_ON;
    
    return;
}

/**
 * @brief	    关闭LED
 * @param[in]	pLED    - 指向被初始化结构体的指针
 * @retval	    None.
 * @note        任务状态下调用无效。
 */
void LED_TurnOFF(pLED_t pLED)
{
    //任务状态下不允许另外修改状态
    if(pLED->isTasking) { return; }
    
    //分类处理
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
    
    //标记状态
    pLED->State = LED_STATE_OFF;
    
    return;
}

/**
 * @brief	    翻转LED亮灭状态
 * @param[in]	pLED    - 指向被初始化结构体的指针
 * @retval	    None.
 * @note        任务状态下调用无效。
 */
void LED_ON_OFF_Toggle(pLED_t pLED)
{
    //任务状态下不允许另外修改状态
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
 * @brief	    调整LED亮度
 * @param[in]	pLED    - 指向被初始化结构体的指针
 * @param[in]	Level   - 期望的亮度等级（取值范围 [0, 100]）
 * @retval	    None.
 * @note        1. 任务状态下调用无效。
 *              2. 调用此函数仅调节亮度本身，不影响开关状态。
 *              3. 仅LED_TYPE_Tunable类型的LED支持此功能，否则会引发错误处理操作。
 */
void LED_Adjustting(pLED_t pLED, float Level)
{
    uint32_t ARR_Value;     //获取的时基周期
    uint32_t CCR_Value = 0; //计算所得的比较值
    
    //任务状态下不允许另外修改状态
    if(pLED->isTasking) { return; }
    
    //错误检查
    if(LED_TYPE_ONOFF == pLED->Type)
    {
        LEDErrorHandler(LED_ERROR_IOTurning);
    }

    //限幅
    Level = (Level < 0 ? 0 : \
            (Level > LEVEL_MAX ? LEVEL_MAX : Level));
    
    //获取周期
    ARR_Value = __HAL_TIM_GET_AUTORELOAD(pLED->htim);
    
    //按占空比映射CCR寄存器值
    CCR_Value = (uint16_t)(Level * ARR_Value * ZOOM_FACTOR);
    
    //设置CCR值
    __HAL_TIM_SET_COMPARE(pLED->htim, pLED->Channel, CCR_Value);
    
    //保存Level
    pLED->Level = Level;
    
    return;
}

/**
 * @brief	    启动一次LED任务
 * @param[in]	pLED    - 指向被初始化结构体的指针
 * @param[in]	Task    - 任务类型，取值 @LEDTask_t
 * @param[in]	Period  - 任务周期，指完成一次完整变化过程所需的时间，单位ms
 * @param[in]	Times   - 任务执行次数，一个周期完成视为一次，此值置0视为无限次
 * @retval	    None.
 * @note        任务状态下调用无效。
 */
void LED_Task_Start(pLED_t pLED, LEDTask_t Task, uint32_t Period, uint32_t Times)
{
    //任务状态下不允许重复进入
    if(pLED->isTasking) { return; }
    
    //记录选定值
    pLED->TaskType = Task;
    pLED->Period   = Period;
    pLED->Times    = Times;
    
    //呼吸灯的额外计算和检查
    if(LED_TASK_BREATHE == Task)
    {
        //类型检查
        if(LED_TYPE_ONOFF == pLED->Type)
        {
            LEDErrorHandler(LED_ERROR_IOTurning);
        }
        
        //清空CCR
        LED_Adjustting(pLED, 0);
    }
    
    //清空工作值
    pLED->PeriodCnt = 0;
    pLED->TimeCnt   = 0;
    
    pLED->isTasking = true;
    
    //开启设备，分类处理
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
    
    //标记状态
    pLED->State = LED_STATE_ON;
    
    return;
}

/**
 * @brief	    终止LED任务
 * @param[in]	pLED    - 指向被初始化结构体的指针
 * @retval	    None.
 * @note        这是唯一可以结束任务、打开任务互斥锁的办法。
 */
void LED_Task_Abort(pLED_t pLED)
{
    //解除互斥锁
    pLED->isTasking = false;
    
    //关闭设备
    LED_TurnOFF(pLED);
    
    return;
}

/**
 * @brief	    变更周期性任务中的LED变化周期（频率）
 * @param[in]	pLED      - 指向被初始化结构体的指针
 * @param[in]	NewPeriod - 重新被指定的变化周期，单位ms
 * @retval	    None.
 * @note        可以在任务执行过程中生效，多用于无限次任务中修改闪烁或呼吸频率的场合。
 *              例如：对温度或功率，可以通过LED变化频率变化来反映其变化。
 */
void LED_Task_AdjPeriod(pLED_t pLED, uint32_t NewPeriod)
{
    //设定新周期
    pLED->Period = NewPeriod;
    
    return;
}

/**
 * @brief	LED周期性状态更新
 * @param	None.
 * @retval	None.
 * @note    此函数期望被周期性地调用，用来实现LED状态的动态调整以完成LED任务。
 *          调用周期为 LED_LOOP_PERIOD ，单位为ms。
 */
void LED_TaskUpdate(void)
{
    //遍历检查LED序列
    for(int i = 0; i < LEDNum; i++)
    {
        if(UserLEDs[i].isTasking)
        {           
            //计时自增
            UserLEDs[i].TimeCnt += (uint32_t)LED_LOOP_PERIOD;
            
            if(UserLEDs[i].TimeCnt >= UserLEDs[i].Period)
            {
                //计次自增
                UserLEDs[i].PeriodCnt++;
                
                //计时器清零
                UserLEDs[i].TimeCnt = 0;
                
                //检查结束条件(Times为0，视为无终止期限)
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
 * @brief	LED任务执行
 * @param	None.
 * @retval	None.
 * @note    此函数期望被在主循环中调用。
 */
void LED_TaskLoop(void)
{
    //定时器内容更新后再执行运算
    if(isTimerUpdated)
    {
        //遍历检查LED序列
        for(int i = 0; i < LEDNum; i++)
        {
            if(UserLEDs[i].isTasking)
            {
                //分类调用处理函数
                pTaskFunc[UserLEDs[i].TaskType](&UserLEDs[i]);
            }
        }
    }
    return;
}

/**
 * @brief	    匀速呼吸灯任务
 * @param[in]	pLED    - 指向被初始化结构体的指针
 * @retval	    None.
 */
static inline void BreatheTask(pLED_t pLED)
{
    uint32_t ARR_Value;        //获取的时基周期
    uint32_t CCR_Value = 0;    //计算所得的比较值
    
    float HalfPeriod = pLED->Period / 2.0;
    float delta = pLED->TimeCnt - HalfPeriod;
        
    //计算亮度等级（三角波中心对齐映射方式实现呼吸灯线性变化规律）
    pLED->Level = LEVEL_MAX * (delta < 0 ? -delta : delta) / HalfPeriod;
    
    //限幅
    pLED->Level = (pLED->Level < 0 ? 0 : \
                  (pLED->Level > LEVEL_MAX ? LEVEL_MAX : pLED->Level));
    
    //获取周期
    ARR_Value = __HAL_TIM_GET_AUTORELOAD(pLED->htim);
    
    //按占空比映射CCR寄存器值
    CCR_Value = (uint16_t)(pLED->Level * ARR_Value * ZOOM_FACTOR);
    
    //设置CCR值
    __HAL_TIM_SET_COMPARE(pLED->htim, pLED->Channel, CCR_Value);
    
    return;
}

/**
 * @brief	    闪烁灯任务
 * @param[in]	pLED    - 指向被初始化结构体的指针
 * @retval	    None.
 */
static inline void TwinkleTask(pLED_t pLED)
{
    static bool TimeFlag = false; //false代表后半周期的翻转还未执行
                                  //true 代表前半周期的翻转还未执行
    bool isCouldToggle = false;
    
    //检测跳变条件,一半一闪。
    isCouldToggle = (!TimeFlag && pLED->TimeCnt >= (pLED->Period / 2)) || 
                    (TimeFlag && pLED->TimeCnt < (pLED->Period / 2));
    
    //判断是否可翻转
    if(isCouldToggle)
    {
        if(LED_STATE_ON != pLED->State)
        {
            //分类处理
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
        
        //标记状态
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
            
            //标记状态
            pLED->State = LED_STATE_OFF;
        }
        
        //翻转标志
        TimeFlag = !TimeFlag;
    }
    
    return;
}

/**
 * @brief	    本文件执行中的错误处理
 * @param[in]	ErrorCode - 错误码
 * @retval	    None.
 */
enum LEDErrorCode LEDErrorCode = LED_NOERROR;
static void LEDErrorHandler(enum LEDErrorCode Code)
{
    LEDErrorCode = Code;
    
    //程序进入此处，请检查变量LEDErrorCode中的错误码以定位出错原因。
    while(1);
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
