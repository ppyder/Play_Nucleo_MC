/**
 ******************************************************************************
 * @file    AS5048a.c
 * @brief   中间层：磁编码器AS5048a驱动库.
 * @version 1.0 示例版本
 * @author  杨亦凡
 * @contact 17863107058(手机)   942041771(qq)
 * @date    2020/04/05
 *
 * @note    1. 目前仅支持与单个AS5048a的通信。
 *          2. 角度信息定时触发读取，此外数据均采用阻塞式数据读取。
 *          3. 为了腾出阻塞等候时间以及读寄存器确认数据的时间，定时触发的数据读取采用了DMA中断方式。
 *          4. 对AS5048来说涉及初始化和通信的操作都是互斥且不可重入的。
 *          5. 组件提供ms级瞬时速度和实时位置信息，并提供相应的可靠性评估标志。
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
#include "AS5048a.h"
#include "TinyDelay.h"
#include <math.h>

/* Private macros -----------------------------------------------------------*/
#define AS5048A_DATAMAX          (16384)                                // 14位数据最大值(0x3FFF + 1)
#define AS5048A_DATA2ANGLE(DATA) ((DATA) * (360.0 / AS5048A_DATAMAX))   // 由原始数据转换为角度

//计算使用的单位是“°/s”，表示使用的转速单位是RPM。
//乘以以下比率，可转换单位。
#define SPEED2RPM_UNIT_RATIO (0.1666667f) //Ratio = 60/360 = 1/6;
#define RPM2SPEED_UNIT_RATIO (6.0f)

//速度滑动平均值滤波器数据缓冲区大小
#define SPEED_FILTER_SIZE 20

/* Public variables --------------------------------------------------------*/
AS5048a_t AS5048a =  // 组件实例
{
    .hspi = NULL,
    .NSSPort = NULL,
    .State = AS5048A_STATE_RESET,
    .LastCMD = AS5048A_CMD_READNOP,
};
HAL_StatusTypeDef AS5048A_HAL_ErrorCode = HAL_OK;// 通信错误码，调试用

/* Private functions --------------------------------------------------------*/
static void UpdatePos_Speed(pAS5048a_t pDev);
static bool AS5048a_ResetZeroPos(pAS5048a_t pDev);
static bool AS5048a_ReadREG_Block(pAS5048a_t pDev, uint16_t ReadCMD, volatile uint16_t *pData);
static bool AS5048a_WriteREG_Block(pAS5048a_t pDev, uint16_t Address, uint16_t Data);
static bool AS5048a_ClearErrorFlags_Block(pAS5048a_t pDev, uint32_t Timeout);
static bool AS5048a_ReadWrite_DMA(pAS5048a_t pDev, uint16_t *pTxBuffer, volatile uint16_t *pRxBuffer);
static void AS5048a_ReadWrite_DMACallBack(pAS5048a_t pDev);
static bool AS5048a_ReadWrite_Block(pAS5048a_t pDev, uint16_t *pTxBuffer, volatile uint16_t *pRxBuffer);
static inline void AS5048A_SetErrorCode(pAS5048a_t pDev, AS5048a_State_t Error);
static inline void AS5048a_NSS_SET(pAS5048a_t pDevice);
static inline void AS5048a_NSS_RESET(pAS5048a_t pDevice);
static inline uint16_t AS5048a_Parity_EVEN(uint16_t Num);
static uint32_t PinBit2PinNum(uint32_t PinBit);

/* Private variables ---------------------------------------------------------*/
float Speed_Buffer[SPEED_FILTER_SIZE] = {0}; // 循环队列:滑动平均值滤波数据缓冲区


/**
 * @brief	    初始化设备AS5048a
 * @param[out]	pDev          - 指向被初始化的组件结构体地址
 * @param[in]	MaxRPM        - 可能的最大转速，用于估计最大角位移以判断转速信息可靠性。
 * @param[in]	NSSPort       - NSS引脚的GPIO端口，取值 GPIOx （x = A,B,C...）
 * @param[in]	NSS_Pin       - NSS引脚的引脚号，取值 GPIO_PIN_x (where x can be (0..15))
 * @param[in]	hspi          - 所调用的片上外设句柄。
 * @param[in]	FrameTime     - SPI传输一帧数据所花的总时间(含程序处理时间)，基于实测，单位ns。
 * @param[in]	MaxErrorTimes - 连续通信中，允许[连续出现的错误次数]的最大值。
 * @retval	    初始化是否成功(bool)，成功为true.
 * @note        1. SPI外设应配置为：软件管理的NSS, 16bit数据, MSB First, <10Mbit/s, CPOL_Low, CPHA_2Edge。
 *              2. 此函数可产生错误标志: AS5048A_STATE_CMD_ERROR 及优先级更高的错误。
 */
bool AS5048a_Init( pAS5048a_t pDev, float MaxRPM, 
                   GPIO_TypeDef* NSSPort, uint16_t NSS_Pin, 
                   SPI_HandleTypeDef* hspi, uint16_t FrameTime, uint16_t MaxErrorTimes )
{
    //非法检查
    if( NULL == pDev  ||  NULL == NSSPort  ||     NULL == hspi      || 0 == FrameTime || 0 == MaxErrorTimes ||
        !IS_GPIO_ALL_INSTANCE(NSSPort)     || !IS_GPIO_PIN(NSS_Pin) || HAL_SPI_STATE_READY != hspi->State)
    { return false; }
        
    //硬件底层初始化
    pDev->hspi = hspi;
    pDev->NSSPort = NSSPort;
    pDev->NSSPin = NSS_Pin;
    pDev->NSSPinNum = PinBit2PinNum(NSS_Pin);
#ifdef USING_BITBAND    
    //初始化位带
    pLED->NSS_Write = &BIT_ADDR((uint32_t)NSSPort + ODR_Addr_Offset, pDev->NSSPinNum);
#endif

    //初始化标志
    pDev->isDebuging      = false;  // 处于非Debug状态
    pDev->isSpeedReliable = true;   // 速度数据可靠
    pDev->isDataInvalid   = false;  // 初始帧数据合法
    
    //初始化工作数据
    pDev->BaseAngle  = 0;
    pDev->RealAngle  = 0;
    pDev->Speed      = 0;
    pDev->MaxRPM     = MaxRPM;
    pDev->ErrorFlags.Value = 0x00;
    
    pDev->FrameTime      = FrameTime;
    pDev->LastCMD        = AS5048A_CMD_READNOP; // 标记上次读取的是空数据
    pDev->PeriodErrorCnt = 0;
    pDev->ErrorMax       = MaxErrorTimes;
    pDev->TempRxBuffer   = 0x00;
    
    pDev->ReadMoment.CarryNum = 0;
    pDev->ReadMoment.Ticks    = 0;
    
    //初始化速度滑动平均值滤波器
    SlidingAveFilterInit(&pDev->SpeedFilter, Speed_Buffer, SPEED_FILTER_SIZE, 0);
    
    //拉高片选脚
    AS5048a_NSS_SET(pDev);
    
    //诊断工作条件
    if(!AS5048a_DiagnoseConditions(pDev)) { return false; }
    
    //重设零位
    if(!AS5048a_ResetZeroPos(pDev))
    {
        AS5048A_SetErrorCode(pDev, AS5048A_STATE_ZP_ERROR);
        return false;
    }
    
    //标记初始化完成
    pDev->State = AS5048A_STATE_READY;
    return true;
}

/**
 * @brief	    从组件获取实时位置信息
 * @param[in]	pDev      - 指向被操作的设备地址
 * @param[out]	pAngle    - 指向存储角度信息的变量地址
 * @retval	    获取是否成功(bool)，成功为true.
 * @note        1. 错误状态下和未初始化状态下不更新位置信息。
 *              2. 当速度数据可靠时，将返回基准位置与读取时间间隙对应的增量位置的和作为估测实时位置。
 *              3. 当速度数据不可靠时，将返回基准位置作为近似的实时位置。
 */
bool AS5048a_GetRealTimeAngle(pAS5048a_t pDev, float *pAngle)
{    
    if(NULL == pDev || IS_AS5048A_ERROR(pDev->State) || AS5048A_STATE_RESET == pDev->State) 
    { return false; }
    
    if(pDev->isSpeedReliable)
    {
        float DeltaTime = 0;
        GlobalTime_t TempMoment;
        
        TimeRef_GetPreciseTime(&TempMoment);
        DeltaTime = TimeRef_TimeMinus(&TempMoment, &pDev->ReadMoment);
        
        (*pAngle) = pDev->BaseAngle + DeltaTime * pDev->Speed * RPM2SPEED_UNIT_RATIO;
    }
    else
    {
        (*pAngle) = pDev->BaseAngle;
    }
    
    return true;
}

/**
 * @brief	    从组件获取实时速度信息
 * @param[in]	pDev      - 指向被操作的设备地址
 * @param[out]	pSpeed    - 指向存储速度信息的变量地址
 * @retval	    获取是否成功(bool)，成功为true.
 * @note        1. 错误状态下和未初始化状态下不更新速度信息。
 *              2. 当速度数据不可靠时，将置零速度。
 */
bool AS5048a_GetSpeed(pAS5048a_t pDev, float *pSpeed)
{
    if(NULL == pDev || IS_AS5048A_ERROR(pDev->State) || AS5048A_STATE_RESET == pDev->State) 
    { return false; }
    
    if(pDev->isSpeedReliable)
    {
        (*pSpeed) = pDev->Speed;
    }
    else
    {
        (*pSpeed) = 0;
        return false;
    }
    return true;
}

/**
 * @brief	    诊断工作条件是否符合要求
 * @param[in]	pDev      - 指向被操作的设备地址
 * @retval	    工作条件是否符合要求(bool)，符合为true.
 * @note        1. 将检查通信条件、磁场状况、CORDIC算法溢出情况以及偏移补偿算法完成情况。
 *              2. 此函数可产生错误标志: AS5048A_STATE_DIG_ERROR 及优先级更高的错误。
 *              3. 此函数可消除错误标志: AS5048A_STATE_DIG_ERROR ，但只能在已有优先级低于或等于此错误的情况下。
 */
bool AS5048a_DiagnoseConditions(pAS5048a_t pDev)
{
    bool Result = true;
    
    if(NULL == pDev || (IS_AS5048A_ERROR(pDev->State) && (pDev->State < AS5048A_STATE_DIG_ERROR)))
    { return false; }
    
    //置位调试标志以暂时屏蔽周期性工作行为
    pDev->isDebuging = true;
    
    //等待可能存在的上个通讯过程结束
    while(AS5048A_STATE_BUSY == AS5048a.State);
    
    //进行通信检查和错误标志更新，给予充分的时间(10ms)清除错误标志
    if(!AS5048a_ClearErrorFlags_Block(pDev, 10000)) 
    { return false; }

    //获取状态数据
lable_ReReadAGC:
    if(!AS5048a_ReadREG_Block(pDev, AS5048A_CMD_READAGC, &pDev->AGCR.Value)) 
    { 
        if(!IS_AS5048A_ERROR(pDev->State))
        {
            //如果没有进入错误状态，说明出现了偶发性的读错误，可重新执行读操作。
            goto lable_ReReadAGC;
        }
        return false; 
    }

lable_ReReadMAG:
    if(!AS5048a_ReadREG_Block(pDev, AS5048A_CMD_READMAG, &pDev->MAGR.Value)) 
    { 
        if(!IS_AS5048A_ERROR(pDev->State))
        {
            //如果没有进入错误状态，说明出现了偶发性的读错误，可重新执行读操作。
            goto lable_ReReadMAG;
        }
        return false; 
    }
        
    //磁场过强、过弱、CORDIC算法溢出都表示工作状态异常
    Result = !(pDev->AGCR.bit_area.Comp_H || pDev->AGCR.bit_area.Comp_L || pDev->AGCR.bit_area.COF);
    
    //自动增益补偿未完成也表示工作状态异常
    Result = Result && (pDev->AGCR.bit_area.OCF);
    
    //标记状态
    if(!Result)
    {
        AS5048A_SetErrorCode(pDev, AS5048A_STATE_DIG_ERROR);
    }
    else
    {
        //初始化函数结束前不应该提前标记初始化完成。
        if(AS5048A_STATE_RESET != pDev->State)
        {
            pDev->State = AS5048A_STATE_READY;
        }
    }
    
    //复位调试标志以恢复周期性工作行为
    pDev->isDebuging = false;
    
    return Result;
}

/**
 * @brief	    触发非阻塞(DMA)方式读取角度
 * @param[in]	pDev      - 指向被操作的设备地址
 * @retval	    None.
 * @note        1. 将完成硬件层通信协议的维护。
 *              2. 将完成错误管理，并可能产生错误标志: AS5048A_STATE_DIG_ERROR 及优先级更高的错误。
 *              3. 建议定时调用。在传输一帧数据留有30%裕量的时间基础上，频率越高速度数据越优质。
 */
void AS5048a_UpdateAngleSpeed_DMA(pAS5048a_t pDev)
{
    uint16_t TempCMD = AS5048A_CMD_READANGLE;
    
    //非自检模式下的待命状态才可用
    if(pDev->isDebuging || AS5048A_STATE_READY != AS5048a.State) { return; }
    
    //检查上次通信中是否有错误
    if(pDev->isDataInvalid)
    {
        pDev->PeriodErrorCnt++;
        if(pDev->PeriodErrorCnt >= pDev->ErrorMax)
        {
            pDev->State = AS5048A_STATE_CMD_ERROR;
            return;
        }
        //将原先的读指令变为清除错误指令
        TempCMD = AS5048A_CMD_READEF;
    }
    else if(pDev->PeriodErrorCnt > 0)   
    {
        //首次回归正常状态时清零计数器
        pDev->PeriodErrorCnt = 0;
    }
    
    //标记设备占用
    pDev->State = AS5048A_STATE_BUSY;
    
    //触发非阻塞读寄存器操作
    if(!AS5048a_ReadWrite_DMA(pDev, &TempCMD, &pDev->TempRxBuffer)) { return; }
        
    //记录本次发送的指令
    pDev->LastCMD = TempCMD;
    
    return;
}

/**
 * @brief	    AS5048a的DMA接收中断回调
 * @param[in]	pDev      - 指向被操作的设备地址
 * @retval	    None.
 * @note        1. 将完成硬件层通信协议的维护。
 *              2. 将对接收到的数据进行校验，并管理通信错误标志 @isDataInvalid 。
 *              3. 将调用数据处理函数。
 */
void AS5048a_DMARx_Callback(pAS5048a_t pDev)
{
    //释放占用标志并维护底层操作
    pDev->State = AS5048A_STATE_READY;
    AS5048a_ReadWrite_DMACallBack(pDev);
     
    /* 对读取到的数据进行处理。
      错误标志置一，说明在上次数据传输中传感器没有接收到来自主机的有效指令。
      即，本次接收到的数据不可信。 */
    pDev->isDataInvalid = (0 != (AS5048A_HOST_ERROR & pDev->TempRxBuffer));
    
    if(!pDev->isDataInvalid)
    {
        switch(pDev->LastCMD)
        {
            case AS5048A_CMD_READANGLE:
                
                UpdatePos_Speed(pDev);
                break;
            
            case AS5048A_CMD_READEF:
                
                pDev->ErrorFlags.Value = pDev->TempRxBuffer;;
                break;
            
            default: break;
        }
    }
    return;
}

/* Private functions define --------------------------------------------------*/
/**
 * @brief	    计算位置和速度
 * @param[in]	pDev      - 指向被操作的设备地址
 * @retval	    None.
 * @note        1. 将在AS5048的DMA接收回调中，确认角度数据可靠后调用。
 */
static void UpdatePos_Speed(pAS5048a_t pDev)
{
    static int32_t InvalidCnt = 0;    // 数据的[连续不可靠次数]计数器
    bool   isThisInvalid      = false;// 数据可靠标志
    
    float   DeltaTime_s = 0;   // 测速间隔时间
    float   DeltaAngle  = 0;   // 测速间隔位移
    float   PositiveAngle = 0; // 角度变化量绝对值
    float   LastAngle   = 0;   // 上次角位置
    float   MaxDelta    = 0;   // 最大转速下的角位置增量
    GlobalTime_t LastMoment;   // 上次读取位置的时刻

    //保存上次读取位置时的信息
    LastAngle = pDev->BaseAngle;
    LastMoment = pDev->ReadMoment;
    
    //更新角度制基准角度以及读取时刻
    pDev->BaseAngle = AS5048A_DATA2ANGLE(pDev->TempRxBuffer & AS5048A_DATAMASK);
    TimeRef_GetPreciseTime(&pDev->ReadMoment);
    
    //获取时间间隔
    DeltaTime_s = TimeRef_TimeMinus(&pDev->ReadMoment, &LastMoment);
        
    //计算可能的最大角位移增量(此值必为正)
    MaxDelta = DeltaTime_s * pDev->MaxRPM * RPM2SPEED_UNIT_RATIO;
    
    //获取角位置增量
    DeltaAngle = pDev->BaseAngle - LastAngle;
    PositiveAngle = fabs(DeltaAngle);
    
    /* 最大转速下增量大于半圈或本次角度增量大于最大转速下的角度增量时，按当前数据算出的速度不可靠。
     *  总是不可靠的话，需要检查读取位置的频率是否满足能够可靠测速的底线标准。
     *  可靠测速的底线标准是，最大转速下至少能在一整圈内读取两次位置。
     * 此法不可检测出因传感器问题导致的数据错误，这样的错误数据应当在应用层中予以消除。*/
    isThisInvalid = (MaxDelta > 180.0f) || 
                    (PositiveAngle > 180.0f && PositiveAngle < (360.0f - MaxDelta));
    
    if(isThisInvalid)
    {
        InvalidCnt++;
        if(InvalidCnt >= AS5048A_MAX_SPEED_INVALID_CNT)
        {
            pDev->isSpeedReliable = false;
        }
    }
    else
    {
        if(fabs(DeltaAngle) > 180.0f) // 若发生了圈数跳变
        {
            DeltaAngle += (DeltaAngle < 0 ? 360.0f : -360.0f);
        }
        
        //由δθ和δt计算角速度
        pDev->RawSpeed = DeltaAngle / DeltaTime_s * SPEED2RPM_UNIT_RATIO;
        
        if(!pDev->isSpeedReliable) //若原先速度数据不可靠，这说明有很长时间速度队列没有更新了，
                                   //滑动平均值滤波会降低测算值跟随实际值的速度，因此要重置数据缓冲区。
        {
            pDev->isSpeedReliable = true;
            SetSlidingAveBuffer(&pDev->SpeedFilter, pDev->RawSpeed);
            
            pDev->Speed = pDev->RawSpeed;
        }
        else
        {
            //滑动平均值滤波
            pDev->Speed = SlidingAveFilter(&pDev->SpeedFilter, pDev->RawSpeed);
        }
        InvalidCnt = 0;
    }
    return;
}

/**
 * @brief	    设定当前位置为零位
 * @param[in]	pDev      - 指向被操作的设备地址
 * @retval	    是否成功(bool)，成功为true.
 * @note        1. 设备每次上电只能设定一次零位，这属于OTP操作。
 *              2. 将在设备的初始化函数中被调用。
 *              3. 若在同一次上电中多次调用该函数，也不会对设备工作产生负面影响，顶多是返回false。
 */
static bool AS5048a_ResetZeroPos(pAS5048a_t pDev)
{
    uint32_t TempAngle = 0; // 零位修正前的读取值
    uint16_t WriteCMD  = 0; // 临时的写指令发送缓冲区
    uint16_t TempValue = 0; // 临时的数据接收缓冲区
    
    AS5048a_ANGR_t ANGR = {0};
    AS5048a_ZPHR_t ZPHR = {0};
    AS5048a_ZPLR_t ZPLR = {0};
    AS5048a_PCR_t  PCR  = {0};
        
    /* 读20次角度寄存器，平均值滤波 */
    for(int i = 0; i < 20; i++)
    {
        if(!AS5048a_ReadREG_Block(pDev, AS5048A_CMD_READANGLE, &ANGR.Value)) { return false; }
        TempAngle += ANGR.bit_area.Data;
    }
    TempAngle /= 20;
    
    /* 置位编程使能位并发送 */
    PCR.bit_area.PE = 0x01;
    if(!AS5048a_WriteREG_Block(pDev, AS5048A_REG_PC, PCR.Value)) { return false; }
    PCR.Value = 0x00;
    
    /* 获取两个零位数据寄存器的值并写入设备 */
    ZPHR.bit_area.Data = (TempAngle >> 6) & 0xFF;   //零位的高八位数据
    ZPLR.bit_area.Data = (TempAngle) & 0x3F;        //零位的低六位数据
    if(!AS5048a_WriteREG_Block(pDev, AS5048A_REG_ZPH, ZPHR.Value)) { return false; }
    if(!AS5048a_WriteREG_Block(pDev, AS5048A_REG_ZPL, ZPLR.Value)) { return false; }
    
    /* 设置烧录位以启用内部编程过程。
       由于进入编程过程后返回的数据为错误数据，因此不使用返回结果检测的功能。
       以零位设置成功检测烧录通信是否成功。    */
    PCR.bit_area.Burn = 0x01;
    WriteCMD = AS5048a_Parity_EVEN(AS5048A_REG_PC & (~AS5048A_WRITE));
    if(!AS5048a_ReadWrite_Block(pDev, &WriteCMD, &TempValue)) { return false; }
    
    WriteCMD = AS5048a_Parity_EVEN(PCR.Value & (~AS5048A_WRITE));
    if(!AS5048a_ReadWrite_Block(pDev, &WriteCMD, &TempValue)) { return false; }
    PCR.Value = 0x00;
    
    /* 必要的延时以等待内部编程过程结束 */
    HAL_Delay(1);
    
    /* 读取角度看是否为零以校验零位置设置是否成功 */
    if(!AS5048a_ReadREG_Block(pDev, AS5048A_CMD_READANGLE, &ANGR.Value)) { return false; }
    
    // 数据可能有幅值为5的抖动
    if(13 < ((ANGR.bit_area.Data + 6) & 0x3FFF)) 
    { return false; }
    
    /* 设置校验位以再次将OTP数据加载到具有修改阈值比较器级别的内部寄存器中 
       由于进入“校验再编程过程”后返回的数据为错误数据，因此不使用返回结果检测的功能。
       以零位设置成功检测校验通信是否成功。    */
    PCR.bit_area.Verify = 0x01;
    WriteCMD = AS5048a_Parity_EVEN(AS5048A_REG_PC & (~AS5048A_WRITE));
    if(!AS5048a_ReadWrite_Block(pDev, &WriteCMD, &TempValue)) { return false; }
    
    WriteCMD = AS5048a_Parity_EVEN(PCR.Value & (~AS5048A_WRITE));
    if(!AS5048a_ReadWrite_Block(pDev, &WriteCMD, &TempValue)) { return false; }
    
    /* 必要的延时以等待内部编程过程结束 */
    HAL_Delay(1);
    
    /* 读取角度看是否为零以校验零位置设置是否成功 */
    if(!AS5048a_ReadREG_Block(pDev, AS5048A_CMD_READANGLE, &ANGR.Value)) { return false; }
    
    // 数据可能有幅值为5的抖动
    if(13 < ((ANGR.bit_area.Data + 6) & 0x3FFF)) 
    { return false; }
    
    return true;
}

/**
 * @brief	    以阻塞方式读取寄存器的值
 * @param[in]	pDev      - 指向被操作的设备地址
 * @param[in]	ReadCMD   - 16bit读指令,取值 @AS5048A_CMD_READ。
 * @param[out]	pData     - 接收数据缓冲区地址
 * @retval	    是否成功(bool)，成功为true.
 * @note        1. ReadCMD为已经加上校验位和功能位的16bit数据
 */
static bool AS5048a_ReadREG_Block(pAS5048a_t pDev, uint16_t ReadCMD, volatile uint16_t *pData)
{
    uint16_t TempCMD = ReadCMD;
    
    //发送读指令
    if(!AS5048a_ReadWrite_Block(pDev, &TempCMD, pData)) { return false; }
    
    //在下次通讯中获取读到的值
    if(!AS5048a_ReadWrite_Block(pDev, &TempCMD, pData)) { return false; }
    
    //检测读指令发送过程是否出错
    if(0 != ((*pData) & AS5048A_HOST_ERROR)) 
    {
        //指定最大重复清除的次数为10
        if(!AS5048a_ClearErrorFlags_Block(pDev, 10*pDev->FrameTime / 1000))
        { return false; }  //这样写是为了方便在调试时判定是否为偶发性读错误，可通过在此打断点以观察。
        
        return false; 
    }
    return true;
}

/**
 * @brief	    以阻塞方式向寄存器写值
 * @param[in]	pDev      - 指向被操作的设备地址
 * @param[in]	Address   - 寄存器的14bit地址
 * @param[in]	Data      - 希望写入寄存器的14bit数据内容
 * @retval	    是否成功(bool)，成功为true.
 * @note        1. 数据为14位。
 */
static bool AS5048a_WriteREG_Block(pAS5048a_t pDev, uint16_t Address, uint16_t Data)
{
    uint16_t WriteCMD = AS5048a_Parity_EVEN(Address & (~AS5048A_WRITE));
    uint16_t TempData;
        
    //发送写指令
    if(!AS5048a_ReadWrite_Block(pDev, &WriteCMD, &TempData)) { return false; }
    
    //计算并发送要写的内容
    WriteCMD = AS5048a_Parity_EVEN(Data & (~AS5048A_WRITE));
    if(!AS5048a_ReadWrite_Block(pDev, &WriteCMD, &TempData)) { return false; }
    
    //检测写指令发送过程是否出错(此时TempData的数据域中是旧的寄存器中的内容)
    if(0 != ((TempData) & AS5048A_HOST_ERROR)) 
    {
        //指定最大重复清除的次数为10
        if(!AS5048a_ClearErrorFlags_Block(pDev, 10*pDev->FrameTime / 1000))
        { return false; }  //这样写是为了方便在调试时判定是否为偶发性写错误，可通过在此打断点以观察。
        
        return false; 
    }
    
    //将写指令变为NOP，读取新内容以校验是否写入
    WriteCMD = AS5048A_CMD_READNOP;
    if(!AS5048a_ReadWrite_Block(pDev, &WriteCMD, &TempData)) { return false; }
    
    if(0 != ((TempData) & AS5048A_HOST_ERROR)) 
    {
        if(!AS5048a_ClearErrorFlags_Block(pDev, 10*pDev->FrameTime / 1000))
        { return false; }
        
        return false; 
    }
    
    //若返回的数据与待写入数据匹配，说明写入成功。
    if(Data != (TempData & AS5048A_DATAMASK)) { return false; }
    
    return true;
}

/**
 * @brief	    获取并清除设备错误寄存器中的错误标志
 * @param[in]	pDev      - 指向被操作的设备地址
 * @param[in]	Timeout   - 超时时间，单位us
 * @retval	    是否成功(bool)，成功为true.
 * @note        1. 超时时间应至少是发送一帧所需时间，低于该时间将按一帧时间对待。
 *              2. 此函数可产生错误标志: AS5048A_STATE_CMD_ERROR 及优先级更高的错误。
 */
static bool AS5048a_ClearErrorFlags_Block(pAS5048a_t pDev, uint32_t Timeout)
{
    //读EF寄存器
    uint16_t WriteCMD = AS5048A_CMD_READEF;
    uint16_t TempData;
    
    //记录读取EF寄存器次数的计数器
    uint32_t ReadCnt = 1;   
    //容许反复读取寄存器的次数，至少一次(1000是换算us和ns单位)
    uint32_t Tolerance = (uint32_t)Timeout * 1000 / pDev->FrameTime;
    Tolerance = (0 == Tolerance ? 1 : Tolerance);
    
    //发送读指令
    if(!AS5048a_ReadWrite_Block(pDev, &WriteCMD, &TempData)) { return false; }
    
    //读错误码并更新在数据结构中
    if(!AS5048a_ReadWrite_Block(pDev, &WriteCMD, &(pDev->ErrorFlags.Value))) { return false; }
    
lable_Relclear:
    //继续发送读指令以确认ErrorFlag被清零
    if(!AS5048a_ReadWrite_Block(pDev, &WriteCMD, &TempData)) { return false; }
    
    //错误标志在后三位
    if(0x00 != (TempData & 0x07))
    {
        ReadCnt++;
        
        if(ReadCnt >= Tolerance) 
        {
            AS5048A_SetErrorCode(pDev, AS5048A_STATE_CMD_ERROR);
            return false; 
        }
        
        //再次清零。
        goto lable_Relclear;
    }
    return true;
}

/**
 * @brief	    以非阻塞(单次DMA)方式传输数据
 * @param[in]	pDev      - 指向被操作的设备地址
 * @param[in]	ReadCMD   - 16bit读指令,取值参见头文件。
 * @param[out]	pData     - 接收数据缓冲区地址
 * @retval	    是否成功(bool)，成功为true.
 * @note        1. ReadCMD为已经加上校验位和功能位的16bit数据
 *              2. 底层操作的错误码将被存放在公共变量 @AS5048A_HAL_ErrorCode 中。
 *              3. 此函数可产生错误标志: AS5048A_STATE_SPI_ERROR 及优先级更高的错误。
 */
static bool AS5048a_ReadWrite_DMA(pAS5048a_t pDev, uint16_t *pTxBuffer, volatile uint16_t *pRxBuffer)
{    
    //延时1us(t_CSnH)
    Tiny_Delay(2);
    AS5048a_NSS_RESET(pDev);
    
    //延时1us(t_L)
    Tiny_Delay(2);
    AS5048A_HAL_ErrorCode = HAL_SPI_TransmitReceive_DMA(pDev->hspi, (uint8_t*)pTxBuffer, (uint8_t*)pRxBuffer, 1);
    
    if(HAL_OK != AS5048A_HAL_ErrorCode)
    {
        AS5048A_SetErrorCode(pDev, AS5048A_STATE_SPI_ERROR);
        return false;
    }
    return true;
}

/**
 * @brief	    以非阻塞(单次DMA)方式传输完成回调函数
 * @param[in]	pDev      - 指向被操作的设备地址
 * @retval	    None.
 * @note        1. 执行用于维护硬件协议层的相关操作。
 *              2. 将在本组件的上层回调函数接口中被调用。
 */
static void AS5048a_ReadWrite_DMACallBack(pAS5048a_t pDev)
{
    //延时1us(t_H)
    Tiny_Delay(2);
    AS5048a_NSS_SET(pDev);
    
    return;
}

/**
 * @brief	    以阻塞方式向设备传输数据
 * @param[in]	pDev      - 指向被操作的设备地址
 * @param[in]	pTxBuffer - 指向发送数据缓冲区地址
 * @param[in]	pRxBuffer - 指向接收数据缓冲区地址
 * @retval	    是否成功(bool)，成功为true.
 * @note        1. 延时时间参照手册并依赖实测(使通信稳定的最短时间)。
 *              2. 偶校验位在位序列的第15位
 *              3. 底层操作的错误码将被存在公共变量 @AS5048A_HAL_ErrorCode中。
 *              4. 此函数可产生错误标志: AS5048A_STATE_SPI_ERROR 及优先级更高的错误。
 */
static bool AS5048a_ReadWrite_Block(pAS5048a_t pDev, uint16_t *pTxBuffer, volatile uint16_t *pRxBuffer)
{
    //延时1us(t_CSnH)
    Tiny_Delay(2);
    AS5048a_NSS_RESET(pDev);
    
    //延时1us(t_L)
    Tiny_Delay(2);
    
    // 发送读寄存器指令(通信频率是us级的，超时时间是按ms读取的，因此2ms没应答，就算超时了)
    AS5048A_HAL_ErrorCode = HAL_SPI_TransmitReceive(pDev->hspi, (uint8_t*)pTxBuffer, (uint8_t*)pRxBuffer, 1, 2);
    
    if(HAL_OK != AS5048A_HAL_ErrorCode)
    {
        AS5048A_SetErrorCode(pDev, AS5048A_STATE_SPI_ERROR);
        return false;
    }
    
    //延时1us(t_H)
    Tiny_Delay(2);
    AS5048a_NSS_SET(pDev);
    
    return true;
}

/**
 * @brief	    设置设备的错误码
 * @param[in]	pDevice - 指向被操作的设备地址
 * @param[in]	Error   - 错误码，取值 @AS5048a_State_t
 * @retval	    None.
 */
static inline void AS5048A_SetErrorCode(pAS5048a_t pDev, AS5048a_State_t Error)
{
    if(NULL == pDev) { return; }
    
    if(IS_AS5048A_ERROR(pDev->State))
    {
        pDev->State = (pDev->State < Error) ? pDev->State : Error;
    }
    else
    {
        pDev->State = Error;
    }
    
    return;
}

/**
 * @brief	    置高片选(NSS)脚电平
 * @param[in]	pDevice - 指向被操作的设备地址
 * @retval	    None.
 */
static inline void AS5048a_NSS_SET(pAS5048a_t pDevice) 
{
#if defined(USING_BITBAND)  
    pDevice->NSS_Write = GPIO_PIN_SET; 
#else 
    HAL_GPIO_WritePin(pDevice->NSSPort, pDevice->NSSPin, GPIO_PIN_SET); 
#endif  
}

/**
 * @brief	    置低片选(NSS)脚电平
 * @param[in]	pDevice - 指向被操作的设备地址
 * @retval	    None.
 */
static inline void AS5048a_NSS_RESET(pAS5048a_t pDevice)
{
#if defined(USING_BITBAND)  
    pDevice->NSS_Write = GPIO_PIN_RESET; 
#else 
    HAL_GPIO_WritePin(pDevice->NSSPort, pDevice->NSSPin, GPIO_PIN_RESET); 
#endif  
}

/**
 * @brief	    将原数据转换为带偶校验位的数据
 * @param[in]	Num - 被校验的数据
 * @retval	    添上校验位的数据(uint16_t)
 * @note        1. 偶校验位存在的意义在于使得整个位序列中“1”的个数为偶数。
 *              2. 偶校验位在位序列的第15位
 */
static inline uint16_t AS5048a_Parity_EVEN(uint16_t Num)
{
    //取低15位
    uint16_t Temp = Num & (~(1 << 15));
    
    Num = Temp;
    
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
    return (Num | (Temp & 1) << 15);
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
 * @note	    1. 仅内部调用
 *			    2. HAL库中引脚位置是位表达, 非数字编号（数字编号方便阅读） @ref GPIO_pins_define
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

//////////////////////////////////////////////////////////////////////////////////////// test
#ifdef AS5048A_DEBUG

bool isWorkNormaly = true;
uint16_t TestTxBuffer = AS5048A_CMD_READANGLE;
uint16_t TempValue[5];
uint16_t TempData[5];

//Test发送接收
void AS5048a_Test(void)
{
    //诊断工作条件
    isWorkNormaly = AS5048a_DiagnoseConditions(&AS5048a);
    
    AS5048a_ReadREG_Block(&AS5048a, TestTxBuffer, TempValue);
    //AS5048a_ReadWrite_Block(&AS5048a, &TestTxBuffer, TempValue);
    
    //AS5048a_ReadWrite_DMA(&AS5048a, &TestTxBuffer, TempValue);
        
    TempData[0] = TempValue[0] & AS5048A_DATAMASK;
        
    /////////////////////////////////////////////////////
    
    AS5048a_ReadREG_Block(&AS5048a, TestTxBuffer, TempValue + 1);
    //AS5048a_ReadWrite_Block(&AS5048a, &TestTxBuffer, TempValue + 1);
    
    //while(AS5048A_STATE_BUSY == AS5048a.State);
    //AS5048a_ReadWrite_DMA(&AS5048a, &TestTxBuffer, TempValue + 1);
    
    TempData[1] = TempValue[1] & AS5048A_DATAMASK;
        
    /////////////////////////////////////////////////////
    
    AS5048a_ReadREG_Block(&AS5048a, TestTxBuffer, TempValue + 2);
    //AS5048a_ReadWrite_Block(&AS5048a, &TestTxBuffer, TempValue + 2);
    
    //while(AS5048A_STATE_BUSY == AS5048a.State);
    //AS5048a_ReadWrite_DMA(&AS5048a, &TestTxBuffer, TempValue + 2);
        
    TempData[2] = TempValue[2] & AS5048A_DATAMASK;
        
    ////////////////////////////////////////////////////////
    
    AS5048a_ReadREG_Block(&AS5048a, TestTxBuffer, TempValue + 3);
    //AS5048a_ReadWrite_Block(&AS5048a, &TestTxBuffer, TempValue + 3);
    
    //while(AS5048A_STATE_BUSY == AS5048a.State);
    //AS5048a_ReadWrite_DMA(&AS5048a, &TestTxBuffer, TempValue + 3);
        
    TempData[3] = TempValue[3] & AS5048A_DATAMASK;
        
    //////////////////////////////////////////////////////////
    
    AS5048a_ReadREG_Block(&AS5048a, TestTxBuffer, TempValue + 4);
    //AS5048a_ReadWrite_Block(&AS5048a, &TestTxBuffer, TempValue + 4);
    
    //while(AS5048A_STATE_BUSY == AS5048a.State);
    //AS5048a_ReadWrite_DMA(&AS5048a, &TestTxBuffer, TempValue + 4);
        
    TempData[4] = TempValue[4] & AS5048A_DATAMASK;
    
    //////////////////////////////////////////////////////////
    
    while(AS5048A_STATE_BUSY == AS5048a.State);
    AS5048a_UpdateAngleSpeed_DMA(&AS5048a);
            
    return;
}

#endif /* AS5048A_DEBUG */

/************************ (C) COPYRIGHT HITwh Excellent Robot Organization(HERO). *****END OF FILE****/
