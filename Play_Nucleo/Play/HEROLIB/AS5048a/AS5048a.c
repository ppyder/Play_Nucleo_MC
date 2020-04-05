/**
 ******************************************************************************
 * @file    AS5048a.c
 * @brief   �м�㣺�ű�����AS5048a������.
 * @version 1.0 ʾ���汾
 * @author  ���ෲ
 * @contact 17863107058(�ֻ�)   942041771(qq)
 * @date    2020/04/05
 *
 * @note    1. Ŀǰ��֧���뵥��AS5048a��ͨ�š�
 *          2. �Ƕ���Ϣ��ʱ������ȡ���������ݾ���������ʽ���ݶ�ȡ��
 *          3. Ϊ���ڳ������Ⱥ�ʱ���Լ����Ĵ���ȷ�����ݵ�ʱ�䣬��ʱ���������ݶ�ȡ������DMA�жϷ�ʽ��
 *          4. ��AS5048��˵�漰��ʼ����ͨ�ŵĲ������ǻ����Ҳ�������ġ�
 *          5. ����ṩms��˲ʱ�ٶȺ�ʵʱλ����Ϣ�����ṩ��Ӧ�Ŀɿ���������־��
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
#define AS5048A_DATAMAX          (16384)                                // 14λ�������ֵ(0x3FFF + 1)
#define AS5048A_DATA2ANGLE(DATA) ((DATA) * (360.0 / AS5048A_DATAMAX))   // ��ԭʼ����ת��Ϊ�Ƕ�

//����ʹ�õĵ�λ�ǡ���/s������ʾʹ�õ�ת�ٵ�λ��RPM��
//�������±��ʣ���ת����λ��
#define SPEED2RPM_UNIT_RATIO (0.1666667f) //Ratio = 60/360 = 1/6;
#define RPM2SPEED_UNIT_RATIO (6.0f)

//�ٶȻ���ƽ��ֵ�˲������ݻ�������С
#define SPEED_FILTER_SIZE 20

/* Public variables --------------------------------------------------------*/
AS5048a_t AS5048a =  // ���ʵ��
{
    .hspi = NULL,
    .NSSPort = NULL,
    .State = AS5048A_STATE_RESET,
    .LastCMD = AS5048A_CMD_READNOP,
};
HAL_StatusTypeDef AS5048A_HAL_ErrorCode = HAL_OK;// ͨ�Ŵ����룬������

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
float Speed_Buffer[SPEED_FILTER_SIZE] = {0}; // ѭ������:����ƽ��ֵ�˲����ݻ�����


/**
 * @brief	    ��ʼ���豸AS5048a
 * @param[out]	pDev          - ָ�򱻳�ʼ��������ṹ���ַ
 * @param[in]	MaxRPM        - ���ܵ����ת�٣����ڹ�������λ�����ж�ת����Ϣ�ɿ��ԡ�
 * @param[in]	NSSPort       - NSS���ŵ�GPIO�˿ڣ�ȡֵ GPIOx ��x = A,B,C...��
 * @param[in]	NSS_Pin       - NSS���ŵ����źţ�ȡֵ GPIO_PIN_x (where x can be (0..15))
 * @param[in]	hspi          - �����õ�Ƭ����������
 * @param[in]	FrameTime     - SPI����һ֡������������ʱ��(��������ʱ��)������ʵ�⣬��λns��
 * @param[in]	MaxErrorTimes - ����ͨ���У�����[�������ֵĴ������]�����ֵ��
 * @retval	    ��ʼ���Ƿ�ɹ�(bool)���ɹ�Ϊtrue.
 * @note        1. SPI����Ӧ����Ϊ����������NSS, 16bit����, MSB First, <10Mbit/s, CPOL_Low, CPHA_2Edge��
 *              2. �˺����ɲ��������־: AS5048A_STATE_CMD_ERROR �����ȼ����ߵĴ���
 */
bool AS5048a_Init( pAS5048a_t pDev, float MaxRPM, 
                   GPIO_TypeDef* NSSPort, uint16_t NSS_Pin, 
                   SPI_HandleTypeDef* hspi, uint16_t FrameTime, uint16_t MaxErrorTimes )
{
    //�Ƿ����
    if( NULL == pDev  ||  NULL == NSSPort  ||     NULL == hspi      || 0 == FrameTime || 0 == MaxErrorTimes ||
        !IS_GPIO_ALL_INSTANCE(NSSPort)     || !IS_GPIO_PIN(NSS_Pin) || HAL_SPI_STATE_READY != hspi->State)
    { return false; }
        
    //Ӳ���ײ��ʼ��
    pDev->hspi = hspi;
    pDev->NSSPort = NSSPort;
    pDev->NSSPin = NSS_Pin;
    pDev->NSSPinNum = PinBit2PinNum(NSS_Pin);
#ifdef USING_BITBAND    
    //��ʼ��λ��
    pLED->NSS_Write = &BIT_ADDR((uint32_t)NSSPort + ODR_Addr_Offset, pDev->NSSPinNum);
#endif

    //��ʼ����־
    pDev->isDebuging      = false;  // ���ڷ�Debug״̬
    pDev->isSpeedReliable = true;   // �ٶ����ݿɿ�
    pDev->isDataInvalid   = false;  // ��ʼ֡���ݺϷ�
    
    //��ʼ����������
    pDev->BaseAngle  = 0;
    pDev->RealAngle  = 0;
    pDev->Speed      = 0;
    pDev->MaxRPM     = MaxRPM;
    pDev->ErrorFlags.Value = 0x00;
    
    pDev->FrameTime      = FrameTime;
    pDev->LastCMD        = AS5048A_CMD_READNOP; // ����ϴζ�ȡ���ǿ�����
    pDev->PeriodErrorCnt = 0;
    pDev->ErrorMax       = MaxErrorTimes;
    pDev->TempRxBuffer   = 0x00;
    
    pDev->ReadMoment.CarryNum = 0;
    pDev->ReadMoment.Ticks    = 0;
    
    //��ʼ���ٶȻ���ƽ��ֵ�˲���
    SlidingAveFilterInit(&pDev->SpeedFilter, Speed_Buffer, SPEED_FILTER_SIZE, 0);
    
    //����Ƭѡ��
    AS5048a_NSS_SET(pDev);
    
    //��Ϲ�������
    if(!AS5048a_DiagnoseConditions(pDev)) { return false; }
    
    //������λ
    if(!AS5048a_ResetZeroPos(pDev))
    {
        AS5048A_SetErrorCode(pDev, AS5048A_STATE_ZP_ERROR);
        return false;
    }
    
    //��ǳ�ʼ�����
    pDev->State = AS5048A_STATE_READY;
    return true;
}

/**
 * @brief	    �������ȡʵʱλ����Ϣ
 * @param[in]	pDev      - ָ�򱻲������豸��ַ
 * @param[out]	pAngle    - ָ��洢�Ƕ���Ϣ�ı�����ַ
 * @retval	    ��ȡ�Ƿ�ɹ�(bool)���ɹ�Ϊtrue.
 * @note        1. ����״̬�º�δ��ʼ��״̬�²�����λ����Ϣ��
 *              2. ���ٶ����ݿɿ�ʱ�������ػ�׼λ�����ȡʱ���϶��Ӧ������λ�õĺ���Ϊ����ʵʱλ�á�
 *              3. ���ٶ����ݲ��ɿ�ʱ�������ػ�׼λ����Ϊ���Ƶ�ʵʱλ�á�
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
 * @brief	    �������ȡʵʱ�ٶ���Ϣ
 * @param[in]	pDev      - ָ�򱻲������豸��ַ
 * @param[out]	pSpeed    - ָ��洢�ٶ���Ϣ�ı�����ַ
 * @retval	    ��ȡ�Ƿ�ɹ�(bool)���ɹ�Ϊtrue.
 * @note        1. ����״̬�º�δ��ʼ��״̬�²������ٶ���Ϣ��
 *              2. ���ٶ����ݲ��ɿ�ʱ���������ٶȡ�
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
 * @brief	    ��Ϲ��������Ƿ����Ҫ��
 * @param[in]	pDev      - ָ�򱻲������豸��ַ
 * @retval	    ���������Ƿ����Ҫ��(bool)������Ϊtrue.
 * @note        1. �����ͨ���������ų�״����CORDIC�㷨�������Լ�ƫ�Ʋ����㷨��������
 *              2. �˺����ɲ��������־: AS5048A_STATE_DIG_ERROR �����ȼ����ߵĴ���
 *              3. �˺��������������־: AS5048A_STATE_DIG_ERROR ����ֻ�����������ȼ����ڻ���ڴ˴��������¡�
 */
bool AS5048a_DiagnoseConditions(pAS5048a_t pDev)
{
    bool Result = true;
    
    if(NULL == pDev || (IS_AS5048A_ERROR(pDev->State) && (pDev->State < AS5048A_STATE_DIG_ERROR)))
    { return false; }
    
    //��λ���Ա�־����ʱ���������Թ�����Ϊ
    pDev->isDebuging = true;
    
    //�ȴ����ܴ��ڵ��ϸ�ͨѶ���̽���
    while(AS5048A_STATE_BUSY == AS5048a.State);
    
    //����ͨ�ż��ʹ����־���£������ֵ�ʱ��(10ms)��������־
    if(!AS5048a_ClearErrorFlags_Block(pDev, 10000)) 
    { return false; }

    //��ȡ״̬����
lable_ReReadAGC:
    if(!AS5048a_ReadREG_Block(pDev, AS5048A_CMD_READAGC, &pDev->AGCR.Value)) 
    { 
        if(!IS_AS5048A_ERROR(pDev->State))
        {
            //���û�н������״̬��˵��������ż���ԵĶ����󣬿�����ִ�ж�������
            goto lable_ReReadAGC;
        }
        return false; 
    }

lable_ReReadMAG:
    if(!AS5048a_ReadREG_Block(pDev, AS5048A_CMD_READMAG, &pDev->MAGR.Value)) 
    { 
        if(!IS_AS5048A_ERROR(pDev->State))
        {
            //���û�н������״̬��˵��������ż���ԵĶ����󣬿�����ִ�ж�������
            goto lable_ReReadMAG;
        }
        return false; 
    }
        
    //�ų���ǿ��������CORDIC�㷨�������ʾ����״̬�쳣
    Result = !(pDev->AGCR.bit_area.Comp_H || pDev->AGCR.bit_area.Comp_L || pDev->AGCR.bit_area.COF);
    
    //�Զ����油��δ���Ҳ��ʾ����״̬�쳣
    Result = Result && (pDev->AGCR.bit_area.OCF);
    
    //���״̬
    if(!Result)
    {
        AS5048A_SetErrorCode(pDev, AS5048A_STATE_DIG_ERROR);
    }
    else
    {
        //��ʼ����������ǰ��Ӧ����ǰ��ǳ�ʼ����ɡ�
        if(AS5048A_STATE_RESET != pDev->State)
        {
            pDev->State = AS5048A_STATE_READY;
        }
    }
    
    //��λ���Ա�־�Իָ������Թ�����Ϊ
    pDev->isDebuging = false;
    
    return Result;
}

/**
 * @brief	    ����������(DMA)��ʽ��ȡ�Ƕ�
 * @param[in]	pDev      - ָ�򱻲������豸��ַ
 * @retval	    None.
 * @note        1. �����Ӳ����ͨ��Э���ά����
 *              2. ����ɴ�����������ܲ��������־: AS5048A_STATE_DIG_ERROR �����ȼ����ߵĴ���
 *              3. ���鶨ʱ���á��ڴ���һ֡��������30%ԣ����ʱ������ϣ�Ƶ��Խ���ٶ�����Խ���ʡ�
 */
void AS5048a_UpdateAngleSpeed_DMA(pAS5048a_t pDev)
{
    uint16_t TempCMD = AS5048A_CMD_READANGLE;
    
    //���Լ�ģʽ�µĴ���״̬�ſ���
    if(pDev->isDebuging || AS5048A_STATE_READY != AS5048a.State) { return; }
    
    //����ϴ�ͨ�����Ƿ��д���
    if(pDev->isDataInvalid)
    {
        pDev->PeriodErrorCnt++;
        if(pDev->PeriodErrorCnt >= pDev->ErrorMax)
        {
            pDev->State = AS5048A_STATE_CMD_ERROR;
            return;
        }
        //��ԭ�ȵĶ�ָ���Ϊ�������ָ��
        TempCMD = AS5048A_CMD_READEF;
    }
    else if(pDev->PeriodErrorCnt > 0)   
    {
        //�״λع�����״̬ʱ���������
        pDev->PeriodErrorCnt = 0;
    }
    
    //����豸ռ��
    pDev->State = AS5048A_STATE_BUSY;
    
    //�������������Ĵ�������
    if(!AS5048a_ReadWrite_DMA(pDev, &TempCMD, &pDev->TempRxBuffer)) { return; }
        
    //��¼���η��͵�ָ��
    pDev->LastCMD = TempCMD;
    
    return;
}

/**
 * @brief	    AS5048a��DMA�����жϻص�
 * @param[in]	pDev      - ָ�򱻲������豸��ַ
 * @retval	    None.
 * @note        1. �����Ӳ����ͨ��Э���ά����
 *              2. ���Խ��յ������ݽ���У�飬������ͨ�Ŵ����־ @isDataInvalid ��
 *              3. ���������ݴ�������
 */
void AS5048a_DMARx_Callback(pAS5048a_t pDev)
{
    //�ͷ�ռ�ñ�־��ά���ײ����
    pDev->State = AS5048A_STATE_READY;
    AS5048a_ReadWrite_DMACallBack(pDev);
     
    /* �Զ�ȡ�������ݽ��д���
      �����־��һ��˵�����ϴ����ݴ����д�����û�н��յ�������������Чָ�
      �������ν��յ������ݲ����š� */
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
 * @brief	    ����λ�ú��ٶ�
 * @param[in]	pDev      - ָ�򱻲������豸��ַ
 * @retval	    None.
 * @note        1. ����AS5048��DMA���ջص��У�ȷ�ϽǶ����ݿɿ�����á�
 */
static void UpdatePos_Speed(pAS5048a_t pDev)
{
    static int32_t InvalidCnt = 0;    // ���ݵ�[�������ɿ�����]������
    bool   isThisInvalid      = false;// ���ݿɿ���־
    
    float   DeltaTime_s = 0;   // ���ټ��ʱ��
    float   DeltaAngle  = 0;   // ���ټ��λ��
    float   PositiveAngle = 0; // �Ƕȱ仯������ֵ
    float   LastAngle   = 0;   // �ϴν�λ��
    float   MaxDelta    = 0;   // ���ת���µĽ�λ������
    GlobalTime_t LastMoment;   // �ϴζ�ȡλ�õ�ʱ��

    //�����ϴζ�ȡλ��ʱ����Ϣ
    LastAngle = pDev->BaseAngle;
    LastMoment = pDev->ReadMoment;
    
    //���½Ƕ��ƻ�׼�Ƕ��Լ���ȡʱ��
    pDev->BaseAngle = AS5048A_DATA2ANGLE(pDev->TempRxBuffer & AS5048A_DATAMASK);
    TimeRef_GetPreciseTime(&pDev->ReadMoment);
    
    //��ȡʱ����
    DeltaTime_s = TimeRef_TimeMinus(&pDev->ReadMoment, &LastMoment);
        
    //������ܵ�����λ������(��ֵ��Ϊ��)
    MaxDelta = DeltaTime_s * pDev->MaxRPM * RPM2SPEED_UNIT_RATIO;
    
    //��ȡ��λ������
    DeltaAngle = pDev->BaseAngle - LastAngle;
    PositiveAngle = fabs(DeltaAngle);
    
    /* ���ת�����������ڰ�Ȧ�򱾴νǶ������������ת���µĽǶ�����ʱ������ǰ����������ٶȲ��ɿ���
     *  ���ǲ��ɿ��Ļ�����Ҫ����ȡλ�õ�Ƶ���Ƿ������ܹ��ɿ����ٵĵ��߱�׼��
     *  �ɿ����ٵĵ��߱�׼�ǣ����ת������������һ��Ȧ�ڶ�ȡ����λ�á�
     * �˷����ɼ����򴫸������⵼�µ����ݴ��������Ĵ�������Ӧ����Ӧ�ò�������������*/
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
        if(fabs(DeltaAngle) > 180.0f) // ��������Ȧ������
        {
            DeltaAngle += (DeltaAngle < 0 ? 360.0f : -360.0f);
        }
        
        //�ɦĦȺͦ�t������ٶ�
        pDev->RawSpeed = DeltaAngle / DeltaTime_s * SPEED2RPM_UNIT_RATIO;
        
        if(!pDev->isSpeedReliable) //��ԭ���ٶ����ݲ��ɿ�����˵���кܳ�ʱ���ٶȶ���û�и����ˣ�
                                   //����ƽ��ֵ�˲��ή�Ͳ���ֵ����ʵ��ֵ���ٶȣ����Ҫ�������ݻ�������
        {
            pDev->isSpeedReliable = true;
            SetSlidingAveBuffer(&pDev->SpeedFilter, pDev->RawSpeed);
            
            pDev->Speed = pDev->RawSpeed;
        }
        else
        {
            //����ƽ��ֵ�˲�
            pDev->Speed = SlidingAveFilter(&pDev->SpeedFilter, pDev->RawSpeed);
        }
        InvalidCnt = 0;
    }
    return;
}

/**
 * @brief	    �趨��ǰλ��Ϊ��λ
 * @param[in]	pDev      - ָ�򱻲������豸��ַ
 * @retval	    �Ƿ�ɹ�(bool)���ɹ�Ϊtrue.
 * @note        1. �豸ÿ���ϵ�ֻ���趨һ����λ��������OTP������
 *              2. �����豸�ĳ�ʼ�������б����á�
 *              3. ����ͬһ���ϵ��ж�ε��øú�����Ҳ������豸������������Ӱ�죬�����Ƿ���false��
 */
static bool AS5048a_ResetZeroPos(pAS5048a_t pDev)
{
    uint32_t TempAngle = 0; // ��λ����ǰ�Ķ�ȡֵ
    uint16_t WriteCMD  = 0; // ��ʱ��дָ��ͻ�����
    uint16_t TempValue = 0; // ��ʱ�����ݽ��ջ�����
    
    AS5048a_ANGR_t ANGR = {0};
    AS5048a_ZPHR_t ZPHR = {0};
    AS5048a_ZPLR_t ZPLR = {0};
    AS5048a_PCR_t  PCR  = {0};
        
    /* ��20�νǶȼĴ�����ƽ��ֵ�˲� */
    for(int i = 0; i < 20; i++)
    {
        if(!AS5048a_ReadREG_Block(pDev, AS5048A_CMD_READANGLE, &ANGR.Value)) { return false; }
        TempAngle += ANGR.bit_area.Data;
    }
    TempAngle /= 20;
    
    /* ��λ���ʹ��λ������ */
    PCR.bit_area.PE = 0x01;
    if(!AS5048a_WriteREG_Block(pDev, AS5048A_REG_PC, PCR.Value)) { return false; }
    PCR.Value = 0x00;
    
    /* ��ȡ������λ���ݼĴ�����ֵ��д���豸 */
    ZPHR.bit_area.Data = (TempAngle >> 6) & 0xFF;   //��λ�ĸ߰�λ����
    ZPLR.bit_area.Data = (TempAngle) & 0x3F;        //��λ�ĵ���λ����
    if(!AS5048a_WriteREG_Block(pDev, AS5048A_REG_ZPH, ZPHR.Value)) { return false; }
    if(!AS5048a_WriteREG_Block(pDev, AS5048A_REG_ZPL, ZPLR.Value)) { return false; }
    
    /* ������¼λ�������ڲ���̹��̡�
       ���ڽ����̹��̺󷵻ص�����Ϊ�������ݣ���˲�ʹ�÷��ؽ�����Ĺ��ܡ�
       ����λ���óɹ������¼ͨ���Ƿ�ɹ���    */
    PCR.bit_area.Burn = 0x01;
    WriteCMD = AS5048a_Parity_EVEN(AS5048A_REG_PC & (~AS5048A_WRITE));
    if(!AS5048a_ReadWrite_Block(pDev, &WriteCMD, &TempValue)) { return false; }
    
    WriteCMD = AS5048a_Parity_EVEN(PCR.Value & (~AS5048A_WRITE));
    if(!AS5048a_ReadWrite_Block(pDev, &WriteCMD, &TempValue)) { return false; }
    PCR.Value = 0x00;
    
    /* ��Ҫ����ʱ�Եȴ��ڲ���̹��̽��� */
    HAL_Delay(1);
    
    /* ��ȡ�Ƕȿ��Ƿ�Ϊ����У����λ�������Ƿ�ɹ� */
    if(!AS5048a_ReadREG_Block(pDev, AS5048A_CMD_READANGLE, &ANGR.Value)) { return false; }
    
    // ���ݿ����з�ֵΪ5�Ķ���
    if(13 < ((ANGR.bit_area.Data + 6) & 0x3FFF)) 
    { return false; }
    
    /* ����У��λ���ٴν�OTP���ݼ��ص������޸���ֵ�Ƚ���������ڲ��Ĵ����� 
       ���ڽ��롰У���ٱ�̹��̡��󷵻ص�����Ϊ�������ݣ���˲�ʹ�÷��ؽ�����Ĺ��ܡ�
       ����λ���óɹ����У��ͨ���Ƿ�ɹ���    */
    PCR.bit_area.Verify = 0x01;
    WriteCMD = AS5048a_Parity_EVEN(AS5048A_REG_PC & (~AS5048A_WRITE));
    if(!AS5048a_ReadWrite_Block(pDev, &WriteCMD, &TempValue)) { return false; }
    
    WriteCMD = AS5048a_Parity_EVEN(PCR.Value & (~AS5048A_WRITE));
    if(!AS5048a_ReadWrite_Block(pDev, &WriteCMD, &TempValue)) { return false; }
    
    /* ��Ҫ����ʱ�Եȴ��ڲ���̹��̽��� */
    HAL_Delay(1);
    
    /* ��ȡ�Ƕȿ��Ƿ�Ϊ����У����λ�������Ƿ�ɹ� */
    if(!AS5048a_ReadREG_Block(pDev, AS5048A_CMD_READANGLE, &ANGR.Value)) { return false; }
    
    // ���ݿ����з�ֵΪ5�Ķ���
    if(13 < ((ANGR.bit_area.Data + 6) & 0x3FFF)) 
    { return false; }
    
    return true;
}

/**
 * @brief	    ��������ʽ��ȡ�Ĵ�����ֵ
 * @param[in]	pDev      - ָ�򱻲������豸��ַ
 * @param[in]	ReadCMD   - 16bit��ָ��,ȡֵ @AS5048A_CMD_READ��
 * @param[out]	pData     - �������ݻ�������ַ
 * @retval	    �Ƿ�ɹ�(bool)���ɹ�Ϊtrue.
 * @note        1. ReadCMDΪ�Ѿ�����У��λ�͹���λ��16bit����
 */
static bool AS5048a_ReadREG_Block(pAS5048a_t pDev, uint16_t ReadCMD, volatile uint16_t *pData)
{
    uint16_t TempCMD = ReadCMD;
    
    //���Ͷ�ָ��
    if(!AS5048a_ReadWrite_Block(pDev, &TempCMD, pData)) { return false; }
    
    //���´�ͨѶ�л�ȡ������ֵ
    if(!AS5048a_ReadWrite_Block(pDev, &TempCMD, pData)) { return false; }
    
    //����ָ��͹����Ƿ����
    if(0 != ((*pData) & AS5048A_HOST_ERROR)) 
    {
        //ָ������ظ�����Ĵ���Ϊ10
        if(!AS5048a_ClearErrorFlags_Block(pDev, 10*pDev->FrameTime / 1000))
        { return false; }  //����д��Ϊ�˷����ڵ���ʱ�ж��Ƿ�Ϊż���Զ����󣬿�ͨ���ڴ˴�ϵ��Թ۲졣
        
        return false; 
    }
    return true;
}

/**
 * @brief	    ��������ʽ��Ĵ���дֵ
 * @param[in]	pDev      - ָ�򱻲������豸��ַ
 * @param[in]	Address   - �Ĵ�����14bit��ַ
 * @param[in]	Data      - ϣ��д��Ĵ�����14bit��������
 * @retval	    �Ƿ�ɹ�(bool)���ɹ�Ϊtrue.
 * @note        1. ����Ϊ14λ��
 */
static bool AS5048a_WriteREG_Block(pAS5048a_t pDev, uint16_t Address, uint16_t Data)
{
    uint16_t WriteCMD = AS5048a_Parity_EVEN(Address & (~AS5048A_WRITE));
    uint16_t TempData;
        
    //����дָ��
    if(!AS5048a_ReadWrite_Block(pDev, &WriteCMD, &TempData)) { return false; }
    
    //���㲢����Ҫд������
    WriteCMD = AS5048a_Parity_EVEN(Data & (~AS5048A_WRITE));
    if(!AS5048a_ReadWrite_Block(pDev, &WriteCMD, &TempData)) { return false; }
    
    //���дָ��͹����Ƿ����(��ʱTempData�����������ǾɵļĴ����е�����)
    if(0 != ((TempData) & AS5048A_HOST_ERROR)) 
    {
        //ָ������ظ�����Ĵ���Ϊ10
        if(!AS5048a_ClearErrorFlags_Block(pDev, 10*pDev->FrameTime / 1000))
        { return false; }  //����д��Ϊ�˷����ڵ���ʱ�ж��Ƿ�Ϊż����д���󣬿�ͨ���ڴ˴�ϵ��Թ۲졣
        
        return false; 
    }
    
    //��дָ���ΪNOP����ȡ��������У���Ƿ�д��
    WriteCMD = AS5048A_CMD_READNOP;
    if(!AS5048a_ReadWrite_Block(pDev, &WriteCMD, &TempData)) { return false; }
    
    if(0 != ((TempData) & AS5048A_HOST_ERROR)) 
    {
        if(!AS5048a_ClearErrorFlags_Block(pDev, 10*pDev->FrameTime / 1000))
        { return false; }
        
        return false; 
    }
    
    //�����ص��������д������ƥ�䣬˵��д��ɹ���
    if(Data != (TempData & AS5048A_DATAMASK)) { return false; }
    
    return true;
}

/**
 * @brief	    ��ȡ������豸����Ĵ����еĴ����־
 * @param[in]	pDev      - ָ�򱻲������豸��ַ
 * @param[in]	Timeout   - ��ʱʱ�䣬��λus
 * @retval	    �Ƿ�ɹ�(bool)���ɹ�Ϊtrue.
 * @note        1. ��ʱʱ��Ӧ�����Ƿ���һ֡����ʱ�䣬���ڸ�ʱ�佫��һ֡ʱ��Դ���
 *              2. �˺����ɲ��������־: AS5048A_STATE_CMD_ERROR �����ȼ����ߵĴ���
 */
static bool AS5048a_ClearErrorFlags_Block(pAS5048a_t pDev, uint32_t Timeout)
{
    //��EF�Ĵ���
    uint16_t WriteCMD = AS5048A_CMD_READEF;
    uint16_t TempData;
    
    //��¼��ȡEF�Ĵ��������ļ�����
    uint32_t ReadCnt = 1;   
    //��������ȡ�Ĵ����Ĵ���������һ��(1000�ǻ���us��ns��λ)
    uint32_t Tolerance = (uint32_t)Timeout * 1000 / pDev->FrameTime;
    Tolerance = (0 == Tolerance ? 1 : Tolerance);
    
    //���Ͷ�ָ��
    if(!AS5048a_ReadWrite_Block(pDev, &WriteCMD, &TempData)) { return false; }
    
    //�������벢���������ݽṹ��
    if(!AS5048a_ReadWrite_Block(pDev, &WriteCMD, &(pDev->ErrorFlags.Value))) { return false; }
    
lable_Relclear:
    //�������Ͷ�ָ����ȷ��ErrorFlag������
    if(!AS5048a_ReadWrite_Block(pDev, &WriteCMD, &TempData)) { return false; }
    
    //�����־�ں���λ
    if(0x00 != (TempData & 0x07))
    {
        ReadCnt++;
        
        if(ReadCnt >= Tolerance) 
        {
            AS5048A_SetErrorCode(pDev, AS5048A_STATE_CMD_ERROR);
            return false; 
        }
        
        //�ٴ����㡣
        goto lable_Relclear;
    }
    return true;
}

/**
 * @brief	    �Է�����(����DMA)��ʽ��������
 * @param[in]	pDev      - ָ�򱻲������豸��ַ
 * @param[in]	ReadCMD   - 16bit��ָ��,ȡֵ�μ�ͷ�ļ���
 * @param[out]	pData     - �������ݻ�������ַ
 * @retval	    �Ƿ�ɹ�(bool)���ɹ�Ϊtrue.
 * @note        1. ReadCMDΪ�Ѿ�����У��λ�͹���λ��16bit����
 *              2. �ײ�����Ĵ����뽫������ڹ������� @AS5048A_HAL_ErrorCode �С�
 *              3. �˺����ɲ��������־: AS5048A_STATE_SPI_ERROR �����ȼ����ߵĴ���
 */
static bool AS5048a_ReadWrite_DMA(pAS5048a_t pDev, uint16_t *pTxBuffer, volatile uint16_t *pRxBuffer)
{    
    //��ʱ1us(t_CSnH)
    Tiny_Delay(2);
    AS5048a_NSS_RESET(pDev);
    
    //��ʱ1us(t_L)
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
 * @brief	    �Է�����(����DMA)��ʽ������ɻص�����
 * @param[in]	pDev      - ָ�򱻲������豸��ַ
 * @retval	    None.
 * @note        1. ִ������ά��Ӳ��Э������ز�����
 *              2. ���ڱ�������ϲ�ص������ӿ��б����á�
 */
static void AS5048a_ReadWrite_DMACallBack(pAS5048a_t pDev)
{
    //��ʱ1us(t_H)
    Tiny_Delay(2);
    AS5048a_NSS_SET(pDev);
    
    return;
}

/**
 * @brief	    ��������ʽ���豸��������
 * @param[in]	pDev      - ָ�򱻲������豸��ַ
 * @param[in]	pTxBuffer - ָ�������ݻ�������ַ
 * @param[in]	pRxBuffer - ָ��������ݻ�������ַ
 * @retval	    �Ƿ�ɹ�(bool)���ɹ�Ϊtrue.
 * @note        1. ��ʱʱ������ֲᲢ����ʵ��(ʹͨ���ȶ������ʱ��)��
 *              2. żУ��λ��λ���еĵ�15λ
 *              3. �ײ�����Ĵ����뽫�����ڹ������� @AS5048A_HAL_ErrorCode�С�
 *              4. �˺����ɲ��������־: AS5048A_STATE_SPI_ERROR �����ȼ����ߵĴ���
 */
static bool AS5048a_ReadWrite_Block(pAS5048a_t pDev, uint16_t *pTxBuffer, volatile uint16_t *pRxBuffer)
{
    //��ʱ1us(t_CSnH)
    Tiny_Delay(2);
    AS5048a_NSS_RESET(pDev);
    
    //��ʱ1us(t_L)
    Tiny_Delay(2);
    
    // ���Ͷ��Ĵ���ָ��(ͨ��Ƶ����us���ģ���ʱʱ���ǰ�ms��ȡ�ģ����2msûӦ�𣬾��㳬ʱ��)
    AS5048A_HAL_ErrorCode = HAL_SPI_TransmitReceive(pDev->hspi, (uint8_t*)pTxBuffer, (uint8_t*)pRxBuffer, 1, 2);
    
    if(HAL_OK != AS5048A_HAL_ErrorCode)
    {
        AS5048A_SetErrorCode(pDev, AS5048A_STATE_SPI_ERROR);
        return false;
    }
    
    //��ʱ1us(t_H)
    Tiny_Delay(2);
    AS5048a_NSS_SET(pDev);
    
    return true;
}

/**
 * @brief	    �����豸�Ĵ�����
 * @param[in]	pDevice - ָ�򱻲������豸��ַ
 * @param[in]	Error   - �����룬ȡֵ @AS5048a_State_t
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
 * @brief	    �ø�Ƭѡ(NSS)�ŵ�ƽ
 * @param[in]	pDevice - ָ�򱻲������豸��ַ
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
 * @brief	    �õ�Ƭѡ(NSS)�ŵ�ƽ
 * @param[in]	pDevice - ָ�򱻲������豸��ַ
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
 * @brief	    ��ԭ����ת��Ϊ��żУ��λ������
 * @param[in]	Num - ��У�������
 * @retval	    ����У��λ������(uint16_t)
 * @note        1. żУ��λ���ڵ���������ʹ������λ�����С�1���ĸ���Ϊż����
 *              2. żУ��λ��λ���еĵ�15λ
 */
static inline uint16_t AS5048a_Parity_EVEN(uint16_t Num)
{
    //ȡ��15λ
    uint16_t Temp = Num & (~(1 << 15));
    
    Num = Temp;
    
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
    return (Num | (Temp & 1) << 15);
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
 * @note	    1. ���ڲ�����
 *			    2. HAL��������λ����λ���, �����ֱ�ţ����ֱ�ŷ����Ķ��� @ref GPIO_pins_define
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

//////////////////////////////////////////////////////////////////////////////////////// test
#ifdef AS5048A_DEBUG

bool isWorkNormaly = true;
uint16_t TestTxBuffer = AS5048A_CMD_READANGLE;
uint16_t TempValue[5];
uint16_t TempData[5];

//Test���ͽ���
void AS5048a_Test(void)
{
    //��Ϲ�������
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
