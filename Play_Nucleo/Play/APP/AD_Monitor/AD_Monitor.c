#include "AD_Monitor.h"
#include "adc.h"

//原始数据结构体定义
typedef struct 
{
    uint16_t Adj_Resistance;        // 电压百分比，单位%
    uint16_t Temperature;           // 芯片温度，单位℃
}ADC_RawValues_t, *pADC_RawValues_t;

#define ADC_14BIT_MAX_VALUE 4096    // 14位ADC测量最大值（输入电压与基准电压相等时的采样值）
#define ADC_V_REF           3.3f    // 基准电压

static inline float DealTemperatureValue(uint16_t Value);
static inline float DealResistanceValue(uint16_t Value);

//DMA搬运来的原始数据
ADC_RawValues_t ADC_RawValues;

//经过初级处理的ADC数据
ADC_Values_t ADC_Values;

//初始化ADC
void AD_MonitorInit(void)
{
    //ADC校准前至少要等上电1-2个ADC周期，不妨延时上1ms再操作。
    HAL_Delay(1);
    
    //上电自动校准一次
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    
    //开启ADC转换（旋钮和内部温度）
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_RawValues, sizeof(ADC_RawValues));
}

//规则通道数据处理函数
void RegularDataDeal(void)
{
    ADC_Values.Adj_Resistance = DealResistanceValue(ADC_RawValues.Adj_Resistance);
    ADC_Values.Temperature = DealTemperatureValue(ADC_RawValues.Temperature);
}

//处理温度数据
static inline float DealTemperatureValue(uint16_t Value)
{
    float V_25 = 1.43f;         // 25℃时的V_TS电压测量典型值(V)，查数据手册所得。
    float Avg_Slope = 4.3e-03f; // 线性温敏电阻测温曲线的平均斜率典型值(V/℃)，查数据手册所得。
    
    //获得本次实测值
    float V_TS = ADC_V_REF * Value / ADC_14BIT_MAX_VALUE;
    
    //公式来源于参考手册
    return ((V_25 - V_TS)/Avg_Slope + 25.0f);
}

//处理滑动变阻器数据,映射到0-100范围的值
static inline float DealResistanceValue(uint16_t Value)
{
    return (100.0f * Value / ADC_14BIT_MAX_VALUE);
}

