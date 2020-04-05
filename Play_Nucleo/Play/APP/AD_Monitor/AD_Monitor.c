#include "AD_Monitor.h"
#include "adc.h"

//ԭʼ���ݽṹ�嶨��
typedef struct 
{
    uint16_t Adj_Resistance;        // ��ѹ�ٷֱȣ���λ%
    uint16_t Internal_Temperature;  // оƬ�¶ȣ���λ��
    uint16_t External_Temperature;  // �ⲿ�¶�
    uint16_t Bus_Voltage;           // ĸ�ߵ�ѹ
}ADC_RawValues_t, *pADC_RawValues_t;

#define ADC_14BIT_MAX_VALUE 4096    // 14λADC�������ֵ�������ѹ���׼��ѹ���ʱ�Ĳ���ֵ��
#define ADC_V_REF           3.3f    // ��׼��ѹ

static inline float DealTemperatureValue(uint16_t Value);
static inline float DealResistanceValue(uint16_t Value);

bool isRegularUpdated = false;

//DMA��������ԭʼ����
ADC_RawValues_t ADC_RawValues;

//�������������ADC����
ADC_Values_t ADC_Values;

//��ʼ��ADC
void AD_MonitorInit(void)
{
    //ADCУ׼ǰ����Ҫ���ϵ�1-2��ADC���ڣ�������ʱ��1ms�ٲ�����
    HAL_Delay(1);
    
    //�ϵ��Զ�У׼һ��
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    
    //����ADCת������ť���ڲ��¶ȣ�
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_RawValues, sizeof(ADC_RawValues));
}

//����ͨ�����ݴ�����(�����Ե��ü���)
void RegularDataDeal(void)
{
    if(isRegularUpdated)
    {
        //������������
        ADC_Values.Adj_Resistance = DealResistanceValue(ADC_RawValues.Adj_Resistance);
        ADC_Values.Temperature = DealTemperatureValue(ADC_RawValues.Internal_Temperature);
        
        isRegularUpdated = false;
    }
    
}

//�����¶�����
static inline float DealTemperatureValue(uint16_t Value)
{
    float V_25 = 1.43f;         // 25��ʱ��V_TS��ѹ��������ֵ(V)���������ֲ����á�
    float Avg_Slope = 4.3e-03f; // ������������������ߵ�ƽ��б�ʵ���ֵ(V/��)���������ֲ����á�
    
    //��ñ���ʵ��ֵ
    float V_TS = ADC_V_REF * Value / ADC_14BIT_MAX_VALUE;
    
    //��ʽ��Դ�ڲο��ֲ�
    return ((V_25 - V_TS)/Avg_Slope + 25.0f);
}

//����������������,ӳ�䵽0-100��Χ��ֵ
static inline float DealResistanceValue(uint16_t Value)
{
    return (100.0f * Value / ADC_14BIT_MAX_VALUE);
}

