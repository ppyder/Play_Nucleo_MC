#ifndef AD_MONITOR_H
#define AD_MONITOR_H

/* �ڴ˰�������оƬ��ͷ�ļ� */
#include "stm32f3xx.h"
/* USER CODE END Includes */

typedef struct 
{
    float Adj_Resistance;
    float Temperature;    
}ADC_Values_t, *pADC_Values_t;

void AD_MonitorInit(void);
void RegularDataDeal(void);

#endif
