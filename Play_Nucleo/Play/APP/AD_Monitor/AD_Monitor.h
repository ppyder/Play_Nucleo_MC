#ifndef AD_MONITOR_H
#define AD_MONITOR_H

#include <stdbool.h>

/* 在此包含所用芯片的头文件 */
#include "stm32f3xx.h"
/* USER CODE END Includes */

typedef struct 
{
    float Adj_Resistance;
    float Temperature;    
}ADC_Values_t, *pADC_Values_t;

extern bool isRegularUpdated;

void AD_MonitorInit(void);
void RegularDataDeal(void);

#endif
