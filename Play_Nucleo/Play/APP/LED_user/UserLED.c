#include "UserLED.h"
#include "tim.h"

//初始化LED
void UserLED_Init(void)
{
    //板载绿色LED，按PWM方式初始化
    LED_Tunable_Init(&UserLEDs[Board_LED], &htim2, TIM_CHANNEL_1, 50);
    
    //开启LED呼吸灯任务
    LED_Task_Start(&UserLEDs[Board_LED], LED_TASK_BREATHE, 3000, 0); ////LED_TASK_TWINKLE
    
    //驱动板红色LED，按IO方式初始化
    LED_ON_OFF_Init(&UserLEDs[Dev_LED], GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
}

