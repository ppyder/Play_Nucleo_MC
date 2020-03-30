#include "UserLED.h"
#include "tim.h"

//��ʼ��LED
void UserLED_Init(void)
{
    //������ɫLED����PWM��ʽ��ʼ��
    LED_Tunable_Init(&UserLEDs[Board_LED], &htim2, TIM_CHANNEL_1, 50);
    
    //����LED����������
    LED_Task_Start(&UserLEDs[Board_LED], LED_TASK_BREATHE, 3000, 0); ////LED_TASK_TWINKLE
    
    //�������ɫLED����IO��ʽ��ʼ��
    LED_ON_OFF_Init(&UserLEDs[Dev_LED], GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    
    //����LED��˸������
    LED_Task_Start(&UserLEDs[Dev_LED], LED_TASK_TWINKLE, 350, 3); //LED_TASK_BREATHE//
}

