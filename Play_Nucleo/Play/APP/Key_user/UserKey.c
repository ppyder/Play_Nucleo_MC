#include "UserKey.h"    //本文件的头
#include "UserLED.h"    //LED控制所需文件

//启用此宏，使用扫描轮询方式读取按键状态
#define USING_SCAN

void Key1DealFunc(KeyEvents_t Event);

//用户按键初始化函数
void UserKeyInit(void)
{
    //初始化按键，设定滤波窗口为5*5ms。
#ifdef USING_SCAN
    Key_Scan_Init( &UserKeys[Key1], Key1DealFunc, KEY_TYPE_SWITCH, //KEY_TYPE_INCHING//
                   5, 5, GPIOC, GPIO_PIN_13, GPIO_PIN_RESET );
#else
    Key_INT_Init( &UserKeys[Key1], Key1DealFunc, KEY_TYPE_SWITCH, //KEY_TYPE_INCHING//
                  EXTI_LINE_13, GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
#endif
}

#include "AS5048a.h"

//按键处理函数
void Key1DealFunc(KeyEvents_t Event)
{
    if(KEY_EVENT_DOWN == Event)
    {
        LED_TurnON(&UserLEDs[Dev_LED]);////&UserLEDs[Board_LED]
    }
    if(KEY_EVENT_UP == Event)
    {
        LED_TurnOFF(&UserLEDs[Dev_LED]);////&UserLEDs[Board_LED]
    }
    if(KEY_EVENT_PRESS == Event)
    {
        AS5048a_Test();
        LED_ON_OFF_Toggle(&UserLEDs[Dev_LED]);////&UserLEDs[Board_LED]
    }
}
