#include "UserKey.h"    //���ļ���ͷ
#include "UserLED.h"    //LED���������ļ�

//���ô˺꣬ʹ��ɨ����ѯ��ʽ��ȡ����״̬
#define USING_SCAN

void Key1DealFunc(KeyEvents_t Event);

//�û�������ʼ������
void UserKeyInit(void)
{
    //��ʼ���������趨�˲�����Ϊ5*5ms��
#ifdef USING_SCAN
    Key_Scan_Init( &UserKeys[Key1], Key1DealFunc, KEY_TYPE_SWITCH, //KEY_TYPE_INCHING//
                   5, 5, GPIOC, GPIO_PIN_13, GPIO_PIN_RESET );
#else
    Key_INT_Init( &UserKeys[Key1], Key1DealFunc, KEY_TYPE_SWITCH, //KEY_TYPE_INCHING//
                  EXTI_LINE_13, GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
#endif
}

#include "AS5048a.h"

//����������
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
