#ifndef _UPPERCOMPUTER_H_
#define _UPPERCOMPUTER_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include "communication.h"

typedef enum
{
    UPPER_BOOL_TYPE,
    UPPER_INTEGER_TYPE,
    UPPER_FLOAT_TYPE,
}UpperParaType;

typedef enum 
{
    UPPER_PARAMETER_MODE = 0x01,
    UPPER_MAP_MODE = 0x02,
    UPPER_OSCILLOMETER_MODE = 0x04,   
}UpperMode;

typedef struct UpperParameter
{
    uint8_t id;
    uint8_t type;
    void* para;
}UpperPara;

typedef struct UpperParameterAdjust
{
    uint8_t auto_id;
    uint8_t size;
    UpperPara **head;
}UpperParaAdjust;

typedef struct UpperMapMode
{
    uint8_t size;
    struct rt_semaphore *map_sem;
}UpperMap;

extern CommnicationTypePtr Upper_Init(uint8_t mode, char* uart_name);
extern void Upper_Destroy(void);
extern void Upper_Enable_Para_Mode(uint8_t paras_number);
extern void Upper_Add_One_Para(UpperParaType type, void* para);
extern void Upper_Add_Paras(UpperParaType type, void* para, int size);
extern void Upper_Enable_Map_Mode(struct rt_semaphore *sem, int point_number);
extern void Upper_Send_Path(uint16_t *x, uint16_t *y, int size);

extern void Test_Para_Mode(void);
extern void Test_Map_Mode(void);
#endif
