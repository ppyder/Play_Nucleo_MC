#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include <rtdevice.h>

#pragma anon_unions

#define RDT_TIMEOUT	100

#define TEST_COMMUNICATION 0

typedef enum
{
	COM_RDT_NOMAL,	//正常
	COM_RDT_WAIT,	//等待应答
	COM_RDT_ACK,	//肯定应答
	COM_RDT_NAK,	//否定应答

}COM_RDT_STATUS;

typedef enum
{	
	RECEIVE_HEAD,
	RECEIVE_DATA,
	RECEIVE_TAIL,	
	RECEIVE_FINISHED
}StateEnum;

typedef enum
{
	COLSE_DEV,
	OPEN_WITH_INT_MODE,
	OPEN_WITH_DMA_MODE,

}COM_RECEIVE_MODE ;

typedef enum
{
	COM_MODE1,	//模式1：数据头 + 数据 + [校验和] + 数据尾
	COM_MODE2,	//模式2：数据头 + 数据长度 + 数据 + [校验和]
	COM_CUSTOM  //自定义模式
	
}COM_MODE;

typedef enum 
{
	UART_PROTOCOL,
	CAN_PROTOCOL,
	IIC_PROTOCOL,
	SPI_PROTOCOL
	
}COM_PROTOCOL;

typedef struct
{
	uint8_t state;	//接收状态
	uint8_t head_count; //已接收数据头个数
	uint8_t tail_count; //已接收数据尾个数
	uint8_t data_size;	//数据长度
}ReceiveState;

typedef struct
{
	uint8_t* head;
	uint8_t* tail;
	
	uint8_t head_size;
	uint8_t tail_size;
	uint8_t data_length;
	
}DataInfo;

typedef struct 
{
	uint8_t* buffer;
	uint16_t buffer_size;
	uint16_t rx_size;
}ReceiveBuffer;

typedef struct
{
	unsigned int can_id;
	unsigned int filter_id;	
}CanConfig;

typedef union DevConfig
{
	CanConfig *can_config;
}DevConfig;
	
typedef struct
{	
	//通信设备句柄
	void* com_dev;	
	//数据头句柄
	DataInfo* data_info;	
	
	//发送处理函数句柄
	uint32_t (*Send_Handle)(uint8_t*);	
	//接收处理函数句柄
	bool (*Receive_Handle)(uint8_t* data, uint32_t size);	
	
	//发送缓冲区
	uint8_t* tx_buffer;
	//接收缓冲区（双缓冲）
	ReceiveBuffer rx_buffer[2];
	
	//当前接收缓冲区
	ReceiveBuffer* current_rx_buffer;
	
	//信号量
	rt_sem_t rx_sem;
	
	//发送缓冲区大小
	uint16_t tx_buffer_size;			
	//通信模式
	uint8_t com_mode;	
	//通信协议类型标识
	uint8_t com_protocol;	
	
	//是否开启数据校验
	bool is_verify;
	
	//是否使用可靠数据传输
	bool is_rdt;
	
	//可靠数据传输应答
	uint8_t rdt_ack;
	
	//接收状态
	ReceiveState recv_state;	
	
	DevConfig;
	
}CommnicationType;

typedef CommnicationType* CommnicationTypePtr;	//CommnicationType的指针类型


/***********************************外部函数声明***********************************************/
extern void Init_Commnication(CommnicationType** module_ptr,  COM_MODE mode, COM_PROTOCOL protocol, 
															void* com_dev_ptr, uint16_t rx_size, uint16_t tx_size, rt_sem_t sem, 
															uint32_t (*Send_Handle_Ptr)(uint8_t*),
															bool (*Receive_Handle_Ptr)(uint8_t* data, uint32_t size));

extern void Destory_Communication(CommnicationType* module_ptr);
extern void Enable_Communication(CommnicationType* module_ptr, uint8_t mode);
extern bool Enable_Receive(CommnicationType* module_ptr, bool enable);
extern bool Set_Data_Info(CommnicationType* module_ptr, DataInfo* data_info);
extern bool Send_Data(CommnicationType* module_ptr, uint8_t* send_data, uint32_t data_size);
extern bool Send_Data_RDT(CommnicationType* module_ptr, uint8_t* send_data, uint32_t data_size);
extern uint8_t Get_Receive_Data(CommnicationType* module_ptr, uint8_t *buf, uint8_t size);
extern CommnicationType* Get_UartInstance(const rt_device_t dev);
extern CommnicationType* Get_CanInstance(const rt_device_t dev, unsigned int filter_id);
extern uint8_t Get_Verify_Sum(uint8_t* data, uint32_t data_size);
extern void Exchange_Buffer(CommnicationType* module_ptr);
extern void Reset_State(ReceiveState* state);
extern int GetRxBufferSize(const ReceiveBuffer* buffer, const ReceiveState *state);

extern void Init_Data_Info(DataInfo** data_info_ptr, uint8_t *head, uint8_t head_size, 
													uint8_t *tail, uint8_t tail_size, uint8_t data_length);

/*
 * brief: 设置CAN设备ID
 * param: module_ptr 通信组件
 * param: can_id 发送ID
 * param: filter_id 接收过滤ID
 * return: 无
*/
extern inline void Set_Can_ID(CommnicationType* module_ptr, unsigned int can_id, unsigned int filter_id)
{	
	if(module_ptr->com_protocol == CAN_PROTOCOL)
	{
		module_ptr->can_config->can_id = can_id;
		module_ptr->can_config->filter_id = filter_id;
	}
}

/*
 * brief: 使能数据校验
 * param: module_ptr 通信组件
 * param: enable 是否使能
 * return: 无
*/
extern inline void Enable_Data_Verify(CommnicationType* module_ptr, bool enable)
{
	module_ptr->is_verify = enable;
	module_ptr->data_info->data_length++;
}

/*
 * brief: 使能可靠数据传输
 * param: module_ptr 通信组件
 * param: enable 是否使能
 * return: 无
*/
extern inline void Enable_RDT(CommnicationType* module_ptr, bool enable)
{
	module_ptr->is_rdt = enable;
	module_ptr->rdt_ack = COM_RDT_NOMAL;
}

/*
 * brief: 获取接收数据大小
 * param: module_ptr 通信组件
 * return: 接收到的数据大小
*/
extern inline uint16_t Get_Receive_Data_Size(CommnicationType* module_ptr)
{
	if(module_ptr->current_rx_buffer == &module_ptr->rx_buffer[0])
	{
		return module_ptr->rx_buffer[1].rx_size;
	}
	else
	{
		return module_ptr->rx_buffer[0].rx_size;
	}	
}

#if TEST_COMMUNICATION
extern bool UART_Example(void);
extern bool Can_Example(void);
extern bool UART_Example_RDT(void);
#endif

#endif
