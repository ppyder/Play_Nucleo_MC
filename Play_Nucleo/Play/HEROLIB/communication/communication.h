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
	COM_RDT_NOMAL,	//����
	COM_RDT_WAIT,	//�ȴ�Ӧ��
	COM_RDT_ACK,	//�϶�Ӧ��
	COM_RDT_NAK,	//��Ӧ��

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
	COM_MODE1,	//ģʽ1������ͷ + ���� + [У���] + ����β
	COM_MODE2,	//ģʽ2������ͷ + ���ݳ��� + ���� + [У���]
	COM_CUSTOM  //�Զ���ģʽ
	
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
	uint8_t state;	//����״̬
	uint8_t head_count; //�ѽ�������ͷ����
	uint8_t tail_count; //�ѽ�������β����
	uint8_t data_size;	//���ݳ���
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
	//ͨ���豸���
	void* com_dev;	
	//����ͷ���
	DataInfo* data_info;	
	
	//���ʹ��������
	uint32_t (*Send_Handle)(uint8_t*);	
	//���մ��������
	bool (*Receive_Handle)(uint8_t* data, uint32_t size);	
	
	//���ͻ�����
	uint8_t* tx_buffer;
	//���ջ�������˫���壩
	ReceiveBuffer rx_buffer[2];
	
	//��ǰ���ջ�����
	ReceiveBuffer* current_rx_buffer;
	
	//�ź���
	rt_sem_t rx_sem;
	
	//���ͻ�������С
	uint16_t tx_buffer_size;			
	//ͨ��ģʽ
	uint8_t com_mode;	
	//ͨ��Э�����ͱ�ʶ
	uint8_t com_protocol;	
	
	//�Ƿ�������У��
	bool is_verify;
	
	//�Ƿ�ʹ�ÿɿ����ݴ���
	bool is_rdt;
	
	//�ɿ����ݴ���Ӧ��
	uint8_t rdt_ack;
	
	//����״̬
	ReceiveState recv_state;	
	
	DevConfig;
	
}CommnicationType;

typedef CommnicationType* CommnicationTypePtr;	//CommnicationType��ָ������


/***********************************�ⲿ��������***********************************************/
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
 * brief: ����CAN�豸ID
 * param: module_ptr ͨ�����
 * param: can_id ����ID
 * param: filter_id ���չ���ID
 * return: ��
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
 * brief: ʹ������У��
 * param: module_ptr ͨ�����
 * param: enable �Ƿ�ʹ��
 * return: ��
*/
extern inline void Enable_Data_Verify(CommnicationType* module_ptr, bool enable)
{
	module_ptr->is_verify = enable;
	module_ptr->data_info->data_length++;
}

/*
 * brief: ʹ�ܿɿ����ݴ���
 * param: module_ptr ͨ�����
 * param: enable �Ƿ�ʹ��
 * return: ��
*/
extern inline void Enable_RDT(CommnicationType* module_ptr, bool enable)
{
	module_ptr->is_rdt = enable;
	module_ptr->rdt_ack = COM_RDT_NOMAL;
}

/*
 * brief: ��ȡ�������ݴ�С
 * param: module_ptr ͨ�����
 * return: ���յ������ݴ�С
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
