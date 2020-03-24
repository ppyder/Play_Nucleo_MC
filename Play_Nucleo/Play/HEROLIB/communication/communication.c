#include "communication.h"
#include "string.h"
#include "com_can.h"
#include "com_uart.h"

#define MAP_SIZE 10
/***********************************��������***********************************************/
//��̬ȫ�ֱ�����˽�б����������ڲ�ʹ��
static uint8_t g_map_count = 0;
static CommnicationType* g_commnication_map[MAP_SIZE] = {NULL};


/***********************************��������***********************************************/
//��̬������˽�к����������ڲ�����
static void Not_Define(void);
static void Init_Receive_Buffer(ReceiveBuffer* recv_buffer, uint16_t size);
static uint32_t Get_Data_Packet_Size(CommnicationType* module_ptr, uint32_t data_size);
static void Package_Data(CommnicationType* module_ptr, uint8_t* send_data,
													uint32_t data_size, uint32_t packet_size);


/***********************************ͨ�����***********************************************/
/*
 * brief: ��ʼ��ͨ�����
 * param: module_ptr ͨ�������ע����Ҫ����Ĳ�����ͨ�����ָ��ĵ�ַ
 * param: mode ͨ��ģʽ
 * param: protocol ͨ��Э�飬UART_PROTOCOL ���� CAN_PROTOCOL
 * param: com_dev_ptr ͨ���豸ָ�룬uart�豸ָ�����can�豸ָ��
 * param: rx_size ���ջ�������С
 * param: tx_size ���ͻ�������С
 * param: sem �����̵߳��ź���
 * param: send_handle_ptr �Զ��巢�ʹ�����ָ�룬����ΪNULL
 * param: receive_handle_ptr �Զ�����մ�����ָ�룬����ΪNULL
 * return: ��
*/
void Init_Commnication(CommnicationType** module_ptr,  COM_MODE mode, COM_PROTOCOL protocol, 
											void* com_dev_ptr, uint16_t rx_size, uint16_t tx_size, rt_sem_t sem, 
											uint32_t (*send_handle_ptr)(uint8_t*),
											bool (*receive_handle_ptr)(uint8_t* data, uint32_t size))
{
	//Ϊͨ����������ڴ�
	*module_ptr = (CommnicationType*)rt_malloc(sizeof(CommnicationType));
	CommnicationTypePtr com_module = *module_ptr;
	
	//��ʼ��ͨ��ģʽ
	com_module->com_mode = mode;
	//��ʼ��ͨ��Э��
	com_module->com_protocol = protocol;
	if(protocol == CAN_PROTOCOL)
	{		
		com_module->can_config = (CanConfig*)rt_malloc(sizeof(CanConfig));
	}

	//��ʼ��ͨ���豸
	com_module->com_dev = com_dev_ptr;
	
	//��ʼ�����ջ�����
	Init_Receive_Buffer(&com_module->rx_buffer[0], rx_size);
	Init_Receive_Buffer(&com_module->rx_buffer[1], rx_size);	
	com_module->current_rx_buffer = &com_module->rx_buffer[0];
	
	//��ʼ�����ͻ�����
	com_module->tx_buffer = (uint8_t*)rt_malloc(sizeof(uint8_t) * tx_size);
	com_module->tx_buffer_size = tx_size;	
	
	//��ʼ���ź���
	com_module->rx_sem = sem;
	
	//��ʼ�����ʹ�����
	com_module->Send_Handle = send_handle_ptr;
	//��ʼ�����մ�����
	com_module->Receive_Handle = receive_handle_ptr;
	
	//��ʼ������״̬
	Reset_State(&com_module->recv_state);
	
	//��ʼ������ͷ
	com_module->data_info = NULL;

	com_module->is_verify = false;
	com_module->is_rdt = false;
	g_commnication_map[g_map_count++] = com_module;
}

/*
 * brief: ����ͨ�����
 * param: module_ptr ͨ�����ָ��
 * return: ��
*/
void Destory_Communication(CommnicationType* module_ptr)
{
	//�ر�ͨ���豸
	Enable_Communication(module_ptr, false);
	
	//�ͷ��ڴ�
	rt_free(module_ptr->rx_buffer[0].buffer);
	rt_free(module_ptr->rx_buffer[1].buffer);
	rt_free(module_ptr->tx_buffer);
	
	if(module_ptr->data_info != NULL)
	{
		rt_free(module_ptr->data_info);
	}

	if(module_ptr->com_protocol == CAN_PROTOCOL)
	{
		rt_free(module_ptr->can_config);
	}

	rt_free(module_ptr);
}

/*
 * brief: ʹ��ͨ�����
 * param: module_ptr ͨ�����ָ��
 * param: enable �Ƿ�ʹ�ܣ�true ���� false
 * return: ��
*/
void Enable_Communication(CommnicationType* module_ptr, uint8_t mode)
{
	if(mode == COLSE_DEV)
	{
		if(module_ptr->com_protocol == UART_PROTOCOL)
		{			
			Close_Uart(module_ptr->com_dev);
		}
		else if(module_ptr->com_protocol == CAN_PROTOCOL)
		{
			Close_Can(module_ptr->com_dev);
		}
	}
	else
	{
		if(module_ptr->com_protocol == UART_PROTOCOL)
		{
			if(mode == OPEN_WITH_INT_MODE)
			{
				Open_Uart(module_ptr->com_dev, RT_DEVICE_FLAG_INT_RX);
			}
			else if(mode == OPEN_WITH_DMA_MODE)
			{
				Open_Uart(module_ptr->com_dev, RT_DEVICE_FLAG_DMA_RX);
			}			
		}
		else if(module_ptr->com_protocol == CAN_PROTOCOL)
		{			
			//���Can�豸�Ƿ��
			if(!Is_Can_Open(module_ptr->com_dev))
			{
				assert(0);	//can�豸û�д�
			}
		}
	}	
}

/*
 * brief: ʹ�ܽ���
 * param: module_ptr ͨ�����ָ��
 * param: enable �Ƿ�ʹ�ܣ�true ���� false
 * return: ����ʹ�ܽ����Ƿ�ɹ���trueΪ�ɹ���
*/
bool Enable_Receive(CommnicationType* module_ptr, bool enable)
{
	rt_err_t ret = RT_EOK;
	
	if(module_ptr->com_protocol == UART_PROTOCOL)
	{
		switch(module_ptr->com_mode)
		{
			case COM_MODE1:
				ret = rt_device_set_rx_indicate(module_ptr->com_dev, Uart_Recv_Mode1);
				break;
			case COM_MODE2:
				ret = rt_device_set_rx_indicate(module_ptr->com_dev, Uart_Recv_Mode2);	
				break;
			case COM_CUSTOM:
				ret = rt_device_set_rx_indicate(module_ptr->com_dev, Uart_Recv_Custom_Mode);	
				break;
			default:
				break;
		}
	}
	else if(module_ptr->com_protocol == CAN_PROTOCOL)
	{
		switch(module_ptr->com_mode)
		{
			case COM_MODE1:
				ret = rt_device_set_rx_indicate(module_ptr->com_dev, Can_Recv_Mode1);
				break;
			case COM_MODE2:				
				break;
			case COM_CUSTOM:
				ret = rt_device_set_rx_indicate(module_ptr->com_dev, Can_Recv_Custom_Mode);	
				break;
			default:
				break;
		}
	}
	
	return ((ret == RT_EOK)? true:false);
}

/*
 * brief: ����������Ϣ����������ͷ������β�����ݳ���
 * param: module_ptr ͨ�����ָ��
 * param: data_info_ptr DataInfoָ��
 * return: �����Ƿ�ɹ���trueΪ�ɹ���
*/
bool Set_Data_Info(CommnicationType* module_ptr, DataInfo* data_info_ptr)
{
	if(data_info_ptr != NULL)
	{
		module_ptr->data_info = data_info_ptr;
		return true;
	}
	
	return false;
}

/*
 * brief: �����ѯ��������
 * param: module_ptr ͨ�����ָ��
 * param: buffer �������ݻ�����
 * param: buffer_size �������ݴ�С
 * return: ʵ�ʽ������ݸ���
*/
uint32_t Receive_Data(CommnicationType* module_ptr, uint8_t* buffer, uint32_t buffer_size)
{	
	if(module_ptr->com_protocol == UART_PROTOCOL)
	{
		return Uart_Receive(module_ptr, buffer, buffer_size);
	}
	else if(module_ptr->com_protocol == CAN_PROTOCOL)
	{
		return Can_Receive(module_ptr, buffer, buffer_size);
	}
}

/*
 * brief: �Կɿ����ݴ���ķ�ʽ��������
 * param: module_ptr ͨ�����ָ��
 * param: send_data �����͵�����
 * param: data_size ���������ݴ�С
 * return: ���ط����Ƿ�ɹ���trueΪ�ɹ���
*/
bool Send_Data_RDT(CommnicationType* module_ptr, uint8_t* send_data, uint32_t data_size)
{
	if(module_ptr->rdt_ack != COM_RDT_NOMAL)
	{
		return false;	//���ڵȴ�Ӧ�𣬲��ܷ�����һ�δ���
	}
	
	int timer = 0;
	uint8_t ack = 0;
	uint8_t buffer[1] = {0xff};
	Send_Data(module_ptr, send_data, data_size);
	
	while(1)
	{
		rt_thread_mdelay(10);		
		
		if(timer > RDT_TIMEOUT)	//��ʱ�ش�
		{
			Send_Data(module_ptr, send_data, data_size);
			timer = 0;
		}
		else
		{
			timer += 10;
		}
		
		if(Receive_Data(module_ptr, buffer, 1) == 1)
		{
			if(buffer[0] == 0x00)	//ACKӦ��
			{
				module_ptr->rdt_ack = COM_RDT_NOMAL;
				break;
			}
			else	//NAKӦ��
			{
				Send_Data(module_ptr, send_data, data_size);
			}
		}		
	}
}

/*
 * brief: ��������
 * param: module_ptr ͨ�����ָ��
 * param: send_data �����͵�����
 * param: data_size ���������ݴ�С
 * return: ���ط����Ƿ�ɹ���trueΪ�ɹ���
*/
bool Send_Data(CommnicationType* module_ptr, uint8_t* send_data, uint32_t data_size)
{
	uint32_t packet_size = 0;
	
	if(module_ptr->com_mode == COM_CUSTOM)
	{
		//�����Զ��巢�ʹ�����
		if(module_ptr->Send_Handle != NULL)
		{
			packet_size = module_ptr->Send_Handle(module_ptr->tx_buffer);
		}
	}
	else if(module_ptr->com_mode == COM_MODE1 | module_ptr->com_mode == COM_MODE2)
	{
		if(NULL == module_ptr->data_info)
		{
			return false;
		}
		packet_size = Get_Data_Packet_Size(module_ptr, data_size);
		Package_Data(module_ptr, send_data, data_size, packet_size);
	}
	
	switch(module_ptr->com_protocol)
	{
		case UART_PROTOCOL:
			Uart_Send(module_ptr->com_dev, module_ptr->tx_buffer, packet_size);
			break;
		case CAN_PROTOCOL:
			Can_Send(module_ptr->com_dev, module_ptr->can_config->can_id, module_ptr->tx_buffer, packet_size);
			break;
		case IIC_PROTOCOL:
			Not_Define();
			break;
		case SPI_PROTOCOL:
			Not_Define();
			break;
		default:
			break;
	}
		
	return true;
}

/*
 * brief: ��ȡ���յ�������
 * param: module_ptr ͨ�����ָ��
 * param: buf �������ݵĻ�����
 * param: size �������ݵĻ�������С
 * return: ����ʵ�ʽ��յ����ݴ�С
*/
extern uint8_t Get_Receive_Data(CommnicationType* module_ptr, uint8_t *buf, uint8_t size)
{
	int act_size = size;
	if(module_ptr->current_rx_buffer == &module_ptr->rx_buffer[0])
	{
		if(act_size > module_ptr->rx_buffer[1].rx_size)
		{
			act_size = module_ptr->rx_buffer[1].rx_size;
		}
		
		memmove(buf, module_ptr->rx_buffer[1].buffer, act_size);
		return act_size;
	}
	else
	{
		if(act_size > module_ptr->rx_buffer[0].rx_size)
		{
			act_size = module_ptr->rx_buffer[0].rx_size;
		}
		
		memmove(buf, module_ptr->rx_buffer[0].buffer, act_size);
		return act_size;
	}
}

/*
 * brief: ��ȡUARTͨ�����
 * param: dev UART�豸ָ��
 * return: ͨ�����ָ��
*/
CommnicationType* Get_UartInstance(const rt_device_t dev)
{
	CommnicationType* instance = NULL;
		
	for(int i=0; i<g_map_count; i++)
	{
		if(g_commnication_map[i]->com_dev == dev)
		{
			instance = g_commnication_map[i];
			break;
		}
	}
	
	return instance;
}

/*
 * brief: ��ȡCANͨ�����
 * param: dev CAN�豸ָ��
 * param: filter_id ͨ���������ID
 * return: ͨ�����ָ��
*/
CommnicationType* Get_CanInstance(const rt_device_t dev, unsigned int filter_id)
{
	CommnicationType* instance = NULL;
		
	for(int i=0; i<g_map_count; i++)
	{
		//
		if(g_commnication_map[i]->com_dev == dev && 
		g_commnication_map[i]->can_config->filter_id == filter_id)
		{
			instance = g_commnication_map[i];
			break;
		}
	}
	
	return instance;
}

/*
 * brief: ˫���彻��������
 * param: module_ptr ͨ�����ָ��
 * return: ��
*/
void Exchange_Buffer(CommnicationType* module_ptr)
{
	if(module_ptr->current_rx_buffer == &module_ptr->rx_buffer[0])
	{
		module_ptr->rx_buffer[1].rx_size = 0;
		module_ptr->current_rx_buffer = &module_ptr->rx_buffer[1];
	}
	else
	{
		module_ptr->rx_buffer[0].rx_size = 0;
		module_ptr->current_rx_buffer = &module_ptr->rx_buffer[0];
	}
}

/*
 * brief: ��ȡ����У���
 * param: data ����
 * param: data_size ���ݴ�С
 * return: У���
*/
uint8_t Get_Verify_Sum(uint8_t* data, uint32_t data_size)
{
	uint8_t verify_sum = 0;
	
	for(int i=0; i<data_size; i++)
	{
			verify_sum += data[i];
			
			if(verify_sum < data[i])
			{
				verify_sum += 1;
			}
	}
	
	return verify_sum;
}

/*
 * brief: ��ȡ���������ݰ���С
 * param: module_ptr ͨ�����ָ��
 * param: data_size ��Ч���ݴ�С
 * return: �������ݰ������ݴ�С
*/
static uint32_t Get_Data_Packet_Size(CommnicationType* module_ptr, uint32_t data_size)
{
	uint32_t size = 0;
	
	if(module_ptr->com_mode == COM_MODE1)
	{
		if(module_ptr->data_info != NULL)
		{
			size += module_ptr->data_info->head_size + module_ptr->data_info->tail_size;
		}
	}
	else if(module_ptr->com_mode == COM_MODE2)
	{
		if(module_ptr->data_info != NULL)
		{
			size += module_ptr->data_info->head_size + 1;
		}
	}
	
	size += data_size;
	
	if(true == module_ptr->is_verify)
	{
		size++;
	}
	
	return size;
}

/*
 * brief: �������
 * param: module_ptr ͨ�����ָ��
 * param: send_data �����͵�����
 * param: data_size �����͵����ݴ�С
 * param: packet_size �������ݰ������ݴ�С
 * return: ��
*/
static void Package_Data(CommnicationType* module_ptr, uint8_t* send_data,
													uint32_t data_size, uint32_t packet_size)
{
	DataInfo *data_info;
	data_info = module_ptr->data_info;	//����ͷ
	
	if(module_ptr->com_mode == COM_MODE1)
	{
		//�������ͷ
		int i = 0;
		for(int j=0; j< data_info->head_size; j++)
		{
			module_ptr->tx_buffer[i++] = data_info->head[j];
		}
		
		//�������
		for(int j=0; j<data_size; j++)
		{
			module_ptr->tx_buffer[i++] = send_data[j];
		}
		
		//���У���
		if(true == module_ptr->is_verify)
		{
			uint8_t verify_sum = Get_Verify_Sum(send_data, data_size);
			module_ptr->tx_buffer[i++] = verify_sum;
		}
		
		//�������β
		for(int j=0; j< data_info->tail_size; j++)
		{
			module_ptr->tx_buffer[i++] = data_info->tail[j];
		}
	}
	else if(module_ptr->com_mode == COM_MODE2)
	{
		//�������ͷ
		int i = 0;
		for(int j=0; j< data_info->head_size; j++)
		{
			module_ptr->tx_buffer[i++] = data_info->head[j];
		}
		
		//������ݳ���
		if(true == module_ptr->is_verify)
		{
			//���ݳ��� + ���ݳ���λ���� + У��ͳ���
			module_ptr->tx_buffer[i++] = data_size + 2;
		}
		else
		{
			//���ݳ��� + ���ݳ���λ����
			module_ptr->tx_buffer[i++] = data_size + 1;
		}
		
		//�������
		for(int j=0; j<data_size; j++)
		{
			module_ptr->tx_buffer[i++] = send_data[j];
		}
		
		//���У���
		if(true == module_ptr->is_verify)
		{
			uint8_t verify_sum = Get_Verify_Sum(send_data, data_size);
			module_ptr->tx_buffer[i++] = verify_sum;
		}	
	}
}

static void Not_Define(void)
{
	assert(0);	//���躯��û�ж���
}

/*
 * brief: ��ʼ��ReceiveBuffer
 * param: recv_buffer ReceiveBufferָ��
 * param: size ���ջ�������С
 * return: ��
*/
static void Init_Receive_Buffer(ReceiveBuffer* recv_buffer, uint16_t size)
{
	recv_buffer->buffer = (uint8_t*)rt_malloc(sizeof(uint8_t) * size);
	recv_buffer->buffer_size = size;
	recv_buffer->rx_size = 0;
}


/***********************************����״̬��***********************************************/
/*
 * brief: ��λ����״̬
 * param: state ReceiveStateָ��
 * return: ��
*/
void Reset_State(ReceiveState* state)
{
	state->state = 0;
	state->head_count = 0;
	state->tail_count = 0;
	state->data_size = 0;	
}


/***********************************������Ϣ��***********************************************/
/*
 * brief: ��ʼ��DataInfo
 * param: data_info_ptr DataInfoָ��
 * param: head ����ͷ�׵�ַ
 * param: head_size ����ͷ��С
 * param: tail ����β�׵�ַ
 * param: tail_size ����β��С
 * param: data_length ��Ч���ݴ�С
 * return: ��
*/
void Init_Data_Info(DataInfo** data_info_ptr, uint8_t *head, uint8_t head_size, uint8_t *tail, 
										uint8_t tail_size, uint8_t data_length)
{
	//�����ڴ�
	*data_info_ptr = (DataInfo*)rt_malloc(sizeof(DataInfo));
	DataInfo* data_info = *data_info_ptr;
	
	if(head != NULL)
	{
		data_info->head = (uint8_t*)rt_malloc(sizeof(head_size));
		for(int i=0; i<head_size; i++)
		{
			data_info->head[i] = head[i];
		}
		data_info->head_size = head_size;			
	}
	else
	{
		data_info->head = NULL;
		data_info->head_size = 0;
	}
	
	if(tail != NULL)
	{
		data_info->tail = (uint8_t*)rt_malloc(sizeof(tail_size));
		for(int i=0; i<tail_size; i++)
		{
			data_info->tail[i] = tail[i];
		}	
		data_info->tail_size = tail_size;
	}
	else
	{
		data_info->tail = NULL;
		data_info->tail_size = 0;
	}
	
	data_info->data_length = data_length;
}

/***********************************����ʹ��***********************************************/
#if TEST_COMMUNICATION
#define SAMPLE_UART_NAME       "uart7"    /* �����豸���� */ 
#define SAMPLE_CAN1_NAME	   "can1"	  /* CAN�豸���� */ 	
#define SAMPLE_CAN2_NAME	   "can2"	  /* CAN�豸���� */ 	


static struct rt_semaphore rx_sem;

/*
 * brief: ���ݴ����̺߳���
 * param: parameter ����
 * return: ��
*/
void Data_Handle_Entry(void *parameter)
{
	CommnicationTypePtr com_instance = (CommnicationTypePtr)parameter;
	
	uint8_t *data = (uint8_t*)rt_malloc(sizeof(uint8_t) * 40);
	volatile int times = 0;
	while (1)
	{
		rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
		uint16_t size = Get_Receive_Data_Size(com_instance);
		if(size > 0)
		{
			size = Get_Receive_Data(com_instance, data, 40);
			if(size == 24)
			{
				data[0] = 'A';
				times++;
				//break;
			}
		}
	}
	
	rt_free(data);
}

/*
 * brief: UARTͨ���������
 * return: ��
*/
bool UART_Example(void)
{
	rt_err_t ret = RT_EOK;
	
	//����ͨ�����
	CommnicationTypePtr com_example_ptr;
	
	//���Ҵ����豸
	rt_device_t serial = rt_device_find(SAMPLE_UART_NAME);
	
	if (!serial)
	{
			rt_kprintf("find uart failed!\n");
			return RT_ERROR;
	}
	
	//��ʼ������ͷ
	uint8_t head[2] = {0x0D, 0x0A};
	uint8_t tail[2] = {0x0A, 0x0D};
	
	DataInfo* data_info;
	Init_Data_Info(&data_info, head, 2, tail, 2, 28);	
	
	//��ʼ��ͨ�����
	Init_Commnication(&com_example_ptr, COM_MODE1, UART_PROTOCOL, serial, 40, 40, &rx_sem, NULL, NULL);
	
	//ʹ��ͨ�����
	Enable_Communication(com_example_ptr, true);
	Enable_Receive(com_example_ptr, true);
	Set_Data_Info(com_example_ptr, data_info);
	//Enable_Data_Verify(com_example_ptr, true);
	
	//��ʼ���ź���
	rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
	
	//�������ݴ����߳�
	rt_thread_t thread = rt_thread_create("Data_Handle", Data_Handle_Entry, com_example_ptr, 1024, 5, 10);
	
	//�����������߳�
	if(thread != RT_NULL)
	{
		rt_thread_startup(thread);
	}
	else
	{
		ret = RT_ERROR;
		return ret;
	}	
	
//	while(1)
//	{		
//		if(false == Send_Data(com_example_ptr, (uint8_t*)"12AB32d23456767676767f432434321", 28))
//		{
//			ret = RT_ERROR;
//			break;
//		}				
//		rt_thread_mdelay(10);
//	}
	
	return ret;
}

bool UART_Example_RDT(void)
{
	rt_err_t ret = RT_EOK;
	
	//����ͨ�����
	CommnicationTypePtr com_example_ptr;
	
	//���Ҵ����豸
	rt_device_t serial = rt_device_find(SAMPLE_UART_NAME);
	
	if (!serial)
	{
			rt_kprintf("find uart failed!\n");
			return RT_ERROR;
	}
	
	//��ʼ������ͷ
	uint8_t head[2] = {0x0D, 0x0A};
	uint8_t tail[2] = {0x0A, 0x0D};
	
	DataInfo* data_info;
	Init_Data_Info(&data_info, head, 2, tail, 2, 28);	
	
	//��ʼ��ͨ�����
	Init_Commnication(&com_example_ptr, COM_MODE1, UART_PROTOCOL, serial, 40, 40, &rx_sem, NULL, NULL);
	
	//ʹ��ͨ�����
	Enable_Communication(com_example_ptr, true);
	Enable_Receive(com_example_ptr, true);
	Set_Data_Info(com_example_ptr, data_info);
	//Enable_Data_Verify(com_example_ptr, true);
	
	//��ʼ���ź���
	rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
	
	//�������ݴ����߳�
	rt_thread_t thread = rt_thread_create("Data_Handle", Data_Handle_Entry, com_example_ptr, 1024, 5, 10);
	
	//�����������߳�
	if(thread != RT_NULL)
	{
		rt_thread_startup(thread);
	}
	else
	{
		ret = RT_ERROR;
		return ret;
	}	
	
	while(1)
	{		
		if(false == Send_Data_RDT(com_example_ptr, (uint8_t*)"12AB32d23456767676767f432434321", 28))
		{
			ret = RT_ERROR;
			break;
		}				
		rt_thread_mdelay(10);
	}
	
	return ret;
}

/*
 * brief: CANͨ���������
 * return: ��
*/
bool Can_Example(void)
{
	rt_err_t ret = RT_EOK;
	
	//����ͨ�����
	CommnicationTypePtr com_tx_ptr;
	CommnicationTypePtr com_rx_ptr;
	
	//����CAN�豸
	rt_device_t tx_can_dev = rt_device_find(SAMPLE_CAN1_NAME);
	rt_device_t rx_can_dev = rt_device_find(SAMPLE_CAN2_NAME);

	if (!tx_can_dev)
	{
		rt_kprintf("find can failed!\n");
		return RT_ERROR;
	}
	
	if (!rx_can_dev)
	{
		rt_kprintf("find can failed!\n");
		return RT_ERROR;
	}

	/* ���жϽ��ռ����ͷ�ʽ�� CAN �豸 */
	ret = rt_device_open(tx_can_dev, RT_DEVICE_FLAG_INT_TX);
	ret = rt_device_open(rx_can_dev, RT_DEVICE_FLAG_INT_RX);

	struct rt_can_filter_item items[1] = {
		RT_CAN_FILTER_STD_INIT(0x486, RT_NULL, RT_NULL) //ƥ��idΪ0x08      
	};
	struct rt_can_filter_config cfg = {1, 1, items};
	/* ����Ӳ�����˱� */
	ret = rt_device_control(rx_can_dev, RT_CAN_CMD_SET_FILTER, &cfg);	
	
	//��ʼ������ͷ
	uint8_t head[2] = {0x0D, 0x0A};
	uint8_t tail[2] = {0x0A, 0x0D};
	
	DataInfo* data_info;
	Init_Data_Info(&data_info, head, 2, tail, 2, 28);	
	
	//��ʼ��CAN1ͨ�����
	Init_Commnication(&com_tx_ptr, COM_MODE1, CAN_PROTOCOL, tx_can_dev, 40, 40, &rx_sem, NULL, NULL);	

	//ʹ��CAN1ͨ�����
	Enable_Communication(com_tx_ptr, true);
	//Enable_Receive(com_tx_ptr, false);
	Set_Data_Info(com_tx_ptr, data_info);
	Set_Can_ID(com_tx_ptr, 0x486, 0);	//���÷���id�͹���id
	Enable_Data_Verify(com_tx_ptr, true);
	
	//��ʼ��CAN2ͨ�����
	Init_Commnication(&com_rx_ptr, COM_MODE1, CAN_PROTOCOL, tx_can_dev, 40, 40, &rx_sem, NULL, NULL);	

	//ʹ��CAN2ͨ�����
	Enable_Communication(com_rx_ptr, true);
	Enable_Receive(com_rx_ptr, true);
	Set_Data_Info(com_rx_ptr, data_info);
	Set_Can_ID(com_rx_ptr, 0x123, 0x486);	//���÷���id�͹���id
	Enable_Data_Verify(com_rx_ptr, true);

	//��ʼ���ź���
	rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
	
	//�������ݴ����߳�
	rt_thread_t thread = rt_thread_create("Data_Handle", Data_Handle_Entry, com_rx_ptr, 1024, 5, 10);
	
	//�����������߳�
	if(thread != RT_NULL)
	{
		rt_thread_startup(thread);
	}
	else
	{
		ret = RT_ERROR;
		return ret;
	}	
	
	while(1)
	{
		if(false == Send_Data(com_tx_ptr, (uint8_t*)"12AB32d23456767676767f432434321", 28))
		{
			ret = RT_ERROR;
			break;
		}			
		rt_thread_mdelay(10);
	}
	
	return ret;
}

#endif
