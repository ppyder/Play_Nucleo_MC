#include "communication.h"
#include "string.h"
#include "com_can.h"
#include "com_uart.h"

#define MAP_SIZE 10
/***********************************变量声明***********************************************/
//静态全局变量（私有变量），仅内部使用
static uint8_t g_map_count = 0;
static CommnicationType* g_commnication_map[MAP_SIZE] = {NULL};


/***********************************函数声明***********************************************/
//静态函数（私有函数），仅内部调用
static void Not_Define(void);
static void Init_Receive_Buffer(ReceiveBuffer* recv_buffer, uint16_t size);
static uint32_t Get_Data_Packet_Size(CommnicationType* module_ptr, uint32_t data_size);
static void Package_Data(CommnicationType* module_ptr, uint8_t* send_data,
													uint32_t data_size, uint32_t packet_size);


/***********************************通信组件***********************************************/
/*
 * brief: 初始化通信组件
 * param: module_ptr 通信组件，注意需要传入的参数是通信组件指针的地址
 * param: mode 通信模式
 * param: protocol 通信协议，UART_PROTOCOL 或者 CAN_PROTOCOL
 * param: com_dev_ptr 通信设备指针，uart设备指针或者can设备指针
 * param: rx_size 接收缓冲区大小
 * param: tx_size 发送缓冲区大小
 * param: sem 接收线程的信号量
 * param: send_handle_ptr 自定义发送处理函数指针，无则为NULL
 * param: receive_handle_ptr 自定义接收处理函数指针，无则为NULL
 * return: 无
*/
void Init_Commnication(CommnicationType** module_ptr,  COM_MODE mode, COM_PROTOCOL protocol, 
											void* com_dev_ptr, uint16_t rx_size, uint16_t tx_size, rt_sem_t sem, 
											uint32_t (*send_handle_ptr)(uint8_t*),
											bool (*receive_handle_ptr)(uint8_t* data, uint32_t size))
{
	//为通信组件申请内存
	*module_ptr = (CommnicationType*)rt_malloc(sizeof(CommnicationType));
	CommnicationTypePtr com_module = *module_ptr;
	
	//初始化通信模式
	com_module->com_mode = mode;
	//初始化通信协议
	com_module->com_protocol = protocol;
	if(protocol == CAN_PROTOCOL)
	{		
		com_module->can_config = (CanConfig*)rt_malloc(sizeof(CanConfig));
	}

	//初始化通信设备
	com_module->com_dev = com_dev_ptr;
	
	//初始化接收缓冲区
	Init_Receive_Buffer(&com_module->rx_buffer[0], rx_size);
	Init_Receive_Buffer(&com_module->rx_buffer[1], rx_size);	
	com_module->current_rx_buffer = &com_module->rx_buffer[0];
	
	//初始化发送缓冲区
	com_module->tx_buffer = (uint8_t*)rt_malloc(sizeof(uint8_t) * tx_size);
	com_module->tx_buffer_size = tx_size;	
	
	//初始化信号量
	com_module->rx_sem = sem;
	
	//初始化发送处理函数
	com_module->Send_Handle = send_handle_ptr;
	//初始化接收处理函数
	com_module->Receive_Handle = receive_handle_ptr;
	
	//初始化接收状态
	Reset_State(&com_module->recv_state);
	
	//初始化数据头
	com_module->data_info = NULL;

	com_module->is_verify = false;
	com_module->is_rdt = false;
	g_commnication_map[g_map_count++] = com_module;
}

/*
 * brief: 析构通信组件
 * param: module_ptr 通信组件指针
 * return: 无
*/
void Destory_Communication(CommnicationType* module_ptr)
{
	//关闭通信设备
	Enable_Communication(module_ptr, false);
	
	//释放内存
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
 * brief: 使能通信组件
 * param: module_ptr 通信组件指针
 * param: enable 是否使能，true 或者 false
 * return: 无
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
			//检测Can设备是否打开
			if(!Is_Can_Open(module_ptr->com_dev))
			{
				assert(0);	//can设备没有打开
			}
		}
	}	
}

/*
 * brief: 使能接收
 * param: module_ptr 通信组件指针
 * param: enable 是否使能，true 或者 false
 * return: 返回使能接收是否成功，true为成功。
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
 * brief: 设置数据信息，包括数据头，数据尾和数据长度
 * param: module_ptr 通信组件指针
 * param: data_info_ptr DataInfo指针
 * return: 返回是否成功，true为成功。
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
 * brief: 程序查询接收数据
 * param: module_ptr 通信组件指针
 * param: buffer 接收数据缓冲区
 * param: buffer_size 接收数据大小
 * return: 实际接收数据个数
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
 * brief: 以可靠数据传输的方式发送数据
 * param: module_ptr 通信组件指针
 * param: send_data 待发送的数据
 * param: data_size 待发送数据大小
 * return: 返回发送是否成功，true为成功。
*/
bool Send_Data_RDT(CommnicationType* module_ptr, uint8_t* send_data, uint32_t data_size)
{
	if(module_ptr->rdt_ack != COM_RDT_NOMAL)
	{
		return false;	//正在等待应答，不能发起下一次传输
	}
	
	int timer = 0;
	uint8_t ack = 0;
	uint8_t buffer[1] = {0xff};
	Send_Data(module_ptr, send_data, data_size);
	
	while(1)
	{
		rt_thread_mdelay(10);		
		
		if(timer > RDT_TIMEOUT)	//超时重传
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
			if(buffer[0] == 0x00)	//ACK应答
			{
				module_ptr->rdt_ack = COM_RDT_NOMAL;
				break;
			}
			else	//NAK应答
			{
				Send_Data(module_ptr, send_data, data_size);
			}
		}		
	}
}

/*
 * brief: 发送数据
 * param: module_ptr 通信组件指针
 * param: send_data 待发送的数据
 * param: data_size 待发送数据大小
 * return: 返回发送是否成功，true为成功。
*/
bool Send_Data(CommnicationType* module_ptr, uint8_t* send_data, uint32_t data_size)
{
	uint32_t packet_size = 0;
	
	if(module_ptr->com_mode == COM_CUSTOM)
	{
		//调用自定义发送处理函数
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
 * brief: 获取接收到的数据
 * param: module_ptr 通信组件指针
 * param: buf 接收数据的缓冲区
 * param: size 接收数据的缓冲区大小
 * return: 返回实际接收的数据大小
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
 * brief: 获取UART通信组件
 * param: dev UART设备指针
 * return: 通信组件指针
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
 * brief: 获取CAN通信组件
 * param: dev CAN设备指针
 * param: filter_id 通信组件过滤ID
 * return: 通信组件指针
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
 * brief: 双缓冲交换缓冲区
 * param: module_ptr 通信组件指针
 * return: 无
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
 * brief: 获取数据校验和
 * param: data 数据
 * param: data_size 数据大小
 * return: 校验和
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
 * brief: 获取待发送数据包大小
 * param: module_ptr 通信组件指针
 * param: data_size 有效数据大小
 * return: 整个数据包的数据大小
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
 * brief: 打包数据
 * param: module_ptr 通信组件指针
 * param: send_data 待发送的数据
 * param: data_size 待发送的数据大小
 * param: packet_size 整个数据包的数据大小
 * return: 无
*/
static void Package_Data(CommnicationType* module_ptr, uint8_t* send_data,
													uint32_t data_size, uint32_t packet_size)
{
	DataInfo *data_info;
	data_info = module_ptr->data_info;	//数据头
	
	if(module_ptr->com_mode == COM_MODE1)
	{
		//添加数据头
		int i = 0;
		for(int j=0; j< data_info->head_size; j++)
		{
			module_ptr->tx_buffer[i++] = data_info->head[j];
		}
		
		//添加数据
		for(int j=0; j<data_size; j++)
		{
			module_ptr->tx_buffer[i++] = send_data[j];
		}
		
		//添加校验和
		if(true == module_ptr->is_verify)
		{
			uint8_t verify_sum = Get_Verify_Sum(send_data, data_size);
			module_ptr->tx_buffer[i++] = verify_sum;
		}
		
		//添加数据尾
		for(int j=0; j< data_info->tail_size; j++)
		{
			module_ptr->tx_buffer[i++] = data_info->tail[j];
		}
	}
	else if(module_ptr->com_mode == COM_MODE2)
	{
		//添加数据头
		int i = 0;
		for(int j=0; j< data_info->head_size; j++)
		{
			module_ptr->tx_buffer[i++] = data_info->head[j];
		}
		
		//添加数据长度
		if(true == module_ptr->is_verify)
		{
			//数据长度 + 数据长度位长度 + 校验和长度
			module_ptr->tx_buffer[i++] = data_size + 2;
		}
		else
		{
			//数据长度 + 数据长度位长度
			module_ptr->tx_buffer[i++] = data_size + 1;
		}
		
		//添加数据
		for(int j=0; j<data_size; j++)
		{
			module_ptr->tx_buffer[i++] = send_data[j];
		}
		
		//添加校验和
		if(true == module_ptr->is_verify)
		{
			uint8_t verify_sum = Get_Verify_Sum(send_data, data_size);
			module_ptr->tx_buffer[i++] = verify_sum;
		}	
	}
}

static void Not_Define(void)
{
	assert(0);	//所需函数没有定义
}

/*
 * brief: 初始化ReceiveBuffer
 * param: recv_buffer ReceiveBuffer指针
 * param: size 接收缓冲区大小
 * return: 无
*/
static void Init_Receive_Buffer(ReceiveBuffer* recv_buffer, uint16_t size)
{
	recv_buffer->buffer = (uint8_t*)rt_malloc(sizeof(uint8_t) * size);
	recv_buffer->buffer_size = size;
	recv_buffer->rx_size = 0;
}


/***********************************接收状态类***********************************************/
/*
 * brief: 复位接收状态
 * param: state ReceiveState指针
 * return: 无
*/
void Reset_State(ReceiveState* state)
{
	state->state = 0;
	state->head_count = 0;
	state->tail_count = 0;
	state->data_size = 0;	
}


/***********************************数据信息类***********************************************/
/*
 * brief: 初始化DataInfo
 * param: data_info_ptr DataInfo指针
 * param: head 数据头首地址
 * param: head_size 数据头大小
 * param: tail 数据尾首地址
 * param: tail_size 数据尾大小
 * param: data_length 有效数据大小
 * return: 无
*/
void Init_Data_Info(DataInfo** data_info_ptr, uint8_t *head, uint8_t head_size, uint8_t *tail, 
										uint8_t tail_size, uint8_t data_length)
{
	//申请内存
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

/***********************************测试使用***********************************************/
#if TEST_COMMUNICATION
#define SAMPLE_UART_NAME       "uart7"    /* 串口设备名称 */ 
#define SAMPLE_CAN1_NAME	   "can1"	  /* CAN设备名称 */ 	
#define SAMPLE_CAN2_NAME	   "can2"	  /* CAN设备名称 */ 	


static struct rt_semaphore rx_sem;

/*
 * brief: 数据处理线程函数
 * param: parameter 参数
 * return: 无
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
 * brief: UART通信组件例程
 * return: 无
*/
bool UART_Example(void)
{
	rt_err_t ret = RT_EOK;
	
	//定义通信组件
	CommnicationTypePtr com_example_ptr;
	
	//查找串口设备
	rt_device_t serial = rt_device_find(SAMPLE_UART_NAME);
	
	if (!serial)
	{
			rt_kprintf("find uart failed!\n");
			return RT_ERROR;
	}
	
	//初始化数据头
	uint8_t head[2] = {0x0D, 0x0A};
	uint8_t tail[2] = {0x0A, 0x0D};
	
	DataInfo* data_info;
	Init_Data_Info(&data_info, head, 2, tail, 2, 28);	
	
	//初始化通信组件
	Init_Commnication(&com_example_ptr, COM_MODE1, UART_PROTOCOL, serial, 40, 40, &rx_sem, NULL, NULL);
	
	//使能通信组件
	Enable_Communication(com_example_ptr, true);
	Enable_Receive(com_example_ptr, true);
	Set_Data_Info(com_example_ptr, data_info);
	//Enable_Data_Verify(com_example_ptr, true);
	
	//初始化信号量
	rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
	
	//设置数据处理线程
	rt_thread_t thread = rt_thread_create("Data_Handle", Data_Handle_Entry, com_example_ptr, 1024, 5, 10);
	
	//创建并启动线程
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
	
	//定义通信组件
	CommnicationTypePtr com_example_ptr;
	
	//查找串口设备
	rt_device_t serial = rt_device_find(SAMPLE_UART_NAME);
	
	if (!serial)
	{
			rt_kprintf("find uart failed!\n");
			return RT_ERROR;
	}
	
	//初始化数据头
	uint8_t head[2] = {0x0D, 0x0A};
	uint8_t tail[2] = {0x0A, 0x0D};
	
	DataInfo* data_info;
	Init_Data_Info(&data_info, head, 2, tail, 2, 28);	
	
	//初始化通信组件
	Init_Commnication(&com_example_ptr, COM_MODE1, UART_PROTOCOL, serial, 40, 40, &rx_sem, NULL, NULL);
	
	//使能通信组件
	Enable_Communication(com_example_ptr, true);
	Enable_Receive(com_example_ptr, true);
	Set_Data_Info(com_example_ptr, data_info);
	//Enable_Data_Verify(com_example_ptr, true);
	
	//初始化信号量
	rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
	
	//设置数据处理线程
	rt_thread_t thread = rt_thread_create("Data_Handle", Data_Handle_Entry, com_example_ptr, 1024, 5, 10);
	
	//创建并启动线程
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
 * brief: CAN通信组件例程
 * return: 无
*/
bool Can_Example(void)
{
	rt_err_t ret = RT_EOK;
	
	//定义通信组件
	CommnicationTypePtr com_tx_ptr;
	CommnicationTypePtr com_rx_ptr;
	
	//查找CAN设备
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

	/* 以中断接收及发送方式打开 CAN 设备 */
	ret = rt_device_open(tx_can_dev, RT_DEVICE_FLAG_INT_TX);
	ret = rt_device_open(rx_can_dev, RT_DEVICE_FLAG_INT_RX);

	struct rt_can_filter_item items[1] = {
		RT_CAN_FILTER_STD_INIT(0x486, RT_NULL, RT_NULL) //匹配id为0x08      
	};
	struct rt_can_filter_config cfg = {1, 1, items};
	/* 设置硬件过滤表 */
	ret = rt_device_control(rx_can_dev, RT_CAN_CMD_SET_FILTER, &cfg);	
	
	//初始化数据头
	uint8_t head[2] = {0x0D, 0x0A};
	uint8_t tail[2] = {0x0A, 0x0D};
	
	DataInfo* data_info;
	Init_Data_Info(&data_info, head, 2, tail, 2, 28);	
	
	//初始化CAN1通信组件
	Init_Commnication(&com_tx_ptr, COM_MODE1, CAN_PROTOCOL, tx_can_dev, 40, 40, &rx_sem, NULL, NULL);	

	//使能CAN1通信组件
	Enable_Communication(com_tx_ptr, true);
	//Enable_Receive(com_tx_ptr, false);
	Set_Data_Info(com_tx_ptr, data_info);
	Set_Can_ID(com_tx_ptr, 0x486, 0);	//设置发送id和过滤id
	Enable_Data_Verify(com_tx_ptr, true);
	
	//初始化CAN2通信组件
	Init_Commnication(&com_rx_ptr, COM_MODE1, CAN_PROTOCOL, tx_can_dev, 40, 40, &rx_sem, NULL, NULL);	

	//使能CAN2通信组件
	Enable_Communication(com_rx_ptr, true);
	Enable_Receive(com_rx_ptr, true);
	Set_Data_Info(com_rx_ptr, data_info);
	Set_Can_ID(com_rx_ptr, 0x123, 0x486);	//设置发送id和过滤id
	Enable_Data_Verify(com_rx_ptr, true);

	//初始化信号量
	rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
	
	//设置数据处理线程
	rt_thread_t thread = rt_thread_create("Data_Handle", Data_Handle_Entry, com_rx_ptr, 1024, 5, 10);
	
	//创建并启动线程
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
