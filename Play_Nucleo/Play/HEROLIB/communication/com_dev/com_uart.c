#include "string.h"
#include "com_uart.h"

static bool Verify_Data(CommnicationType* com_instance, uint8_t pos, uint8_t size);
static void Handle_Data(CommnicationType* com_instance);

static bool Uart_Send_ACK(CommnicationType* module_ptr, bool ack)
{
	if(ack)
	{
		return rt_device_write(module_ptr->com_dev, 0, (void*)((uint8_t)0x00), 1);
	}
	else
	{
		return rt_device_write(module_ptr->com_dev, 0, (void*)((uint8_t)0xff), 1);
	}
}

inline static void Receive_Finish(CommnicationType* module_ptr, bool success)
{
	if(module_ptr->is_rdt)
	{
		Uart_Send_ACK(module_ptr, success);
	}
		
	if(success)
	{			
			rt_sem_release(module_ptr->rx_sem);	//�ͷ��ź���
	}	
}

uint32_t Uart_Receive(CommnicationType* module_ptr, uint8_t* buffer, uint32_t buffer_size)
{
	uint32_t receive_size = rt_device_read(module_ptr->com_dev, 0, buffer, buffer_size);
	return receive_size;
}

bool Uart_Send(rt_device_t dev, const void* buffer, rt_size_t size)
{
	rt_size_t send_size = rt_device_write(dev, 0, buffer, size);
	
	if(send_size != size)
	{
		return false;
	}
	return true;
}

rt_err_t Uart_Recv_Mode1(rt_device_t dev, rt_size_t size)
{
	CommnicationType* com_instance = Get_UartInstance(dev);
	
	if(!com_instance)
	{
		return RT_ERROR;
	}
	
	if(com_instance->is_rdt && com_instance->rdt_ack != COM_RDT_NOMAL)
	{
		return RT_EOK;	//RDT���ͣ�������
	}
	
	uint8_t *buffer = (uint8_t*)malloc(size * sizeof(uint8_t));
	if(buffer == NULL)
	{
		free(buffer);
		return RT_ERROR;
	}
	
	rt_size_t receive_size = rt_device_read(dev, 0, buffer, size);	
	
	//���մ���
	if(receive_size != size)
	{		
		free(buffer);
		return RT_ERROR;
	}
	
	ReceiveBuffer *rx_buffer = com_instance->current_rx_buffer;

	//���㻺����ʣ��ռ�
	int remained = rx_buffer->buffer_size - rx_buffer->rx_size;
	if(receive_size > remained)
	{
		receive_size = remained;
	}
	
	for(int i=0; i<receive_size; i++)
	{
		rx_buffer->buffer[rx_buffer->rx_size++] = buffer[i];
	}	
	
	if(rx_buffer->rx_size >= com_instance->data_info->data_length)
	{
		Handle_Data(com_instance);
	}
	
	free(buffer);
	
	return true;
}

rt_err_t Uart_Recv_Mode2(rt_device_t dev, rt_size_t size)
{
	CommnicationType* com_instance = Get_UartInstance(dev);
	
	if(!com_instance)
	{
		return RT_ERROR;
	}
	
	ReceiveState *state = &(com_instance->recv_state);
	ReceiveBuffer* rx_buffer = com_instance->current_rx_buffer;
	
	bool is_finished = false;
	uint8_t *buffer = (uint8_t*)malloc(size * sizeof(uint8_t));
	uint8_t read_count = 0;
	
	rt_size_t receive_size = rt_device_read(dev, 0, buffer, size);	
	
	//���մ���
	if(receive_size != size)
	{
		Reset_State(state);
		free(buffer);
		return RT_ERROR;
	}
		
	while(read_count < receive_size)
	{
		switch(state->state)
		{
			case RECEIVE_HEAD:	//��������ͷ
			{				
				if(buffer[read_count++] == com_instance->data_info->head[state->head_count])
				{
					state->head_count++;
					if(state->head_count >= com_instance->data_info->head_size)
					{
						state->state = RECEIVE_DATA;
					}
				}
				else
				{
					read_count -= state->head_count;
					state->head_count = 0;
				}
				break;
			}
			case RECEIVE_DATA:	//��������
			{
				if(state->data_size == 0)
				{
					state->data_size = buffer[read_count++] - 1;	//ɾ�����ݳ���λ
				}
				else
				{
					rx_buffer->buffer[rx_buffer->rx_size++] = buffer[read_count++];				
				}				
				
				if(rx_buffer->rx_size > rx_buffer->buffer_size)
				{
					Reset_State(state);
					rx_buffer->rx_size = 0;
					continue;
				}
				else if(rx_buffer->rx_size >= state->data_size)
				{
					state->state = RECEIVE_FINISHED;	//�������
					
					//У������
					if(true == com_instance->is_verify)
					{
						uint8_t verify_sum = Get_Verify_Sum(rx_buffer->buffer, rx_buffer->rx_size-1);
						
						if(verify_sum == rx_buffer->buffer[rx_buffer->rx_size - 1])
						{
							is_finished = true;
							rx_buffer->rx_size = rx_buffer->rx_size -1;	//ɾ��У���
							
							Exchange_Buffer(com_instance);
							rx_buffer = com_instance->current_rx_buffer;
							Reset_State(state);
							continue;
						}
						else
						{
							//����У��ʧ��
							Reset_State(state);
							rx_buffer->rx_size = 0;
							continue;	
						}
					}
				}
				break;
			}
			case RECEIVE_FINISHED:
			{
				break;
			}
			default:
				break;
		}
	}
	
	free(buffer);
	
	if(true == is_finished)
	{
		rt_sem_release(com_instance->rx_sem);	//�ͷ��ź���
	}
	return RT_EOK;
}

rt_err_t Uart_Recv_Mode3(rt_device_t dev, rt_size_t size)
{
	CommnicationType* com_instance = Get_UartInstance(dev);
	
	if(!com_instance)
	{
		return RT_ERROR;
	}

	ReceiveState *state = &(com_instance->recv_state);
	ReceiveBuffer* rx_buffer = com_instance->current_rx_buffer;
	
	uint8_t *buffer = (uint8_t*)malloc(size * sizeof(uint8_t));
	uint8_t read_count = 0;
	
	rt_size_t receive_size = rt_device_read(dev, 0, buffer, size);	
	
	//���մ���
	if(receive_size != size)
	{
		Reset_State(state);
		free(buffer);
		return RT_ERROR;
	}
	
	while(read_count < receive_size)
	{
		if(state->state == RECEIVE_HEAD)	//��������ͷ
		{
			if(buffer[read_count++] == com_instance->data_info->head[state->head_count])
			{
				state->head_count++;
				if(state->head_count >= com_instance->data_info->head_size)
				{
					state->state = RECEIVE_DATA;
				}
			}
			else
			{
				read_count -= state->head_count;
				state->head_count = 0;
			}
		}
		else if(state->state == RECEIVE_DATA)	//��������
		{
			rx_buffer->buffer[rx_buffer->rx_size++] = buffer[read_count++];				
				
			//û���ҵ�����β
			if(rx_buffer->rx_size > rx_buffer->buffer_size)
			{
				Reset_State(state);
				rx_buffer->rx_size = 0;
				continue;
			}
			
			//������β
			if(rx_buffer->rx_size >= com_instance->data_info->tail_size)
			{
				int i = 0;
				for(; i<com_instance->data_info->tail_size; i++)
				{
					//�ж�����β
					if(rx_buffer->buffer[rx_buffer->rx_size - com_instance->data_info->tail_size + i]
						!= com_instance->data_info->tail[i])
					{
						break;
					}
				}
				
				if(i == com_instance->data_info->tail_size)
				{
					state->state = RECEIVE_FINISHED;	//�������
					rx_buffer->rx_size -= com_instance->data_info->tail_size;	//ɾ�����յ�����β
					
					//У������
					if(true == com_instance->is_verify)
					{						
						if(Verify_Data(com_instance, 0, rx_buffer->rx_size) == true)
						{	//����У��ɹ�
							
							if(com_instance->is_verify)
							{
								rx_buffer->rx_size = rx_buffer->rx_size -1;	//ɾ�����������е�У���
							}
							
							Exchange_Buffer(com_instance);	//˫���彻��������
							rx_buffer = com_instance->current_rx_buffer;
							Reset_State(state);	//���ý���״̬
							rt_sem_release(com_instance->rx_sem);	//�ͷ��ź���
						}
						else
						{
							//����У��ʧ��					
							Reset_State(state);
							rx_buffer->rx_size = 0;
						}
					}				
				}
			}
		}		
	}
	
	free(buffer);
	
	return RT_EOK;
}

rt_err_t Uart_Recv_Custom_Mode(rt_device_t dev, rt_size_t size)
{
	CommnicationType* com_instance = Get_UartInstance(dev);
	
	if(!com_instance)
	{
		return RT_ERROR;
	}
	
	uint8_t *buffer = malloc(size * sizeof(uint8_t));
	rt_size_t receive_size = rt_device_read(dev, 0, buffer, size);	
	
	//���մ���
	if(receive_size != size)
	{		
		free(buffer);
		return RT_ERROR;
	}
	
	bool ret = com_instance->Receive_Handle(buffer, size);
	free(buffer);
	
	return ((ret == true)? RT_EOK:RT_ERROR);
}

static bool Verify_Data(CommnicationType* com_instance, uint8_t pos, uint8_t size)
{
	ReceiveBuffer *rx_buffer = com_instance->current_rx_buffer;
	if(true == com_instance->is_verify)
	{
		uint8_t verify_sum = Get_Verify_Sum((rx_buffer->buffer) + pos, size-1);
		
		if(verify_sum == rx_buffer->buffer[pos + size - 1])
		{
			return true;	//У��ɹ�
		}
		else
		{
			return false; 	//У��ʧ��
		}
	}
	
	return true;	//��У��
}

static void Handle_Data(CommnicationType* com_instance)
{
	ReceiveBuffer *rx_buffer = com_instance->current_rx_buffer;
	ReceiveState *recv_state = &(com_instance->recv_state);
	DataInfo *data_info = com_instance->data_info;
	
	int size = rx_buffer->rx_size;
	int head_pos = 0;
	int index;
	
	while(size >= data_info->data_length)
	{
		index = 0;
		while(index < size)
		{
			if(recv_state->state == RECEIVE_HEAD)	//��������ͷ
			{
				if(rx_buffer->buffer[index++] == data_info->head[recv_state->head_count])
				{
					recv_state->head_count++;
					if(recv_state->head_count >= com_instance->data_info->head_size)
					{
						recv_state->state = RECEIVE_DATA;
						head_pos = index - data_info->head_size;
					}
				}
				else
				{
					index -= recv_state->head_count;
					recv_state->head_count = 0;
				}
			}
			else if(recv_state->state == RECEIVE_DATA)	//��������
			{
				//����������Ƿ���һ���������ݰ���������
				if((size - index) >= (data_info->data_length - data_info->head_size))
				{
					recv_state->state = RECEIVE_TAIL;
				}
				else
				{
					break;
				}
			}
			else if(recv_state->state == RECEIVE_TAIL)	//��������β
			{
				index += data_info->data_length - data_info->head_size - data_info->tail_size;
				int i = 0;
				for(; i<data_info->tail_size; i++)
				{ //�ж�����β
					if(rx_buffer->buffer[index++] != data_info->tail[i])
					{
						break;
					}
				}
				//У������
				if(i == com_instance->data_info->tail_size)
				{
					int vaild_data_length = data_info->data_length - data_info->head_size - data_info->tail_size;
					if(Verify_Data(com_instance, head_pos + data_info->head_size, vaild_data_length) == true)
					{
						//�л�������
						Exchange_Buffer(com_instance);
						memmove(com_instance->current_rx_buffer->buffer, rx_buffer->buffer + index, size-index);
						com_instance->current_rx_buffer->rx_size = size-index;
						
						int data_pos = index - vaild_data_length - data_info->tail_size;
						
						if(com_instance->is_verify)
						{
							vaild_data_length -= 1;
						}
						
						//�����յ������ݷ��뻺����
						memmove(rx_buffer->buffer, rx_buffer->buffer + data_pos, vaild_data_length);
						rx_buffer->rx_size = vaild_data_length;
						
						rx_buffer = com_instance->current_rx_buffer;						
						size -= index;
						
						Receive_Finish(com_instance, true);
						//rt_sem_release(com_instance->rx_sem);	//�ͷ��ź���						
						break;
					}
					else
					{
						Receive_Finish(com_instance, false);
					}
				}
				
				//û�ҵ�����β����У��ʧ��
				int data_pos = head_pos + data_info->head_size;
				memmove(rx_buffer->buffer, rx_buffer->buffer + data_pos, size - data_pos);
				rx_buffer->rx_size = size - data_pos;
				size -= data_pos;					
				break;			
			}
		}
		//û�ҵ�����ͷ
		if(recv_state->state == RECEIVE_HEAD)
		{
			int pos = size - (data_info->head_size - 1);
			memmove(rx_buffer->buffer, rx_buffer->buffer + pos, data_info->head_size - 1);
			rx_buffer->rx_size = data_info->head_size - 1;
			size -= index;
		}
		else if(recv_state->state == RECEIVE_DATA)	//��⵽����ͷ��������λ������
		{
			memmove(rx_buffer->buffer, rx_buffer->buffer + index - data_info->head_size,
						size - (index - data_info->head_size));
			rx_buffer->rx_size = size - (index - data_info->head_size);
			size -= index;
		}
		
		Reset_State(recv_state);
	}
}
