#include "com_can.h"
#include "string.h"

static bool Verify_Data(CommnicationType* com_instance, uint8_t pos, uint8_t size);
static void Handle_Data(CommnicationType* com_instance);

static bool Can_Send_ACK(CommnicationType* module_ptr, bool ack)
{
	struct rt_can_msg msg;

	msg.id = module_ptr->can_config->can_id;             
	msg.ide = RT_CAN_STDID;     // ��׼��ʽ
	msg.rtr = RT_CAN_DTR;       // ����֡
	msg.len = 1;	
	
	if(ack)
	{
		msg.data[0] = 0x00;
		return rt_device_write(module_ptr->com_dev, 0, &msg, sizeof(msg));
	}
	else
	{
		msg.data[0] = 0xff;
		return rt_device_write(module_ptr->com_dev, 0, &msg, sizeof(msg));
	}
}

uint32_t Can_Receive(CommnicationType* module_ptr, uint8_t* buffer, uint32_t buffer_size)
{
	struct rt_can_msg rxmsg;
	rxmsg.hdr = -1;
	rt_size_t receive_size = rt_device_read(module_ptr->com_dev, 0, &rxmsg, sizeof(rxmsg));
	
	if(receive_size == 0 || rxmsg.id != module_ptr->can_config->filter_id)
	{
		return 0;
	}
	
	uint32_t size = (buffer_size >= rxmsg.len)? rxmsg.len:buffer_size;
	memcpy(buffer, rxmsg.data, size);
	return size;
}

bool Can_Send(rt_device_t dev, unsigned int id, const void* buffer, rt_size_t size)
{
	struct rt_can_msg msg;

	msg.id = id;             
	msg.ide = RT_CAN_STDID;     // ��׼��ʽ
	msg.rtr = RT_CAN_DTR;       // ����֡
	
	int index = 0, send_size = 0, frame_size = 8;

	while (size)
	{
		if(size < frame_size)
		{		
			frame_size = size;						
		}
		
		msg.len = frame_size;
		memcpy(msg.data, (char*)(buffer) + index, frame_size);

		//����һ֡ CAN ����
		send_size = rt_device_write(dev, 0, &msg, sizeof(msg));
		
		if(send_size == 0)
		{
			return false;
		}

		size -= frame_size;	
		index += frame_size;
	}
	
	return true;
}

rt_err_t Can_Recv_Mode1(rt_device_t dev, rt_size_t size)
{	
	struct rt_can_msg rxmsg;
	rxmsg.hdr = -1;
	rt_size_t receive_size = rt_device_read(dev, 0, &rxmsg, sizeof(rxmsg));
	
	//���մ���
	if(receive_size == 0)
	{
		return RT_ERROR;
	}

	CommnicationType* com_instance = Get_CanInstance(dev, rxmsg.id);
	
	if(!com_instance)
	{
		return RT_ERROR;
	}
	
	int data_szie = rxmsg.len;

	ReceiveBuffer *rx_buffer = com_instance->current_rx_buffer;

	//���㻺����ʣ��ռ�
	int remained = rx_buffer->buffer_size - rx_buffer->rx_size;
	if(data_szie > remained)
	{
		data_szie = remained;
	}
	
	for(int i=0; i<data_szie; i++)
	{
		rx_buffer->buffer[rx_buffer->rx_size++] = rxmsg.data[i];
	}	
	
	if(rx_buffer->rx_size >= com_instance->data_info->data_length)
	{
		Handle_Data(com_instance);
	}	
	
	return true;
}

rt_err_t Can_Recv_Custom_Mode(rt_device_t dev, rt_size_t size)
{
	struct rt_can_msg rxmsg;
	rt_size_t receive_size = rt_device_read(dev, 0, &rxmsg, size);
	
	//���մ���
	if(receive_size != size)
	{
		return RT_ERROR;
	}

	CommnicationType* com_instance = Get_CanInstance(dev, rxmsg.id);
	
	if(!com_instance)
	{
		return RT_ERROR;
	}
	
	bool ret = com_instance->Receive_Handle(rxmsg.data, rxmsg.len);
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
						
						rt_sem_release(com_instance->rx_sem);	//�ͷ��ź���						
						break;
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
