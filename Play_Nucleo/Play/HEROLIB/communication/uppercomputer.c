#include "uppercomputer.h"
#include <rtdevice.h>
		
/***********************************�곣��***********************************************/
#define UPPER_DATA_SIZE 6
#define UPPER_HEAD1 0xEF
#define UPPER_HEAD2 0xFE
#define UPPER_TAIL1 0x0A
#define UPPER_TAIL2 0x0D

/***********************************�꺯��***********************************************/

/******************************ȫ�־�̬��������*******************************************/
static CommnicationTypePtr upper_com_ptr;
static struct rt_semaphore upper_rx_sem;
static UpperParaAdjust upper_para_adjust;
static UpperMap upper_map;
static uint8_t upper_mode;


/*********************************��������*******************************************/
void Upper_Data_Entry(void *parameter);
static void Upper_Handle_ParaData(uint8_t *data, int size);
static void Upper_Handle_MapData(uint8_t *data, int size);


/*
 * brief: ��λ��ͨ�ų�ʼ��
 * param: mode ������λ��ģʽ�����磺UPPER_PARAMETER_MODE | UPPER_MAP_MODE
 * param: uart_name UART�豸�����ƣ����磺uart1
 * return: ͨ�����ָ��
*/
CommnicationTypePtr Upper_Init(uint8_t mode, char* uart_name)
{
	upper_mode = mode;

	rt_device_t com_dev = rt_device_find(uart_name);
	if (!com_dev)
	{
		rt_kprintf("find can failed!\n");
		assert(0); //�򿪴���ʧ��
	}

    //��ʼ������ͷ
	uint8_t head[2] = {UPPER_HEAD1, UPPER_HEAD2};	
	
	DataInfo* data_info;
	Init_Data_Info(&data_info, head, 2, NULL, 0, UPPER_DATA_SIZE + 2);

	Init_Commnication(&upper_com_ptr, COM_MODE1, UART_PROTOCOL, com_dev, UPPER_DATA_SIZE + 5, 
											UPPER_DATA_SIZE + 5, &upper_rx_sem, NULL, NULL);

	//ʹ��CAN2ͨ�����
	Enable_Communication(upper_com_ptr, true);
	Enable_Receive(upper_com_ptr, true);
	Set_Data_Info(upper_com_ptr, data_info);
	Enable_Data_Verify(upper_com_ptr, true);

	//��ʼ���ź���
	rt_sem_init(&upper_rx_sem, "upper_rx_sem", 0, RT_IPC_FLAG_FIFO);

	//�������ݴ����߳�
	rt_thread_t thread = rt_thread_create("Data_Handle", Upper_Data_Entry, upper_com_ptr,
                                             1024, 5, 10);

	//�����������߳�
	if(thread != RT_NULL)
	{
		rt_thread_startup(thread);
	}
	else
	{
		assert(0);  //�̴߳���ʧ��
	}

    return upper_com_ptr;
}

/*
 * brief: ������λ��ͨ��ģ��
 * param: ��
 * return: ��
*/
void Upper_Destroy()
{
    //����ͨ�����
    Destory_Communication(upper_com_ptr);
    
    //�ͷ�UpperPara������ڴ�
    for(int i=0; i<upper_para_adjust.size; i++)
    {
        if(upper_para_adjust.head[i] == NULL)
        {
            break;
        }
        else
        {
            free(upper_para_adjust.head[i]);
        }
    }

    //�ͷ�head������ڴ�
    free(upper_para_adjust.head);
}

/*********************************��������ģ��*******************************************/

/*
 * brief: ʹ����λ������ģʽ
 * param: paras_number ����������
 * return: ��
*/
void Upper_Enable_Para_Mode(uint8_t paras_number)
{
    UpperPara **paras = (UpperPara**)rt_malloc(sizeof(UpperPara*) * paras_number);
    upper_para_adjust.auto_id = 0;
    upper_para_adjust.size = paras_number;
    upper_para_adjust.head = paras;

    for(int i=0; i<paras_number; i++)
    {
        paras[i] = NULL;
    }
}

/*
 * brief: ��Ӳ���
 * param: type ��������
 * param: para ������ַ
 * return: ��
*/
void Upper_Add_One_Para(UpperParaType type, void* para)
{
    if(upper_para_adjust.auto_id >= upper_para_adjust.size)
    {
        assert(0);  //���������������õ�����
    }

    UpperPara *upper_para = (UpperPara*)rt_malloc(sizeof(UpperPara));
		upper_para_adjust.head[upper_para_adjust.auto_id] = upper_para;
    upper_para->id = upper_para_adjust.auto_id++;
    upper_para->type = type;
    
		if(type == UPPER_BOOL_TYPE)
		{
				upper_para->para = (bool*)para;
		}
		else if(type == UPPER_INTEGER_TYPE)
		{
				upper_para->para = (int*)para;
		}
		else if(type == UPPER_FLOAT_TYPE)
		{
				upper_para->para = (float*)para;
		}
		else
		{
				upper_para->para = (int*)para;
		}
}

/*
 * brief: ��Ӷ������
 * param: type �������ͣ�ע�������������ͱ�������ͬ��
 * param: para �����׵�ַ
 * param: para ��������
 * return: ��
*/
void Upper_Add_Paras(UpperParaType type, void* para, int size)
{
    if(upper_para_adjust.size - upper_para_adjust.auto_id < size)
    {
        assert(0);  //���������������õ�����
    }

    for(int i=0; i<size; i++)
    {   
        UpperPara *upper_para = (UpperPara*)rt_malloc(sizeof(UpperPara));
				upper_para_adjust.head[upper_para_adjust.auto_id] = upper_para;
        upper_para->id = upper_para_adjust.auto_id++;
        upper_para->type = type;

        if(type == UPPER_BOOL_TYPE)
        {
            upper_para->para = &((bool*)para)[i];
        }
        else if(type == UPPER_INTEGER_TYPE)
        {
            upper_para->para = &((int*)para)[i];
        }
        else if(type == UPPER_FLOAT_TYPE)
        {
            upper_para->para =  &((float*)para)[i];
        }
				else
				{
						upper_para->para = &((int*)para)[i];
				}
    }
}

/*********************************��ͼģ��*******************************************/

/*
 * brief: ʹ����λ����ͼģʽ
 * param: sem ������λ��·�����߳��ź���
 * param: point_number ·���е�ĸ����������յĵ�ĸ����ﵽpoint_number�����ͷ��ź���
 * return: ��
*/
void Upper_Enable_Map_Mode(struct rt_semaphore *sem, int point_number)
{
    upper_map.map_sem = sem;
    upper_map.size = point_number;
}

/*
 * brief: ����λ�����͵�ǰ�����˵�·��
 * param: x ·��x������׵�ַ
 * param: y ·��y������׵�ַ
 * param: size ·���е�ĸ���
 * return: ��
*/
void Upper_Send_Path(uint16_t *x, uint16_t *y, int size)
{
    uint8_t data[6];
    data[0] = UPPER_MAP_MODE & 0xff;

    //��λ��ǰ
    for(int i=0; i<size; i++)
    {
        data[1] = x[i] & 0xff;
        data[2] = x[i] >> 8 & 0xff;
        data[3] = y[i] & 0xff;
        data[4] = y[i] >> 8 & 0xff;
        data[5] = 0;

        Send_Data(upper_com_ptr, data, sizeof(data));
    }
}

/*
 * brief: ��λ�������߳�
 * param: parameter ����
 * return: ��
*/
void Upper_Data_Entry(void *parameter)
{
	CommnicationTypePtr com_instance = (CommnicationTypePtr)parameter;
	
	uint8_t *data = (uint8_t*)rt_malloc(sizeof(uint8_t) * UPPER_DATA_SIZE);

  uint16_t size;
	while (1)
	{
		rt_sem_take(&upper_rx_sem, RT_WAITING_FOREVER);
		size = Get_Receive_Data_Size(com_instance);
		if(size != UPPER_DATA_SIZE)
		{
				continue;
		}

		Get_Receive_Data(com_instance, data, size);		

		if((upper_mode & UPPER_PARAMETER_MODE) == UPPER_PARAMETER_MODE && data[0] == UPPER_PARAMETER_MODE)
		{
				Upper_Handle_ParaData(data + 1, size - 1);
		}
		else if((upper_mode & UPPER_MAP_MODE) == UPPER_MAP_MODE && data[0] == UPPER_MAP_MODE)
		{
				Upper_Handle_MapData(data + 1, size - 1);
		}
	}
	free(data);
}

/*
 * brief: ��λ������ģʽ���ݴ���
 * param: data ����
 * param: size ���ݸ���
 * return: ��
*/
void Upper_Handle_ParaData(uint8_t *data, int size)
{
    uint8_t id = data[0];
    for(int i=0; i<upper_para_adjust.auto_id; i++)
    {
        if(id == upper_para_adjust.head[i]->id)
        {
            if(upper_para_adjust.head[i]->type == UPPER_BOOL_TYPE)
            {
                *((bool*)upper_para_adjust.head[i]->para) = (bool)data[1];
								break;
            }
            else if(upper_para_adjust.head[i]->type == UPPER_INTEGER_TYPE)
            {
                int value = 0;
                for(int j=0; j<4; j++)
                {
                    value = value | (data[j+1] & 0xff) << (8 * j);  //��λ��ǰ
                }

                *((int*)upper_para_adjust.head[i]->para) = value;
								break;
            }
            else if(upper_para_adjust.head[i]->type == UPPER_FLOAT_TYPE)
            {
                int value = 0;
                for(int j=0; j<4; j++)
                {
                    value = value | (data[j+1] & 0xff) << (8 * j); //��λ��ǰ
                }

                *((float*)upper_para_adjust.head[i]->para) = *((float*)(&value));
								break;
            }
        }
    }
}

/*
 * brief: ��λ����ͼģʽ���ݴ���
 * param: data ����
 * param: size ���ݸ���
 * return: ��
*/
void Upper_Handle_MapData(uint8_t *data, int size)
{
    static int count = 0;
    
    uint16_t x = (data[0] & 0xff) | (data[1] & 0xff) << 8;
    uint16_t y = (data[2] & 0xff) | (data[3] & 0xff) << 8;

    count++;

    if(count >= upper_map.size)
    {
        rt_sem_release(upper_map.map_sem);
        count = 0;
    }    
}

/********************************��λ����������*******************************************/
#define TEST_UPPER 1

#if TEST_UPPER

#define UPPER_UART "uart7"

static int int_para[5] = {0, 1, 2, 3, 4};
static float float_para[3] = {1.23, 4.56, 0.11};
static bool bool_para[2] = {true, false};

static struct rt_semaphore map_sem;

void Test_Para_Mode()
{    
    Upper_Init(UPPER_PARAMETER_MODE, UPPER_UART);
    
    Upper_Enable_Para_Mode(10);
    
    Upper_Add_Paras(UPPER_INTEGER_TYPE, int_para, 5);
    Upper_Add_Paras(UPPER_FLOAT_TYPE, float_para, 3);
    Upper_Add_Paras(UPPER_BOOL_TYPE, bool_para, 2);
		
    while(1)
    {			
			rt_thread_mdelay(20);
    }
}


void Test_Map_Entry(void *parameter)
{
    while(1)
    {
        rt_sem_take(&map_sem, RT_WAITING_FOREVER);
        rt_kprintf("receiving!\n");
    }
}

void Test_Map_Mode()
{
	Upper_Init(UPPER_MAP_MODE, UPPER_UART);
	
	rt_sem_init(&map_sem, "upper_rx_sem", 0, RT_IPC_FLAG_FIFO);
	Upper_Enable_Map_Mode(&map_sem, 5);
	
	rt_thread_t thread = rt_thread_create("MAP_Handle", Test_Map_Entry, NULL, 1024, 5, 10);
	
	//�����������߳�
	if(thread != RT_NULL)
	{
		rt_thread_startup(thread);
	}
	else
	{
		assert(0);
	}

	uint16_t x[10] = {100, 200, 300, 550, 340};
	uint16_t y[10] = {200, 110, 230, 450, 444};
	
	while(1)
	{
			Upper_Send_Path(x, y, sizeof(x));
			rt_thread_mdelay(10);
	}
}

//#include "Oscilloscope.h"

//void Test_Oscilloscope()
//{
//    Oscilloscope_open();
//}

#endif
