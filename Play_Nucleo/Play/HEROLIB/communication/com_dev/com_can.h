#ifndef _COM_CAN_H_
#define _COM_CAN_H_

#include <stdbool.h>
#include <rtdevice.h>
#include "communication.h"

extern inline bool Open_Can(rt_device_t dev, uint32_t mode)
{	
	rt_err_t ret = RT_EOK;	
	if(!(dev->open_flag & RT_DEVICE_OFLAG_OPEN))
	{
		ret = rt_device_open(dev, mode);
	}	
	
	return ((ret == RT_EOK)? true:false);
}

extern inline bool Is_Can_Open(rt_device_t dev)
{
	//判断设备是否打开
	return (dev->open_flag & RT_DEVICE_OFLAG_OPEN);
}

extern inline bool Close_Can(rt_device_t dev)
{
	rt_err_t ret;
	rt_device_close(dev);
	return ((ret == RT_EOK)? true:false);
}

extern uint32_t Can_Receive(CommnicationType* module_ptr, uint8_t* buffer, uint32_t buffer_size);
extern bool Can_Send(rt_device_t dev, unsigned int id, const void* buffer, rt_size_t size);

extern rt_err_t Can_Recv_Mode1(rt_device_t dev, rt_size_t size);
extern rt_err_t Can_Recv_Custom_Mode(rt_device_t dev, rt_size_t size);

#endif
