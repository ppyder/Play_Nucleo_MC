#ifndef _COM_UART_H_
#define _COM_UART_H_

#include <stdbool.h>
#include <rtdevice.h>
#include "communication.h"

extern inline bool Open_Uart(rt_device_t dev, uint32_t mode)
{	
	rt_err_t ret;
	ret = rt_device_open(dev, mode);
	return ((ret == RT_EOK)? true:false);
}

extern inline bool Close_Uart(rt_device_t dev)
{
	rt_err_t ret;
	ret = rt_device_close(dev);
	return ((ret == RT_EOK)? true:false);
}

extern uint32_t Uart_Receive(CommnicationType* module_ptr, uint8_t* buffer, uint32_t buffer_size);
extern bool Uart_Send(rt_device_t dev, const void* buffer, rt_size_t size);
extern rt_err_t Uart_Recv_Mode1(rt_device_t dev, rt_size_t size);
extern rt_err_t Uart_Recv_Mode2(rt_device_t dev, rt_size_t size);
extern rt_err_t Uart_Recv_Mode3(rt_device_t dev, rt_size_t size);
extern rt_err_t Uart_Recv_Custom_Mode(rt_device_t dev, rt_size_t size);

#endif
