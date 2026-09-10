//
// Created by ASUS on 2026/9/4.
//

#include <stdint.h>
#include "../../bsp/uart/uart.h"
#include "../../protocol/remote.h"
#include "../../application/global_data.h"
#include "cmsis_os2.h"
#include "uart_receive.h"

void uart1_recv_callback(struct uart_device *device,const uint8_t *data,uint16_t length)
{
}
void uart7_recv_callback(struct uart_device *device,const uint8_t *data,uint16_t length)
{
}
void uart10_recv_callback(struct uart_device *device,const uint8_t *data,uint16_t length)
{
}

void uart5_recv_callback(struct uart_device *device,const uint8_t *data,uint16_t length)
{
    int result;
    (void)device;
    result = remote_parse(data, length, &global_data.remote_data);

    if (result > 0)
    {
        global_data.remote_data.rc.remote_update_tick = osKernelGetTickCount();
        global_data.remote_data.rc.remote_ready = 1U;
    }
}