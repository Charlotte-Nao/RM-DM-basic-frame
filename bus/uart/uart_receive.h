//
// Created by ASUS on 2026/9/4.
//

#ifndef DM_UART_RECEIVE_H
#define DM_UART_RECEIVE_H

void uart1_recv_callback(struct uart_device *device,const uint8_t *data,uint16_t length);
void uart7_recv_callback(struct uart_device *device,const uint8_t *data,uint16_t length);
void uart10_recv_callback(struct uart_device *device,const uint8_t *data,uint16_t length);

#endif //DM_UART_RECEIVE_H