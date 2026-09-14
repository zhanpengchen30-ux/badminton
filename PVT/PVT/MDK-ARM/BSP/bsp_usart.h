#ifndef __BSP_USART_H
#define __BSP_USART_H

#include "usart.h"

extern uint8_t sbus_rx_buf[18]; // 暴露出接收缓冲区

void RC_Init(void); // 初始化接收函数

#endif
