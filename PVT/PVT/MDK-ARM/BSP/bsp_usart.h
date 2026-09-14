#ifndef __BSP_USART_H
#define __BSP_USART_H

#include "usart.h"

extern uint8_t sbus_rx_buf[18]; // ±©Â¶³ö½ÓÊÕ»º³åÇø

/* 超过该时间没有收到完整有效帧，认为遥控器失联。 */
#define RC_TIMEOUT_MS 100U

void RC_Init(void); // ³õÊ¼»¯½ÓÊÕº¯Êý
uint8_t RC_IsOnline(void);

#endif
