#include "dr16.h"
#include "usart.h"
#include <string.h>

RC_ctrl_t rc_ctrl;
uint8_t dbus_rx_buf[18];

// 使用的是 USART3 对应的 huart3
extern UART_HandleTypeDef huart3;

volatile uint32_t dr16_recv_count = 0;
volatile uint32_t dr16_valid_count = 0;

void DR16_UART_Init(void) {
    HAL_UART_DMAStop(&huart3);
    
    // 确保 USART3 中断优先级安全
    HAL_NVIC_SetPriority(USART3_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    
    // 启动空闲中断DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, dbus_rx_buf, 18);
    // 关闭DMA半传输中断，只留传输完成/空闲中断
    if (huart3.hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    }
}

//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
//    if (huart->Instance == USART3) {
//        dr16_recv_count++;
//        // 只有完整接收到18个字节时才解包
//        if (Size == 18) {
//            dr16_valid_count++;
//            DR16_Decode(dbus_rx_buf);
//        }
//        
//        // 清除可能导致死锁的错误标志位
//        __HAL_UART_CLEAR_OREFLAG(huart);
//        __HAL_UART_CLEAR_NEFLAG(huart);
//        __HAL_UART_CLEAR_FEFLAG(huart);

//        // 重启下一轮接收
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, dbus_rx_buf, 18);
//        if (huart3.hdmarx != NULL) {
//            __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
//        }
//    }
//}

//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
//    if (huart->Instance == USART3) {
//        __HAL_UART_CLEAR_OREFLAG(huart);
//        __HAL_UART_CLEAR_NEFLAG(huart);
//        __HAL_UART_CLEAR_FEFLAG(huart);
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, dbus_rx_buf, 18);
//    }
//}

void DR16_Decode(volatile const uint8_t *buff) {
    if (buff == NULL) return;

    // 1. 解析 4 个摇杆通道 (范围: -660 ~ 660, 中位 0)
    rc_ctrl.rc.ch[0] = ((buff[0] | (buff[1] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[1] = (((buff[1] >> 3) | (buff[2] << 5)) & 0x07FF) - RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[2] = (((buff[2] >> 6) | (buff[3] << 2) | (buff[4] << 10)) & 0x07FF) - RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[3] = (((buff[4] >> 1) | (buff[5] << 7)) & 0x07FF) - RC_CH_VALUE_OFFSET;

    // 2. 解析 2 个三位开关 (1:上, 3:中, 2:下)
    rc_ctrl.rc.s[0] = ((buff[5] >> 4) & 0x0003);        // S1 (左侧开关)
    rc_ctrl.rc.s[1] = ((buff[5] >> 6) & 0x0003);        // S2 (右侧开关，修正了之前的逻辑)

    // 3. 解析左侧上方侧边拨轮通道 (通道 4，位于 Byte 16 和 17)
    rc_ctrl.rc.ch[4] = ((buff[16] | (buff[17] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;
}
