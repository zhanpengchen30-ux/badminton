#include "bsp_usart.h"
#include "dr16.h"  

uint8_t sbus_rx_buf[18]; 
extern UART_HandleTypeDef huart3; 
static volatile uint32_t rc_last_update_tick = 0U;

void RC_Init(void) {
    rc_last_update_tick = 0U;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, sbus_rx_buf, 18);
    
    if (huart3.hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    }
}

uint8_t RC_IsOnline(void)
{
    uint32_t last_tick = rc_last_update_tick;

    if (last_tick == 0U) {
        return 0U;
    }

    return ((uint32_t)(HAL_GetTick() - last_tick) <= RC_TIMEOUT_MS) ? 1U : 0U;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART3) { 
        if (Size == 18) {
            DR16_Decode(sbus_rx_buf); // 调用遥控器解包函数
            rc_last_update_tick = HAL_GetTick();
        }
        
        // 清标志防卡死
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        
        // 重新开启下一次接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, sbus_rx_buf, 18);
        if (huart3.hdmarx != NULL) {
            __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) { 
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, sbus_rx_buf, 18); 
    }
}
