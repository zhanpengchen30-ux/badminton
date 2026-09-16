#include "bsp_usart.h"
#include "dr16.h"  

uint8_t sbus_rx_buf[18]; 
extern UART_HandleTypeDef huart3; 
extern void Custom_Ctrl_RxCallback(UART_HandleTypeDef *huart, uint16_t Size);

void RC_Init(void) {
   
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, sbus_rx_buf, 18);
    
    if (huart3.hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART3) { 
        if (Size == 18) {
            DR16_Decode(sbus_rx_buf); // 调用遥控器解包函数
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
		
if (huart->Instance == USART6)
{
    Custom_Ctrl_RxCallback(huart, Size);
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
