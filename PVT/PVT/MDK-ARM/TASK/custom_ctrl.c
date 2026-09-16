#include "custom_ctrl.h"
#include "usart.h"

Custom_Ctrl_t custom_ctrl = {0};

// DMA 双倍接收缓冲区（防止偶发拼包溢出）
static uint8_t zigbee_rx_buf[32]; 

extern UART_HandleTypeDef huart6; // 你在 CubeMX 中配置的 USART6

/**
 * @brief  初始化并启动 USART6 的 DMA 空闲中断接收
 */
void Custom_Ctrl_Init(void)
{
    // 启动空闲中断 DMA 接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, zigbee_rx_buf, sizeof(zigbee_rx_buf));
    
    // 关闭半传输中断，只保留帧完成和空闲中断
    if (huart6.hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
    }
}

/**
 * @brief  串口事件回调函数 (一旦 Zigbee 收到一整包数据并产生空闲中断，硬件自动触发)
 */

void Custom_Ctrl_RxCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART6)
    {
        if (Size >= sizeof(Master_Arm_Data_t))
        {
            Custom_Ctrl_Decode(zigbee_rx_buf, Size);
        }

        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);

        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, zigbee_rx_buf, sizeof(zigbee_rx_buf));
        if (huart6.hdmarx != NULL) {
            __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
        }
    }
}

/**
 * @brief  核心解包与角度映射函数
 */
void Custom_Ctrl_Decode(uint8_t *buf, uint16_t len)
{
    // 1. 查找 0xAA 0x55 帧头 (滑窗匹配，抗无线杂波与字节错位)
    for (uint16_t i = 0; i <= len - sizeof(Master_Arm_Data_t); i++)
    {
        if (buf[i] == 0xAA && buf[i + 1] == 0x55)
        {
            // 2. 指针强转，1 行代码无缝提取 6 轴数据
            Master_Arm_Data_t *p_data = (Master_Arm_Data_t*)&buf[i];

            // 3. 映射 5 个达妙电机关节 (angles[1] ~ angles[5])
            // 假设主手传的是角度制 (0~360度)，这里转为达妙识别的弧度制 (rad)
            for (int j = 0; j < 5; j++)
            {
                float deg = (float)p_data->angles[j + 1];
                custom_ctrl.arm_target[j] = deg * 3.1415926f / 180.0f;
            }

            // 4. 映射第 6 个电机 GM6020 挥拍手掌 (angles[0])
            // 将主手手掌的角度线性映射到 GM6020 的 [5103 ~ 7215 ~ 9237] 击打区间
            // 假设主手手掌角度为 -90 ~ +90 度，映射到 ±2000 个编码器刻度：
            float palm_deg = (float)p_data->angles[0];
            custom_ctrl.gm6020_target = 7215.0f + (palm_deg / 90.0f) * 2000.0f;

            // 5. 刷新在线时间戳 (心跳防丢包)
            custom_ctrl.last_update_time = HAL_GetTick();
            custom_ctrl.online = 1;

            return; // 成功解析一包后退出
        }
    }
}
