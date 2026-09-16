#ifndef __CUSTOM_CTRL_H
#define __CUSTOM_CTRL_H

#include "main.h"

// 1. 无线通信的 14 字节定长保命数据帧格式 (必须 pack(1) 对齐，防止内存填充空隙)
#pragma pack(push, 1)
typedef struct {
    uint8_t head1;       // 固定 0xAA
    uint8_t head2;       // 固定 0x55
    int16_t angles[6];   // 6 个关节编码器角度 (每个占 2 字节，共 12 字节)
                         // angles[0]   : 手掌挥拍 (对应主手 DRK_encoder[0])
                         // angles[1~5] : 5 个手臂关节 (对应主手 DRK_encoder[1~5])
} Master_Arm_Data_t;
#pragma pack(pop)

// 2. 解包后的控制数据结构体
typedef struct {
    float   arm_target[5];     // 5 个达妙电机的目标角度 (弧度制 rad)
    float   gm6020_target;     // GM6020 挥拍电机的目标角度
    uint32_t last_update_time; // 上次收到数据的时间戳 (用于掉线保护)
    uint8_t  online;           // 1: 在线连接中, 0: 掉线断开
} Custom_Ctrl_t;

extern Custom_Ctrl_t custom_ctrl;

// 函数声明
void Custom_Ctrl_Init(void);
void Custom_Ctrl_Decode(uint8_t *buf, uint16_t len);
void Custom_Ctrl_RxCallback(UART_HandleTypeDef *huart, uint16_t Size);

#endif
