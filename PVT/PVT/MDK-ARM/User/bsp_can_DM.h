#ifndef __BSP_CAN_DM_H
#define __BSP_CAN_DM_H
#include "can.h"

#define Motar_mode 3    
#define MOTOR_NUM  5    

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);

typedef struct
{
    int p_int[MOTOR_NUM], v_int[MOTOR_NUM], t_int[MOTOR_NUM];
    float position[MOTOR_NUM], velocity[MOTOR_NUM], torque[MOTOR_NUM];
    uint8_t error_code[MOTOR_NUM];
    
    float target_pos[MOTOR_NUM];
    float target_vel[MOTOR_NUM];
    float target_cur[MOTOR_NUM];
    
    uint8_t Tx_Data[8];
    uint8_t RxData[8];
    CAN_RxHeaderTypeDef Rx_pHeader;
} CANx_t;

typedef struct {
    uint16_t ecd;          // 单圈角度
    uint16_t last_ecd;     
    int32_t  round_count;  
    int32_t  total_ecd;    // 连续总角度
    int16_t  speed_rpm;
    int16_t  given_current;
} GM6020_t;

extern GM6020_t GM6020;
void GM6020_SendVoltage(int16_t voltage);

extern CANx_t CAN_1, CAN_2;

void Motor_enable(void);
void Motor_disable(void);
void Motor_control(void);
void psi_ctrl(CAN_HandleTypeDef* hcan, uint16_t id, float _pos, float _vel, float _cur);

#endif
