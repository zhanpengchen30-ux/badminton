#include "bsp_can_DM.h"

CANx_t CAN_1, CAN_2;
char Selection = 0;

GM6020_t GM6020 = {0};

uint8_t Data_Enable[8]    = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
uint8_t Data_Failure[8]   = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
uint8_t Data_Save_zero[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
uint8_t Data_Clear_Err[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};

const uint16_t DM_Ctrl_ID[5] = {0x301, 0x302, 0x303, 0x304, 0x305};

static void CAN_SendCmd(CAN_HandleTypeDef* hcan, uint16_t std_id, uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef Tx_Header;
    uint32_t TxMailbox;
    Tx_Header.StdId = std_id;
    Tx_Header.IDE   = CAN_ID_STD;
    Tx_Header.RTR   = CAN_RTR_DATA;
    Tx_Header.DLC   = len;
    
    uint32_t timeout = 0;
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) {
        timeout++;
        if (timeout > 500) return;
    }
    HAL_CAN_AddTxMessage(hcan, &Tx_Header, data, &TxMailbox);
}

void Motor_enable(void)
{
    for (int i = 0; i < 5; i++)
    {
        CAN_SendCmd(&hcan1, DM_Ctrl_ID[i], Data_Clear_Err, 8);
        HAL_Delay(5);
        CAN_SendCmd(&hcan1, DM_Ctrl_ID[i], Data_Enable, 8);
        HAL_Delay(10);
    }
}

void Motor_disable(void)
{
    for (int i = 0; i < 5; i++)
    {
        CAN_SendCmd(&hcan1, DM_Ctrl_ID[i], Data_Failure, 8);
        HAL_Delay(2);
    }
}

void psi_ctrl(CAN_HandleTypeDef* hcan, uint16_t id, float _pos, float _vel, float _cur)
{
    CAN_TxHeaderTypeDef Tx_Header;
    uint32_t TxMailbox;
    uint8_t Tx_Buf[8];
    
    uint16_t u16_vel = (uint16_t)(_vel * 100.0f);
    uint16_t u16_cur = (uint16_t)(_cur * 10000.0f);
    uint8_t *pbuf = (uint8_t*)&_pos;
    uint8_t *vbuf = (uint8_t*)&u16_vel;
    uint8_t *ibuf = (uint8_t*)&u16_cur;
    
    Tx_Header.StdId = id;
    Tx_Header.IDE   = CAN_ID_STD;
    Tx_Header.RTR   = CAN_RTR_DATA;
    Tx_Header.DLC   = 0x08;

    Tx_Buf[0] = *pbuf;
    Tx_Buf[1] = *(pbuf + 1);
    Tx_Buf[2] = *(pbuf + 2);
    Tx_Buf[3] = *(pbuf + 3);
    Tx_Buf[4] = *vbuf;
    Tx_Buf[5] = *(vbuf + 1);
    Tx_Buf[6] = *ibuf;
    Tx_Buf[7] = *(ibuf + 1);

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0) {
        HAL_CAN_AddTxMessage(hcan, &Tx_Header, Tx_Buf, &TxMailbox);
    }
}

void Motor_control(void)
{
    psi_ctrl(&hcan1, 
             DM_Ctrl_ID[Selection], 
             CAN_1.target_pos[Selection], 
             CAN_1.target_vel[Selection], 
             CAN_1.target_cur[Selection]);

    Selection++;
    if (Selection >= 5) {
        Selection = 0;
    }
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_1.Rx_pHeader, CAN_1.RxData) == HAL_OK)
        {
            // GM6020 反馈解析 (0x206)
            if (CAN_1.Rx_pHeader.StdId == 0x206)
            {
                GM6020.last_ecd      = GM6020.ecd;
                GM6020.ecd           = (CAN_1.RxData[0] << 8) | CAN_1.RxData[1];
                GM6020.speed_rpm     = (CAN_1.RxData[2] << 8) | CAN_1.RxData[3];
                GM6020.given_current = (CAN_1.RxData[4] << 8) | CAN_1.RxData[5];

                static uint8_t first_flag = 1;
                if (first_flag) {
                    GM6020.last_ecd = GM6020.ecd;
                    first_flag = 0;
                }

                // 多圈过零自动累加
                if (GM6020.ecd - GM6020.last_ecd > 4096) {
                    GM6020.round_count--;
                } else if (GM6020.ecd - GM6020.last_ecd < -4096) {
                    GM6020.round_count++;
                }

                // 计算绝对连续总角度
                GM6020.total_ecd = GM6020.round_count * 8192 + GM6020.ecd;
                return;
            }

            // 达妙 5 轴反馈
            int index = -1;
            switch(CAN_1.Rx_pHeader.StdId) {
                case 0x11: index = 0; break;
                case 0x12: index = 1; break;
                case 0x13: index = 2; break;
                case 0x14: index = 3; break;
                case 0x15: index = 4; break;
                default: break;
            }

            if (index >= 0 && index < 5)
            {
                CAN_1.error_code[index] = (CAN_1.RxData[0] >> 4);
                CAN_1.p_int[index] = (CAN_1.RxData[1] << 8) | CAN_1.RxData[2];
                CAN_1.v_int[index] = (CAN_1.RxData[3] << 4) | (CAN_1.RxData[4] >> 4);
                CAN_1.t_int[index] = ((CAN_1.RxData[4] & 0x0F) << 8) | CAN_1.RxData[5];

                CAN_1.position[index] = uint_to_float(CAN_1.p_int[index], P_MIN, P_MAX, 16);
                CAN_1.velocity[index] = uint_to_float(CAN_1.v_int[index], V_MIN, V_MAX, 12);
                CAN_1.torque[index]   = uint_to_float(CAN_1.t_int[index], T_MIN, T_MAX, 12);
            }
        }
    }
}

void GM6020_SendVoltage(int16_t voltage)
{
    CAN_TxHeaderTypeDef Tx_Header;
    uint32_t TxMailbox;
    uint8_t Tx_Buf[8] = {0};

    if (voltage > 25000)  voltage = 25000;
    if (voltage < -25000) voltage = -25000;

    Tx_Header.StdId = 0x1FF;
    Tx_Header.IDE   = CAN_ID_STD;
    Tx_Header.RTR   = CAN_RTR_DATA;
    Tx_Header.DLC   = 8;

    Tx_Buf[0] = 0;
    Tx_Buf[1] = 0;
    Tx_Buf[2] = (uint8_t)(voltage >> 8);
    Tx_Buf[3] = (uint8_t)(voltage & 0xFF);
    Tx_Buf[4] = 0;
    Tx_Buf[5] = 0;
    Tx_Buf[6] = 0;
    Tx_Buf[7] = 0;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
        HAL_CAN_AddTxMessage(&hcan1, &Tx_Header, Tx_Buf, &TxMailbox);
    }
}
