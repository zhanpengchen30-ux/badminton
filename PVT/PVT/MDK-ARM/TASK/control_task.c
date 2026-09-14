#include "control_task.h"
#include "dr16.h"
#include "bsp_can_DM.h"
#include "FreeRTOS.h"
#include "task.h"

/* ================= GM6020 真实待命零位与行程 ================= */
#define GM6020_READY_TOTAL     5527.0f   // 中间待命基准角度
#define SWING_TOTAL_RANGE      2100.0f   // 向上或向下最大挥拍行程

extern GM6020_t GM6020;
extern void GM6020_SendVoltage(int16_t voltage);

static uint8_t motor_enabled = 0;

/* ================= 1. 达妙姿态基准数据 ================= */
const float FRONT_READY[5] = {  0.173f, -0.417f, -0.896f, 1.594f, 2.813f };
const float FRONT_HIT[5]   = {  0.230f,  1.055f, -2.381f, 1.569f, 2.805f };

const float LEFT_READY[5]  = {  1.709f, -0.309f, -0.907f, 1.606f, 1.260f };
const float LEFT_HIT[5]    = {  1.737f,  0.860f, -2.112f, 1.833f, 1.232f };

const float RIGHT_READY[5] = { -1.391f, -0.234f, -1.001f, 1.569f, 1.232f };
const float RIGHT_HIT[5]   = { -1.341f,  0.744f, -2.009f, 1.673f, 1.214f };

/* ================= 2. 参数调节 ================= */
#define YAW_MAX_DEGREE   60.0f  
#define YAW_MAX_RANGE    (YAW_MAX_DEGREE * 3.1415926f / 180.0f) 

static float current_yaw   = 0.173f;
static float current_ratio = 0.0f;
static float target_ratio  = 0.0f;

#define RAMP_STEP      0.003f   
#define YAW_RAMP_STEP  0.018f   

// 【修复 1】：PID 参数类型修正为 int32_t，彻底支持连续多圈负数和大数值
static int16_t GM6020_PID_Calc(float target_ecd, int32_t now_total, int16_t now_rpm)
{
    float error = target_ecd - (float)now_total;
    float Kp = 30.0f;  // 力量不够可适当改大至 35~40
    float Kd = 2.0f;   // 阻尼防抖

    float output = Kp * error - Kd * (float)now_rpm;
    return (int16_t)output;
}

void Control_Task(void *argument)
{
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 达妙电机速度与电流初始化
    CAN_1.target_vel[0] = 6.0f;
    CAN_1.target_cur[0] = 6.0f;
    CAN_1.target_pos[0] = FRONT_READY[0];

    CAN_1.target_vel[1] = 4.0f;
    CAN_1.target_cur[1] = 6.0f;
    CAN_1.target_pos[1] = FRONT_READY[1];

    CAN_1.target_vel[2] = 200.0f; 
    CAN_1.target_cur[2] = 6.5f;
    CAN_1.target_pos[2] = FRONT_READY[2];

    for (int i = 3; i < 5; i++) {
        CAN_1.target_vel[i] = 3.5f;
        CAN_1.target_cur[i] = 5.0f;
        CAN_1.target_pos[i] = FRONT_READY[i];
    }

    while (1)
    {
        const float *base_pos = NULL;
        const float *hit_pos  = NULL;

        /* ================= 1. 右拨杆使能工作状态 ================= */
        if (rc_ctrl.rc.s[1] == 3)
        {
            // === A. GM6020 拨轮 (ch[4]) 挥拍控制 ===
            int16_t wheel = rc_ctrl.rc.ch[4];
            float swing_offset = 0.0f;

            // 【修复 2】：死区修复！改为常规的 30，确保推滚轮能正常响应
            if (wheel > 30 || wheel < -30) {
                swing_offset = ((float)wheel / 660.0f) * SWING_TOTAL_RANGE;
            } else {
                swing_offset = 0.0f; 
            }

            // 目标计算：在待命位置 5527 基础上双向增减
            float gm6020_target_total = GM6020_READY_TOTAL + swing_offset;

            // 【修复 3】：对称限幅！放宽到 [3200, 7800]，上拨下拨都有 2100 以上的完整行程
            if (gm6020_target_total > 7800.0f) gm6020_target_total = 7800.0f;
            if (gm6020_target_total < 3200.0f) gm6020_target_total = 3200.0f;

            // 调用多圈连续 PID
            int16_t voltage_out = GM6020_PID_Calc(gm6020_target_total, GM6020.total_ecd, GM6020.speed_rpm);
            GM6020_SendVoltage(voltage_out);

            // === B. 达妙击球方向选择 ===
            if (rc_ctrl.rc.s[0] == 1)      // 上：左击球
            {
                base_pos = LEFT_READY;
                hit_pos  = LEFT_HIT;
            }
            else if (rc_ctrl.rc.s[0] == 3) // 中：正面击球
            {
                base_pos = FRONT_READY;
                hit_pos  = FRONT_HIT;
            }
            else if (rc_ctrl.rc.s[0] == 2) // 下：右击球
            {
                base_pos = RIGHT_READY;
                hit_pos  = RIGHT_HIT;
            }

            // === C. 达妙控制执行 ===
            if (base_pos != NULL && hit_pos != NULL)
            {
                if (motor_enabled == 0) {
                    Motor_enable();
                    current_ratio = 0.0f;
                    current_yaw = base_pos[0];
                    motor_enabled = 1;
                }

                int16_t ch_yaw = rc_ctrl.rc.ch[0];
                float yaw_ratio = 0.0f;
                if (ch_yaw > 30) {
                    yaw_ratio = (float)(ch_yaw - 30) / (660.0f - 30.0f);
                } else if (ch_yaw < -30) {
                    yaw_ratio = (float)(ch_yaw + 30) / (660.0f - 30.0f);
                }
                if (yaw_ratio > 1.0f)  yaw_ratio = 1.0f;
                if (yaw_ratio < -1.0f) yaw_ratio = -1.0f;

                float desired_yaw = base_pos[0] - (yaw_ratio * YAW_MAX_RANGE);

                if (current_yaw < desired_yaw) {
                    current_yaw += YAW_RAMP_STEP;
                    if (current_yaw > desired_yaw) current_yaw = desired_yaw;
                } else if (current_yaw > desired_yaw) {
                    current_yaw -= YAW_RAMP_STEP;
                    if (current_yaw < desired_yaw) current_yaw = desired_yaw;
                }

                int16_t ch_hit = rc_ctrl.rc.ch[1];
                if (ch_hit > 30) {
                    target_ratio = (float)(ch_hit - 30) / (660.0f - 30.0f);
                    if (target_ratio > 1.0f) target_ratio = 1.0f;
                } else {
                    target_ratio = 0.0f;
                }

                if (current_ratio < target_ratio) {
                    current_ratio += RAMP_STEP;
                    if (current_ratio > target_ratio) current_ratio = target_ratio;
                } else if (current_ratio > target_ratio) {
                    current_ratio -= RAMP_STEP;
                    if (current_ratio < target_ratio) current_ratio = target_ratio;
                }

                CAN_1.target_pos[0] = current_yaw;
                for (int i = 1; i < 5; i++) {
                    CAN_1.target_pos[i] = base_pos[i] + current_ratio * (hit_pos[i] - base_pos[i]);
                }

                Motor_control();
            }
        }
        /* ================= 2. 急停模式 ================= */
        else
        {
            GM6020_SendVoltage(0);

            if (motor_enabled == 1) {
                Motor_disable();
                motor_enabled = 0;
                current_ratio = 0.0f;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
