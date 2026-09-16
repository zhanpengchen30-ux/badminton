#include "control_task.h"
#include "dr16.h"
#include "bsp_can_DM.h"
#include "FreeRTOS.h"
#include "task.h"
#include "custom_ctrl.h" 

/* ================= GM6020 直线化物理位置  ================= */
#define GM6020_HIT_POS         4103.0f   // 击打上面
#define GM6020_MID_POS         6215.0f   // 中间待命值 
#define GM6020_WIND_POS        8237.0f   // 往下蓄力

extern GM6020_t GM6020;
extern void GM6020_SendVoltage(int16_t voltage);

static uint8_t motor_enabled = 0;

/* ================= 达妙姿态基准数据 ================= */
const float FRONT_READY[5] = {  0.173f, -0.417f, -0.896f, 1.594f, 2.613f };
const float FRONT_HIT[5]   = {  0.230f,  1.055f, -2.381f, 1.569f, 2.205f };

const float LEFT_READY[5]  = {  1.709f, -0.309f, -0.907f, 1.606f, 1.260f };
const float LEFT_HIT[5]    = {  1.737f,  0.860f, -2.112f, 1.833f, 1.232f };

const float RIGHT_READY[5] = { -1.391f, -0.234f, -1.001f, 1.569f, 1.232f };
const float RIGHT_HIT[5]   = { -1.341f,  0.744f, -2.009f, 1.673f, 1.214f };

#define YAW_MAX_DEGREE   60.0f  
#define YAW_MAX_RANGE    (YAW_MAX_DEGREE * 3.1415926f / 180.0f) 

static float current_yaw   = 0.173f;
static float current_ratio = 0.0f;
static float target_ratio  = 0.0f;

#define RAMP_STEP      0.003f   
#define YAW_RAMP_STEP  0.018f   

// 滚轮平滑滤波比例 (-1.0 到 +1.0)
static float smooth_wheel_ratio = 0.0f;
#define RATIO_RAMP_STEP        0.012f    // 约 80ms 平滑，消除瞬间猛推冲击

/* ================= 调试专用监控探针 ================= */
volatile int16_t  dbg_rc_wheel     = 0;    // 1. 遥控器滚轮真实读数
volatile float    dbg_gm6020_target = 0;   // 2. 单片机算出的目标直线坐标
volatile float    dbg_gm6020_actual = 0;   // 3. 电机拉平后的真实直线坐标
volatile float    dbg_gm6020_error  = 0;   // 4. 真实误差
volatile int16_t  dbg_voltage_out   = 0;   // 5. PID 输出给电机的电压
volatile uint32_t dbg_mcu_heartbeat = 0;   // 6. 单片机心跳计数器

// 将 GM6020 单圈编码器拉平成无缝直线坐标 
static float Get_GM6020_Linear_Ecd(uint16_t ecd)
{
    if (ecd < 3800) {
        return (float)ecd + 8192.0f; 
    } else {
        return (float)ecd;           
    }
}

static int16_t GM6020_PID_Calc(float target, float current, int16_t now_rpm)
{
    float error = target - current; 
    dbg_gm6020_error = error;

    float Kp = 32.0f;  
    float Kd = 2.0f;   

    float output = Kp * error - Kd * (float)now_rpm;

    // 防止溢出反转
    if (output > 25000.0f)  output = 25000.0f;
    if (output < -25000.0f) output = -25000.0f;

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
        dbg_mcu_heartbeat++;

        // 1. 检查自定义控制器是否超时断连 (> 100ms 认为掉线)
        if (HAL_GetTick() - custom_ctrl.last_update_time > 100) {
            custom_ctrl.online = 0;
        }

        /* ================= 方案 A：自定义主手在线，执行 1:1 示教随动 ================= */
        if (custom_ctrl.online == 1)
        {
            if (motor_enabled == 0) {
                Motor_enable();
                motor_enabled = 1;
            }

            // 1. 达妙 5 个轴直接跟随手柄目标 (注意加上安全限幅)
            for (int i = 0; i < 5; i++) {
                CAN_1.target_pos[i] = custom_ctrl.arm_target[i];
            }
            Motor_control();

            // 2. GM6020 挥拍跟随手掌角度
            float current_linear = Get_GM6020_Linear_Ecd(GM6020.ecd);
            dbg_gm6020_actual = current_linear;
            dbg_gm6020_target = custom_ctrl.gm6020_target;

            int16_t voltage = GM6020_PID_Calc(custom_ctrl.gm6020_target, current_linear, GM6020.speed_rpm);
            dbg_voltage_out = voltage;
            GM6020_SendVoltage(voltage);
        }
        /* ================= 方案 B：自定义控制器掉线，无缝切回 DR16 遥控器 ================= */
        else if (rc_ctrl.rc.s[1] == 3)
        {
            const float *base_pos = NULL;
            const float *hit_pos  = NULL;

            // 1. 读取滚轮真实数据
            int16_t wheel = rc_ctrl.rc.ch[4];
            dbg_rc_wheel = wheel;

            // 2. 滚轮归一化目标比例 (-1.0 ~ +1.0)
            float target_wheel_ratio = 0.0f;
            if (wheel > 30) {
                target_wheel_ratio = (float)(wheel - 30) / (660.0f - 30.0f);  // 向上推：0.0 到 +1.0
            } else if (wheel < -30) {
                target_wheel_ratio = (float)(wheel + 30) / (660.0f - 30.0f); // 向下拨：0.0 到 -1.0
            } else {
                target_wheel_ratio = 0.0f; // 回中严格为 0！
            }

            if (target_wheel_ratio > 1.0f)  target_wheel_ratio = 1.0f;
            if (target_wheel_ratio < -1.0f) target_wheel_ratio = -1.0f;

            // 3. 平滑滤波比例，避免瞬间大阶跃
            if (smooth_wheel_ratio < target_wheel_ratio) {
                smooth_wheel_ratio += RATIO_RAMP_STEP;
                if (smooth_wheel_ratio > target_wheel_ratio) smooth_wheel_ratio = target_wheel_ratio;
            } else if (smooth_wheel_ratio > target_wheel_ratio) {
                smooth_wheel_ratio -= RATIO_RAMP_STEP;
                if (smooth_wheel_ratio < target_wheel_ratio) smooth_wheel_ratio = target_wheel_ratio;
            }

            // 4. 计算目标
            float target_linear = GM6020_MID_POS;
            if (smooth_wheel_ratio >= 0.0f) {
                target_linear = GM6020_MID_POS - smooth_wheel_ratio * (GM6020_MID_POS - GM6020_HIT_POS);
            } else {
                target_linear = GM6020_MID_POS + (-smooth_wheel_ratio) * (GM6020_WIND_POS - GM6020_MID_POS);
            }

            if (target_linear < 4103.0f) target_linear = 4103.0f;
            if (target_linear > 8237.0f) target_linear = 8237.0f;

            dbg_gm6020_target = target_linear;

            float current_linear = Get_GM6020_Linear_Ecd(GM6020.ecd);
            dbg_gm6020_actual = current_linear;

            int16_t voltage_out = GM6020_PID_Calc(target_linear, current_linear, GM6020.speed_rpm);
            dbg_voltage_out = voltage_out;

            GM6020_SendVoltage(voltage_out);

            // === 达妙击球方向选择 ===
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

            // === 达妙控制执行 ===
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
        /* ================= 方案 C：急停模式 (失能保护) ================= */
        else
        {
            GM6020_SendVoltage(0);
            dbg_voltage_out = 0;
            smooth_wheel_ratio = 0.0f;

            if (motor_enabled == 1) {
                Motor_disable();
                motor_enabled = 0;
                current_ratio = 0.0f;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
