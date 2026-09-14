#include "control_task.h"
#include "dr16.h"
#include "bsp_can_DM.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_usart.h"

/* ================= GM6020 ÕæÊµ´ýÃüÁãÎ»ÓëÐÐ³Ì ================= */
#define SWING_TOTAL_RANGE      2100.0f   // ÏòÉÏ»òÏòÏÂ×î´ó»ÓÅÄÐÐ³Ì
#define GM6020_VOLTAGE_LIMIT   25000.0f

extern GM6020_t GM6020;
extern void GM6020_SendVoltage(int16_t voltage);

static uint8_t motor_enabled = 0;
static uint8_t gm6020_ready_captured = 0U;
static float gm6020_ready_total = 0.0f;

/* ================= 1. ´ïÃî×ËÌ¬»ù×¼Êý¾Ý ================= */
const float FRONT_READY[5] = {  0.173f, -0.417f, -0.896f, 1.594f, 2.813f };
const float FRONT_HIT[5]   = {  0.230f,  1.055f, -2.381f, 1.569f, 2.805f };

const float LEFT_READY[5]  = {  1.709f, -0.309f, -0.907f, 1.606f, 1.260f };
const float LEFT_HIT[5]    = {  1.737f,  0.860f, -2.112f, 1.833f, 1.232f };

const float RIGHT_READY[5] = { -1.391f, -0.234f, -1.001f, 1.569f, 1.232f };
const float RIGHT_HIT[5]   = { -1.341f,  0.744f, -2.009f, 1.673f, 1.214f };

/* ================= 2. ²ÎÊýµ÷½Ú ================= */
#define YAW_MAX_DEGREE   60.0f  
#define YAW_MAX_RANGE    (YAW_MAX_DEGREE * 3.1415926f / 180.0f) 

static float current_yaw   = 0.173f;
static float current_ratio = 0.0f;
static float target_ratio  = 0.0f;

#define RAMP_STEP      0.003f   
#define YAW_RAMP_STEP  0.018f   

// ¡¾ÐÞ¸´ 1¡¿£ºPID ²ÎÊýÀàÐÍÐÞÕýÎª int32_t£¬³¹µ×Ö§³ÖÁ¬Ðø¶àÈ¦¸ºÊýºÍ´óÊýÖµ
static int16_t GM6020_PID_Calc(float target_ecd, int32_t now_total, int16_t now_rpm)
{
    float error = target_ecd - (float)now_total;
    float Kp = 30.0f;  // Á¦Á¿²»¹»¿ÉÊÊµ±¸Ä´óÖÁ 35~40
    float Kd = 2.0f;   // ×èÄá·À¶¶

    float output = Kp * error - Kd * (float)now_rpm;

    /* 必须在 float 转 int16_t 之前限幅，避免强误差时发生实现相关的截断/溢出。 */
    if (output > GM6020_VOLTAGE_LIMIT) {
        output = GM6020_VOLTAGE_LIMIT;
    } else if (output < -GM6020_VOLTAGE_LIMIT) {
        output = -GM6020_VOLTAGE_LIMIT;
    }

    return (int16_t)output;
}

void Control_Task(void *argument)
{
    vTaskDelay(pdMS_TO_TICKS(1000));

    // ´ïÃîµç»úËÙ¶ÈÓëµçÁ÷³õÊ¼»¯
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

        /* 遥控器失联时，停止所有本任务控制的执行器。 */
        if (!RC_IsOnline())
        {
            GM6020_SendVoltage(0);
            if (motor_enabled == 1) {
                Motor_disable();
                motor_enabled = 0;
            }
            current_ratio = 0.0f;
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        /*
         * 第一次收到有效反馈时捕获当前机械位置作为待命基准。
         * 这样基准不会在每次挥拍或每次重新使能时回到固定的 5527。
         */
        if (!GM6020.feedback_valid)
        {
            GM6020_SendVoltage(0);
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        if (!gm6020_ready_captured)
        {
            gm6020_ready_total = (float)GM6020.total_ecd;
            gm6020_ready_captured = 1U;
        }

        /* ================= 1. ÓÒ²¦¸ËÊ¹ÄÜ¹¤×÷×´Ì¬ ================= */
        if (rc_ctrl.rc.s[1] == 3)
        {
            // === A. GM6020 ²¦ÂÖ (ch[4]) »ÓÅÄ¿ØÖÆ ===
            int16_t wheel = rc_ctrl.rc.ch[4];
            float swing_offset = 0.0f;

            // ¡¾ÐÞ¸´ 2¡¿£ºËÀÇøÐÞ¸´£¡¸ÄÎª³£¹æµÄ 30£¬È·±£ÍÆ¹öÂÖÄÜÕý³£ÏìÓ¦
            if (wheel > 30 || wheel < -30) {
                swing_offset = ((float)wheel / 660.0f) * SWING_TOTAL_RANGE;
            } else {
                swing_offset = 0.0f; 
            }

            // 以首次有效反馈捕获的待命位置为基准，禁止每次挥拍重置到固定值
            float gm6020_target_total = gm6020_ready_total + swing_offset;

            // ¡¾ÐÞ¸´ 3¡¿£º¶Ô³ÆÏÞ·ù£¡·Å¿íµ½ [3200, 7800]£¬ÉÏ²¦ÏÂ²¦¶¼ÓÐ 2100 ÒÔÉÏµÄÍêÕûÐÐ³Ì
            if (gm6020_target_total > 7800.0f) gm6020_target_total = 7800.0f;
            if (gm6020_target_total < 3200.0f) gm6020_target_total = 3200.0f;

            // µ÷ÓÃ¶àÈ¦Á¬Ðø PID
            int16_t voltage_out = GM6020_PID_Calc(gm6020_target_total, GM6020.total_ecd, GM6020.speed_rpm);
            GM6020_SendVoltage(voltage_out);

            // === B. ´ïÃî»÷Çò·½ÏòÑ¡Ôñ ===
            if (rc_ctrl.rc.s[0] == 1)      // ÉÏ£º×ó»÷Çò
            {
                base_pos = LEFT_READY;
                hit_pos  = LEFT_HIT;
            }
            else if (rc_ctrl.rc.s[0] == 3) // ÖÐ£ºÕýÃæ»÷Çò
            {
                base_pos = FRONT_READY;
                hit_pos  = FRONT_HIT;
            }
            else if (rc_ctrl.rc.s[0] == 2) // ÏÂ£ºÓÒ»÷Çò
            {
                base_pos = RIGHT_READY;
                hit_pos  = RIGHT_HIT;
            }

            // === C. ´ïÃî¿ØÖÆÖ´ÐÐ ===
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
        /* ================= 2. ¼±Í£Ä£Ê½ ================= */
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
