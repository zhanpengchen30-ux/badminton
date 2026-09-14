#include "PID.h"
#include <math.h>
#include <stdlib.h>  // 新增此行
/* PID初始化 */
void pid_init1(pid_type_def *pid, pid_mode_t mode, float PID[3], 
             float max_out, float max_iout, float ff_gain) {
    if (pid == NULL) return;
    
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->ff_gain = ff_gain;
    
    // 清零历史数据
    pid->err[0] = pid->err[1] = pid->err[2] = 0.0f;
    pid->err_sum = 0.0f;
		pid->ff_gain = ff_gain; // 当ff_gain=0时，前馈项自动失效					 
}

/* PID计算 */
float pid_calc1(pid_type_def *pid, float act_val, float flag_val) {
    if (pid == NULL) return 0.0f;

    /* 误差计算 */
    pid->err[2] = pid->err[1];
    pid->err[1] = pid->err[0];
    pid->err[0] = flag_val - act_val;

    /* 积分项处理 */
    if (fabs(pid->err[0]) < pid->max_iout / pid->Ki) {
        pid->err_sum += pid->err[0];
    } else {
        pid->err_sum *= 0.95f; // 抗饱和衰减
    }
    
    /* 微分项滤波 */
    static float d_filter = 0.0f;
    d_filter = 0.6f * d_filter + 0.4f * (pid->err[0] - pid->err[1]);

    /* 各分量计算 */
    float p_out = pid->Kp * pid->err[0];
    float i_out = pid->Ki * pid->err_sum;
    float d_out = pid->Kd * d_filter;

    /* 模式相关处理 */
    float output = 0.0f;
    switch (pid->mode) {
        case PID_POSITION:
            output = p_out + i_out + d_out;
            break;
            
        case PID_VELOCITY:
            // 速度环增加微分前馈
            d_out = pid->Kd * (pid->err[0] - pid->err[1]);
            output = p_out + i_out + d_out;
            break;
    }

    /* 输出限幅 */
    output = fmaxf(fminf(output, pid->max_out), -pid->max_out);
    
    return output;
}
