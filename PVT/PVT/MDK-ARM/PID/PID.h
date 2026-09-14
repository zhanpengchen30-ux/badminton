#ifndef __PID_H
#define __PID_H
#include <stddef.h>  // 新增此行，定义NULL宏
typedef enum {
    PID_POSITION,
    PID_VELOCITY  // 添加缺失的枚举值
} pid_mode_t;

typedef struct {
    pid_mode_t mode;
    float Kp, Ki, Kd;
    float err[3];
    float err_sum;
    float max_out;
    float max_iout;
    float ff_gain;
} pid_type_def;

void pid_init1(pid_type_def *pid, pid_mode_t mode, float PID[3], 
             float max_out, float max_iout, float ff_gain);
float pid_calc1(pid_type_def *pid, float act_val, float flag_val);

#endif
