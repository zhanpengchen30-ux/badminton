#ifndef _PID_H
#define _PID_H

#include "main.h"
#include "struct_typedef.h"


//#define LIMIT_MIN_MAX(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))





typedef struct _pid_struct_t
{
  float kp;
  float ki;
  float kd;
  float i_max;
  float out_max;
  
  float ref;      // target value
  float fdb;      // feedback value
  float err[3];   // error and last error

  float p_out;
  float i_out;
  float d_out;
  float output;
	
	float delta_u;						//本次增量值
	float delta_out;					//本次增量式输出 = last_delta_out + delta_u
	float last_delta_out;
}pid_struct_t;

typedef struct
{
	pid_struct_t outer;
	pid_struct_t inner;
	float output;
}pid_Cascade_t;

void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max);

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;
							
  float pid_calc_cloud(pid_struct_t *pid, float ref, float fdb);            
float  pid_calc(pid_struct_t *pid, float ref, float fdb);
float pid_CascadeCalc(pid_Cascade_t *pid,float angleRef,float angleFdb,float speedFdb);
float pid_calc_3508(pid_struct_t *pid, float ref, float fdb);
int ComputeMinOffset(int target, int value); 
extern pid_Cascade_t     motor_pid_Cas_Yaw;
extern pid_Cascade_t     motor_pid_Cas_Picth;
extern pid_Cascade_t     motor_pid_Cas_bopan;
extern pid_struct_t      motor_pid_3508[2];
void motor_pid_init(void );


#define FollowYaw_pidInit       \
    {                           \
        0,                      \
            0,                  \
            0,                  \
            0,                  \
            0,                  \
            2000.0f,            \
            0.0f,               \
            0.0f,               \
            0,                  \
            0,                  \
            0,                  \
            0,                  \
            10000,              \
            0,                  \
            0,                  \
            &ClassisFollow_PID, \
    }


#endif

