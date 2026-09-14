#include "pid.h"
#include "iir_filter.h"
#include "bsp.can.h"
#include <stdio.h>
/**
  * @brief  init pid parameter
  * @param  pid struct
    @param  parameter
  * @retval None
  */
	float yaw_target_filter[3] = {0.0f};
	float yaw_speed_filter[3] = {0.0f};
	pid_Cascade_t motor_pid_Cas_Yaw;//YAW轴双环PID
	pid_struct_t   motor_pid_3508[2];
	pid_Cascade_t  motor_pid_Cas_Picth;
	pid_Cascade_t   motor_pid_Cas_bopan;
	#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
				else                  \
				{                      \
					input = input;        \
				}                  \
    }        \

		
int ComputeMinOffset(int target, int value) 
{
    int err = target - value;
	
    if (err > 4096)
    {
        err -= 8191;
    }
    else if (err < -4096)
    {
        err += 8191;
    }
    return err;
}		
		


void motor_pid_init(void ){
	//云台YAW
	pid_init(&motor_pid_Cas_Yaw.outer,20.0f,0,100,29999,29999);
	pid_init(&motor_pid_Cas_Yaw.inner,200.0f,0.0f,250.0f,29999,29999);
	
//	pid_init(&motor_pid_Cas_Yaw.outer,1000.0f,0.0f,0,5000, 29999);
//	pid_init(&motor_pid_Cas_Yaw.inner,80.0f,80.0f,100.0f,5000, 29999);
	
	//摩擦轮
	pid_init (&motor_pid_3508[0],10.0f,0.0f,0,3000, 15000);
	pid_init (&motor_pid_3508[1],10.0f,0.0f,0,3000, 15000);
	
	//拨盘
	pid_init(&motor_pid_Cas_bopan.outer,1.2f,0.0f,0,3000, 15000);
	pid_init(&motor_pid_Cas_bopan.inner,10.0f,0.0f,0,3000, 15000);
	//Picth轴pid
	pid_init (&motor_pid_Cas_Picth.outer ,0.15f,0.0f,0,5000, 29999);
	pid_init (&motor_pid_Cas_Picth.inner ,80.0f,80.0f,100.0f,5000, 29999);
	
}

void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max)
{
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->i_max   = i_max;
  pid->out_max = out_max;
}

	float pid_calc(pid_struct_t *pid, float ref, float fdb)//位置式
                                     //目标值      反馈值
{
  pid->ref = ref;
  pid->fdb = fdb;
  pid->err[1] = pid->err[0];
  pid->err[0] = pid->ref - pid->fdb;
	
//if(pid->ref - pid->fdb>4096)
//{ pid->err[0]=  pid->ref - pid->fdb-8192;}
//if(pid->ref - pid->fdb< -4096)	
//{ 
//	pid->err[0] =8192+pid->ref - pid->fdb;
//}//过零处理
	
	

  pid->p_out  = pid->kp * pid->err[0];//比例部分
  pid->i_out += pid->ki * pid->err[0];//积分部分
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);//微分部分

   LimitMax(pid->i_out, pid->i_max );
  pid->output = pid->p_out + pid->i_out + pid->d_out;//控制量
  LimitMax(pid->output ,pid->out_max);

  return pid->output;
}





float pid_calc_cloud(pid_struct_t *pid, float ref, float fdb)//位置式
{
  pid->ref = ref;
  pid->fdb = fdb;

	pid->err[0] = ComputeMinOffset(pid->ref,pid->fdb);//过零处理
  pid->p_out  = pid->kp * pid->err[0];//比例部分
  pid->i_out += pid->ki * pid->err[0];//积分部分
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);//微分部分
//  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);//限幅
    if(pid->output > 29999)
	{
	pid->output = 29990;
	}
  if(pid->output < -29999)
	{
	pid->output = -29990;
	}
  pid->output = pid->p_out + pid->i_out + pid->d_out;//控制量
//  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
	  if(pid->output > 29999)
	{
	pid->output = 29990;
	}
  if(pid->output < -29999)
	{
	pid->output = -29990;
	}
	pid->err[1] = pid->err[0];
  return pid->output;
}




//float pid_CascadeCalc(pid_Cascade_t *pid,float angleRef,float angleFdb,float speedFdb)
//{//外层控制器输出控制量作为内层控制器的参考值进行计算
//	float tar_yaw_outer_angle;
//	float_Batwolf(&Gimbal_motor_info[0].target_rotor_angle,&tar_yaw_outer_angle,yaw_target_filter);
//	pid_calc_cloud(&pid->outer,angleRef,angleFdb);//外层角度
//	
//	
//	float tar_yaw_inner_speed;
//	float_Batwolf(&pid->outer.output,&tar_yaw_inner_speed,yaw_speed_filter);
//	pid_calc(&pid->inner,pid->outer.output,speedFdb);//内层速度
//	pid->output=pid->inner.output;//控制量为内层控制器的输出
//	return pid->output;
//	
//}

float pid_CascadeCalc(pid_Cascade_t *pid,float angleRef,float angleFdb,float speedFdb)
{
	pid_calc(&pid->outer,angleRef,angleFdb);
	pid_calc_3508(&pid->inner,pid->outer.output,speedFdb);
	pid->output=pid->inner.output;
	return pid->output;
}





float pid_calc_3508(pid_struct_t *pid, float ref, float fdb)//位置式
                                     //目标值      反馈值
{
  pid->ref = ref;
  pid->fdb = fdb;
  pid->err[1] = pid->err[0];
  pid->err[0] = pid->ref - pid->fdb;
  pid->p_out  = pid->kp * pid->err[0];//比例部分
  pid->i_out += pid->ki * pid->err[0];//积分部分
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);//微分部分

  LimitMax(pid->i_out, pid->i_max );

  pid->output = pid->p_out + pid->i_out + pid->d_out;//控制量

 LimitMax(pid->output ,pid->out_max);
  return pid->output;
}






void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}




