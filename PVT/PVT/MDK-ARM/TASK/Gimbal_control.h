#ifndef __GIMBAL_CONTROL_H
#define __GIMBAL_CONTROL_H
#include "pid.h"

#define Gimbal_mode          12   //只动云台
#define Chassis_follow_mode  11  //云台跟随底盘
#define dissable              9 //失能
#define Chassis_tuoluo       13 //小陀螺
#define fire_shoot           14 //发射模式
#define  picth_mode          15 //picth轴动
#define  pich_down_max     4500
#define  pich_up_max      2800

void read_imu_begin(void );
void GImbal_cloud(float yaw,float picth);
void Gimbal_control(void);
void Fire_shoot(void);

typedef struct
{
    int16_t  set_voltage;
		int16_t  set_current;
    uint16_t rotor_angle;
		float    target_rotor_angle;
    int16_t  rotor_speed;
		int16_t  speed_rpm;
    int16_t  torque_current;
    uint8_t  temp;
		uint16_t last_rotor_angle;
		int32_t  turn_count;
		float 	 total_angle;
		float 	 target_total_rotor_angle;
	
	  uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
} M6020s_t;





extern M6020s_t M6020s_Yaw;

#endif

