#include "Gimbal_control.h"
#include "DJI_IMU.h"
#include "pid.h"
#include "bsp.can.h"
M6020s_t M6020s_Yaw;
float   M6020_picth;
uint8_t imu_begin_flag;
uint16_t follow;




void read_imu_begin(void){
	
float imu_begin_angle;
float imu_pitch_start_angle;
	
	
if(imu_begin_flag == 0)
{
imu_begin_angle = 22.7527f * DJI_C_IMU.total_yaw;
Gimbal_motor_info[0].target_rotor_angle   =imu_begin_angle;

imu_pitch_start_angle = 4096;
M6020_picth      = imu_pitch_start_angle;

imu_begin_flag = 1;

}
	
}

void GImbal_cloud(float yaw,float picth){
	
	
	read_imu_begin();
	
	Gimbal_motor_info[0].target_rotor_angle  += yaw;
	

	M6020_picth                    += picth;
	
	
	if(M6020_picth>pich_down_max){
	M6020_picth =	pich_down_max;
}
if(M6020_picth<pich_up_max)
{
		M6020_picth = pich_up_max; 
}



Gimbal_motor_info[0].set_voltage  = pid_CascadeCalc(&motor_pid_Cas_Yaw, 0.1f*  Gimbal_motor_info[0].target_rotor_angle  ,22.7527f * DJI_C_IMU.total_yaw,Gimbal_motor_info[0].rotor_speed);	
Gimbal_motor_info[1].set_voltage  = pid_CascadeCalc(&motor_pid_Cas_Picth,M6020_picth,22.7527f * DJI_C_IMU.pitch,Gimbal_motor_info[1].rotor_speed);
	
	
	
set_M6020_yaw_voltage(0,0,Gimbal_motor_info[0].set_voltage,0);
set_M6020_Picth_voltage(
	            0, 
							0, 
							0, 
							Gimbal_motor_info[1].set_voltage);

}
void Fire_shoot(void){//发射  左上右中 
	
	
	
for(int i=0;i<2;i++){
	
Gimbal_motor_info_3508[i].set_voltage=pid_calc_3508(&motor_pid_3508[i],DR16.rc.ch4_DW,Gimbal_motor_info_3508[i].rotor_speed); 

}

Gimbal_motor_info_2006.set_voltage=pid_CascadeCalc(&motor_pid_Cas_bopan,2000,Gimbal_motor_info_2006.rotor_angle,Gimbal_motor_info_2006.rotor_speed);



//set_motor_voltage_3508(0,Gimbal_motor_info_3508[0].set_voltage,Gimbal_motor_info_3508[1].set_voltage,Gimbal_motor_info_2006.set_voltage,0);

}









void Gimbal_control(void){
	
	
if(DR16_Export_Data.chassis_mode == Gimbal_mode  || DR16_Export_Data.chassis_mode== Chassis_follow_mode   ||  DR16_Export_Data.chassis_mode== Chassis_tuoluo     ){
	
	
	GImbal_cloud(-0.01f *DR16.rc.ch0,-0.005f *DR16.rc.ch1);
	
	
}
if(DR16_Export_Data.chassis_mode ==dissable){
	
	
	
imu_begin_flag = 0;
set_M6020_yaw_voltage(0,0,0,0);
set_M6020_Picth_voltage(0,0,0,0);
set_motor_voltage_3508(0,0,0,0,0);
	
}	

if(DR16_Export_Data.chassis_mode==fire_shoot){ //左上右中   发射
	
	
	Fire_shoot();
	GImbal_cloud(-0.0001f *DR16.rc.ch0,-1.0 *DR16.rc.ch1);
	
	
	
}

	

	
	
	

	
	
	
	
}

