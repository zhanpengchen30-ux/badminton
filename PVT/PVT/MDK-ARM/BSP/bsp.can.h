#ifndef __BSP_CAN
#define __BSP_CAN

#include "can.h"



#define Gimbal_ID_BASE      0x207
#define Gimbal_ID_BASE_2006      0x201
#define CAN_CONTROL_ID_BASE   0x1ff
#define CAN_CONTROL_ID_EXTEND 0x2ff
#define Chassis_MOTOR_MAX_NUM         4
#define Gimbal_MOTOR_MAX_NUM         2

typedef struct
{
    uint16_t can_id;
    int16_t  set_voltage;
		int16_t  set_current;
    uint16_t rotor_angle;
    int16_t  rotor_speed;
		int16_t  speed_rpm;
    int16_t  torque_current;
    uint8_t  temp;
	  float   target_rotor_angle;
	  int target_angle;
		int target_speed;
}moto_info_t;

typedef struct{
	int16_t	 	speed_rpm;
  int16_t  	real_current;
  int16_t  	given_current;
  uint8_t  	hall;
	uint16_t 	angle;				//abs angle range:[0,8191]
	uint16_t 	last_angle;	//abs angle range:[0,8191]
	uint16_t	offset_angle;
	int32_t		round_cnt;
	int32_t		total_angle;
	uint8_t		buf_idx;
	uint16_t	angle_buf[5];
	uint16_t	fited_angle;
	uint32_t	msg_cnt;
	
}moto_measure_t;

typedef struct
{
		uint64_t follow_sign :1 ;//跟随
    uint64_t spinning_sigh:1;//小陀螺
    uint64_t Locked_sigh:1;//锁车
		uint64_t	run_away_sign:1;//逃跑
		uint64_t fire_sign:1;//四面
		uint64_t auto_fire:1;//自瞄
		uint64_t win_fire:1;//单挑
		uint64_t super_spinning_sigh:1;//超级小陀螺
	  uint64_t :56;
} Flags_t;


union Flag
{
	uint8_t date[8];
	Flags_t Flags;

};


typedef struct
{
    struct
    {
        float x;
        float y;
    } mouse;

    struct
    {

        uint32_t Press_Flag;                //键鼠按下标志
        uint32_t Click_Press_Flag;          //键鼠单击标志
        uint32_t Long_Press_Flag;           //键鼠长按标志
        uint8_t PressTime[18]; //键鼠按下持续时间
    } KeyMouse;                             //鼠标的对外输出。

    struct
    {
        float Forward_Back_Value; //Vx
        float Omega_Value;        //自旋值。
        float Left_Right_Value;   //Vy
        float Pitch_Value;
        float Yaw_Value;
        float Dial_Wheel; //拨轮
    } Robot_TargetValue;  //遥控计算比例后的运动速度
    
    uint16_t infoUpdateFrame; //帧率
    uint8_t OffLineFlag; 		//设备离线标志
		int16_t chassis_mode;//地盘模式
		
} DR16_Export_Data_t;         //供其他文件使用的输出数据。




typedef struct
{
    uint8_t DR16Buffer[22];
    struct
    {
        int16_t ch0; //yaw
        int16_t ch1; //pitch
        int16_t ch2; //left_right
        int16_t ch3; //forward_back
        uint8_t s_left;
        uint8_t s_right;
        int16_t ch4_DW; //拨轮
    } rc;               //遥控器接收到的原始值。

    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;

        uint8_t keyLeft;
        uint8_t keyRight;
    } mouse;
} DR16_t;

extern     DR16_t DR16;
extern DR16_Export_Data_t DR16_Export_Data;


	
void set_M6020_yaw_voltage(int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void set_motor_voltage_3508(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void set_motor_voltage_Gimba(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4); //6020

//void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan);
void set_M6020_Picth_voltage(int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void CAN_1_Filter_Config(void);
void CAN_2_Filter_Config(void);
extern   moto_info_t Chassis_motor_info[Chassis_MOTOR_MAX_NUM];
extern   moto_info_t Gimbal_motor_info[Gimbal_MOTOR_MAX_NUM];
extern   moto_info_t Gimbal_motor_follw;
extern   moto_info_t Gimbal_motor_info_3508[2];
extern   moto_info_t Gimbal_motor_info_2006;
#endif
