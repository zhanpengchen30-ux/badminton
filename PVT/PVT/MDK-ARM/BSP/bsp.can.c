#include "bsp.can.h"
#include "main.h"

moto_info_t Gimbal_motor_info[Gimbal_MOTOR_MAX_NUM];
moto_info_t Gimbal_motor_info_3508[2];
moto_info_t Gimbal_motor_info_2006;//拨盘
DR16_t DR16;//遥控器
DR16_Export_Data_t DR16_Export_Data;
uint8_t             tx_data_6020[8];



//union Flag flag;
//Flags_t flags;

//CAN_RxHeaderTypeDef rx_header_6020;
uint16_t can_cnt;
uint16_t can_cnt1;
int16_t  rx_data_3508[8];

void get_total_angle(moto_measure_t *p);
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan);
/**
  * @brief  init can filter, start can, enable can rx interrupt
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */


/**
  * @brief  CAN1??????
  */
void CAN_1_Filter_Config(void)
{
  CAN_FilterTypeDef  sFilterConfig;
    
  sFilterConfig.FilterBank = 0;                       //CAN?????,??0-27
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;   //CAN?????,?????????
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  //CAN?????,16??32?
  sFilterConfig.FilterIdHigh = 0x0000;			//32??,?????ID??16?
  sFilterConfig.FilterIdLow = 0x0000;					//32??,?????ID??16?
  sFilterConfig.FilterMaskIdHigh = 0x0000;			//?????,??????
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;				//???????????,?????FIFO
  sFilterConfig.FilterActivation = ENABLE;    		//?????
  sFilterConfig.SlaveStartFilterBank = 14;

	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	HAL_CAN_Start(&hcan1) ;
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
  * @brief  CAN2??????
  */

  uint8_t      rx_data_CAN1[8];	
  uint8_t      rx_data_CAN2[8];	
  uint8_t      DATE[8];	

 void CAN_2_Filter_Config(void)
{
  CAN_FilterTypeDef  sFilterConfig;
    
  sFilterConfig.FilterBank = 14;                       //CAN?????,??0-27
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;   //CAN?????,?????????
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  //CAN?????,16??32?
  sFilterConfig.FilterIdHigh = 0x0000;			//32??,?????ID??16?
  sFilterConfig.FilterIdLow = 0x0000;					//32??,?????ID??16?
  sFilterConfig.FilterMaskIdHigh = 0x0000;			//?????,??????
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;				//???????????,?????FIFO
  sFilterConfig.FilterActivation = ENABLE;    		//?????
	
	HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
	HAL_CAN_Start(&hcan2) ;
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
  * @brief  can rx callback, get motor feedback info
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//CAN总线接收回调函数
//{
//  CAN_RxHeaderTypeDef rx_header;//用来存储接收到的CAN帧的头部信息，包括帧ID、帧类型、帧长度等
////  uint8_t             rx_data[8];
//	
//  
//	if(hcan->Instance == CAN2)
// 
//   { HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data_CAN2); 
//	 
//	 
//		  switch(rx_header.StdId){
//		 
//		 case 0x201: 
//		Gimbal_motor_info_3508[0].rotor_angle    = ((rx_data_CAN2[0] << 8) | rx_data_CAN2[1]);
//    Gimbal_motor_info_3508[0].rotor_speed   = ((rx_data_CAN2[2] << 8) | rx_data_CAN2[3]);
//    Gimbal_motor_info_3508[0].torque_current = ((rx_data_CAN2[4] << 8) | rx_data_CAN2[5]);
//    Gimbal_motor_info_3508[0].temp          =   ( (rx_data_CAN2[6] << 8) | rx_data_CAN2[7] );
//		 break;
//		 case 0x202:
//		Gimbal_motor_info_3508[1].rotor_angle    = ((rx_data_CAN2[0] << 8) | rx_data_CAN2[1]);
//    Gimbal_motor_info_3508[1].rotor_speed   = ((rx_data_CAN2[2] << 8) | rx_data_CAN2[3]);
//    Gimbal_motor_info_3508[1].torque_current = ((rx_data_CAN2[4] << 8) | rx_data_CAN2[5]);
//    Gimbal_motor_info_3508[1].temp          =   ( (rx_data_CAN2[6] << 8) | rx_data_CAN2[7] );
//		 break;
//		 case 0x203:
//		Gimbal_motor_info_2006.rotor_angle    = ((rx_data_CAN2[0] << 8) | rx_data_CAN2[1]);
//    Gimbal_motor_info_2006.rotor_speed   = ((rx_data_CAN2[2] << 8) | rx_data_CAN2[3]);
//    Gimbal_motor_info_2006.torque_current = ((rx_data_CAN2[4] << 8) | rx_data_CAN2[5]);
//    Gimbal_motor_info_2006.temp          =   ( (rx_data_CAN2[6] << 8) | rx_data_CAN2[7] );
//		 break;
//			}
//			
//	 
//	 
// }
//  
// 
//	 
//	
//	CAN_RxHeaderTypeDef rx_header_CAN1;
//	
//	if(hcan->Instance == CAN1)
//  {
//	 
//	
//	
//    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header_CAN1, rx_data_CAN1); 
//	
//	
//	 
//	 
//	 switch(rx_header_CAN1.StdId){
//		 
//		 case 0x207: 
//		Gimbal_motor_info[0].rotor_angle    = ((rx_data_CAN1[0] << 8) | rx_data_CAN1[1]);
//    Gimbal_motor_info[0].rotor_speed   = ((rx_data_CAN1[2] << 8) | rx_data_CAN1[3]);
//    Gimbal_motor_info[0].torque_current = ((rx_data_CAN1[4] << 8) | rx_data_CAN1[5]);
//    Gimbal_motor_info[0].temp          =   ( (rx_data_CAN1[6] << 8) | rx_data_CAN1[7] );//将CAN帧中的转子角度、转速、扭矩电流和温度等
//		 break;
//		 
//		 case 0x208 : 
//		Gimbal_motor_info[1].rotor_angle    = ((rx_data_CAN1[0] << 8) | rx_data_CAN1[1]);
//    Gimbal_motor_info[1].rotor_speed   = ((rx_data_CAN1[2] << 8) | rx_data_CAN1[3]);
//    Gimbal_motor_info[1].torque_current = ((rx_data_CAN1[4] << 8) | rx_data_CAN1[5]);
//    Gimbal_motor_info[1].temp          =  ( (rx_data_CAN1[6] << 8) | rx_data_CAN1[7] ) ;//将CAN帧中的转子角度、转速、扭矩电流和温度
//		 break;
//		case 0x196:
//		DR16.rc.ch0 = ((rx_data_CAN1[0] << 8) | rx_data_CAN1[1]);
//    DR16.rc.ch1 =((rx_data_CAN1[2] << 8) | rx_data_CAN1[3]);
//    DR16_Export_Data.chassis_mode=((rx_data_CAN1[4] << 8) | rx_data_CAN1[5]);
//		DR16.rc.ch4_DW= ( (rx_data_CAN1[6] << 8) | rx_data_CAN1[7] );
//		  break;
//		
////	case 0x180:
////		
////		memcpy(flag.date, rx_data_CAN1,sizeof(rx_data_CAN1));
////	
////	break;
//	
//			
//		
//		 
//		 
//	 }
// 
// }
//  if (can_cnt1 == 500)
//  {
//    can_cnt1 = 0;
//	}
//	
//	
//}




/**
  * @brief  send motor control message through can bus
  * @param  id_range to select can control id 0x1ff or 0x2ff
  * @param  motor voltage 1,2,3,4 or 5,6,7
  * @retval None
  */
void set_motor_voltage_3508(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)//形参表示4个电机的电压值
{
  CAN_TxHeaderTypeDef tx_header;//用来存储发送的CAN帧的头部信息，包括帧ID、帧类型、帧长度等
  uint8_t             tx_data[8];//用来存储接发送的CAN帧的数据部分
    
//  tx_header.StdId = (id_range == 0)?(0x1ff):(0x2ff);//如果id为0则将StdId设置为0x1ff，否则设置为0x2ff。
	tx_header.StdId = 0x200;
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;//定义发送格式

  tx_data[0] = (v1>>8)&0xff;//将v1的高8位存储到0
  tx_data[1] =    (v1)&0xff;//将v1的低8位存储到1
  tx_data[2] = (v2>>8)&0xff;
  tx_data[3] =    (v2)&0xff;
  tx_data[4] = (v3>>8)&0xff;
  tx_data[5] =    (v3)&0xff;
  tx_data[6] = (v4>>8)&0xff;
  tx_data[7] =    (v4)&0xff;//定义发送内容
	
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0); //函数会根据指定的邮箱号将CAN帧发送到对应的邮箱
												//CAN, 信息格式，  信息内容，  指向邮箱号的指针
}



void set_M6020_yaw_voltage(int16_t v1, int16_t v2, int16_t v3, int16_t v4)//发送至云台YAW轴电机
{
  CAN_TxHeaderTypeDef tx_header_6020;//用来存储发送的CAN帧的头部信息，包括帧ID、帧类型、帧长度等
  uint8_t             tx_data_6020[8];//用来存储接发送的CAN帧的数据部分
    
	tx_header_6020.StdId = 0x1ff;
  tx_header_6020.IDE   = CAN_ID_STD;
  tx_header_6020.RTR   = CAN_RTR_DATA;
  tx_header_6020.DLC   = 8;//定义发送格式

  tx_data_6020[0] = v1>>8;//将v1的高8位存储到0
  tx_data_6020[1] =    v1;//将v1的低8位存储到1
  tx_data_6020[2] = v2>>8;
  tx_data_6020[3] =    v2;
  tx_data_6020[4] = v3>>8;
  tx_data_6020[5] =    v3;
  tx_data_6020[6] = v4>>8;
  tx_data_6020[7] =    v4;//定义发送内容
	
  HAL_CAN_AddTxMessage(&hcan1, &tx_header_6020, tx_data_6020,(uint32_t*)CAN_TX_MAILBOX0); //函数会根据指定的邮箱号将CAN帧发送到对应的邮箱
												//CAN, 信息格式，  信息内容，  指向邮箱号的指针
}


void set_M6020_Picth_voltage(int16_t v1, int16_t v2, int16_t v3, int16_t v4)//发送至云台YAW轴电机
{
  CAN_TxHeaderTypeDef tx_header_6020;//用来存储发送的CAN帧的头部信息，包括帧ID、帧类型、帧长度等
  uint8_t             tx_data_6020[8];//用来存储接发送的CAN帧的数据部分
    
	tx_header_6020.StdId = 0x1ff;
  tx_header_6020.IDE   = CAN_ID_STD;
  tx_header_6020.RTR   = CAN_RTR_DATA;
  tx_header_6020.DLC   = 8;//定义发送格式

  tx_data_6020[0] = v1>>8;//将v1的高8位存储到0
  tx_data_6020[1] =    v1;//将v1的低8位存储到1
  tx_data_6020[2] = v2>>8;
  tx_data_6020[3] =    v2;
  tx_data_6020[4] = v3>>8;
  tx_data_6020[5] =    v3;
  tx_data_6020[6] = v4>>8;
  tx_data_6020[7] =    v4;//定义发送内容
	
  HAL_CAN_AddTxMessage(&hcan1, &tx_header_6020, tx_data_6020,(uint32_t*)CAN_TX_MAILBOX0); //函数会根据指定的邮箱号将CAN帧发送到对应的邮箱
												//CAN, 信息格式，  信息内容，  指向邮箱号的指针
}







