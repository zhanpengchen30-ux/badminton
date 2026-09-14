#ifndef USER_CAN_H
#define USER_CAN_H

#include "main.h"
#include "can.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "can.h"
//CAN数据队列句柄
extern QueueHandle_t CAN1_Queue;
extern QueueHandle_t CAN2_Queue;

typedef struct
{
	CAN_TxHeaderTypeDef CAN_TxMessage;				//CAN 发送缓冲区句柄
	uint8_t CAN_TxMessageData[8];				//存储CAN发送数据 数组
}CAN_Tx_Typedef;



typedef struct
{
//	uint8_t CANx;															//指定哪个CAN
	CAN_RxHeaderTypeDef CAN_RxMessage;				//CAN 接收缓冲区句柄
	uint8_t CAN_RxMessageData[1];				//存储CAN接收数据 数组
}CAN_Rx_TypeDef;


uint8_t CAN_Senddata(CAN_HandleTypeDef *hcan,uint16_t ID,uint8_t* pData,uint16_t Len);

 void CAN_MOVE_0X195_setCurrent(CAN_HandleTypeDef hcanx,float iq1, float iq2, float iq3, float iq4, float iq5);
void can_filter_init(void);

#endif

