#include "user_can.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"




//CAN发送数据帧函数
//参数1：选择要发送的CAN
//参数2：要发送数据帧的 ID类型
//参数3：要发送的数据帧 ID
//参数4：要发送的数据 数组

uint8_t CAN_Senddata(CAN_HandleTypeDef *hcan,uint16_t ID,uint8_t* pData,uint16_t Len)
{
		static CAN_TxHeaderTypeDef Tx_Header;
	uint32_t used_mailbox;
	/* Check the parameters */
	assert_param(hcan != NULL);

	Tx_Header.StdId = ID;
	Tx_Header.ExtId = ID;
	Tx_Header.IDE = 0;
	Tx_Header.RTR = 0;
	Tx_Header.DLC = Len;

	HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, &used_mailbox);

	return 1;
}
int16_t  							 tx_195[8];
uint8_t             CAN_keep[8];




void CAN_MOVE_0X195_setCurrent(CAN_HandleTypeDef hcanx,float iq1, float iq2, float iq3, float iq4, float iq5)
 {

 

 	tx_195[0] = (uint16_t)iq1 >> 8;
 	tx_195[1] = (uint16_t)iq1 ;
 	tx_195[2] = (uint16_t)iq2 >> 8;
 	tx_195[3] = (uint16_t)iq2;
 	tx_195[4] = (uint16_t)iq3 >> 8;
 	tx_195[5] = (uint16_t)iq3;
 	tx_195[6] = (uint16_t)iq4 >> 8;
 	tx_195[7] = (uint16_t)iq4;
	 
 for(int i=0;i<8;i++){
 	CAN_keep[i]= tx_195[i];
	
}


 CAN_Senddata(&hcan1, 0x195, 	CAN_keep, 8);
 } 