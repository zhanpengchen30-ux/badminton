#include "Gimbal_task.h"
#include "cmsis_os.h"
#include "Gimbal_control.h"
#include "bsp_usart.h"
#include "bsp.can.h"


void Gimbal_task(void const *pvParameters){
	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(1); 

	
//vTaskDelayUntil(&xLastWakeTime, TimeIncrement);	

	
	for(;;){
		if (RC_IsOnline()) {
			Gimbal_control();
		} else {
			/* 遥控器失联时覆盖旧云台控制路径的残留输出。 */
			set_M6020_yaw_voltage(0, 0, 0, 0);
			set_M6020_Picth_voltage(0, 0, 0, 0);
			set_motor_voltage_3508(0, 0, 0, 0, 0);
		}
		
		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);

}
		

}
