#include "Gimbal_task.h"
#include "cmsis_os.h"
#include "Gimbal_control.h"


void Gimbal_task(void const *pvParameters){
	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(1); 

	
//vTaskDelayUntil(&xLastWakeTime, TimeIncrement);	

	
for(;;){
		
				
	Gimbal_control();	
	
	vTaskDelayUntil(&xLastWakeTime, TimeIncrement);

}
		

}
