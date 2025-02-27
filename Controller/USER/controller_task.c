#include "controller_task.h"

extern IMU_Data_t BMI088;

Controller controller={0};
void controller_task(void const *argu) // ???????
{

	while (1)
	{
//		taskENTER_CRITICAL();
		// for (size_t i = 0; i < 3; i++)
		// {
		// 	if (i == 2)
		// 		controller.now_speed[i] = controller.last_speed[i] + (BMI088.Accel[i]-9.7f) * 0.001f;
		// 	else
		// 		controller.now_speed[i] = controller.last_speed[i] + BMI088.Accel[i] * 0.001f;
			
		// 	controller.last_speed[i] = controller.now_speed[i];
		// 	controller.clc_total_distance[i] = controller.now_speed[i] * 0.001f + 0.5f * BMI088.Accel[i] * 0.001f * 0.001f;
		// }
//		taskEXIT_CRITICAL();
	}
}
