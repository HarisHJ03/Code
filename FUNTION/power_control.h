#ifndef __POWER_CONTROL_H__
#define __POWER_CONTROL_H__

#include "stm32f4xx.h"
#include "chassis_task.h"

#define Debug_Power				40//没有裁判系统时的功率限制
#define Deta_Power				30//触发缓启动的功率差

extern float Charge_factor;


float Chassis_Power_Control(chassis_t *chassis_power_control);


#endif 
