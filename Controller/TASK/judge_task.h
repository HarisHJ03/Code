#ifndef _judge_task_H
#define _judge_task_H


#include "stm32f4xx.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "comm_task.h"
#include "detect_task.h"
#include "data_packet.h"
#include "judge_rx_data.h"
#include "judge_tx_data.h"

#include "struct_typedef.h"

void judge_tx_task(void *parm);
void judge_rx_task(void *parm);

extern float send_data[6];

#endif

