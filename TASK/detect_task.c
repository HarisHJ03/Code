#include "detect_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "modeswitch_task.h"
#include "pc_rx_data.h"

RCC_ClocksTypeDef RCC_Clocks;

UBaseType_t detect_stack_surplus;

global_err_t global_err;

void detect_task(void *parm)
{
	uint32_t detect_wake_time = osKernelSysTick();
  while(1)
  {
    module_offline_detect();//模块离线检测
   // module_offline_callback();//发现模块离线后的处理方法，由我们自己定义
    
    RCC_GetClocksFreq(&RCC_Clocks);//检测外部晶振是否起振
    
    detect_stack_surplus = uxTaskGetStackHighWaterMark(NULL);    
    vTaskDelayUntil(&detect_wake_time, 50);
  }
}

err_status detect_id[ERROR_LIST_LENGTH];

void detect_param_init(void)
{
  for(uint8_t id = CHASSIS_M1_OFFLINE; id <= JUDGE_SYS_OFFLINE; id++)
  {
    global_err.list[id].param.set_timeout = 500;//允许中断通讯的最大时间
    global_err.list[id].param.last_times = 0;//记录上一次通讯的时间
    global_err.list[id].param.delta_times = 0;//记录距离上一次通讯的时间
    global_err.list[id].err_exist = 0;//规定时间内没有接收到数据该标志位就会置1
    global_err.err_now_id[id] = BOTTOM_DEVICE;//用ID的方式记录是哪个部位通讯出现问题
  }
  global_err.list[PC_SYS_OFFLINE].param.set_timeout = 1000;
  global_err.list[PC_SYS_OFFLINE].param.last_times = 0;
  global_err.list[PC_SYS_OFFLINE].param.delta_times = 0;
  global_err.list[PC_SYS_OFFLINE].err_exist = 0;
  global_err.err_now_id[PC_SYS_OFFLINE] = BOTTOM_DEVICE;
	
	global_err.list[IMU_OFFLINE].param.set_timeout = 200;
  global_err.list[IMU_OFFLINE].param.last_times = 0;
  global_err.list[IMU_OFFLINE].param.delta_times = 0;
  global_err.list[IMU_OFFLINE].err_exist = 0;
  global_err.err_now_id[IMU_OFFLINE] = BOTTOM_DEVICE;
}

//记录本次通讯的系统时间，用于比较通讯是否出现异常
void err_detector_hook(int err_id)
{
  global_err.list[err_id].param.last_times = HAL_GetTick();
}

//如果500ms还没有接收到数据（PC是1000ms,陀螺仪是200ms），就认为对应的设备出了问题
void module_offline_detect(void)
{
  for (uint8_t id = CHASSIS_M1_OFFLINE; id <= IMU_OFFLINE; id++)
  {
    global_err.list[id].param.delta_times = HAL_GetTick() - global_err.list[id].param.last_times;
    if(global_err.list[id].param.delta_times > global_err.list[id].param.set_timeout)
    {
      global_err.err_now_id[id] = (err_id)id;
      global_err.list[id].err_exist = 1;
    }
    else
    {
      global_err.err_now_id[id] = BOTTOM_DEVICE;
      global_err.list[id].err_exist = 0;
    }
  }
}

//通讯出现异常时的处理方法，IMU通过蜂鸣器提示，其他的设备通过板载LED灯提示
void module_offline_callback(void)
{
	static uint16_t buzz_flag = 0;
	static uint16_t buzz_time = 0;
  for(uint8_t id = CHASSIS_M1_OFFLINE; id <= IMU_OFFLINE; id++)
  {
    switch(global_err.err_now_id[id])
    {
      case CHASSIS_M1_OFFLINE:
      {
        GPIO_ResetBits(GPIOG,GPIO_Pin_1);
      }break;
      case CHASSIS_M2_OFFLINE:
      {
        GPIO_ResetBits(GPIOG,GPIO_Pin_2);
      }break;       
      case CHASSIS_M3_OFFLINE:
      {
        GPIO_ResetBits(GPIOG,GPIO_Pin_3);
      }break;
      case CHASSIS_M4_OFFLINE:
      {
        GPIO_ResetBits(GPIOG,GPIO_Pin_4);
      }break;        
      case GIMBAL_YAW_OFFLINE:
      {
				GPIO_ResetBits(GPIOG,GPIO_Pin_5);
      }break;        
      case GIMBAL_PIT_OFFLINE:
      {
				GPIO_ResetBits(GPIOG,GPIO_Pin_6);
      }break;       
      case TRIGGER_MOTO_OFFLINE:
      {
        GPIO_ResetBits(GPIOG,GPIO_Pin_7);
      }break;       
      case FRI_MOTO1_OFFLINE:
      {
        /*用户处理代码*/
      }break;       
      case FRI_MOTO2_OFFLINE:
      {
        /*用户处理代码*/
      }break;       
      case REMOTE_CTRL_OFFLINE:
      {
				GPIO_ResetBits(GPIOG,GPIO_Pin_8);//遥控离线的处理放在了modeswitch任务中
      }break;        
      case JUDGE_SYS_OFFLINE:
      {
        
      }break;        
      case PC_SYS_OFFLINE:
      {
        pc_recv_mesg.mode_Union.info.visual_valid = 0;//通讯意外中断时防止云台乱转
				pc_recv_mesg.mode_Union.info.navigation_determine = 0;
      }break;
			case IMU_OFFLINE:
			{	//IMU无法读取数据时，开发板蜂鸣器每秒叫一下
				switch(buzz_flag)
				{
					case 0:
						buzz_time = HAL_GetTick();
						buzz_flag = 1;
						TIM12->CCR1 = 50;
						break;
					case 1:
						if((HAL_GetTick() - buzz_time) > 1000)
						{
							TIM12->CCR1 = 0;
							buzz_time = HAL_GetTick();
							buzz_flag = 2;
						}break;
					case 2:
						if((HAL_GetTick() - buzz_time) > 1000)
						{
							buzz_flag = 0;
						}break;
				}
			}break;
      default:
      {
        GPIO_SetBits(GPIOG,GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8);
				TIM12->CCR1 = 0;
      }break;
    }
  }
}

uint8_t gimbal_is_controllable(void)
{
  if (gimbal_mode == GIMBAL_RELEASE
   || global_err.list[REMOTE_CTRL_OFFLINE].err_exist
   || global_err.list[GIMBAL_YAW_OFFLINE].err_exist
   || global_err.list[GIMBAL_PIT_OFFLINE].err_exist)
    return 0;
  else
    return 1;
}

uint8_t chassis_is_controllable(void)
{
  if (chassis_mode == CHASSIS_RELEASE 
   || global_err.list[REMOTE_CTRL_OFFLINE].err_exist)
    return 0;
  else
    return 1;
}

uint8_t shoot_is_controllable(void)
{
  if (shoot_mode == SHOOT_DISABLE 
   || global_err.list[REMOTE_CTRL_OFFLINE].err_exist)
    return 0;
  else
    return 1;
}
