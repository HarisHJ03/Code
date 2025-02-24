#include "info_get_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "comm_task.h"
#include "keyboard.h"
#include "remote_ctrl.h"
#include "bsp_can.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "modeswitch_task.h"
#include "imu_task.h"
#include "sys_config.h"
#include "pc_rx_data.h"
#include "stdlib.h"
#include "math.h"
#include "bsp_flash.h"

UBaseType_t info_stack_surplus;

extern TaskHandle_t can_msg_send_Task_Handle;
extern TaskHandle_t gimbal_Task_Handle;
extern TaskHandle_t chassis_Task_Handle;
extern TaskHandle_t shoot_Task_Handle;
extern TaskHandle_t shoot2_Task_Handle; //---------------3/16新增

extern cali_sys_t cali_param;
extern int direction;
extern int direction_change;

int  Navigation_Gimbal_yaw_offset;//----------------- 一上电就将flash中的值和现在的motor编码值作差

int test;
void info_get_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;
  while(1)
  {
    STAUS = xTaskNotifyWait((uint32_t) NULL, 
										        (uint32_t) MODE_SWITCH_INFO_SIGNAL,               //等待模式转换后的通知
									        	(uint32_t *)&Signal, 
									        	(TickType_t) portMAX_DELAY );
		if(STAUS == pdTRUE)
		{
			if(Signal & MODE_SWITCH_INFO_SIGNAL)
			{
        taskENTER_CRITICAL();//操作系统的临界保护函数，防止下面的操作被任务切换和中断所打断
        
        keyboard_global_hook();
        get_chassis_info();
        get_gimbal_info();
        get_shoot_info();
        get_global_last_info();
                
        taskEXIT_CRITICAL();//退出临界保护
        if(global_mode == RELEASE_CTRL)//当遥控右边打到最下时，电机全部断电
        {
				direction=1;
				gimbal.pit_center_offset = cali_param.pitch_offset;
				gimbal.yaw_center_offset = cali_param.yaw_offset;

          MEMSET(&glb_cur,motor_current_t);
          xTaskGenericNotify( (TaskHandle_t) can_msg_send_Task_Handle, 
                              (uint32_t) MODE_SWITCH_MSG_SIGNAL, 
                              (eNotifyAction) eSetBits, 
                              (uint32_t *)NULL );
        } 
        else//正常执行云台底盘发射这4个任务
        {
          xTaskGenericNotify( (TaskHandle_t) gimbal_Task_Handle, 
                            (uint32_t) INFO_GET_GIMBAL_SIGNAL, 
                            (eNotifyAction) eSetBits, 
                            (uint32_t *)NULL );
          xTaskGenericNotify( (TaskHandle_t) chassis_Task_Handle, 
                            (uint32_t) INFO_GET_CHASSIS_SIGNAL, 
                            (eNotifyAction) eSetBits, 
                            (uint32_t *)NULL );
          xTaskGenericNotify( (TaskHandle_t) shoot_Task_Handle, 
                            (uint32_t) INFO_GET_SHOOT_SIGNAL, 
                            (eNotifyAction) eSetBits, 
                            (uint32_t *)NULL );
					 xTaskGenericNotify( (TaskHandle_t) shoot2_Task_Handle,           //--------------新加
                            (uint32_t) INFO_GET_SHOOT2_SIGNAL, 
                            (eNotifyAction) eSetBits, 
                            (uint32_t *)NULL );
        }
        
        info_stack_surplus = uxTaskGetStackHighWaterMark(NULL);       
      }
    }

  }
}

static void get_chassis_info(void)
{
  /* 得到底盘轮速 */
  for (uint8_t i = 0; i < 4; i++)
  {
     chassis.wheel_spd_fdb[i] = moto_chassis[i].speed_rpm;
  }
  /* get remote and keyboard chassis control information */
  keyboard_chassis_hook();
  remote_ctrl_chassis_hook();
}
 int debug_yaw_spd = 1;
int debug_pit_spd = -1;//左上为正是+号。左下为正为-号

static void get_gimbal_info(void)
{
  /* 转换成相对角度 */
  static float yaw_ecd_ratio = YAW_MOTO_POSITIVE_DIR*YAW_DECELE_RATIO/ENCODER_ANGLE_RATIO;
  static float pit_ecd_ratio = PIT_MOTO_POSITIVE_DIR*PIT_DECELE_RATIO/ENCODER_ANGLE_RATIO;
	
	static int i=0;
		if(direction_change==1)
    {//检测到换头指令
		i++;
		if(i==1)
    {
		gimbal.pid.yaw_angle_ref += 180;	
		if(gimbal.yaw_center_offset>4096)
		gimbal.yaw_center_offset-=4096;
		else
		gimbal.yaw_center_offset+=4096;
	}
		i=1;
	}else
		i=0;
	//计算相对角度
    /*根据航向角的编码器值 moto_yaw.ecd 和航向角的中心偏移值 gimbal.yaw_center_offset，
    调用 get_relative_pos 函数来获取航向角的相对位置。而yaw_ecd_ratio是相对位置（编码器值）
    与角度之间的转换比率，为每编码值所对应的角度，其乘以相对位置（编码值）即为相对角度*/
		Navigation_Gimbal_yaw_offset = moto_yaw.ecd - gimbal.yaw_center_offset;//算出导航时的角度与初始归中值的角度差
//		Navigation_Gimbal_yaw_offset = get_relative_pos(moto_yaw.ecd, gimbal.yaw_center_offset);
	
 	//让底盘和云台分离
	if(( gimbal_mode ==GIMBAL_NAVIGATION_MODE && last_gimbal_mode!=GIMBAL_NAVIGATION_MODE )//  
			|| pc_recv_mesg.mode_Union.info.navigation_determine == 1)
	{
				
			 gimbal.sensor.yaw_relative_angle = yaw_ecd_ratio*get_relative_pos(moto_yaw.ecd - Navigation_Gimbal_yaw_offset, gimbal.yaw_center_offset);
    /*偏航角同理*/
			gimbal.sensor.pit_relative_angle = pit_ecd_ratio*get_relative_pos(moto_pit.ecd, gimbal.pit_center_offset);

	}
	else{
	  gimbal.sensor.yaw_relative_angle = yaw_ecd_ratio*get_relative_pos(moto_yaw.ecd, gimbal.yaw_center_offset);
    /*偏航角同理*/
  gimbal.sensor.pit_relative_angle = pit_ecd_ratio*get_relative_pos(moto_pit.ecd, gimbal.pit_center_offset);
      
	}
                                                                        
	 /* get gimbal relative palstance */
  //the Z axis(yaw) of gimbal coordinate system corresponds to the IMU Z axis
//  gimbal.sensor.yaw_palstance = yaw_spd * mpu_data.yaw_spd; // 16.384f; //unit: dps
//  //the Y axis(pitch) of gimbal coordinate sys em corresponds to the IMU -Y axis
//  gimbal.sensor.pit_palstance = pit_spd * mpu_data.pit_spd;
  /*获取遥控和键鼠的数据*/
  keyboard_gimbal_hook();
  remote_ctrl_gimbal_hook();   //*一个参数 权重  GIMBAL_RC_MOVE_RATIO_PIT
  
}


static int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset)
{
  int16_t tmp = 0;
  if (center_offset >= 4096)						//分中轴线象限
  {
		if (raw_ecd - center_offset > - 4096)
      tmp = raw_ecd - center_offset;							//在右边 = 负数
    else
      tmp = raw_ecd + 8192 - center_offset;				//在左边 = 正数
  }
  else
  {
		if (raw_ecd - center_offset > 4096)
      tmp = raw_ecd - 8192 - center_offset;				//在右边 = 负数
    else
      tmp = raw_ecd - center_offset;							//在左边 = 正数
  }
  return tmp;
}
static void get_shoot_info(void)
{
  remote_ctrl_shoot_hook();                             //遥控控制射击命令 里面已经写了控制两个枪管  -----------3/16
  keyboard_shoot_hook();                                //键盘控制射击命令 里面已经写了控制两个枪管  -----------3/16
	shoot.para_mode					= SHOOTNOR_MODE; 
	shoot2.para_mode					= SHOOTNOR_MODE;                 //射击正常模式

  
}

static void get_global_last_info(void)
{
  glb_sw.last_sw1 = rc.sw1;
  glb_sw.last_sw2 = rc.sw2; 
	glb_sw.last_iw = rc.iw;
}
