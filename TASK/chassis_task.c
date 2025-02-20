#include "chassis_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "modeswitch_task.h"
#include "comm_task.h"
#include "gimbal_task.h"
#include "info_get_task.h"
#include "detect_task.h"
#include "pid.h"
#include "sys_config.h"
#include "stdlib.h"
// #include "math.h"
#include "pc_rx_data.h"
#include "remote_ctrl.h"
#include "keyboard.h"
#include "bsp_can.h"
#include "judge_rx_data.h"
#include "data_packet.h"
#include "arm_math.h"
#include "power_control.h"
float a1, b1, a2, b2, a3, b3, a4, b4;
float ob_total_power;
ramp_t chassis_v_ramp;
/*2023赛季规则

初始状态			100 40

功率优先
		1级		150 60
		2级		200 80
		3级		250 100
血量优先
		1级		200 45
		2级		300 50
		3级		400 55

*/

UBaseType_t chassis_stack_surplus;
extern TaskHandle_t can_msg_send_Task_Handle;

extern pid_t pid_chassis_vw;
extern pid_t pid_chassis_power_buffer;
extern float yaw_ecd_ratio;

float ramp_out ;

chassis_t chassis;

//extern robot_rx_data pc_recv_mesg;

// 前三个旋转pid 后三个速度pid

#if (INFANTRY_NUM == INFANTRY_3)

float chassis_pid[3] = {10.0f, 0.0, 0.0f};
float chassis_spd_pid[4][3] = {{10.5, 0.0f, 0},
							   {10.5, 0.0f, 0},
							   {10.5, 0.0f, 0},
							   {10.5, 0.0f, 0}};
//float chassis_pid[3] = {0.0f, 0.0, 0.0f};
//float chassis_spd_pid[4][3] = {{0, 0.0f, 0},
//							   {0, 0.0f, 0},
//							   {0, 0.0f, 0},
//							   {0, 0.0f, 0}};
////////////////////
#elif (INFANTRY_NUM == INFANTRY_4)
float chassis_pid[3] = {10.0f, 0.0, 0.0f};
float chassis_spd_pid[4][3] = {{10.5, 0.0f, 0},
							   {10.5, 0.0f, 0},
							   {10.5, 0.0f, 0},
							   {10.5, 0.0f, 0}};

#elif (INFANTRY_NUM == INFANTRY_5)
float chassis_pid[3] = {10.0f, 0.0, 0.0f};
//float chassis_spd_pid[4][3] = {{10.5, 0.0f, 0},\
//								{10, 0.0f, 0},\
//								{10, 0.0f, 0},\
//								{10, 0.0f, 0}};
float chassis_spd_pid[4][3] = {{10, 0.0f, 0},
							   {10, 0.0f, 0},
							   {10, 0.0f, 0},
							   {10, 0.0f, 0}};
#else
#error "INFANTRY_NUM define error!"
#endif

//#define POWER_NEW//新功率算法，若要用老功率算法就注释掉这一行
								 
float	power_new_pid[3] = {10.0f, 0.1f, 0.0f}; //新功率算法PID（代替下边的cap_vol_pid）{350.0f, 1.0f, 0.0f}
float cap_vol_pid[3] = {1.0f, 0, 0.0f};		   // 超级电容电压控制PID
float chassis_power_buffer_pid[3] = {1, 0, 0}; // 缓冲焦耳控制PID
#ifndef POWER_NEW
float chassis_vw_pid[3] = {0.1, 0, 0.1};	    // 小陀螺转速控制PID
#else
float chassis_vw_pid[3] = {0.0f, 0.3f, 0.0f};	   // 小陀螺转速控制PID
#endif

float cap_store = 24;		  // 电容存储大小(给定初值为24)
float cx = 0, cy = 0, cw = 0; // 缓启动赋值变量
uint8_t fast_flag = 0;		  // 加速状态标志位
int Speed_up = 0;			  // 加速键标志位


void chassis_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;

	while (1)
	{
		STAUS = xTaskNotifyWait((uint32_t)NULL,
								(uint32_t)INFO_GET_CHASSIS_SIGNAL,
								(uint32_t *)&Signal,
								(TickType_t)portMAX_DELAY);
		if (STAUS == pdTRUE)
		{

			if (Signal & INFO_GET_CHASSIS_SIGNAL)
			{
				/*底盘vw旋转的pid*/
				PID_Struct_Init(&pid_chassis_angle, chassis_pid[0], chassis_pid[1], chassis_pid[2], MAX_CHASSIS_VR_SPEED, 50, DONE);
				PID_Struct_Init(&pid_chassis_power_buffer, chassis_power_buffer_pid[0], chassis_power_buffer_pid[1], chassis_power_buffer_pid[2], 50, 10, DONE);
				PID_Struct_Init(&pid_chassis_vw, chassis_vw_pid[0], chassis_vw_pid[1], chassis_vw_pid[2], 600, 600, DONE);

				/*底盘vx,vy平移的pid*/
				for (int i = 0; i < 4; i++)
					PID_Struct_Init(&pid_spd[i], chassis_spd_pid[i][0], chassis_spd_pid[i][1], chassis_spd_pid[i][2], 16000, 1500, DONE);

				if (chassis_mode != CHASSIS_RELEASE ) // 云台归中之后底盘才能动 //&& gimbal.state != GIMBAL_INIT_NEVER
				{
					switch (chassis_mode)
					{
					/*云台底盘跟随模式*/
					case CHASSIS_NORMAL_MODE:
					{
						chassis_normal_handler();
					}
					break;
					/*小陀螺模式*/
					case CHASSIS_DODGE_MODE:
					{
						chassis_dodge_handler();
					}
					break;
					/*底盘停止模式*/
					case CHASSIS_STOP_MODE:
					{
						chassis_stop_handler();
					}
					break;
					case CHASSIS_NAVIGATION_MODE:
					{
						chassis_navigation_handler();
					}
					break;

					default:
					{
					}
					break;
					}
					#ifndef POWER_NEW
					if (FAST_SPD && fast_flag)
					{
						cx = chassis.vx;
						cy = chassis.vy;
						cw = chassis.vw;
						
					}
					else
					{
						// 缓启动
						buffer(&cx, chassis.vx, 20, 50); 						
						buffer(&cy, chassis.vy, 20, 50);
						buffer(&cw, chassis.vw, 1, 1);
					}
					
					#else
						cx = chassis.vx;
						cy = chassis.vy;
						cw = chassis.vw;
					#endif					


					   VAL_LIMIT(cx, -2000, 2000);
								
								
          //ramp_out =  ramp_calc(&chassis_v_ramp);
					//mecanum_calc(cx, chassis.vy, chassis.vw, chassis.wheel_spd_ref);
					   if(rc.ch2 == 0 && rc.ch1 == 0) cx = 0;
					 mecanum_calc(cx,cy , chassis.vw, chassis.wheel_spd_ref);
					
					  
					
					// 将缓启动的数据代入解算（前后缓启动，左右以及小陀螺不进行缓启动）
					

					for (int i = 0; i < 4; i++)
						chassis.current[i] = pid_calc(&pid_spd[i], chassis.wheel_spd_fdb[i], chassis.wheel_spd_ref[i]);

					ob_total_power = Chassis_Power_Control(&chassis); // 功率控制

					if (!chassis_is_controllable())
						memset(chassis.current, 0, sizeof(chassis.current));

					  memcpy(glb_cur.chassis_cur, chassis.current, sizeof(chassis.current));
				}
				else
					memset(glb_cur.chassis_cur, 0, sizeof(glb_cur.chassis_cur));
				xTaskGenericNotify((TaskHandle_t)can_msg_send_Task_Handle,
								   (uint32_t)CHASSIS_MOTOR_MSG_SIGNAL,
								   (eNotifyAction)eSetBits,
								   (uint32_t *)NULL);
			}
		}

		chassis_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
	}
}

void Cap_refresh()
{
	// 不断刷新并记录当前电池所能给电容充的最大电压
	// 电容的最大压值随着电池格数进行变化，因而需要不断刷新
	static int i = 0;
	if (judge_recv_mesg.power_heat_data.buffer_energy == 60 && (chassis.CapData[0] - chassis.CapData[1] < 2 || chassis.CapData[1] >= 20))
	{
		if (i != 10)
			i++;
	}
	else
		i = 0;
	if (i == 10)
		cap_store = chassis.CapData[1];
	if (chassis.CapData[1] > cap_store)
		cap_store = chassis.CapData[1];
	if (cap_store < 20.f)
		cap_store = 20;
};

extern int Speed_up;
#ifndef POWER_NEW
//老功率算法--电容pid
// 底盘功率控制函数，通过电容总值，电容当前值，底盘功率限制值，计算出底盘x、y轴上的速度
void chassis_power_contorl(pid_t *power_pid, float *power_vx, float *power_vy, float *power_yaw_speed, float real_time_Cap_remain, float real_time_Cap_can_store, float judge_power_limit)
{
	static float max = 2000;
	static float min = 1200;						  // 1000;
	static float parameter;							  // 变速系数
	static float Cap_low = 14.0f;					  // 何时停止加速
	float cap_flag = real_time_Cap_can_store - 18.0f; // 18~real_time_Cap_can_store这一段进行按比例赋值
	if (real_time_Cap_remain < 15.0f)
	{
		*power_yaw_speed = (5 - (15.0f - real_time_Cap_remain)) / 5 * 0.01f;
	} // 低电容按比例减少鼠标左右移动速度
	else
	{
		*power_yaw_speed = 0.01f;
	}

	if (real_time_Cap_remain > 18.0f && real_time_Cap_remain < real_time_Cap_can_store) // 当前电容在18和满电容之间
	{																					// 核心   通过计算得出底盘x轴或y轴的最小速度
		min = 1350 + (cap_flag - (real_time_Cap_can_store - real_time_Cap_remain)) / cap_flag * (judge_power_limit - 45) / 45 * 500;
	}
	else
	{				// 当电容小于18时直接用这个
		min = 1150; // 1000;
	}

	// 按加速键后Speed_up = 0,以3508的最快速度进行冲刺，Speed_up = 1即未按加速键
	if (Speed_up == 1)
	{
		if (real_time_Cap_remain == real_time_Cap_can_store) // 电容充满
		{
			pid_calc(power_pid, real_time_Cap_remain + 0.5f, real_time_Cap_can_store);
		}
		else
		{
			if (real_time_Cap_remain != 0)
				pid_calc(power_pid, real_time_Cap_remain, real_time_Cap_can_store);
		}
	}
	else if (real_time_Cap_remain < Cap_low)
		Speed_up = 1; // 一旦检测到电容低则自动关闭加速

	if (km.vy > 0 || rm.vy > 0)
	{
		if (Speed_up == 0)
		{
			*power_vy = CHASSIS_KB_MAX_SPEED_Y;
		}
		else
		{
			if (*power_vy < 0)
			{
				*power_vy = -*power_vy;
			}
			*power_vy -= parameter * power_pid->out;
			VAL_LIMIT(*power_vy, min , max); // 低电容平移，速度下限更低
		}
	}
	else
	{
		if (km.vy < 0 || rm.vy < 0)
		{
			if (Speed_up == 0)
				*power_vy = -CHASSIS_KB_MAX_SPEED_Y;
			else
			{
				if (*power_vy > 0)
				{
					*power_vy = -*power_vy;
				}
				*power_vy += parameter * power_pid->out;
				VAL_LIMIT(*power_vy, -max, -min );
			}
		}
		else
			*power_vy = 0;
	}

	if (km.vx > 0 || rm.vx > 0)
	{
		if (Speed_up == 0)
		{
			*power_vx = CHASSIS_KB_MAX_SPEED_X;
		} 
		else
		{
			if (*power_vx < 0)
			{
				*power_vx = -*power_vx;
			}
			*power_vx -= parameter * power_pid->out;
			VAL_LIMIT(*power_vx, min, max);
		}
	}
	else
	{
		if (km.vx < 0 || rm.vx < 0)
		{
			if (Speed_up == 0)
				*power_vx = -CHASSIS_KB_MAX_SPEED_X;
			else
			{
				if (*power_vx > 0)
				{
					*power_vx = -*power_vx;
				}
				*power_vx += parameter * power_pid->out;
				VAL_LIMIT(*power_vx, -max, -min);
			}
		}
		else
		{
			*power_vx = 0;
		}
	}

	if (SLOW_SPD) // 慢速
	{
		if (*power_vx > 0)
			*power_vx = 1000;
		else if (*power_vx < 0)
			*power_vx = -1000;
		
		if (*power_vy > 0)
			*power_vy = 1000;
		else if (*power_vy < 0)
			*power_vy = -1000;
	}
	else if (fast_flag && (rc.ch2 == 660 || FAST_SPD)) // 快速
	{
		Speed_up = 0;
	}
	else // 正常速度
	{
		Speed_up = 1;
	}

	if (real_time_Cap_remain > Cap_low && !FAST_SPD)
	{
		fast_flag = 1;
	} // 防止加速过程中反复开关加速模式（抽搐）
	if (real_time_Cap_remain < Cap_low)
	{
		fast_flag = 0;
	}
}

//导航模式
static void chassis_navigation_handler()
{
	 float nor_chassis_vx,nor_chassis_vy;
		//计算底盘x、y轴上的速度
		chassis_power_contorl(&pid_power,&nor_chassis_vx,&nor_chassis_vy,&yaw_speed,chassis.CapData[1],cap_store,(float)judge_recv_mesg.game_robot_state.chassis_power_limit);
		int position_ref = 0;
		chassis.vx=nor_chassis_vx;
		chassis.vy=nor_chassis_vy;
		if(pc_recv_mesg.mode_Union.info.navigation_determine == 1 ){
			chassis.vx = 1000 * -pc_recv_mesg.linear_speed_x;
			chassis.vy = 1000 * -pc_recv_mesg.linear_speed_y;}
		else {chassis.vx = 0; chassis.vy = 0;}
		if (input_flag == 2)
			{
		  /*		
 			  chassis.pid_yaw_angle_ref = rm.chassis_yaw_v * GIMBAL_RC_MOVE_RATIO_YAW*50;
        chassis.yaw_navigation_relative_angle = get_chassis_relative_pos(chassis.pid_yaw_angle_ref, gimbal.yaw_center_offset); 
				chassis.vw = (-pid_calc(&pid_chassis_angle,chassis.pid_yaw_angle_ref, position_ref));
			*/
			chassis.vw = (-pid_calc(&pid_chassis_angle, gimbal.sensor.yaw_relative_angle, position_ref));
			
			}
		else
			{chassis.vw = 0;}
		VAL_LIMIT(chassis.vx, -2500, 2500);
		VAL_LIMIT(chassis.vy, -2500, 2500);
}

////导航模式
//static void chassis_navigation_handler()
//{
//	 float nor_chassis_vx,nor_chassis_vy;
//		//计算底盘x、y轴上的速度
//		chassis_power_contorl(&pid_power,&nor_chassis_vx,&nor_chassis_vy,&yaw_speed,chassis.CapData[1],cap_store,(float)judge_recv_mesg.game_robot_state.chassis_power_limit);
//		int position_ref = 0;
//		chassis.vx=nor_chassis_vx;
//		chassis.vy=nor_chassis_vy;
//		if(pc_recv_mesg.mode_Union.info.navigation_determine == 1 ){
//			chassis.vx = 1000 * -pc_recv_mesg.linear_speed_x;
//			chassis.vy = 1000 * -pc_recv_mesg.linear_speed_y;}
//		else {chassis.vx = 0; chassis.vy = 0;}
//		if (input_flag == 2)
//			{chassis.vw = (-pid_calc(&pid_chassis_angle, gimbal.sensor.yaw_relative_angle, position_ref));}
//		else
//			{chassis.vw = 0;}
//		VAL_LIMIT(chassis.vx, -1500, 1500);
//		VAL_LIMIT(chassis.vy, -1500, 1500);
//}
float dodge_min_2 = 270;
// 小陀螺模式
static void chassis_dodge_handler(void)
{
	uint8_t speed_change;
	float chassis_vx_dodge, chassis_vy_dodge;
	float dodge_angle;
	float dodge_min = 150;
	float dodge_max = 400;
	float dodge_chassis_vx, dodge_chassis_vy;
	static int dodge_cap = 0;
	if(pc_recv_mesg.mode_Union.info.navigation_determine == 1 && pc_recv_mesg.mode_Union.info.dodge_ctrl == 0)
	{
		dodge_min = 200;
		dodge_max = 400;
	}
	
	if(pc_recv_mesg.mode_Union.info.dodge_ctrl == 1 || pc_recv_mesg.mode_Union.info.visual_valid == 1
		|| (pc_recv_mesg.linear_speed_x == 0&& pc_recv_mesg.linear_speed_y == 0))
	{
			dodge_min = dodge_min_2;//250
			dodge_max = 500;
	}
	
	if (last_chassis_mode != CHASSIS_DODGE_MODE)
	{
		PID_Clear(&pid_chassis_vw);
	}
	if( pc_recv_mesg.mode_Union.info.visual_valid == 0)//pc_recv_mesg.navigation_determine == 1 &&/*&& pc_recv_mesg.mode_Union.info.visual_valid == 0 
		//&&((pc_recv_mesg.linear_speed_x <= 0.4 && pc_recv_mesg.linear_speed_x >= -0.4)
		//|| (pc_recv_mesg.linear_speed_y <= 0.4 && pc_recv_mesg.linear_speed_y >= -0.4))*/
	{
		rm.vx = -pc_recv_mesg.linear_speed_x * 1000;
		rm.vy = pc_recv_mesg.linear_speed_y * 1000;
		VAL_LIMIT(rm.vx, -1500, 1500);
		VAL_LIMIT(rm.vy, -1500, 1500);
	}
	else{
		rm.vx = 0;
		rm.vy = 0;
	}
	/*小陀螺*/
	dodge_angle = gimbal.sensor.yaw_relative_angle;
	//
	dodge_chassis_vx = rm.vx ; //* 0.5f
  dodge_chassis_vy = rm.vy ;
//	if ((rm.vx != 0 || rm.vy != 0) && (km.vx == 0 || km.vy == 0)) // 小陀螺时遥控方向降低速度
//	{
//		dodge_chassis_vx = rm.vx ; //* 0.5f
//		dodge_chassis_vy = rm.vy ; //* 0.5f
//	}
//	else
//		// 键盘时则通过功率算法来控制速度，防止掉电容
//		chassis_power_contorl(&pid_power, &dodge_chassis_vx, &dodge_chassis_vy, &yaw_speed, chassis.CapData[1], cap_store, (float)judge_recv_mesg.game_robot_state.chassis_power_limit);

	chassis.vy = (dodge_chassis_vx * arm_sin_f32(PI / 180 * dodge_angle) + dodge_chassis_vy * arm_cos_f32(PI / 180 * dodge_angle));
	chassis.vx = (dodge_chassis_vx * arm_cos_f32(PI / 180 * dodge_angle) - dodge_chassis_vy * arm_sin_f32(PI / 180 * dodge_angle));

	if (Speed_up == 1)//不加速				
	{
		if(judge_recv_mesg.game_robot_state.chassis_power_limit>=45&&judge_recv_mesg.game_robot_state.chassis_power_limit<=220)
		{
			pid_calc(&pid_chassis_vw,ob_total_power,(judge_recv_mesg.game_robot_state.chassis_power_limit)*Charge_factor);
		}
		else//没接裁判系统
			pid_calc(&pid_chassis_vw,ob_total_power,(Debug_Power+5)*Charge_factor);//没有连接裁判系统，小车底盘期望功率达到45w
	}
	else if (Speed_up == 0)//按下加速功率多100w
	{
		if(judge_recv_mesg.game_robot_state.chassis_power_limit>=45&&judge_recv_mesg.game_robot_state.chassis_power_limit<=220)
		{
			pid_calc(&pid_chassis_vw,ob_total_power,(judge_recv_mesg.game_robot_state.chassis_power_limit+105));
		}
		else//没接裁判系统
		{
			pid_calc(&pid_chassis_vw,ob_total_power,105+Debug_Power);//没有连接裁判系统，小车底盘期望功率达到150w
		}
	}	

	chassis.vw = dodge_min+pid_chassis_vw.out;
	if (chassis.vw < dodge_min)
		chassis.vw = dodge_min;
	if (chassis.vw > dodge_max)
		chassis.vw = dodge_max;
	
	if(rc.sw2 == RC_MI)
	{
		dodge_min = -200;
		dodge_max = -500;
		chassis.vw = dodge_min+pid_chassis_vw.out;
	if (chassis.vw > dodge_min)
		chassis.vw = dodge_min;
	if (chassis.vw < dodge_max)
		chassis.vw = dodge_max;
	}
}

#else
//新功率算法
void chassis_power_contorl(pid_t *power_pid,float *power_vx,float *power_vy,float *power_yaw_speed,float real_time_Cap_remain,float real_time_Cap_can_store,float judge_power_limit)
{
    static  float max = 3300;
    static float min = 1150;//1000;
    static float Cap_low = 15.0f;      // 何时停止加速
		float Ref_temp = 0;//功率换启动临时变量

		*power_yaw_speed = 0.01f;


		if (real_time_Cap_remain < Cap_low)
				Speed_up = 1; // 一旦检测到电容低则自动关闭加速
		
		if(!SLOW_SPD)
		{
			Charge_factor = 1.0f;
		}
		else//降速给底盘充电
		{
			Charge_factor = 0.6f;// 1-0.6=40% 的电量给电容充电
		}
		
		if(km.vy != 0 || rm.vy != 0 || km.vx != 0 || rm.vx != 0)//底盘输入不为0,才进行pid运算（防止底盘静止时，pid->out积满）
		{
				if (Speed_up == 1)//不加速				
				{
					if(judge_recv_mesg.game_robot_state.chassis_power_limit>=45&&judge_recv_mesg.game_robot_state.chassis_power_limit<=220)
					{
						if((judge_recv_mesg.game_robot_state.chassis_power_limit)*Charge_factor - ob_total_power > Deta_Power)//功率相差过大
						{
							Ref_temp = ob_total_power + Deta_Power;							
							pid_calc(power_pid,ob_total_power,Ref_temp);
						}
						else
							pid_calc(power_pid,ob_total_power,(judge_recv_mesg.game_robot_state.chassis_power_limit)*Charge_factor);
					}
					else//没接裁判系统
						pid_calc(power_pid,ob_total_power,(Debug_Power+5)*Charge_factor);//没有连接裁判系统，小车底盘期望功率达到45w			
				}
			
		}
	if(Speed_up == 1)
	{
		if (km.vy > 0 || rm.vy > 0)
		{
			 *power_vy = (min + power_pid->out);
			 VAL_LIMIT(*power_vy, min * 0.7f, max);
		}
		else if (km.vy < 0 || rm.vy < 0)
		{
			 *power_vy = -(min + power_pid->out);
			 VAL_LIMIT(*power_vy, -max, -min * 0.7f);
		}
		else
			*power_vy = 0;

		if (km.vx > 0 || rm.vx > 0)
		{
				*power_vx = (min + power_pid->out);
			 VAL_LIMIT(*power_vx, min, max);
		}
		else if (km.vx < 0 || rm.vx < 0)
		{
			*power_vx = -(min + power_pid->out);
			VAL_LIMIT(*power_vx, -max, -min);
		}
		else
			*power_vx = 0;
	}
	else if(Speed_up == 0)
	{
		if (km.vy > 0 || rm.vy > 0)
		{
			 *power_vy = CHASSIS_KB_MAX_SPEED_Y;
		}
		else if (km.vy < 0 || rm.vy < 0)
		{
			 *power_vy = -CHASSIS_KB_MAX_SPEED_Y;
		}
		else
			*power_vy = 0;

		if (km.vx > 0 || rm.vx > 0)
		{
				*power_vx = CHASSIS_KB_MAX_SPEED_X;
		}
		else if (km.vx < 0 || rm.vx < 0)
		{
			*power_vx = -CHASSIS_KB_MAX_SPEED_X;
		}
		else
			*power_vx = 0;
	}
		
	if (fast_flag && (rc.ch2 == 660 || FAST_SPD)) // 快速
	{
		Speed_up = 0;
	}
	else // 正常速度
	{
		Speed_up = 1;
	}

	if ((real_time_Cap_remain > Cap_low )&& (!FAST_SPD))
	{
		fast_flag = 1;
	} // 防止加速过程中反复开关加速模式（抽搐）
	if (real_time_Cap_remain < Cap_low)
	{
		fast_flag = 0;
	}
}
//导航模式
static void chassis_navigation_handler()
{
	 float nor_chassis_vx,nor_chassis_vy;
		//计算底盘x、y轴上的速度
		chassis_power_contorl(&pid_power,&nor_chassis_vx,&nor_chassis_vy,&yaw_speed,chassis.CapData[1],cap_store,(float)judge_recv_mesg.game_robot_state.chassis_power_limit);
		int position_ref = 0;
		chassis.vx=nor_chassis_vx;
		chassis.vy=nor_chassis_vy;
		if(pc_recv_mesg.mode_Union.info.navigation_determine == 1 ){
			chassis.vx = 1000 * -pc_recv_mesg.linear_speed_x;
			chassis.vy = 1000 * -pc_recv_mesg.linear_speed_y;}
		else {chassis.vx = 0; chassis.vy = 0;}
		if (input_flag == 2)
			{chassis.vw = (-pid_calc(&pid_chassis_angle, gimbal.sensor.yaw_relative_angle, position_ref));}
		else
			{chassis.vw = 0;}
		VAL_LIMIT(chassis.vx, -2500, 2500);
		VAL_LIMIT(chassis.vy, -2500, 2500);
}

//static void chassis_navigation_handler()
//{
//	 float nor_chassis_vx,nor_chassis_vy;
//		//计算底盘x、y轴上的速度
//		chassis_power_contorl(&pid_power,&nor_chassis_vx,&nor_chassis_vy,&yaw_speed,chassis.CapData[1],cap_store,(float)judge_recv_mesg.game_robot_state.chassis_power_limit);
//		int position_ref = 0;
//		chassis.vx=nor_chassis_vx;
//		chassis.vy=nor_chassis_vy;
//		if(pc_recv_mesg.mode_Union.info.navigation_determine == 1 ){
//			chassis.vx = 1000 * -pc_recv_mesg.linear_speed_x;
//			chassis.vy = 1000 * -pc_recv_mesg.linear_speed_y;}
//		else {chassis.vx = 0; chassis.vy = 0;}
//		if (input_flag == 2)
//			{chassis.vw = (-pid_calc(&pid_chassis_angle, gimbal.sensor.yaw_relative_angle, position_ref));}
//		else
//			{chassis.vw = 0;}
//		VAL_LIMIT(chassis.vx, -1500, 1500);
//		VAL_LIMIT(chassis.vy, -1500, 1500);
//}


// 小陀螺模式
static void chassis_dodge_handler(void)
{
	uint8_t speed_change;
	float chassis_vx_dodge, chassis_vy_dodge;
	float dodge_angle;
	float dodge_min = 150;
	float dodge_max = 400;
	float dodge_chassis_vx, dodge_chassis_vy;
	static int dodge_cap = 0;
	if(pc_recv_mesg.mode_Union.info.navigation_determine == 1 && pc_recv_mesg.mode_Union.info.dodge_ctrl == 0)
	{
		dodge_min = 100;
		dodge_max = 200;
	}
	
	if(pc_recv_mesg.mode_Union.info.dodge_ctrl == 1 || pc_recv_mesg.mode_Union.info.visual_valid == 1
		|| (pc_recv_mesg.linear_speed_x == 0&& pc_recv_mesg.linear_speed_y == 0))
	{
			dodge_min = 200;
			dodge_max = 500;
	}
	
	if (last_chassis_mode != CHASSIS_DODGE_MODE)
	{
		PID_Clear(&pid_chassis_vw);
	}
	if( pc_recv_mesg.mode_Union.info.visual_valid == 0)//pc_recv_mesg.navigation_determine == 1 &&/*&& pc_recv_mesg.mode_Union.info.visual_valid == 0 
		//&&((pc_recv_mesg.linear_speed_x <= 0.4 && pc_recv_mesg.linear_speed_x >= -0.4)
		//|| (pc_recv_mesg.linear_speed_y <= 0.4 && pc_recv_mesg.linear_speed_y >= -0.4))*/
	{
		rm.vx = -pc_recv_mesg.linear_speed_x * 1000;
		rm.vy = pc_recv_mesg.linear_speed_y * 1000;
		VAL_LIMIT(rm.vx, -1500, 1500);
		VAL_LIMIT(rm.vy, -1500, 1500);
	}
	else{
		rm.vx = 0;
		rm.vy = 0;
	}
	/*小陀螺*/
	dodge_angle = gimbal.sensor.yaw_relative_angle;
	//
	dodge_chassis_vx = rm.vx ; //* 0.5f
  dodge_chassis_vy = rm.vy ;
//	if ((rm.vx != 0 || rm.vy != 0) && (km.vx == 0 || km.vy == 0)) // 小陀螺时遥控方向降低速度
//	{
//		dodge_chassis_vx = rm.vx ; //* 0.5f
//		dodge_chassis_vy = rm.vy ; //* 0.5f
//	}
//	else
//		// 键盘时则通过功率算法来控制速度，防止掉电容
//		chassis_power_contorl(&pid_power, &dodge_chassis_vx, &dodge_chassis_vy, &yaw_speed, chassis.CapData[1], cap_store, (float)judge_recv_mesg.game_robot_state.chassis_power_limit);

	chassis.vy = (dodge_chassis_vx * arm_sin_f32(PI / 180 * dodge_angle) + dodge_chassis_vy * arm_cos_f32(PI / 180 * dodge_angle));
	chassis.vx = (dodge_chassis_vx * arm_cos_f32(PI / 180 * dodge_angle) - dodge_chassis_vy * arm_sin_f32(PI / 180 * dodge_angle));

	if (Speed_up == 1)//不加速				
	{
		if(judge_recv_mesg.game_robot_state.chassis_power_limit>=45&&judge_recv_mesg.game_robot_state.chassis_power_limit<=220)
		{
			pid_calc(&pid_chassis_vw,ob_total_power,(judge_recv_mesg.game_robot_state.chassis_power_limit)*Charge_factor);
		}
		else//没接裁判系统
			pid_calc(&pid_chassis_vw,ob_total_power,(Debug_Power+5)*Charge_factor);//没有连接裁判系统，小车底盘期望功率达到45w
	}
	else if (Speed_up == 0)//按下加速功率多100w
	{
		if(judge_recv_mesg.game_robot_state.chassis_power_limit>=45&&judge_recv_mesg.game_robot_state.chassis_power_limit<=220)
		{
			pid_calc(&pid_chassis_vw,ob_total_power,(judge_recv_mesg.game_robot_state.chassis_power_limit+105));
		}
		else//没接裁判系统
		{
			pid_calc(&pid_chassis_vw,ob_total_power,105+Debug_Power);//没有连接裁判系统，小车底盘期望功率达到150w
		}
	}	

	chassis.vw = dodge_min+pid_chassis_vw.out;
	if (chassis.vw < dodge_min)
		chassis.vw = dodge_min;
	if (chassis.vw > dodge_max)
		chassis.vw = dodge_max;
	
	if(rc.sw2 == RC_MI)
	{
		dodge_min = -200;
		dodge_max = -500;
		chassis.vw = dodge_min+pid_chassis_vw.out;
	if (chassis.vw > dodge_min)
		chassis.vw = dodge_min;
	if (chassis.vw < dodge_max)
		chassis.vw = dodge_max;
	}
}
#endif

//// 小陀螺模式
//static void chassis_dodge_handler(void)
//{
//	float chassis_vx_dodge, chassis_vy_dodge;
//	float dodge_angle;
//	float dodge_min = 150;
//	float dodge_max = 500;
//	float dodge_chassis_vx, dodge_chassis_vy;
//	static int dodge_cap = 0;

//	if (last_chassis_mode != CHASSIS_DODGE_MODE)
//	{
//		/* 刚刚进入小陀螺时，旋转速度通过计算得到 */
//		chassis.vw = dodge_min + ((float)judge_recv_mesg.game_robot_state.chassis_power_limit - 45) / 45 * 150;

//		if (cap_store - chassis.CapData[1] < 0.5)
//			dodge_cap = chassis.CapData[1] - 0.5; // 防止充放平衡
//		else
//			dodge_cap = chassis.CapData[1];
//	}

//	/*小陀螺*/
//	dodge_angle = gimbal.sensor.yaw_relative_angle;
//	//
//	if ((rm.vx != 0 || rm.vy != 0) && (km.vx == 0 || km.vy == 0)) // 小陀螺时遥控方向降低速度
//	{
//		dodge_chassis_vx = rm.vx * 0.5f;
//		dodge_chassis_vy = rm.vy * 0.5f;
//	}
//	else
//		// 键盘时则通过功率算法来控制速度，防止掉电容
//		chassis_power_contorl(&pid_power, &dodge_chassis_vx, &dodge_chassis_vy, &yaw_speed, chassis.CapData[1], cap_store, (float)judge_recv_mesg.game_robot_state.chassis_power_limit);

//	chassis.vy = (dodge_chassis_vx * arm_sin_f32(PI / 180 * dodge_angle) + dodge_chassis_vy * arm_cos_f32(PI / 180 * dodge_angle));
//	chassis.vx = (dodge_chassis_vx * arm_cos_f32(PI / 180 * dodge_angle) - dodge_chassis_vy * arm_sin_f32(PI / 180 * dodge_angle));

//	pid_calc(&pid_chassis_vw, chassis.CapData[1], dodge_cap);

//	chassis.vw -= pid_chassis_vw.out;
//	if (chassis.vw < dodge_min)
//		chassis.vw = dodge_min;
//	if (chassis.vw > dodge_max)
//		chassis.vw = dodge_max;
//}

// 普通模式
static void chassis_normal_handler(void)
{
	float nor_chassis_vx, nor_chassis_vy;

	// 计算底盘x、y轴上的速度
	chassis_power_contorl(&pid_power, &nor_chassis_vx, &nor_chassis_vy, &yaw_speed, chassis.CapData[1], cap_store, (float)judge_recv_mesg.game_robot_state.chassis_power_limit);

	int position_ref = 0;
	if (chassis_mode == CHASSIS_NORMAL_MODE)
	{
		if (input_flag == 1)
		// w轴方向跟着云台动
		{
			//底盘测试专用
/*			chassis.pid_yaw_angle_ref = rm.chassis_yaw_v * GIMBAL_RC_MOVE_RATIO_YAW*50;
        chassis.yaw_navigation_relative_angle = get_chassis_relative_pos(chassis.pid_yaw_angle_ref, gimbal.yaw_center_offset); 
				chassis.vw = (-pid_calc(&pid_chassis_angle,chassis.pid_yaw_angle_ref, position_ref));
*/
			chassis.vw = (-pid_calc(&pid_chassis_angle, gimbal.sensor.yaw_relative_angle, position_ref));
		}
		else
		{
			chassis.vw = 0;
		}
	}
//	if(nor_chassis_vx<-2000) nor_chassis_vx = -2000;
	chassis.vx = nor_chassis_vx;
	chassis.vy = nor_chassis_vy;
	
}


// 停止模式
static void chassis_stop_handler(void)
{
	for (int i = 0; i < 4; i++)
	{
		chassis.current[i] = pid_calc(&pid_spd[i], chassis.wheel_spd_fdb[i], 0);
	}

	chassis.vy = 0;
	chassis.vx = 0;
	chassis.vw = 0;
}
// 初始化
void chassis_param_init(void)
{
	memset(&chassis, 0, sizeof(chassis));

	/*底盘vw旋转的pid*/
	PID_Struct_Init(&pid_chassis_angle, chassis_pid[0], chassis_pid[1], chassis_pid[2], MAX_CHASSIS_VR_SPEED, 50, INIT);
	#ifndef POWER_NEW
	PID_Struct_Init(&pid_power, cap_vol_pid[0], cap_vol_pid[1], cap_vol_pid[2], 100, 20, INIT);//老功率算法
	#else
  PID_Struct_Init(&pid_power, power_new_pid[0], power_new_pid[1], power_new_pid[2], 3000, 2000, INIT);//新功率算法
	#endif
	/*底盘vx,vy平移的pid*/
	/////////////////
	for (int i = 0; i < 4; i++)
		PID_Struct_Init(&pid_spd[i], chassis_spd_pid[i][0], chassis_spd_pid[i][1], chassis_spd_pid[i][2], 10000, 500, INIT);
	////////////////
}

/**
 * @brief mecanum chassis velocity decomposition
 * @param input : ↑=+vx(mm/s)  ←=+vy(mm/s)  ccw=+vw(deg/s)
 *        output: every wheel speed(rpm)
 * @trans 输入：		前后左右的量
 *				 输出：		每个轮子对应的速度
 * @note  1=FR 2=FL 3=BL 4=BR
 * @work	 分析演算公式计算的效率
 */
int rotation_center_gimbal = 0;
static void mecanum_calc(float vx, float vy, float vw, int16_t speed[]) // 底盘解算，得到底盘获得相应速度需要的四个电机值
{
	static float rotate_ratio_fr;
	static float rotate_ratio_fl;
	static float rotate_ratio_bl;
	static float rotate_ratio_br;
	static float wheel_rpm_ratio;

	taskENTER_CRITICAL();
	//@work
	rotate_ratio_fr = ((WHEELBASE + WHEELTRACK) / 2.0f - GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET) / RADIAN_COEF;
	rotate_ratio_fl = ((WHEELBASE + WHEELTRACK) / 2.0f - GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET) / RADIAN_COEF;
	rotate_ratio_bl = ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET) / RADIAN_COEF;
	rotate_ratio_br = ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET) / RADIAN_COEF;
	wheel_rpm_ratio = 60.0f / (PERIMETER * CHASSIS_DECELE_RATIO);
	taskEXIT_CRITICAL();

	VAL_LIMIT(vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED); // mm/s
	VAL_LIMIT(vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED); // mm/s
	/*小陀螺以外的模式，vw限制正常*/
	if (chassis_mode != CHASSIS_DODGE_MODE && !chassis.dodge_ctrl)
		VAL_LIMIT(vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED); // deg/s
	/*小陀螺时，vw不受限制*/
	//	else
	//		VAL_LIMIT(vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s

	int16_t wheel_rpm[4];
	float max = 0;

	wheel_rpm[0] = (-vx - vy - vw * rotate_ratio_fr) * wheel_rpm_ratio;
	wheel_rpm[1] = (vx - vy - vw * rotate_ratio_fl) * wheel_rpm_ratio;
	wheel_rpm[2] = (vx + vy - vw * rotate_ratio_bl) * wheel_rpm_ratio;
	wheel_rpm[3] = (-vx + vy - vw * rotate_ratio_br) * wheel_rpm_ratio;

	// find max item
	for (uint8_t i = 0; i < 4; i++)
	{
		if (abs(wheel_rpm[i]) > max)
			max = abs(wheel_rpm[i]);
	}
	// equal proportion
	if (max > MAX_WHEEL_RPM)
	{
		float rate = MAX_WHEEL_RPM / max;
		for (uint8_t i = 0; i < 4; i++)

			wheel_rpm[i] *= rate;
	}
	memcpy(speed, wheel_rpm, 4 * sizeof(int16_t));
}


int16_t get_chassis_relative_pos(int16_t raw_ecd, int16_t center_offset)
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

// 缓启动函数
void buffer(float *a, float b, float high_parameter, float low_parameter)
{

	if (((*a - b) < high_parameter) && ((*a - b) > -low_parameter))
	{
		*a = b;
	}
	else
	{
				
			if (*a < b)
				*a += high_parameter;
			if (*a > b)
				*a -= low_parameter;
		
		
		
	}
}

