#include "pc_tx_data.h"
#include "pc_rx_data.h"
#include "judge_rx_data.h"
#include "modeswitch_task.h"
#include "gimbal_task.h"
#include "keyboard.h"
#include "rc.h"
#include "shoot_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

/*小电脑发送*/
static SemaphoreHandle_t pc_txdata_mutex;
fifo_s_t  pc_txdata_fifo;
static uint8_t   pc_txdata_buf[PC_TX_FIFO_BUFLEN];

void pc_tx_param_init(void)
{
    /* create the judge_rxdata_mutex mutex  */  
  pc_txdata_mutex = xSemaphoreCreateMutex();
  
  /* judge data fifo init */
  fifo_s_init(&pc_txdata_fifo, pc_txdata_buf, PC_TX_FIFO_BUFLEN, pc_txdata_mutex);
}


/* data send */
robot_tx_data       pc_send_mesg;
robot_judge1_data   pc_send_judge1;

uint8_t pc_tx_packet_id = GIMBAL_DATA_ID;
int count_cali_count = 0;
void pc_send_data_packet_pack(void)
{
	get_upload_data();
	
	//SENTRY_DATA_ID 哨兵  UP_REG_ID小电脑帧头
	data_packet_pack(SENTRY_DATA_ID, (uint8_t *)&pc_send_mesg, 
							 sizeof(robot_tx_data), UP_REG_ID);
//	data_packet_pack(CHASSIS_DATA_ID,(uint8_t *)&pc_send_judge1,
//							 sizeof(robot_judge1_data),UP_REG_ID);
	data_packet_pack(GIMBAL_DATA_ID,(uint8_t *)&judge_recv_mesg.game_robot_HP,
							sizeof(game_robot_HP_t),UP_REG_ID);
//	data_packet_pack(SHOOT_TASK_DATA_ID,(uint8_t *)&judge_recv_mesg.sentry_info,
//							sizeof(sentry_info_t),UP_REG_ID);
}

void get_upload_data(void)
{
  taskENTER_CRITICAL();
  
	get_judge_data();
	
  get_infantry_info();
  
	
  taskEXIT_CRITICAL();
}

void get_judge_data(void)
{
	
	
	/* 获取敌方颜色 */
	if(judge_recv_mesg.game_robot_state.robot_id>10)
		pc_send_mesg.robot_color = red;//敌方红		0
	else
		pc_send_mesg.robot_color = blue;//敌方蓝	1
	
	pc_send_judge1.game_progress = judge_recv_mesg.game_state.game_progress;
	pc_send_judge1.stage_remain_time = judge_recv_mesg.game_state.stage_remain_time;
	pc_send_judge1.robot_id = judge_recv_mesg.game_robot_state.robot_id;
	pc_send_judge1.current_HP = judge_recv_mesg.game_robot_state.current_HP;
	pc_send_judge1.maximum_HP = judge_recv_mesg.game_robot_state.maximum_HP;
	pc_send_judge1.armor_id = judge_recv_mesg.robot_hurt.armor_id;
	pc_send_judge1.HP_deduction_reason = judge_recv_mesg.robot_hurt.HP_deduction_reason;
	pc_send_judge1.projectile_allowance_17mm = judge_recv_mesg.bullet_remaining.projectile_allowance_17mm;
	pc_send_judge1.cmd_keyboard = judge_recv_mesg.minimap_interactive_data.cmd_keyboard;
	pc_send_judge1.target_position_x = judge_recv_mesg.minimap_interactive_data.target_position_x;
	pc_send_judge1.target_position_y = judge_recv_mesg.minimap_interactive_data.target_position_y;
	pc_send_judge1.dart_info = judge_recv_mesg.dart_remaining_time.fly_goal;
}

void get_infantry_info(void)
{
		uint8_t mode = gimbal_mode;
/*
  不发实时弹速，随机误差太大。（后续如机械电控部分可稳定弹速（误差+-0.1m/s）,则可以考虑添加）

  RMUC22规则步兵17mm弹速：

  爆发优先 1级  	15m/s     冷却优先 1级    15m/s     弹速优先 1级    30m/s
  爆发优先 2级 		15m/s     冷却优先 2级    18m/s     弹速优先 2级    30m/s
  爆发优先 3级 		15m/s     冷却优先 3级    18m/s	    弹速优先 3级    30m/s
*/
  
//  if(judge_data_limit.shooter_id1_17mm_speed_limit==15)
//    pc_send_mesg.bullet_level=1;
//  if(judge_data_limit.shooter_id1_17mm_speed_limit==18)
//    pc_send_mesg.bullet_level=2;
//  if(judge_data_limit.shooter_id1_17mm_speed_limit==30)
//    pc_send_mesg.bullet_level=3;

  /* get gimable ctrl mode */
	switch(mode)
	{
		case GIMBAL_TRACK_ARMOR:
    { 
			pc_send_mesg.task_mode = TRACK_AMOR_MODE;		//自瞄模式
      pc_send_mesg.robot_pitch = gimbal.sensor.pit_relative_angle;//0
      pc_send_mesg.robot_yaw = gimbal.sensor.yaw_gyro_angle;//发送陀螺仪的数据//0;(不发)
      /*反小陀螺-打哨兵模式按键 长按C发送1，长按V发送2*/
      /* 松手发送0               向左补偿    向右补偿 */
      //direction:2 拓展装甲板标志位
//      if(Left_offset == 1)  
//        pc_send_mesg.direction=  1;
//      else if(Right_offset == 1)
//        pc_send_mesg.direction = 2;
//      else if(SENTRY_MODE == 1)
//        pc_send_mesg.direction = 3;
//      else
//        pc_send_mesg.direction = 0;
		}break;
    
//		case GIMBAL_SHOOT_BUFF:
//    {			
//      if(gimbal.small_buff_ctrl)
//			{
//				pc_send_mesg.task_mode = SMALL_BUFF_MODE;		//小能量机关
//        //发送编码位角度？看视觉需求，用电机还是陀螺仪
//        pc_send_mesg.robot_pitch = -gimbal.sensor.pit_relative_angle;
//        pc_send_mesg.robot_yaw =  gimbal.sensor.yaw_relative_angle;
//			}
//      else if(gimbal.big_buff_ctrl)
//			{
//        //发送编码位角度？看视觉需求，用电机还是陀螺仪
//				pc_send_mesg.task_mode = BIG_BUFF_MODE;			//大能量机关
//        pc_send_mesg.robot_pitch = -gimbal.sensor.pit_relative_angle;
//        pc_send_mesg.robot_yaw =  gimbal.sensor.yaw_relative_angle;
//			}
//			else
//			{
//        //发送编码位角度？看视觉需求，用电机还是陀螺仪
//				pc_send_mesg.task_mode = NORMAL_CTRL_MODE;	//普通模式		
//        pc_send_mesg.robot_pitch = -gimbal.sensor.pit_relative_angle;
//        pc_send_mesg.robot_yaw =  gimbal.sensor.yaw_relative_angle;
//			}
//		}break;
		
		default:
    {
//			pc_send_mesg.task_mode = NORMAL_CTRL_MODE;
			pc_send_mesg.robot_pitch = gimbal.sensor.pit_relative_angle;//0
//      pc_send_mesg.robot_pitch = -gimbal.sensor.pit_gyro_angle;
      pc_send_mesg.robot_yaw =  gimbal.sensor.yaw_gyro_angle; //默认发送陀螺仪角度
		}break;
	}
	
}
