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

/*С���Է���*/
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
	
	//SENTRY_DATA_ID �ڱ�  UP_REG_IDС����֡ͷ
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
	
	
	/* ��ȡ�з���ɫ */
	if(judge_recv_mesg.game_robot_state.robot_id>10)
		pc_send_mesg.robot_color = red;//�з���		0
	else
		pc_send_mesg.robot_color = blue;//�з���	1
	
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
  ����ʵʱ���٣�������̫�󡣣��������е��ز��ֿ��ȶ����٣����+-0.1m/s��,����Կ�����ӣ�

  RMUC22���򲽱�17mm���٣�

  �������� 1��  	15m/s     ��ȴ���� 1��    15m/s     �������� 1��    30m/s
  �������� 2�� 		15m/s     ��ȴ���� 2��    18m/s     �������� 2��    30m/s
  �������� 3�� 		15m/s     ��ȴ���� 3��    18m/s	    �������� 3��    30m/s
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
			pc_send_mesg.task_mode = TRACK_AMOR_MODE;		//����ģʽ
      pc_send_mesg.robot_pitch = gimbal.sensor.pit_relative_angle;//0
      pc_send_mesg.robot_yaw = gimbal.sensor.yaw_gyro_angle;//���������ǵ�����//0;(����)
      /*��С����-���ڱ�ģʽ���� ����C����1������V����2*/
      /* ���ַ���0               ���󲹳�    ���Ҳ��� */
      //direction:2 ��չװ�װ��־λ
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
//				pc_send_mesg.task_mode = SMALL_BUFF_MODE;		//С��������
//        //���ͱ���λ�Ƕȣ����Ӿ������õ������������
//        pc_send_mesg.robot_pitch = -gimbal.sensor.pit_relative_angle;
//        pc_send_mesg.robot_yaw =  gimbal.sensor.yaw_relative_angle;
//			}
//      else if(gimbal.big_buff_ctrl)
//			{
//        //���ͱ���λ�Ƕȣ����Ӿ������õ������������
//				pc_send_mesg.task_mode = BIG_BUFF_MODE;			//����������
//        pc_send_mesg.robot_pitch = -gimbal.sensor.pit_relative_angle;
//        pc_send_mesg.robot_yaw =  gimbal.sensor.yaw_relative_angle;
//			}
//			else
//			{
//        //���ͱ���λ�Ƕȣ����Ӿ������õ������������
//				pc_send_mesg.task_mode = NORMAL_CTRL_MODE;	//��ͨģʽ		
//        pc_send_mesg.robot_pitch = -gimbal.sensor.pit_relative_angle;
//        pc_send_mesg.robot_yaw =  gimbal.sensor.yaw_relative_angle;
//			}
//		}break;
		
		default:
    {
//			pc_send_mesg.task_mode = NORMAL_CTRL_MODE;
			pc_send_mesg.robot_pitch = gimbal.sensor.pit_relative_angle;//0
//      pc_send_mesg.robot_pitch = -gimbal.sensor.pit_gyro_angle;
      pc_send_mesg.robot_yaw =  gimbal.sensor.yaw_gyro_angle; //Ĭ�Ϸ��������ǽǶ�
		}break;
	}
	
}
