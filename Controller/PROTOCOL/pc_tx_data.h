#ifndef _pc_tx_data_H
#define _pc_tx_data_H

#include "stm32f4xx.h"
#include "data_packet.h"

#define PC_TX_FIFO_BUFLEN 500

typedef enum
{
	TRACK_AMOR_MODE			=0,
	BIG_BUFF_MODE				=1,
	NORMAL_CTRL_MODE		=2,
	SMALL_BUFF_MODE			=3,
}mode_t;

typedef enum
{
	red  = 0,
	blue = 1,
}enemy_color_t;

/**
 * @brief  鐢垫帶->PC鐨勬暟鎹robot_tx_data (13瀛楄妭) 锛屾暟鎹抚闀匡細5 + 13 + 2 = 20 (瀛楄妭)
 */
#pragma pack(1)
//13字节
typedef struct
{
  uint8_t robot_color : 1;  // 颜色 (0 红 / 1 蓝)
  uint8_t task_mode : 1;    // 识别模式  (0 自瞄 / 1 大小符)
  uint8_t visual_valid : 1; // 视觉有效位 (0/1)
  uint8_t direction : 2;    // 拓展装甲板 (0-3)
  uint8_t bullet_level : 3; // 弹速等级 1 2 3级
  float robot_pitch;        // 欧拉角(度)
  float robot_yaw;          // 欧拉角(度)
  float time_stamp;         // 电控时间戳(ms) 
} robot_tx_data;
//18字节
typedef struct
{
	uint8_t dart_info : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	uint8_t robot_id;
	uint16_t current_HP : 8;
	uint16_t maximum_HP : 8;
	uint8_t armor_id : 4;
  uint8_t HP_deduction_reason : 4;
	uint16_t projectile_allowance_17mm;
	uint8_t cmd_keyboard;
	float target_position_x; 
	float target_position_y;
} robot_judge1_data;

//typedef struct
//{
//	uint16_t red_1_robot_HP;
//  uint16_t red_2_robot_HP;
//  uint16_t red_3_robot_HP;
//  uint16_t red_4_robot_HP;
//  uint16_t red_5_robot_HP;
//  uint16_t red_7_robot_HP;
//  uint16_t red_outpost_HP;
//  uint16_t red_base_HP;
//  uint16_t blue_1_robot_HP;
//  uint16_t blue_2_robot_HP;
//  uint16_t blue_3_robot_HP;
//  uint16_t blue_4_robot_HP;
//  uint16_t blue_5_robot_HP;
//  uint16_t blue_7_robot_HP;
//  uint16_t blue_outpost_HP;
//  uint16_t blue_base_HP;
//} robot_HP_data;
#pragma pack()

extern fifo_s_t  pc_txdata_fifo;
extern robot_tx_data pc_send_mesg;
extern robot_judge1_data pc_send_judge1;

void pc_tx_param_init(void);
void pc_send_data_packet_pack(void);
void get_upload_data(void);
void get_infantry_info(void);
void get_judge_data(void);


#endif
