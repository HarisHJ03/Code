#ifndef _judge_rx_data_H
#define _judge_rx_data_H


#include "stm32f4xx.h"
#include "data_packet.h"

#define JUDGE_RX_FIFO_BUFLEN 500

/**代码相对于23赛季裁判系统通信缺少模块如下***
*0x020B -- 地面机器人位置数据 (服务器→己方哨兵机器人)  
*0x020C -- 雷达标记进度数据 (服务器→己方雷达机器人)   不添加
*0x020D -- 哨兵自主决策相关信息同步 (服务器→己方哨兵机器人)     
*0x020E -- 雷达自主决策信息同步 (服务器→己方雷达机器人)  不添加

*0x0306 -- 自定义控制器与选手端交互数据 (自定义控制器→选手端)  不添加
*0x0307 -- 选手端小地图接收哨兵数据 (哨兵/半自动控制机器人→对应操作手选手端) 
*0x0308 -- 选手端小地图接收机器人消息 (己方机器人→己方选手端) 不添加
**/
/**代码相对于23赛季裁判系统通信删除模块如下***
*0x0004 -- 飞镖发射状态，飞镖发射后发送    已删除
*0x0005 -- 人工智能挑战赛加成与惩罚状态    已删除
*0x0103 -- 请求补给站补弹数据，由参赛队发送 已删除
**/
typedef enum //接收发送的ID号都在里面
{
	GAME_STATE_ID                      = 0x0001,  //比赛状态数据
	GAME_RESULT_ID 	                   = 0x0002,  //比赛结果数据
	GAME_ROBOT_HP_ID                   = 0x0003,	//比赛机器人血量数据            
	EVENT_DATA_ID 				             = 0x0101,	//场地事件数据
	SUPPLY_PROJECTILE_ACTION_ID        = 0x0102,	//场地补给站动作标识数据，动作发生后发送
	REFEREE_WARNING_ID                 = 0x0104,  // 裁判系统警告信息
	DART_REMAINING_TINME_ID            = 0x0105,  //飞镖发射口倒计时
	GAME_ROBOT_STATE_ID                = 0x0201,	//机器人状态数据                  
	POWER_HEAT_DATA_ID                 = 0x0202,	//实时功率热量数据
	GAME_ROBOT_POS_ID                  = 0x0203,	//机器人位置数据
	BUFF_ID                            = 0x0204,	//机器人增益数据
	AERIAL_ROBOT_ENERGY_ID             = 0x0205,	//空中机器人能量状态数据
	ROBOT_HURT_ID                      = 0x0206,	//伤害状态数据，伤害发生后发送
	SHOOT_DATA_ID                      = 0x0207,	//实时射击数据，子弹发射后发送
	BULLET_REMAINING_ID                = 0x0208,  //子弹剩余数量：空中机器人，哨兵机器人以及ICRA机器人
  RFID_STATE_ID                      = 0x0209,  //机器人RFID状态
	DAT_CLIENT_CMD_ID                  = 0x020A,  //飞镖机器人客户端指令数据
/* NEW */
	GROUND_ROBOT_POSITION_ID           = 0x020B,  //地面机器人位置数据
	RADAR_MARK_DATA_ID                 = 0x020C,  //雷达标记进度数据
	SENTRY_INFO_ID                     = 0x020D,  //哨兵自主决策相关信息同步
	RADAR_INFO_ID                      = 0x020E,  //雷达自主决策信息同步
/* NEW */	
	STUDENT_INTERACTIVE_HEADER_DATA_ID    = 0x0301,	//机器人间交互数据 和 客户端通信（发送方触发发送） 接收发送都有

  CUSTOM_CONTROLLER_INTERACTION_DATA_ID = 0x0302, //自定义控制器交互数据接口，通过客户端触发发送，上限 30HZ	 |图传链路
	CLIENT_MINIMAP_INTERACTIVE_DATDA_ID   = 0x0303, //客户端小地图交互数据
	KEYBOARD_AND_MOUSE_INFORMATION_ID     = 0x0304, //键盘、鼠标信息，通过图传串口发送  |图传链路
/* NEW */
  MAP_ROBOT_DATA_ID                     = 0x0305, //选手端小地图接收雷达数据
	CUSTOM_CLIENT_DATA_ID                 = 0x0306, //自定义控制器与选手端交互数据
	MAP_DATA_ID                           = 0x0307, //选手端小地图接收哨兵数据
	CUSTOM_INFO_ID                        = 0x0308, //选手端小地图接收机器人消息
/* NEW */	
} judge_data_id_e;

/*裁判系统数据结构体*/
/*0001*/
typedef __packed struct
{
 uint8_t game_type : 4;
 uint8_t game_progress : 4;
 uint16_t stage_remain_time;
 uint64_t SyncTimeStamp;
}game_status_t;

/*0002*/
typedef __packed struct
{
 uint8_t winner;
}game_result_t;

/*0003*/
typedef __packed struct
{
 uint16_t red_1_robot_HP;
 uint16_t red_2_robot_HP;
 uint16_t red_3_robot_HP;
 uint16_t red_4_robot_HP;
 uint16_t red_5_robot_HP;
 uint16_t red_7_robot_HP;
 uint16_t red_outpost_HP;
 uint16_t red_base_HP;
 uint16_t blue_1_robot_HP;
 uint16_t blue_2_robot_HP;
 uint16_t blue_3_robot_HP;
 uint16_t blue_4_robot_HP;
 uint16_t blue_5_robot_HP;
 uint16_t blue_7_robot_HP;
 uint16_t blue_outpost_HP;
 uint16_t blue_base_HP;
}game_robot_HP_t;

/*0101*/
typedef __packed struct
{ __packed union
  {
		uint32_t event_data;
	__packed struct 
	{
		//0：未占领 1：被已方占领 2：被对方占领
        uint32_t self_supply_station : 3;  
		/* 	bit 0：我方补给站前 0/1
				bit 1: 我方补给站内 0/1
				bit 2: 我方补给区(RMUL) 0/1
		*/
        uint32_t bit3_5 : 3;
		/*	bit 3: 能量机关激活点 0/1
				bit 4: 小能量机关 0/1
				bit 5: 大能量机关 0/1
		*/
        uint32_t bit6_11 : 6;
		/*	bit 6-7: 己方环形高地 0/1/2
				bit 8-9: 已方梯形高地 0/1/2
				bit 10-11: 己方梯形高地 0/1/2
		*/
        uint32_t bit12_18 : 7;
		//	己方基地虚拟护盾的剩余值百分比 0-6
        uint32_t bit19_27 : 9;
		//  飞镖最后一次击中己方前哨站或基地的时间 0-8
        uint32_t bit28_29 : 2;
		//	飞镖最后一次击中的位置 0 前哨站/1 基地固定目标/2 基地随机目标
        uint32_t bit30_31 : 2;
		//	中心增益点的占领情况 0/1/2 /3 被双方占领
    } fields;
	}event_data;
}event_data_t;

/*0102*/
typedef __packed struct
{
 uint8_t reserved;
 uint8_t supply_robot_id;
 uint8_t supply_projectile_step;
 uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

/*0104*/
typedef __packed struct
{
 uint8_t level;
 uint8_t offending_robot_id;
 uint8_t count;
}referee_warning_t;

/*0105*/
typedef __packed struct
{
	uint8_t dart_remaining_time;
	uint16_t get_goal : 2;
	uint16_t recent_get_time : 3;
	uint16_t fly_goal : 2;
	uint16_t reserve : 9;
}dart_info_t;

/*0201*/
typedef __packed struct
{
 uint8_t robot_id;
 uint8_t robot_level;
 uint16_t current_HP;
 uint16_t maximum_HP;
 uint16_t shooter_barrel_cooling_value;
 uint16_t shooter_barrel_heat_limit;
 uint16_t chassis_power_limit;
 uint8_t power_management_gimbal_output : 1;
 uint8_t power_management_chassis_output : 1;
 uint8_t power_management_shooter_output : 1;
}robot_status_t;

/*0202*/
typedef __packed struct
{
 uint16_t chassis_voltage;
 uint16_t chassis_current;
 float chassis_power;
 uint16_t buffer_energy;
 uint16_t shooter_17mm_1_barrel_heat;
 uint16_t shooter_17mm_2_barrel_heat;
 uint16_t shooter_42mm_barrel_heat;
}power_heat_data_t;

/*0203*/
typedef __packed struct
{
 float x;
 float y;
 float angle;
}robot_pos_t;

/*0204*/
typedef __packed struct
{
 uint8_t recovery_buff;
 uint8_t cooling_buff;
 uint8_t defence_buff;
 uint8_t vulnerability_buff;
 uint16_t attack_buff;
}buff_t;

/*0205*/
typedef __packed struct
{
 uint8_t airforce_status;
 uint8_t time_remain;
}air_support_data_t;

/*0206*/
typedef __packed struct
{
 uint8_t armor_id : 4;
 uint8_t HP_deduction_reason : 4;
}hurt_data_t;

/*0207*/
typedef __packed struct
{
 uint8_t bullet_type;
 uint8_t shooter_number;
 uint8_t launching_frequency;
 float initial_speed;
}shoot_data_t;

/*0208*/
typedef __packed struct
{
 uint16_t projectile_allowance_17mm;
 uint16_t projectile_allowance_42mm;
 uint16_t remaining_gold_coin;
}projectile_allowance_t;

/*0209*/
typedef __packed struct
{
	__packed union
  {
		uint32_t rfid_status;
	__packed struct 
	{
		//0:未检测 1:已检测
		//0-6
		uint32_t self_base : 1;  //基地
		uint32_t self_circular_highland : 1;   //己方环形高地
		uint32_t enemy_circular_highland : 1;  //敌方环形高地
		uint32_t self_R3_highland : 1; //己方 R3/B3 梯形高地
		uint32_t enemy_R3_highland : 1;//敌方 R3/B3 梯形高地
		uint32_t self_R4_highland : 1; //己方 R4/B4 梯形高地
		uint32_t enemy_R4_highland : 1;//敌方 R4/B4 梯形高地
		//7-12
		uint32_t energy_buff : 1;//己方能量机关激活点
		uint32_t self_fly_slope_front  : 1;//己方飞坡增益点前
		uint32_t self_fly_slope_back   : 1;//己方飞坡增益点后
		uint32_t ememy_fly_slope_front : 1;//敌方飞坡增益点前
		uint32_t ememy_fly_slope_back  : 1;//敌方飞坡增益点后
		uint32_t self_outpost : 1;//己方前哨站增益点
		//13-19
		uint32_t self_blood_point : 1;//己方补血点
		uint32_t self_sentry_patrol  : 1;//己方哨兵巡逻区
		uint32_t ememy_sentry_patrol : 1;//敌方哨兵巡逻区
		uint32_t self_Resource_Island  : 1;//己方大资源岛
		uint32_t ememy_Resource_Island : 1;//敌方大资源岛
		uint32_t self_exchange_area : 1;//己方兑换区
		uint32_t centry_buff : 1;//中心增益点(RMUL)
		//20-31
		uint32_t reserve : 12;//保留位
    } fields;
	}rfid_status;
}rfid_status_t;

/*020A*/
typedef __packed struct
{
 uint8_t dart_launch_opening_status;
 uint8_t reserved;
 uint16_t target_change_time;
 uint16_t latest_launch_cmd_time;
}dart_client_cmd_t;

/*020B*/
typedef __packed struct
{
 float hero_x;
 float hero_y;
 float engineer_x;
 float engineer_y;
 float standard_3_x;
 float standard_3_y;
 float standard_4_x;
 float standard_4_y;
 float standard_5_x;
 float standard_5_y;
}ground_robot_position_t;
/*020C*/
typedef __packed struct
{
 uint8_t mark_hero_progress;
 uint8_t mark_engineer_progress;
 uint8_t mark_standard_3_progress;
 uint8_t mark_standard_4_progress;
 uint8_t mark_standard_5_progress;
 uint8_t mark_sentry_progress;
}radar_mark_data_t;
/*020D*/
typedef __packed struct
{
	uint32_t have_exchange_bullet_number : 11;//已兑换发弹量
	uint32_t have_exchange_remote_bullet_time : 4;//已成功远程兑换发弹量次数
	uint32_t have_exchange_remote_blood_time : 4;//已成功远程兑换血量次数
	uint32_t reserved : 13;
} sentry_info_t;
/*020E*/
typedef __packed struct
{
 uint8_t radar_info;
} radar_info_t;

typedef __packed struct
{
 uint16_t data_cmd_id;
 uint16_t sender_ID;
 uint16_t receiver_ID;
}ext_student_interactive_header_data_t;

/*0302*/
typedef __packed struct
{
  uint8_t data[30];
}robot_interactive_data_t;

/*0303*/
typedef __packed struct
{
 float target_position_x;
 float target_position_y;
 uint8_t cmd_keyboard;
 uint8_t target_robot_id;
 uint8_t cmd_source;
}map_command1_t;

/*0304*/
typedef __packed struct
{
 int16_t mouse_x;
 int16_t mouse_y;
 int16_t mouse_z;
 int8_t left_button_down;
 int8_t right_button_down;
 uint16_t keyboard_value;
 uint16_t reserved;
}remote_control_t;

typedef struct
{
  game_status_t                      	        game_state;               	 	   //0x0001
  game_result_t                      	        game_result;               	     //0x0002
  game_robot_HP_t             			 	        game_robot_HP;     		           //0x0003
  event_data_t  										          event_data;                	     //0x0101
  ext_supply_projectile_action_t     	        supply_projectile_action;  	     //0x0102
 	referee_warning_t                           referee_warning;                 //0x0104
	dart_info_t                                 dart_remaining_time;             //0x0105
	
  robot_status_t        		                  game_robot_state;      		       //0x0201
  power_heat_data_t   				                power_heat_data;     		 	       //0x0202
  robot_pos_t   				                      game_robot_pos;				           //0x0203
  buff_t						                          buff;						                 //0x0204
  air_support_data_t					                aerial_robot_energy;			       //0x0205
  hurt_data_t						                      robot_hurt;						           //0x0206
  shoot_data_t						                    shoot_data;			           	     //0x0207
	projectile_allowance_t                      bullet_remaining;                //0x0208
	rfid_status_t                               rfid_state;                      //0x0209
	dart_client_cmd_t                           dart_client_cmd;                 //0x020A
	ground_robot_position_t                     ground_robot_position;           //0x020B
	radar_mark_data_t                           radar_mark_data;                 //0x020C
	sentry_info_t                               sentry_info;                     //0x020D
	radar_info_t                                radar_info;                      //0x020E
  //ext_student_interactive_header_data_t   	  student_interactive_header_data; //0x0301
 
	ext_student_interactive_header_data_t       student_interactive_header_data_t;			
  robot_interactive_data_t                    robot_interactive_data;            //0x0302
	map_command1_t                              minimap_interactive_data;          //0x0303
	remote_control_t                            mouse_keyboard_informationt;       //0x0304 //图传链路
	
} judge_rxdata_t;

typedef struct
{
	uint8_t robot_level;
	uint16_t shooter_id1_17mm_speed_limit;
	uint16_t shooter_id1_17mm_cooling_rate;
	uint16_t shooter_id1_17mm_cooling_limit;
	
  uint16_t shooter_id2_17mm_speed_limit;
  uint16_t shooter_id2_17mm_cooling_rate;
  uint16_t shooter_id2_17mm_cooling_limit;
	uint16_t chassis_power_limit;
}judge_data_limit_t;
extern judge_data_limit_t judge_data_limit; 
extern judge_rxdata_t judge_recv_mesg;//读取回来保存在该变量
extern uart_dma_rxdata_t judge_rx_obj;
extern unpack_data_t judge_unpack_obj;

void judgement_rx_param_init(void);
void judgement_data_handler(uint8_t *p_frame);

#endif

