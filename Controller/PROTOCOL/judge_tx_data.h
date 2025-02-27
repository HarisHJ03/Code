#ifndef _judge_tx_data_H
#define _judge_tx_data_H


#include "stm32f4xx.h"
#include "data_packet.h"

#define JUDGE_TX_FIFO_BUFLEN 500//用于给发送邮箱的初始化
#define STUDENT_DATA_LENGTH   sizeof(ext_student_interactive_data_t_t)//交互数据的长度 

//机器人间交互数据包
typedef __packed struct
{
	uint16_t data_cmd_id;//内容ID、发送者ID、接受者ID和交互数据
	uint16_t sender_ID;
	uint16_t receiver_ID;
	uint8_t data[113];//最大113
}ext_student_interactive_data_t_t;

//学生交互id
typedef enum
{
	//学生交互机器人id
	STUDENT_RED_HERO_ID       			= 1,
	STUDENT_RED_ENGINEER_ID	  			= 2,
	STUDENT_RED_INFANTRY3_ID  			= 3,
	STUDENT_RED_INFANTRY4_ID  			= 4,
	STUDENT_RED_INFANTRY5_ID  			= 5,
	STUDENT_RED_AERIAL_ID	    			= 6,
	STUDENT_RED_SENTRY_ID	    			= 7,
	STUDENT_RED_RADAR_ID            = 8,
	STUDENT_RED_RadarStation_ID	    = 9,
	STUDENT_RED_Outpost_ID	    		= 10,//前哨站(前哨和基地的ID用于小地图交互数据)
	STUDENT_RED_Base_ID	    				= 11,//基地
	STUDENT_BLUE_HERO_ID	    			= 101,
	STUDENT_BLUE_ENGINEER_ID  			= 102,
	STUDENT_BLUE_INFANTRY3_ID 			= 103,
	STUDENT_BLUE_INFANTRY4_ID 			= 104,
	STUDENT_BLUE_INFANTRY5_ID 			= 105,
	STUDENT_BLUE_AERIAL_ID    			= 106,
	STUDENT_BLUE_SENTRY_ID	  			= 107,
	STUDENT_BLUE_RADAR_ID           = 108,
	STUDENT_BLUE_RadarStation_ID	  = 109,
	STUDENT_BLUE_Outpost_ID	    		= 110,
	STUDENT_BLUE_Base_ID	    			= 111,
	
	//机器人对应客户端id
	RED_HERO_CLIENT_ID	      = 0x0101,
	RED_ENGINEER_CLIENT_ID    = 0x0102,
	RED_INFANTRY3_CLIENT_ID	  = 0x0103,
	RED_INFANTRY4_CLIENT_ID	  = 0x0104,
	RED_INFANTRY5_CLIENT_ID   = 0x0105,
	RED_AERIAL_CLIENT_ID      = 0x0106,
	BLUE_HERO_CLIENT_ID	      = 0x0165,
	BLUE_ENGINEER_CLIENT_ID   = 0x0166,
	BLUE_INFANTRY3_CLIENT_ID  = 0x0167,
	BLUE_INFANTRY4_CLIENT_ID  = 0x0168,
	BLUE_INFANTRY5_CLIENT_ID  = 0x0169,
	BLUE_AERIAL_CLIENT_ID	    = 0x016A,	
	
	//裁判系统服务器（用于哨兵和雷达自主决策指令）
	JUDGE_SERVER_ID           = 0x8080,
}interactive_id_e;

//内容ID和命令ID
typedef enum
{
	/*stm32是小端模式，比如一个32位无符号数0x12345678,
	从低地址到高地址依次存储78、56、34、12
	
	32bit宽的数0x12345678在小端模式及大端模式CPU中存放的方式（假设地址从0x4000开始存放）为：
	内存地址		小端模式存放内容		大端模式存放内容
	0x4000			0x78								0x12
	0x4001			0x56								0x34
	0x4002			0x34								0x56
	0x4003			0x12								0x78							*/
	//内容ID
	RobotCommunication														= 0x0200,//可以在0x0200~0x02FF中自行添加一个或多个作为交互ID
	Client_Delete_Graph														= 0x0100,
	Client_Draw_One_Graph													= 0x0101,
	Client_Draw_Two_Graph													= 0x0102,
	Client_Draw_Five_Graph												= 0x0103,
	Client_Draw_Seven_Graph												= 0x0104,
	Client_Draw_Character_Graph										= 0x0110,
	/* NEW */
	SENTRY_CMD_ID                                 = 0x0120, //哨兵自主决策指令
	RADAR_CMD_ID                                  = 0x0121, //雷达自主决策指令
  /* NEW */
	//命令ID
	Robot_communicative_data											= 0x0301,
	Custom_control_interactive_port								= 0x0302,
	Client_map_intercactive_data									= 0x0303,
	Keyboard_mouse_data														= 0x0304,
	
}Content_ID;

//客户端删除图形
typedef __packed struct
{
	uint16_t data_cmd_id;//内容ID、发送者ID、接受者ID和图形设置
	uint16_t sender_ID;
	uint16_t receiver_ID;
	uint8_t operate_tpye;
	uint8_t layer;
}ext_client_custom_graphic_delete_t;

//图形数据配置，具体查看裁判系统串口协议附录
typedef __packed struct
{
	uint8_t graphic_name[3];
	uint32_t operate_tpye:3;// 类型说明符 位域名：位域长度,后面的位域长度表示该位域占了多少个位
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11; 
	uint32_t radius:10;
	uint32_t end_x:11;
	uint32_t end_y:11;
}graphic_data_struct_t;

//客户端绘制一个图形
typedef __packed struct
{
	uint16_t data_cmd_id;//内容ID、发送者ID、接受者ID和图形设置
	uint16_t sender_ID;
	uint16_t receiver_ID;
	graphic_data_struct_t grapic_data_struct;
}ext_client_custom_graphic_single_t;

//客户端绘制二个图形
typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
	graphic_data_struct_t grapic_data_struct[2];
}ext_client_custom_graphic_double_t;

//客户端绘制五个图形
typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
	graphic_data_struct_t grapic_data_struct[5];
}ext_client_custom_graphic_five_t;

//客户端绘制七个图形
typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
	graphic_data_struct_t grapic_data_struct[7];
}ext_client_custom_graphic_seven_t;

//客户端绘制字符
typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];//需要发送的字符内容
}ext_client_custom_character_t;

//图形设置
typedef enum
{
	//图形命名，可自行添加
	Fric												  =1,
	DODGE												  =2,
	SENTRY_line									  =3,
	Pitch												  =4,
	line_1s												=5,
	line_2s												=6,
	line_3s												=7,		
	line_4s												=8,
	line_5s												=9,
	line_6s												=10,
	line_7s												=11,
	line_8s												=12,
	line_9s												=13,
	line_10s											=14,
	line_11s											=15,
	line_12s											=16,
	line_13s											=17,
	line_14s											=18,
	line_15s											=19,
	line_16l											=20,
	line_17l											=21,
	line_18l											=22,
	line_19l											=23,
	line_20l											=24,
	line_1v												=25,
	line_1c												=26,
	line_2c												=27,
	line_3c												=28,
	line_4c												=29,
	line_5c												=30,	
	Text													=31,
	Chassis												=32,
	Gimbal												=33,
	Shoot													=34,
	Cap														=35,
	View													=36,
	circle                        =37,
	Distance											=38,
	Compensates                   =39,
	Speed													=40,
	fps                           =41,
	//图形操作
	Null_operate								=0,//空操作
	Delete_graph								=1,//删除图层
	Delete_all									=2,//删除所有
	
	Add													=1,//增加
	Change											=2,//修改
	Delete											=3,//删除
	//图形类型
	Straight_line								=0,//直线
	Rectangle										=1,//矩形
	Circle											=2,//整圆
	ellipse											=3,//椭圆
	Circular_arc								=4,//圆弧
	Floatnumber									=5,//浮点数
	Intnumber										=6,//整形数
	Character										=7,//字符
	//图层数
	layer0											=0,
	layer1											=1,
	layer2											=2,
	layer3											=3,
	layer4											=4,
	layer5											=5,
	layer6											=6,
	layer7											=7,
	layer8											=8,
	layer9											=9,
	//颜色
	Redblue											=0,//红蓝主色
	Yellow											=1,//黄色
	Green												=2,//绿色
	Orange											=3,//橙色
	Amaranth										=4,//紫红色
	Pink												=5,//粉色
	Cyan												=6,//青色
	Black												=7,//黑色
	White												=8,//白色
}client_graphic_draw_operate_data_e;

/* 0301下的0120 */
typedef __packed struct  
{ 
				uint16_t data_cmd_id;//内容ID、发送者ID、接受者ID和图形设置
				uint16_t sender_ID;
				uint16_t receiver_ID;
	      uint32_t confirm_rebirth      :  1; // 是否确认读条复活  0 不复活/1 复活
				uint32_t exchange_rebirth     :  1; // 是否确认兑换复活  0 不兑换/1 兑换（花费金币：[ROUNDUP（（420- 比赛剩余时长）/60）×80+机器人等级×20]金币/1 台）
				uint32_t bullet_number        : 11; // 想要兑换的发弹量  可将其从0修改至X，则消耗X金币成功兑换X允许发弹量。此后哨兵可将其从X修改至X+Y
				uint32_t remote_bullet_times  :  4; // 远程兑换发弹量次数,开始为0，单调递增且每次仅能增加1
				uint32_t remote_blood_times   :  4; // 远程兑换血量的请求次数，开始为0，单调递增且每次仅能增加1 (花费金币：[50+ROUNDUP（（420- 比赛剩余时长） /60）×20]金币/1 次)
				uint32_t reserved             : 11; // 保留位
} sentry_cmd_t;
/*
在哨兵发送该子命令时，服务器将按照从相对低位到相对高位
的原则依次处理这些指令，处理至全部成功或不能处理为止。
举例：若队伍金币数为 0，此时哨兵战亡，“是否确认复活”
的值为 1，“是否想要兑换立即复活”的值为 1，“想要兑换
的发弹量值”为 100。（假定之前哨兵未兑换过发弹量）由于
此时队伍金币数不足以使哨兵兑换立即复活，则服务器将会忽
视后续指令，等待哨兵发送的下一组指令。
bit 21-31：保留
*/
/* 0301下的0121 */
typedef __packed struct
{
uint8_t radar_cmd;
} radar_cmd_t;

/*0307*/
typedef __packed struct
{
 uint8_t intention;
 uint16_t start_position_x;
 uint16_t start_position_y;
 int8_t delta_x[49];
 int8_t delta_y[49];
 uint16_t sender_id;
}map_data_t;

/*0308*/
typedef __packed struct
{ 
 uint16_t sender_id;
 uint16_t receiver_id;
 uint16_t user_data[30];
}custom_info_t;
/** 
  * @brief  the data structure receive from judgement
  */
typedef struct
{
	//0x0200~0x02FF发送的内容
  ext_student_interactive_data_t_t	  					ext_student_interactive_data;
	ext_client_custom_graphic_five_t							ext_client_custom_graphic_long_line;
	ext_client_custom_character_t									ext_client_custom_character_text;
	//0x0100~0x0110发送的内容
	ext_client_custom_graphic_delete_t						ext_client_custom_graphic_delete;
	ext_client_custom_graphic_single_t						ext_client_custom_graphic_single;
	ext_client_custom_graphic_double_t						ext_client_custom_graphic_double;
	ext_client_custom_graphic_five_t							ext_client_custom_graphic_five;
	ext_client_custom_graphic_five_t 							ext_client_custom_graphic_short_line2;
	ext_client_custom_graphic_five_t  						ext_client_custom_graphic_short_line3;
	ext_client_custom_graphic_five_t							ext_client_custom_graphic_car_line;
	ext_client_custom_graphic_seven_t							ext_client_custom_graphic_seven;
	ext_client_custom_character_t									ext_client_custom_character;
	ext_client_custom_character_t									ext_client_custom_character_chassis;//底盘状态
	ext_client_custom_character_t									ext_client_custom_character_shoot;  //射击状态
	ext_client_custom_character_t									ext_client_custom_character_gimbal; //云台状态
	//0x0120~0x0121发送的内容
	sentry_cmd_t                                  sentry_cmd;                         //0x0120 哨兵自主决策指令
	radar_cmd_t                                   radar_cmd;                          //0x0121 雷达自主决策指令
	//0x0307~0x0308发送的内容
	map_data_t                                    map_data;                           //0x0307 发送路径坐标数据    
	custom_info_t                                 custom_info;                        //0x0308 己方机器人端发送自定义消息
} judge_txdata_t;

extern judge_txdata_t judge_send_mesg;
extern fifo_s_t  judge_txdata_fifo;

void judgement_tx_param_init(void);
void judgement_client_packet_pack(uint8_t *p_data);
void sentry_cmd_packet_pack();//uint8_t *p_data
void map_data_packet_pack(uint8_t *p_data);
void judgement_client_graphics_draw_pack(uint8_t text_twist);
static void client_graphic_DODGE_low_voltage (void);
static void client_graphic_draw_accelerate (void);
static void client_graphic_delete_accelerate (void);
static void client_graphic_draw_pitch (void);
static void client_graphic_draw_5m_line (void);
extern float pit_set,yaw_set;
extern float cap_store;
#endif 

