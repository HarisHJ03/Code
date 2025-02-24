#include "judge_tx_data.h"
#include "dma.h"
#include "judge_rx_data.h"
#include "string.h"
#include "shoot_task.h"
#include "remote_ctrl.h"
#include "chassis_task.h"
#include "modeswitch_task.h"
#include "gimbal_task.h"
#include "math.h"
#include "pc_rx_data.h"

judge_txdata_t judge_send_mesg;//裁判系统发送结构体，用户自定义发送的数据打包在里面，到时候可以在debug里查看与修改
static SemaphoreHandle_t judge_txdata_mutex;
fifo_s_t  judge_txdata_fifo;//用来暂时存放数据的邮箱
static uint8_t   judge_txdata_buf[JUDGE_TX_FIFO_BUFLEN];
/////////////
extern float nowfps;
////////
void judgement_tx_param_init(void)
{
  /* create the judge_rxdata_mutex mutex  */  
  judge_txdata_mutex = xSemaphoreCreateMutex();
  
  /* judge data fifo init */
  fifo_s_init(&judge_txdata_fifo, judge_txdata_buf, JUDGE_TX_FIFO_BUFLEN, judge_txdata_mutex);
}

//#define Hero 1 //步兵用此处要注释掉，英雄用此处为1 不同处在准心 !!!
#ifdef Hero
static void client_graphic_draw_short_line1(void);
#else
static void client_graphic_draw_short_line1(void);
static void client_graphic_draw_Circle(void);	
#endif

/*以下数据均是需要看实际显示效果修改的，所以在绘图时可以先用它赋值给结构体，便于我们在debug中改变结构体的
	值，等合适的值确定下来，再将这个确定的值以常数赋值给结构体*/
uint32_t debug_start_angle = 0;
uint32_t debug_end_angle = 0;
uint32_t debug_width = 0;
uint32_t debug_start_x = 800;
uint32_t debug_start_y = 0;
uint32_t debug_radius = 0;
uint32_t debug_end_x = 1400;
uint32_t debug_end_y = 0;

/**
参数——
num1：浮点数电压，pit用;num1、num2补偿角用 
name: Cap,Pitch; type: ADD,Change; color: 颜色;
start_x、start_y: 起始坐标  
**/
static void client_grapjic_draw_float(float num1,float num2,char name,char type,char layer,char color,uint16_t start_x,uint16_t start_y)
{
	char float_string[20] = {0};
	if(name == Pitch) 
  {
    if(num1>=0)strcpy(float_string ,"Pit: 00.0");
		if(num1<0) strcpy(float_string ,"Pit:-00.0");
  }
	if(name == Speed) 
    strcpy(float_string ,"Spd:0000\n\nDog:000");

	if(name == Cap) 
		strcpy(float_string ,"ALL:00.0\n\nCap:00.0");
	////////////////////
	if(name == fps) 
	  strcpy(float_string ,"FPS:00.0\n");	
	////////////////////
	if(name == Distance)
		strcpy(float_string ,"Dis: 00.0");
	if(name == Compensates)//补偿yaw、pit
  { 
    if(num1>=0 && num2>=0)strcpy(float_string ,"Y: 00.0\n\nP: 00.0");
		if(num1<0 && num2>=0) strcpy(float_string ,"Y:-00.0\n\nP: 00.0");
    if(num1>=0 && num2<0) strcpy(float_string ,"Y: 00.0\n\nP:-00.0");
    if(num1<0 && num2<0)  strcpy(float_string ,"Y:-00.0\n\nP:-00.0");
  }
	if(name == Pitch||name == Distance)
	{
		float_string[5] = (uint8_t)fabs(num1)/10+48; //48——'0'ASCII码
		float_string[6] = (uint8_t)fabs(num1)%10+48;
		float_string[7] = '.';
		float_string[8] = (uint8_t)(fmod(fabs(num1)*10.0,10.0)+48);
	}
	else if(name == Cap){
		
		float_string[4] = (uint8_t)fabs(num1)/10+48; //48——'0'ASCII码
		float_string[5] = (uint8_t)fabs(num1)%10+48;
		float_string[6] = '.';
		float_string[7] = (uint8_t)(fmod(fabs(num1)*10.0,10.0)+48);

		float_string[14] = (uint8_t)fabs(num2)/10+48; //48——'0'ASCII码
		float_string[15] = (uint8_t)fabs(num2)%10+48;
		float_string[16] = '.';
		float_string[17] = (uint8_t)(fmod(fabs(num2)*10.0,10.0)+48);
	}
	/////////////////////////////
		else if(name == fps){
		
		float_string[4] = (uint8_t)fabs(num1)/10+48; //48——'0'ASCII码
		float_string[5] = (uint8_t)fabs(num1)%10+48;
		float_string[6] = '.';
		float_string[7] = (uint8_t)(fmod(fabs(num1)*10.0,10.0)+48);	
	
	
	}
	///////////////////////////
	else if(name == Compensates)
	{
		float_string[3] = (uint8_t)fabs(num1)/10+48; //48——'0'ASCII码
		float_string[4] = (uint8_t)fabs(num1)%10+48;
		float_string[5] = '.';
		float_string[6] = (uint8_t)(fmod(fabs(num1)*10.0,10.0)+48);	

		float_string[12] = (uint8_t)fabs(num2)/10+48; //48——'0'ASCII码
		float_string[13] = (uint8_t)fabs(num2)%10+48;
		float_string[14] = '.';
		float_string[15] = (uint8_t)(fmod(fabs(num2)*10.0,10.0)+48);		
	}else if(name == Speed){

		float_string[4] = (uint16_t)fabs(num1)/1000+48; //48——'0'ASCII码
		float_string[5] = (uint16_t)fabs(num1)/100-(uint16_t)fabs(num1)/1000*10+48;
		float_string[6] = (uint16_t)fabs(num1)/10-(uint16_t)fabs(num1)/100*10+48;
		float_string[7] = (uint16_t)fabs(num1)%10+48;
		
		float_string[14] = (uint16_t)fabs(num2)/100+48;
		float_string[15] = (uint16_t)fabs(num2)/10-(uint16_t)fabs(num2)/100*10+48;
		float_string[16] = (uint16_t)fabs(num2)%10+48;
		
	}
	memcpy(&judge_send_mesg.ext_client_custom_character.data[0],float_string,sizeof(float_string));
	judge_send_mesg.ext_client_custom_character.grapic_data_struct.graphic_name[0] = name;
	judge_send_mesg.ext_client_custom_character.grapic_data_struct.operate_tpye = type;
	judge_send_mesg.ext_client_custom_character.grapic_data_struct.graphic_tpye = Character;
	judge_send_mesg.ext_client_custom_character.grapic_data_struct.layer = layer;
	judge_send_mesg.ext_client_custom_character.grapic_data_struct.color =  color ;
	judge_send_mesg.ext_client_custom_character.grapic_data_struct.start_angle =  20 ;
	judge_send_mesg.ext_client_custom_character.grapic_data_struct.end_angle =  12 ;
	judge_send_mesg.ext_client_custom_character.grapic_data_struct.width = 3;
	judge_send_mesg.ext_client_custom_character.grapic_data_struct.start_x = start_x;
	judge_send_mesg.ext_client_custom_character.grapic_data_struct.start_y = start_y;
	judge_send_mesg.ext_client_custom_character.grapic_data_struct.radius = 0;
	judge_send_mesg.ext_client_custom_character.grapic_data_struct.end_x = 0;
	judge_send_mesg.ext_client_custom_character.grapic_data_struct.end_y = 0;		
}

/**
参数——
str：字符; name: Gimbal,Chassis,Shoot; type: ADD,Change; color: 颜色;
start_x、start_y: 起始坐标  
**/
static void client_graphic_draw_String(char *str,char name,char type,char layer,char color,uint16_t start_x,uint16_t start_y)
{
	char string[15] = {0};
	strcpy(string,str);
	if(name == Gimbal) 
	{
		memcpy(&judge_send_mesg.ext_client_custom_character_gimbal.data[0], string, sizeof(string));
		judge_send_mesg.ext_client_custom_character_gimbal.grapic_data_struct.graphic_name[0] = name;
		judge_send_mesg.ext_client_custom_character_gimbal.grapic_data_struct.operate_tpye = type;
		judge_send_mesg.ext_client_custom_character_gimbal.grapic_data_struct.graphic_tpye = Character;
		judge_send_mesg.ext_client_custom_character_gimbal.grapic_data_struct.layer = layer;
		judge_send_mesg.ext_client_custom_character_gimbal.grapic_data_struct.color =  color ;
		judge_send_mesg.ext_client_custom_character_gimbal.grapic_data_struct.start_angle =  20 ;
		judge_send_mesg.ext_client_custom_character_gimbal.grapic_data_struct.end_angle =sizeof(string) ;
		judge_send_mesg.ext_client_custom_character_gimbal.grapic_data_struct.width = 3;
		judge_send_mesg.ext_client_custom_character_gimbal.grapic_data_struct.start_x = start_x;
		judge_send_mesg.ext_client_custom_character_gimbal.grapic_data_struct.start_y = start_y;
		judge_send_mesg.ext_client_custom_character_gimbal.grapic_data_struct.radius = 0;
		judge_send_mesg.ext_client_custom_character_gimbal.grapic_data_struct.end_x = 0;
		judge_send_mesg.ext_client_custom_character_gimbal.grapic_data_struct.end_y = 0;	
	}
	else if(name == Chassis)
	{
		memcpy(&judge_send_mesg.ext_client_custom_character_chassis.data[0], string, sizeof(string));
		judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.graphic_name[0] = name;
		judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.operate_tpye = type;
		judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.graphic_tpye = Character;
		judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.layer = layer;
		judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.color =  color;
		judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_angle =  20 ;
		judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_angle =sizeof(string) ;
		judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.width = 3;
		judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_x = start_x;
		judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_y = start_y;
		judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.radius = 0;
		judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_x = 0;
		judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_y = 0;		
	}
	else if(name == Shoot)
	{
		memcpy(&judge_send_mesg.ext_client_custom_character_shoot.data[0], string, sizeof(string));
		judge_send_mesg.ext_client_custom_character_shoot.grapic_data_struct.graphic_name[0] = name;
		judge_send_mesg.ext_client_custom_character_shoot.grapic_data_struct.operate_tpye = type;
		judge_send_mesg.ext_client_custom_character_shoot.grapic_data_struct.graphic_tpye = Character;
		judge_send_mesg.ext_client_custom_character_shoot.grapic_data_struct.layer = layer;
		judge_send_mesg.ext_client_custom_character_shoot.grapic_data_struct.color =  color;
		judge_send_mesg.ext_client_custom_character_shoot.grapic_data_struct.start_angle =  20 ;
		judge_send_mesg.ext_client_custom_character_shoot.grapic_data_struct.end_angle =sizeof(string) ;
		judge_send_mesg.ext_client_custom_character_shoot.grapic_data_struct.width = 3;
		judge_send_mesg.ext_client_custom_character_shoot.grapic_data_struct.start_x = start_x;
		judge_send_mesg.ext_client_custom_character_shoot.grapic_data_struct.start_y = start_y;
		judge_send_mesg.ext_client_custom_character_shoot.grapic_data_struct.radius = 0;
		judge_send_mesg.ext_client_custom_character_shoot.grapic_data_struct.end_x = 0;
		judge_send_mesg.ext_client_custom_character_shoot.grapic_data_struct.end_y = 0;		
	}
	else //视觉有效位
	{
		memcpy(&judge_send_mesg.ext_client_custom_character.data[0], string, sizeof(string));
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.graphic_name[0] = name;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.operate_tpye = type;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.graphic_tpye = Character;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.layer = layer;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.color =  color ;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.start_angle =  20;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.end_angle =sizeof(string) ;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.width = 3;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.start_x = start_x;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.start_y = start_y;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.radius = 0;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.end_x = 0;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.end_y = 0;	
		
	}
}

static void client_graphic_draw_car_line(void)//车身线 图层4
{
	
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[0].graphic_name[0] = line_1c;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[0].operate_tpye = Add;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[0].graphic_tpye = Straight_line;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[0].layer = layer4;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[0].color = Cyan;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[0].start_angle = 0;	
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[0].end_angle = 0;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[0].width = 40;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[0].start_x = 665;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[0].start_y = 200;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[0].radius = 0;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[0].end_x = 1255;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[0].end_y = 200;
	
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[1].graphic_name[0] = line_2c;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[1].operate_tpye = Add;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[1].graphic_tpye = Straight_line;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[1].layer = layer4;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[1].color = Black;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[1].start_angle = 0;	
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[1].end_angle = 0;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[1].width = 30;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[1].start_x = 670;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[1].start_y = 200;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[1].radius = 0;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[1].end_x = 1250;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[1].end_y = 200;

	float length=670+(chassis.CapData[1]-14)/(cap_store-14)*(1250-670);
	if(length<=670)length=670;
	if(length>=1250)length=1250;
	uint32_t color;
	if(length>670&&length<=800)color=Pink;
	if(length>800&&length<=1000)color=Orange;
	if(length>1000&&length<=1250)color=Redblue;
	
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].graphic_name[0] = line_3c;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].operate_tpye = Change;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].graphic_tpye = Straight_line;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].layer = layer4;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].color = color;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].start_angle = 0;	
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].end_angle = 0;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].width = 30;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].start_x = 670;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].start_y = 200;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].radius = 0;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].end_x = length;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].end_y = 200;


}

static void client_graphic_draw_line(void){

	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].graphic_name[0] = line_3c;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].operate_tpye = Add;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].graphic_tpye = Straight_line;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].layer = layer4;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].color = Redblue;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].start_angle = 0;	
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].end_angle = 0;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].width = 30;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].start_x = 670;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].start_y = 200;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].radius = 0;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].end_x = 1250;
	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].end_y = 200;


}
	
	
	
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[0].graphic_name[0] = line_1c;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[0].operate_tpye = Add;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[0].graphic_tpye = Straight_line;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[0].layer = layer4;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[0].color = Redblue;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[0].start_angle = 0;	
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[0].end_angle = 0;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[0].width = 3;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[0].start_x = 600;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[0].start_y = 2;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[0].radius = 0;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[0].end_x = 675;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[0].end_y = 90;
//	
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[1].graphic_name[0] = line_2c;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[1].operate_tpye = Add;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[1].graphic_tpye = Straight_line;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[1].layer = layer4;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[1].color = Redblue;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[1].start_angle = 0;	
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[1].end_angle = 0;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[1].width = 3;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[1].start_x = 750;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[1].start_y = 180;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[1].radius = 0;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[1].end_x = 1200;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[1].end_y = 180;
//	
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].graphic_name[0] = line_3c;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].operate_tpye = Add;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].graphic_tpye = Straight_line;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].layer = layer4;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].color = Redblue;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].start_angle = 0;	
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].end_angle = 0;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].width = 3;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].start_x = 1200;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].start_y = 180;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].radius = 0;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].end_x = 1275;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[2].end_y = 90;
//	
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[3].graphic_name[0] = line_4c;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[3].operate_tpye = Add;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[3].graphic_tpye = Straight_line;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[3].layer = layer4;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[3].color = Redblue;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[3].start_angle = 0;	
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[3].end_angle = 0;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[3].width = 3;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[3].start_x = 1275;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[3].start_y = 90;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[3].radius = 0;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[3].end_x = 1350;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[3].end_y = 2;

//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[4].graphic_name[0] = line_5c;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[4].operate_tpye = Add;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[4].graphic_tpye = Straight_line;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[4].layer = layer4;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[4].color = Redblue;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[4].start_angle = 0;	
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[4].end_angle = 0;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[4].width = 3;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[4].start_x = 675;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[4].start_y = 90;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[4].radius = 0;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[4].end_x = 750;
//	judge_send_mesg.ext_client_custom_graphic_car_line.grapic_data_struct[4].end_y = 180;
//}
			 int cha=0;
			 int last_chassis=0;
			 int gim=0;
			 int last_gimbal=0;
			 int sho=0;
			 int last_shoot=0;
			 int view=0;
			 int last_view=0;
			 int c=0,g=0,s=0,v=0;



//客户端自定义UI界面
uint8_t Line_mask = 0;
uint8_t text_mask = 1;
uint8_t Chassis_mask = 0;
uint8_t Gimbal_mask = 0;
uint8_t Shoot_mask = 0;
uint8_t	view_mask = 0;	
uint8_t Idle_mask = 0;//空闲标志
void judgement_client_graphics_draw_pack(uint8_t text_twist)
{
	uint8_t current_robot_id = 0;
	uint16_t receiver_ID = 0;
	static uint8_t i;
	static uint8_t again_flag;
	
	current_robot_id = judge_recv_mesg.game_robot_state.robot_id;  //读取当前机器人的id
	//自定义UI receiver_ID只能选当前机器人的对应的客户端
	switch(current_robot_id)
	{
		//red robot
		case STUDENT_RED_HERO_ID:
		{
			receiver_ID = RED_HERO_CLIENT_ID;
		}break;
		case STUDENT_RED_ENGINEER_ID:
		{
			receiver_ID = RED_ENGINEER_CLIENT_ID;
		}break;
		case STUDENT_RED_AERIAL_ID:
		{
			receiver_ID = RED_AERIAL_CLIENT_ID;
		}break;

		case STUDENT_RED_INFANTRY3_ID:
		{
			receiver_ID = RED_INFANTRY3_CLIENT_ID;
		}break;
		case STUDENT_RED_INFANTRY4_ID:
		{
			receiver_ID = RED_INFANTRY4_CLIENT_ID;
		}break;
		case STUDENT_RED_INFANTRY5_ID:
		{
			receiver_ID = RED_INFANTRY5_CLIENT_ID;
		}break;
		
		//blue robot
		case STUDENT_BLUE_HERO_ID:
		{
			receiver_ID = BLUE_HERO_CLIENT_ID;
		}break;
		case STUDENT_BLUE_ENGINEER_ID:
		{
			receiver_ID = BLUE_ENGINEER_CLIENT_ID;
		}break;
		case STUDENT_BLUE_AERIAL_ID:
		{
			receiver_ID = BLUE_AERIAL_CLIENT_ID;
		}break;
		case STUDENT_BLUE_INFANTRY3_ID:
		{
			receiver_ID = BLUE_INFANTRY3_CLIENT_ID;
		}break;	
		case STUDENT_BLUE_INFANTRY4_ID:
		{
			receiver_ID = BLUE_INFANTRY4_CLIENT_ID;
		}break;
		case STUDENT_BLUE_INFANTRY5_ID:
		{
			receiver_ID = BLUE_INFANTRY5_CLIENT_ID;
		}break;

	}
	
	//清除数组
	memset(&judge_send_mesg.ext_client_custom_character.data[0],0,sizeof(judge_send_mesg.ext_client_custom_character.data[0]));
	memset(&judge_send_mesg.ext_client_custom_character_chassis.data[0],0,sizeof(judge_send_mesg.ext_client_custom_character_chassis.data[0]));
	memset(&judge_send_mesg.ext_client_custom_character_shoot.data[0],0,sizeof(judge_send_mesg.ext_client_custom_character_shoot.data[0]));
	memset(&judge_send_mesg.ext_client_custom_character_gimbal.data[0],0,sizeof(judge_send_mesg.ext_client_custom_character_gimbal.data[0]));
	
	if(rc.sw2 == RC_DN && rc.sw1 == RC_MI && !again_flag  )//打中打下可重复刷新，提高容错率
	{
		Line_mask = 1;
		text_mask = 1;
		again_flag = 1;
	}
	else if(!(rc.sw2 == RC_DN && rc.sw1 == RC_MI))
		again_flag = 0;

	
	if(rc.sw2 == RC_MI) //启动机器人不画UI防止复活后重复画准心，英雄打到上档RC_UP，步兵打中档RC_MI
		Line_mask = 4;
	
	//画线
	if(Line_mask == 1)
	{
		if(i++ < 5)
		{
			judge_send_mesg.ext_client_custom_graphic_five.data_cmd_id = Client_Draw_Five_Graph;
			judge_send_mesg.ext_client_custom_graphic_five.sender_ID = (uint16_t)current_robot_id;
			judge_send_mesg.ext_client_custom_graphic_five.receiver_ID = receiver_ID;
			client_graphic_draw_short_line1 ();
			data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_graphic_five,
										sizeof(judge_send_mesg.ext_client_custom_graphic_five), DN_REG_ID);
		}
		else
		{
			Line_mask = 2;
			i = 0;
		}
	}
	if(Line_mask == 2)
	{
		if(i++ < 5)
		{	
			judge_send_mesg.ext_client_custom_graphic_car_line.data_cmd_id = Client_Draw_Five_Graph;
			judge_send_mesg.ext_client_custom_graphic_car_line.sender_ID = (uint16_t)current_robot_id;
			judge_send_mesg.ext_client_custom_graphic_car_line.receiver_ID = receiver_ID;
			client_graphic_draw_line();
			data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_graphic_car_line,
										sizeof(judge_send_mesg.ext_client_custom_graphic_car_line), DN_REG_ID);
		}
		else
		{
			Line_mask = 3;
			i = 0;
		}
	}

	if(Line_mask == 3)
	{
		if(i++ < 5)
		{	
		    judge_send_mesg.ext_client_custom_graphic_single.data_cmd_id = Client_Draw_One_Graph;
			judge_send_mesg.ext_client_custom_graphic_single.sender_ID = (uint16_t)current_robot_id;
			judge_send_mesg.ext_client_custom_graphic_single.receiver_ID = receiver_ID;
			client_graphic_draw_Circle ();//步兵圆形准心
			data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_graphic_single,
											sizeof(judge_send_mesg.ext_client_custom_graphic_single), DN_REG_ID);
		}
		else
		{	
			Line_mask = 4;
			i = 0;
		}
	}	


	//画文字
	
	if(Line_mask == 4)
	{
		if(text_mask == 1) //Add
		{
			i++;
			if(i >= 1 && i<6) //图层3 底盘
			{
				judge_send_mesg.ext_client_custom_character_chassis.data_cmd_id = Client_Draw_Character_Graph;
				judge_send_mesg.ext_client_custom_character_chassis.sender_ID = (uint16_t)current_robot_id;
				judge_send_mesg.ext_client_custom_character_chassis.receiver_ID = receiver_ID;
				client_graphic_draw_String("Cha: NOR",Chassis,Add,layer3,Green,80,830);
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_chassis,
												sizeof(judge_send_mesg.ext_client_custom_character_chassis), DN_REG_ID);
			}	
			else if(i >= 6 && i< 11)//云台
			{
				judge_send_mesg.ext_client_custom_character_gimbal.data_cmd_id = Client_Draw_Character_Graph;
				judge_send_mesg.ext_client_custom_character_gimbal.sender_ID = (uint16_t)current_robot_id;
				judge_send_mesg.ext_client_custom_character_gimbal.receiver_ID = receiver_ID;
				client_graphic_draw_String("Gim: NOR",Gimbal,Add,layer3,Green,80,770);
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_gimbal,
												sizeof(judge_send_mesg.ext_client_custom_character_gimbal), DN_REG_ID);
			}
			else if(i >= 11 && i<16)//发射
			{				
				judge_send_mesg.ext_client_custom_character_shoot.data_cmd_id = Client_Draw_Character_Graph;
				judge_send_mesg.ext_client_custom_character_shoot.sender_ID = (uint16_t)current_robot_id;
				judge_send_mesg.ext_client_custom_character_shoot.receiver_ID = receiver_ID;	
				client_graphic_draw_String("FR:F,BAL:F",Shoot,Add,layer3,Green,80,890);
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_shoot,
												sizeof(judge_send_mesg.ext_client_custom_character_shoot), DN_REG_ID);
			}
			else if(i == 16)//Cap、View、Pit
				client_grapjic_draw_float(cap_store,chassis.CapData[1],Cap,Add,layer2,Green,1100,620); //图层2
			else if(i == 17)
				client_grapjic_draw_float(gimbal.sensor.pit_relative_angle,0,Pitch,Add,layer2,Redblue,1400,890);
			else if(i == 18)
			{
				memset(&judge_send_mesg.ext_client_custom_character,0,sizeof(judge_send_mesg.ext_client_custom_character));//清空数组
				client_graphic_draw_String("View:F",View,Add,layer2,Pink,1100,467);
			}
			else if(i == 19)//补偿
			{
				memset(&judge_send_mesg.ext_client_custom_character,0,sizeof(judge_send_mesg.ext_client_custom_character));//清空数组
				client_grapjic_draw_float(yaw_set,pit_set,Compensates,Add,layer2,Yellow,1600,890);			
			}
			else if(i == 20)
			{
				client_grapjic_draw_float(chassis.vx,chassis.vw,Speed,Add,layer2,Green,1400,830);
			}
			//////////////////////////
			else if(i == 21){
			    client_grapjic_draw_float(nowfps,0,fps,Add,layer2,Yellow,1100,670);
				
			}
			///////////////
			else
			{
				text_mask = 2;
				i = 0;
			}
			
			
			
			if(i >= 16)
			{
				judge_send_mesg.ext_client_custom_character.data_cmd_id = Client_Draw_Character_Graph;
				judge_send_mesg.ext_client_custom_character.sender_ID = (uint16_t)current_robot_id;
				judge_send_mesg.ext_client_custom_character.receiver_ID = receiver_ID;	
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character,
												sizeof(judge_send_mesg.ext_client_custom_character), DN_REG_ID);				
			}
		}
		if(text_mask == 2)//Change
		{
			judge_send_mesg.ext_client_custom_character_chassis.data_cmd_id = Client_Draw_Character_Graph;
			judge_send_mesg.ext_client_custom_character_chassis.sender_ID = (uint16_t)current_robot_id;
			judge_send_mesg.ext_client_custom_character_chassis.receiver_ID = receiver_ID;

			judge_send_mesg.ext_client_custom_character_gimbal.data_cmd_id = Client_Draw_Character_Graph;
			judge_send_mesg.ext_client_custom_character_gimbal.sender_ID = (uint16_t)current_robot_id;
			judge_send_mesg.ext_client_custom_character_gimbal.receiver_ID = receiver_ID;

			judge_send_mesg.ext_client_custom_character_shoot.data_cmd_id = Client_Draw_Character_Graph;
			judge_send_mesg.ext_client_custom_character_shoot.sender_ID = (uint16_t)current_robot_id;
			judge_send_mesg.ext_client_custom_character_shoot.receiver_ID = receiver_ID;
			
			memset(&judge_send_mesg.ext_client_custom_character,0,sizeof(judge_send_mesg.ext_client_custom_character));//清空数组
			judge_send_mesg.ext_client_custom_character.data_cmd_id = Client_Draw_Character_Graph;
			judge_send_mesg.ext_client_custom_character.sender_ID = (uint16_t)current_robot_id;
			judge_send_mesg.ext_client_custom_character.receiver_ID = receiver_ID;


			
			switch(chassis_mode){
				case CHASSIS_DODGE_MODE:{
				cha=1;
					break;
				}
				case CHASSIS_NORMAL_MODE:{
				cha=2;
					break;
				}
				default :{
				break;};
			}
			
			switch(gimbal_mode){
				case GIMBAL_TRACK_ARMOR:{
				gim=1;
					break;
				}
				case GIMBAL_NORMAL_MODE:{
				gim=2;
					break;
				}
				case GIMBAL_SHOOT_BUFF :{
				gim=3;
					break;
				}
				default :{
				break;};
			}
			
			if(shoot.fric_wheel_run == 1 && shoot.ball_storage_open == 1 )
				sho=1;
			if(shoot.fric_wheel_run == 1 && shoot.ball_storage_open == 0)
				sho=2;
			if(shoot.fric_wheel_run == 0 && shoot.ball_storage_open == 0)
				sho=3;
			if(shoot.fric_wheel_run == 0 && shoot.ball_storage_open == 1)
				sho=4;

			if (pc_recv_mesg.mode_Union.info.visual_valid == 1)
			{
				view = 1;
			}
			else
				view = 2;

			if(cha!=last_chassis||gim!=last_gimbal||sho!=last_shoot||view!=last_view
				||c<5||g<5||s<5||v<5){
				
				if(cha!=last_chassis)c=0;
				if(gim!=last_gimbal)g=0;
				if(sho!=last_shoot)s=0;
				if(view!=last_view)v=0;
				
				if(c<5){
					
			switch(cha){
				case 1:
				{
				client_graphic_draw_String("Cha: DOG",Chassis,Change,layer3,Pink,80,830);
				Idle_mask = 0;
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_chassis,
												sizeof(judge_send_mesg.ext_client_custom_character_chassis), DN_REG_ID);
					break;
				}

				case 2:{
				client_graphic_draw_String("Cha: NOR",Chassis,Change,layer3,Green,80,830);
				Idle_mask = 0;
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_chassis,
				sizeof(judge_send_mesg.ext_client_custom_character_chassis), DN_REG_ID);
					break;
				}
				
				default :{
				break;};

			}
			
			c++;

		}else{
				if(g<5){
					
			switch(gim){
				case 1:{
				client_graphic_draw_String("Gim: TRA",Gimbal,Change,layer3,Pink,80,770);
				Idle_mask = 0;
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_gimbal,
												sizeof(judge_send_mesg.ext_client_custom_character_gimbal), DN_REG_ID);		
					break;
				}

				case 2 :{
				client_graphic_draw_String("Gim: NOR",Gimbal,Change,layer3,Green,80,770);
				Idle_mask = 0;
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_gimbal,
												sizeof(judge_send_mesg.ext_client_custom_character_gimbal), DN_REG_ID);	
					break;
				}

					case 3 :{
				client_graphic_draw_String("Gim: BUF",Gimbal,Change,layer3,Orange,80,770);
				Idle_mask = 0;
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_gimbal,
												sizeof(judge_send_mesg.ext_client_custom_character_gimbal), DN_REG_ID);			
					break;
				}
				
				default :{
				break;};

			}
			g++;
		}else{
		
			if(s<5){
				switch(sho){
				case 1:{
				client_graphic_draw_String("FR:T,BAL:T",Shoot,Change,layer3,Pink,80,890);
				Idle_mask = 0;
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_shoot,
												sizeof(judge_send_mesg.ext_client_custom_character_shoot), DN_REG_ID);
					break;
				}

				case 2 :{
				client_graphic_draw_String("FR:T,BAL:F",Shoot,Change,layer3,Pink,80,890);
				Idle_mask = 0;
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_shoot,
												sizeof(judge_send_mesg.ext_client_custom_character_shoot), DN_REG_ID);		
					break;
				}

					case 3 :{
				client_graphic_draw_String("FR:F,BAL:F",Shoot,Change,layer3,Green,80,890);
				Idle_mask = 0;
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_shoot,
												sizeof(judge_send_mesg.ext_client_custom_character_shoot), DN_REG_ID);			
					break;
				}
					
					case 4 :{
				client_graphic_draw_String("FR:F,BAL:T",Shoot,Change,layer3,Pink,80,890);
				Idle_mask = 0;
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_shoot,
												sizeof(judge_send_mesg.ext_client_custom_character_shoot), DN_REG_ID);
					break;
				}
				
				
				default :{
				break;};

			}
			s++;
			}else{
		
				if(v<5){
			switch(view){
				case 1:
				{
				client_graphic_draw_String("View:T",View,Change,layer2,Green,1100,467);
				Idle_mask = 0;
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character,
				sizeof(judge_send_mesg.ext_client_custom_character), DN_REG_ID);		
					break;
				}

				case 2:{

				client_graphic_draw_String("View:F",View,Change,layer2,Pink,1100,467);
				Idle_mask = 0;
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character,
													sizeof(judge_send_mesg.ext_client_custom_character), DN_REG_ID);		
					break;
				}
				
				default :{
				break;};

			}	
				v++;	
		}
		}
		}
		}
		}

			last_chassis=cha;
			last_gimbal=gim;
			last_shoot=sho;
			last_view=view;


			if(Idle_mask == 1) //空闲时执行
			{
				if(text_twist == 1 && gimbal_mode != GIMBAL_SHOOT_BUFF) //实时显示电容 ，打能量机关除外
				{
					if(chassis.CapData[1]<14.5f)
						client_grapjic_draw_float(cap_store,chassis.CapData[1],Cap,Change,layer2,Yellow,1100,620);
					else if(chassis.CapData[1]>14.5f)
						client_grapjic_draw_float(cap_store,chassis.CapData[1],Cap,Change,layer2,Green,1100,620);
					
					data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character,
													sizeof(judge_send_mesg.ext_client_custom_character), DN_REG_ID);					
				}
				else if (text_twist == 1 && gimbal_mode == GIMBAL_SHOOT_BUFF)//打能量机关，实时显示补偿
				{
					client_grapjic_draw_float(pc_recv_mesg.aim_yaw,pc_recv_mesg.aim_pitch,Compensates,Change,layer2,Yellow,1600,890);	
					data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character,
													sizeof(judge_send_mesg.ext_client_custom_character), DN_REG_ID);					
				}
				//////////////////////////////////////
				if(text_twist == 1) //实时显示fps
				{
					client_grapjic_draw_float(nowfps,0,fps,Change,layer2,Yellow,1100,670);
					data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character,
					sizeof(judge_send_mesg.ext_client_custom_character), DN_REG_ID);
				}
				//////////////////////////////////////
				if(text_twist == 2) //实时显示Pitch
				{
					client_grapjic_draw_float(gimbal.sensor.pit_relative_angle,0,Pitch,Change,layer2,Redblue,1400,890);
					data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character,
													sizeof(judge_send_mesg.ext_client_custom_character), DN_REG_ID);					
				}
				if(text_twist == 3)
				{
					static float num=0;
					if(chassis.vx)num=chassis.vx;
					else num=chassis.vy;
					client_grapjic_draw_float(num,chassis.vw,Speed,Change,layer2,Green,1400,830);
					data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character,
													sizeof(judge_send_mesg.ext_client_custom_character), DN_REG_ID);					
				}
				
				if(text_twist == 4)
				{
			judge_send_mesg.ext_client_custom_graphic_car_line.data_cmd_id = Client_Draw_Five_Graph;
			judge_send_mesg.ext_client_custom_graphic_car_line.sender_ID = (uint16_t)current_robot_id;
			judge_send_mesg.ext_client_custom_graphic_car_line.receiver_ID = receiver_ID;
			
			client_graphic_draw_car_line();
			
			data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_graphic_car_line,
			sizeof(judge_send_mesg.ext_client_custom_graphic_car_line), DN_REG_ID);			
					
				}
				
				
			}
			Idle_mask = 1;			
		}
		
	}	
}

static void client_graphic_draw_short_line1(void) //步兵准星，图层7
{
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[0].graphic_name[0] = line_1s;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[0].operate_tpye = Add;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[0].graphic_tpye = Straight_line;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[0].layer = layer7;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[0].color = Yellow;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[0].start_angle = 0;	
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[0].end_angle = 0;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[0].width = 1;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[0].start_x = 910;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[0].start_y = 430;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[0].radius = 0;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[0].end_x = 1010;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[0].end_y = 430;
	
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[1].graphic_name[0] = line_2s;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[1].operate_tpye = Add;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[1].graphic_tpye = Straight_line;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[1].layer = layer7;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[1].color = Yellow;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[1].start_angle = 0;	
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[1].end_angle = 0;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[1].width = 1;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[1].start_x = 880;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[1].start_y = 475;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[1].radius = 0;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[1].end_x = 1040;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[1].end_y = 475;
	
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[2].graphic_name[0] = line_3s;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[2].operate_tpye = Add;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[2].graphic_tpye = Straight_line;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[2].layer = layer7;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[2].color = Yellow;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[2].start_angle = 0;	
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[2].end_angle = 0;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[2].width = 1;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[2].start_x = 910;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[2].start_y = 487;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[2].radius = 0;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[2].end_x =1010;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[2].end_y = 487;
	
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[3].graphic_name[0] = line_4s;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[3].operate_tpye = Add;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[3].graphic_tpye = Straight_line;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[3].layer = layer7;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[3].color = Yellow;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[3].start_angle = 0;	
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[3].end_angle = 0;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[3].width = 1;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[3].start_x = 880;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[3].start_y = 500;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[3].radius = 0;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[3].end_x = 1040;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[3].end_y = 500;
	
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[4].graphic_name[0] = line_5s;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[4].operate_tpye = Add;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[4].graphic_tpye = Straight_line;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[4].layer = layer7;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[4].color = Yellow;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[4].start_angle = 0;	
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[4].end_angle = 0;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[4].width = 1;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[4].start_x = 960;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[4].start_y = 600;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[4].radius = 0;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[4].end_x = 960;
	judge_send_mesg.ext_client_custom_graphic_five.grapic_data_struct[4].end_y = 380;
}

static void client_graphic_draw_Circle(void) //步兵准心2 图层6
{
	judge_send_mesg.ext_client_custom_graphic_single.grapic_data_struct.graphic_name[0] = circle;
	judge_send_mesg.ext_client_custom_graphic_single.grapic_data_struct.operate_tpye = Add;
	judge_send_mesg.ext_client_custom_graphic_single.grapic_data_struct.graphic_tpye = Circle;
	judge_send_mesg.ext_client_custom_graphic_single.grapic_data_struct.layer = layer6;
	judge_send_mesg.ext_client_custom_graphic_single.grapic_data_struct.color =  Green ;
	judge_send_mesg.ext_client_custom_graphic_single.grapic_data_struct.start_angle =  0 ;
	judge_send_mesg.ext_client_custom_graphic_single.grapic_data_struct.end_angle =  0 ;
	judge_send_mesg.ext_client_custom_graphic_single.grapic_data_struct.width = 2;
	judge_send_mesg.ext_client_custom_graphic_single.grapic_data_struct.start_x = 960;
	judge_send_mesg.ext_client_custom_graphic_single.grapic_data_struct.start_y = 437;//540中心
	judge_send_mesg.ext_client_custom_graphic_single.grapic_data_struct.radius = 50;
	judge_send_mesg.ext_client_custom_graphic_single.grapic_data_struct.end_x = 960;//960中心
	judge_send_mesg.ext_client_custom_graphic_single.grapic_data_struct.end_y = 437;	
}


//己方机器人通信
void judgement_client_packet_pack(uint8_t *p_data)
{
	uint8_t current_robot_id = 0;
	uint16_t receiver_ID = 0;
	
	current_robot_id = judge_recv_mesg.game_robot_state.robot_id;  //读取当前机器人的id
	//此处选择的receiver_ID可以自行选择
	switch(current_robot_id)
	{
		//red robot
		case STUDENT_RED_HERO_ID:
		{
			receiver_ID = STUDENT_RED_SENTRY_ID;
		}break;
		case STUDENT_RED_ENGINEER_ID:
		{
			receiver_ID = STUDENT_RED_SENTRY_ID;
		}break;
		case STUDENT_RED_AERIAL_ID:
		{
			receiver_ID = STUDENT_RED_SENTRY_ID;
		}break;
		case STUDENT_RED_SENTRY_ID:
		{
			receiver_ID = STUDENT_RED_SENTRY_ID;
		}break;
		case STUDENT_RED_INFANTRY3_ID:
		{
			receiver_ID = STUDENT_RED_SENTRY_ID;
		}break;
		case STUDENT_RED_INFANTRY4_ID:
		{
			receiver_ID = STUDENT_RED_SENTRY_ID;
		}break;
		case STUDENT_RED_INFANTRY5_ID:
		{
			receiver_ID = STUDENT_RED_SENTRY_ID;
		}break;
		
		//blue robot
		case STUDENT_BLUE_HERO_ID:
		{
			receiver_ID = STUDENT_BLUE_SENTRY_ID;
		}break;
		case STUDENT_BLUE_ENGINEER_ID:
		{
			receiver_ID = STUDENT_BLUE_SENTRY_ID;
		}break;
		case STUDENT_BLUE_AERIAL_ID:
		{
			receiver_ID = STUDENT_BLUE_SENTRY_ID;
		}break;
		case STUDENT_BLUE_SENTRY_ID:
		{
			receiver_ID = STUDENT_BLUE_SENTRY_ID;
		}break;
		
		case STUDENT_BLUE_INFANTRY3_ID:
		{
			receiver_ID = STUDENT_BLUE_SENTRY_ID;
		}break;	
		case STUDENT_BLUE_INFANTRY4_ID:
		{
			receiver_ID = STUDENT_BLUE_SENTRY_ID;
		}break;
		case STUDENT_BLUE_INFANTRY5_ID:
		{
			receiver_ID = STUDENT_BLUE_SENTRY_ID;
		}break;

	}

	judge_send_mesg.ext_student_interactive_data.data_cmd_id = RobotCommunication;
	judge_send_mesg.ext_student_interactive_data.sender_ID = (uint16_t)current_robot_id;
	judge_send_mesg.ext_student_interactive_data.receiver_ID = receiver_ID;
	//将自定义的数据复制到发送结构体中
	memcpy(&judge_send_mesg.ext_student_interactive_data.data[0], p_data,sizeof(judge_send_mesg.ext_student_interactive_data.data));
	//该函数的功能为将需要发送的数据打包，便于下一步通过串口3发送给裁判系统
	data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID, (uint8_t *)&judge_send_mesg.ext_student_interactive_data,
									 STUDENT_DATA_LENGTH, DN_REG_ID);
}

void sentry_cmd_packet_pack() //未修改完 //uint8_t *p_data
{
	uint8_t current_robot_id = 0;
	
	current_robot_id = judge_recv_mesg.game_robot_state.robot_id;  //读取当前机器人的id
	
	judge_send_mesg.sentry_cmd.data_cmd_id = SENTRY_CMD_ID;
	judge_send_mesg.sentry_cmd.sender_ID = (uint16_t)current_robot_id;
	judge_send_mesg.sentry_cmd.receiver_ID = JUDGE_SERVER_ID;
	
	if(judge_recv_mesg.game_robot_state.current_HP == 0)
	{
		judge_send_mesg.sentry_cmd.confirm_rebirth = 1;
	}
	else
	{
		judge_send_mesg.sentry_cmd.confirm_rebirth = 0;
	}
	judge_send_mesg.sentry_cmd.exchange_rebirth = 0;
	judge_send_mesg.sentry_cmd.remote_blood_times = 0;
	judge_send_mesg.sentry_cmd.remote_bullet_times = 0;
	judge_send_mesg.sentry_cmd.bullet_number = 0;
	judge_send_mesg.sentry_cmd.reserved = 0;

	data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.sentry_cmd,
										sizeof(judge_send_mesg.sentry_cmd), DN_REG_ID);
}
void map_data_packet_pack(uint8_t *p_data) //未修改完
{
	judge_send_mesg.map_data.sender_id = judge_recv_mesg.game_robot_state.robot_id;   //读取当前机器人的id
	
		//该函数的功能为将需要发送的数据打包，便于下一步通过串口3发送给裁判系统
	data_packet_pack(MAP_DATA_ID, (uint8_t *)&judge_send_mesg.map_data,
									 sizeof(judge_send_mesg.map_data), DN_REG_ID);
}