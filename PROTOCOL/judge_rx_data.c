#include "judge_rx_data.h"
#include "dma.h"
#include "string.h"
#include "judge_task.h"
#include "chassis_task.h"
#include "modeswitch_task.h"
#include "remote_ctrl.h"


/*裁判系统的数据接收 --裁判系统发送数据，stm32接收；
  发送对应的命令码cmd_id,然后读取相应信息*/
	
judge_rxdata_t judge_recv_mesg;
judge_data_limit_t judge_data_limit;

/* judge system dma receive data object */
uart_dma_rxdata_t judge_rx_obj;
//----------解包任务
/* unpack object */
unpack_data_t judge_unpack_obj;

static SemaphoreHandle_t judge_rxdata_mutex;
fifo_s_t  judge_rxdata_fifo;
static uint8_t   judge_rxdata_buf[JUDGE_RX_FIFO_BUFLEN];

void judgement_rx_param_init(void)
{
  /* create the judge_rxdata_mutex mutex  */  
  judge_rxdata_mutex = xSemaphoreCreateMutex();
    
  /* judge data fifo init fifo存储器--先进先出*/
  fifo_s_init(&judge_rxdata_fifo, judge_rxdata_buf, JUDGE_RX_FIFO_BUFLEN, judge_rxdata_mutex); //添加judge_rxdata_fifo的互斥量

  
  /* initial judge data dma receiver object 初始裁判数据dma接收对象 */
  judge_rx_obj.dma_stream = DMA2_Stream1;
  judge_rx_obj.data_fifo = &judge_rxdata_fifo;
  judge_rx_obj.buff_size = JUDGE_RX_FIFO_BUFLEN;
  judge_rx_obj.buff[0] = judge_rxbuf[0];
  judge_rx_obj.buff[1] = judge_rxbuf[1];

  /* initial judge data unpack object 初始裁判数据解包对象 */
  judge_unpack_obj.data_fifo = &judge_rxdata_fifo;
  judge_unpack_obj.p_header = (frame_header_t *)judge_unpack_obj.protocol_packet;//协议包的帧头数据
  judge_unpack_obj.index = 0;
  judge_unpack_obj.data_len = 0;
  judge_unpack_obj.unpack_step = STEP_HEADER_SOF;
  
}



uint8_t interover_data_overflow;
/*一帧数据包括：帧头HEADER + 命令CMD + 数据data + 校验位CRC*/
/*@ p_frame：数组首地址*/
uint16_t CMD_ID;
void judgement_data_handler(uint8_t *p_frame)
{
  frame_header_t *p_header = (frame_header_t*)p_frame;//帧头--数据帧起始字节（固定值0xA5）+数据帧的长度+包序号+CRC8校验
  memcpy(p_header, p_frame, HEADER_LEN);              //将帧头信息复制到p_header变量
  
  uint16_t data_length = p_header->data_length;              //数据长度
  uint16_t cmd_id      = *(uint16_t *)(p_frame + HEADER_LEN);// CMD的开始地址 = 数组首地址 + 帧头长度（将cmd_id赋值特定地址内的值）
  uint8_t *data_addr   = p_frame + HEADER_LEN + CMD_LEN;     //数据的开始地址 = 数组首地址 + 帧头长度 + CMD长度 
  CMD_ID =  cmd_id; 
  switch (cmd_id)
  {
    case GAME_STATE_ID:                                               //001,比赛状态数据,1HZ周期发送
    {
      memcpy(&judge_recv_mesg.game_state, data_addr, data_length);//读取比赛状态数据保存在结构体judge_recv_mesg中
		
		/**judge_recv_mesg.game_state.game_progress代表当前比赛阶段信息**/
		 if (judge_recv_mesg.game_state.game_progress == 3)  //5 seconds count down 5秒倒计时
		 {
			 /*用户代码*/
		 }
      
      if (judge_recv_mesg.game_state.game_progress == 1)   //prepare stage 准备阶段
      {
        if (judge_recv_mesg.game_state.stage_remain_time < 240)//阶段保持时间
        {
          /*用户代码*/
        }
      }
      else
      {
          /*用户代码*/
      }
			
			if(judge_recv_mesg.game_state.game_progress == 4 && judge_recv_mesg.game_state.stage_remain_time >= 0 )
			{
//					rc.sw1 = RC_MI;
//					rc.sw2 = RC_UP;
				global_mode = FULL_AUTOMATIC_CTRL;
			}
    }
    break;
		
	  case GAME_RESULT_ID:                                              //002,比赛结果数据
      memcpy(&judge_recv_mesg.game_result, data_addr, data_length);   
    break;
		
    case GAME_ROBOT_HP_ID:                                            //003，比赛机器人血量数据,1HZ周期发送
      memcpy(&judge_recv_mesg.game_robot_HP, data_addr, data_length); 
    break;
			
    case EVENT_DATA_ID:                                               //004,场地事件数据，1HZ周期发送
      memcpy(&judge_recv_mesg.event_data, data_addr, data_length);    
    break; 
		
    case SUPPLY_PROJECTILE_ACTION_ID:                                 //102，场地补给站动作标识符，动作发生后发送
      memcpy(&judge_recv_mesg.supply_projectile_action, data_addr, data_length);
    break;
		
		case REFEREE_WARNING_ID:                                          //104，裁判警告数据，警告后发送
      memcpy(&judge_recv_mesg.referee_warning, data_addr, data_length);
		break;
		
		case DART_REMAINING_TINME_ID:                                     //105，飞镖发射口倒计时,1HZ周期发送
			memcpy(&judge_recv_mesg.dart_remaining_time, data_addr, data_length);
		break;
		
		case GAME_ROBOT_STATE_ID:                                         //201，机器人状态10HZ周期发送
			memcpy(&judge_recv_mesg.game_robot_state, data_addr, data_length);
		break;
		
		case POWER_HEAT_DATA_ID:                                          //202，实时功率热量数据，50HZ周期发送
			memcpy(&judge_recv_mesg.power_heat_data, data_addr, data_length);
		break;
		
		case GAME_ROBOT_POS_ID:                                           //203，机器人位置数据，10HZ周期发送
			memcpy(&judge_recv_mesg.game_robot_pos, data_addr, data_length);
		break;
		
    case BUFF_ID:                                                     //204，机器人增益数据，1HZ周期发送
			memcpy(&judge_recv_mesg.buff, data_addr, data_length);
		break;
		
		case AERIAL_ROBOT_ENERGY_ID:                                      //205，空中机器人能量状态数据，10HZ周期发送，只有空中主控发送
			memcpy(&judge_recv_mesg.aerial_robot_energy, data_addr, data_length);
		break;
		
		case ROBOT_HURT_ID:                                               //206 伤害状态数据，伤害发生后发送
			memcpy(&judge_recv_mesg.robot_hurt, data_addr, data_length);
		break;
		
		case SHOOT_DATA_ID:                                               //207，实时射击数据，子弹发射后发送
			memcpy(&judge_recv_mesg.shoot_data, data_addr, data_length);
			judge_data_limit.shooter_id1_17mm_speed_limit = 30;  //m/s
			judge_data_limit.shooter_id2_17mm_speed_limit = 30;  //m/s
		break;
		
		case BULLET_REMAINING_ID:                                         //208，弹丸剩余量，仅哨兵、空中ICRA机器人，1HZ发送
			memcpy(&judge_recv_mesg.bullet_remaining, data_addr, data_length);
		break;
		
		case RFID_STATE_ID:                                               //209，机器人RFID状态，1HZ发送
			memcpy(&judge_recv_mesg.rfid_state, data_addr, data_length);
		break;
		
		case DAT_CLIENT_CMD_ID:                                           //20A，飞镖机器人客户端指令数据，10HZ周期发送
			memcpy(&judge_recv_mesg.dart_client_cmd, data_addr, data_length);
		break;
		
		case GROUND_ROBOT_POSITION_ID:                                    //20B，地面机器人位置数据
			memcpy(&judge_recv_mesg.ground_robot_position, data_addr, data_length);
		break;

		case RADAR_MARK_DATA_ID:                                          //20C，雷达标记进度数据
			memcpy(&judge_recv_mesg.radar_mark_data, data_addr, data_length);
		break;

		case SENTRY_INFO_ID:                                              //020D，哨兵自主决策相关信息同步
			memcpy(&judge_recv_mesg.sentry_info, data_addr, data_length);
		break;
		
		case RADAR_INFO_ID:                                               //020E，雷达自主决策信息同步
			memcpy(&judge_recv_mesg.radar_info, data_addr, data_length);
		break;
//	  case STUDENT_INTERACTIVE_HEADER_DATA_ID:                         //301 机器人间交互数据，发送方触发发送
//	  {
//		  if(data_length <= 119)                                         //数据段头结构长度+交互数据长度
//		  {
//			  memcpy(&judge_recv_mesg.student_interactive_header_data, data_addr, data_length);
//			  interover_data_overflow = 0;
//		  }
//			else
//			{
//				interover_data_overflow = 1;
//			}
//		}break;	

		case CUSTOM_CONTROLLER_INTERACTION_DATA_ID:                                        //302，自定义控制器交互数据接口，通过客户端触发发送，上限 30HZ
			memcpy(&judge_recv_mesg.robot_interactive_data, data_addr, data_length);
		break;
		
		case CLIENT_MINIMAP_INTERACTIVE_DATDA_ID:                                          //303，客户端小地图交互数据，触发发送
			memcpy(&judge_recv_mesg.minimap_interactive_data, data_addr, data_length);
		break;
		
		case KEYBOARD_AND_MOUSE_INFORMATION_ID:                                            //304，键盘、鼠标信息，通过图传串口发送
			memcpy(&judge_recv_mesg.mouse_keyboard_informationt, data_addr, data_length);
    break;
		
		case MAP_ROBOT_DATA_ID:                                                            //305，选手端小地图接收雷达数据
//			memcpy(&judge_recv_mesg.dart_client_cmd, data_addr, data_length);
		break;
		
		case CUSTOM_CLIENT_DATA_ID:                                               				 //306，自定义控制器与选手端交互数据
//			memcpy(&judge_recv_mesg.dart_client_cmd, data_addr, data_length);
		break;
		
		case MAP_DATA_ID:                                               									 //307，选手端小地图接收哨兵数据
//			memcpy(&judge_recv_mesg.dart_client_cmd, data_addr, data_length);
		break;
		
		case CUSTOM_INFO_ID:                                               								 //308，选手端小地图接收机器人消息
//			memcpy(&judge_recv_mesg.dart_client_cmd, data_addr, data_length);
		break;
		
		default:
    {
		}
    break;
  }
  
}
