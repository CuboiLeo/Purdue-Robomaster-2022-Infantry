#ifndef UMPIRE_H
#define UMPIRE_H
#include "sys.h"
#include "usart.h"

#define UART_RX_LEN     200

//数据包格式
typedef __packed struct
{
	uint8_t SOF;//帧起始字节，固定值0xA5
	uint8_t Seq;//包序号
	uint8_t CRC8;//包头crc8检验
	uint16_t DataLength;//数据段data长度
	uint16_t CmdID;//命令码ID
	uint16_t FrameTail;//整包crc16检验	
}Frame_TypeDef;


typedef __packed struct
{
  uint8_t game_type : 4;//比赛类型
  uint8_t game_progress : 4;//当前比赛阶段
  uint16_t stage_remain_time;//当前阶段剩余时间
  uint64_t SyncTimeStamp;
}ext_game_state_t;


typedef __packed struct
{
  uint8_t winner;//0 平局 1 红方胜利 2 蓝方胜利
}ext_game_result_t;


typedef __packed struct
{
	uint16_t red_1_robot_HP;//红 1 英雄机器人血量，未上场以及罚下血量为 0
	uint16_t red_2_robot_HP;//红 2 工程机器人血量 
	uint16_t red_3_robot_HP;//红 3 步兵机器人血量 
	uint16_t red_4_robot_HP;//红 4 步兵机器人血量 
	uint16_t red_5_robot_HP;//红 5 步兵机器人血量 
	uint16_t red_7_robot_HP;//红 7 哨兵机器人血量
	uint16_t red_outpost_HP;//红方前哨站血量
	uint16_t red_base_HP;//红方基地血量 
	uint16_t blue_1_robot_HP;//蓝 1 英雄机器人血量 
	uint16_t blue_2_robot_HP;//蓝 2 工程机器人血量 
	uint16_t blue_3_robot_HP;//蓝 3 步兵机器人血量 
	uint16_t blue_4_robot_HP;//蓝 4 步兵机器人血量 
	uint16_t blue_5_robot_HP;//蓝 5 步兵机器人血量 
	uint16_t blue_7_robot_HP;//蓝 7 哨兵机器人血量 
	uint16_t blue_outpost_HP;//蓝方前哨站血量
	uint16_t blue_base_HP;//蓝方基地血量
}ext_game_robot_HP_t;


typedef __packed struct
{
  uint32_t event_type;
//bit 0-1：己方停机坪占领状态
// 0 为无机器人占领；  1 为空中机器人已占领但未停桨；  2 为空中机器人已占领并停桨
//bit 2：己方补给站 1 号补血点占领状态 1 为已占领；
//bit 3：己方补给站 2 号补血点占领状态 1 为已占领；
//bit 4：己方补给站 3 号补血点占领状态 1 为已占领；
//bit 5-7：己方能量机关状态：
//bit 5 为打击点占领状态，1 为占领；  bit 6 为小能量机关激活状态，1 为已激活； bit 7 为大能量机关激活状态，1 为已激活；
//bit 8：己方关口占领状态 1 为已占领；
//bit 9：己方碉堡占领状态 1 为已占领；
//bit 10：己方资源岛占领状态 1 为已占领；
//bit 11：己方基地护盾状态： ? 1 为基地有虚拟护盾血量； ? 0 为基地无虚拟护盾血量；
//bit 12 -27: 保留
//bit 28-29：ICRA 红方防御加成
//0：防御加成未激活；  1：防御加成 5s 触发激活中；；
//2：防御加成已激活
//bit 14-15：ICRA 蓝方防御加成
//0：防御加成未激活；
//1：防御加成 5s 触发激活中；
//2：防御加成已激活
//其余保留	
}ext_event_data_t;


typedef __packed struct
{
  uint8_t supply_projectile_id;//补给站口 ID：1：1 号补给口；2：2 号补给口                             
  uint8_t supply_robot_id;//补弹机器人 ID：0 为当前无机器人补弹，1 为红方英雄机器人补弹，2 为红方工程机器人补弹，3/4/5 为红方步兵机器人补弹，11 为蓝方英雄机器人补弹，12 为蓝方工程机器人补弹，13/14/15 为蓝方步兵机器人补弹 
  uint8_t supply_projectile_step;//出弹口开闭状态：0 为关闭，1 为子弹准备中，2 为子弹下落 
  uint8_t supply_projectile_num;
//补弹数量：
//50：50 颗子弹；
//100：100 颗子弹；
//150：150 颗子弹；
//200：200 颗子弹。
}ext_supply_projectile_action_t;


typedef __packed struct
{
  uint8_t supply_projectile_id;//补给站补弹口 ID：1：1 号补给口
  uint8_t supply_robot_id;//补弹机器人 ID：1 为红方英雄机器人补弹，2 为红方工程机器人补弹，3/4/5 为红方步兵机器人补弹，11 为蓝方英雄机器人补弹，12 为蓝方工程机器人补弹，13/14/15 为蓝方步兵机器人补弹 
  uint8_t supply_num;//补弹数目：50 ：请求 50 颗子弹下落
}ext_supply_projectile_booking_t;


typedef __packed struct
{
	uint8_t level;//警告等级：
	uint8_t foul_robot_id;//犯规机器人 ID： 1 级以及 5 级警告时，机器人 ID 为 0 二三四级警告时，机器人 ID 为犯规机器人 ID 
}ext_referee_warning_t;


typedef __packed struct
{
  uint8_t robot_id;
//	机器人 ID：
//1：红方英雄机器人；
//2：红方工程机器人；
//3/4/5：红方步兵机器人；
//6：红方空中机器人；
//7：红方哨兵机器人；
//11：蓝方英雄机器人；
//12：蓝方工程机器人；
//13/14/15：蓝方步兵机器人；
//16：蓝方空中机器人；
//17：蓝方哨兵机器人。
 uint8_t robot_level;
//	机器人等级：
//1：一级；2：二级；3：三级。
  uint16_t remain_HP;//机器人剩余血量	
  uint16_t max_HP;//机器人上限血量
  uint16_t shooter_id1_17mm_cooling_rate;//机器人 1 号 17mm 枪口每秒冷却值
  uint16_t shooter_id1_17mm_cooling_limit;//机器人 1 号 17mm 枪口热量上限
  uint16_t  shooter_id1_17mm_speed_limit;//机器人 1 号 17mm 枪口速度上线 单位 m/s
  uint16_t shooter_id2_17mm_cooling_rate;//机器人 2 号 17mm 枪口每秒冷却值
  uint16_t shooter_id2_17mm_cooling_limit;//机器人 2 号 17mm 枪口热量上限
  uint16_t  shooter_id2_17mm_speed_limit;//机器人 2 号 17mm 枪口速度上线 单位 m/s
  uint16_t shooter_id1_42mm_cooling_rate;//机器人 42mm 枪口每秒冷却值
  uint16_t shooter_id1_42mm_cooling_limit;//机器人 42mm 枪口热量上限
  uint16_t shooter_id1_42mm_speed_limit;//机器人 42mm 枪口速度上线 单位 m/s
  uint16_t chassis_power_limit;//机器人底盘功率限制上限
  uint8_t mains_power_gimbal_output : 1;
  uint8_t mains_power_chassis_output : 1;
  uint8_t mains_power_shooter_output : 1;
// 主控电源输出情况：
//0 bit：gimbal 口输出： 1 为有 24V 输出，0 为无 24v 输出；
//1 bit：chassis 口输出：1 为有 24V 输出，0 为无 24v 输出；
//2 bit：shooter 口输出：1 为有 24V 输出，0 为无 24v 输出；
}ext_game_robot_state_t;


typedef __packed struct
{
  uint16_t chassis_volt;//底盘输出电压 单位 毫伏 
  uint16_t chassis_current;//底盘输出电流 单位 毫安  
  uint16_t chassis_power_buffer;//底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
  uint16_t shooter_id1_17mm_cooling_heat;//1号 17mm 枪口热
  uint16_t shooter_id2_17mm_cooling_heat;//2 号 17mm 枪口热量
  uint16_t shooter_id1_42mm_cooling_heat;//42mm 枪口热量
	float chassis_power;//底盘输出功率 单位 W 瓦 
}ext_power_heat_data_t;


typedef __packed struct
{
  float x;//位置 x 坐标，单位 m
  float y;//位置 y 坐标，单位 m
  float z;//位置 z 坐标，单位 m
  float yaw;//位置枪口，单位度
}ext_game_robot_pos_t;


typedef __packed struct
{
  uint8_t power_rune_buff;
//bit 0：机器人血量补血状态
//bit 1：枪口热量冷却加速
//bit 2：机器人防御加成
//bit 3：机器人攻击加成
//其他 bit 保留
}ext_buff_musk_t;


typedef __packed struct
{
  uint8_t energy_point;//积累的能量点
  uint8_t attack_time;//可攻击时间 单位 s。30s 递减至 0
}aerial_robot_energy_t;


typedef __packed struct
{
  uint8_t armor_id : 4;
//bit 0-3：当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人
//的五个装甲片，其他血量变化类型，该变量数值为 0。
  uint8_t hurt_type : 4;
//bit 4-7：血量变化类型
//0x0 装甲伤害扣血；
//0x1 模块掉线扣血；0x2 超射速扣血；0x3 超枪口热量扣血0x4 超底盘功率扣血0x5 装甲撞击扣血。
}ext_robot_hurt_t;


typedef __packed struct
{
  uint8_t bullet_type;//子弹类型: 1：17mm 弹丸 2：42mm 弹丸
  uint8_t bullet_freq;//子弹射频 单位 Hz
  float bullet_speed;//子弹射速 单位 m/s
  float bullet_speed_last; 
}ext_shoot_data_t;


typedef __packed struct
{
  uint16_t bullet_remaining_num;//子弹剩余发射数目
}ext_bullet_remaining_t;


typedef __packed struct
{
  uint16_t data_cmd_id;//数据段的内容 ID
  uint16_t send_ID;//发送者的 ID  ， 需要校验发送者的 ID 正确性，例如红 1 发送给红 5，此项需要校验红 1
  uint16_t receiver_ID;//接收者的 ID ， 需要校验接收者的 ID 正确性，例如不能发送到敌对机器人的ID
}ext_student_interactive_header_data_t;


extern Frame_TypeDef Frame;
extern ext_game_state_t Match_status_data;//比赛状态数据
extern ext_game_result_t Match_Result_data;//比赛结果数据
extern ext_game_robot_HP_t Robot_survival_data;//机器人存活数据
extern ext_event_data_t Site_event_data;//场地事件数据
extern ext_supply_projectile_action_t Stop_action_sign;//补给站动作标识
extern ext_supply_projectile_booking_t Request_the_depot_to_reload;//请求补给站补弹子弹
extern ext_referee_warning_t Warning_Informations;
extern ext_game_robot_state_t Game_robot_state;//比赛机器人状态
extern ext_power_heat_data_t Umpire_PowerHeat;//实时功率热量数据
extern ext_game_robot_pos_t Robot_position;//机器人位置
extern ext_buff_musk_t Robot_gain;//机器人增益
extern aerial_robot_energy_t Air_robot_energy_state;//空中机器人能量状态
extern ext_robot_hurt_t The_damage_state;//伤害状态
extern ext_shoot_data_t Real_time_shooting_information;//实时射击信息
extern int bullet_speed_last;
extern int bullet_speed;
extern int offline_flag;

void Comm_umpire_Init(void);//初始化
static void Get_UmpireData(uint16_t Length);//获取裁判系统数据
static void DealDataPack(uint16_t CmdID, uint16_t DataLength);//解析数据

#endif

