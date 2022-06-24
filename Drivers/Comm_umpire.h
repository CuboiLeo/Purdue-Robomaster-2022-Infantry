#ifndef UMPIRE_H
#define UMPIRE_H
#include "sys.h"
#include "usart.h"

#define UART_RX_LEN     200

//���ݰ���ʽ
typedef __packed struct
{
	uint8_t SOF;//֡��ʼ�ֽڣ��̶�ֵ0xA5
	uint8_t Seq;//�����
	uint8_t CRC8;//��ͷcrc8����
	uint16_t DataLength;//���ݶ�data����
	uint16_t CmdID;//������ID
	uint16_t FrameTail;//����crc16����	
}Frame_TypeDef;


typedef __packed struct
{
  uint8_t game_type : 4;//��������
  uint8_t game_progress : 4;//��ǰ�����׶�
  uint16_t stage_remain_time;//��ǰ�׶�ʣ��ʱ��
  uint64_t SyncTimeStamp;
}ext_game_state_t;


typedef __packed struct
{
  uint8_t winner;//0 ƽ�� 1 �췽ʤ�� 2 ����ʤ��
}ext_game_result_t;


typedef __packed struct
{
	uint16_t red_1_robot_HP;//�� 1 Ӣ�ۻ�����Ѫ����δ�ϳ��Լ�����Ѫ��Ϊ 0
	uint16_t red_2_robot_HP;//�� 2 ���̻�����Ѫ�� 
	uint16_t red_3_robot_HP;//�� 3 ����������Ѫ�� 
	uint16_t red_4_robot_HP;//�� 4 ����������Ѫ�� 
	uint16_t red_5_robot_HP;//�� 5 ����������Ѫ�� 
	uint16_t red_7_robot_HP;//�� 7 �ڱ�������Ѫ��
	uint16_t red_outpost_HP;//�췽ǰ��վѪ��
	uint16_t red_base_HP;//�췽����Ѫ�� 
	uint16_t blue_1_robot_HP;//�� 1 Ӣ�ۻ�����Ѫ�� 
	uint16_t blue_2_robot_HP;//�� 2 ���̻�����Ѫ�� 
	uint16_t blue_3_robot_HP;//�� 3 ����������Ѫ�� 
	uint16_t blue_4_robot_HP;//�� 4 ����������Ѫ�� 
	uint16_t blue_5_robot_HP;//�� 5 ����������Ѫ�� 
	uint16_t blue_7_robot_HP;//�� 7 �ڱ�������Ѫ�� 
	uint16_t blue_outpost_HP;//����ǰ��վѪ��
	uint16_t blue_base_HP;//��������Ѫ��
}ext_game_robot_HP_t;


typedef __packed struct
{
  uint32_t event_type;
//bit 0-1������ͣ��ƺռ��״̬
// 0 Ϊ�޻�����ռ�죻  1 Ϊ���л�������ռ�쵫δͣ����  2 Ϊ���л�������ռ�첢ͣ��
//bit 2����������վ 1 �Ų�Ѫ��ռ��״̬ 1 Ϊ��ռ�죻
//bit 3����������վ 2 �Ų�Ѫ��ռ��״̬ 1 Ϊ��ռ�죻
//bit 4����������վ 3 �Ų�Ѫ��ռ��״̬ 1 Ϊ��ռ�죻
//bit 5-7��������������״̬��
//bit 5 Ϊ�����ռ��״̬��1 Ϊռ�죻  bit 6 ΪС�������ؼ���״̬��1 Ϊ�Ѽ�� bit 7 Ϊ���������ؼ���״̬��1 Ϊ�Ѽ��
//bit 8�������ؿ�ռ��״̬ 1 Ϊ��ռ�죻
//bit 9�������ﱤռ��״̬ 1 Ϊ��ռ�죻
//bit 10��������Դ��ռ��״̬ 1 Ϊ��ռ�죻
//bit 11���������ػ���״̬�� ? 1 Ϊ���������⻤��Ѫ���� ? 0 Ϊ���������⻤��Ѫ����
//bit 12 -27: ����
//bit 28-29��ICRA �췽�����ӳ�
//0�������ӳ�δ���  1�������ӳ� 5s ���������У���
//2�������ӳ��Ѽ���
//bit 14-15��ICRA ���������ӳ�
//0�������ӳ�δ���
//1�������ӳ� 5s ���������У�
//2�������ӳ��Ѽ���
//���ౣ��	
}ext_event_data_t;


typedef __packed struct
{
  uint8_t supply_projectile_id;//����վ�� ID��1��1 �Ų����ڣ�2��2 �Ų�����                             
  uint8_t supply_robot_id;//���������� ID��0 Ϊ��ǰ�޻����˲�����1 Ϊ�췽Ӣ�ۻ����˲�����2 Ϊ�췽���̻����˲�����3/4/5 Ϊ�췽���������˲�����11 Ϊ����Ӣ�ۻ����˲�����12 Ϊ�������̻����˲�����13/14/15 Ϊ�������������˲��� 
  uint8_t supply_projectile_step;//�����ڿ���״̬��0 Ϊ�رգ�1 Ϊ�ӵ�׼���У�2 Ϊ�ӵ����� 
  uint8_t supply_projectile_num;
//����������
//50��50 ���ӵ���
//100��100 ���ӵ���
//150��150 ���ӵ���
//200��200 ���ӵ���
}ext_supply_projectile_action_t;


typedef __packed struct
{
  uint8_t supply_projectile_id;//����վ������ ID��1��1 �Ų�����
  uint8_t supply_robot_id;//���������� ID��1 Ϊ�췽Ӣ�ۻ����˲�����2 Ϊ�췽���̻����˲�����3/4/5 Ϊ�췽���������˲�����11 Ϊ����Ӣ�ۻ����˲�����12 Ϊ�������̻����˲�����13/14/15 Ϊ�������������˲��� 
  uint8_t supply_num;//������Ŀ��50 ������ 50 ���ӵ�����
}ext_supply_projectile_booking_t;


typedef __packed struct
{
	uint8_t level;//����ȼ���
	uint8_t foul_robot_id;//��������� ID�� 1 ���Լ� 5 ������ʱ�������� ID Ϊ 0 �����ļ�����ʱ�������� ID Ϊ��������� ID 
}ext_referee_warning_t;


typedef __packed struct
{
  uint8_t robot_id;
//	������ ID��
//1���췽Ӣ�ۻ����ˣ�
//2���췽���̻����ˣ�
//3/4/5���췽���������ˣ�
//6���췽���л����ˣ�
//7���췽�ڱ������ˣ�
//11������Ӣ�ۻ����ˣ�
//12���������̻����ˣ�
//13/14/15���������������ˣ�
//16���������л����ˣ�
//17�������ڱ������ˡ�
 uint8_t robot_level;
//	�����˵ȼ���
//1��һ����2��������3��������
  uint16_t remain_HP;//������ʣ��Ѫ��	
  uint16_t max_HP;//����������Ѫ��
  uint16_t shooter_id1_17mm_cooling_rate;//������ 1 �� 17mm ǹ��ÿ����ȴֵ
  uint16_t shooter_id1_17mm_cooling_limit;//������ 1 �� 17mm ǹ����������
  uint16_t  shooter_id1_17mm_speed_limit;//������ 1 �� 17mm ǹ���ٶ����� ��λ m/s
  uint16_t shooter_id2_17mm_cooling_rate;//������ 2 �� 17mm ǹ��ÿ����ȴֵ
  uint16_t shooter_id2_17mm_cooling_limit;//������ 2 �� 17mm ǹ����������
  uint16_t  shooter_id2_17mm_speed_limit;//������ 2 �� 17mm ǹ���ٶ����� ��λ m/s
  uint16_t shooter_id1_42mm_cooling_rate;//������ 42mm ǹ��ÿ����ȴֵ
  uint16_t shooter_id1_42mm_cooling_limit;//������ 42mm ǹ����������
  uint16_t shooter_id1_42mm_speed_limit;//������ 42mm ǹ���ٶ����� ��λ m/s
  uint16_t chassis_power_limit;//�����˵��̹�����������
  uint8_t mains_power_gimbal_output : 1;
  uint8_t mains_power_chassis_output : 1;
  uint8_t mains_power_shooter_output : 1;
// ���ص�Դ��������
//0 bit��gimbal ������� 1 Ϊ�� 24V �����0 Ϊ�� 24v �����
//1 bit��chassis �������1 Ϊ�� 24V �����0 Ϊ�� 24v �����
//2 bit��shooter �������1 Ϊ�� 24V �����0 Ϊ�� 24v �����
}ext_game_robot_state_t;


typedef __packed struct
{
  uint16_t chassis_volt;//���������ѹ ��λ ���� 
  uint16_t chassis_current;//����������� ��λ ����  
  uint16_t chassis_power_buffer;//���̹��ʻ��� ��λ J ���� ��ע�����¸��ݹ��������� 250J
  uint16_t shooter_id1_17mm_cooling_heat;//1�� 17mm ǹ����
  uint16_t shooter_id2_17mm_cooling_heat;//2 �� 17mm ǹ������
  uint16_t shooter_id1_42mm_cooling_heat;//42mm ǹ������
	float chassis_power;//����������� ��λ W �� 
}ext_power_heat_data_t;


typedef __packed struct
{
  float x;//λ�� x ���꣬��λ m
  float y;//λ�� y ���꣬��λ m
  float z;//λ�� z ���꣬��λ m
  float yaw;//λ��ǹ�ڣ���λ��
}ext_game_robot_pos_t;


typedef __packed struct
{
  uint8_t power_rune_buff;
//bit 0��������Ѫ����Ѫ״̬
//bit 1��ǹ��������ȴ����
//bit 2�������˷����ӳ�
//bit 3�������˹����ӳ�
//���� bit ����
}ext_buff_musk_t;


typedef __packed struct
{
  uint8_t energy_point;//���۵�������
  uint8_t attack_time;//�ɹ���ʱ�� ��λ s��30s �ݼ��� 0
}aerial_robot_energy_t;


typedef __packed struct
{
  uint8_t armor_id : 4;
//bit 0-3����Ѫ���仯����Ϊװ���˺�������װ�� ID��������ֵΪ 0-4 �Ŵ��������
//�����װ��Ƭ������Ѫ���仯���ͣ��ñ�����ֵΪ 0��
  uint8_t hurt_type : 4;
//bit 4-7��Ѫ���仯����
//0x0 װ���˺���Ѫ��
//0x1 ģ����߿�Ѫ��0x2 �����ٿ�Ѫ��0x3 ��ǹ��������Ѫ0x4 �����̹��ʿ�Ѫ0x5 װ��ײ����Ѫ��
}ext_robot_hurt_t;


typedef __packed struct
{
  uint8_t bullet_type;//�ӵ�����: 1��17mm ���� 2��42mm ����
  uint8_t bullet_freq;//�ӵ���Ƶ ��λ Hz
  float bullet_speed;//�ӵ����� ��λ m/s
  float bullet_speed_last; 
}ext_shoot_data_t;


typedef __packed struct
{
  uint16_t bullet_remaining_num;//�ӵ�ʣ�෢����Ŀ
}ext_bullet_remaining_t;


typedef __packed struct
{
  uint16_t data_cmd_id;//���ݶε����� ID
  uint16_t send_ID;//�����ߵ� ID  �� ��ҪУ�鷢���ߵ� ID ��ȷ�ԣ������ 1 ���͸��� 5��������ҪУ��� 1
  uint16_t receiver_ID;//�����ߵ� ID �� ��ҪУ������ߵ� ID ��ȷ�ԣ����粻�ܷ��͵��жԻ����˵�ID
}ext_student_interactive_header_data_t;


extern Frame_TypeDef Frame;
extern ext_game_state_t Match_status_data;//����״̬����
extern ext_game_result_t Match_Result_data;//�����������
extern ext_game_robot_HP_t Robot_survival_data;//�����˴������
extern ext_event_data_t Site_event_data;//�����¼�����
extern ext_supply_projectile_action_t Stop_action_sign;//����վ������ʶ
extern ext_supply_projectile_booking_t Request_the_depot_to_reload;//���󲹸�վ�����ӵ�
extern ext_referee_warning_t Warning_Informations;
extern ext_game_robot_state_t Game_robot_state;//����������״̬
extern ext_power_heat_data_t Umpire_PowerHeat;//ʵʱ������������
extern ext_game_robot_pos_t Robot_position;//������λ��
extern ext_buff_musk_t Robot_gain;//����������
extern aerial_robot_energy_t Air_robot_energy_state;//���л���������״̬
extern ext_robot_hurt_t The_damage_state;//�˺�״̬
extern ext_shoot_data_t Real_time_shooting_information;//ʵʱ�����Ϣ
extern int bullet_speed_last;
extern int bullet_speed;
extern int offline_flag;

void Comm_umpire_Init(void);//��ʼ��
static void Get_UmpireData(uint16_t Length);//��ȡ����ϵͳ����
static void DealDataPack(uint16_t CmdID, uint16_t DataLength);//��������

#endif

