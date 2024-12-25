#ifndef __DRIVER_MOTOR_H__
#define __DRIVER_MOTOR_H__
#include "main.h" 
#include "fdcan.h"
#include "DRIVER_FDCAN.h"

#pragma pack(1)
typedef union{
    int32_t     L;
    uint8_t     u8[4];
    uint16_t    u16[2];
    uint32_t    u32;
    float       F;
} COMData32;

// ������������ģʽ 1Byte
typedef struct{
    uint8_t id     :4;      // ���ID: 0,1...,13,14 15��ʾ�����е���㲥����(��ʱ�޷���)
    uint8_t status :3;      // ����ģʽ: 0.���� 1.FOC�ջ� 2.������У׼ 3.����
    uint8_t none   :1;      // ����λ
} RIS_Mode_t;  

// �����������Ʋ��� 12Byte
typedef struct{
    int16_t tor_des;        // �����ؽ����Ť�� unit: N.m      (q8)
    int16_t spd_des;        // �����ؽ�����ٶ� unit: rad/s    (q8)
    int32_t pos_des;        // �����ؽ����λ�� unit: rad      (q15)
    int16_t k_pos;          // �����ؽڸն�ϵ�� unit: -1.0-1.0 (q15)
    int16_t k_spd;          // �����ؽ�����ϵ�� unit: -1.0-1.0 (q15) 
} RIS_Comd_t;   

// ��������״̬���� 11Byte
typedef struct{
    int16_t  torque;        // ʵ�ʹؽ����Ť�� unit: N.m     (q8)
    int16_t  speed;         // ʵ�ʹؽ�����ٶ� unit: rad/s   (q8)
    int32_t  pos;           // ʵ�ʹؽ����λ�� unit: rad     (q15)
    int8_t   temp;          // ����¶�: -128~127��C
    uint8_t  MError :3;     // ��������ʶ: 0.���� 1.���� 2.���� 3.��ѹ 4.���������� 5-7.����
    uint16_t force  :12;    // �����ѹ���������� 12bit (0-4095)
    uint8_t  none   :1;     // ����λ
} RIS_Fbk_t;  

//��������������ݰ�
typedef struct{
    // ���� ��������������ݰ�
    uint8_t head[2];    // ��ͷ         2Byte
    RIS_Mode_t mode;    // �������ģʽ  1Byte
    RIS_Comd_t comd;    // ����������� 12Byte
    uint16_t   CRC16;   // CRC          2Byte
} unitreeMotorSendPack_t;  

//����������ݰ�
typedef struct{
    uint8_t head[2];    // ��ͷ         2Byte
    RIS_Mode_t mode;    // �������ģʽ  1Byte
    RIS_Fbk_t   fbk;    // ����������� 11Byte
    uint16_t  CRC16;    // CRC          2Byte
} unitreeMotorRevPack_t;  
#pragma pack()

//�����������ýṹ��
//ʵ�ʸ�FOC��ָ������Ϊ��
// K_P*delta_Pos + K_W*delta_W + T
typedef struct{
	unitreeMotorSendPack_t motorSendpack;      //����������ݽṹ��
	unsigned short id;									//���ID��0����ȫ�����
	int hexLen;                         //���͵�16�����������鳤��, 34
	long long sendTime;                 //���͸������ʱ��, ΢��(us)         
	unsigned short mode;                // 0:����, 5:����ת��, 10:�ջ�FOC����
	float t;                            //�����ؽڵ�������أ������������أ���Nm��
	float w;                            //�����ؽ��ٶȣ����������ٶȣ�(rad/s)
	float pos;                          //�����ؽ�λ�ã�rad��
	float kp;                           //�ؽڸն�ϵ��
	float kw;                           //�ؽ��ٶ�ϵ��
	COMData32 res;                    	//ͨѶ�������ֽڣ�����ʵ�ֱ��һЩͨѶ����	
} unitreeMotorSendConfig_t;

//�����������ýṹ��
typedef struct{
    unitreeMotorRevPack_t motorRevpack;        //����������ݽṹ��
		unsigned char id;              			//���ID
    int hexLen;                         //���յ�16�����������鳤��, 78
    long long resvTime;                 //���ո������ʱ��, ΢��(us)
    unsigned char mode;                 //0:����, 5:����ת��, 10:�ջ�FOC����
    float t;                            //��ǰʵ�ʵ���������
		float w;														//speed
    float pos;                          //��ǰ���λ�ã�����0������������ؽڻ����Ա�����0��Ϊ׼��
		float footForce;										//�����ѹ���������� 12bit (0-4095)
		int correct;                        //���������Ƿ�������1������0��������
		int temp;                           //�¶�
		unsigned char errorCode;            //������
	
		int32_t enconder;
} unitreeMotorRevConfig_t;

typedef struct{
	TickType_t startTime;
	TickType_t duringTime;
}driverMotorConfig_t;

typedef enum{
  Motor_Enable,
  Motor_Disable,
  Motor_Save_Zero_Position,
  DM_Motor_CMD_Type_Num,
}dmMotorCMD;

typedef enum{
  mit_mode,
  pos_vel_mode,
  vel_mode,
  num,
}dmMotorMode;

typedef struct {
  int16_t  state; 	
  uint16_t  pos_int;
  uint16_t  vel_int;
  uint16_t  tor_int;
  float  pos;  
  float  vel;  
  float  tor;  
  float  temp_mos;   
  float  temp_rotor;  
}dmMotorRevPack_t;

typedef struct{
  uint32_t Master_ID;   
  uint32_t CAN_ID;  
}dmMotorCanFrame_t;

typedef struct{
	uint16_t ID;
  dmMotorCanFrame_t canFrameInfo;
	dmMotorRevPack_t revPack;  
}dmMotorRevConfig_t;

typedef struct{
	float  kp;
	float  kd;
	float  pos; 
  float  vel;  	
  float  tor;  
}dmMotorSendConfig_t;

#define LIMIT(IN, MIN, MAX) IN=(IN>MAX)?MAX:((IN<MIN)?MIN:IN)
#define SET_485_DE_UP() HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET)
#define SET_485_DE_DOWN() HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET)
#define UNITREE_MOTOR_NUM 4
#define DM_MOTOR_NUM 2
#define DM_P_MAX 12.5f
#define DM_V_MAX 45.f
#define DM_T_MAX 10.f

extern unitreeMotorSendConfig_t unitreeMotorSendConfig[UNITREE_MOTOR_NUM];   
extern unitreeMotorRevConfig_t unitreeMotorRevConfig[UNITREE_MOTOR_NUM];
extern unitreeMotorSendConfig_t test;

void unitreeMotorInit(void);
static void unitreeLoadPack(unitreeMotorSendConfig_t *pData);
static void unitreeMotorSendUpdata(unitreeMotorSendConfig_t *send_config);
static uint8_t unitreeMotorRevUpdata(unitreeMotorRevConfig_t *revConfig, unitreeMotorRevPack_t *revPack);
void unitreeMotorTask(void);


void dmMotorInit(void);
void dmMotorTask(void);
extern dmMotorRevConfig_t dmMotorRevConfig[2];
extern dmMotorSendConfig_t dmMotorSendConfig[2];
//extern void DM_Motor_Multi_Info_Update(uint8_t *Data,dmMotorRevConfig_t *DM_Motor);
extern void dmMotorSendCMD(FDCAN_TxFrame_TypeDef *txFrame,uint16_t txId,uint8_t cmd);
extern void dmMotorSendUpdata(FDCAN_TxFrame_TypeDef *txFrame,dmMotorRevConfig_t *motorConfig,uint8_t mode,float pos, float vel, float kp, float kd, float tor);
extern void dmMotorRevUpdata(uint8_t *rxBuf,dmMotorRevConfig_t *motorConfig);
#endif
