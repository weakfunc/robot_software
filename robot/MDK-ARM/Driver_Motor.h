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
} motorSendPack_t;  

//����������ݰ�
typedef struct{
    uint8_t head[2];    // ��ͷ         2Byte
    RIS_Mode_t mode;    // �������ģʽ  1Byte
    RIS_Fbk_t   fbk;    // ����������� 11Byte
    uint16_t  CRC16;    // CRC          2Byte
} motorRevPack_t;  

#pragma pack()

//�����������ýṹ��
//ʵ�ʸ�FOC��ָ������Ϊ��
// K_P*delta_Pos + K_W*delta_W + T
typedef struct{
	motorSendPack_t motorSendpack;      //����������ݽṹ��
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
    motorRevPack_t motorRevpack;        //����������ݽṹ��
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
}DM_Motor_CMD_e;

typedef enum{
  MIT_Mode,
  Position_Velocity_Mode,
  Velocity_Mode,
  DM_Motor_Mode_Type_Num,
}DM_Motor_Mode_e;

typedef struct {
  int16_t  State; 	
  uint16_t  P_int;
  uint16_t  V_int;
  uint16_t  T_int;
  float  Position;  
  float  Velocity;  
  float  Torque;  
  float  Temperature_MOS;   
  float  Temperature_Rotor;  
}DM_Motor_Data_Typedef;

typedef struct{
  uint32_t Master_ID;   
  uint32_t CAN_ID;  
}Motor_CANFrameInfo_typedef;

typedef struct{
	uint16_t ID;
  Motor_CANFrameInfo_typedef CANFrameInfo;
	DM_Motor_Data_Typedef Data;  
}DM_Motor_Info_Typedef;


typedef struct{
	float  KP;
	float  KD;
	float  Position; 
  float  Velocity;  	
  float  Torque;  
}DM_Motor_Control_Typedef;

#define LIMIT(IN, MIN, MAX) IN=(IN>MAX)?MAX:((IN<MIN)?MIN:IN)
#define SET_485_DE_UP() HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET)
#define SET_485_DE_DOWN() HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET)
#define UNITREE_MOTOR_NUM 2
#define DM_P_MAX 12.5f
#define DM_V_MAX 45.f
#define DM_T_MAX 10.f

extern unitreeMotorSendConfig_t motorSendConfig[UNITREE_MOTOR_NUM];   
extern unitreeMotorRevConfig_t motorRevConfig[UNITREE_MOTOR_NUM];
extern unitreeMotorSendConfig_t test;

void unitreeMotorInit(void);
static void unitreeLoadPack(unitreeMotorSendConfig_t *pData);
static void unitreeReceivePack(unitreeMotorRevConfig_t *pData);
void unitreeMotorSendUpdata(unitreeMotorSendConfig_t *pData);
void unitreeMotorRevUpdata(unitreeMotorRevConfig_t *rData);
void unitreeMotorTask(void);

extern DM_Motor_Info_Typedef DM_6220_Motor;

extern DM_Motor_Control_Typedef DM_Motor_Control;

extern void DM_Motor_Info_Update(uint8_t *rxBuf,DM_Motor_Info_Typedef *DM_Motor);

extern void DM_Motor_Multi_Info_Update(uint8_t *Data,DM_Motor_Info_Typedef *DM_Motor);

extern void DM_Motor_Command(FDCAN_TxFrame_TypeDef *TxFrame,uint16_t TxStdId,uint8_t CMD);

extern void DM_Motor_CAN_TxMessage(FDCAN_TxFrame_TypeDef *TxFrame,DM_Motor_Info_Typedef *DM_Motor,uint8_t Mode,
	                                             float Postion, float Velocity, float KP, float KD, float Torque);
#endif
