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

// 包参数，控制模式 1Byte
typedef struct{
    uint8_t id     :4;      // 电机ID: 0,1...,13,14 15表示向所有电机广播数据(此时无返回)
    uint8_t status :3;      // 工作模式: 0.锁定 1.FOC闭环 2.编码器校准 3.保留
    uint8_t none   :1;      // 保留位
} RIS_Mode_t;  

// 包参数，控制参数 12Byte
typedef struct{
    int16_t tor_des;        // 期望关节输出扭矩 unit: N.m      (q8)
    int16_t spd_des;        // 期望关节输出速度 unit: rad/s    (q8)
    int32_t pos_des;        // 期望关节输出位置 unit: rad      (q15)
    int16_t k_pos;          // 期望关节刚度系数 unit: -1.0-1.0 (q15)
    int16_t k_spd;          // 期望关节阻尼系数 unit: -1.0-1.0 (q15) 
} RIS_Comd_t;   

// 包参数，状态数据 11Byte
typedef struct{
    int16_t  torque;        // 实际关节输出扭矩 unit: N.m     (q8)
    int16_t  speed;         // 实际关节输出速度 unit: rad/s   (q8)
    int32_t  pos;           // 实际关节输出位置 unit: rad     (q15)
    int8_t   temp;          // 电机温度: -128~127°C
    uint8_t  MError :3;     // 电机错误标识: 0.正常 1.过热 2.过流 3.过压 4.编码器故障 5-7.保留
    uint16_t force  :12;    // 足端气压传感器数据 12bit (0-4095)
    uint8_t  none   :1;     // 保留位
} RIS_Fbk_t;  

//电机控制命令数据包
typedef struct{
    // 定义 电机控制命令数据包
    uint8_t head[2];    // 包头         2Byte
    RIS_Mode_t mode;    // 电机控制模式  1Byte
    RIS_Comd_t comd;    // 电机期望数据 12Byte
    uint16_t   CRC16;   // CRC          2Byte
} motorSendPack_t;  

//电机反馈数据包
typedef struct{
    uint8_t head[2];    // 包头         2Byte
    RIS_Mode_t mode;    // 电机控制模式  1Byte
    RIS_Fbk_t   fbk;    // 电机反馈数据 11Byte
    uint16_t  CRC16;    // CRC          2Byte
} motorRevPack_t;  

#pragma pack()

//发送数据配置结构体
//实际给FOC的指令力矩为：
// K_P*delta_Pos + K_W*delta_W + T
typedef struct{
	motorSendPack_t motorSendpack;      //电机控制数据结构体
	unsigned short id;									//电机ID，0代表全部电机
	int hexLen;                         //发送的16进制命令数组长度, 34
	long long sendTime;                 //发送该命令的时间, 微秒(us)         
	unsigned short mode;                // 0:空闲, 5:开环转动, 10:闭环FOC控制
	float t;                            //期望关节的输出力矩（电机本身的力矩）（Nm）
	float w;                            //期望关节速度（电机本身的速度）(rad/s)
	float pos;                          //期望关节位置（rad）
	float kp;                           //关节刚度系数
	float kw;                           //关节速度系数
	COMData32 res;                    	//通讯，保留字节，用于实现别的一些通讯内容	
} unitreeMotorSendConfig_t;

//接收数据配置结构体
typedef struct{
    motorRevPack_t motorRevpack;        //电机接收数据结构体
		unsigned char id;              			//电机ID
    int hexLen;                         //接收的16进制命令数组长度, 78
    long long resvTime;                 //接收该命令的时间, 微秒(us)
    unsigned char mode;                 //0:空闲, 5:开环转动, 10:闭环FOC控制
    float t;                            //当前实际电机输出力矩
		float w;														//speed
    float pos;                          //当前电机位置（主控0点修正，电机关节还是以编码器0点为准）
		float footForce;										//足端气压传感器数据 12bit (0-4095)
		int correct;                        //接收数据是否完整（1完整，0不完整）
		int temp;                           //温度
		unsigned char errorCode;            //错误码
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
