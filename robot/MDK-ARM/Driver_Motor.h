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
} unitreeMotorSendPack_t;  

//电机反馈数据包
typedef struct{
    uint8_t head[2];    // 包头         2Byte
    RIS_Mode_t mode;    // 电机控制模式  1Byte
    RIS_Fbk_t   fbk;    // 电机反馈数据 11Byte
    uint16_t  CRC16;    // CRC          2Byte
} unitreeMotorRevPack_t;  
#pragma pack()

//发送数据配置结构体
//实际给FOC的指令力矩为：
// K_P*delta_Pos + K_W*delta_W + T
typedef struct{
	unitreeMotorSendPack_t motorSendpack;      //电机控制数据结构体
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
    unitreeMotorRevPack_t motorRevpack;        //电机接收数据结构体
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
