#ifndef __DRIVER_MOTOR_H__
#define __DRIVER_MOTOR_H__
#include "main.h" 


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
typedef struct{
    motorSendPack_t motorSendpack;   //����������ݽṹ��
    int hexLen;                        //���͵�16�����������鳤��, 34
    long long sendTime;                //���͸������ʱ��, ΢��(us)
    // �����͵ĸ�������
    unsigned short id;                  //���ID��0����ȫ�����
    unsigned short mode;                // 0:����, 5:����ת��, 10:�ջ�FOC����
    //ʵ�ʸ�FOC��ָ������Ϊ��
    // K_P*delta_Pos + K_W*delta_W + T
    float t;                            //�����ؽڵ�������أ�������������أ���Nm��
    float w;                            //�����ؽ��ٶȣ�����������ٶȣ�(rad/s)
    float pos;                          //�����ؽ�λ�ã�rad��
    float kp;                           //�ؽڸն�ϵ��
    float kw;                          //�ؽ��ٶ�ϵ��
    COMData32 res;                    	//ͨѶ�������ֽڣ�����ʵ�ֱ��һЩͨѶ����
} motorSendConfig_t;

//�����������ýṹ��
typedef struct{
    motorRevPack_t motor_recv_data;     //����������ݽṹ��
    int hex_len;                        //���յ�16�����������鳤��, 78
    long long resv_time;                //���ո������ʱ��, ΢��(us)
    int correct;                        //���������Ƿ�������1������0��������
    //����ó��ĵ������
    unsigned char motor_id;             //���ID
    unsigned char mode;                 //0:����, 5:����ת��, 10:�ջ�FOC����
    int Temp;                           //�¶�
    unsigned char MError;               //������
    float T;                            //��ǰʵ�ʵ���������
		float W;														//speed
    float Pos;                          //��ǰ���λ�ã�����0������������ؽڻ����Ա�����0��Ϊ׼��
		float footForce;										//�����ѹ���������� 12bit (0-4095)
} motorRevConfig_t;


#define LIMIT(IN, MIN, MAX) IN=(IN>MAX)?MAX:((IN<MIN)?MIN:IN)
#define SET_485_DE_UP() HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET)
#define SET_485_DE_DOWN() HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET)
#define SET_485_RE_UP() HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_SET)
#define SET_485_RE_DOWN() HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET)


extern motorSendConfig_t motorSendConfig;   
extern motorRevConfig_t motorRevConfig;
void motorSendTest(void);
static void loadSendPack(motorSendConfig_t *pData);
void motorSendUpdata(motorSendConfig_t *pData);

#endif