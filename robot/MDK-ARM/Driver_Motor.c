#include "DRIVER_MOTOR.h"
#include "DRIVER_CRC.h"
#include "usart.h"
#include "SUPERVISION.h"
#include "dma.h"
#include "DRIVER_DWT.h"



unitreeMotorRevConfig_t unitreeMotorRevConfig[UNITREE_MOTOR_NUM];
unitreeMotorRevPack_t unitreeMotorRevPack;
driverMotorConfig_t driverMotorConfig;

unitreeMotorSendConfig_t unitreeMotorSendConfig[UNITREE_MOTOR_NUM] = {
{ .id = 0, .mode = 1, .t = 0, .w = 10, .pos = 0, .kp = 0, .kw = 0.02},
{ .id = 1, .mode = 1, .t = 0, .w = 15, .pos = 0, .kp = 0, .kw = 0.02},
{ .id = 2, .mode = 1, .t = 0, .w = 20, .pos = 0, .kp = 0, .kw = 0.02},
{ .id = 3, .mode = 1, .t = 0, .w = 25, .pos = 0, .kp = 0, .kw = 0.02}
};   
dmMotorRevConfig_t dmMotorRevConfig[2] = { 
{	.canFrameInfo = {.CAN_ID = 0x01, .Master_ID = 0x11, },},
{ .canFrameInfo = {.CAN_ID = 0x02, .Master_ID = 0x12, },}
};
dmMotorSendConfig_t dmMotorSendConfig[2] = {
{ .vel = 5.0f, .kd = 0.07f},
{ .vel = 10.0f, .kd = 0.07f},
};


/*
brief:宇树电机初始化
return:NULL
commit:NULL
*/
void unitreeMotorInit(){
//	for(int i=0; i<UNITREE_MOTOR_NUM; i++){
//		unitreeMotorSendConfig[i].id = i;
//		unitreeMotorSendConfig[i].mode = 1;
//		unitreeMotorSendConfig[i].t = 0;
//		unitreeMotorSendConfig[i].w = 10;
//		unitreeMotorSendConfig[i].pos = 0;
//		unitreeMotorSendConfig[i].kp = 0;
//		unitreeMotorSendConfig[i].kw = 0.02;
//	}
}

/*
brief:宇树电机任务
return:NULL
commit:NULL
*/
void unitreeMotorTask(){
	for(int i=0; i<UNITREE_MOTOR_NUM; i++){
		unitreeMotorSendUpdata(&unitreeMotorSendConfig[i]);
		unitreeMotorRevUpdata(unitreeMotorRevConfig, &unitreeMotorRevPack);
		DWT_Delay(0.0002f);
	}
}

/*
brief:达妙电机初始化
return:NULL
commit:NULL
*/
void dmMotorInit(){
	dmMotorSendCMD(&FDCAN1TxFrame, 0x01, enable);
	DWT_Delay(0.0002f);
	dmMotorSendCMD(&FDCAN1TxFrame, 0x02, enable);
	
//	for(int i=0; i<DM_MOTOR_NUM; i++){
//		dmMotorSendConfig[i].vel = 5.0f;
//		dmMotorSendConfig[i].kd = 0.07f;
//	}
}

/*
brief:达妙电机任务
return:NULL
commit:NULL
*/
void dmMotorTask(){
	dmMotorSendUpdata(&FDCAN1TxFrame,&dmMotorRevConfig[0],mit_mode,dmMotorSendConfig[0].pos,dmMotorSendConfig[0].vel,dmMotorSendConfig[0].kp,dmMotorSendConfig[0].kd,dmMotorSendConfig[0].tor);
	DWT_Delay(0.0002f);
	dmMotorSendUpdata(&FDCAN1TxFrame,&dmMotorRevConfig[1],mit_mode,dmMotorSendConfig[1].pos,dmMotorSendConfig[1].vel,dmMotorSendConfig[1].kp,dmMotorSendConfig[1].kd,dmMotorSendConfig[1].tor);
}

/*
brief:宇树电机发送更新
return:NULL
commit:NULL
*/
void unitreeMotorSendUpdata(unitreeMotorSendConfig_t *motorConfig){
	motorConfig->hexLen = 17;
	motorConfig->motorSendpack.head[0] = 0xFE;
	motorConfig->motorSendpack.head[1] = 0xEE;
	
	LIMIT(motorConfig->kp, 0.0f, 25.599f);
	LIMIT(motorConfig->kw, 0.0f, 25.599f);
	LIMIT(motorConfig->t, -127.99f, 127.99f);
	LIMIT(motorConfig->w, -804.00f, 804.00f);
	LIMIT(motorConfig->pos, -411774.0f, 411774.0f); 
	
	motorConfig->motorSendpack.mode.id   = motorConfig->id;
	motorConfig->motorSendpack.mode.status  = motorConfig->mode;
	motorConfig->motorSendpack.comd.k_pos  = motorConfig->kp/25.6f*32768;
	motorConfig->motorSendpack.comd.k_spd  = motorConfig->kw/25.6f*32768;
	motorConfig->motorSendpack.comd.pos_des  = motorConfig->pos/6.2832f*32768;
	motorConfig->motorSendpack.comd.spd_des  = motorConfig->w/6.2832f*256;
	motorConfig->motorSendpack.comd.tor_des  = motorConfig->t*256;
	motorConfig->motorSendpack.CRC16 = crc_ccitt(0, (uint8_t *)&motorConfig->motorSendpack, 15);
	
	SET_485_DE_UP();
	if(motorConfig->id < 2)
		HAL_UART_Transmit_DMA(&huart3, (uint8_t *)motorConfig, sizeof(motorConfig->motorSendpack));
	else
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)motorConfig, sizeof(motorConfig->motorSendpack));
	SET_485_DE_DOWN();
}

/*
brief:宇树电机反馈数据解析，删去了CRC校验
return:电机ID溢出判断。1：错误 0：正常
commit:编码器是15bit，2^15=32768。参数均为转子侧。减速比为6.33
*/
uint8_t unitreeMotorRevUpdata(unitreeMotorRevConfig_t *motorConfig, unitreeMotorRevPack_t *revPack){
	uint8_t id;
	HAL_UART_Receive_DMA(&huart3, (uint8_t *)revPack, sizeof(unitreeMotorRevPack_t));
	HAL_UART_Receive_DMA(&huart2, (uint8_t *)revPack, sizeof(unitreeMotorRevPack_t));
	
	if(revPack->head[0] == 0xFD && revPack->head[1] == 0xEE){
		id = revPack->mode.id;
		if(id > UNITREE_MOTOR_NUM) return 1;
		motorConfig[id].id = id;
		motorConfig[id].mode = revPack->mode.status;
		motorConfig[id].temp = revPack->fbk.temp;
		motorConfig[id].errorCode = revPack->fbk.MError;
		motorConfig[id].w = ((float)revPack->fbk.speed/256)*6.2832f ;
		motorConfig[id].t = ((float)revPack->fbk.torque)/256;
		motorConfig[id].pos = 6.2832f*((float)revPack->fbk.pos)/32768;
		motorConfig[id].footForce = revPack->fbk.force;
		motorConfig[id].correct = 1;
//		motorConfig[id].enconder = (int)(motorConfig[id].pos * (180/3.1416f)) % 360;		
		return 0;
	}else{
		sysLog("Unitree Motor Receive data Head error", "WARNING");
		return 1;
	}
}

static float uint_to_float(int X_int, float X_min, float X_max, int Bits){
	float span = X_max - X_min;
	float offset = X_min;
	return ((float)X_int)*span/((float)((1<<Bits)-1)) + offset;
}

static int float_to_uint(float X_float, float X_min, float X_max, int bits){
	float span = X_max - X_min;
	float offset = X_min;
	return (int) ((X_float-offset)*((float)((1<<bits)-1))/span);
}

/*
brief:达妙电机发送模式控制数据
return:NULL
commit:NULL
*/
void dmMotorSendCMD(FDCAN_TxFrame_TypeDef *txFrame,uint16_t txId,uint8_t cmd){
	txFrame->Header.Identifier = txId;
	txFrame->Data[0] = 0xFF;
	txFrame->Data[1] = 0xFF;
	txFrame->Data[2] = 0xFF;
	txFrame->Data[3] = 0xFF;
	txFrame->Data[4] = 0xFF;
	txFrame->Data[5] = 0xFF;
	txFrame->Data[6] = 0xFF;
	switch(cmd){
	case enable: txFrame->Data[7] = 0xFC; break;
	case disable: txFrame->Data[7] = 0xFD; break;
	case save_zero_pos: txFrame->Data[7] = 0xFE; break;
	default: break;   
	}
	HAL_FDCAN_AddMessageToTxFifoQ(txFrame->hcan,&txFrame->Header,txFrame->Data);
}

/*
brief:达妙电机发送更新
return:NULL
commit:NULL
*/
void dmMotorSendUpdata(FDCAN_TxFrame_TypeDef *txFrame,dmMotorRevConfig_t *motorConfig,uint8_t mode,float pos, float vel, float kp, float kd, float tor){

	if(mode > vel_mode) mode = mit_mode;	
	 
	if(mode == mit_mode) {

		 uint16_t pos_temp, vel_temp, tor_temp, kp_temp, kd_temp;
		 
		 pos_temp  =  float_to_uint(pos,-DM_P_MAX,DM_P_MAX,16) ;
		 vel_temp =  float_to_uint(vel,-DM_V_MAX,DM_V_MAX,12);
		 tor_temp = float_to_uint(tor,-DM_T_MAX,DM_T_MAX,12);
		 kp_temp = float_to_uint(kp,0,500,12);
		 kd_temp = float_to_uint(kd,0,5,12);

		 txFrame->Header.Identifier = motorConfig->canFrameInfo.CAN_ID;
		 txFrame->Data[0] = (uint8_t)(pos_temp>>8);
		 txFrame->Data[1] = (uint8_t)(pos_temp);
		 txFrame->Data[2] = (uint8_t)(vel_temp>>4);
		 txFrame->Data[3] = (uint8_t)((vel_temp&0x0F)<<4) | (uint8_t)(kp_temp>>8);
		 txFrame->Data[4] = (uint8_t)(kp_temp);
		 txFrame->Data[5] = (uint8_t)(kd_temp>>4);
		 txFrame->Data[6] = (uint8_t)((kd_temp&0x0F)<<4) | (uint8_t)(tor_temp>>8);
		 txFrame->Data[7] = (uint8_t)(tor_temp);
		
		 HAL_FDCAN_AddMessageToTxFifoQ(txFrame->hcan,&txFrame->Header,txFrame->Data);
	}else if(mode == pos_vel_mode){

		 kp = 0; kd = 0; tor = 0;
	 
		 uint8_t *pos_temp, *vel_temp;
		 
		 pos_temp = (uint8_t *)&pos; 
		 vel_temp = (uint8_t *)&vel; 
	 
		 txFrame->Header.Identifier = motorConfig->canFrameInfo.CAN_ID + 0x100;
		 
		 txFrame->Data[0] = *(pos_temp);
		 txFrame->Data[1] = *(pos_temp + 1);
		 txFrame->Data[2] = *(pos_temp + 2);
		 txFrame->Data[3] = *(pos_temp + 3);
		 txFrame->Data[4] = *(vel_temp);
		 txFrame->Data[5] = *(vel_temp + 1);
		 txFrame->Data[6] = *(vel_temp + 2);
		 txFrame->Data[7] = *(vel_temp + 3);
		
		 HAL_FDCAN_AddMessageToTxFifoQ(txFrame->hcan,&txFrame->Header,txFrame->Data);

	}else if(mode == vel_mode){

		 pos = 0;kp = 0; kd = 0; tor = 0;
	 
		 uint8_t *vel_temp;
		 
		 vel_temp = (uint8_t *)&vel; 
	 
		 txFrame->Header.Identifier = motorConfig->canFrameInfo.CAN_ID + 0x200;
		 
		 txFrame->Data[0] = *(vel_temp);
		 txFrame->Data[1] = *(vel_temp + 1);
		 txFrame->Data[2] = *(vel_temp + 2);
		 txFrame->Data[3] = *(vel_temp + 3);
		 txFrame->Data[4] = 0;
		 txFrame->Data[5] = 0;
		 txFrame->Data[6] = 0;
		 txFrame->Data[7] = 0;
		
		 HAL_FDCAN_AddMessageToTxFifoQ(txFrame->hcan,&txFrame->Header,txFrame->Data);
	}
	}

/*
brief:达妙电机接收更新
return:NULL
commit:NULL
*/
void dmMotorRevUpdata(uint8_t *Data,dmMotorRevConfig_t *motorConfig){	
	  motorConfig->revPack.state = Data[0]>>4;
		motorConfig->revPack.pos_int = ((uint16_t)(Data[1]) <<8) | ((uint16_t)(Data[2]));
		motorConfig->revPack.vel_int = ((uint16_t)(Data[3]) <<4) | ((uint16_t)(Data[4])>>4);
		motorConfig->revPack.tor_int = ((uint16_t)(Data[4]&0xF) <<8) | ((uint16_t)(Data[5]));
		motorConfig->revPack.tor=  uint_to_float(motorConfig->revPack.tor_int,-DM_T_MAX,DM_T_MAX,12);
		motorConfig->revPack.pos=uint_to_float(motorConfig->revPack.pos_int,-DM_P_MAX,DM_P_MAX,16);
    motorConfig->revPack.vel=uint_to_float(motorConfig->revPack.vel_int,-DM_V_MAX,DM_V_MAX,12);
    motorConfig->revPack.temp_mos   = (float)(Data[6]);
	  motorConfig->revPack.temp_rotor = (float)(Data[7]);
}

/*
  brief：宇树官方SDK，电机反馈解析
*/
//void unitreeMotorRevUpdata(unitreeMotorRevConfig_t *rev_config){
//	uint8_t *headCheck = (uint8_t *)&motorRevConfig->motorRevpack;
//	HAL_UART_Receive_DMA(&huart3, (uint8_t *)rev_config, sizeof(rev_config->motorRevpack));
//	
//	if(headCheck[0] == 0xFD && headCheck[1] == 0xEE){
//		if(rev_config->motorRevpack.CRC16 != crc_ccitt(0, (uint8_t *)&rev_config->motorRevpack, 14)){
//			sysLog("Unitree Motor Receive data CRC error", "WARNING");
//			rev_config->correct = 0;
//		}
//		else{
//			rev_config->id = rev_config->motorRevpack.mode.id;
//			rev_config->mode = rev_config->motorRevpack.mode.status;
//			rev_config->temp = rev_config->motorRevpack.fbk.temp;
//			rev_config->errorCode = rev_config->motorRevpack.fbk.MError;
//			rev_config->w = ((float)rev_config->motorRevpack.fbk.speed/256)*6.2832f ;
//			rev_config->t = ((float)rev_config->motorRevpack.fbk.torque) / 256;
//			rev_config->pos = 6.2832f*((float)rev_config->motorRevpack.fbk.pos) / 32768;
//			rev_config->footForce = rev_config->motorRevpack.fbk.force;
//			rev_config->correct = 1;	
//		}
//	}else{
//		sysLog("Unitree Motor Receive data Head error", "WARNING");
//	}
//}
