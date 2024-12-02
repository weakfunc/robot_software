#include "DRIVER_MOTOR.h"
#include "DRIVER_CRC.h"
#include "usart.h"
#include "SUPERVISION.h"
#include "dma.h"
#include "DRIVER_DWT.h"

uniTreeMotorSendConfig_t motorSendConfig[UNITREE_MOTOR_NUM];   
uniTreeMotorRevConfig_t motorRevConfig[UNITREE_MOTOR_NUM];

uniTreeMotorSendConfig_t test;
driverMotorConfig_t driverMotorConfig;


uint8_t rxTemp[20];

void motorSendTest(){
	for(int i=0; i<UNITREE_MOTOR_NUM; i++){
		motorSendConfig[i].id = i;
		motorSendConfig[i].mode = 1;
		motorSendConfig[i].t=0;
		motorSendConfig[i].w=20;
		motorSendConfig[i].pos=0;
		motorSendConfig[i].kp=0;
		motorSendConfig[i].kw=0.05;
	}
	HAL_UART_Receive_DMA(&huart3, rxTemp, 16);
}





static void unitreeLoadPack(uniTreeMotorSendConfig_t *send_config){
	send_config->hexLen = 17;
	
	send_config->motorSendpack.head[0] = 0xFE;
	send_config->motorSendpack.head[1] = 0xEE;
	
	LIMIT(send_config->kp, 0.0f, 25.599f);
	LIMIT(send_config->kw, 0.0f, 25.599f);
	LIMIT(send_config->t, -127.99f, 127.99f);
	LIMIT(send_config->w, -804.00f, 804.00f);
	LIMIT(send_config->pos, -411774.0f, 411774.0f);
	
	send_config->motorSendpack.mode.id   = send_config->id;
	send_config->motorSendpack.mode.status  = send_config->mode;
	send_config->motorSendpack.comd.k_pos  = send_config->kp/25.6f*32768;
	send_config->motorSendpack.comd.k_spd  = send_config->kw/25.6f*32768;
	send_config->motorSendpack.comd.pos_des  = send_config->pos/6.2832f*32768;
	send_config->motorSendpack.comd.spd_des  = send_config->w/6.2832f*256;
	send_config->motorSendpack.comd.tor_des  = send_config->t*256;
	send_config->motorSendpack.CRC16 = crc_ccitt(0, (uint8_t *)&send_config->motorSendpack, 15);
}
static void unitreeReceivePack(uniTreeMotorRevConfig_t *rData){
	if(rData->motorRevpack.CRC16 != crc_ccitt(0, (uint8_t *)&rData->motorRevpack, 14)){
			sysLog("Receive data CRC error", "WARNING");
			rData->correct = 0;
	}
	else{
			rData->id = rData->motorRevpack.mode.id;
			rData->mode = rData->motorRevpack.mode.status;
			rData->temp = rData->motorRevpack.fbk.temp;
			rData->errorCode = rData->motorRevpack.fbk.MError;
			rData->w = ((float)rData->motorRevpack.fbk.speed/256)*6.2832f ;
			rData->t = ((float)rData->motorRevpack.fbk.torque) / 256;
			rData->pos = 6.2832f*((float)rData->motorRevpack.fbk.pos) / 32768;
			rData->footForce = rData->motorRevpack.fbk.force;
			rData->correct = 1;
	}
}
uint8_t *headCheck = (uint8_t *)&motorRevConfig->motorRevpack;
void unitreeMotorSendUpdata(uniTreeMotorSendConfig_t *send_config){
	unitreeLoadPack(send_config);
	SET_485_DE_UP();
	HAL_UART_Transmit_DMA(&huart3, (uint8_t *)send_config, sizeof(send_config->motorSendpack));
	SET_485_DE_DOWN();
}
void unitreeMotorRevUpdata(uniTreeMotorRevConfig_t *rev_config){
//	uint8_t *headCheck = (uint8_t *)&rev_config->motorRevpack;
	HAL_UART_Receive_DMA(&huart3, (uint8_t *)rev_config, sizeof(rev_config->motorRevpack));
	if(headCheck[0] == 0xFD && headCheck[1] == 0xEE){
			unitreeReceivePack(rev_config);
	}
}

void unitreeTask(){

		unitreeMotorSendUpdata(&motorSendConfig[0]);
		unitreeMotorRevUpdata(&motorRevConfig[0]);
		DWT_Delay(0.0002f);
		unitreeMotorSendUpdata(&motorSendConfig[1]);
		unitreeMotorRevUpdata(&motorRevConfig[1]);
//		DWT_Delay(0.0002f);
}



