#include "DRIVER_MOTOR.h"
#include "DRIVER_CRC.h"
#include "usart.h"
#include "SUPERVISION.h"
#include "dma.h"

uniTreeMotorSendConfig_t motorSendConfig;   
uniTreeMotorRevConfig_t motorRevConfig;

uniTreeMotorSendConfig_t test;
driverMotorConfig_t driverMotorConfig;


uint8_t rxTemp[20];

void motorSendTest(uniTreeMotorSendConfig_t *data){
	data->id = 0;
	data->mode = 1;
	data->t=0;
	data->w=20;
	data->pos=0;
	data->kp=0;
	data->kw=0.05;
	HAL_UART_Receive_DMA(&huart3, rxTemp, 17);
}

static void unitreeLoadPack(uniTreeMotorSendConfig_t *pData){
	pData->hexLen = 17;
	
	pData->motorSendpack.head[0] = 0xFE;
	pData->motorSendpack.head[1] = 0xEE;
	
	LIMIT(pData->kp, 0.0f, 25.599f);
	LIMIT(pData->kw, 0.0f, 25.599f);
	LIMIT(pData->t, -127.99f, 127.99f);
	LIMIT(pData->w, -804.00f, 804.00f);
	LIMIT(pData->pos, -411774.0f, 411774.0f);
	
	pData->motorSendpack.mode.id   = pData->id;
	pData->motorSendpack.mode.status  = pData->mode;
	pData->motorSendpack.comd.k_pos  = pData->kp/25.6f*32768;
	pData->motorSendpack.comd.k_spd  = pData->kw/25.6f*32768;
	pData->motorSendpack.comd.pos_des  = pData->pos/6.2832f*32768;
	pData->motorSendpack.comd.spd_des  = pData->w/6.2832f*256;
	pData->motorSendpack.comd.tor_des  = pData->t*256;
	pData->motorSendpack.CRC16 = crc_ccitt(0, (uint8_t *)&pData->motorSendpack, 15);
}

static void unitreeReceivePack(uniTreeMotorRevConfig_t *rData){
	if(rData->motorRevpack.CRC16 != crc_ccitt(0, (uint8_t *)&rData->motorRevpack, 14)){
			sysLog("Receive data CRC error", "WARNING");
			rData->correct = 0;
	}
	else{
			rData->motorId = rData->motorRevpack.mode.id;
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

void unitreeMotorSendUpdata(uniTreeMotorSendConfig_t *pData){
	unitreeLoadPack(pData);
	SET_485_DE_UP();
	HAL_UART_Transmit(&huart3, (uint8_t *)pData, sizeof(pData->motorSendpack), 10);
	SET_485_DE_DOWN();
}



void unitreeMotorRevUpdata(uniTreeMotorRevConfig_t *rData){
//	uint16_t rxlen = 0;
//  HAL_UARTEx_ReceiveToIdle(&huart3, (uint8_t *)rData, sizeof(rData->motorRevpack), &rxlen, 10);
//  if(rxlen == 0) 
//		sysLog("MotorRevTimeout", "ERROR");
//  if(rxlen != sizeof(rData->motorRevpack))
//		sysLog("MotorRevLenError", "ERROR");
	uint8_t *revPack = (uint8_t *)&motorRevConfig.motorRevpack;
	HAL_UART_Receive_DMA(&huart3, (uint8_t *)rData, sizeof(rData->motorRevpack));
	if(revPack[0] == 0xFD && revPack[1] == 0xEE){
			rData->correct = 1;
			unitreeReceivePack(rData);
	}
}

