#include "DRIVER_MOTOR.h"
#include "DRIVER_CRC.h"
#include "usart.h"

motorSendConfig_t motorSendConfig;   
motorRevConfig_t motorRevConfig;

void motorSendTest(){
	motorSendConfig.id = 0;
	motorSendConfig.mode = 1;
	motorSendConfig.t=0;
	motorSendConfig.w=20;
	motorSendConfig.pos=0;
	motorSendConfig.kp=0;
	motorSendConfig.kw=0.05;
}

static void loadSendPack(motorSendConfig_t *pData){
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

void motorSendUpdata(motorSendConfig_t *pData){
	loadSendPack(pData);
	SET_485_DE_UP();
	HAL_UART_Transmit(&huart3, (uint8_t *)pData, sizeof(pData->motorSendpack), 10);
}
