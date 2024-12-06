#include "DRIVER_MOTOR.h"
#include "DRIVER_CRC.h"
#include "usart.h"
#include "SUPERVISION.h"
#include "dma.h"
#include "DRIVER_DWT.h"


unitreeMotorSendConfig_t motorSendConfig[UNITREE_MOTOR_NUM];   
unitreeMotorRevConfig_t motorRevConfig[UNITREE_MOTOR_NUM];

driverMotorConfig_t driverMotorConfig;

void unitreeMotorInit(){
	for(int i=0; i<UNITREE_MOTOR_NUM; i++){
		motorSendConfig[i].id = i;
		motorSendConfig[i].mode = 1;
		motorSendConfig[i].t = 0;
		motorSendConfig[i].w = 0;
		motorSendConfig[i].pos = 0;
		motorSendConfig[i].kp = 0;
		motorSendConfig[i].kw = 0;
	}
}

void unitreeMotorTask(){
	for(int i=0; i<UNITREE_MOTOR_NUM; i++){
		unitreeMotorSendUpdata(&motorSendConfig[i]);
		unitreeMotorRevUpdata(&motorRevConfig[i]);
		DWT_Delay(0.0002f);
	}
}

void unitreeMotorSendUpdata(unitreeMotorSendConfig_t *send_config){
//  unitreeLoadPack(send_config);
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
	SET_485_DE_UP();
	HAL_UART_Transmit_DMA(&huart3, (uint8_t *)send_config, sizeof(send_config->motorSendpack));
	SET_485_DE_DOWN();
}

void unitreeMotorRevUpdata(unitreeMotorRevConfig_t *rev_config){
	uint8_t *headCheck = (uint8_t *)&motorRevConfig->motorRevpack;
	HAL_UART_Receive_DMA(&huart3, (uint8_t *)rev_config, sizeof(rev_config->motorRevpack));
	if(headCheck[0] == 0xFD && headCheck[1] == 0xEE){
//	unitreeReceivePack(rev_config);
		if(rev_config->motorRevpack.CRC16 != crc_ccitt(0, (uint8_t *)&rev_config->motorRevpack, 14)){
			sysLog("Receive data CRC error", "WARNING");
			rev_config->correct = 0;
		}
		else{
			rev_config->id = rev_config->motorRevpack.mode.id;
			rev_config->mode = rev_config->motorRevpack.mode.status;
			rev_config->temp = rev_config->motorRevpack.fbk.temp;
			rev_config->errorCode = rev_config->motorRevpack.fbk.MError;
			rev_config->w = ((float)rev_config->motorRevpack.fbk.speed/256)*6.2832f ;
			rev_config->t = ((float)rev_config->motorRevpack.fbk.torque) / 256;
			rev_config->pos = 6.2832f*((float)rev_config->motorRevpack.fbk.pos) / 32768;
			rev_config->footForce = rev_config->motorRevpack.fbk.force;
			rev_config->correct = 1;
		}
	}else{
		sysLog("Receive data Head error", "WARNING");
	}
}
/*
static void unitreeReceivePack(unitreeMotorRevConfig_t *rData){
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

static void unitreeLoadPack(unitreeMotorSendConfig_t *send_config){
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
*/

DM_Motor_Info_Typedef DM_6220_Motor = { //6220电机结构体
	 .CANFrameInfo = {
		.CAN_ID = 0x01, 
	  .Master_ID = 0x01, 
	 },
};

DM_Motor_Control_Typedef DM_Motor_Control;

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


void DM_Motor_Command(FDCAN_TxFrame_TypeDef *TxFrame,uint16_t TxStdId,uint8_t CMD){
	 TxFrame->Header.Identifier = TxStdId;
	 TxFrame->Data[0] = 0xFF;
   TxFrame->Data[1] = 0xFF;
 	 TxFrame->Data[2] = 0xFF;
	 TxFrame->Data[3] = 0xFF;
	 TxFrame->Data[4] = 0xFF;
	 TxFrame->Data[5] = 0xFF;
	 TxFrame->Data[6] = 0xFF;
	 switch(CMD){
		  case Motor_Enable :
	        TxFrame->Data[7] = 0xFC; 
	    break;
			case Motor_Disable :
	        TxFrame->Data[7] = 0xFD; 
      break;
			case Motor_Save_Zero_Position :
	        TxFrame->Data[7] = 0xFE; 
			break;
			default:
	    break;   
	}
   HAL_FDCAN_AddMessageToTxFifoQ(TxFrame->hcan,&TxFrame->Header,TxFrame->Data);
}

void DM_Motor_CAN_TxMessage(FDCAN_TxFrame_TypeDef *TxFrame,DM_Motor_Info_Typedef *DM_Motor,uint8_t Mode,float Postion, float Velocity, float KP, float KD, float Torque){

	 if(Mode > Velocity_Mode) Mode = MIT_Mode;	
		 
	 if(Mode == MIT_Mode) {
		
			 uint16_t Postion_Tmp,Velocity_Tmp,Torque_Tmp,KP_Tmp,KD_Tmp;
			 
			 Postion_Tmp  =  float_to_uint(Postion,-DM_P_MAX,DM_P_MAX,16) ;
			 Velocity_Tmp =  float_to_uint(Velocity,-DM_V_MAX,DM_V_MAX,12);
			 Torque_Tmp = float_to_uint(Torque,-DM_T_MAX,DM_T_MAX,12);
			 KP_Tmp = float_to_uint(KP,0,500,12);
			 KD_Tmp = float_to_uint(KD,0,5,12);

			 TxFrame->Header.Identifier = DM_Motor->CANFrameInfo.CAN_ID;
			 
			 TxFrame->Data[0] = (uint8_t)(Postion_Tmp>>8);
			 TxFrame->Data[1] = (uint8_t)(Postion_Tmp);
			 TxFrame->Data[2] = (uint8_t)(Velocity_Tmp>>4);
			 TxFrame->Data[3] = (uint8_t)((Velocity_Tmp&0x0F)<<4) | (uint8_t)(KP_Tmp>>8);
			 TxFrame->Data[4] = (uint8_t)(KP_Tmp);
			 TxFrame->Data[5] = (uint8_t)(KD_Tmp>>4);
			 TxFrame->Data[6] = (uint8_t)((KD_Tmp&0x0F)<<4) | (uint8_t)(Torque_Tmp>>8);
			 TxFrame->Data[7] = (uint8_t)(Torque_Tmp);
			
			 HAL_FDCAN_AddMessageToTxFifoQ(TxFrame->hcan,&TxFrame->Header,TxFrame->Data);
	 }else if(Mode == Position_Velocity_Mode){
	 
		   KP = 0; KD = 0; Torque = 0;
		 
       uint8_t *Postion_Tmp,*Velocity_Tmp;
		   
		   Postion_Tmp = (uint8_t *)&Postion; 
		   Velocity_Tmp = (uint8_t *)&Velocity; 
		 
	     TxFrame->Header.Identifier = DM_Motor->CANFrameInfo.CAN_ID + 0x100;
			 
			 TxFrame->Data[0] = *(Postion_Tmp);
			 TxFrame->Data[1] = *(Postion_Tmp + 1);
			 TxFrame->Data[2] = *(Postion_Tmp + 2);
			 TxFrame->Data[3] = *(Postion_Tmp + 3);
			 TxFrame->Data[4] = *(Velocity_Tmp);
			 TxFrame->Data[5] = *(Velocity_Tmp + 1);
			 TxFrame->Data[6] = *(Velocity_Tmp + 2);
			 TxFrame->Data[7] = *(Velocity_Tmp + 3);
			
			 HAL_FDCAN_AddMessageToTxFifoQ(TxFrame->hcan,&TxFrame->Header,TxFrame->Data);
	 
	 }else if(Mode == Velocity_Mode){
	 
	     Postion = 0;KP = 0; KD = 0; Torque = 0;
		 
       uint8_t *Velocity_Tmp;
		   
		   Velocity_Tmp = (uint8_t *)&Velocity; 
		 
	     TxFrame->Header.Identifier = DM_Motor->CANFrameInfo.CAN_ID + 0x200;
			 
			 TxFrame->Data[0] = *(Velocity_Tmp);
			 TxFrame->Data[1] = *(Velocity_Tmp + 1);
			 TxFrame->Data[2] = *(Velocity_Tmp + 2);
			 TxFrame->Data[3] = *(Velocity_Tmp + 3);
			 TxFrame->Data[4] = 0;
			 TxFrame->Data[5] = 0;
			 TxFrame->Data[6] = 0;
			 TxFrame->Data[7] = 0;
			
			 HAL_FDCAN_AddMessageToTxFifoQ(TxFrame->hcan,&TxFrame->Header,TxFrame->Data);
	 }
}

void DM_Motor_Info_Update(uint8_t *Data,DM_Motor_Info_Typedef *DM_Motor){	
	  DM_Motor->Data.State = Data[0]>>4;
		DM_Motor->Data.P_int = ((uint16_t)(Data[1]) <<8) | ((uint16_t)(Data[2]));
		DM_Motor->Data.V_int = ((uint16_t)(Data[3]) <<4) | ((uint16_t)(Data[4])>>4);
		DM_Motor->Data.T_int = ((uint16_t)(Data[4]&0xF) <<8) | ((uint16_t)(Data[5]));
		DM_Motor->Data.Torque=  uint_to_float(DM_Motor->Data.T_int,-DM_T_MAX,DM_T_MAX,12);
		DM_Motor->Data.Position=uint_to_float(DM_Motor->Data.P_int,-DM_P_MAX,DM_P_MAX,16);
    DM_Motor->Data.Velocity=uint_to_float(DM_Motor->Data.V_int,-DM_V_MAX,DM_V_MAX,12);
    DM_Motor->Data.Temperature_MOS   = (float)(Data[6]);
	  DM_Motor->Data.Temperature_Rotor = (float)(Data[7]);
}


