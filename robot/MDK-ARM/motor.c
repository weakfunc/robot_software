#include "MOTOR.h"
#include "SUPERVISION.h"

motorTaskConfig_t motorTaskConfig;

void motorDriverInit(){
	unitreeMotorInit();

	sysLog("motorDriverInit", "INFO");
}

void motorUpdataTask(){
	unitreeMotorTask();
	DM_Motor_CAN_TxMessage(&FDCAN1TxFrame,&DM_6220_Motor,MIT_Mode,DM_Motor_Control.Position,DM_Motor_Control.Velocity,DM_Motor_Control.KP,DM_Motor_Control.KD,DM_Motor_Control.Torque);
    /* USER CODE END WHILE */
//	unitreeMotorSendUpdata(&motorSendConfig[0]);
//	unitreeMotorRevUpdata(&motorRevConfig[0]);
	//unitreeMotorSendUpdata(&test);
}
