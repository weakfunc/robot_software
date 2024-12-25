#include "MOTOR.h"
#include "SUPERVISION.h"

motorTaskConfig_t motorTaskConfig;

void motorDriverInit(){
	unitreeMotorInit();
	dmMotorInit();
	sysLog("motorDriverInit", "INFO");
}

void motorUpdataTask(){
	unitreeMotorTask();
	dmMotorTask();

    /* USER CODE END WHILE */
//	unitreeMotorSendUpdata(&motorSendConfig[0]);
//	unitreeMotorRevUpdata(&motorRevConfig[0]);
	//unitreeMotorSendUpdata(&test);
}
