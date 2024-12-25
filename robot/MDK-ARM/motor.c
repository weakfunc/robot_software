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
}
