#include "MOTOR.h"

motorTaskConfig_t motorTaskConfig;

void motorUpdataTask(){
	unitreeMotorSendUpdata(&motorSendConfig);
	unitreeMotorRevUpdata(&motorRevConfig);
	//unitreeMotorSendUpdata(&test);
}
