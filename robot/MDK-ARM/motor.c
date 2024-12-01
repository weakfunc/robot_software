#include "MOTOR.h"

motorTaskConfig_t motorTaskConfig;

void motorUpdataTask(){
	motorSendUpdata(&motorSendConfig);
}
