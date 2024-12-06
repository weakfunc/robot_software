#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "main.h" 
#include "Driver_Motor.h"

typedef struct{
	TickType_t taskTime;

}motorTaskConfig_t;

void motorUpdataTask(void);
void motorDriverInit(void);

#endif

