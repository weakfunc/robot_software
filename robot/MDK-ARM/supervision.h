#ifndef __SUPERVISION_H__
#define __SUPERVISION_H__
#include "main.h" 
#include "DRIVER_WS2812.h"
#include "DRIVER_BEEP.h"

typedef struct{
	TickType_t taskTime;
}supervisionTaskConfig_t;

#define LOG_USART huart1

void supervisionDriverInit(void);
void supervisionUpdataTask(void);
void sysLog(const char str[], const char state[]);

#endif
