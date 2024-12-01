#ifndef __SUPERVISION_H__
#define __SUPERVISION_H__
#include "main.h" 
#include "DRIVER_WS2812.h"
#include "DRIVER_BEEP.h"

typedef struct{
	TickType_t taskTime;
}supervisionTaskConfig_t;

void supervisionUpdataTask(void);


#endif
