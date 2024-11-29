#ifndef __SUPERVISION_H__
#define __SUPERVISION_H__
#include "main.h" 
#include "Driver_ws2812.h"

typedef struct{
	uint8_t r;
	uint8_t g;
	uint8_t b;
}rgbCmd_t;

void supervisionTask(void);


#endif
