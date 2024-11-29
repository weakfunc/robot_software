#include "SUPERVISION.h"

rgbCmd_t rgbCmd;

void supervisionTask(){
	rgbCmd.r ++;
	rgbCmd.g += 5;
	rgbCmd.b += 10;
	WS2812_Ctrl(rgbCmd.r, rgbCmd.g, rgbCmd.b);
}

