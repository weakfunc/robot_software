#ifndef __DRIVER_WS2812_H__
#define __DRIVER_WS2812_H__
#include "main.h" 
#define WS2812_SPI_UNIT     hspi6
#define TOGGLE_TIME 10

typedef struct{
	uint8_t r;
	uint8_t g;
	uint8_t b;
	uint8_t ledState;
}rgbCmdStruct_t;

enum rgbColor{
	RED = 0,
	GREEN,
	BLUE,
	YELLOW,
	CLEAR,
};

extern SPI_HandleTypeDef WS2812_SPI_UNIT;
extern rgbCmdStruct_t rgbCmd;

void ws2812Init(void);
void ws2812Updata(void);
void ws2812SetColor(uint8_t color);
#endif
