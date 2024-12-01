#include "DRIVER_WS2812.h"
#define WS2812_LOWLEVEL    0xC0     // 0Ты
#define WS2812_HIGHLEVEL   0xF0     // 1Ты
rgbCmdStruct_t rgbCmd;

void ws2812Init(){
	rgbCmd.r = 0;
	rgbCmd.g = 255;
	rgbCmd.b = 0;
	rgbCmd.ledState = 0;
}

void ws2812Updata(){
	uint8_t txbuf[24];
	uint8_t res = 0;
	for (int i = 0; i < 8; i++)
	{
			txbuf[7-i]  = (((rgbCmd.g>>i)&0x01) ? WS2812_HIGHLEVEL : WS2812_LOWLEVEL)>>1;
			txbuf[15-i] = (((rgbCmd.r>>i)&0x01) ? WS2812_HIGHLEVEL : WS2812_LOWLEVEL)>>1;
			txbuf[23-i] = (((rgbCmd.b>>i)&0x01) ? WS2812_HIGHLEVEL : WS2812_LOWLEVEL)>>1;
	}
	HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 0, 0xFFFF);
	while (WS2812_SPI_UNIT.State != HAL_SPI_STATE_READY);
	HAL_SPI_Transmit(&WS2812_SPI_UNIT, txbuf, 24, 0xFFFF);
	for (int i = 0; i < 100; i++)
	{
			HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 1, 0xFFFF);
	}
}

void ws2812SetColor(uint8_t color){
	switch(color){
		case GREEN: 
			rgbCmd.r = 0;
			rgbCmd.g = 255;
			rgbCmd.b = 0;
			rgbCmd.ledState = 1;
		break;
		case RED: 
			rgbCmd.r = 255;
			rgbCmd.g = 0;
			rgbCmd.b = 0;
			rgbCmd.ledState = 1;
		break;
		case BLUE: 
			rgbCmd.r = 0;
			rgbCmd.g = 0;
			rgbCmd.b = 255;
			rgbCmd.ledState = 1;
		break;
		case CLEAR: 
			rgbCmd.r = 0;
			rgbCmd.g = 0;
			rgbCmd.b = 0;
			rgbCmd.ledState = 0;
		break;
	}
}
