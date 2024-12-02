#include "SUPERVISION.h"
#include "usart.h"
#include "stdio.h"

supervisionTaskConfig_t supervisionTaskConfig;

void supervisionDriverInit(){
	ws2812Init();
	beepInit();
	sysLog("supervisionDriverInit", "INFO");
}

void supervisionUpdataTask(){
	supervisionTaskConfig.taskTime++;
	musicUpdata(&driverBeepConifg.state);
	if(supervisionTaskConfig.taskTime%10 == 0){
		if(rgbCmd.ledState == 0){
			ws2812SetColor(GREEN);
		}else{
			ws2812SetColor(CLEAR);
		}
		ws2812Updata();
	}
}

void sysLog(const char str[], const char state[]){
	printf("%s:", state);
	printf("%s\n", str);
}

int fputc(int ch, FILE *f){
		HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
		return ch;
}

