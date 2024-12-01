#include "SUPERVISION.h"
supervisionTaskConfig_t supervisionTaskConfig;

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

