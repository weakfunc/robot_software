#include "DRIVER_BEEP.h"


driverBeepConifg_t driverBeepConifg;

const uint16_t note[] = {
	480, 428, 381, 359, 321, 286, 255,		//低八度
	240, 214, 190, 179, 160, 143, 127,					//中八度
	120, 107, 95, 90, 80, 71, 64,						//高八度
};
const uint16_t sound[] = {
500 ,0
};

const muiscList_t musicList[] = {
//无
	{{},{},1},			
//上控
	{
		{ALTO_DO,ALTO_DO,ALTO_DO,ALTO_DO,ALTO_DO,ALTO_DO,ALTO_DO,ALTO_DO,ALTO_DO,ALTO_RE,ALTO_RE,ALTO_RE,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI},    
		{H_NOISE,H_NOISE,H_NOISE,H_NOISE,N_NOISE,N_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,N_NOISE},16
	},
//下控
	{
		{ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_RE,ALTO_RE,ALTO_RE,ALTO_DO,ALTO_DO,ALTO_DO,ALTO_DO},		
		{H_NOISE,H_NOISE,H_NOISE,H_NOISE,N_NOISE,N_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,N_NOISE},16
	},
	{
		{BASS_DO,BASS_DO,BASS_DO,BASS_DO,BASS_DO,BASS_DO,BASS_DO,BASS_DO,BASS_DO,BASS_RE,BASS_RE,BASS_RE,BASS_MI,BASS_MI,BASS_MI,BASS_MI},    
		{H_NOISE,H_NOISE,H_NOISE,H_NOISE,N_NOISE,N_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,N_NOISE},16
	},
};

void musicUpdata(uint8_t *state){
	static int i = 0;
	static uint8_t *musicFrequentLoad = NULL;
	static uint8_t *musicSoundtLoad = NULL;
	if(*state != NORMAL || driverBeepConifg.remain == 0x01){
		if(driverBeepConifg.firstHere == 0){
			driverBeepConifg.remain = 1;
			driverBeepConifg.firstHere = 1;
			musicFrequentLoad = (uint8_t*)pvPortMalloc(sizeof(uint8_t) * (musicList[*state].len + 1));
			musicSoundtLoad = (uint8_t*)pvPortMalloc(sizeof(uint8_t) * (musicList[*state].len + 1));
			for(int j = 0; j < musicList[*state].len; j++){
				musicFrequentLoad[j] = note[musicList[*state].frequent[j]];
				musicSoundtLoad[j] = sound[musicList[*state].sound[j]];
			}
			musicFrequentLoad[musicList[*state].len] = 1;
			musicSoundtLoad[musicList[*state].len] = 0;
		}
		beepUpdata(musicFrequentLoad[i], musicSoundtLoad[i]);
		i++;
		if(i > musicList[*state].len){
			i = 0;
			*state = 0;
			driverBeepConifg.firstHere = 0;
			driverBeepConifg.remain = 0;
			vPortFree(musicFrequentLoad);
			vPortFree(musicSoundtLoad);
		}
	}
}

void beepInit(){
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
	driverBeepConifg.state = 1;
}
void beepUpdata(uint16_t frequent_val,uint16_t sound_val){
	BEEP_FREQUENCE(frequent_val);
	BEEP_SOUND(sound_val);
}
