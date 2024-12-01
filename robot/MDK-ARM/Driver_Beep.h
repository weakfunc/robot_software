#ifndef __DRIVER_BEEP_H__
#define __DRIVER_BEEP_H__
#include "main.h" 
#include "tim.h"


#define BEEP_SOUND(VALUE)  (TIM12->CCR2 = (VALUE))
#define BEEP_FREQUENCE(VALUE)  (TIM12->PSC = (VALUE))

#define NODE_MAX_LENGHT 64
#define MUSIC_MAX_LENGHT 2

typedef struct{
	uint8_t frequent[NODE_MAX_LENGHT];
	uint8_t sound[NODE_MAX_LENGHT];
	uint8_t len;
}muiscList_t;

typedef struct{
	uint8_t state;
	uint8_t remain;
	uint8_t firstHere;
}driverBeepConifg_t;

enum musicList{
	NORMAL = 0x00,
	START,
};

enum note{
	BASS_DO = 0,
	BASS_RE,
	BASS_MI,
	BASS_FA,
	BASS_SOL,
	BASS_LA,
	BASS_SI,
	ALTO_DO,
	ALTO_RE,
	ALTO_MI,
	ALTO_FA,
	ALTO_SOL,
	ALTO_LA,
	ALTO_SI,
	HIGH_DO,
	HIGH_RE,
	HIGH_MI,
	HIGH_FA,
	HIGH_SOL,
	HIGH_LA,
	HIGH_SI,
};

enum sound{
	H_NOISE = 0,
	N_NOISE,
};

extern driverBeepConifg_t driverBeepConifg;

void musicUpdata(uint8_t *state);
void beepUpdata(uint16_t frequent_val,uint16_t sound_val);
void beepInit(void);
#endif
