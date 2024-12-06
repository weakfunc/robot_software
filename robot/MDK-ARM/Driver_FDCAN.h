#ifndef __DRIVER_FDCAN_H__
#define __DRIVER_FDCAN_H__

#include "stm32h7xx.h"


extern void BSP_FDCAN_Init(void);

typedef struct {
		FDCAN_HandleTypeDef *hcan;
    FDCAN_TxHeaderTypeDef Header;
    uint8_t				Data[8];
}FDCAN_TxFrame_TypeDef;

typedef struct {
		FDCAN_HandleTypeDef *hcan;
    FDCAN_RxHeaderTypeDef Header;
    uint8_t 			Data[8];
}FDCAN_RxFrame_TypeDef;

extern FDCAN_TxFrame_TypeDef FDCAN1TxFrame;
	   
#endif



