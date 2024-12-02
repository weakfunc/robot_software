/** 
 *******************************************************************************
 * @file      : bsp_dwt.h
 * @brief     : 
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRIVER_DWT_H__
#define __DRIVER_DWT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdint.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct
{
    uint32_t s;
    uint16_t ms;
    uint16_t us;
} DWT_Time_t;

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief ��ʼ��DWT,�������ΪCPUƵ��,��λMHz
 *
 * @param CPU_Freq_mHz c��Ϊ168MHz,A��Ϊ180MHz
 */
void DWT_Init(uint32_t CPU_Freq_mHz);

/**
 * @brief ��ȡ���ε���֮���ʱ����,��λΪ��/s
 *
 * @param cnt_last ��һ�ε��õ�ʱ���
 * @return float ʱ����,��λΪ��/s
 */
float DWT_GetDeltaT(uint32_t *cnt_last);

/**
 * @brief ��ȡ���ε���֮���ʱ����,��λΪ��/s,�߾���
 *
 * @param cnt_last ��һ�ε��õ�ʱ���
 * @return double ʱ����,��λΪ��/s
 */
double DWT_GetDeltaT64(uint32_t *cnt_last);

/**
 * @brief ��ȡ��ǰʱ��,��λΪ��/s,����ʼ�����ʱ��
 *
 * @return float ʱ����
 */
float DWT_GetTimeline_s(void);

/**
 * @brief ��ȡ��ǰʱ��,��λΪ����/ms,����ʼ�����ʱ��
 *
 * @return float
 */
float DWT_GetTimeline_ms(void);

/**
 * @brief ��ȡ��ǰʱ��,��λΪ΢��/us,����ʼ�����ʱ��
 *
 * @return uint64_t
 */
uint64_t DWT_GetTimeline_us(void);

/**
 * @brief DWT��ʱ����,��λΪ��/s
 * @attention �ú��������ж��Ƿ�����Ӱ��,�������ٽ����͹ر��ж�ʱʹ��
 * @note ��ֹ��__disable_irq()��__enable_irq()֮��ʹ��HAL_Delay()����,Ӧʹ�ñ�����
 *
 * @param Delay ��ʱʱ��,��λΪ��/s
 */
void DWT_Delay(float Delay);

/**
 * @brief DWT����ʱ���ắ��,�ᱻ����timeline��������
 * @attention �����ʱ�䲻����timeline����,����Ҫ�ֶ����øú�������ʱ����,����CYCCNT�����ʱ��ʱ���᲻׼ȷ
 */
void DWT_SysTimeUpdate(void);


#ifdef __cplusplus
}
#endif

#endif /* __FILE_H_ */
