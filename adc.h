/*
 * adc.h
 *
 *  Created on: Feb 24, 2017
 *      Author: vidursarin
 */

#ifndef SRC_ADC_H_
#define SRC_ADC_H_

#include "em_adc.h"
#include "em_cmu.h"
#include "em_dma.h"
#include "dmactrl.h"
#include "gpio.h"

#define CALC_PRESCALE_VAL 49
#define LOWER_TEMP_BOUND 15
#define UPPER_TEMP_BOUND 35
#define SET_TEMP_GRADIENT (-6.27)
#define ADC_SLEEP_MODE 1

/*DMA*/
#define ADC0_DMA_Channel 0
#define MAX_CONVERSION 750
#define DEF_HPROT_VAL 0
#define CALC_PRESCALE_VAL 49
#define CONFIG_ADC_CHNL acmpChannel6


void ADC0_Init(void);

void cb_ADC0_DMA(unsigned int channel, bool primary, void *user);

void ADC0_DMA_Setup(void);

void DMA_Initialize(void);

#endif /* SRC_ADC_H_ */
