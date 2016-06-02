/*
 * adc.c
 *
 *  Created on: 2 Jun 2016
 *      Author: osboxes
 */

#include "adc.h"


void adc_init(uint8_t canal)
{
	ADC_CLOCK_SETUP_T ADCSetup;

	Chip_SCU_ADC_Channel_Config(0, canal);

	Chip_ADC_Init(LPC_ADC0, &ADCSetup );

	Chip_ADC_EnableChannel(LPC_ADC0, canal, ENABLE);

	Chip_ADC_SetSampleRate(LPC_ADC0, &ADCSetup, ADC_MAX_SAMPLE_RATE);

}

void adc_convertir(void)
{
	Chip_ADC_SetStartMode(LPC_ADC0, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
}

uint16_t adc_pool(uint8_t canal)
{
	uint16_t data = 0;

	while(Chip_ADC_ReadStatus(LPC_ADC0, canal, ADC_DR_DONE_STAT) != SET){}

	Chip_ADC_ReadValue(LPC_ADC0, canal, &data);

	return data;
}

