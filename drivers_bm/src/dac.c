/*
 * dac.c
 *
 *  Created on: 1 Jun 2016
 *      Author: osboxes
 */

#include "dac.h"
#include "chip.h"

void dac_init(void)
{
	Chip_SCU_DAC_Analog_Config();
	Chip_DAC_Init(LPC_DAC);

	Chip_DAC_ConfigDAConverterControl(LPC_DAC, DAC_DMA_ENA);

	Chip_DAC_UpdateValue(LPC_DAC, 0);
}


void dac_set(uint32_t salida)
{
	if(salida > 1023)
	{
		Chip_DAC_UpdateValue(LPC_DAC, 1023);
	}
	else
	{
		Chip_DAC_UpdateValue(LPC_DAC, salida);
	}

}
