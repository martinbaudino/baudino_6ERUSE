/*
 * adc.h
 *
 *  Created on: 2 Jun 2016
 *      Author: osboxes
 */

#ifndef ADC_H_
#define ADC_H_

#include "stdint.h"
#include "chip.h"

void adc_init(uint8_t canal);

uint16_t adc_pool(uint8_t canal);

void adc_convertir(void);

#endif /* ADC_H_ */
