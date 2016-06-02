/*
 * dac.h
 *
 *  Created on: 1 Jun 2016
 *      Author: osboxes
 */

#ifndef DAC_H_
#define DAC_H_

#include "stdint.h"

void dac_init(void);

void dac_set(uint32_t salida);

#endif /* DAC_H_ */
