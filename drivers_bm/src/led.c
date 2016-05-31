/* Copyright 2016, XXXXXXXXX  
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief Blinking Bare Metal driver led
 **
 **
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */

/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Baremetal Bare Metal LED Driver
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * yyyymmdd v0.0.1 initials initial version
 */

/*==================[inclusions]=============================================*/

#ifndef CPU
#error CPU shall be defined
#endif
#if (lpc4337 == CPU)
#include "chip.h"
#elif (mk60fx512vlq15 == CPU)
#else
#endif
#include "led.h"













/*==================[macros and definitions]=================================*/
#define PB2 	2
#define GPIO0	0
#define GPIO1	1
#define GPIO5	5
#define P2_0 	0
#define P2_1 	1
#define P2_2 	2
#define P2_10 	14
#define P2_11	11
#define P2_12	12
#define GPIO_INPUT	0
#define GPIO_OUTPUT 1

#define PIN0 	1<<0
#define PIN1 	1<<1
#define PIN2 	1<<2
#define PIN14 	1<<14
#define PIN11 	1<<11
#define PIN12 	1<<12



/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
/** \brief Main function
 *
 * This is the main entry point of the software.
 *
 * \returns 0
 *
 * \remarks This function never returns. Return value is only to avoid compiler
 *          warnings or errors.
 */

void led_on(uint8_t leds)
{
	switch(leds)
	{
	case LED0_R:
		Chip_GPIO_SetValue(LPC_GPIO_PORT, GPIO5, PIN0);
		break;

	case LED0_G:
		Chip_GPIO_SetValue(LPC_GPIO_PORT, GPIO5, PIN1);
		break;

	case LED0_B:
		Chip_GPIO_SetValue(LPC_GPIO_PORT, GPIO5, PIN2);
		break;

	case LED1:
		Chip_GPIO_SetValue(LPC_GPIO_PORT, GPIO0, PIN14);
		break;

	case LED2:
		Chip_GPIO_SetValue(LPC_GPIO_PORT, GPIO1, PIN11);
		//Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, GPIO1,11);
		break;

	case LED3:
		Chip_GPIO_SetValue(LPC_GPIO_PORT, GPIO1, PIN12);
		break;
	default:
		break;
	}



}

void led_off(uint8_t leds)
{
	switch(leds)
	{
	case LED0_R:
		Chip_GPIO_ClearValue(LPC_GPIO_PORT, GPIO5, PIN0);
		break;

	case LED0_G:
		Chip_GPIO_ClearValue(LPC_GPIO_PORT, GPIO5, PIN1);
		break;

	case LED0_B:
		Chip_GPIO_ClearValue(LPC_GPIO_PORT, GPIO5, PIN2);
		break;

	case LED1:
		Chip_GPIO_ClearValue(LPC_GPIO_PORT, GPIO0, PIN14);
		break;

	case LED2:
		Chip_GPIO_ClearValue(LPC_GPIO_PORT, GPIO1, PIN11);
		//Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, GPIO1,11);
		break;

	case LED3:
		Chip_GPIO_ClearValue(LPC_GPIO_PORT, GPIO1, PIN12);
		break;
	default:
		break;
	}

}

void led_toggle(uint8_t leds)
{
	switch(leds)
	{
	case LED0_R:
		Chip_GPIO_SetPortToggle(LPC_GPIO_PORT, GPIO5, PIN0);
		break;

	case LED0_G:
		Chip_GPIO_SetPortToggle(LPC_GPIO_PORT, GPIO5, PIN1);
		break;

	case LED0_B:
		Chip_GPIO_SetPortToggle(LPC_GPIO_PORT, GPIO5, PIN2);
		break;

	case LED1:
		Chip_GPIO_SetPortToggle(LPC_GPIO_PORT, GPIO0, PIN14);
		break;

	case LED2:
		Chip_GPIO_SetPortToggle(LPC_GPIO_PORT, GPIO1, PIN11);
		break;

	case LED3:
		Chip_GPIO_SetPortToggle(LPC_GPIO_PORT, GPIO1, PIN12);
		break;
	default:
		break;

	}

}

void leds_init(void)
{
	Chip_GPIO_Init(LPC_GPIO_PORT);

	Chip_SCU_PinMux(PB2, P2_0, MD_PUP, FUNC4); /* mapea P2_0 en GPIO5[0], LED0R y habilita el pull up*/
	Chip_SCU_PinMux(PB2, P2_1, MD_PUP, FUNC4); /* mapea P2_1 en GPIO5[1], LED0G y habilita el pull up*/
	Chip_SCU_PinMux(PB2, P2_2, MD_PUP, FUNC4); /* mapea P2_2 en GPIO5[2], LED0B y habilita el pull up*/

	Chip_SCU_PinMux(PB2, P2_10, MD_PUP, FUNC0); /* mapea P2_10 en GPIO0[14], LED1 y habilita el pull up*/
	Chip_SCU_PinMux(PB2, P2_11, MD_PUP, FUNC0); /* mapea P2_11 en GPIO1[11], LED2 y habilita el pull up*/
	Chip_SCU_PinMux(PB2, P2_12, MD_PUP, FUNC0); /* mapea P2_12 en GPIO1[12], LED3 y habilita el pull up*/

	Chip_GPIO_SetDir(LPC_GPIO_PORT, GPIO5, PIN0|PIN1|PIN2, GPIO_OUTPUT);
	Chip_GPIO_SetDir(LPC_GPIO_PORT, GPIO0, PIN14, GPIO_OUTPUT);
	Chip_GPIO_SetDir(LPC_GPIO_PORT, GPIO1, PIN11|PIN12, GPIO_OUTPUT);

	Chip_GPIO_ClearValue(LPC_GPIO_PORT, GPIO5, PIN0|PIN1|PIN2);
	Chip_GPIO_ClearValue(LPC_GPIO_PORT, GPIO0, PIN14);
	Chip_GPIO_ClearValue(LPC_GPIO_PORT, GPIO1, PIN11|PIN12);
}


/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

