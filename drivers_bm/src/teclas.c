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
#include "teclas.h"

/*==================[macros and definitions]=================================*/
#define PB1 	1
#define GPIO0	0
#define GPIO1	1
#define GPIO5	5
#define P1_0 	0
#define P1_1 	1
#define P1_2 	2
#define P1_6	6
#define GPIO_INPUT	0
#define GPIO_OUTPUT 1

#define PIN4 	1<<4
#define PIN8 	1<<8
#define PIN9 	1<<9


/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
void teclas_init(void)
{
	Chip_GPIO_Init(LPC_GPIO_PORT);

	Chip_SCU_PinMux(PB1, P1_0, MD_PUP | MD_EZI | MD_ZI, FUNC0); /* mapea P1_0 en GPIO0[4] */
	Chip_SCU_PinMux(PB1, P1_1, MD_PUP | MD_EZI | MD_ZI, FUNC0); /* mapea P1_1 en GPIO0[8] */
	Chip_SCU_PinMux(PB1, P1_2, MD_PUP | MD_EZI | MD_ZI, FUNC0); /* mapea P1_2 en GPIO0[9] */

	Chip_SCU_PinMux(PB1, P1_6, MD_PUP | MD_EZI | MD_ZI, FUNC0); /* mapea P1_6 en GPIO1[9] */

	Chip_GPIO_SetDir(LPC_GPIO_PORT, GPIO0, PIN4|PIN8|PIN9, GPIO_INPUT);
	Chip_GPIO_SetDir(LPC_GPIO_PORT, GPIO1, PIN9, GPIO_INPUT);

}

uint8_t teclas_leer(void)
{
	uint32_t lectura;
	uint8_t tec_leidas = 0x0F;

	lectura = Chip_GPIO_ReadValue(LPC_GPIO_PORT, GPIO0);


	if(lectura & PIN4)
	{
		tec_leidas &= ~TEC1;
	}

	if(lectura & PIN8)
	{
		tec_leidas &= ~TEC2;
	}

	if(lectura & PIN9)
	{
		tec_leidas &= ~TEC3;
	}

	lectura = Chip_GPIO_ReadValue(LPC_GPIO_PORT, GPIO1);

	if(lectura & PIN9)
	{
		tec_leidas &= ~TEC4;
	}

	return tec_leidas;
}
/** \brief Main function
 *
 * This is the main entry point of the software.
 *
 * \returns 0
 *
 * \remarks This function never returns. Return value is only to avoid compiler
 *          warnings or errors.
 */




/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

