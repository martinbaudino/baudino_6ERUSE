/* Copyright 2016, 6ta Escuela RUSE
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

/** \brief Actividad 2: Blinking con driver
 **
 ** Blinking, con Driver propio led.c y led.h.
 **
 ** Empleando LPCOpen (consulte la Guía Rápida de Puertos de Entrada -Salida),
 ** diseñe un driver para el manejo de los leds de la EDU-CIAA.
 **
 ** Descomprima la carpeta drivers_bm dentro de la carpeta /projects. La misma
 ** corresponde a la capa de drivers de dispositivos de la HAL. Cada nuevo driver
 ** a desarrollar incorporará archivos nuevo_driver.h y nuevo_driver.c a las
 ** subcarpetas inc y src respectivamente.
 **
 ** Descomprima la carpeta 1_blinking_baremetal dentro de la carpeta /projects.
 ** La misma corresponde a la capa de aplicación de este ejemplo. Cada nueva
 ** aplicación a desarrollar incorporará una nueva carpeta similar a esta.
 **
 ** Declare funciones en led.h para encender, apagar y cambiar de estado -toggle-
 ** de cualquiera de los leds de la placa. Defínalas en led.c
 **
 ** Diseñe una aplicación similar a Blinking empleando LPCOpen, siguiendo la
 ** lógica de la figura.
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */

/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Baremetal Bare Metal example source file
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20160530 v0.0.1 initials initial version
 */

/*==================[inclusions]=============================================*/
#include "act2.h"       /* <= own header */


/*==================[macros and definitions]=================================*/

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


int main(void)
{
   /* perform the needed initialization here */

	uint32_t cuenta;	// Para lazo de retardo
	uint8_t led;		// Para ciclar leds

	leds_init();


	while(1)
	{
		/* Contador que cicla todos los estados posibles de los LEDs */
		for(led = LED0_R; led <= LED3; led++)
		{
			for(cuenta=10000000; cuenta!=0; cuenta--);
			led_toggle(led);
		}
	}

}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

