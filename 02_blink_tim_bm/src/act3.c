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

/** \brief Actividad 3: Psicodelia
 **
 ** Psicodelia: Aplicación
 **
 ** Diseñe una aplicación que haga parpadear de manera secuencial todos los
 ** leds de la placa (incluyendo el RGB) con un periodo de 100 ms cada uno.
 ** Se deberá emplear la interrupción del RIT Timer.
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
 * 20160531 v0.0.1 initials initial version
 */

/*==================[inclusions]=============================================*/
#include "act3.h"       /* <= own header */


/*==================[macros and definitions]=================================*/

#define PER_BASE 10U	// Período para base de tiempos (10ms)
#define PER_SFT0 10U	// Período de timer de soft (10x10ms)
#define MAX_TM	1U
#define GATILLADO 1U
#define NO_GATILLADO 0U
#define SFTIM0	0U

/*==================[internal data declaration]==============================*/

/* Estructura para temporizadores por soft. 10ms de base */
struct sft_tmr {
	uint32_t reload;	// Período de disparo
	uint32_t cuenta;	// Valor de cuenta
	uint8_t	 disparo;	// Cuenta llegó al límite. Aplicación debe limpiarla
}sft_tmrs[MAX_TM];

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


	leds_init();
	sft_tim_init();
	base_tiempo_init(PER_BASE);



	while(1)
	{
		leds_procesar();
	}

}

void sft_tim_init(void)
{
	sft_tmrs[SFTIM0].reload = PER_SFT0;
	sft_tmrs[SFTIM0].cuenta = PER_SFT0;
	sft_tmrs[SFTIM0].disparo = NO_GATILLADO;
}

void ISR_RIT_Handler(void)
{
	uint8_t timers;

	for(timers = 0; timers < MAX_TM; timers++)
	{
		if(sft_tmrs[timers].cuenta != 0)
		{
			sft_tmrs[timers].cuenta--;
		}
		else
		{
			sft_tmrs[timers].cuenta = sft_tmrs[timers].reload;
			sft_tmrs[timers].disparo = GATILLADO;	// Aplicación debe limpiar los timers gatillados
		}
	}

	limp_rit_int(PER_BASE);
}

void leds_procesar(void)
{
	static uint8_t led=1;

	if (sft_tmrs[0].disparo)
	{
		sft_tmrs[0].disparo = NO_GATILLADO;
		led_toggle(led);
		if(led == LED3)
		{
			led = LED0_R;
		}else{
			led <<= 1;
		}
		led_toggle(led);
	}
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

