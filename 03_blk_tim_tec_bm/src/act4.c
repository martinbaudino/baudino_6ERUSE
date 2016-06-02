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

/** \brief Actividad 4: Control de Leds
 **
 ** Control de Leds
 **
 ** Programe una aplicación que emplee los drivers led.c y teclas.c para
 ** seleccionar el led que parapdea de la EDU-CIAA según la consigna 3.1.c de
 ** la Guia de Ejercitación Práctica.
 **
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
 * yyyymmdd v0.0.1 initials initial version
 */

/*==================[inclusions]=============================================*/
#include "act4.h"       /* <= own header */


/*==================[macros and definitions]=================================*/
#define MAX_TM		2U

#define SFT_LEDS	0U
#define SFT_BOUNCE	1U

#define PER_BASE 	10U	// Período para base de tiempos (10ms)
#define PER_MIN		1U
#define PER_MAX 	200
#define PER_INC 	PER_MIN


#define PER_LEDS 	25U	// Período de conmutación de LEDs (25x10ms)
#define PER_BOUNCE 	7U	// Período para antirrebotes de teclas (7x10ms)

#define GATILLADO 1U
#define NO_GATILLADO 0U






/*==================[internal data declaration]==============================*/
/* Estructura para temporizadores por soft. 10ms de base */
struct sft_tmr {
	uint32_t reload;	// Período de disparo
	uint32_t cuenta;	// Valor de cuenta
	uint8_t	 disparo;	// Cuenta llegó al límite. Aplicación debe limpiarla
	void (*fp_procesar)(void);
}sft_tmrs[MAX_TM];


uint8_t curr_led = LED0_R;


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
	teclas_init();

	sft_tim_init();
	base_tiempo_init(PER_BASE);

	while(1)
	{
		sft_tim_procesar();
	}

}

void leds_ciclar(uint8_t comando)
{
	if(comando & TEC1)
	{
		if(curr_led == LED0_R)
		{
			curr_led = LED3;
		}
		else
		{
			curr_led--;
		}
	}
	else if(comando & TEC2)
	{
		if(comando & TEC1)
		{
			if(curr_led == LED3)
			{
				curr_led = LED0_R;
			}
			else
			{
				curr_led++;
		}
	}


	return curr_led;
}

void ISR_RIT_Handler(void)
{
	static uint8_t led=0;

	led_toggle(led);
	if(led == LED3)
	{
		led = LED0_R;
	}else{
		led++;
	}

	limp_rit_int();
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

