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

/** \brief Actividad 6: Led Multicolor
 **
 ** Led Multicolor
 **
 ** Diseñe un driver, (timers.c y timers.h), que le permita a la aplicación
 ** acceder a los temporizadores del microcontrolador.
 **
 ** Programe una aplicación que emplee los drivers led.c, teclas.c y timers.c con
 ** el siguiente comportamiento:
 **  	- Al energizar la placa deben parpadear alternadamente los leds Rojo y Verde,
 **  	250ms encendido cada led.
 **  	- Al presionar la TECLA 1, se debe variar el color del led RGB en, al menos,
 **  	16 colores diferentes.
 **
 ** La temporización de los leds se debe realizar a través del RIT Timer. (ver
 ** Temporizadores e Interrupciones)
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
 * 20160601 v0.0.1 initials initial version
 */

/*==================[inclusions]=============================================*/
#include "act6.h"       /* <= own header */


/*==================[macros and definitions]=================================*/
#define MAX_TM		1U

#define PER_BASE 	1U	// Período para base de tiempos (1ms)
#define PER_MIN		5U
#define PER_MAX 	200
#define PER_INC 	PER_MIN

#define PER_SFT0 	10U	// Período de timer de soft (25x10ms)


#define GATILLADO 1U
#define NO_GATILLADO 0U

#define SFTIM0	0U

#define PALETA	 6U
#define COLORES  3U



/*==================[internal data declaration]==============================*/
/* Estructura para temporizadores por soft. 10ms de base */
struct sft_tmr {
	uint32_t reload;	// Período de disparo
	uint32_t cuenta;	// Valor de cuenta
	uint8_t	 disparo;	// Cuenta llegó al límite. Aplicación debe limpiarla
}sft_tmrs[MAX_TM];


uint8_t paleta_m [PALETA][COLORES] = {{10, 0, 0}, {0,10,0}, {0,0,10}, {10,0,10},{10,10,0}, {10,10,10}};


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
		teclas_procesar();
		leds_procesar();
	}

}

void leds_ciclar(uint8_t comando)
{
	if(comando & TEC1)
	{

	}
	else
	{


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

uint8_t teclas_procesar(void)
{
	static uint8_t prim_vez = 0;
	static uint8_t teclas_puls = 0;

	if(prim_vez == 0)
	{
		teclas_puls = teclas_leer();
		sft_tmrs[SFTIM0].cuenta = PER_SFT0;
		sft_tmrs[SFTIM0].disparo = NO_GATILLADO;
		prim_vez = 1;
	}
	else
	{
		if(sft_tmrs[SFTIM0].disparo == GATILLADO)
		{
			if(teclas_puls == teclas_leer())
			{
				leds_ciclar(teclas_puls);
			}
			sft_tmrs[SFTIM0].disparo = NO_GATILLADO;
			prim_vez = 0;
		}
	}
	return 0;
}

void leds_procesar(void)
{

	if (sft_tmrs[SFTIM0].disparo == GATILLADO )
	{
		sft_tmrs[SFTIM0].disparo = NO_GATILLADO;
		led_on(LED0_R|LED0_G|LED0_B);
	}
	else
	{
		if( (PER_SFT0 - sft_tmrs[SFTIM0].cuenta) > 5 )
		{
			led_off(LED0_R);
		}
		if( (PER_SFT0 - sft_tmrs[SFTIM0].cuenta) > 5 )
		{
			led_off(LED0_G);
		}
		if( (PER_SFT0 - sft_tmrs[SFTIM0].cuenta) > 5 )
		{
			led_off(LED0_B);
		}
	}
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

