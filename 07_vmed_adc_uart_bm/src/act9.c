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

/** \brief Actividad 9: Transmosión por UART
 **
 ** Realice el ejercicio 6.e de la Guia de Ejercitación Práctica que se
 ** enuncia a continuación:
 **
 ** Diseñe e implemente un firmware sobre la EDU-CIAA que permita adquirir una
 ** señal analógica de excursión entre 0 y 3.3V, presente en el CH1. El sistema
 ** debe enviar por el puerto serie una cadena de caracteres con el valor en
 ** decimal del dato convertido.
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
 * 20160602 v0.0.1 initials initial version
 */

/*==================[inclusions]=============================================*/
#include "act9.h"       /* <= own header */


/*==================[macros and definitions]=================================*/
#define MAX_TM		2U

#define SFT_LEDS	0U
#define SFT_BOUNCE	1U

#define PER_BASE 	10U	// Período para base de tiempos (10ms)
#define PER_MIN		1U
#define PER_MAX 	(200U * PER_MIN)
#define PER_INC 	(10 * PER_MIN)


#define PER_LEDS 	1U	// Período de conmutación de LEDs (1x10ms)
#define PER_BOUNCE 	70U	// Período para antirrebotes de teclas (7x1ms)

#define GATILLADO 	 1U
#define NO_GATILLADO 0U

#define AMP_MAX 	1023
#define AMP_INC 	50
#define AMP_START 	0U
#define AMP_MIN		1U

#define AMP_ROJA	1020
#define AMP_VERDE	100





/*==================[internal data declaration]==============================*/
/* Estructura para temporizadores por soft. 10ms de base */
struct sft_tmr {
	uint32_t reload;	// Período de disparo
	uint32_t cuenta;	// Valor de cuenta
	uint8_t	 disparo;	// Cuenta llegó al límite. Aplicación debe limpiarla
	void (*fp_procesar)(void);
}sft_tmrs[MAX_TM];


uint8_t curr_led = LED0_R;

uint16_t resultado = 0;
uint16_t lim_max = AMP_ROJA;
uint16_t lim_min = AMP_VERDE;



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

	adc_init(ADC_CH1);
	dac_init();

	InitUart(UART2, 115200);

	sft_tim_init();
	base_tiempo_init(PER_BASE);

	while(1)
	{
		sft_tim_procesar();
		resultado = adc_pool(ADC_CH1);

	}

}

void opciones_procesar(uint8_t comando)
{


	if(comando & TEC1)
	{
		if(lim_max > AMP_VERDE)
		{
			lim_max -= AMP_INC;
		}
		else
		{
			lim_max = AMP_VERDE;
		}
	}
	else if(comando & TEC2)
	{
		if(lim_max < AMP_ROJA)
		{
			lim_max += AMP_INC;
		}
		else
		{
			lim_max = AMP_ROJA;
		}

	}

	if(comando & TEC3)
	{
		if(lim_min > AMP_VERDE)
		{
			lim_min -= AMP_INC;
		}
		else
		{
			lim_min = AMP_VERDE;
		}

	}
	else if(comando & TEC4)
	{
		if(lim_min < AMP_ROJA)
		{
			lim_min += AMP_INC;
		}
		else
		{
			lim_min = AMP_ROJA;
		}

	}
}

void sft_tim_init(void)
{
	sft_tmrs[SFT_LEDS].reload = PER_LEDS;
	sft_tmrs[SFT_LEDS].cuenta = PER_LEDS;
	sft_tmrs[SFT_LEDS].disparo = NO_GATILLADO;
	sft_tmrs[SFT_LEDS].fp_procesar = &leds_procesar;

	sft_tmrs[SFT_BOUNCE].reload = PER_BOUNCE;
	sft_tmrs[SFT_BOUNCE].cuenta = PER_BOUNCE;
	sft_tmrs[SFT_BOUNCE].disparo = NO_GATILLADO;
	sft_tmrs[SFT_BOUNCE].fp_procesar = &teclas_procesar;
}

void sft_tim_procesar(void)
{
	uint8_t timers;

	for(timers = 0; timers < MAX_TM; timers++)
	{
		if (sft_tmrs[timers].disparo == GATILLADO )
		{
			sft_tmrs[timers].disparo = NO_GATILLADO;
			sft_tmrs[timers].fp_procesar();
		}
	}
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
	adc_convertir();

	limp_rit_int(PER_BASE);
}

void teclas_procesar(void)
{
	static uint8_t prim_vez = 0;
	static uint8_t teclas_puls = 0;

	if(prim_vez == 0)
	{
		teclas_puls = teclas_leer();
		sft_tmrs[SFT_BOUNCE].cuenta = PER_BOUNCE;
		prim_vez = 1;
	}
	else
	{
		if(teclas_puls == teclas_leer())
		{
			opciones_procesar(teclas_puls);
		}
		prim_vez = 0;
	}
}

void leds_procesar(void)
{
		dac_set(resultado);

		if(resultado > lim_max)
		{
			led_on(LED0_R);
		}
		else
		{
			led_off(LED0_R);
		}

		if(resultado < lim_min)
		{
			led_on(LED0_G);
		}
		else
		{
			led_off(LED0_G);
		}

		WriteUartByte(UART2, 'g');

}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

