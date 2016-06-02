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

/** \brief Actividad 7: Diente de sierra / Teclado
 **
 ** Realice el ejercicio 4.1.b de la Guia de Ejercitación Práctica que se
 ** enuncia a continuación:
 **
 ** Diseñe e implemente un firmware sobre la EDU-CIAA que genera una señal
 ** tipo diente de sierra de periodo 100 ms y excursión de 0 a 3V de parámetros,
 ** amplitud y frecuencia variables.
 **
 ** Tec 1: Aumenta la amplitud de la señal.
 ** Tec 2: Disminuye la amplitud de la señal.
 ** Tec 3: Aumenta el periodo de la señal.
 ** Tec 4: Disminuye el periodo de la señal.
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
#include "act7.h"       /* <= own header */


/*==================[macros and definitions]=================================*/
#define MAX_TM		2U

#define SFT_LEDS	0U
#define SFT_BOUNCE	1U

#define PER_BASE 	1U	// Período para base de tiempos (10ms)
#define PER_MIN		1U
#define PER_MAX 	(200U * PER_MIN)
#define PER_INC 	(10 * PER_MIN)


#define PER_LEDS 	100U	// Período de conmutación de LEDs (100x1ms)
#define PER_BOUNCE 	70U	// Período para antirrebotes de teclas (7x1ms)

#define GATILLADO 	 1U
#define NO_GATILLADO 0U

#define AMP_MAX 	1023
#define AMP_INC 	(AMP_MAX / PER_LEDS)
#define AMP_START 	0U
#define AMP_MIN		1U





/*==================[internal data declaration]==============================*/
/* Estructura para temporizadores por soft. 10ms de base */
struct sft_tmr {
	uint32_t reload;	// Período de disparo
	uint32_t cuenta;	// Valor de cuenta
	uint8_t	 disparo;	// Cuenta llegó al límite. Aplicación debe limpiarla
	void (*fp_procesar)(void);
}sft_tmrs[MAX_TM];


uint8_t curr_led = LED0_R;

float amp_inc = AMP_INC;
uint32_t curr_amp = AMP_START;


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

	dac_init();

	sft_tim_init();
	base_tiempo_init(PER_BASE);

	while(1)
	{
		sft_tim_procesar();
	}

}

void opciones_procesar(uint8_t comando)
{
	static int32_t amp_fin = AMP_MAX;



	if(comando & TEC1)
	{
		led_off(curr_led);
		if(curr_led == LED3)
		{
			curr_led = LED0_R;
		}
		else
		{
			curr_led <<= 1;
		}

		if(amp_fin < AMP_MAX)
		{
			amp_fin += AMP_INC;
			amp_inc = amp_fin / sft_tmrs[SFT_LEDS].reload;
		}
		else
		{
			amp_fin = AMP_MAX;
			amp_inc = amp_fin / sft_tmrs[SFT_LEDS].reload;
		}
	}
	else if(comando & TEC2)
	{
		led_off(curr_led);
		if(curr_led == LED0_R)
		{
			curr_led = LED3;
		}
		else
		{
			curr_led >>= 1;
		}

		amp_fin -= AMP_INC;
		if(amp_fin > AMP_MIN)
		{
			amp_inc = amp_fin / sft_tmrs[SFT_LEDS].reload;
		}
		else
		{
			amp_fin = AMP_MIN;
			amp_inc = amp_fin / sft_tmrs[SFT_LEDS].reload;
		}
	}

	if(comando & TEC3)
	{
		if(sft_tmrs[SFT_LEDS].reload <= PER_MIN)
		{
			sft_tmrs[SFT_LEDS].reload = PER_MIN;
		}
		else
		{
			sft_tmrs[SFT_LEDS].reload -= PER_INC;
		}

		amp_inc = amp_fin / sft_tmrs[SFT_LEDS].reload;
	}
	else if(comando & TEC4)
	{
		if(sft_tmrs[SFT_LEDS].reload >= PER_MAX)
		{
			sft_tmrs[SFT_LEDS].reload = PER_MAX;
		}
		else
		{
			sft_tmrs[SFT_LEDS].reload += PER_INC;
		}

		amp_inc = amp_fin / sft_tmrs[SFT_LEDS].reload;
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
	dac_set(curr_amp += ((uint32_t)amp_inc));
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
		led_toggle(curr_led);
		curr_amp = 0;
		dac_set(curr_amp);
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

