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
#include "tpfin.h"       /* <= own header */


/*==================[macros and definitions]=================================*/

/**
 * Cantidad de Timers por software implementados
 */
#define MAX_TM		5U

/**
 * Nombres descriptivos para indización en vector de Timers por Soft
 */
#define SFT_LEDS	0U	// Timer para parpadeo de LEDs
#define SFT_BOUNCE	1U	// Timer para antirrebote de pulsadores
#define SFT_SIGGEN  2U	// Timer para generador de señales
#define SFT_ADC  	3U	// Timer para lectura de ADC
#define SFT_UART	4U	// Timer para procesamiento de datos de UART

/**
 * Equivalencias para períodos de los Timers por Soft. Valores son múltiplos
 * de PER_BASE
 */
#define PER_BASE 	1U	// Período para base de tiempos en ms (1ms)
#define PER_MIN		1U
#define PER_MAX 	(200U * PER_MIN)
#define PER_INC 	(10 * PER_MIN)


#define PER_LEDS 	500U	// Período de conmutación de LEDs (500ms)
#define PER_BOUNCE 	 70U	// Período para antirrebotes de teclas (70ms)
#define PER_SIGGEN	 50U	// Período de señal de calibración (50ms) (f_gen = 10Hz)
#define PER_ADC		 10U	// Período muestreo (10ms) (f_sample = 100Hz)
#define PER_UART	1000U

/**
 * Definiciones de gatillado de timers por software
 */
#define GATILLADO 	 1U
#define NO_GATILLADO 0U

/**
 * Amplitudes para el DAC
 */
#define AMP_MAX 	1023U
#define AMP_INC 	50U
#define AMP_START 	0U
#define AMP_MIN		1U

#define GEN_EST_L	0U
#define GEN_EST_H	1U
#define GEN_AMP_L	570U
#define GEN_AMP_H	290U


/**
 * Definiciones para control del buffer del ADC
 */
#define ADC_BUF_SIZE	20U
#define ADC_BUF_CLEAR	0U
#define ADC_BUF_FULL	1U


/*==================[internal data declaration]==============================*/

/**
 * Estructura para temporizadores por soft. PER_BASE ms como base de tiempos
 */
struct sft_tmr {
	uint32_t reload;	// Período de disparo
	uint32_t cuenta;	// Valor de cuenta
	uint8_t	 disparo;	// Cuenta llegó al límite. Aplicación debe limpiarla
	void (*fp_procesar)(void);
}sft_tmrs[MAX_TM];


uint8_t curr_led = LED0_R;
uint8_t buffer = ADC_BUF_CLEAR;
uint16_t adc_samples[ADC_BUF_SIZE] = {0};
uint8_t transmit = 0;

float gain = 1;
float offset = 0;

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

	uint8_t run_buf = 0;

	uint8_t tramo =0;

	uint16_t u_v_max = 0;
	uint16_t u_v_min = 0;


	float v_max = 0;
	float v_min = 0;

	leds_init();
	teclas_init();

	sigGen_init();

	adc_init(ADC_CH1);


	InitUart(UART2, 115200);

	sft_tim_init();
	base_tiempo_init(PER_BASE);

	while(1)
	{
		if((transmit == 1) && (buffer == ADC_BUF_FULL))
		{

			if(tramo == 0)
			{
				u_v_max = 0;
				u_v_min = 1023;
				for(run_buf = 0; run_buf <= ADC_BUF_SIZE; run_buf++)
				{
					if(adc_samples[run_buf] < u_v_min)
					{
						u_v_min = adc_samples[run_buf];
					}
					if(adc_samples[run_buf] > u_v_max)
					{
						u_v_max = adc_samples[run_buf];
					}
				}

				v_max = (gain * (u_v_max * 3.3 / 1023)) + offset;
				v_min = (gain * (u_v_min * 3.3 / 1023)) + offset;


				WriteUartNBytes(UART2, "Gain: ", 6);
				SendUartFloatAscii(UART2, gain, 3);
				tramo++;
			}
			else if(tramo == 1)
			{
				WriteUartNBytes(UART2, "Offset: ", 8);
				SendUartFloatAscii(UART2, offset, 3);
				tramo++;
			}
			else if(tramo == 2)
			{
				WriteUartNBytes(UART2, "Vmax: ", 6);
				SendUartFloatAscii(UART2, v_max, 3);
				tramo++;
			}
			else if(tramo == 3)
			{
				WriteUartNBytes(UART2, "Vmin: ", 6);
				SendUartFloatAscii(UART2, v_min, 3);
				tramo = 0;;
			}

			buffer = ADC_BUF_CLEAR;
			transmit = 0;
		}

	}

}

void opciones_procesar(uint8_t comando)
{


	if(comando & TEC1)
	{
		if(gain >= 0.8)
		{
			gain -= 0.1;
		}
	}
	else if(comando & TEC2)
	{
		if(gain <= 1.2)
		{
			gain += 0.1;
		}
	}

	if(comando & TEC3)
	{
		if(offset >= -0.2)
		{
			offset -= 0.1;
		}
	}
	else if(comando & TEC4)
	{
		if(offset <= 0.2)
		{
			offset += 0.1;
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

	sft_tmrs[SFT_SIGGEN].reload = PER_SIGGEN;
	sft_tmrs[SFT_SIGGEN].cuenta = PER_SIGGEN;
	sft_tmrs[SFT_SIGGEN].disparo = NO_GATILLADO;
	sft_tmrs[SFT_SIGGEN].fp_procesar = &sigGen_procesar;

	sft_tmrs[SFT_ADC].reload = PER_SIGGEN;
	sft_tmrs[SFT_ADC].cuenta = PER_SIGGEN;
	sft_tmrs[SFT_ADC].disparo = NO_GATILLADO;
	sft_tmrs[SFT_ADC].fp_procesar = &adc_procesar;

	sft_tmrs[SFT_UART].reload = PER_UART;
	sft_tmrs[SFT_UART].cuenta = PER_UART;
	sft_tmrs[SFT_UART].disparo = NO_GATILLADO;
	sft_tmrs[SFT_UART].fp_procesar = &uart_procesar;


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
			sft_tmrs[timers].fp_procesar();
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
	led_toggle(LED0_R);
}

void sigGen_init(void)
{
	dac_init();

}

void sigGen_procesar(void)
{
	static uint8_t estado = GEN_EST_L;

	if(estado == GEN_EST_L)
	{
		dac_set(GEN_AMP_H);
		estado = GEN_EST_H;
	}
	else
	{
		dac_set(GEN_AMP_L);
		estado = GEN_EST_L;
	}

}


void adc_procesar(void)
{
	static uint8_t curr_sample = ADC_BUF_CLEAR;

	if( buffer != ADC_BUF_FULL)
	{
		adc_samples[curr_sample++] = adc_pool(ADC_CH1);

		if(curr_sample == ADC_BUF_SIZE)
		{
			curr_sample = 0;
			buffer = ADC_BUF_FULL;
		}
		led_toggle(LED1);
	}
	else
	{
		curr_sample = 0;
	}
}

void uart_procesar(void)
{
	transmit = 1;
}


/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

