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

/** \brief Código fuente del driver de LEDs
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
 * 20160530 v0.0.1 initials initial version
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

/*
 * Definiciones privadas del módulo.
 * Con nombres referidos al hardware como en edu-ciaa-nxp-pinout-a4-v3r2-es.pdf
 */

/* Bloques de Puertos */
#define PB2 	2

/* Puertos/Función de Pin */
#define GPIO0	0
#define GPIO1	1
#define GPIO5	5

/* Nombres de Pines en uC*/
#define P2_0 	0
#define P2_1 	1
#define P2_2 	2
#define P2_10 	14
#define P2_11	11
#define P2_12	12

/* Números de Pines en Puertos */
#define PIN0 	1<<0
#define PIN1 	1<<1
#define PIN2 	1<<2
#define PIN14 	1<<14
#define PIN11 	1<<11
#define PIN12 	1<<12

/* Direcciones de Datos */
#define GPIO_INPUT	0
#define GPIO_OUTPUT 1


/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

/**
 * Función:
 * void led_on(uint8_t leds): Función de encendido de LEDs
 *
 * Parámetros:
 * uint8_t leds: Campo de bits para banderas con nombres de LED como en placa EduCIAA.
 *
 * Devuelve:
 * void: Nada
 */
void led_on(uint8_t leds)
{
	if(leds & LED0_R)
	{
		Chip_GPIO_SetValue(LPC_GPIO_PORT, GPIO5, PIN0);
	}
	if(leds & LED0_G)
	{
		Chip_GPIO_SetValue(LPC_GPIO_PORT, GPIO5, PIN1);
	}

	if(leds & LED0_B)
	{
		Chip_GPIO_SetValue(LPC_GPIO_PORT, GPIO5, PIN2);
	}

	if(leds & LED1)
	{
		Chip_GPIO_SetValue(LPC_GPIO_PORT, GPIO0, PIN14);
	}

	if(leds & LED2)
	{
		Chip_GPIO_SetValue(LPC_GPIO_PORT, GPIO1, PIN11);
	}

	if(leds & LED3)
	{
		Chip_GPIO_SetValue(LPC_GPIO_PORT, GPIO1, PIN12);
	}
}

 /**
  * Función:
  * void led_off(uint8_t leds): Función de apagado de LEDs
  *
  * Parámetros:
  * uint8_t leds: Campo de bits para banderas con nombres de LED como en placa EduCIAA.
  *
  * Devuelve:
  * void: Nada
  */
void led_off(uint8_t leds)
{
	if(leds & LED0_R)
	{
		Chip_GPIO_ClearValue(LPC_GPIO_PORT, GPIO5, PIN0);
	}
	if(leds & LED0_G)
	{
		Chip_GPIO_ClearValue(LPC_GPIO_PORT, GPIO5, PIN1);
	}

	if(leds & LED0_B)
	{
		Chip_GPIO_ClearValue(LPC_GPIO_PORT, GPIO5, PIN2);
	}

	if(leds & LED1)
	{
		Chip_GPIO_ClearValue(LPC_GPIO_PORT, GPIO0, PIN14);
	}

	if(leds & LED2)
	{
		Chip_GPIO_ClearValue(LPC_GPIO_PORT, GPIO1, PIN11);
	}

	if(leds & LED3)
	{
		Chip_GPIO_ClearValue(LPC_GPIO_PORT, GPIO1, PIN12);
	}
}

/**
 * Función:
 * void led_toggle(uint8_t leds): Función de conmutación de LEDs
 *
 * Parámetros:
 * uint8_t leds: Campo de bits para banderas con nombres de LED como en placa EduCIAA.
 *
 * Devuelve:
 * void: Nada
 */
void led_toggle(uint8_t leds)
{
	uint8_t check;

	check = (leds & LED0_R);
	if(check)
	{
		Chip_GPIO_SetPortToggle(LPC_GPIO_PORT, GPIO5, PIN0);
	}
	if(leds & LED0_G)
	{
		Chip_GPIO_SetPortToggle(LPC_GPIO_PORT, GPIO5, PIN1);
	}

	if(leds & LED0_B)
	{
		Chip_GPIO_SetPortToggle(LPC_GPIO_PORT, GPIO5, PIN2);
	}

	if(leds & LED1)
	{
		Chip_GPIO_SetPortToggle(LPC_GPIO_PORT, GPIO0, PIN14);
	}

	if(leds & LED2)
	{
		Chip_GPIO_SetPortToggle(LPC_GPIO_PORT, GPIO1, PIN11);
	}

	if(leds & LED3)
	{
		Chip_GPIO_SetPortToggle(LPC_GPIO_PORT, GPIO1, PIN12);
	}
}

/**
 * Función:
 * void leds_init(uint8_t leds): Función de configuración de LEDs.
 * Configura el Multiplexado de los pines como salidas, activa los pull-ups,
 * e inicializa los LEDs apagados.
 *
 * Parámetros:
 * uint8_t leds: Campo de bits para banderas con nombres de LED como en placa EduCIAA.
 *
 * Devuelve:
 * void: Nada
 */
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

