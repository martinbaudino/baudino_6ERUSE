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

#ifndef LED_H
#define LED_H
/** \brief Encabezado del driver de LEDs
 **
 ** Contiene definiciones de interfaces de LEDs y funciones de drivers.
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Baremetal Bare Metal example header file
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
#include "stdint.h"


/*==================[macros]=================================================*/
#define lpc4337            1
#define mk60fx512vlq15     2

/*
 * Definiciones de acceso público de la librería, para pasar a funciones
 */

/* Campos de bits de LEDs */
#define LED0_R	0x01
#define LED0_G	0x02
#define LED0_B	0x04
#define LED1	0x08
#define LED2	0x10
#define LED3	0x20

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/* Función:
 * void led_on(uint8_t leds): Función de encendido de LEDs
 *
 * Parámetros:
 * uint8_t leds: Campo de bits para banderas con nombres de LED como en placa EduCIAA.
 *
 * Devuelve:
 * void: Nada
 */
void led_on(uint8_t leds);


 /* Función:
  * void led_off(uint8_t leds): Función de apagado de LEDs
  *
  * Parámetros:
  * uint8_t leds: Campo de bits para banderas con nombres de LED como en placa EduCIAA.
  *
  * Devuelve:
  * void: Nada
  */
void led_off(uint8_t leds);


/* Función:
 * void led_toggle(uint8_t leds): Función de conmutación de LEDs
 *
 * Parámetros:
 * uint8_t leds: Campo de bits para banderas con nombres de LED como en placa EduCIAA.
 *
 * Devuelve:
 * void: Nada
 */
void led_toggle(uint8_t leds);


/* Función:
 * void leds_init(uint8_t leds): Función de configuración de LEDs.
 * Configura el Multiplexado de los pines, activa los pull-ups, como salidas
 * e inicializa los LEDs apagados.
 *
 * Parámetros:
 * uint8_t leds: Campo de bits para banderas con nombres de LED como en placa EduCIAA.
 *
 * Devuelve:
 * void: Nada
 */
void leds_init(void);



/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* LED_H */


