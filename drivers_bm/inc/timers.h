/** \brief Encabezado de Drivers de temporización con RITTIMER
 **
 **
 **
 **/

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20160531 v0.0.1 initials initial version
 */

#ifndef TIMERS_BM
#define TIMERS_BM



#include "stdint.h"

/**
 * Función:
 * void base_tiempo_init(uint32_t tiempo_base): Inicialización de base de tiempo
 * con RIT. Arranca el contador y activa fuente de Interupción.
 *
 * Parámetros:
 * uint32_t tiempo_base: Período de disparo del Temporizador en ms
 *
 * Devuelve:
 * void: Nada
 */
void base_tiempo_init(uint32_t tiempo_base);


/* Función:
 * void limp_rit_int(uint32_t tiempo_base): Limpieza de bandera de interrupción
 * del RIT, con actualización del período de disparo.
 *
 * Parámetros:
 * uint32_t tiempo_base: Período de disparo del Temporizador en ms
 *
 * Devuelve:
 * void: Nada
 */
void limp_rit_int(uint32_t retardo);


#endif


