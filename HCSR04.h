/************************************************************************/
/*					HCSR04 Library (.h) - ATMEGA328P
					Ingenieria en Mecatronica - UNER					*/
/************************************************************************/


#ifndef HCSR04_H_
#define HCSR04_H_

#include <stdint.h>

typedef enum
{
	HCSR04_STATE_READY,
	HCSR04_STATE_NO_INIT,
	HCSR04_STATE_BUSY,
}_eHCSR04State;

/*!
@brief HCSR04_AddNew Función de inicialización

Esta Función agrega un nuevo HCSR04

@param [in] WritePin: Puntero a una función que permite escribir un pin.
@param [in] ticks: Indica la cantidad de instrucciones por micro segundo que puede ejecutar el MCU.

@retVal La función devuelve un número que identifica al módulo HCSR04
*/
unsigned int HCSR04_AddNew(void (*WritePin)(uint8_t value), uint32_t ticks);
				 

/*!
@brief HCSR04_Read

@param [in] name:
@param [out] name:

@retVal description
*/
uint16_t HCSR04_Read(unsigned int handleHCSR04);

_eHCSR04State HCSR04_Start(unsigned int handleHCSR04);

_eHCSR04State HCSR04_StartBlocking(unsigned int handleHCSR04);

_eHCSR04State HCSR04_State(unsigned int handleHCSR04);

void HCSR04_TriggerReady(unsigned int handleHCSR04);

void HCSR04_RiseEdgeTime(unsigned int handleHCSR04, uint16_t usTimeRise);

void HCSR04_FallEdgeTime(unsigned int handleHCSR04, uint16_t usTimeFall);

void HCSR04_Task();

void HCSR04_AttachOnReadyMeasure(unsigned int handleHCSR04, void (*OnReadyMeasure)(uint16_t distance));



#endif /* HCSR04_H_ */