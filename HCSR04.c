/************************************************************************/
/*					HCSR04 Library (.c) - ATMEGA328P
					Ingenieria en Mecatronica - UNER					*/
/************************************************************************/

#include "HCSR04.h"
#include <stdlib.h>

typedef struct sHandleHCSR04
{
	void (*WritePin)(uint8_t value);
	uint16_t usTimeRise;
	uint16_t usTimeFall;
	union
	{
		struct {
			uint8_t TRIGGERON: 1;
			uint8_t WAITINGRISEEDGE: 1;
			uint8_t WAITINGFALLEDGE: 1;
			uint8_t EDGEREADY: 1;
			uint8_t spare: 4;
		}bit;
		uint8_t byte;	
	}flags;
	uint16_t lastDistance;
	uint32_t ticks;
	void (*OnReadyMeasure)(uint16_t distance);
	_eHCSR04State state;
	struct sHandleHCSR04 *nextHCSR04;
} _sHCSR04Handle;

static _sHCSR04Handle *myHandleList = NULL;
static _sHCSR04Handle *myHandleCurrent = NULL;
static _sHCSR04Handle *myHandleAux = NULL;

unsigned int HCSR04_AddNew(void (*WritePin)(uint8_t value), uint32_t ticks)
{
	myHandleAux = (_sHCSR04Handle *)malloc(sizeof(_sHCSR04Handle));
	
	if (myHandleAux != NULL)
	{
		myHandleAux->WritePin = WritePin;
		myHandleAux->ticks = ticks;
		myHandleAux->flags.byte = 0;
		myHandleAux->lastDistance = 0;
		myHandleAux->usTimeRise = 0;
		myHandleAux->usTimeFall = 0;
		myHandleAux->OnReadyMeasure = NULL;
		myHandleAux->state = HCSR04_STATE_READY;
		myHandleAux->nextHCSR04 = myHandleList;	
		if (myHandleList == NULL)
			myHandleCurrent = myHandleAux;
		myHandleAux->WritePin(0);
		myHandleList = myHandleAux;	
	}
	return (unsigned int)myHandleAux;
}

uint16_t HCSR04_Read(unsigned int handleHCSR04)
{
	myHandleAux = (_sHCSR04Handle *)handleHCSR04;
	if (myHandleAux != NULL)
		return myHandleAux->lastDistance; 
	return 0;
}

_eHCSR04State HCSR04_State(unsigned int handleHCSR04)
{
	myHandleAux = (_sHCSR04Handle *)handleHCSR04;
	if (myHandleAux != NULL)
		return HCSR04_STATE_NO_INIT;
	return myHandleAux->state;		
}

_eHCSR04State HCSR04_StartBlocking(unsigned int handleHCSR04)
{
	myHandleAux = (_sHCSR04Handle *)handleHCSR04;
	if (myHandleAux != NULL)
	{
		uint8_t nticks10us = myHandleAux->ticks*3;
		myHandleAux->WritePin(1);
		while (nticks10us)
			nticks10us--;
		myHandleAux->WritePin(0);
		myHandleAux->flags.byte = 0;
		myHandleAux->state = HCSR04_STATE_BUSY;
		return HCSR04_STATE_READY;
	}
	return HCSR04_STATE_NO_INIT;	
}

_eHCSR04State HCSR04_Start(unsigned int handleHCSR04)
{
	myHandleAux = (_sHCSR04Handle *)handleHCSR04;
	if (myHandleAux != NULL)
	{
		myHandleAux->WritePin(1);
		myHandleAux->state = HCSR04_STATE_BUSY;
		return HCSR04_STATE_READY;
	}
	return HCSR04_STATE_NO_INIT;
}

void HCSR04_TriggerReady(unsigned int handleHCSR04)
{
	myHandleAux = (_sHCSR04Handle *)handleHCSR04;
	if (myHandleAux != NULL)
		myHandleAux->WritePin(0);
}

void HCSR04_RiseEdgeTime(unsigned int handleHCSR04, uint16_t usTimeRise)
{
	myHandleAux = (_sHCSR04Handle *)handleHCSR04;
	if (myHandleAux != NULL)
	{
		myHandleAux->usTimeRise = usTimeRise;
		myHandleAux->flags.byte = 0;
	}
}

void HCSR04_FallEdgeTime(unsigned int handleHCSR04, uint16_t usTimeFall)
{
	myHandleAux = (_sHCSR04Handle *)handleHCSR04;
	if (myHandleAux != NULL)
	{
		myHandleAux->usTimeFall = usTimeFall;
		myHandleAux->flags.bit.EDGEREADY = 1;
	}
}

void HCSR04_Task()
{
	if (myHandleCurrent != NULL)
	{
		if (myHandleCurrent->flags.bit.EDGEREADY)
		{
			myHandleCurrent->flags.byte = 0;
			myHandleCurrent->lastDistance = myHandleCurrent->usTimeFall - myHandleCurrent->usTimeRise;
			myHandleCurrent->lastDistance /= 58;
			if (myHandleCurrent->OnReadyMeasure != NULL)
				myHandleCurrent->OnReadyMeasure(myHandleAux->lastDistance);
			myHandleCurrent->state = HCSR04_STATE_READY;
		}
		myHandleCurrent = myHandleAux->nextHCSR04;
		if (myHandleCurrent == NULL)
			myHandleCurrent = myHandleList;
	}
}

void HCSR04_AttachOnReadyMeasure(unsigned int handleHCSR04, void (*OnReadyMeasure)(uint16_t distance))
{
	myHandleAux = (_sHCSR04Handle *)handleHCSR04;
	if (myHandleAux != NULL)
		myHandleAux->OnReadyMeasure = OnReadyMeasure;
}


