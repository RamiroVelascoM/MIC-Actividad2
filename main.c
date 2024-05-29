/*
 * ActividadN2.c
 *
 * Created: 20 mar. 2024 16:41:57
 * Authors:	Leiva, Spiazzi & Velasco
 * Micro controller: Arduino UNO ATMEGA328P
 */ 

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/common.h>
#include <stdio.h>
#include <stdbool.h>
#include "HCSR04.h"

/************************************************************************/
/*               Type definitions (flags and unions)					*/
/************************************************************************/

//---------Flags and variables---------//

typedef union{
	struct{
		uint8_t bit0: 1;
		uint8_t bit1: 1;
		uint8_t bit2: 1;
		uint8_t bit3: 1;
		uint8_t bit4: 1;
		uint8_t bit5: 1;
		uint8_t bit6: 1;
		uint8_t bit7: 1;
	}bits;
	uint8_t byte;
}_uFlags;

//------------Box activities-----------//

typedef enum{
	BOX_SIZEA		= 0,
	BOX_SIZEB		= 1,
	BOX_SIZEC		= 2,
	DETECTING_BOX	= 3,
	DETECTING_SPACE = 4,
	BOX_DETECTED	= 5,
	SPACE_DETECTED	= 6,
	WAITING_TO_ACT  = 7,
	TIME_TO_ACT		= 8
}_eBoxType;

//---------------Tx & Rx---------------//

typedef struct __attribute__((packed, aligned(1))){
	uint8_t *buf;
	uint8_t sizeBuf;
	uint8_t iw;
	uint8_t ir;
	uint8_t timeOut;
	uint8_t header;
	uint8_t cks;
}_sRX;

typedef struct __attribute__((packed, aligned(1))){
	uint8_t *buf;
	uint8_t sizeBuf;
	uint8_t iw;
	uint8_t ir;
	uint8_t cks;
}_sTX;

typedef struct __attribute__((packed, aligned(1))){
	uint8_t		info;
	uint8_t		size;
	uint16_t	height;
	uint16_t	objective;
	uint16_t	time;
	char		letter[1];
	_eBoxType	stage;
}_sBOX;

/************************************************************************/
/*						    	Definitions								*/
/************************************************************************/
//------------Auxiliary------------//

#define allFlags			flags.byte

//-------------Tx & Rx-------------//

#define SIZEBUFRX			128
#define SIZEBUFTX			128

//---------------I/O---------------//

#define LEDSTATUS			PINB5
#define HCSR04_1_ECHO		PINB0
#define HCSR04_1_TRIG		PINB1

//---------Time Management---------//

#define IS10MS				0
#define MEASURETIME			10						// Every MEASURETIME*10 ms, there will be a new distance measured.

//----------Box Activities---------//

#define SIZEA_MIN			4
#define SIZEA_MAX			6
#define SIZEB_MIN			7
#define SIZEB_MAX			9
#define SIZEC_MIN			10
#define SIZEC_MAX			12
#define DEFAULT_DISTANCE	20

#define VELOCIDAD_CINTA		1
#define ON					1
#define OFF					0
#define TIME_SIZEA_MS		2000
#define TIME_SIZEB_MS		4000
#define TIME_SIZEC_MS		6000
#define TIME_ACTUATOR		500

#define INFO_SENT			flags.bits.bit0
#define DOUBLECHECK			flags.bits.bit1

/************************************************************************/
/*						  Variables and buffers							*/
/************************************************************************/
//------------Auxiliary------------//

char		strAux[64];
uint16_t	aux16				= 0;
_uFlags		flags;

//-------------Tx & Rx-------------//

_sRX		rx;
_sTX		tx;
uint8_t		bufRX[SIZEBUFRX], bufTX[SIZEBUFTX];

//---------Time Management---------//

uint8_t		countMeasureTime	= 6;
uint8_t		count100ms			= 10;
uint8_t		count1000ms			= 10;

//------------HeartBeat------------//

uint32_t	hbMask				= 0x80000000;
uint32_t	heartbeat			= 0xAF000000;

//----------Box Activities---------//

char		sizeIndicator;
_eBoxType	detectionStage		= SPACE_DETECTED;
uint16_t	boxHeight			= 0;
uint16_t	boxIndex			= 0;
uint16_t	MAX_BOXES			= 128; //(12/(0.08*VELOCIDAD_CINTA))
_sBOX		myBox[128];

//-------------HCSR04--------------//

uint32_t	handleHCSR04_1;

/************************************************************************/
/*							  Initialization							*/
/************************************************************************/

//---------Configurations----------//

void iniTimer();
void iniPorts();
void IniUSART();

//---------Time Management---------//

void every10ms();

//-----------HeartBeat-------------//

void heartbeatTask();

//------------Tx & Rx--------------//

void PutStrOnTx(_sTX *aTx, const char *str);
void sendDataUSART();

//-------------HSCR04--------------//

void WritePINHCSR04_1(uint8_t value);
void HCSR04_1_ReadyDistance(uint16_t distance);

//----------Box Activities---------//

void boxTypeTask();

void init_Boxes();
void boxID();
void boxTask();

/************************************************************************/
/*								Interruptions							*/
/************************************************************************/

ISR(USART_RX_vect)
{
	rx.buf[rx.iw++] = UDR0;							// Charges data from UDR0 to Rx buffer
	rx.iw &= rx.sizeBuf;							// Sends data from buffer
}

ISR(TIMER1_COMPA_vect)
{
	OCR1A += 20000;									// Adds 10 ms in the timer1 comparator
	GPIOR0 |= _BV(IS10MS);							// Flag that indicates that 10 ms have passed
}

ISR(TIMER1_COMPB_vect)
{
	HCSR04_TriggerReady(handleHCSR04_1);
	TIFR1 |= _BV(ICF1);								// Timer/Counter1 Output Compare A Match Flag enabled
	TCCR1B = (1 << ICNC1) | (1 << ICES1);			// Input Capture Noise Canceler and Input Capture Edge Select activated
	TCCR1B |= (1 << CS11);							// Prescaler definition (x8): CS12 = 0 and CS10 = 0
	TIMSK1 = _BV(ICIE1) | _BV(OCIE1A);				// Input Capture Interrupt and Output Compare A Match Interrupt enabled
}

ISR(TIMER1_CAPT_vect)
{
	if (TCCR1B & _BV(ICES1))						// Rising edge will trigger the capture
	{
		TCCR1B = (1 << ICNC1) | (1 << CS11);
		HCSR04_RiseEdgeTime(handleHCSR04_1, ICR1 >> 1);
	}
	else											// Falling edge is used as trigger
	{
		TIMSK1 &= ~_BV(ICIE1);
		HCSR04_FallEdgeTime(handleHCSR04_1, ICR1 >> 1);
	}
}

/************************************************************************/
/*								Functions								*/
/************************************************************************/

//-------------Configurations--------------//
void iniPorts()
{
	DDRB = (1 << HCSR04_1_TRIG) | (1 << LEDSTATUS); // Sets PB1 (Trigger) and PB5 (Led) as OUTPUT ports
}

void iniTimer()
{
	TCCR1A = 0;										// Operation mode of Timer1: NORMAL
	OCR1A = 20000;									// Setting limit of time counter: 10 milliseconds
	TIFR1 = TIFR1;									// Clear all bits for the timer1's interruption
	TIMSK1 = (1 << OCIE1A);							// Enables interruption of Comparator A (Timer1)
	TCCR1B = (1 << ICNC1) | (1 << CS11);			// Input Capture Noise Canceler activated, Prescaler definition (x8)
}

void IniUSART()
{
	UCSR0A = UCSR0A;								// All bits in UCSR0A are cleared
	UCSR0A = (1 << U2X0);							// Doubles the transfer rate for asynchronous communication
	UCSR0B = (1 << RXCIE0) | (1 << RXEN0);			// RX Complete Interrupt and Receiver enabled
	UCSR0B = (1 << TXEN0);							// Transmitter enabled
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);			// 8-bit character size for the Receiver and Transmitter buffers
	UBRR0 = 16;										// Sets baud rate of transmission: [UBRR0 = (16 MHz / 8*BR) - 1] (rounded)

	rx.buf = bufRX;									// Creates a local Rx buffer
	rx.sizeBuf = SIZEBUFRX-1;						// Size of Rx buffer has to be decremented by 1 (128 -> 0-127)
	rx.iw = 0;										// Writing index of Rx buffer is initialized in 0
	rx.ir = 0;										// Reading index of Rx buffer is initialized in 0
	rx.header = 0;									// Header of Rx buffer is initialized in 0
	
	tx.buf = bufTX;									// Creates a local tx buffer
	tx.sizeBuf = SIZEBUFTX-1;						// Size of Tx buffer has to be decremented by 1 (128 -> 0-127)
	tx.iw = 0;										// Writing index of Tx buffer is initialized in 0
	tx.ir = 0;										// Reading index of Tx buffer is initialized in 0
}

//------------Time Management-------------//
void every10ms()
{
	GPIOR0 &= _BV(LEDSTATUS);						// Activation of timer: 10 cycles * 10 ms = 100 ms
	countMeasureTime--;									// 50 milliseconds counter decrements
	count100ms--;									// 100 milliseconds counter decrements
	
	if (!countMeasureTime)
	{
		aux16 = TCNT1;								// Loads actual time in TCNT1 into aux16
		aux16 += 10;								// Adds 15 (us) to the previously saved time at TCNT1
		OCR1B = aux16;								// Then loads the value into OCR1B, generating an Output Compare Interrupt
		TIFR1 = _BV(OCF1B) | _BV(OCF1A);			// Flag set after the counter value in TCNT1 equals OCR1A and OCR1B
		TIMSK1 = _BV(OCIE1B) | _BV(OCIE1A);			// Timer/Counter1 Output Compare A and B Match interrupts are enabled.
		HCSR04_Start(handleHCSR04_1);				// Enables a new measure of the HCSR04
	}
	
	if (!count100ms)								// If 100 ms have passed
	{
		count100ms = 10;							// Restarts 100 ms counter
		heartbeatTask();							// Control the heartbeat sequence of the system
		boxTask();
		if (count1000ms)							// If 1000 ms haven't passed
			count1000ms--;							// Counter decrements
	}
}

//--------------HeartBeat----------------//
void heartbeatTask()
{
	if (heartbeat & hbMask)
		PORTB |= (1 << LEDSTATUS);					// Turn on LED
	else
		PORTB &= ~_BV(LEDSTATUS);					// Turn off LED
	
	hbMask >>= 1;									// Displace hbMask one place to the right
	if (!hbMask)
		hbMask = 0x80000000;						// If there's a 0 in that place, changes the actual positions to compare the right way
}

//----------------HCSR04-----------------//
void WritePINHCSR04_1(uint8_t value)
{
	if (value)
		PORTB |= _BV(HCSR04_1_TRIG);				// Sets a HIGH state (1) in the TRIGGER pin
	else
		PORTB &= ~_BV(HCSR04_1_TRIG);				// Sets a LOW state (0) in the TRIGGER pin
}

void HCSR04_1_ReadyDistance(uint16_t distance)
{
	/*
	if (DEFAULT_DISTANCE >= distance)
		boxHeight = DEFAULT_DISTANCE - distance;		// Saves last measurement in a variable
	else
		boxHeight = 0;
	if (!countMeasureTime)
	{
		countMeasureTime = MEASURETIME;					// Restarts 100 ms counter (measure cicle time)
		boxTypeTask();
	}
	*/
	if (DEFAULT_DISTANCE >= distance)
		myBox[boxIndex].height = DEFAULT_DISTANCE - distance;		// Saves last measurement in a variable
	else
		myBox[boxIndex].height = 0;
	if (!countMeasureTime)
	{
		countMeasureTime = MEASURETIME;					// Restarts 100 ms counter (measure cicle time)
		boxID(&myBox[boxIndex]);
	}
}

//-------------Communications-------------//
void PutStrOnTx(_sTX *aTx, const char *str)
{
	uint8_t i = 0;
	while (str[i])									// While there is information that has not been read yet
	{
		aTx->buf[aTx->iw++] = str[i++];				// Loads data into the Tx buffer
		aTx->iw &= aTx->sizeBuf;					// Updates index in the Tx buffer
	}
}

void sendDataUSART()
{
	if (tx.ir != tx.iw)								// If Tx's reading and writing indexes are different
	{
		if (UCSR0A & _BV(UDRE0))					// If Tx buffer is empty, and ready to receive new data
		{
			UDR0 = tx.buf[tx.ir++];					// Then new data is loaded into the Tx buffer
			tx.ir &= tx.sizeBuf;					// New size of the Tx buffer is updated
		}
	}
}

//-------------Box Activities-------------//
void init_Boxes(){
	for (uint16_t i=0; i<MAX_BOXES; i++)
	{
		myBox[i].height = 0;
		myBox[i].info = false;
		myBox[i].objective = 0;
		myBox[i].time = 0;
		myBox[i].stage = DETECTING_SPACE;
	}
}

/*
void boxTypeTask()
{
	if (boxHeight < SIZEA_MIN || boxHeight > SIZEC_MAX)		// If distance measured does not match any type of box
	{
		if (detectionStage == DETECTING_BOX)
		{
			detectionStage = SPACE_DETECTED;				// If there is a problem and identifies a BOX by mistake 
		}
		else if (detectionStage == BOX_DETECTED)
		{
			detectionStage = DETECTING_SPACE;				// First detection of a SPACE after detecting a BOX
		}
		else if (detectionStage == DETECTING_SPACE)
		{
			detectionStage = SPACE_DETECTED;				// Confirms the detection of a SPACE
			INFO_SENT = false;								// Sends info
		}
	}
	else													// If distance measured does match any type of box
	{
		if (detectionStage == DETECTING_SPACE)
		{
			detectionStage = BOX_DETECTED;					// If there is a problem and identifies a SPACE by mistake 
		}
		else if (detectionStage == SPACE_DETECTED)
		{
			detectionStage = DETECTING_BOX;					// First detection of a BOX after detecting a SPACE
		}
		else if (detectionStage == DETECTING_BOX)
		{
			detectionStage = BOX_DETECTED;					// Confirms the detection of a BOX
			INFO_SENT = false;								// Sends info
		}
	}
	
	switch (detectionStage)												// Depending on the last detection
	{
		case SPACE_DETECTED:											// If detected a SPACE
			if (INFO_SENT)												// Sends info once
				break;
			sprintf(strAux, ">> NO BOX \n");
			PutStrOnTx(&tx, strAux);
			INFO_SENT = true;
			break;
		case BOX_DETECTED:												// If detected a BOX
			if (INFO_SENT)												// Sends info once
				break;
			if (boxHeight >= SIZEA_MIN && boxHeight <= SIZEA_MAX)
			{
				sprintf(&sizeIndicator, "A");
			}
			else if (boxHeight >= SIZEB_MIN && boxHeight <= SIZEB_MAX)
			{
				sprintf(&sizeIndicator, "B");
			}
			else if (boxHeight >= SIZEC_MIN && boxHeight <= SIZEC_MAX)
			{
				sprintf(&sizeIndicator, "C");
			}
			boxIndex++;
			sprintf(strAux, "BOX:%.3d - TYPE:%c - SIZE:%.2dcm \n", boxIndex, sizeIndicator, boxHeight);
			PutStrOnTx(&tx, strAux);
			INFO_SENT = true;
			break;
		default:
			break;
	}
}
*/

void boxID(){
	if (myBox[boxIndex].height < SIZEA_MIN || myBox[boxIndex].height > SIZEC_MAX)		// If distance measured does not match any type of box
	{
		if (myBox[boxIndex].stage == DETECTING_BOX)
		{
			myBox[boxIndex].stage = SPACE_DETECTED;				// If there is a problem and identifies a BOX by mistake
		}
		else if (myBox[boxIndex].stage == BOX_DETECTED)
		{
			myBox[boxIndex].stage = DETECTING_SPACE;				// First detection of a SPACE after detecting a BOX
		}
		else if (myBox[boxIndex].stage == DETECTING_SPACE)
		{
			myBox[boxIndex].stage = SPACE_DETECTED;				// Confirms the detection of a SPACE
			myBox[boxIndex].info = false;								// Sends info
		}
	}
	else													// If distance measured does match any type of box
	{
		if (myBox[boxIndex].stage == DETECTING_SPACE)
		{
			myBox[boxIndex].stage = BOX_DETECTED;					// If there is a problem and identifies a SPACE by mistake
		}
		else if (myBox[boxIndex].stage == SPACE_DETECTED)
		{
			myBox[boxIndex].stage = DETECTING_BOX;					// First detection of a BOX after detecting a SPACE
		}
		else if (myBox[boxIndex].stage == DETECTING_BOX)
		{
			myBox[boxIndex].stage = BOX_DETECTED;					// Confirms the detection of a BOX
			myBox[boxIndex].info = false;								// Sends info
			boxIndex++;
			boxIndex &= (MAX_BOXES-1);
		}
	}
	
	switch (myBox[boxIndex-1].stage)												// Depending on the last detection
	{
		case SPACE_DETECTED:											// If detected a SPACE
			if (myBox[boxIndex-1].info)												// Sends info once
				break;
			sprintf(strAux, ">> NO BOX \n");
			PutStrOnTx(&tx, strAux);
			myBox[boxIndex-1].info = true;
			break;
		case BOX_DETECTED:												// If detected a BOX
			if (myBox[boxIndex-1].info)												// Sends info once
				break;
			if (myBox[boxIndex-1].height >= SIZEA_MIN && myBox[boxIndex-1].height <= SIZEA_MAX)
			{
				sprintf(&myBox[boxIndex-1].letter[0], "A");
				myBox[boxIndex].objective = TIME_SIZEA_MS;
			}
			else if (myBox[boxIndex-1].height >= SIZEB_MIN && myBox[boxIndex-1].height <= SIZEB_MAX)
			{
				sprintf(&myBox[boxIndex-1].letter[0], "B");
				myBox[boxIndex].objective = TIME_SIZEB_MS;
			}
			else if (myBox[boxIndex-1].height >= SIZEC_MIN && myBox[boxIndex-1].height <= SIZEC_MAX)
			{
				sprintf(&myBox[boxIndex-1].letter[0], "C");
				myBox[boxIndex-1].objective = TIME_SIZEC_MS;
			}
			myBox[boxIndex-1].stage = WAITING_TO_ACT;
			sprintf(strAux, "[IN] << BOX:%.3d - TYPE:%c - SIZE:%.2dcm \n", boxIndex, myBox[boxIndex-1].letter[0], myBox[boxIndex-1].height);
			PutStrOnTx(&tx, strAux);
			myBox[boxIndex-1].info = true;
			break;
		default:
			break;
	}
}

void boxTask(){
	for (uint16_t i=0; i<boxIndex; i++){
		if (myBox[i].stage == WAITING_TO_ACT){
			if (myBox[i].time == myBox[i].objective){
				myBox[i].time = 0;
				myBox[i].objective = 200;
				myBox[i].stage = TIME_TO_ACT;
			}
			else{
				myBox[i].time += 100;
			}
		}
		else if (myBox[i].stage == TIME_TO_ACT){
			if (myBox[i].time == myBox[i].objective){
				myBox[i].time = 0;
				myBox[i].objective = 0;
				myBox[i].stage = DETECTING_SPACE;
				sprintf(strAux, "[OUT] << BOX:%.3d - TYPE:%c - SIZE:%.2dcm \n", i+1, myBox[i].letter[0], myBox[i].height);
				PutStrOnTx(&tx, strAux);
				myBox[i].info = true;
			}
			else{
				myBox[i].time += 100;
			}
		}
	}
}

//-------------Main function--------------//
int main(void)
{
	cli();
	allFlags = false;								// Initializes flags
	iniPorts();										// Initializes ports
	iniTimer();										// Initializes timer1
	IniUSART();										// Initializes USART
	
	init_Boxes();									// Iniializes BOXES
	
	handleHCSR04_1 = HCSR04_AddNew(WritePINHCSR04_1, 10);					// Configures the sensor's handle, setting HIGH the trigger pin for 10 us
	HCSR04_AttachOnReadyMeasure(handleHCSR04_1, HCSR04_1_ReadyDistance);	// Configures the sensor' state, setting it ready to take a new measure
	
	PutStrOnTx(&tx, "READY\r\n");
	sei();
	
	while (1)
	{
		if (GPIOR0 & _BV(IS10MS))					// If 10 ms have passed
			every10ms();
		
		sendDataUSART();							// Does USART tasks
		HCSR04_Task();								// Does measurement tasks
	}
}