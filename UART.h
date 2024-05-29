/*************************************************************************
UART LIBRARY FOR ATMEGA328P (Universal Asynchronous Receiver and Transmitter)
Ingenieria Mecatronica - Microcontroladores - UNER FCAL
*************************************************************************/

#ifndef UART_H_
#define UART_H_

typedef struct __attribute__((packed, aligned(1))){
	uint8_t *buf;
	uint8_t sizeBuf;					/*!< DEBE ser de 2^n-1*/
	uint8_t iw;
	uint8_t ir;
	uint8_t timeOut;
	uint8_t header;
	uint8_t cks;
}_sRX;

typedef struct __attribute__((packed, aligned(1))){
	uint8_t *buf;
	uint8_t sizeBuf;					/*!< DEBE ser de 2^n-1*/
	uint8_t iw;
	uint8_t ir;
	uint8_t cks;
}_sTX;

#define SIZEBUFRX		128				/*!< DEBE ser de 2^n*/
#define SIZEBUFTX		128				/*!< DEBE ser de 2^n*/

void PutHeaderOnTx(_sTX *aTx, uint8_t id, uint8_t legth);
void PutByteOnTx(_sTX *aTx, uint8_t value);
void PutBufOnTx(_sTX *aTx, uint8_t *buf, uint8_t length);
void PutStrOnTx(_sTX *aTx, const char *str);

//SRAM
_sRX rx;
_sTX tx;

uint8_t bufRX[SIZEBUFRX], bufTX[SIZEBUFTX];
uint8_t aux8;
char strAux[32];

ISR(USART_RX_vect){
	rx.buf[rx.iw++] = UDR0;
	rx.iw &= rx.sizeBuf;
}

void IniUSART(){
	UCSR0A = UCSR0A;
	UCSR0A = (1 << U2X0);
	UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
	UBRR0 = 16;
}

void PutHeaderOnTx(_sTX *aTx, uint8_t id, uint8_t legth){
	
}

void PutByteOnTx(_sTX *aTx, uint8_t value){
	
}

void PutBufOnTx(_sTX *aTx, uint8_t *buf, uint8_t length){
	
}

void PutStrOnTx(_sTX *aTx, const char *str){
	uint8_t i = 0;
	
	while(str[i]){
		aTx->buf[aTx->iw++] = str[i++];
		aTx->iw &= aTx->sizeBuf;
	}
}


#endif /* UART_H_ */
