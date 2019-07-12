#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h>
#include "uart.h"

FILE mystdout = FDEV_SETUP_STREAM(PutChar, NULL, _FDEV_SETUP_WRITE);
FILE mystdin =	FDEV_SETUP_STREAM(NULL, GetChar, _FDEV_SETUP_READ);

int PutChar(char c, FILE	*stream)
{
	//Poll UART status register until the "data register empty" bit is set.
	loop_until_bit_is_set(UCSR0A, UDRE0);
	//The data register is free for use, so place data onto stream
	UDR0 = c;
	return 0;
}

int GetChar(FILE *stream)
{
	//Poll UART status register until the "receive complete" bit is set.
	loop_until_bit_is_set(UCSR0A, RXC0);
	//Return the data received from the stream
	return UDR0;
}

void init_uart(void)
{		
	UCSR0B = (1<<RXEN0) | (1<<TXEN0);	//Enable the RX/TX on UART0
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);	//8-data-bits per char, 1 stop bit
	UBRR0 = 7;							//Baud rate, 115200 when CPU is 14.7456 MHz
	stdout = &mystdout;
	stdin = &mystdin;		
}