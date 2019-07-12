#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h>

int PutChar(char c, FILE *stream);
int GetChar(FILE *stream);
void init_uart(void);

