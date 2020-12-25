#ifndef _UART_H
#define _UART_H

#include "default.h"

#include <stdio.h>
#include <avr/io.h>

void uart_init(void);
int uart_write(char c, FILE *fp);

#endif
