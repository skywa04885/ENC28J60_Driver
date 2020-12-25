#include "uart.h"

static FILE uart = FDEV_SETUP_STREAM(uart_write, NULL, _FDEV_SETUP_WRITE);

void uart_init(void)
{
    UBRR0L = 16;
    UBRR0H = 0;
    
    UCSR0A = _BV(U2X0);
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
    UCSR0B = _BV(TXEN0);

    stdout = stderr = &uart;
}

int uart_write(char c, FILE *fp)
{
    if (c == '\n') uart_write('\r', fp);

    while (!(UCSR0A & _BV(UDRE0)));
    UDR0 = c;
    return c;
}
