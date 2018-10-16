#ifndef __UART_PUTSTRING_H
#define __UART_PUTSTRING_H

void print_buffer(void *buf,const char *fmt, ...);
extern void altera_avalon_uart_lwhal_putstring(void *buf,void *base);

void uart_thread(void const *argument);

#endif // __UART_PUTSTRING_H
