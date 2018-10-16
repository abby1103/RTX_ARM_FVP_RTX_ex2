#include "serial.h"
#include "socal/alt_uart.h"
#include "socal/hps.h"
#include "socal/socal.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "alt_16550_uart.h"
#include "altera_avalon_uart_regs.h"
#include "altera_avalon_uart_lwhal.h"
#include "hwlib.h"
#include "cmsis_os.h"
#include "uart_NMEA.h"

int len = 0;
char define_size[500];



/** @Function Description:  Writes a single character to the UART.
  * @API Type:              External
  * @param base             UART base address
  * @param character        Character to be written
  * @return                 None
  *
  */

void print_buffer(void *buf,const char *fmt, ...){  			/*transfer sentence to characters*/

		struct buffer *buf_ptr = (struct buffer *)buf;

		va_list  vl;
		va_start(vl, fmt);
		len = vsnprintf((buf_ptr->string)+(buf_ptr->index), sizeof(define_size), fmt, vl);  /*how many characters in a sentence include '\n' '\r'*/
		va_end(vl);
		buf_ptr->index+=len;

		return;
}


void altera_avalon_uart_lwhal_putstring(void *buf,void *base)
{
	struct buffer *buf_ptr = (struct buffer *)buf;

	for( int i = 0 ; i < buf_ptr->index ; i++ ){
		altera_avalon_uart_lwhal_putchar(base, (unsigned int)*(buf_ptr->string+i));
		*(buf_ptr->string+i) = 0;
	}
	buf_ptr->index=0; /* reset index */
}

