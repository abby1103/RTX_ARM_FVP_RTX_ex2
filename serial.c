 /**************************************************************************//**
 * @file     Serial.c
 * @brief    Simple polled UART driver. Needs to be completed 
 * @version
 * @date     07 February 2013
 *
 * @note
 *
 ******************************************************************************/
/* Copyright (c) 2011 - 2013 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/
   
#include "serial.h"

#include "socal/alt_uart.h"
#include "socal/hps.h"
#include "socal/socal.h"




/*----------------------------------------------------------------------------
  Write character to Serial Port
 *----------------------------------------------------------------------------*/
void SER_PutChar(char c)
{
	while(1 != ALT_UART_LSR_THRE_GET(alt_read_word(0xFFC02000 + 0x14)));	// Wait for UART TX to become free. Note that FIFOs are not being used here
	//while(1 != ALT_UART_LSR_THRE_GET(UART0->slr)); why??
	    UART0->rbr_thr_dll = c;
}

void SER_PutString(char s[])
{
	int i = 0;
	while (s[i] != 0x00)
	{
        /* Write character to THR */
		SER_PutChar(s[i]);
		i++;
	}
}

/*----------------------------------------------------------------------------
  Read character from Serial Port   (blocking read)
 *----------------------------------------------------------------------------*/
void SER_GetChar(char* c)
{
	*c = UART0->rbr_thr_dll;
}

/*----------------------------------------------------------------------------
  Serial Uart interrupt handler
 *----------------------------------------------------------------------------*/
void interrupt_SER(void)
{
	//your code here
}

