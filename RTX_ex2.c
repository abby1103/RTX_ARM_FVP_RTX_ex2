#include "cmsis_os.h"

#include <stdint.h>
#include "VE_A9_MP.h"
#include "gic.h"
#include "tracking.h"
#include "allocate.h"
#include "namuru.h"
#include "Serial.h"
#include "display.h"
#include "message.h"
#include "ephemeris.h"
#include "measure.h"
#include "position.h"
#include "ekf_position.h"
#include "uart_NMEA.h"

#include "socal/socal.h"
#define LWFPGASLAVES 0xFF200000
void serialInterface(unsigned int data);

osThreadId allocate_thread_id;
osThreadId display_thread_id;
osThreadId message_thread_id;
osThreadId measure_thread_id;
osThreadId ekf_position_thread_id;
osThreadId uart_thread_id;

osThreadDef(allocate_thread,  osPriorityNormal,      1, 0);
osThreadDef(display_thread,   osPriorityAboveNormal, 1, 0);
osThreadDef(message_thread,   osPriorityHigh,        1, 0);
osThreadDef(measure_thread,   osPriorityRealtime,    1, 0);
osThreadDef(ekf_position_thread,   osPriorityAboveNormal,    1, 0);
osThreadDef(uart_thread,      osPriorityNormal, 1, 0);

int main(void)
{
	/* front-end configure register */
    serialInterface(0xA2931A30);
    serialInterface(0x85502881);
    serialInterface(0xEADF1DC2);
    serialInterface(0x9EC00083);
    serialInterface(0x0C000804);
    serialInterface(0x80000705);
    serialInterface(0x80000006);
    serialInterface(0x10061B47);

	allocate_thread_id = osThreadCreate(osThread(allocate_thread), NULL);
	display_thread_id  = osThreadCreate(osThread(display_thread),  NULL);
	message_thread_id  = osThreadCreate(osThread(message_thread),  NULL);
	measure_thread_id  = osThreadCreate(osThread(measure_thread),  NULL);
	ekf_position_thread_id  = osThreadCreate(osThread(ekf_position_thread),  NULL);
	uart_thread_id	   = osThreadCreate(osThread(uart_thread)	 , NULL);

  	initialize_allocation();
	control_block->prog_tic       = 4999999;
	control_block->prog_accum_int = 24999; /* 0.1ms position update rate */
	GIC_EnableIRQ(tacking_IRQ);


	while (1) 
	{

	}
}


void serialInterface(unsigned int data)
{

	int sdata;
	int i;
	int sclk = 0;

	alt_write_word(LWFPGASLAVES + 0x2000, 0);
	alt_write_word(LWFPGASLAVES + 0x2000, 4);
	alt_write_word(LWFPGASLAVES + 0x2000, 6);
	alt_write_word(LWFPGASLAVES + 0x2000, 4);
	alt_write_word(LWFPGASLAVES + 0x2000, 0);

	for (i = 0; i < 32; i++)
	{
		sdata = data >> 31;
		alt_write_word(LWFPGASLAVES + 0x2000, sdata | (sclk << 1));
		sclk  = 1;
		alt_write_word(LWFPGASLAVES + 0x2000, sdata | (sclk << 1));
		sclk  = 0;
		alt_write_word(LWFPGASLAVES + 0x2000, sdata | (sclk << 1));
		data <<= 1;
	}

	alt_write_word(LWFPGASLAVES + 0x2000, 0);
	alt_write_word(LWFPGASLAVES + 0x2000, 4);
	alt_write_word(LWFPGASLAVES + 0x2000, 6);
	alt_write_word(LWFPGASLAVES + 0x2000, 4);
	alt_write_word(LWFPGASLAVES + 0x2000, 0);
}
