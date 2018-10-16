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

#include "socal/socal.h"
#include "uart0_support.h"
#define LWFPGASLAVES 0xFF200000

osThreadId allocate_thread_id;
osThreadId display_thread_id;
osThreadId message_thread_id;
osThreadId measure_thread_id;

osThreadDef(allocate_thread,  osPriorityNormal,      1, 0);
osThreadDef(display_thread,   osPriorityAboveNormal, 1, 0);
osThreadDef(message_thread,   osPriorityHigh,        1, 0);
osThreadDef(measure_thread,   osPriorityRealtime,    1, 0);


int main(void)
{
	uart0_init();
	
	allocate_thread_id = osThreadCreate(osThread(allocate_thread), NULL);
	display_thread_id  = osThreadCreate(osThread(display_thread),  NULL);
	message_thread_id  = osThreadCreate(osThread(message_thread),  NULL);
	measure_thread_id  = osThreadCreate(osThread(measure_thread),  NULL);

  	initialize_allocation();
	control_block->prog_tic       = 4999999;
	control_block->prog_accum_int = 24999;
	GIC_EnableIRQ(tacking_IRQ);

	while (1) 
	{

	}
}
