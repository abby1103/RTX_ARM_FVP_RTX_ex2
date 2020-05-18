/*
 * ddcp_attitude.c
 *
 *  Created on: 2020/4/28
 *      Author: me249
 */
#include "cmsis_os.h"

#include <stdlib.h>
#include <math.h>
#include "ddcp_attitude.h"



void attitude_thread(void const *argument)
{

	while (1) {
	        osSignalWait(0x0009, osWaitForever);
	}

}
