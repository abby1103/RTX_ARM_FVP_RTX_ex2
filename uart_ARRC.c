/*
 * uart_ARRC.c
 *
 *  Created on: 2019/3/25
 *      Author: me249
 */


#include "cmsis_os.h"

#include <stdio.h>
#include "uart_ARRC.h"
#include "altera_avalon_uart_lwhal.h"
#include "uart_putstring.h"
#include "position.h"
#include "constants.h"
#include "position.h"
#include "measure.h"
#include "time.h"
#include "tracking.h"
#include <math.h>
#include "constants.h"

struct buffer arrc1_buf;
struct buffer arrc2_buf;

void uart_thread(void const *argument)
{
    while(1){
    	osSignalWait(0x0006, osWaitForever);

		time_t          std_time;
		std_time = get_standard_time();

        //*******           LLH                 ***********//
        double          lat_DD, lon_DD;
        double          height;

        lat_DD = receiver_llh.lat * RADTODEG;
        lon_DD = receiver_llh.lon * RADTODEG;
        height = receiver_llh.hgt;

        //********      SV PRN IN LOCK          ***********//
        unsigned short ch;
		unsigned short SV[N_CHANNELS];
		unsigned short i = 0;
		for (ch = 0; ch < N_CHANNELS; ch+=2) {
				if( CH[ch].state == CHANNEL_LOCK) {
					SV[i] = CH[ch].prn + 1 ;
					i++;
				}
		}

		for (ch = i; ch < N_CHANNELS; ch++){
			SV[ch] = 0;
		}

		//********            buffer            ***********//
        print_buffer( &arrc1_buf,"T=%d:%d:%2.2f\n\rECEF = (X:%e Y:%e Z:%e)\n\rVelocity in ECEF = (X:%e Y:%e Z:%e)\n\rLLH  = (Lat:%2.5f Lon:%2.5f Hgt:%6.2f)\n\r",
					 std_time.hours,
					 std_time.minutes,
					 std_time.seconds,
					 receiver_pvt.x,
		             receiver_pvt.y,
		             receiver_pvt.z,
		             receiver_pvt_velocity.vx,
		             receiver_pvt_velocity.vy,
		             receiver_pvt_velocity.vz,
		             lat_DD,
		             lon_DD,
		             height);

        print_buffer( &arrc2_buf,"PNinLock:%d,%d,%d,%d,%d,%d,%d,%d\n\r",
        			 SV[0],SV[1],SV[2],SV[3],SV[4],SV[5],SV[6],SV[7]);


        //**********            uart                ***********//

        /*print GPGGA*/
       // altera_avalon_uart_lwhal_putstring(&gga_buf,(void*)(0xFF200400));	/*print*/
        /*print GPGSA*/
       // altera_avalon_uart_lwhal_putstring(&gsa_buf,(void*)(0xFF200400));	/*print*/
        /*print GPRMC*/
      //  altera_avalon_uart_lwhal_putstring(&rmc_buf,(void*)(0xFF200400));	/*print*/
        /*print GPVTG*/
      //  altera_avalon_uart_lwhal_putstring(&vtg_buf,(void*)(0xFF200400));	/*print*/
        /*print GPVTG*/
		altera_avalon_uart_lwhal_putstring(&arrc1_buf,(void*)(0xFF200400));	/*print*/
		/*print GPVTG*/
		altera_avalon_uart_lwhal_putstring(&arrc2_buf,(void*)(0xFF200400));	/*print*/


    }
}
