/*
 * ddcp_attitude.c
 *
 *  Created on: 2020/4/28
 *      Author: me249
 */
#include "cmsis_os.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "ddcp_attitude.h"
#include "position.h"
#include "measure.h"
#include "ambiguity_resolution.h"
#include "attitude_sol.h"
#include "threads.h"
#include "serial.h"
#include "namuru.h"
#include "tracking.h"

void attitude_thread(void const *argument)
{
	double sdcp[14];
	double sdstd = 0.05;	// 0.5 cm
	double small_an[3] = { 0.2618 , 0.2618 , 0.2618 }, old_an[3];
	double angle[3];
	char string[120];

	ECEF_pos P_ant0;
	llh_pos ant0_llh;
	ECEF_pos P_sat[7];
	double pseudo_range[7];
	int i;
	int ddcp_debug = 0;

	unsigned int   accum_count;
	double start_time, end_time, delta_t;
	int breakpoint;

	while (1) {
	        osSignalWait(0x0007, osWaitForever);

	        accum_count = 24999 - status_block->accum_count;
	        start_time = ((double)accum_int_count + (double)(accum_count)/24999) * 0.5; // unit : ms
	        // ddcp_debug table-------------------------------------------------//
	        // 0 : success														//
	        // 1 : false to find any Ncands										//
	        // 2 : No memory available for cand_b1b2							//
	        // 3 : No memory available for SVD in std_baseline for cand_angle	//
	        // 4 : false to find any cands in small angle						//
	        // 5 : error from std_baseline
	        // 6 : error from GSO
	        // 7 : error from search_moreN
	        // 8 : error from cand_b2_paired
	        // 9 : Failed to converge in rotation_matrix
	        //------------------------------------------------------------------//

	    	/*
	    	P_ant0.x = receiver_pvt.x;
	    	P_ant0.y = receiver_pvt.y;
	    	P_ant0.z = receiver_pvt.z;
	    	ant0_llh.lat = receiver_llh.lat;
	    	ant0_llh.lon = receiver_llh.lon;

	    	for (i = 0; i < all_lock_num; i++){
	    		P_sat[i].x = sat_position[i].x;
				P_sat[i].y = sat_position[i].y;
				P_sat[i].z = sat_position[i].z;
				pseudo_range[i] = m_rho[i];
	    	}
	    	*/
	        old_an[0] = 1.884956;
	        old_an[1] = -0.191986;
	        old_an[2] = 0.750492;

	    	P_ant0.x = -2983991;
	    	P_ant0.y = 4966682;
	    	P_ant0.z = 2657618;
	    	ant0_llh.lat = 24.7857955626183 * PI / 180;
	    	ant0_llh.lon = 120.997519699485 * PI / 180;
	    	P_sat[0].x = -26397410;
	    	P_sat[0].y = 2759056;
	    	P_sat[0].z = 2513184;
	    	P_sat[1].x = -16091230;
	    	P_sat[1].y = -745167.4;
	    	P_sat[1].z = 21017630;
	    	P_sat[2].x = -10599240;
	    	P_sat[2].y = 11992170;
	    	P_sat[2].z = 20883060;
	    	P_sat[3].x = 2131617;
	    	P_sat[3].y = 21763920;
	    	P_sat[3].z = 15990030;
	    	P_sat[4].x = -14498430;
	    	P_sat[4].y = 21499260;
	    	P_sat[4].z = 4677167;
	    	pseudo_range[0] = 23517709.5612178;
	    	pseudo_range[1] = 23270474.3916308;
	    	pseudo_range[2] = 20964642.4979180;
	    	pseudo_range[3] = 22046992.0709391;
	    	pseudo_range[4] = 20248136.1851951;


	        for(i = 0; i < all_lock_num; i++){
	        	// single difference carrier phase
	        	sdcp[i] = meas_carrier[all_lock_num].ch1_phase - meas_carrier[all_lock_num].ch0_phase;
	        	sdcp[i + all_lock_num] = meas_carrier[all_lock_num].ch2_phase - meas_carrier[all_lock_num].ch0_phase;

	        	// if ch1 or ch2 have difference invert state with ch0, add 0.5 cycle
	        	if(meas_carrier[all_lock_num].ch0_data_inverted != meas_carrier[all_lock_num].ch1_data_inverted){
	        		sdcp[i] += 0.5;
	        	}
	        	if(meas_carrier[all_lock_num].ch0_data_inverted != meas_carrier[all_lock_num].ch2_data_inverted){
	        		sdcp[i + all_lock_num] += 0.5;
	        	}
	        }

	        sprintf( string,
						 "start!!\n\r");
			SER_PutString( string);

	        ddcp_debug = attitude_sol(all_lock_num, P_ant0, ant0_llh, P_sat, pseudo_range, sdcp, sdstd, NULL, small_an, angle);
	        //ddcp_debug = attitude_sol(5, P_ant0, ant0_llh, P_sat, pseudo_range, sdcp, sdstd, old_an, small_an, angle);

	        accum_count = 24999 - status_block->accum_count;
	        end_time = ((double)accum_int_count + (double)(accum_count)/24999) * 0.5;  // unit : ms
	        delta_t = end_time - start_time;
	    	sprintf( string,
	    	             "angle = %f, %f, %f\n\r delta_t = %f ms; debug = %d\n\r",
	    	             angle[0], angle[1], angle[2], delta_t, ddcp_debug);
	    	SER_PutString( string);
	    	breakpoint = 1;
	}

}
