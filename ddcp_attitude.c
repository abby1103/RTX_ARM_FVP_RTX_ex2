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
#include "position.h"
#include "measure.h"
#include "ambiguity_resolution.h"
#include "attitude_sol.h"
#include "threads.h"



void attitude_thread(void const *argument)
{
	while (1) {
	        osSignalWait(0x0007, osWaitForever);

	        int ddcp_debug = 0;
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

	    	double sdcp[14];
	    	double sdstd = 0.05;	// 0.5 cm
	    	double small_an[3] = { 0.2618 , 0.2618 , 0.2618 };
	    	double angle[3], old_an[3] = {1.884956, -0.191986, 0.750492};
	    	int i;

	    	sat_position[0].x = -26397410;
	    	sat_position[0].y = 2759056;
	    	sat_position[0].z = -2513184;
	    	sat_position[1].x = -16091230;
	    	sat_position[1].y = -745167.4;
	    	sat_position[1].z = 21017630;
	    	sat_position[2].x = -10599240;
	    	sat_position[2].y = 11992170;
	    	sat_position[2].z = 20883060;
	    	sat_position[3].x = 2131617;
	    	sat_position[3].y = 21763920;
	    	sat_position[3].z = 15990030;
	    	sat_position[4].x = -14498430;
	    	sat_position[4].y = 21499260;
	    	sat_position[4].z = 4677167;
	    	receiver_pvt.x = -2983991;
	    	receiver_pvt.y = 4966682;
	    	receiver_pvt.z = 2657618;
	    	m_rho[0] = 23517709.5612178;
	    	m_rho[1] = 23270474.3916308;
	    	m_rho[2] = 20964642.4979180;
	    	m_rho[3] = 22046992.0709391;
	    	m_rho[4] = 20248136.1851951;
	    	receiver_llh.lat = 24.7857955626183 * PI / 180;
	    	receiver_llh.lon = 120.997519699485 * PI / 180;

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
	        ddcp_debug = attitude_sol(all_lock_num, sdcp, sdstd, old_an, small_an, angle);

	}

}
