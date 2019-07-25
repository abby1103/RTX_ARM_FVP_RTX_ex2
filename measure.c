/*
 * measure.c Take measurements each TIC interrupt (~100ms)
 * Copyright (C) 2005  Andrew Greenberg
 * Distributed under the GNU GENERAL PUBLIC LICENSE (GPL) Version 2 (June 1991).
 * See the "COPYING" file distributed with this software for more information.
 */
#include "cmsis_os.h"
#include "measure.h"
#include "constants.h"
#include "message.h"
#include "pseudorange.h"
#include "time.h"
#include "tracking.h"
#include "namuru.h"
#include "position.h"
#include "ephemeris.h"
#include "DOP.h"
#include "threads.h"

/******************************************************************************
 * Globals
 ******************************************************************************/
unsigned int    channels_ready;
measurement_t   meas[N_CHANNELS];
gpstime_t       meas_time;
DOP				receiver_DOP;

/******************************************************************************
 * Grab the time in bits from the tracking loops.
 *
 * Note, when the TIC comes right at the end of a message bit transition, then
 * an accum_int may increment a time_in_bits for one of the channels *while*
 * we're running in the measure_thread(). we handle that case by calling this
 * small routine directly from the accumulate DSR in interrupts.c .
 ******************************************************************************/
void grab_bit_times( void)
{
    unsigned short ch;

    for( ch = 0; ch < N_CHANNELS; ch++)
        meas[ch].meas_bit_time = CH[ch].time_in_bits;
}

/******************************************************************************
 * Grab the latched measurement data from the accumulators after a TIC.
 ******************************************************************************/
void measure_thread(void const *argument)
{
    static unsigned short prev_channels;
    xyz_t ecef_temp;
    unsigned short   ch;
    unsigned short  satnum;
    unsigned short   posts_left;
    unsigned short   pr_count;
    while(1) {
    	osSignalWait(0x0003, osWaitForever);
        grab_bit_times();
        increment_time_with_tic();

        meas_time = get_time();
        /*
         * If the channel is 1) locked and 2) the signal is good and 3) the
         * clock is set vaguely correctly and 4) we've set the epoch counter
         * for this channel, THEN grab the measurement data.
         */
        channels_ready = 0;
        for( ch = 0; ch < N_CHANNELS; ch++)
        {
            if( (CH[ch].state == CHANNEL_LOCK)
                 && CH[ch].avg > LOCK_THRESHOLD
                 && (get_clock_state() >= SF1_CLOCK)
                 && messages[ch].set_epoch_flag
                 )
            {
                unsigned short   raw_epoch;
                meas[ch].code_phase = (ch_block->channels[ch].code_meas >> 10) & 0x07FF;
                raw_epoch = ch_block->channels[ch].epoch;
                meas[ch].code_dco_phase = ch_block->channels[ch].code_meas & 0x3FF;
                /* NOTE: Carrier phase measurements are not supported. */
                meas[ch].carrier_cycles = ch_block->channels[ch].carr_meas >> 10;

                /*
                 * If a TIC hits right before a dump, it's possible for the
                 * code phase to latch 2046. In this case the epoch counter
                 * won't be incrmented yet. If so, increment the epoch manually
                 * (see the GP4020 design manual section 7.6.17). Of course, we
                 * have to handle rollover of the epoch counter too.
                 */
                if (meas[ch].code_phase < MAX_CODE_PHASE) {
                    meas[ch].epoch_bits  = raw_epoch >> 5;
                    meas[ch].epoch_codes = raw_epoch & 0x1f;
                } else {
                    meas[ch].code_phase -= MAX_CODE_PHASE;
                    if ((raw_epoch & 0x1f) == 19) {
                        meas[ch].epoch_codes = 0;
                        meas[ch].epoch_bits = (raw_epoch >> 5) + 1;
                        if( meas[ch].epoch_bits > 49)
                            meas[ch].epoch_bits = 0;
                    } else {
                        meas[ch].epoch_bits  = raw_epoch >> 5;
                        meas[ch].epoch_codes = (raw_epoch & 0x1f) + 1;
                    }
                }
                meas[ch].doppler = CH[ch].carrier_freq;
                meas[ch].prn = CH[ch].prn;
                meas[ch].valid = 1;

                /*
                 * Tell the pseudorange thread which measurements it can use.
                 * Note that we don't want to just call the pr thread from here
                 * because we're in a DSR and should get out ASAP
                 */
                channels_ready |= (1 << ch);
            } else {
                /*
                 * Read the code_phase counter to indicate that
                 * we didn't miss any interrupts (but toss the value)
                 */
                meas[ch].valid = 0; /* Can't use this measurement */
            }
        }

        /*
         * And finally, flag all the valid measurements to the pseudorange
         * thread and if there are none, but there were last TIC, then
         * explicitely clear the pseudoranges.
         */
        if (channels_ready) {		 
		    pr_time = meas_time;
            /*        
		     * OK we're awake: for each measurement that we get, (Which we assume
		     * is valid, since it wouldn't get up to this thread if it weren't!),
		     * produce a pseudorange. Clear it to zero if it's not valid.
             */
		    pr_count = 0;
		    for (ch = 0; ch < N_CHANNELS; ++ch) {
		        if (channels_ready & (1 << ch)) {
		            calculate_pseudorange(ch);
		            pr[ch].valid = 1;
		            pr_count++;
		        }
		        else
		            pr[ch].valid = 0;
		    }

		    /*
             * If it's enabled, log the pseudorange data
             *
		     * If we have any satellites, send them to the position thread. But
		     * note that the position thread is going to take a BZILLION years to
		     * process it all. Because we want to keep producing psuedoranges while
		     * the position thread runs, we copy over the pr's so the position
		     * thread can use a private copy.
             */
		    if (pr_count > 0) {    
		        for( ch = 0; ch < N_CHANNELS; ++ch)
		            pr2[ch] = pr[ch];
		        pr2_time = pr_time;
		        satnum = 0;
		        for (ch = 0; ch < N_CHANNELS; ++ch) {
		            if (ephemeris[ch].valid && pr2[ch].valid) {
		            	sat_pos_by_ch_old[ch] = sat_pos_by_ch[ch];
						sat_pos_by_ch[ch] = SatPosEphemeris(ch).pos;
						sat_vel_by_ch[ch] = SatPosEphemeris(ch).vel;
						sat_vel_by_ch_test[ch].vx = (sat_pos_by_ch[ch].x - sat_pos_by_ch_old[ch].x) * 10;
						sat_vel_by_ch_test[ch].vy = (sat_pos_by_ch[ch].y - sat_pos_by_ch_old[ch].y) * 10;
						sat_vel_by_ch_test[ch].vz = (sat_pos_by_ch[ch].z - sat_pos_by_ch_old[ch].z) * 10;
						sat_vel_by_ch[ch].td = (sat_pos_by_ch[ch].tb - sat_pos_by_ch_old[ch].tb) * 10;
						sat_azel[ch]      = satellite_azel(sat_pos_by_ch[ch]);

		                /*
                         * Pack the satellite positions into an array for efficiency
		                 * in the calculate_position function.
                         */
						sat_position[satnum].x       = sat_pos_by_ch[ch].x;
						sat_position[satnum].y       = sat_pos_by_ch[ch].y;
						sat_position[satnum].z       = sat_pos_by_ch[ch].z;
						sat_position[satnum].vx       = sat_vel_by_ch[ch].vx;
						sat_position[satnum].vy       = sat_vel_by_ch[ch].vy;
						sat_position[satnum].vz       = sat_vel_by_ch[ch].vz;
						sat_position[satnum].channel = ch;
						sat_position[satnum].prn     = pr2[ch].prn;

		                /* Pseudorange measurement. No ionospheric delay correction. */
		                m_rho[satnum] = pr2[ch].range +
		                    (SPEED_OF_LIGHT * sat_pos_by_ch[ch].tb);
		                m_rho_dot[satnum] = pr2[ch].delta_range;
		                m_rho_dot_test[satnum] =pr[ch].delta_range_test;
		                satnum++;
		            }
		        }

		        /* If we've got 4 or more satellites, position! */
		        if (satnum >= 4) {
		            positioning = 1;
		            receiver_pvt = calculate_position(satnum);
		            receiver_pvt_velocity = calculate_velocity(satnum);
		            receiver_pvt_velocity_test = calculate_velocity_test(satnum);
		            if (receiver_pvt.valid) {
		                receiver_pvt.b /= SPEED_OF_LIGHT;
                        
                        /*
		                 * Correct the clock with the latest bias. But not that the
		                 * clock correction function may be smoothing the correction
		                 * and doing other funky things.
                         */
		                set_clock_correction( receiver_pvt.b);

						ecef_temp.x  = receiver_pvt.x;
						ecef_temp.y  = receiver_pvt.y;
						ecef_temp.z  = receiver_pvt.z;
						receiver_llh = ecef_to_llh(ecef_temp);
						receiver_DOP = CALCULATE_DOP(satnum);
						//osSignalSet(ekf_position_thread_id,  0x0005);
		            }
		        } else {
		            positioning = 0;
		        }       
		    } else {
		        clear_pseudoranges();
		    }
		} else if(prev_channels)
            clear_pseudoranges();

        prev_channels = channels_ready;
    }
}
