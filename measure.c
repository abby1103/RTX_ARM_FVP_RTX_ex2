// measure.c Take measurements each TIC interrupt (~100ms)
// Copyright (C) 2005  Andrew Greenberg
// Distributed under the GNU GENERAL PUBLIC LICENSE (GPL) Version 2 (June 1991).
// See the "COPYING" file distributed with this software for more information.
#include "cmsis_os.h"
//#include <cyg/kernel/kapi.h>
//#include <cyg/infra/diag.h>
#include "measure.h"
#include "constants.h"
//#include "gp4020.h"
//#include "gp4020_io.h"
#include "message.h"
#include "pseudorange.h"
#include "time.h"
#include "tracking.h"
#include "namuru.h"
#include "position.h"
#include "ephemeris.h"

/******************************************************************************
 * Globals
 ******************************************************************************/
unsigned int    channels_ready;
measurement_t   meas[N_CHANNELS];
gpstime_t       meas_time;

//cyg_sem_t       measure_semaphore;


/******************************************************************************
 * Grab the time in bits from the tracking loops.
 *
 * Note, when the TIC comes right at the end of a message bit transition, then
 * an accum_int may increment a time_in_bits for one of the channels *while*
 * we're running in the measure_thread(). we handle that case by calling this
 * small routine directly from the accumulate DSR in interrupts.c .
 ******************************************************************************/
void
grab_bit_times( void)
{
    unsigned short ch;

    for( ch = 0; ch < N_CHANNELS; ch++)
        meas[ch].meas_bit_time = CH[ch].time_in_bits; // meas_bit_time
}

/******************************************************************************
 * Grab the latched measurement data from the accumulators after a TIC.
 ******************************************************************************/
void
measure_thread(void const *argument) // input 'data' not used
{
    static unsigned short prev_channels;

    //cyg_flag_value_t channels_ready;
    xyz_t ecef_temp;
    unsigned short   ch;
    unsigned short  satnum;
    //unsigned short   carrier_low;
    //unsigned short   carrier_high;
    //unsigned short   meas_status_a;
    unsigned short   posts_left;
    unsigned short   pr_count;

    //volatile union _gp4020_channel_control *channel_control =
    //        (volatile union _gp4020_channel_control *)
    //        GP4020_CORR_CHANNEL_CONTROL;

    //cyg_semaphore_init( &measure_semaphore, 0);

    while(1)
    {
        //cyg_semaphore_wait( &measure_semaphore);
    	osSignalWait(0x0003, osWaitForever);
        //setbits32( GPS4020_GPIO_WRITE, 4); // DEBUG: LED #2 on

        //meas_status_a = in16( GP4020_CORR_MEAS_STATUS_A);
    	gpio1_set(2);
        // We just had a Tic (~100ms) so update the receiver clock
        grab_bit_times();
        increment_time_with_tic();

        // Get the current GPS time to mark the time of measurement
        meas_time = get_time();

        // If the channel is 1) locked and 2) the signal is good and 3) the
        // clock is set vaguely correctly and 4) we've set the epoch counter
        // for this channel, THEN grab the measurement data.
        channels_ready = 0;
        for( ch = 0; ch < N_CHANNELS; ch++)
        {
            if( (CH[ch].state == CHANNEL_LOCK)
                 && CH[ch].avg > LOCK_THRESHOLD
                 && (get_clock_state() >= SF1_CLOCK)
                 && messages[ch].set_epoch_flag
				 && ch%2 == 0)
            {
                unsigned short   raw_epoch;
                // Grab the latched data from the correlators (in their
                // numerical order -- go optimized compiler, go!)
                //meas[ch].code_phase = channel_control[ch].read.code_phase;
                meas[ch].code_phase = (ch_block->channels[ch].code_meas >> 10) & 0x07FF;
                //carrier_low = channel_control[ch].read.carrier_cycle_low;
                //meas[ch].carrier_dco_phase =
                        //channel_control[ch].read.carrier_dco_phase;
                raw_epoch = ch_block->channels[ch].epoch;
                //meas[ch].code_dco_phase =
                        //channel_control[ch].read.code_dco_phase;
                //meas[ch].code_dco_phase = ch_block->channels[ch].carr_meas & 0x3FF; change by Lin
                meas[ch].code_dco_phase = ch_block->channels[ch].code_meas & 0x3FF;
                //carrier_high = channel_control[ch].read.carrier_cycle_high;

                // NOTE: Carrier phase measurements are not supported (YET)
                //meas[ch].carrier_cycles = (carrier_high << 16) | carrier_low;
                meas[ch].carrier_cycles = ch_block->channels[ch].carr_meas >> 10;

                // If a TIC hits right before a dump, it's possible for the
                // code phase to latch 2046. In this case the epoch counter
                // won't be incrmented yet. If so, increment the epoch manually
                // (see the GP4020 design manual section 7.6.17). Of course, we
                // have to handle rollover of the epoch counter too.
                if( meas[ch].code_phase < MAX_CODE_PHASE)
                {
                    // normal case
                    meas[ch].epoch_bits  = raw_epoch >> 5;
                    meas[ch].epoch_codes = raw_epoch & 0x1f;
                }
                else // overflow
                {
                    meas[ch].code_phase -= MAX_CODE_PHASE;
                    //if( (raw_epoch | 0x1f) == 19)   change by Lin
                    if( (raw_epoch & 0x1f) == 19)
                    {
                        meas[ch].epoch_codes = 0;
                        meas[ch].epoch_bits = (raw_epoch >> 5) + 1;
                        if( meas[ch].epoch_bits > 49) // Not sure we
                            meas[ch].epoch_bits = 0; // need to do this?
                    }
                    else
                    {
                        meas[ch].epoch_bits  = raw_epoch >> 5;
                        meas[ch].epoch_codes = (raw_epoch & 0x1f) + 1;
                    }
                }

                // Note: meas_bit_time set in grab_bit_times() above.
                meas[ch].doppler = CH[ch].carrier_freq;
                meas[ch].prn = CH[ch].prn;
                meas[ch].valid = 1;

                // Tell the pseudorange thread which measurements it can use.
                // Note that we don't want to just call the pr thread from here
                // because we're in a DSR and should get out ASAP
                channels_ready |= (1 << ch);
            }
            else
            {
                // Read the code_phase counter to indicate that
                // we didn't miss any interrupts (but toss the value)
                //carrier_low = channel_control[ch].read.code_phase;
                meas[ch].valid = 0; /* Can't use this measurement */
            }
        }

        // And finally, flag all the valid measurements to the pseudorange
        // thread and if there are none, but there were last TIC, then
        // explicitely clear the pseudoranges.
        if(channels_ready)
        {		 
		    pr_time = meas_time;        
		    // OK we're awake: for each measurement that we get, (Which we assume
		    // is valid, since it wouldn't get up to this thread if it weren't!),
		    // produce a pseudorange. Clear it to zero if it's not valid.
		    pr_count = 0;
		    for(ch = 0; ch < N_CHANNELS; ++ch)
		    {
		        if(channels_ready & (1 << ch))
		        {
		            calculate_pseudorange(ch);
		            pr[ch].valid = 1;
		            pr_count++;
		        }
		        else
		            pr[ch].valid = 0;
		    }

		    // If it's enabled, log the pseudorange data

		    // If we have any satellites, send them to the position thread. But
		    // note that the position thread is going to take a BZILLION years to
		    // process it all. Because we want to keep producing psuedoranges while
		    // the position thread runs, we copy over the pr's so the position
		    // thread can use a private copy.
		    if(pr_count > 0)
		    {    
		        // FIXME position_ready should be IPC; what's best? nothing
		        // seems to fit this case!
		        for( ch = 0; ch < N_CHANNELS; ++ch)
		            pr2[ch] = pr[ch]; // Better to avoid this copy (later!)
		        pr2_time = pr_time;
		        // Run the position thread which will clear position_busy
		        // when it's done.
		        //cyg_semaphore_post( &position_semaphore);
		        satnum = 0;
		        for(ch = 0; ch < N_CHANNELS; ++ch)
		        {
		            if(ephemeris[ch].valid && pr2[ch].valid)
		            {
		                // get the satellite ECEF XYZ + time (correction?) of the
		                // satellite. Note that sat_position is ECEF*T* type.
						sat_pos_by_ch[ch] = SatPosEphemeris(ch);
						sat_azel[ch]      = satellite_azel(sat_pos_by_ch[ch]);

		                // Pack the satellite positions into an array for efficiency
		                // in the calculate_position function.
						sat_position[satnum].x       = sat_pos_by_ch[ch].x;
						sat_position[satnum].y       = sat_pos_by_ch[ch].y;
						sat_position[satnum].z       = sat_pos_by_ch[ch].z;
						sat_position[satnum].channel = ch;
						sat_position[satnum].prn     = pr2[ch].prn;

		                // Doppler measurement is not supported.
		                // SHOCK!! SHOCK, I TELL YOU.

		                // Pseudorange measurement. No ionospheric delay correction.
		                m_rho[satnum] = pr2[ch].range +
		                    (SPEED_OF_LIGHT * sat_pos_by_ch[ch].tb);
		                // (.tb) is the per-satellite clock correction calculated from
		                // ephemeris data.

		                satnum++; // Number of valid satellites
		            }
		        }

		        // If we've got 4 or more satellites, position!
		        if(satnum >= 0)
		        {
		            // Let the world know we're now calculating position
		            positioning = 1;

		            receiver_pvt = calculate_position(satnum);

		            if( receiver_pvt.valid)
		            {
		                // Move from meters to seconds
		                receiver_pvt.b /= SPEED_OF_LIGHT;

		                // Correct the clock with the latest bias. But not that the
		                // clock correction function may be smoothing the correction
		                // and doing other funky things.
		                set_clock_correction( receiver_pvt.b);

		                // Put receiver position in LLH
						ecef_temp.x  = receiver_pvt.x;
						ecef_temp.y  = receiver_pvt.y;
						ecef_temp.z  = receiver_pvt.z;
						receiver_llh = ecef_to_llh(ecef_temp);
		            }
		            // And call the display log function, which will just
		            // instantly return if we're not logging
		        }
		        else
		        {
		            // Not enough sats to calc position.
		            positioning = 0;
		        }
		        // FIXME: shouldn't be called from this thread. HACK!
		        // Flag pseudorange.c that we're ready for a new pseudorange set
		        // in pr2.          
		    }
		    else
		    {
		        // Uh... no pseudoranges? That's bad.
		        //diag_printf( "PSEUDORANGE: CALLED WITH NO SATELLITES.\n");
		        clear_pseudoranges();
		    }
		}
        else if(prev_channels)
            clear_pseudoranges();

        prev_channels = channels_ready;
        // Block until there's a valid set of measurements posted from the
        // measurement thread.
        //cyg_semaphore_wait( &position_semaphore);

        //setbits32( GPS4020_GPIO_WRITE, (1 << 7));   // DEBUG LED 7

        // OK we're awake: use a COPY of the PRs/measurements, because they're
        // going to change in about 100ms and there's NO WAY IN HELL we're
        // going to be done by then. And check that the ephemeris is still
        // good while we're at it.
        
        //clearbits32( GPS4020_GPIO_WRITE, (1 << 7));   // DEBUG LED 7
        gpio1_clr(2);
    }
}
