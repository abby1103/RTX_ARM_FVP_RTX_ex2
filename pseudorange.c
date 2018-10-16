// pseudorange.c Process measurements into pseudoranges
// Copyright (C) 2005  Andrew Greenberg
// Distributed under the GNU GENERAL PUBLIC LICENSE (GPL) Version 2 (June 1991).
// See the "COPYING" file distributed with this software for more information.

//#include <cyg/kernel/kapi.h>
//#include <cyg/infra/diag.h>
#include <math.h>
#include "constants.h"
#include "pseudorange.h"
//#include "gp4020.h"
//#include "gp4020_io.h"
#include "measure.h"
#include "position.h"
#include "time.h"

#include "tracking.h"

/******************************************************************************
 * Globals
 ******************************************************************************/

//cyg_flag_t    pseudorange_flag;

extern chan_t CH[N_CHANNELS];

pseudorange_t pr[N_CHANNELS];
gpstime_t     pr_time;

pseudorange_t pr2[N_CHANNELS];
gpstime_t     pr2_time;

/******************************************************************************
 * Clear the pseudoranges (fast).
 ******************************************************************************/
void
clear_pseudoranges( void)
{
    unsigned int ch;

    // Clear the valid flags
    for( ch = 0; ch < N_CHANNELS; ch++)
    {
        pr[ch].valid = 0;  // need to clear this flag, why do anything else?
        pr[ch].prn   = 0;
        pr[ch].range = 0;
        pr[ch].delta_range = 0;
    }
    // If there are no pseudoranges, we can't position, so clear the
    // position as well
    clear_position(); // Still dislike this technique.
                      // Does it at least optimize properly?
}


/******************************************************************************
 * Calculate them pseudoranges.
 ******************************************************************************/
void
calculate_pseudorange( unsigned short ch)
{
    unsigned short bit_count_remainder;
    unsigned long  bit_count_modded;
    
    // The epoch counter is more accurate than our bit counter in tracking.c:
    // lock(), so make sure they agree. This means fixing up the least few
    // significant bits. We've seen them disagree by two, but no more.

    // Figure out how many "50 bit counts" are in the current bit counter.
    // Why didn't they make the counter rational, like 0 - 30 bits? Oh well.
    // TODO, re-write this to avoid the mod?
    bit_count_remainder = meas[ch].meas_bit_time % 50;
    bit_count_modded = meas[ch].meas_bit_time - bit_count_remainder;

    // Fix up in case the epoch counter is on a different side of
    // a 50 bit "word" from us. Otherwise trust the almighty epoch counter.
#define TOL 24 // 50/2 -1 // 5 is the largest seen in practice?
    if( (bit_count_remainder < TOL) && (meas[ch].epoch_bits > (50 - TOL)))
        bit_count_modded -= 50;
    if( (bit_count_remainder > (50 - TOL)) && (meas[ch].epoch_bits < TOL))
        bit_count_modded += 50;

    // This can all be re-written to fixed point, don't know if we should.
    pr[ch].sat_time = (bit_count_modded + meas[ch].epoch_bits) * .02 +
        CODE_TIME *
        ( meas[ch].epoch_codes +
            (1/(double)MAX_CODE_PHASE) *
            ( meas[ch].code_phase +
                meas[ch].code_dco_phase / (double)(CODE_DCO_LENGTH)
            )
        );
    //        - QUARTER_CHIP_TIME; // Why this? What about analog delay?

    // NO ionospheric corrections, no nuthin': mostly for debug
    // FIXME: What about GPS week for this calculation?!
    pr[ch].range_old = pr[ch].range;
	pr[ch].range = (pr_time.seconds - pr[ch].sat_time) * SPEED_OF_LIGHT;

    /*
     * Delta range measurement
     *
     * This part should be revisited to adopt the way we calculate the pseudorange.
     * Set a timer and check the NCO value in it.
     */
    pr[ch].delta_range = (double)(CARRIER_REF - CH[ch].carrier_freq) * (SPEED_OF_LIGHT / 1575420000) * 0.037252902;
    pr[ch].delta_range_test = (pr[ch].range-pr[ch].range_old) * 1000;

    // Record the following information for debugging
    pr[ch].prn = meas[ch].prn;
    pr[ch].bit_time =  bit_count_modded;
    pr[ch].epoch_bits = meas[ch].epoch_bits;
    pr[ch].epoch_ms = meas[ch].epoch_codes;
}

/******************************************************************************
 * Wake up on valid measurements and produce pseudoranges. Flag the navigation
 * thread if we have four or more valid pseudoranges
 ******************************************************************************/
