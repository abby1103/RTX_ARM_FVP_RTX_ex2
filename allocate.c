#include "cmsis_os.h"

#include "allocate.h"
#include "constants.h"
#include "namuru.h"
#include "time.h"
#include "tracking.h"

/*******************************************************************************
 * Global variables
 ******************************************************************************/

/*******************************************************************************
 * Static (module level) variables
 ******************************************************************************/
 
static unsigned short PrnCode[37] = 
{
    0x3EC, 0x3D8, 0x3B0, 0x360, 0x096, 0x12C, 
    0x196, 0x32C, 0x258, 0x374, 0x2E8, 0x3A0, 
    0x340, 0x280, 0x100, 0x200, 0x226, 0x04C, 
    0x098, 0x130, 0x260, 0x0C0, 0x0CE, 0x270, 
    0x0E0, 0x1C0, 0x380, 0x300, 0x056, 0x0AC, 
    0x158, 0x2B0, 0x160, 0x0B0, 0x316, 0x22C, 
    0x0B0
};

static double last_lock[N_CHANNELS];

/******************************************************************************
 * Let tracking.c tell the allocate thread that a channel is locked... but
 * only record this fact if we have the HOW clock since we need an absolute
 * time reference.
 ******************************************************************************/
void channel_locked( unsigned short ch)
{
    if( get_clock_state() >= HOW_CLOCK)
        last_lock[ch] = get_time_in_seconds();
}
/******************************************************************************
 * Initialize a channel into acquition mode
 ******************************************************************************/
static void initialize_channel( unsigned short ch, unsigned short prn)
{
    /* Allocate the free satellite and get ready to check/allocate the next */
    CH[ch].prn = prn;

    /* Set SATCNTL register for C/A code, and LATE for dither arm */
    //namuru_ch_write(ch, PRN_KEY, PrnCode[prn]);
    ch_block->channels[ch].prn_key = PrnCode[prn];
    /* WAIT 300NS UNTIL NEXT ACCESS */
    /* Set carrier NCO */ 
    CH[ch].carrier_corr = 0;
    CH[ch].carrier_freq = CARRIER_REF + CH[ch].carrier_corr;
    CH[ch+1].carrier_freq = CARRIER_REF + CH[ch].carrier_corr;
    //namuru_ch_write(ch, CARR_NCO, CH[ch].carrier_freq);
    ch_block->channels[ch].carr_nco = CH[ch].carrier_freq;
    ch_block->channels[ch+1].carr_nco = CH[ch+1].carrier_freq;
    /* WAIT 300NS UNTIL NEXT ACCESS */
    
    /* Initialize the code and frequency search variables */
    CH[ch].codes = 0;

    // FIXME IS THIS RIGHT? Shouldn't it be chan[ch].n_freq=search_min_f like
    // in Cliff's code?
    CH[ch].n_freq   = 0;
    CH[ch].del_freq = 1;
    CH[ch].ms_count = 0;
    /* Set code NCO */
    CH[ch].code_freq = CODE_REF;
    //namuru_ch_write(ch, CODE_NCO, CH[ch].code_freq);
    ch_block->channels[ch].code_nco = CH[ch].code_freq;
    /* WAIT 300NS UNTIL NEXT ACCESS */

    /* Bit sync and frame sync flags */
    CH[ch].bit_sync = 0;

    /* Signal strength */
    CH[ch].sum = 0.0;
    CH[ch].avg = NOISE_FLOOR; /* 30 dBHz, noise floor */

    /* Turn on the channel */
    //channel_power_control( ch, CHANNEL_ON);
    /* WAIT 300NS UNTIL NEXT ACCESS */

    /* Clear missed accumulation counter */
//     CH[ch].missed = 0;

    /* Clear prompt and dither vector magnitudes */
    CH[ch].early_mag  = 0;
    CH[ch].prompt_mag = 0;
    CH[ch].late_mag   = 0;
    /* Epoch counter set flags */
    CH[ch].load_1ms_epoch_count  = 0;
    CH[ch].sync_20ms_epoch_count = 0;
        
    /* Clear the number of bits since the week began */
    CH[ch].time_in_bits = 0;
    
    /* Clear the sat navigation message for this channel,
     * including the ephemeris since we're switching sats */
    clear_messages(ch);
    clear_ephemeris(ch);
    
    /* Update channel state */
    CH[ch].state = CHANNEL_ACQUISITION;
    CH[ch+1].state = CHANNEL_ACQUISITION;
}



static void restart_channel( unsigned short ch)
{
    /* Set carrier NCO */                
    CH[ch].carrier_corr = 0;
    CH[ch].carrier_freq = CARRIER_REF + CH[ch].carrier_corr;
    //namuru_ch_write(ch, CARR_NCO, CH[ch].carrier_freq);
    ch_block->channels[ch].carr_nco = CH[ch].carrier_freq;
    /* WAIT 300NS UNTIL NEXT ACCESS */

    /* Initialize the code and frequency search variables */
    CH[ch].codes = 0;

    // FIXME IS THIS RIGHT? Shouldn't it be chan[ch].n_freq=search_min_f like
    // in Cliff's code?
    CH[ch].n_freq = 0;
    CH[ch].del_freq = 1;

    CH[ch].ms_count = 0;

    /* Set code NCO */
    CH[ch].code_freq = CODE_REF;
    //namuru_ch_write(ch, CODE_NCO, CH[ch].code_freq);
    ch_block->channels[ch].code_nco = CH[ch].code_freq;
    /* WAIT 300NS UNTIL NEXT ACCESS */

    /* Bit sync and frame sync flags */
    CH[ch].bit_sync = 0;

    /* Signal strength */
    CH[ch].sum = 0.0;
    CH[ch].avg = NOISE_FLOOR; /* 30 dBHz, noise floor */

    /* Clear missed accumulation counter */
//     CH[ch].missed = 0;

    /* Clear prompt and dither vector magnitudes */
    CH[ch].early_mag  = 0;
    CH[ch].prompt_mag = 0;
    CH[ch].late_mag   = 0;

    /* Epoch counter set flags */
    CH[ch].load_1ms_epoch_count = 0;
    CH[ch].sync_20ms_epoch_count = 0;
        
    /* Clear the number of bits since the week began */
    CH[ch].time_in_bits = 0;
       
    /* Inform the message thread to start looking for a 
    * frame sync again, and reset the epoch counter,
    * but don't reset the ephemeris */
    //clear_messages(ch);
    
    /* Update channel state */
    CH[ch].state = CHANNEL_ACQUISITION;
}

/******************************************************************************
 * We don't know anything about any satellite (cold power on with no memory)
 * so blindly go through satellites, one at a now through channels.
 ******************************************************************************/
void cold_allocate_channel(unsigned short ch)
{
    static unsigned short next_satellite = 0;  // Search satellites 1st to last

    unsigned short i;
    unsigned short already_allocated;
    do {
        next_satellite++;
        
        // Now remove the currently dead/missing PRNS (see comment in next
        // function header) - from Coast Guard's GPS NANU
        //if( (next_satellite == 12)
        //     || (next_satellite == 17)
        //     || (next_satellite == 31)
        //     || (next_satellite == 32))
        //     next_satellite++;
        
        if( next_satellite >= 32)
             next_satellite = 0; // check satellites 1 to 32     
             
        already_allocated = 0;
        for( i = 0; i < N_CHANNELS; i++)
        {
            if( (CH[i].prn == next_satellite) && (CH[i].state != CHANNEL_OFF))
            {
                already_allocated = 1;
                break;  // Exit the for loop.
            }
        }
    } while (already_allocated);
    
    initialize_channel(ch, next_satellite);
}



/******************************************************************************
 * These satellites are currently aloft and operational as of 2005/04/26:
 *   
 * PRN:   1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 13, 14, 15, 16
 * SLOT: F6, D7, C2, D4, B4, C1, C4, A3, A1, E3, D2, F3, F1, D5, B1
 *   
 * PRN:  18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30 31
 * SLOT: E4, C3, E1, D3, E2, F4, D1, A2, F2, A4, B3, F5, B2 C5
 *   
 * Sorted by satellite orbit ("slot"), they are:
 *   
 * A1: 09   B1: 16  C1: 06  D1: 24  E1: 20  F1: 14 
 * A2: 25   B2: 30  C2: 03  D2: 11  E2: 22  F2: 26 
 * A3: 08   B3: 28  C3: 19  D3: 21  E3: 10  F3: 13 
 * A4: 27   B4: 04  C4: 07  D4: 04  E4: 18  F4: 23 
 * A5:      B5:     C5: 31  D5: 15  E5:     F5: 29 
 * A6:      B6:     C6:     D6:     E6:     F6: 01 
 * A7:      B7:     C7:     D7: 02  E7:     F7:    
 *   
 * Randomly, we pick two satellites from each orbit configuration, #1 and #3.
 *   
 * This means we start with: 08,09,16,28,06,19,24,21,20,10,14,13
 * which sorted by order is: 06,08,09,10,13,14,16,19,20,21,24,28
 ******************************************************************************/

void
initialize_allocation( void)
{

    initialize_channel(  0, 31);
    //initialize_channel(  1, 7);
    initialize_channel(  2, 30);
    //initialize_channel(  3,3);
    initialize_channel(  4,24);
    //initialize_channel(  5,22);
    initialize_channel(  6,13);
    //initialize_channel(  7,19);
    initialize_channel(  8,25);
    //initialize_channel(  9,21);
    initialize_channel( 10,9);
    //initialize_channel( 11,28);
    //initialize_channel( 12, 30);


	//Tainan
	/*
	initialize_channel(  0, 17);
	initialize_channel(  1, 23);
	initialize_channel(  2, 20);
	initialize_channel(  3,11);
	initialize_channel(  4,21);
	initialize_channel(  5,24);
	initialize_channel(  6,13);
	initialize_channel(  7,14);
	initialize_channel(  8,0);
	initialize_channel(  9,1);
	initialize_channel( 10,2);
	initialize_channel( 11,3);*/
}

/******************************************************************************
 * We don't know anything about any satellite (cold power on with no memory)
 * so blindly go through satellites, one at a now through channels.
 ******************************************************************************/
void allocate_thread(void const *argument) // input 'data' not used
{
    unsigned short  ch;
    double          now;

    // There SHOULDN'T be a race condition here because initialize_allocate()
    // should keep this from being called for a while on startup.
    //cyg_flag_init(&allocate_flag);

    while(1)
    {
        // Block. If tracking() in tracking.c turns off any channels, then it 
        // will set the flag bits and wake this thread up.
        //to_allocate = cyg_flag_wait()
        osSignalWait(0x0001, osWaitForever);

        // Current now, in seconds.       
        now = get_time_in_seconds();
        
        for( ch = 0; ch < N_CHANNELS; ++ch)
        {
            // If the channel is locked, then update it's last good now stamp.
            // This relies on this function being called often - but with
            // 12 channels, and more often than not less than 8 sats in view,
            // this will happen every few minutes as a search stops.
            if (CH[ch].state == CHANNEL_LOCK 
                    && get_clock_state() >= HOW_CLOCK)
                last_lock[ch] = now;
            
            // If a channel is off but it's got a good now stamp, then
            // restart it.
            else if( CH[ch].state == CHANNEL_OFF)
            {
                // Handle a week rollover
                if (now - last_lock[ch] < 0)
                    now += SECONDS_IN_WEEK;
                
                // Restart the channel - doesn't touch any existing
                // ephemeris - if it's been 20 minutes
                if( now - last_lock[ch] < 1200);
                    //restart_channel(ch);
            }
        }
        
        // At this point, we're guaranteed that all the channels with
        // possibly valid sats are allocated or reallocated. Now cold 
        // allocate the rest.
        for( ch = 0; ch < N_CHANNELS; ++ch)
        {
            if( CH[ch].state == CHANNEL_OFF)
                cold_allocate_channel(ch);
        }
    }
}
