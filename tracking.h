// tracking.h header file for tracking.c
// Copyright (C) 2005  Andrew Greenberg
// Distributed under the GNU GENERAL PUBLIC LICENSE (GPL) Version 2 (June 1991).
// See the "COPYING" file distributed with this software for more information.

#ifndef __TRACKING_H
#define __TRACKING_H

#include "namuru.h" // N_CHANNELS

/*******************************************************************************
 * Definitions
 ******************************************************************************/

// Clock info
#define SYSTEM_CLOCK  50e6                  /* nominal [Hz] */
#define SAMPLE_CLOCK  16.368e6  /* sets correlator timing */

// Carrier NCO timing
/* L1 carrier is down converted by the GP2015 front end to an IF frequency
 * of 1.4053968254MHz */
#define IF_FREQ       4.092e6

#define CARR_FREQ_RES (SAMPLE_CLOCK / (1 << 30)) // 0.037252902Hz [Hz] (Carrier NCO is 30 bits)                                              
#define CODE_FREQ_RES (SAMPLE_CLOCK / (1 << 29)) // 0.074505805 [Hz] (Carrier NCO is 29 bits)                    

#define CODE_REF      21968758//(unsigned int)(0.5 + 2 * CHIP_RATE / CODE_FREQ_RES)
#define CARRIER_REF   87875031//(unsigned int)(0.5 + IF_FREQ / CARR_FREQ_RES)

// Noise floor defines
//#define NOISE_FLOOR     527
#define NOISE_FLOOR     1055
//#define LOCK_THRESHOLD  746
#define LOCK_THRESHOLD  2100
//#define AcqThresh       2200
#define AcqThresh       2500


//Tracking parameter 
//#define CarrSrchWidth   24
//#define CarrSrchStep    10737//500Hz
#define CarrSrchWidth	96
#define CarrSrchStep	2684
//#define PullCodeK
//#define PullCodeD
//#define PullInTime
//#define PhaseTest
//#define PullCarrK
//#define PullCarrD
//#define Rms
//#define CodeCorr
//#define TrackCarrK
//#define TrackCarrD
//#define TrackCodeK
//#define TrackCodeD
//#define TrackDiv

typedef enum {
    CHANNEL_OFF         = 0,
    CHANNEL_ON          = 1,
    CHANNEL_ACQUISITION = 1,
    CHANNEL_CONFIRM     = 2,
    CHANNEL_PULL_IN     = 3,
    CHANNEL_LOCK        = 4
} TRACKING_ENUM;

/*******************************************************************************
 * Structures
 ******************************************************************************/

typedef struct
{
    TRACKING_ENUM       state;
    unsigned short      prn;
    int                 i_early, i_prompt, i_late; // Track arms (signed!)
    int                 q_early, q_prompt, q_late; // Track arms (signed!)
    int                 i_prompt_old, q_prompt_old;//for FLL
    long                carrier_corr;//not used now
    long                carrier_freq,d_carrier_freq, carrier_freq_old, carrier_freq_old_old,carrier_freq_qq;       // in NCO hex units
    long                code_freq;          // in NCO hex units
    signed short        n_freq;             // Carrier frequency search bin
    signed short        del_freq;           // Frequency search delta
    unsigned short      codes;              // Current code phase
                                            // (in 1/2 chips, 0 - 2044)
    long                i_confirm;          //
    long                n_thresh;           //
    unsigned short      missed;             // # of missed accumulator dumps
    long                early_mag, prompt_mag, late_mag;         //
    unsigned short      ch_time;
    long                sum;
    long                th_rms;
    long                delta_code_phase, delta_code_phase_old;
    long                delta_carrier_phase, delta_carrier_phase_old, delta_carrier_phase_old_old;
    long                delta_carrier_freq, delta_carrier_freq_old;
    long                old_theta;
    unsigned long       ms_sign;
    unsigned short      ms_count;
    long                avg;
    unsigned short      check_average;

    long                i_early_20 ,i_prompt_20, i_late_20;
    long                q_early_20 ,q_prompt_20, q_late_20;

    unsigned short      bit_sync;
    unsigned short      bit;

    unsigned short      clear_message;
    unsigned short      missed_message_bit;

    unsigned short      load_1ms_epoch_count;   // Flags to coordinate loading
    unsigned short      sync_20ms_epoch_count;  // the epoch counters.
    
    unsigned long       time_in_bits;           // Time of week, in bits

    int                 sign_pos, prev_sign_pos;  // Expected bits edges: current and previous.
    int                 sign_count;               // How many times bit edges distance is more then 19 ms!
    int ch_debug;
    long				doppler_freq;


    int 				no_view;                // How many times the antenna lose lock
    int                 phase_info;           // dual antenna: how two channel data coherency


} chan_t;

/*******************************************************************************
 * Prototypes (Globally visible functions)
 ******************************************************************************/

// Called from gpl-gps.c
void tracking(void);
void initialize_tracking(void);

// called from allocate.c
//void set_code_dco_rate( unsigned long ch, unsigned long freq);
//void set_carrier_dco_rate( unsigned short ch, unsigned long freq);
void channel_power_control(unsigned short ch, unsigned short on);

/*******************************************************************************
 * Externs (globally visible variables)
 ******************************************************************************/
extern unsigned int channels_with_bits;
extern chan_t CH[N_CHANNELS];


#endif // __TRACKING_H



