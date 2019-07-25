/*
 * Always include cmsis_os.h first then system header,
 * altera hardware abstraction layer finally user defined header.
 */
#include "cmsis_os.h"

/* system header */
#include <stdlib.h>
#include <math.h>

/* HAL header */
#include "socal/socal.h"

/* User defined header */
#include "constants.h"
#include "tracking.h"
#include "namuru.h"
#include "threads.h"
#include "measure.h"
#include "message.h"

/* Later on, we scale 1 radian = 2^14 */
#define PI_SHIFT14       (long)(0.5 + PI * (1 << 14))
#define PI_OVER2_SHIFT14 (long)(0.5 + PI * (1 << 13))
#define sign(x) (x > 0 ? 1 : (x == 0) ? 0 : -1)

#define D_DATA_STORED 200000

/*******************************************************************************
 * Global variables
 ******************************************************************************/
chan_t CH[N_CHANNELS];
unsigned int channels_with_bits;

unsigned int d_pull_in_monitor[32];

// debug global variables
typedef struct {
    TRACKING_ENUM       state;
    unsigned short      prn;
    int                 i_early, i_prompt, i_late;
    int                 q_early, q_prompt, q_late;
    long                carrier_freq;       // in NCO hex units
    long                code_freq;          // in NCO hex units
 //   signed short        del_freq;           // Frequency search delta
 //   unsigned short      codes;              // Current code phase
    int 				del_carr_phase;
} d_ch_log_t;

d_ch_log_t d_data[D_DATA_STORED];
int d_data_index = 0;

void d_log_data(int ch);
// debug end

static void data_bit_coherency(unsigned short ch);
static void data_bit_coherency_init(unsigned short ch);

int signExtension(int instr) {
    int value = (0x0000FFFF & instr);
    int mask = 0x00008000;
    if (mask & instr) {
        value += 0xFFFF0000;
    }
    return value;
}

/******************************************************************************
 * Compute the approximate magnitude (square norm) of 2 numbers
 *
 * The "correct" function is naturally sqrt(a^2+b^2).
 * This computation is too slow for our application however.
 * We use the leading order approximation (mag = a+b/2) for b<a with sgn fixes.
 * This is probably as good as possible without a multiply.
 *
 * Haven't tried a fit, but based on endpoints a+(sqrt(2)-1)*b is a better
 * approximation. Everything else seems to have either a couple multiplies and
 * a divide, or an actual square root. It's a fact that if there's no hardware
 * multiply the binary square root is actually faster than a multiply on a GP
 * machine, but since the ARM7TDMI has a multiply the root is slower for us.
 ******************************************************************************/
static long lmag( long a, long b)
{
    if( a | b)
    {
        long c, d;
        c = labs( a);
        d = labs( b);

        if( c > d)
            return( c + (d >> 1));
        else
            return( d + (c >> 1));
    }
    else return( 0);
}
/******************************************************************************
 * classic signum function written for the short datatype
 ******************************************************************************/
static short sgn(long data)
{
    return( data < 0 ? -1: data != 0);
}

// Added smag() to deal with shorts only (should speed things up)
/* Point #1, this is quite possibly _not_ faster, because the ARM internally is
 * 32bit, so each op has to be masked to 16bits. Only 16bit memory accesses for
 * data are speeded up, which this has few of.
 * Point #2, do you want to return unsigned? perhaps this doesn't matter. I'm
 * not enough of a C-head. Is this only a compile-time thing or will it force
 * some sort of run-time conversion? Signed seems safer.
 * Point #3, avoiding the library call (labs) _shouldn't_ be faster, but i
 * concede it might be, and it almost certainly isn't slower.
 */
static unsigned short smag(short a, short b)
{
    if( a < 0) a = -a;
    if( b < 0) b = -b;

    if( a > b)
        return( a + (b >> 1));
    else
        return( b + (a >> 1));
}
/*******************************************************************************
FUNCTION fix_atan2( long y,long x)
RETURNS  long integer

PARAMETERS
        y  long   quadrature fixed point value
        x  long   in-phase fixed point value

PURPOSE
// This is the phase discriminator function.
      This function computes the fixed point arctangent represented by
      y and x in the parameter list
      1 radian = 2^14 = 16384
      based on the power series  2^14*( (y/x)-2/9*(y/x)^3 )

// This is not a particularly good approximation.
// In particular, 0.2332025325081921, rather than 2/9 is the best
// fit parameter. The next simplest thing to do is fit {x,x^3}
// I'm assuming the interval of validity is [0,1).
// Fitting {1,x,x^3} is bad because a discriminator must come smoothly to
// zero.
// I wonder, in the average math library, if it might be faster to do the
// division un-signed?
// It's not much more expensive to fit x+a*x^2+b*x^3 (one more add).
// The compiler may not properly optimize ()/9.

WRITTEN BY
    Clifford Kelley
    Fixed for y==x  added special code for x==0 suggested by Joel Barnes, UNSW
*******************************************************************************/
static long fix_atan2( long y, long x)
{
    long result=0,n,n3;

    // 4 quadrants, one invalid case

    if( (x == 0) && (y == 0)) /* Invalid case */
        return(result);
    if( x > 0 && x >= labs(y))
    {
        n      = (y << 14) / x;
        n3     = ((((n*n) >> 14) * n) >> 13) / 9;
        result = n - n3;
    }
    else if( x <= 0 && -x >= labs(y))
    {
        n  = (y << 14) / x;
        n3 = ((((n * n) >> 14) * n) >> 13) / 9;
        if(y >  0)
            result = n - n3 + PI_SHIFT14;
        else if( y <= 0)
            result = n - n3 - PI_SHIFT14;
    }
    else if( y > 0 &&  y > labs(x))
    {
        n      = (x << 14) / y;
        n3     = ((((n * n) >> 14) * n) >> 13) / 9;
        result = PI_OVER2_SHIFT14 - n + n3;
    }
    else if( y < 0 && -y > labs(x))
    {
        n      = (x << 14)/y;
        n3     = ((((n * n) >> 14) * n) >> 13) / 9;
        result = -n + n3 - PI_OVER2_SHIFT14;
    }
    return(result);
}


/******************************************************************************
 * Need to set up CH[] structure and initialize the loop dynamic parameters.
 *
 * Currently called from cyg_user_start() in gpl-gps.c
 ******************************************************************************/


/******************************************************************************
 FUNCTION acquire( unsigned long ch)
 RETURNS  None.

 PARAMETERS
 ch  char // Which correlator channel to use

 PURPOSE  to perform initial acquire by searching code and frequency space
 looking for a high correlation

 WRITTEN BY
 Clifford Kelley

 ******************************************************************************/
static void acquire( unsigned short ch)
{
    /* Search carrier frequency bins */
    if (abs(CH[ch].n_freq) <= CarrSrchWidth) {
        CH[ch].early_mag  = smag( CH[ch].i_early , CH[ch].q_early);
        CH[ch].prompt_mag = smag( CH[ch].i_prompt, CH[ch].q_prompt);
        CH[ch].late_mag   = smag( CH[ch].i_late  , CH[ch].q_late);

        CH[ch+1].early_mag  = smag( CH[ch+1].i_early , CH[ch+1].q_early);
		CH[ch+1].prompt_mag = smag( CH[ch+1].i_prompt, CH[ch+1].q_prompt);
		CH[ch+1].late_mag   = smag( CH[ch+1].i_late  , CH[ch+1].q_late);

		if((CH[ch].early_mag > AcqThresh) || (CH[ch].prompt_mag > AcqThresh) || (CH[ch].late_mag > AcqThresh)
		        		||(CH[ch+1].early_mag > AcqThresh) || (CH[ch+1].prompt_mag > AcqThresh) || (CH[ch+1].late_mag > AcqThresh))
		{
            CH[ch].state     = CHANNEL_CONFIRM;
            CH[ch].i_confirm = 0;
            CH[ch].n_thresh  = 0;
            return;
        }

        ch_block->channels[ch].code_slew = 2;
    	CH[ch].codes += 2;
        
        if (CH[ch].codes >= 2044) {
            CH[ch].codes = 0;
            if (CH[ch].n_freq & 1) {
                /* Odd search bins map to the "right" */
                CH[ch].carrier_freq = CARRIER_REF + CH[ch].carrier_corr + CarrSrchStep * (1 + (CH[ch].n_freq >> 1));
            } else {
                /* Even search bins are to the "left" of CARRIER_REF */
                CH[ch].carrier_freq = CARRIER_REF + CH[ch].carrier_corr - CarrSrchStep * (CH[ch].n_freq >> 1);
            }
            CH[ch+1].carrier_freq = CH[ch].carrier_freq;
            ch_block->channels[ch].carr_nco = CH[ch].carrier_freq;
            ch_block->channels[ch+1].carr_nco = CH[ch+1].carrier_freq;
            CH[ch].n_freq++;
        }
    } else {
        CH[ch].state = CHANNEL_OFF;
    }
}

/*******************************************************************************
 FUNCTION confirm(unsigned long ch)
 RETURNS  None.

 PARAMETERS
 ch  char  channel number

 PURPOSE  to lock the presence of a high correlation peak using an n of m
 algorithm

 WRITTEN BY
 Clifford Kelley

*******************************************************************************/
static void confirm(unsigned short ch)
{
    CH[ch].i_confirm++;

    CH[ch].early_mag  = smag( CH[ch].i_early , CH[ch].q_early );
    CH[ch].prompt_mag = smag( CH[ch].i_prompt, CH[ch].q_prompt);
    CH[ch].late_mag   = smag( CH[ch].i_late  , CH[ch].q_late  );
    
    CH[ch+1].early_mag  = smag( CH[ch+1].i_early , CH[ch+1].q_early );
    CH[ch+1].prompt_mag = smag( CH[ch+1].i_prompt, CH[ch+1].q_prompt);
    CH[ch+1].late_mag   = smag( CH[ch+1].i_late  , CH[ch+1].q_late  );

    if((CH[ch].early_mag > AcqThresh) || (CH[ch].prompt_mag > AcqThresh) || (CH[ch].late_mag > AcqThresh)
        		||(CH[ch+1].early_mag > AcqThresh) || (CH[ch+1].prompt_mag > AcqThresh) || (CH[ch+1].late_mag > AcqThresh))
        CH[ch].n_thresh++;

    if (CH[ch].i_confirm >= 5) {
        if (CH[ch].n_thresh >= 4) {
            CH[ch].state                   = CHANNEL_PULL_IN;
            CH[ch].ch_time                 = 0;
            CH[ch].sum                     = 0;
            CH[ch].th_rms                  = 0;
            CH[ch].bit_sync                = 0;
            CH[ch].delta_code_phase_old    = 0;
            CH[ch].delta_carrier_phase_old = 0;
            CH[ch].delta_carrier_phase_old_old=0;
            CH[ch].delta_carrier_freq_old  = 0;
            CH[ch].carrier_freq_old=CH[ch].carrier_freq;
            CH[ch].carrier_freq_old_old=CH[ch].carrier_freq;
            CH[ch].carrier_freq_qq= CH[ch].carrier_freq;
            CH[ch].old_theta               = 0;
            CH[ch].delta_carrier_freq  =0 ;
            CH[ch].delta_carrier_phase = 0 ;
            CH[ch].ms_sign                 = 0x12345; /* Some garbage data */
            CH[ch].ms_count                = 0;

            CH[ch+1].bit_sync                = 0;
			CH[ch+1].delta_code_phase_old    = 0;
			CH[ch+1].delta_carrier_phase_old = 0;
        } else {
            /* Keep searching - assumes search parameters are still ok */
            CH[ch].state = CHANNEL_ACQUISITION;
            /* Clear sync flags */
            CH[ch].bit_sync = 0;
        }
    }
}


/*******************************************************************************
 FUNCTION pull_in( unsigned long ch)
 RETURNS  None.

 PARAMETERS
 ch  char  channel number

 PURPOSE
 pull in the frequency by trying to track the signal with a
 combination FLL and PLL
 it will attempt to track for xxx ms, the last xxx ms of data will be
 gathered to determine if we have both code and carrier lock
 if so we will transition to track

 WRITTEN BY
 Clifford Kelley

*******************************************************************************/

static void pull_in (unsigned short ch)
{
    long cross, dot;  
    if ((CH[ch].i_early != 0) && (CH[ch].q_early != 0) && (CH[ch].i_late != 0) && (CH[ch].q_late != 0)) {
        CH[ch].early_mag       = lmag(CH[ch].i_early, CH[ch].q_early);
        CH[ch].prompt_mag      = lmag( CH[ch].i_prompt, CH[ch].q_prompt);
        CH[ch].late_mag        = lmag(CH[ch].i_late, CH[ch].q_late);
        CH[ch].delta_code_phase = (CH[ch].early_mag - CH[ch].late_mag) << 14;
        CH[ch].delta_code_phase = (CH[ch].delta_code_phase / (CH[ch].early_mag + CH[ch].late_mag)) >> 1;
    }

    if ((CH[ch+1].i_early != 0) && (CH[ch+1].q_early != 0) && (CH[ch+1].i_late != 0) && (CH[ch+1].q_late != 0)) {
            CH[ch+1].early_mag       = lmag(CH[ch+1].i_early, CH[ch+1].q_early);
            CH[ch+1].prompt_mag      = lmag( CH[ch+1].i_prompt, CH[ch+1].q_prompt);
            CH[ch+1].late_mag        = lmag(CH[ch+1].i_late, CH[ch+1].q_late);
            CH[ch+1].delta_code_phase = (CH[ch+1].early_mag - CH[ch+1].late_mag) << 14;
            CH[ch+1].delta_code_phase = (CH[ch+1].delta_code_phase / (CH[ch+1].early_mag + CH[ch+1].late_mag)) >> 1;
    }

    if(CH[ch].prompt_mag > CH[ch+1].prompt_mag) {
            CH[ch].code_freq += ((449 * (CH[ch].delta_code_phase - CH[ch].delta_code_phase_old) + 59 * CH[ch].delta_code_phase) >> 14);
    } else {
            //CH[ch].code_freq += ((449 * (CH[ch+1].delta_code_phase - CH[ch+1].delta_code_phase_old) + 59 * CH[ch+1].delta_code_phase) >> 14);
    		CH[ch].code_freq += ((449 * (CH[ch+1].delta_code_phase - CH[ch+1].delta_code_phase_old) + 59 * CH[ch+1].delta_code_phase) >> 14);
    }

    ch_block->channels[ch].code_nco = CH[ch].code_freq;

    CH[ch].delta_code_phase_old = CH[ch].delta_code_phase;
    CH[ch+1].delta_code_phase_old = CH[ch+1].delta_code_phase;

    if ((CH[ch].i_prompt != 0) && (CH[ch].q_prompt != 0) && (CH[ch].i_prompt_old != 0) && (CH[ch].q_prompt_old != 0)) {
        cross = CH[ch].i_prompt_old * CH[ch].q_prompt-CH[ch].i_prompt * CH[ch].q_prompt_old;
        dot   = labs(CH[ch].i_prompt * CH[ch].i_prompt_old + CH[ch].q_prompt * CH[ch].q_prompt_old);
        cross = cross >> 14;
        dot   = dot   >> 14;
        CH[ch].delta_carrier_freq  = fix_atan2(cross, dot) ;
        CH[ch].delta_carrier_phase = fix_atan2((CH[ch].q_prompt * sign(CH[ch].i_prompt)), labs(CH[ch].i_prompt)) ;
    }
    CH[ch].carrier_freq += ((904 * CH[ch].delta_carrier_phase - 798 * CH[ch].delta_carrier_phase_old + 50 * CH[ch].delta_carrier_freq) >> 14);

    CH[ch].delta_carrier_phase_old_old = CH[ch].delta_carrier_phase_old;
    CH[ch].delta_carrier_phase_old = CH[ch].delta_carrier_phase;
    CH[ch].delta_carrier_freq_old=CH[ch].delta_carrier_freq;
    CH[ch].carrier_freq_old_old = CH[ch].carrier_freq_old;
    CH[ch].carrier_freq_old = CH[ch].carrier_freq;
    ch_block->channels[ch].carr_nco = CH[ch].carrier_freq;


    if ((CH[ch+1].i_prompt != 0) && (CH[ch+1].q_prompt != 0) && (CH[ch+1].i_prompt_old != 0) && (CH[ch+1].q_prompt_old != 0)) {
            cross = CH[ch+1].i_prompt_old * CH[ch+1].q_prompt-CH[ch+1].i_prompt * CH[ch+1].q_prompt_old;
            dot   = labs(CH[ch+1].i_prompt * CH[ch+1].i_prompt_old + CH[ch+1].q_prompt * CH[ch+1].q_prompt_old);
            cross = cross >> 14;
            dot   = dot   >> 14;
            CH[ch+1].delta_carrier_freq  = fix_atan2(cross, dot) ;
            CH[ch+1].delta_carrier_phase = fix_atan2((CH[ch+1].q_prompt * sign(CH[ch+1].i_prompt)), labs(CH[ch+1].i_prompt)) ;
        }
	CH[ch+1].carrier_freq += ((904 * CH[ch+1].delta_carrier_phase - 798 * CH[ch+1].delta_carrier_phase_old + 50 * CH[ch+1].delta_carrier_freq) >> 14);

	CH[ch+1].delta_carrier_phase_old_old = CH[ch+1].delta_carrier_phase_old;
	CH[ch+1].delta_carrier_phase_old = CH[ch+1].delta_carrier_phase;
	CH[ch+1].delta_carrier_freq_old=CH[ch+1].delta_carrier_freq;
	CH[ch+1].carrier_freq_old_old = CH[ch+1].carrier_freq_old;
	CH[ch+1].carrier_freq_old = CH[ch+1].carrier_freq;
	ch_block->channels[ch+1].carr_nco = CH[ch+1].carrier_freq;

    /* detect bits edges according to sign change of prompt in-phase correlator output. */
    if (sign(CH[ch].i_prompt) == -sign(CH[ch].i_prompt_old)) { 
        CH[ch].prev_sign_pos = CH[ch].sign_pos;
        CH[ch].sign_pos      = CH[ch].ch_time;

        /* Bits edges always multiples of 20ms */
        if ((CH[ch].sign_pos - CH[ch].prev_sign_pos) > 19)
            CH[ch].sign_count++;
        else
            CH[ch].sign_count = 0;
    }
    CH[ch].ch_time++;

    if (sign(CH[ch+1].i_prompt) == -sign(CH[ch+1].i_prompt_old)) {
		CH[ch+1].prev_sign_pos = CH[ch+1].sign_pos;
		CH[ch+1].sign_pos      = CH[ch+1].ch_time;

		// Bits edges always multiples of 20ms
		if ((CH[ch+1].sign_pos - CH[ch+1].prev_sign_pos) > 19)
			CH[ch+1].sign_count++;
		else
			CH[ch+1].sign_count = 0;
	}
        CH[ch+1].ch_time++;



    if ((CH[ch].sign_count > 30) || (CH[ch+1].sign_count > 10) ) {
        CH[ch].state = CHANNEL_LOCK;
        CH[ch].ms_count = 0;
        CH[ch].bit_sync = 1;
        CH[ch].load_1ms_epoch_count = 1;
        data_bit_coherency_init(ch);
        CH[ch].phase_info_old = CH[ch].phase_info;
        CH[ch+1].phase_info_old = CH[ch+1].phase_info;
        CH[ch].ch_debug=66;
        CH[ch+1].ch_debug=66;
        CH[ch].ch_debug2=0;
    }

    if (CH[ch].ch_time >= 3500) {
        CH[ch].state    = CHANNEL_ACQUISITION;
        ch_block->channels[ch].code_slew = 2;
    	CH[ch].codes += 2;
    	d_pull_in_monitor[CH[ch].prn]++;
    	CH[ch].code_freq = CODE_REF;
    	ch_block->channels[ch].code_nco = CH[ch].code_freq;
    }
}

/*******************************************************************************
 FUNCTION lock( unsigned long ch)
 RETURNS  None.

 PARAMETERS  char ch  , channel number

 PURPOSE track carrier and code, and partially decode the navigation message
 (to determine TOW, subframe etc.)

 WRITTEN BY
 Clifford Kelley
 added Carrier Aiding as suggested by Jenna Cheng, UCR
*******************************************************************************/
static void lock( unsigned long ch)
{

	unsigned int current_ch;

	int i;

    long cross, dot;
    CH[ch].ms_count++;

    if( CH[ch].ms_count > 19) // Efficient modulo 20
        CH[ch].ms_count = 0; 

    CH[ch].i_early_20  += CH[ch].i_early;
    CH[ch].q_early_20  += CH[ch].q_early;
    CH[ch].i_prompt_20 += CH[ch].i_prompt;
    CH[ch].q_prompt_20 += CH[ch].q_prompt;
    CH[ch].i_late_20   += CH[ch].i_late;
    CH[ch].q_late_20   += CH[ch].q_late;

    if ((CH[ch].i_prompt != 0) && (CH[ch].q_prompt != 0) && (CH[ch].i_prompt_old != 0) && (CH[ch].q_prompt_old != 0)) {
           cross = CH[ch].i_prompt_old * CH[ch].q_prompt-CH[ch].i_prompt * CH[ch].q_prompt_old;
           dot   = labs(CH[ch].i_prompt * CH[ch].i_prompt_old + CH[ch].q_prompt * CH[ch].q_prompt_old);
           cross = cross >> 14;
           dot   = dot   >> 14;
           CH[ch].delta_carrier_freq  = fix_atan2(cross, dot) ;
           CH[ch].delta_carrier_phase = fix_atan2((CH[ch].q_prompt * sign(CH[ch].i_prompt)), labs(CH[ch].i_prompt));
    }

    /*double delta_freq = ((double)264.04465*(double)CH[ch].delta_carrier_phase-517.87888*(double)CH[ch].delta_carrier_phase_old+253.88705*(double)CH[ch].delta_carrier_phase_old_old)/16384.0;
    CH[ch].carrier_freq = 2 * CH[ch].carrier_freq_old - CH[ch].carrier_freq_old_old + ((long)delta_freq);*/

    CH[ch].carrier_freq += ((273 * CH[ch].delta_carrier_phase - 263 * CH[ch].delta_carrier_phase_old) >> 14);

    CH[ch].delta_carrier_phase_old_old = CH[ch].delta_carrier_phase_old;
	CH[ch].delta_carrier_phase_old = CH[ch].delta_carrier_phase;
	CH[ch].delta_carrier_freq_old=CH[ch].delta_carrier_freq;
	CH[ch].carrier_freq_old_old = CH[ch].carrier_freq_old;
	CH[ch].carrier_freq_old = CH[ch].carrier_freq;



	CH[ch+1].i_early_20  += CH[ch+1].i_early;
	CH[ch+1].q_early_20  += CH[ch+1].q_early;
	CH[ch+1].i_prompt_20 += CH[ch+1].i_prompt;
	CH[ch+1].q_prompt_20 += CH[ch+1].q_prompt;
	CH[ch+1].i_late_20   += CH[ch+1].i_late;
	CH[ch+1].q_late_20   += CH[ch+1].q_late;

	if ((CH[ch+1].i_prompt != 0) && (CH[ch+1].q_prompt != 0) && (CH[ch+1].i_prompt_old != 0) && (CH[ch+1].q_prompt_old != 0)) {
		   cross = CH[ch+1].i_prompt_old * CH[ch+1].q_prompt-CH[ch+1].i_prompt * CH[ch+1].q_prompt_old;
		   dot   = labs(CH[ch+1].i_prompt * CH[ch+1].i_prompt_old + CH[ch+1].q_prompt * CH[ch+1].q_prompt_old);
		   cross = cross >> 14;
		   dot   = dot   >> 14;
		   CH[ch+1].delta_carrier_freq  = fix_atan2(cross, dot) ;
		   CH[ch+1].delta_carrier_phase = fix_atan2((CH[ch+1].q_prompt * sign(CH[ch+1].i_prompt)), labs(CH[ch+1].i_prompt));
	}

	CH[ch+1].carrier_freq += ((273 * CH[ch+1].delta_carrier_phase - 263 * CH[ch+1].delta_carrier_phase_old) >> 14);

	CH[ch+1].delta_carrier_phase_old_old = CH[ch+1].delta_carrier_phase_old;
	CH[ch+1].delta_carrier_phase_old = CH[ch+1].delta_carrier_phase;
	CH[ch+1].delta_carrier_freq_old=CH[ch+1].delta_carrier_freq;
	CH[ch+1].carrier_freq_old_old = CH[ch+1].carrier_freq_old;
	CH[ch+1].carrier_freq_old = CH[ch+1].carrier_freq;

	for (i = ch; i < ch + 2; i++) {
		if (abs(CH[i].i_prompt) < LOCK_THRESHOLD) {
			CH[i].no_view++;
		} else {
			CH[i].no_view = 0;
		}
	}

	if ((CH[ch].no_view) > 50 && (CH[ch+1].no_view == 0)) {
		CH[ch].carrier_freq = CH[ch+1].carrier_freq;
	} else if ((CH[ch+1].no_view) > 50 && (CH[ch].no_view == 0)) {
		CH[ch+1].carrier_freq = CH[ch].carrier_freq;
	}
	ch_block->channels[ch].carr_nco = CH[ch].carrier_freq;
	ch_block->channels[ch+1].carr_nco = CH[ch+1].carrier_freq;

	if (abs(CH[ch].i_prompt) > abs(CH[ch+1].i_prompt)) {
		CH[ch].doppler_freq += (CARRIER_REF - CH[ch].carrier_freq);
	}else {
		CH[ch].doppler_freq += (CARRIER_REF - CH[ch+1].carrier_freq);
	}


    if (CH[ch].ms_count == 19) {
        /* Work on sum of last 20ms of data */
        CH[ch].early_mag  = lmag( CH[ch].i_early_20, CH[ch].q_early_20); 
        CH[ch].prompt_mag = lmag( CH[ch].i_prompt_20, CH[ch].q_prompt_20);
        CH[ch].late_mag   = lmag( CH[ch].i_late_20, CH[ch].q_late_20);

        if (CH[ch].early_mag | CH[ch].late_mag) {
        	CH[ch].delta_code_phase = (CH[ch].early_mag - CH[ch].late_mag) << 14;
        	CH[ch].delta_code_phase = (CH[ch].delta_code_phase / (CH[ch].early_mag + CH[ch].late_mag)) >> 1;
        }

        CH[ch+1].early_mag  = lmag( CH[ch+1].i_early_20, CH[ch+1].q_early_20);
		CH[ch+1].prompt_mag = lmag( CH[ch+1].i_prompt_20, CH[ch+1].q_prompt_20);
		CH[ch+1].late_mag   = lmag( CH[ch+1].i_late_20, CH[ch+1].q_late_20);

		if (CH[ch+1].early_mag | CH[ch+1].late_mag) {
			CH[ch+1].delta_code_phase = (CH[ch+1].early_mag - CH[ch+1].late_mag) << 14;
			CH[ch+1].delta_code_phase = (CH[ch+1].delta_code_phase / (CH[ch+1].early_mag + CH[ch+1].late_mag)) >> 1;
		}

		if(abs(CH[ch].i_prompt_20) > abs(CH[ch+1].i_prompt_20)) {
			current_ch = ch;
			CH[ch].sum += CH[ch].prompt_mag;
		} else {
			current_ch = ch + 1;
			CH[ch].sum += CH[ch+1].prompt_mag;
		}
		CH[ch].ch_debug=current_ch - ch+1;
		CH[ch+1].ch_debug=66;


        //CH[ch].code_freq += ((114 * CH[ch].delta_code_phase - 108 * CH[ch].delta_code_phase_old) >> 14);
        CH[ch].code_freq += ((284 * CH[current_ch].delta_code_phase - 251 * CH[current_ch].delta_code_phase_old) >> 14);

        CH[ch].delta_code_phase_old = CH[ch].delta_code_phase;
        CH[ch+1].delta_code_phase_old = CH[ch+1].delta_code_phase;

        ch_block->channels[ch].code_nco = CH[ch].code_freq;

        /* Data bit */
        data_bit_coherency(ch);

        if ((CH[ch].phase_info == 0) && (CH[ch+1].phase_info == 0))
        {
        	CH[ch].ch_debug2=9;
        	if (CH[current_ch].phase_info_old == 0)
        		CH[current_ch].phase_info = 1;

        	else
        		CH[current_ch].phase_info = CH[current_ch].phase_info_old;
        }
        CH[ch].phase_info_old = CH[ch].phase_info;
        CH[ch+1].phase_info_old = CH[ch+1].phase_info;

        CH[ch].bit = ((CH[current_ch].i_prompt_20 * CH[current_ch].phase_info) > 0);


        /*
         * Flag that this bit is ready to process (written to the message_flag
         * in the tracking() function after we've gone through all the channels
         */
        channels_with_bits |= (1 << ch);

        /*
         * Increment the time, in bits, since the week began. Used in
         * the measurement thread. Also set to the true time of
         * week when we get the TOW from a valid subframe in the
         * messages thread.
		 */
        CH[ch].time_in_bits++;
        if(CH[ch].time_in_bits >= BITS_IN_WEEK)
            CH[ch].time_in_bits -= BITS_IN_WEEK;

        CH[ch].check_average++;
        if (CH[ch].check_average > 4) {
            CH[ch].check_average = 0;
            CH[ch].avg = CH[ch].sum / 5;
            CH[ch].sum = 0;

            if((CH[ch].bit_sync) && (CH[ch].avg < LOCK_THRESHOLD)) {
                /* Signal loss. Clear channel. */
				clear_messages(ch);
				CH[ch].state                   = CHANNEL_PULL_IN;
			    CH[ch].ch_time                 = 0;
			    CH[ch].sum                     = 0;
			    CH[ch].th_rms                  = 0;
			    CH[ch].bit_sync                = 0;
			    CH[ch].delta_code_phase_old    = 0;
			    CH[ch].delta_carrier_phase_old = 0;
			    CH[ch].delta_carrier_phase_old_old = 0;
			    CH[ch].delta_carrier_freq_old = 0;
		        CH[ch].delta_carrier_freq  =0 ;
		        CH[ch].delta_carrier_phase = 0 ;
			    CH[ch].old_theta               = 0;
			    CH[ch].ms_sign                 = 0x12345; /* Some garbage data */
			    CH[ch].ms_count                = 0;
			    CH[ch].sign_count = 0;
			    CH[ch+1].sign_count = 0;
			    CH[ch].code_freq = CODE_REF;
            }
        }
		CH[ch].i_early_20  = 0;
		CH[ch].q_early_20  = 0;
		CH[ch].i_prompt_20 = 0;
		CH[ch].q_prompt_20 = 0;
		CH[ch].i_late_20   = 0;
		CH[ch].q_late_20   = 0;

		CH[ch+1].i_early_20  = 0;
		CH[ch+1].q_early_20  = 0;
		CH[ch+1].i_prompt_20 = 0;
		CH[ch+1].q_prompt_20 = 0;
		CH[ch+1].i_late_20   = 0;
		CH[ch+1].q_late_20   = 0;
    }
}
/*******************************************************************************
 FUNCTION tracking( void)
 RETURNS  None.

 PARAMETERS  None

 PURPOSE Main routine which runs on an accum_int.

 WRITTEN BY
 Clifford Kelley
 added Carrier Aiding as suggested by Jenna Cheng, UCR
*******************************************************************************/
void tracking(void)
{
    unsigned int   to_allocate;
    unsigned short ch;
    unsigned int   status;
    unsigned int   new_data;

	status     = status_block->status;
	new_data   = status_block->new_data;
    
    for (ch = 0; ch < N_CHANNELS; ch++) {
    	if((new_data & (1 << ch)) || (new_data & (1 << (ch-1))))
    	        {
    	            CH[ch].i_prompt_old = CH[ch].i_prompt;
    	            CH[ch].q_prompt_old = CH[ch].q_prompt;

    	            /* Collect channel data accumulation. */
    	            CH[ch].i_early  = signExtension(ch_block->channels[ch].i_early);
    	            CH[ch].q_early  = signExtension(ch_block->channels[ch].q_early);
    	            CH[ch].i_prompt = signExtension(ch_block->channels[ch].i_prompt);
    	            CH[ch].q_prompt = signExtension(ch_block->channels[ch].q_prompt);
    	            CH[ch].i_late   = signExtension(ch_block->channels[ch].i_late);
    	            CH[ch].q_late   = signExtension(ch_block->channels[ch].q_late);

    	            // If the last dump was the first dump in a new satellite
    	            // message data bit, then lock() sets the load_1ms_epoch_flag
    	            // so that we can set the 1m epoch counter here. Why here?
    	            // GP4020 Baseband Processor Design Manual, pg 60: "Ideally,
    	            // epoch counter accesses should occur following the reading of
    	            // the accumulation register at each DUMP." Great, thanks for
    	            // the tip, now how 'bout you tell us WHY?!
    	            if(CH[ch].load_1ms_epoch_count)
    	            {
    	            	ch_block->channels[ch].epoch_load = 1;
    	                CH[ch].load_1ms_epoch_count = 0;
    	            }

    	            // We expect the 1ms epoch counter to always stay sync'd until
    	            // we lose lock. To sync the 20ms epoch counter (the upper bits)
    	            // we wait until we get a signal from the message thread that
    	            // we just got the TLM+HOW words; this means we're 60 bits into
    	            // the message. Since the damn epoch counter counts to *50* (?!)
    	            // we mod it with 60 which gives us 10 (0xA00 when shifted 8).
    	            if(CH[ch].sync_20ms_epoch_count){
    	            	ch_block->channels[ch].epoch_load =
    	            		(ch_block->channels[ch].epoch_check & 0x1f) | 0x140;
    	                CH[ch].sync_20ms_epoch_count = 0;
    	            }


    	        }
    }
    to_allocate = 0;

    for(ch = 0; ch < N_CHANNELS; ch += 2){
            if((new_data & (1 << ch)) && (CH[ch].state != CHANNEL_OFF))
            {
                switch( CH[ch].state)
                {
                    case CHANNEL_ACQUISITION:
                        acquire(ch);
                        break;
                    case CHANNEL_CONFIRM:
                        confirm(ch);
                        break;
                    case CHANNEL_PULL_IN:
                        pull_in(ch);
                        break;
                    case CHANNEL_LOCK:
                        lock(ch);
                        break;
                    default:
                        CH[ch].state = CHANNEL_OFF;
                        // TODO: assert an error here
                        break;
                }
                d_log_data(ch);
            }
            // If the channel is off, set a flag saying so
            if(CH[ch].state == CHANNEL_OFF)
                to_allocate |= (1 << ch);
        }
    if(to_allocate)
        osSignalSet(allocate_thread_id, 0x0001);
    if(channels_with_bits)
        osSignalSet(message_thread_id,  0x0002);
    if(status & 0x01) {
    	osSignalSet(measure_thread_id,  0x0003);
    	osSignalSet(display_thread_id,  0x0004);
    	osSignalSet(uart_thread_id	 ,  0x0006);
    }

}

void d_log_data(int ch) {
    /*
        Condition for logging data is defined by ?
        This first edition will log every data for one single channel (0) w/o condition
    */
    if(ch == 2 && d_data_index < D_DATA_STORED - 1) {
    	d_data[d_data_index].state = CH[ch].state;
    	d_data[d_data_index].prn = CH[ch].prn;
    	d_data[d_data_index].i_early = CH[ch+1].i_prompt;
    	d_data[d_data_index].q_early = CH[ch+1].q_prompt;
    	d_data[d_data_index].i_prompt = CH[ch].i_prompt;
    	d_data[d_data_index].q_prompt = CH[ch].q_prompt;
    	d_data[d_data_index].i_late = CH[ch].phase_info;
    	d_data[d_data_index].q_late = CH[ch+1].phase_info;
    	d_data[d_data_index].carrier_freq = CH[ch].carrier_freq;
    	d_data[d_data_index].code_freq = CH[ch].i_prompt_20;
    	d_data[d_data_index].del_carr_phase = CH[ch+1].i_prompt_20;
        d_data_index++;
    }

    if(d_data_index == D_DATA_STORED) {
        for(;;)
            ;
    }
}

static void data_bit_coherency(unsigned short ch)
{
    unsigned short i;

    for (i = ch; i < ch + 2; i++) {
        if ((abs(CH[i].i_prompt_20) < (10 * LOCK_THRESHOLD)) && (abs(CH[i].i_prompt) < LOCK_THRESHOLD))
            CH[i].phase_info = 0;
    }

    /* Dual antenna data coherency mode */
    //if ((abs(CH[ch].i_prompt) > LOCK_THRESHOLD) && (abs(CH[ch+1].i_prompt) > LOCK_THRESHOLD)) {
    if ((abs(CH[ch].i_prompt_20) > (10 * LOCK_THRESHOLD)) && (abs(CH[ch+1].i_prompt_20) > (10 * LOCK_THRESHOLD))) {
        if ((CH[ch].phase_info == 0) && (CH[ch+1].phase_info != 0)) {
            if (sgn(CH[ch].i_prompt_20) == sgn(CH[ch+1].i_prompt_20))
                CH[ch].phase_info = CH[ch+1].phase_info;
            else
                CH[ch].phase_info = CH[ch+1].phase_info * -1;
        } else if ((CH[ch+1].phase_info == 0) && (CH[ch].phase_info != 0)) {
            if (sgn(CH[ch].i_prompt_20) == sgn(CH[ch+1].i_prompt_20))
                CH[ch+1].phase_info = CH[ch].phase_info;
            else
                CH[ch+1].phase_info = CH[ch].phase_info * -1;
        }
    }
}

/*
 * Set channel 0 phase_info to 0 and channel 1 phase_info to -1
 */
static void data_bit_coherency_init(unsigned short ch)
{
	if ((CH[ch].sign_count > 30) && (CH[ch+1].sign_count > 20))
	{
		CH[ch].phase_info = 1;
		if (sgn(CH[ch].i_prompt) == sgn(CH[ch+1].i_prompt))
			CH[ch+1].phase_info = CH[ch].phase_info;
		else
			CH[ch+1].phase_info = CH[ch].phase_info * -1;
	}
	else if((CH[ch].sign_count > 30))
	{
		CH[ch].phase_info = 1;
		CH[ch+1].phase_info = 0;
	}
	else if((CH[ch+1].sign_count > 30))
	{
		CH[ch].phase_info = 0;
		CH[ch+1].phase_info = 1;
	}

}

