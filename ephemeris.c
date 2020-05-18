// ephemeris.c gpl-gps Satellite navigation message to ephemeris processing
// Copyright (C) 2005  Andrew Greenberg
// Distributed under the GNU GENERAL PUBLIC LICENSE (GPL) Version 2 (June 1991).
// See the "COPYING" file distributed with this software for more information.
#include "cmsis_os.h"
//#include <cyg/kernel/kapi.h>
//#include <cyg/infra/diag.h>
#include <math.h>
#include "constants.h"
#include "ephemeris.h"
//#include "gp4020.h"
//#include "gp4020_io.h"
#include "message.h"
//#include "switches.h"
#include "time.h"

/******************************************************************************
 * Defines
 ******************************************************************************/

// c_2pX == 2^+X, c_2mX == 2^-X
#define c_2p4     16.0
#define c_2m5     0.03125
#define c_2m11    4.8828125e-4
#define c_2m19    1.9073486328125e-6
#define c_2m20    9.5367431640625e-7
#define c_2m21    4.76837158203125e-7
#define c_2m23    1.19209289550781e-7   // NaN
#define c_2m24    5.96046447753906e-8   // Nan
#define c_2m27    7.45058059692383e-9
#define c_2m29    1.86264514923096e-9
#define c_2m30    9.31322574615479e-10
#define c_2m31    4.65661287307739e-10  // NaN
#define c_2m33    1.16415321826935E-10
#define c_2m38    3.63797880709171e-12  // NaN
#define c_2m43    1.13686837721616e-13  // NaN
#define c_2m50    8.881784197e-16       // NaN
#define c_2m55    2.77555756156289e-17  // NaN

/******************************************************************************
 * Globals
 ******************************************************************************/

// Right now we're declaring a message structure per channel, since the 
// messages come out of locked channels_ready.. but you could argue they should
// be in a per-satellite array.

ephemeris_t ephemeris[N_CHANNELS];

unsigned char log_eph[N_CHANNELS];

//cyg_flag_t  ephemeris_channel_flag;
//cyg_flag_t  ephemeris_subframe_flags[N_CHANNELS];

/******************************************************************************
 * Statics
 ******************************************************************************/

// None


/******************************************************************************
 * If the channel is reallocated, then clear the ephemeris data for that
 * channel. Called from allocate.c
 ******************************************************************************/

void
clear_ephemeris( unsigned short ch)
{
    ephemeris[ch].valid = 0;
    ephemeris[ch].have_subframe = 0;
    ephemeris[ch].prn = 0;
}

   
/******************************************************************************
 * Convert subframe bits to ephemeris values.
 *
 * Note that subframes aren't passed up from the message_thread unless they're
 * already considered valid (all parity checks have passed and been removed)
 ******************************************************************************/
void 
process_subframe1( unsigned short ch)
{
    signed   long temp;
    unsigned long utemp;
   
    // map the messages structure to OSGPS's "sf" and make this whole thing
    // a bit more readable.  --more copying :(
    subframe_t * sf = messages[ch].subframes; 
        
    // Calculate the `Issue Of Data Clock' (IODC)
    utemp = ((sf[1-1].word[3-1] & 0x3) << 8 ) 
            | (sf[1-1].word[8-1] >> 16);
    
    // Skedaddle if we already have a valid ephemeris and we have this subframe,
    // and the IODC hasn't changed.
    if( (ephemeris[ch].valid) 
         && (ephemeris[ch].have_subframe & (1 << 0))
         && (ephemeris[ch].iodc == (unsigned short)utemp))
    {
        return;
    }
        
    ephemeris[ch].iodc = (unsigned short)utemp;
            
    ephemeris[ch].ura = (unsigned short)((sf[1-1].word[3-1] & 0xF00) >> 8);
    ephemeris[ch].health = (unsigned short)((sf[1-1].word[3-1] & 0xFC) >> 2);

    // According to ICD-GPDS-200C sect. 20.3.3.3.1.4, if the MSB of the 6 bit
    // health is set, the satellite's nav message is toast. Bad satellite!
    if( ephemeris[ch].health & (1 << 5))
    {
        clear_ephemeris( ch);
        return;
    }
    
    // Grab the PRN for good measure
    ephemeris[ch].prn = messages[ch].prn;
    
    // If we haven't already, then update the time with the week number in
    // this subframe. Note that we have NO stinking clue what the true
    // year is because the week is modulo 1024 which is about 20 years.
    // So we'll just guess it's past 2000 :) and before ~ 2020 which
    // means adding 1024 to the current week.
    //utemp = (sf[1-1].word[3-1] >> 14) + 1024;
    // From 2019/04/07 needs to add 2048 for the current week;
    utemp = (sf[1-1].word[3-1] >> 14) + 2048;
    set_time_with_weeks( (unsigned short)utemp);
    
    // Get the rest of the ephemerides         
    utemp = sf[1-1].word[8-1] & 0xffff;
    ephemeris[ch].toc = (double)utemp * c_2p4;

    // The following variables are signed integers so if the MSB is set,
    // 'deal 
    // with the sign. Standard sign extending |= 0xFFFFFF00 wasn't
    // working for whatever reason?! Yes, this sucks but the if makes it
    // faster than another multiply, even by -1.
    // TODO try (-((1<<n) - x)) for (x) of (n)bits.
    temp = sf[1-1].word[7-1] & 0xff;
    if( temp & (1 << 7))
        temp |= ~0xff;
    ephemeris[ch].tgd = (double)temp * c_2m31;
            
    temp = sf[1-1].word[9-1] >> 16;
    if( temp & (1 << 7))
        temp |= ~0xff;
        ephemeris[ch].af2 = (double)temp * c_2m55;
    
    temp = sf[1-1].word[9-1] & 0xffff;
    if( temp & (1 << 15))
        temp |= ~0xFFFF;
        ephemeris[ch].af1 = (double)temp * c_2m43;
            
    temp = sf[1-1].word[10-1] >> 2;
    if( temp & (1 << 21))
        temp |= ~0x3FFFFF;
    ephemeris[ch].af0 = (double)temp * c_2m31;
    
    // Got subframe 1
    ephemeris[ch].have_subframe |= (1 << 0);

    // If the switch is on, log the SF1 ephemeris to the serial port
//#ifdef ENABLE_EPHEMERIS_LOG
//    log_eph[ch] |= (1 << 0);
//#endif
}
        

void 
process_subframe2( unsigned short ch)
{
    unsigned short  short_temp;
    unsigned long   ultemp;
    long            temp;
    
    // map the messages structure to OSGPS's "sf"
    subframe_t* sf = messages[ch].subframes;
    
    short_temp = (unsigned short)(sf[2-1].word[3-1] >> 16);
    
    // Skedaddle if we already have a valid ephemeris, and we have this
    // subframe, and the `Issue Of Data Ephemeris' (IODE) hasn't changed.
    if( (ephemeris[ch].valid) 
         && (ephemeris[ch].have_subframe & (1 << 1))
         && (ephemeris[ch].iode == short_temp))
    {
        return;
    }

    // Some of these data words are signed; check their sign bit and extend
    // as appropriate
    
    ephemeris[ch].iode = (unsigned short)short_temp;

    // Grab the PRN for good measure    
    ephemeris[ch].prn = messages[ch].prn;
    
    temp = sf[2-1].word[3-1] & 0xFFFF;
    if( temp & (1 << 15))
        temp |= ~0xFFFF;
    ephemeris[ch].crs = (double)temp * c_2m5;
    
    temp = sf[2-1].word[4-1] >> 8;
    if( temp & (1 << 15))
        temp |= ~0xFFFF;
    ephemeris[ch].dn = (double)temp * (c_2m43 * PI);

    temp = ((sf[2-1].word[4-1] & 0xFF) << 24) | sf[2-1].word[5-1];
    ephemeris[ch].ma = (double)temp * (c_2m31 * PI);

    temp = sf[2-1].word[6-1] >> 8;
    if( temp & (1 << 15))
        temp |= ~0xFFFF;
    ephemeris[ch].cuc = (double)temp * c_2m29;

    temp = ((sf[2-1].word[6-1] & 0xFF) << 24) | sf[2-1].word[7-1];
    ephemeris[ch].ety = (double)temp * c_2m33;
    
    temp = sf[2-1].word[8-1] >> 8;
    if( temp & (1 << 15))
        temp |= ~0xFFFF;
    ephemeris[ch].cus = (double)temp * c_2m29;

    ultemp = (((sf[2-1].word[8-1] & 0xFF) << 24) | sf[2-1].word[9-1]);
    ephemeris[ch].sqra = (double)ultemp * c_2m19;
    
    ultemp = (sf[2-1].word[10-1] >> 8);
    ephemeris[ch].toe = (double)ultemp * c_2p4;
    
    // Got subframe 2
    ephemeris[ch].have_subframe |= (1 << 1);

    // If the switch is on, log the SF2 ephemeris to the serial port
//#ifdef ENABLE_EPHEMERIS_LOG
//    log_eph[ch] |= (1 << 1);
//#endif

}
        
            
void 
process_subframe3( unsigned short ch)
{
    long temp;
    unsigned short short_temp;
    // map the messages structure to OSGPS's "sf"
    subframe_t* sf = messages[ch].subframes;
    
    short_temp  =  (unsigned short)(sf[3-1].word[10-1] >> 16);

    // Skedaddle if we already have a valid ephemeris, we have this subframe,
    // and the IODE hasn't changed.
    
    if( (ephemeris[ch].valid) 
         && (ephemeris[ch].have_subframe & (1 << 3))
         && (ephemeris[ch].iode == short_temp))
    {
        return;
    }

    // Grab the PRN for good measure    
    ephemeris[ch].prn = messages[ch].prn;

    // Some of these data words are signed; check their sign bit and extend
    // as appropriate
            
    temp = sf[3-1].word[3-1] >> 8;
    if( temp & (1 << 15))
        temp |=  ~0xFFFF;
    ephemeris[ch].cic = (double)temp * c_2m29;
    
    temp = ((sf[3-1].word[3-1] & 0xFF) << 24) | sf[3-1].word[4-1];
    ephemeris[ch].w0 = (double)temp * (c_2m31 * PI);

    temp = sf[3-1].word[5-1] >> 8;
    if( temp & (1 << 15))
        temp |=  ~0xFFFF;
    ephemeris[ch].cis = (double)temp * c_2m29;
    
    temp = ((sf[3-1].word[5-1] & 0xFF) << 24) | sf[3-1].word[6-1];
    ephemeris[ch].inc0 = (double)temp * (c_2m31 * PI);
    
    temp = sf[3-1].word[7-1] >> 8;
    if( temp & (1 << 15))
        temp |=  ~0xFFFF;
    ephemeris[ch].crc = (double)temp * c_2m5;
    
    temp = ((sf[3-1].word[7-1] & 0xFF) << 24) | sf[3-1].word[8-1];
    ephemeris[ch].w = (double)temp * (c_2m31 * PI);
    
    temp = sf[3-1].word[9-1];
    if( temp & (1 << 23))
        temp |=  ~0xFFFFFF;
    ephemeris[ch].omegadot = (double)temp * (c_2m43 * PI);
    
    temp = (sf[3-1].word[10-1] >> 2) & 0x3FFF;
    if( temp & (1 << 13))
        temp |=  ~0x3FFF;
    ephemeris[ch].idot = (double)temp * (c_2m43 * PI);

    // Got subframe 3
    ephemeris[ch].have_subframe |=  (1 << 2);

    // If the switch is on, log the SF3 ephemeris to the serial port
//#ifdef ENABLE_EPHEMERIS_LOG
//    log_eph[ch] |= (1 << 2);
//#endif
}

void 
process_subframe4( unsigned short ch)
{
    // Sure, you got subframe 4, why not?
    ephemeris[ch].have_subframe |=  (1 << 3);

    // If the switch is on, log the SF4 ephemeris to the serial port
//#ifdef ENABLE_EPHEMERIS_LOG
//     log_eph[ch] |= (1 << 3);
//#endif

}
    
void 
process_subframe5( unsigned short ch)
{
    // Sure, you got subframe 5, why not?
    ephemeris[ch].have_subframe |=  (1 << 4);

    // If the switch is on, log the SF5 ephemeris to the serial port
//#ifdef ENABLE_EPHEMERIS_LOG
//     log_eph[ch] |= (1 << 4);
//#endif

}

/******************************************************************************
 * Stuff incoming bits from the tracking interrupt into words and subframes in
 * the messages structure.
 ******************************************************************************/
