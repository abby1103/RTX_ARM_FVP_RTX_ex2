// display.c gpl-gps display output
// Copyright (C) 2005  Andrew Greenberg
// Distributed under the GNU GENERAL PUBLIC LICENSE (GPL) Version 2 (June 1991).
// See the "COPYING" file distributed with this software for more information.
#include "cmsis_os.h"
//#include <cyg/kernel/kapi.h>
//#include <cyg/infra/diag.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "display.h"
#include "constants.h"
#include "ephemeris.h"
#include "message.h"
#include "position.h"
#include "pseudorange.h"
#include "serial.h"
//#include "switches.h"
//#include "threads.h"
#include "time.h"
#include "tracking.h"
//#include "namuru.h"


unsigned short display_command = DISPLAY_POSITION;
static unsigned short ephemeris_mode;


/******************************************************************************
 * Send out the VT100 series clear-screen command
 ******************************************************************************/
static void
clear_screen( void)
{
    char clear_screen[] = "\033[2J";

    SER_PutString( clear_screen);
}

/******************************************************************************
 * Display position/clock info
 ******************************************************************************/
static void
display_position( void)
{
    static unsigned short  was_positioning;

    char string[120];
    char header[] = 
    "\033[32mCh: PN C PrV EpV   Pseudorange\tDelta Pseudorange\tDelta Pseudorange Test\n\r\033[0m";
// Ch: PN C PrV EpV U pseudorange   El. Az. ----x---- ----y---- ----z----\n\r";

    time_t          std_time;
    gpstime_t       gps_time;
    unsigned short  clock_state;
    double          lat, lon;
    double          az, el;

    unsigned short  ch;
    unsigned short  channel_state;
//     char            in_use;

    gps_time = get_time();
    std_time = get_standard_time();
    clock_state = get_clock_state();

    lat = receiver_llh.lat * RADTODEG;
    lon = receiver_llh.lon * RADTODEG;

    // Print the Clock/Time info first
    sprintf( string,
             "\033[HTime = %d/%d/%d %d:%d:%2.3f (state:%d)\033[K\n\r\n\r",
             std_time.years,
             std_time.months,
             std_time.days,
             std_time.hours,
             std_time.minutes,
             std_time.seconds,
             clock_state);
    SER_PutString( string);

    // Print the ECEF position info (even if it hasn't been set yet!)
    sprintf( string,
             "ECEF = (X:%e Y:%e Z:%e) tb:%1.3e\033[K\n\r\n\r",
             receiver_pvt.x,
             receiver_pvt.y,
             receiver_pvt.z,
             receiver_pvt.b);
    SER_PutString( string);

    // Print the LLH position info (even if it hasn't been set yet!)
    sprintf( string,
             "LLH  = (Lat:%2.5f Lon:%2.5f Hgt:%6.2f)\033[K\n\r\n\r",
             lat,
             lon,
             receiver_llh.hgt);
    SER_PutString( string);

    // Print out some position.c info
    sprintf( string, "\
State: positioning = %d, last position valid = %d\n\r\n\r",
             positioning, receiver_pvt.valid);
    SER_PutString( string);

    // beep the bell if we just got busy in position.
    if( positioning)
    {
        if( !was_positioning)
        {
            sprintf( string, "\007"); // bell code
            SER_PutString( string);
            was_positioning = 1;
        }
    }
    else
        was_positioning = 0;
    
    // Now put out a summary of what the hell is going on in the receiver
    SER_PutString( header);

    // Send out data on all 12 channels if there's no error
    for( ch = 0; ch < N_CHANNELS; ch++)
    {
        switch( CH[ch].state)
        {
            case CHANNEL_ACQUISITION:
                channel_state = 'A';
                break;
            case CHANNEL_CONFIRM:
                channel_state = 'C';
                break;
            case CHANNEL_PULL_IN:
                channel_state = 'P';
                break;
            case CHANNEL_LOCK:
                channel_state = 'L';
                break;
            default:
                channel_state = '-';
                break;
        }

        if( (pr[ch].valid) && ephemeris[ch].valid)
        {
            az = sat_azel[ch].az * RADTODEG;
            el = sat_azel[ch].el * RADTODEG;
                        
            sprintf( string,
//                   "%2d: %2d %c   %d   %d  % e %3.f %3.f %.3e %.3e %.3e\033[K\n\r",
                     "%2d: %2d %c   %d   %d  %e  %e  %e\033[K\n\r",
                     ch,
                     CH[ch].prn,
                     channel_state,
                     pr[ch].valid,
                     ephemeris[ch].valid,
                     pr[ch].range,
                     pr[ch].delta_range,
                     pr[ch].delta_range_test
            );
//                      sat_pos_by_ch[ch].x,
//                      sat_pos_by_ch[ch].y,
//                      sat_pos_by_ch[ch].z);
            SER_PutString( string);
        }
        else
        {
            sprintf( string, 
                     "%2d: %2d %c   %d   %d\033[K\n\r",
                     ch,
                     CH[ch].prn,
                     channel_state,
                     pr[ch].valid,
                     ephemeris[ch].valid);
            SER_PutString( string);
        }
    }
}
    

/******************************************************************************
 * Display pseudorange info
 ******************************************************************************/
static void
display_pseudorange( void)
{
    unsigned short ch;
    unsigned char channel_state;
    char header[] =
        "\033[32m\033[HCH: PN S bit%50 eb ems Pseudorange Average\n\r\033[0m";
    char string[80];

    SER_PutString(header);

    // Send out data on all 12 channels if there's no error
    for( ch = 0; ch < N_CHANNELS; ch++)
    {
        switch( CH[ch].state)
        {
            case CHANNEL_ACQUISITION:
                channel_state = 'A';
                break;
            case CHANNEL_CONFIRM:
                channel_state = 'C';
                break;
            case CHANNEL_PULL_IN:
                channel_state = 'P';
                break;
            case CHANNEL_LOCK:
                channel_state = 'L';
                break;
            default:
                channel_state = '-';
                break;
        }

		sprintf( string,
				"%2d: %2d %c [%e, %e, %e]\t [%e, %e, %e]\033[K\n\r",
				ch,
				pr[ch].prn,
				channel_state,
				sat_position[ch].x,
				sat_position[ch].y,
				sat_position[ch].z,
				sat_position[ch].vx,
				sat_position[ch].vy,
				sat_position[ch].vz
	   );
		SER_PutString( string);
    }
}


/******************************************************************************
 * Display ephemeris_thread info
 ******************************************************************************/
static void
display_ephemeris( void)
{
    unsigned short ch;
    char string[120];
    char header0[] = 
    "\033[32m\033[HCH: PN V SF UR HE IODC  ------tgd------ ------toc------\033[K\n\r\033[0m";
    char header1[] =
    "\033[32m\033[HCH: ------af2------ ------af1------ ------af0------\033[K\n\r\033[0m";
    char header2[] =
    "\033[32m\033[HCH: IODE ------Crs------ ------dn------- ------Mo------- ------Cuc------\033[K\n\r\033[0m";
    char header3[] = 
    "\033[32m\033[HCH: ------e-------- ------Cus------ ------sqA------ --toe--\033[K\n\r\033[0m";
    char header4[] = 
    "\033[32m\033[HCH: ------cic------ ------w0------- ------cis------ ------inc0-----\033[K\n\r\033[0m";
    char header5[] = 
    "\033[32m\033[HCH: ------crc------ ------w-------- ---omegadot---- ------idot-----\033[K\n\r\033[0m";

    if( ephemeris_mode == 0)
    {
        SER_PutString( header0);

        for( ch = 0; ch < N_CHANNELS; ch++)
        {
            if( ephemeris[ch].valid)
            {
                sprintf( string,
                         "%2d: %2d %d %2x %2d %2x %4d % 15e % 15e\033[K\n\r",
                         ch,
                         ephemeris[ch].prn,
                         ephemeris[ch].valid,
                         ephemeris[ch].have_subframe,
                         ephemeris[ch].ura,
                         ephemeris[ch].health,
                         ephemeris[ch].iodc,
                         ephemeris[ch].tgd,
                         ephemeris[ch].toc);
                SER_PutString( string);
            }
            else
            {
                sprintf( string,
                         "%2d: %2d %d %2x %2d %2x\033[K\n\r",
                         ch,
                         ephemeris[ch].prn,
                         ephemeris[ch].valid,
                         ephemeris[ch].have_subframe,
                         ephemeris[ch].ura,
                         ephemeris[ch].health);
                SER_PutString( string);
            }
        }
    }
    else if( ephemeris_mode == 1)
    {
        SER_PutString( header1);

        for( ch = 0; ch < N_CHANNELS; ch++)
        {
            if( ephemeris[ch].valid)
            {
                sprintf( string,
                         "%2d: % 15e % 15e % 15e\033[K\n\r",
                         ch,
                         ephemeris[ch].af2,
                         ephemeris[ch].af1,
                         ephemeris[ch].af0);
                SER_PutString( string);
            }
            else
            {
                sprintf( string, "%2d:\033[K\n\r", ch);
                SER_PutString( string);
            }
        }
    }
    else if( ephemeris_mode == 2)
    {
        SER_PutString( header2);

        for( ch = 0; ch < N_CHANNELS; ch++)
        {
            if( ephemeris[ch].valid)
            {
                sprintf( string,
                         "%2d: %4d % 15e % 15e % 15e % 15e\033[K\n\r",
                         ch,
                         ephemeris[ch].iode,
                         ephemeris[ch].crs,
                         ephemeris[ch].dn,
                         ephemeris[ch].ma,
                         ephemeris[ch].cuc);
                SER_PutString( string);
            }
            else
            {
                sprintf( string, "%2d:\033[K\n\r", ch);
                SER_PutString( string);
            }
        }
    }
    else if( ephemeris_mode == 3)
    {
        SER_PutString( header3);

        for( ch = 0; ch < N_CHANNELS; ch++)
        {
            if( ephemeris[ch].valid)
            {
                sprintf( string,
                         "%2d: % 15e % 15e % 15e %6.1f\033[K\n\r",
                         ch,
                         ephemeris[ch].ety,
                         ephemeris[ch].cus,
                         ephemeris[ch].sqra,
                         ephemeris[ch].toe);
                SER_PutString( string);
            }
            else
            {
                sprintf( string, "%2d:\033[K\n\r", ch);
                SER_PutString( string);
            }
        }
    }
    else if( ephemeris_mode == 4)
    {
        SER_PutString( header4);

        for( ch = 0; ch < N_CHANNELS; ch++)
        {
            if( ephemeris[ch].valid)
            {
                sprintf( string,
                         "%2d: % 15e % 15e % 15e % 15e\033[K\n\r",
                         ch,
                         ephemeris[ch].cic,
                         ephemeris[ch].w0,
                         ephemeris[ch].cis,
                         ephemeris[ch].inc0);
                SER_PutString( string);
            }
            else
            {
                sprintf( string, "%2d:\033[K\n\r", ch);
                SER_PutString( string);
            }
        }
    }
    else if( ephemeris_mode == 5)
    {
        SER_PutString( header5);

        for( ch = 0; ch < N_CHANNELS; ch++)
        {
            if( ephemeris[ch].valid)
            {
                sprintf( string,
                         "%2d: % 15e % 15e % 15e % 15e\033[K\n\r",
                         ch,
                         ephemeris[ch].crc,
                         ephemeris[ch].w,
                         ephemeris[ch].omegadot,
                         ephemeris[ch].idot);
                SER_PutString( string);
            }
            else
            {
                sprintf( string, "%2d:\033[K\n\r", ch);
                SER_PutString( string);
            }
        }
    }
    else
        ephemeris_mode = 0;
}

/******************************************************************************
 * Log the ephemeris from the satellites as it comes in. Called only when
 * ephemeris is processed, which is either right after a new lock or after
 * the IODE/IODC is updated.
 ******************************************************************************/

 // Display/log functions that can be turned on/off by switches.
void
log_ephemeris(unsigned short ch, unsigned short subframe)
{
    char string[200];

    switch (subframe)
    {
        case 0:
            sprintf( string,
                     "SF1,%d,%d,%d,%x,%d,%x,%d,%e,%e,%e,%e,%e",
                     ch,
                     ephemeris[ch].prn,
                     ephemeris[ch].valid,
                     ephemeris[ch].have_subframe,
                     ephemeris[ch].ura,
                     ephemeris[ch].health,
                     ephemeris[ch].iodc,
                     ephemeris[ch].tgd,
                     ephemeris[ch].toc,
                     ephemeris[ch].af2,
                     ephemeris[ch].af1,
                     ephemeris[ch].af0);
            SER_PutString( string);
            break;

        case 1:
            sprintf( string,
                     "SF2,%d,%d,%d,%x,%d,%e,%e,%e,%e,%e,%e,%e,%e",
                     ch,
                     ephemeris[ch].prn,
                     ephemeris[ch].valid,
                     ephemeris[ch].have_subframe,
                     ephemeris[ch].iode,
                     ephemeris[ch].crs,
                     ephemeris[ch].dn,
                     ephemeris[ch].ma,
                     ephemeris[ch].cuc,
                     ephemeris[ch].ety,
                     ephemeris[ch].cus,
                     ephemeris[ch].sqra,
                     ephemeris[ch].toe);
            SER_PutString( string);
            break;
    
        case 2:
            sprintf( string,
                     "SF3,%d,%d,%d,%x,%e,%e,%e,%e,%e,%e,%e,%e",
                     ch,
                     ephemeris[ch].prn,
                     ephemeris[ch].valid,
                     ephemeris[ch].have_subframe,
                     ephemeris[ch].cic,
                     ephemeris[ch].w0,
                     ephemeris[ch].cis,
                     ephemeris[ch].inc0,
                     ephemeris[ch].crc,
                     ephemeris[ch].w,
                     ephemeris[ch].omegadot,
                     ephemeris[ch].idot);
            SER_PutString( string);
            break;
    }
}
/******************************************************************************
 * Display tracking_thread info
 ******************************************************************************/
static void
display_tracking( void)
{
    char header[] =
        "\033[32m\033[HCh: PN  Iprmt  Qprmt State    Avg\n\r\033[0m";
    char string[80];

    unsigned short ch;
    unsigned short channel_state;
    unsigned short channel_bitsync;
    unsigned short channel_framesync;

    // display header line
    SER_PutString(header);

    // Send out data on all 12 channels if there's no error
    for( ch = 0; ch < N_CHANNELS; ch++)
    {
        switch( CH[ch].state)
        {
            case CHANNEL_ACQUISITION:
                channel_state = 'A';
                break;
            case CHANNEL_CONFIRM:
                channel_state = 'C';
                break;
            case CHANNEL_PULL_IN:
                channel_state = 'P';
                break;
            case CHANNEL_LOCK:
                channel_state = 'L';
                break;
            default:
                channel_state = '-';
                break;
        }

        // Is the channel bit sync'ed?
        if( CH[ch].bit_sync == 1)
            channel_bitsync = 'B';
        else
            channel_bitsync = '-';

        // Is the channel frame sync'ed?
        if(messages[ch].frame_sync == 1)
            channel_framesync = 'F';
        else
            channel_framesync = '-';

        sprintf(string,
            "%2d: %2d %6d %6d %c(%c%c) %6ld  %d  %d %d\n\r",
            ch,
            CH[ch].prn + 1,
            CH[ch].i_prompt,
            CH[ch].q_prompt,
            channel_state,
            channel_bitsync,
            channel_framesync,
            CH[ch].avg,
            CH[ch].carrier_freq,
            CH[ch].code_freq,
            CH[ch].delta_code_phase);

        SER_PutString(string);
    }
}




/******************************************************************************
 * Display debug info
 ******************************************************************************/
static void
display_debug( void)
{
#ifdef ENABLE_DEBUG_DISPLAY
    
    char string[80];

    sprintf(string, "\033[HAccumulator int.   = %4d\033[K\n\r\n\r", 
            cyg_thread_measure_stack_usage( accum_int_handle));
    SER_PutString( string);
    
    sprintf(string, "Allocate thread    = %4d\033[K\n\r", 
            cyg_thread_measure_stack_usage( allocate_thread_handle));
    SER_PutString( string);
    
    sprintf(string, "Display thread     = %4d\033[K\n\r", 
            cyg_thread_measure_stack_usage( display_thread_handle));
    SER_PutString( string);
    
    sprintf(string, "Ephemeris thread   = %4d\033[K\n\r", 
            cyg_thread_measure_stack_usage( ephemeris_thread_handle));
    SER_PutString( string);
    
    sprintf(string, "Measure thread     = %4d\033[K\n\r", 
            cyg_thread_measure_stack_usage( measure_thread_handle));
    SER_PutString( string);
    
    sprintf(string, "Message thread     = %4d\033[K\n\r", 
            cyg_thread_measure_stack_usage( message_thread_handle));
    SER_PutString( string);
        
    sprintf(string, "Position thread    = %4d\033[K\n\r", 
            cyg_thread_measure_stack_usage( position_thread_handle));
    SER_PutString( string);
    
    sprintf(string, "Pseudorange thread = %4d\033[K\n\r", 
            cyg_thread_measure_stack_usage( pseudorange_thread_handle));
    SER_PutString( string);
    
#endif // DEBUG_DISPLAY
}

        
/******************************************************************************
 * Display message_thread info
 ******************************************************************************/
static void
display_messages( void)
{
    char header[] = 
    	"\033[32m\033[HCh: PN Mi   TOW SF SF1V SF2V SF3V SF4V SF5V State   Avg\n\r\033[0m";
    char string[80];

    unsigned short ch;
    unsigned short channel_state;
    unsigned short channel_bitsync;
    unsigned short channel_framesync;
    unsigned short TOW;

    SER_PutString( header);

    // Send out data on all 12 channels
    for( ch = 0; ch < N_CHANNELS; ch++)
    {
        switch( CH[ch].state)
        {
            case CHANNEL_ACQUISITION:
                channel_state = 'A';
                break;
            case CHANNEL_CONFIRM:
                channel_state = 'C';
                break;
            case CHANNEL_PULL_IN:
                channel_state = 'P';
                break;
            case CHANNEL_LOCK:
                channel_state = 'L';
                break;
            default:
                channel_state = '-';
                break;
        }

        // Is the channel bit sync'ed?
        if( CH[ch].bit_sync == 1)
            channel_bitsync = 'B';
        else
            channel_bitsync = '-';

        // Is the channel frame sync'ed?
        if( messages[ch].frame_sync == 1)
            channel_framesync = 'F';
        else
            channel_framesync = '-';

        // Find the TOW for subframe 1 if valid
        if( messages[ch].subframes[0].valid)
            TOW = messages[ch].subframes[0].TOW;
        else
            TOW = 0;

        sprintf( string, 
        	"%2d: %2d %2d %5d %2d %4lx %4lx %4lx %4lx %4lx %c(%c%c) %5ld\n\r",
            ch,
            CH[ch].prn,
            CH[ch].missed_message_bit,
            TOW,
            messages[ch].subframe + 1,
            messages[ch].subframes[0].valid,
            messages[ch].subframes[1].valid,
            messages[ch].subframes[2].valid,
            messages[ch].subframes[3].valid,
            messages[ch].subframes[4].valid,
            channel_state,
            channel_bitsync,
            channel_framesync,
            CH[ch].avg);

        SER_PutString( string);
    }
}


/******************************************************************************
 * Display pages of GPL-GPS info on /dev/ser2.
 ******************************************************************************/
void
display_thread(void const *argument) // input 'data' not used
{
    unsigned short  current_display = DISPLAY_NOT_USED; // force clear screen
    int got_byte;
    static char c;

    //uart2_initialize();

    while (1)
    {
        // setbits32( GPS4020_GPIO_WRITE, 0x80);  // DEBUG: Set LED 5

		SER_GetChar(&c);

        if (c == 't')
            display_command = DISPLAY_TRACKING;
        else if (c == 'm')
            display_command = DISPLAY_MESSAGES;
        else if (c == 's')
            display_command = DISPLAY_STOP;
        else if (c == 'e')
            if (display_command == DISPLAY_EPHEMERIS)
                ephemeris_mode++;
            else
                display_command = DISPLAY_EPHEMERIS;
        else if (c == 'r')
            display_command = DISPLAY_PSEUDORANGE;
        else if (c == 'p')
            display_command = DISPLAY_POSITION;
        else if (c == 'd')
            display_command = DISPLAY_DEBUG;
        else if (c == 'l')
            display_command = DISPLAY_LOG;
       

		if( current_display != display_command)
        {
            current_display = display_command;
            clear_screen();
        }

        // Choose the page to display based on user input from the
        // input_thread
        if(display_command == DISPLAY_TRACKING)
            display_tracking();

        else if( display_command == DISPLAY_MESSAGES)
            display_messages();

        else if( display_command == DISPLAY_EPHEMERIS)
            display_ephemeris();

        else if( display_command == DISPLAY_PSEUDORANGE)
            display_pseudorange();

        else if( display_command == DISPLAY_POSITION)
            display_position();

        else if( display_command == DISPLAY_DEBUG)
            display_debug();

        osDelay(1000);
    }
}
