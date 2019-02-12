/* 
 * display.c gpl-gps display output
 * Copyright (C) 2005  Andrew Greenberg
 * Distributed under the GNU GENERAL PUBLIC LICENSE (GPL) Version 2 (June 1991).
 * See the "COPYING" file distributed with this software for more information.
 */
#include "cmsis_os.h"

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
#include "time.h"
#include "tracking.h"
#include "measure.h"
#include "ekf_position.h"

unsigned short display_command = DISPLAY_POSITION;
static unsigned short ephemeris_mode;

/******************************************************************************
 * Send out the VT100 series clear-screen command
 ******************************************************************************/
static void clear_screen( void)
{
    char clear_screen[] = "\033[2J";

    SER_PutString( clear_screen);
}

/******************************************************************************
 * Display position/clock info
 ******************************************************************************/
static void display_position( void)
{
    static unsigned short  was_positioning;

    char string[120];
    char header[] = 
    "\033[32mCh: PN C PrV EpV   Pseudorange\n\r\033[0m";

    time_t          std_time;
    gpstime_t       gps_time;
    unsigned short  clock_state;
    double          lat, lon;
    double          az, el;

    unsigned short  ch;
    unsigned short  channel_state;

    gps_time = get_time();
    std_time = get_standard_time();
    clock_state = get_clock_state();

    lat = receiver_llh.lat * RADTODEG;
    lon = receiver_llh.lon * RADTODEG;

    /* Print the Clock/Time info first */
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

    /* Print the ECEF position info (even if it hasn't been set yet!) */
    sprintf( string,
             "ECEF = (X:%e Y:%e Z:%e) tb:%1.3e\033[K\n\r\n\r",
             receiver_pvt.x,
             receiver_pvt.y,
             receiver_pvt.z,
             receiver_pvt.b);
    SER_PutString( string);

    /* Print the ECEF velocity info (even if it hasn't been set yet!) */
    sprintf( string,
             "Velocity in ECEF = (X:%e Y:%e Z:%e) tb:%1.3e\033[K\n\r\n\r",
             receiver_pvt_velocity.vx,
             receiver_pvt_velocity.vy,
             receiver_pvt_velocity.vz,
             receiver_pvt_velocity.df);
    SER_PutString( string);


    /* Print the LLH position info (even if it hasn't been set yet!) */
    sprintf( string,
             "LLH  = (Lat:%2.5f Lon:%2.5f Hgt:%6.2f)\033[K\n\r\n\r",
             lat,
             lon,
             receiver_llh.hgt);
    SER_PutString( string);

    /* Print out some position.c info */
    sprintf( string, "\
        State: positioning = %d, last position valid = %d\n\r\n\r",
             positioning, receiver_pvt.valid);
    SER_PutString( string);

	/* Print DOP */
        sprintf( string,
                 "DOP  = HDOP:%.4f PDOP:%.4f GDOP:%.4f)\033[K\n\r\n\r",
                 receiver_DOP.HDOP,
                 receiver_DOP.PDOP,
                 receiver_DOP.GDOP);
        SER_PutString( string);

    /* print ekf */
        sprintf( string,
                         "position  = x:%e y:%e z:%e b:%1.3e)\033[K\n\r\n\r",
                         ekf_pos.x,
                         ekf_pos.y,
                         ekf_pos.z,
                         ekf_pos.b);
                SER_PutString( string);

		sprintf( string,
						 "volecity  = vx:%e vy:%e vz:%e df:%1.3e)\033[K\n\r\n\r",
						 ekf_pos.vx,
						 ekf_pos.vy,
						 ekf_pos.vz,
						 ekf_pos.df);
				SER_PutString( string);

		sprintf( string,
						 "acceleration  = ax:%e ay:%e az:%e)\033[K\n\r\n\r",
						 ekf_pos.ax,
						 ekf_pos.ay,
						 ekf_pos.az);
				SER_PutString( string);

    /* beep the bell if we just got busy in position. */
    if (positioning) {
        if (!was_positioning) {
            sprintf( string, "\007"); // bell code
            SER_PutString( string);
            was_positioning = 1;
        }
    } else
        was_positioning = 0;
    
    /* Now put out a summary of what the hell is going on in the receiver */
    SER_PutString( header);

    /* Send out data on all 12 channels if there's no error */
    for( ch = 0; ch < N_CHANNELS; ch++) {
        switch( CH[ch].state) {
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

        if ((pr[ch].valid) && ephemeris[ch].valid) {
            az = sat_azel[ch].az * RADTODEG;
            el = sat_azel[ch].el * RADTODEG;
                        
            sprintf( string,
                     "%2d: %2d %c   %d   %d  %e \033[K\n\r",
                     ch,
                     CH[ch].prn,
                     channel_state,
                     pr[ch].valid,
                     ephemeris[ch].valid,
                     pr[ch].range
            );
            SER_PutString( string);
        } else {
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
static void display_pseudorange( void)
{
    unsigned short ch;
    unsigned char channel_state;
    char header[] =
        "\033[32m\033[HCH: PN S bit%50 eb ems Pseudorange Average\n\r\033[0m";
    char string[80];

    SER_PutString(header);

    /* Send out data on all 12 channels if there's no error */
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

        if (pr[ch].valid) {
            sprintf( string,
                    "%2d: %2d %c %6ld %2d %2d  %e %5ld\033[K\n\r",
                    ch,
                    pr[ch].prn,
                    channel_state,
                    pr[ch].bit_time,
                    pr[ch].epoch_bits,
                    pr[ch].epoch_ms,
                    pr[ch].range,
                    CH[ch].avg);
            SER_PutString( string);
        } else {
            sprintf( string,
                    "%2d: %2d %c\033[K\n\r",
                    ch,
                    pr[ch].prn,
                    channel_state);
            SER_PutString(string);
        }
    }
}


/******************************************************************************
 * Display ephemeris_thread info
 ******************************************************************************/
static void display_ephemeris( void)
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

    if (ephemeris_mode == 0) {
        SER_PutString( header0);

        for (ch = 0; ch < N_CHANNELS; ch++) {
            if (ephemeris[ch].valid) {
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
            } else {
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
    else if (ephemeris_mode == 1) {
        SER_PutString( header1);

        for (ch = 0; ch < N_CHANNELS; ch++) {
            if (ephemeris[ch].valid) {
                sprintf( string,
                         "%2d: % 15e % 15e % 15e\033[K\n\r",
                         ch,
                         ephemeris[ch].af2,
                         ephemeris[ch].af1,
                         ephemeris[ch].af0);
                SER_PutString( string);
            } else {
                sprintf( string, "%2d:\033[K\n\r", ch);
                SER_PutString( string);
            }
        }
    } else if (ephemeris_mode == 2) {
        SER_PutString( header2);

        for (ch = 0; ch < N_CHANNELS; ch++) {
            if (ephemeris[ch].valid) {
                sprintf( string,
                         "%2d: %4d % 15e % 15e % 15e % 15e\033[K\n\r",
                         ch,
                         ephemeris[ch].iode,
                         ephemeris[ch].crs,
                         ephemeris[ch].dn,
                         ephemeris[ch].ma,
                         ephemeris[ch].cuc);
                SER_PutString( string);
            } else {
                sprintf( string, "%2d:\033[K\n\r", ch);
                SER_PutString( string);
            }
        }
    } else if (ephemeris_mode == 3) {
        SER_PutString( header3);

        for (ch = 0; ch < N_CHANNELS; ch++) {
            if (ephemeris[ch].valid) {
                sprintf( string,
                         "%2d: % 15e % 15e % 15e %6.1f\033[K\n\r",
                         ch,
                         ephemeris[ch].ety,
                         ephemeris[ch].cus,
                         ephemeris[ch].sqra,
                         ephemeris[ch].toe);
                SER_PutString( string);
            } else {
                sprintf( string, "%2d:\033[K\n\r", ch);
                SER_PutString( string);
            }
        }
    } else if (ephemeris_mode == 4) {
        SER_PutString( header4);

        for (ch = 0; ch < N_CHANNELS; ch++) {
            if (ephemeris[ch].valid) {
                sprintf( string,
                         "%2d: % 15e % 15e % 15e % 15e\033[K\n\r",
                         ch,
                         ephemeris[ch].cic,
                         ephemeris[ch].w0,
                         ephemeris[ch].cis,
                         ephemeris[ch].inc0);
                SER_PutString( string);
            } else {
                sprintf( string, "%2d:\033[K\n\r", ch);
                SER_PutString( string);
            }
        }
    } else if (ephemeris_mode == 5) {
        SER_PutString( header5);

        for (ch = 0; ch < N_CHANNELS; ch++) {
            if (ephemeris[ch].valid) {
                sprintf( string,
                         "%2d: % 15e % 15e % 15e % 15e\033[K\n\r",
                         ch,
                         ephemeris[ch].crc,
                         ephemeris[ch].w,
                         ephemeris[ch].omegadot,
                         ephemeris[ch].idot);
                SER_PutString( string);
            } else {
                sprintf( string, "%2d:\033[K\n\r", ch);
                SER_PutString( string);
            }
        }
    } else
        ephemeris_mode = 0;
}

/******************************************************************************
 * Log the ephemeris from the satellites as it comes in. Called only when
 * ephemeris is processed, which is either right after a new lock or after
 * the IODE/IODC is updated.
 ******************************************************************************/

 // Display/log functions that can be turned on/off by switches.
void log_ephemeris(unsigned short ch, unsigned short subframe)
{
    char string[200];

    switch (subframe) {
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
static void display_tracking( void)
{
    char header[] =
        "\033[32m\033[HCh: PN  Iprmt  Qprmt State    Avg\n\r\033[0m";
    char string[80];

    unsigned short ch;
    unsigned short channel_state;
    unsigned short channel_bitsync;
    unsigned short channel_framesync;

    SER_PutString(header);

    /* Send out data on all channels if there's no error */
    for (ch = 0; ch < N_CHANNELS; ch++) {
        switch( CH[ch].state) {
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

        if (CH[ch].bit_sync == 1)
            channel_bitsync = 'B';
        else
            channel_bitsync = '-';

        if (messages[ch].frame_sync == 1)
            channel_framesync = 'F';
        else
            channel_framesync = '-';

        sprintf(string,
            "%2d: %2d %6d %6d %6d %6d %c(%c%c) %6ld\n\r",
            ch,
            CH[ch].prn + 1,
            CH[ch].i_early,
            CH[ch].i_prompt,
            CH[ch].i_late,
            CH[ch].q_prompt,
            channel_state,
            channel_bitsync,
            channel_framesync,
            CH[ch].avg);

        SER_PutString(string);
    }
}




/******************************************************************************
 * Display debug info
 ******************************************************************************/
static void display_debug( void)
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
static void display_messages( void)
{
    char header[] = 
    	"\033[32m\033[HCh: PN Mi   TOW SF SF1V SF2V SF3V SF4V SF5V State   Avg  DBug\n\r\033[0m";
    char string[80];

    unsigned short ch;
    unsigned short channel_state;
    unsigned short channel_bitsync;
    unsigned short channel_framesync;
    unsigned short TOW;

    SER_PutString( header);

    for (ch = 0; ch < N_CHANNELS; ch++) {
        switch( CH[ch].state) {
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

        if( CH[ch].bit_sync == 1)
            channel_bitsync = 'B';
        else
            channel_bitsync = '-';

        if( messages[ch].frame_sync == 1)
            channel_framesync = 'F';
        else
            channel_framesync = '-';

        if( messages[ch].subframes[0].valid)
            TOW = messages[ch].subframes[0].TOW;
        else
            TOW = 0;

        sprintf( string, 
        	"%2d: %2d %2d %5d %2d %4lx %4lx %4lx %4lx %4lx %c(%c%c) %5ld %2d\n\r",
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
            CH[ch].avg,
            CH[ch].ch_debug);

        SER_PutString( string);
    }
}

static void display_where(void)
{
	char string[1000];
	static int count = 0;
	time_t std_time;
	std_time = get_standard_time();
		if (count < 1000 && positioning && receiver_pvt.valid) {
		sprintf(string, "%e, %e, %e, %e, %e, %e,%f,%f,0;\n\r%e,%e,%e,%e,%e,%e,%e,%e,%e;\n\r",
				receiver_pvt.x,
				receiver_pvt.y,
				receiver_pvt.z,
				receiver_pvt_velocity.vx,
				receiver_pvt_velocity.vy,
				receiver_pvt_velocity.vz,
				receiver_pvt.b,
				receiver_pvt_velocity.df,
				ekf_pos.x,
				ekf_pos.y,
				ekf_pos.z,
				ekf_pos.vx,
				ekf_pos.vy,
				ekf_pos.vz,
				ekf_pos.ax,
				ekf_pos.ay,
				ekf_pos.az


	            );
		SER_PutString(string);
		count++;
	}
}


static void display_ekfparameter(void)
{
	char string[1000];
	static int count = 0;
	if (count < 5000 && positioning && receiver_pvt.valid) {
		sprintf(string, "%d, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f;\n\r%d, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f;\n\r%d, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f;\n\r%d, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f;\n\r",

				sat_position[0].prn,
				sat_position[0].x,
				sat_position[0].y,
				sat_position[0].z,
				sat_position[0].vx,
				sat_position[0].vy,
				sat_position[0].vz,
				m_rho[0],
				m_rho_dot[0],

				sat_position[1].prn,
				sat_position[1].x,
				sat_position[1].y,
				sat_position[1].z,
				sat_position[1].vx,
				sat_position[1].vy,
				sat_position[1].vz,
				m_rho[1],
				m_rho_dot[1],

				sat_position[2].prn,
				sat_position[2].x,
				sat_position[2].y,
				sat_position[2].z,
				sat_position[2].vx,
				sat_position[2].vy,
				sat_position[2].vz,
				m_rho[2],
				m_rho_dot[2],

				sat_position[3].prn,
				sat_position[3].x,
				sat_position[3].y,
				sat_position[3].z,
				sat_position[3].vx,
				sat_position[3].vy,
				sat_position[3].vz,
				m_rho[3],
				m_rho_dot[3]

	            );
		SER_PutString(string);
		count++;
	}
}


void display_thread(void const *argument)
{
    unsigned short  current_display = DISPLAY_NOT_USED;
    int got_byte;
    static char c;

    while (1) {
        osSignalWait(0x0004, osWaitForever);

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
        else if (c == 'w')
        	display_command = DISPLAY_WHERE;
        else if (c == 'k')
        	display_command = DISPLAY_EKFPa;
       

		if( current_display != display_command)
        {
            current_display = display_command;
            clear_screen();
        }

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

        else if( display_command == DISPLAY_WHERE)
            display_where();

        else if( display_command == DISPLAY_EKFPa)
            display_ekfparameter();
    }
}
