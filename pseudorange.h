// pseudorange.h: Header file for the pseudorange.c file
// Copyright (C) 2005  Andrew Greenberg
// Distributed under the GNU GENERAL PUBLIC LICENSE (GPL) Version 2 (June 1991).
// See the "COPYING" file distributed with this software for more information.

#ifndef __PSEUDORANGE_H
#define __PSEUDORANGE_H

#include "time.h"
#include "namuru.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

// NONE?

/*******************************************************************************
 * Declarations
 ******************************************************************************/

typedef struct
{
    unsigned short  valid;
    unsigned short  prn;
    double          sat_time;        // Time of transmission
    double          range;
    double			range_old;
    double          delta_range;
    double 			delta_range_test;
    double          residual;
    
    unsigned long   bit_time;       // for debugging epoch counter stuff
    unsigned short  epoch_bits;
    unsigned short  epoch_ms;
} pseudorange_t;

void clear_pseudoranges(void);
void calculate_pseudorange(unsigned short ch);
/*******************************************************************************
 * Externs
 ******************************************************************************/

//extern cyg_flag_t pseudorange_flag;

extern pseudorange_t pr[N_CHANNELS];
extern gpstime_t     pr_time;

extern pseudorange_t pr2[N_CHANNELS];
extern gpstime_t     pr2_time;

#endif // __PSEUDORANGE_H
