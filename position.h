// position.h: Header file for the position.c file
// Copyright (C) 2005  Andrew Greenberg
// Distributed under the GNU GENERAL PUBLIC LICENSE (GPL) Version 2 (June 1991).
// See the "COPYING" file distributed with this software for more information.

#ifndef __POSITION_H
#define __POSITION_H

#include "constants.h"
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
    double az;
    double el;
} azel_t;

typedef struct
{
    double lat;
    double lon;
    double hgt;
} llh_t;

typedef struct
{
    double x, y, z;
} xyz_t;

typedef struct
{
    double x, y, z;
} vxyz_t;


typedef struct
{
    double  x, y, z;
    double  tb;
} xyzt_t;

typedef struct
{
	double	vx, vy, vz;
	double	td;
} vxyzt_t;

typedef struct
{
    double  x, y, z;
    double  tb;
    double	vx, vy, vz;
    unsigned short channel;
    unsigned short prn;
} satpos_t;

typedef struct
{
	xyzt_t pos;
	vxyzt_t vel;
} satinfo_t;

typedef struct
{
    double x,y,z,b;
    double vx,vy,vz,df;
    double error;           // Convergence error from position calculation
    unsigned short valid;   // if the error is valid or not.
    unsigned short n;       // number of satellites used in the solution
} pvt_t;

void clear_position( void);
llh_t ecef_to_llh( xyz_t pos);
pvt_t calculate_position( unsigned short nsats_used);
pvt_t calculate_velocity( unsigned short nsats_used);
pvt_t calculate_velocity_test( unsigned short nsats_used);
azel_t satellite_azel( xyzt_t satpos);
satinfo_t SatPosEphemeris( unsigned short ch);


/*******************************************************************************
 * Externs
 ******************************************************************************/

//extern cyg_sem_t position_semaphore;
extern unsigned short position_busy;
extern unsigned short positioning;

extern pvt_t    receiver_pvt;
extern pvt_t	receiver_pvt_velocity;
extern pvt_t	receiver_pvt_velocity_test;
extern llh_t    receiver_llh;

extern azel_t   sat_azel[N_CHANNELS];
extern xyzt_t	sat_pos_by_ch[N_CHANNELS];
extern xyzt_t	sat_pos_by_ch_old[N_CHANNELS];
extern vxyzt_t  sat_vel_by_ch[N_CHANNELS];
extern vxyzt_t  sat_vel_by_ch_test[N_CHANNELS];

extern satpos_t sat_position[N_CHANNELS];
extern double   m_rho[N_CHANNELS];
extern double 	m_rho_dot[N_CHANNELS];
extern double 	m_rho_dot_test[N_CHANNELS];

#endif // __POSITION_H
