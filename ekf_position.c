/* gps_ekf: TinyEKF test case using You Chong's GPS example:
 * 
 *   http://www.mathworks.com/matlabcentral/fileexchange/31487-extended-kalman-filter-ekf--for-gps
 * 
 * Reads file gps.csv of satellite data and writes file ekf.csv of mean-subtracted estimated positions.
 *
 *
 * References:
 *
 * 1. R G Brown, P Y C Hwang, "Introduction to random signals and applied 
 * Kalman filtering : with MATLAB exercises and solutions",1996
 *
 * 2. Pratap Misra, Per Enge, "Global Positioning System Signals, 
 * Measurements, and Performance(Second Edition)",2006
 * 
 * Copyright (C) 2015 Simon D. Levy
 *
 * MIT License
 */

#include "cmsis_os.h"

#include <stdlib.h>
#include <math.h>

#include "tinyekf_config.h"
#include "tiny_ekf.h"
#include "ekf_position.h"
#include "pseudorange.h"
#include "position.h"


ekf_t ekf;
pvat_t ekf_pos;

//extern xyz_t  satxyz[12];

/*
 * temporaily global to log in display.
 *
 * MUST fix in future release
 */
double SV_Pos[8][3];
double SV_Rho[6];

// positioning interval
static const double T = 0.1;

static void blkfill(ekf_t * ekf, const double * a, int off)
{
    off *= 3;

    ekf->Q[off]   [off]   = a[0]; 
    ekf->Q[off]   [off+1] = a[1];
    ekf->Q[off]   [off+2] = a[2];
    ekf->Q[off+1] [off]   = a[3];
    ekf->Q[off+1] [off+1] = a[4];
    ekf->Q[off+1] [off+2] = a[5];
    ekf->Q[off+2] [off]   = a[6];
    ekf->Q[off+2] [off+1] = a[7];
    ekf->Q[off+2] [off+2] = a[8];
}

static void blkfill2(ekf_t * ekf, const double * a, int off, int start)
{
    off *= 2;
    off += start;

    ekf->Q[off]   [off]   = a[0]; 
    ekf->Q[off]   [off+1] = a[1];
    ekf->Q[off+1] [off]   = a[2];
    ekf->Q[off+1] [off+1] = a[3];
}


static void init(ekf_t * ekf)
{
    // Set Q, see [1]
    //const double Sf    = 36;
   // const double Sg    = 0.01;
    const double sigma = 5;         // state transition variance
   /* const double Qb[4] =
    {
    		Sf*T+Sg*T*T*T/3,	Sg*T*T/2,
    		Sg*T*T/2, 			Sg*T
    };*/

    /* TODO: find the governing equation for this */
    const double Qxyz[9] =
    {
    	sigma*sigma*T*T*T/3, 	sigma*sigma*T*T/2, 	0,
    	sigma*sigma*T*T/2,		sigma*sigma*T, 		10e-10,
    	0, 			   			10e-10, 			10e-10
    };

    blkfill(ekf,  Qxyz, 0);
    blkfill(ekf,  Qxyz, 1);
    blkfill(ekf,  Qxyz, 2);
   // blkfill2(ekf, Qb,   0, 9);


    // initial covariances of state noise, measurement noise
    double P0 = 10;
    double R0 = 36;
    double RV0 = 0.0025;


    int i;

    for (i = 0; i<9; ++i)
        ekf->P[i][i] = P0;

    for (i = 0; i<3; ++i)
        ekf->R[i][i] = R0;
    
    for (i = 0; i<3; ++i)
        ekf->R[i + 3][i + 3] = RV0;

    // position
    ekf->x[0] = receiver_pvt.x;
    ekf->x[3] = receiver_pvt.y;
    ekf->x[6] = receiver_pvt.z;

    // velocity
    ekf->x[1] = receiver_pvt_velocity.vx;
    ekf->x[4] = receiver_pvt_velocity.vy;
    ekf->x[7] = receiver_pvt_velocity.vz;

    // acceleration
    ekf->x[2] = 0;
    ekf->x[5] = 0;
    ekf->x[8] = 0;

    /*// clock bias
    ekf->x[9] = receiver_pvt.b;

    // clock drift
    ekf->x[10] = receiver_pvt_velocity.df;
*/
}

static void model(ekf_t * ekf, double SV[8][3])
{ 


	int i, j;

   /* for (j=0; j<8; j+=3) {
        ekf->fx[j] = ekf->x[j] + T * ekf->x[j+1] + 0.5 * T * T * ekf->x[j+2];
        ekf->fx[j+1] = ekf->x[j+1] + T * ekf->x[j+2];
        ekf->fx[j+2] = ekf->x[j+2]; 
    }*/

	for (j=0; j<8; j+=3) {
	        ekf->fx[j] = ekf->x[j] + T * ekf->x[j+1];
	        ekf->fx[j+1] = ekf->x[j+1] + T * ekf->x[j+2];
	        ekf->fx[j+2] = ekf->x[j+2];
	    }
    /*for (j = 9; j < 10; j += 2) {
        ekf->fx[j] = ekf->x[j] + T * ekf->x[j+1];
        ekf->fx[j+1] = ekf->x[j+1];
    }*/

    for (j=0; j<9; ++j)
        ekf->F[j][j] = 1;

    for (j=0; j<3; ++j) {
        ekf->F[3*j][3*j+1] = T;
        //ekf->F[3*j][3*j+2] = 0.5 * T * T;
        ekf->F[3*j][3*j+2] = 0;
        ekf->F[3*j+1][3*j+2] = T;
    }

    	//ekf->F[9][10] = T;
    

    double dx[4][3];
    double rho_no_time_bias[4] = {0};
    double hx_temp[8];

    for (i=0; i<4; ++i) {

    	hx_temp[i] = 0;
        for (j=0; j<3; ++j) {
            double d = ekf->fx[j*3] - SV[i][j];
            dx[i][j] = d;
            hx_temp[i] += d*d;
        }
        rho_no_time_bias[i] = pow(hx_temp[i], 0.5);
        hx_temp[i] = rho_no_time_bias[i];
    }

    for (i=0; i<3; ++i){
    ekf->hx[i] = hx_temp[0] - hx_temp[i+1];
    }

    double dv[4][3];
    double dv_dx[4][3];
    double sum_dvdx[4];

    for (i = 0; i < 4; ++i) {
    	hx_temp[i + 4] = 0;
        for (j = 0; j < 3; ++j) {
            double d = ekf->fx[j*3 + 1] - SV[i + 4][j] ;
            dv[i][j] = d;
            dv_dx[i][j] = dv[i][j] * dx[i][j];
            sum_dvdx[i] += dv_dx[i][j];
            hx_temp[i + 4] = sum_dvdx[i];
        }
        hx_temp[i + 4] /= rho_no_time_bias[i];
    }

    for (i=0; i<3; ++i){
        ekf->hx[i+3] = hx_temp[4] - hx_temp[i+1+4];
        }


    double H_temp[8][9];
    for (i=0; i<4; ++i) {
        for (j=0; j<3; ++j) 
        	H_temp[i][j*3]  = dx[i][j] / rho_no_time_bias[i];

    }
    for (i=0; i<3; ++i){
    	for (j=0; j<3; ++j)
    		ekf->H[i][j*3] = H_temp[0][j*3] - H_temp[i+1][j*3];
        }


    for (i = 0; i < 4; ++i) {
        for (j = 0; j < 3; ++j) {
        	H_temp[i + 4][j*3] = - sum_dvdx[i] * dx[i][j] / pow(rho_no_time_bias[i], 3) + dv[i][j] / rho_no_time_bias[i] ;
        	H_temp[i + 4][j*3 + 1] = dx[i][j] / rho_no_time_bias[i];
        }

    }
    for (i=0; i<3; ++i){
    	for (j=0; j<3; ++j) {
			ekf->H[i+3][j*3] = H_temp[4][j*3] - H_temp[i+1+4][j*3];
			ekf->H[i+3][j*3+1] = H_temp[4][j*3+1] - H_temp[i+1+4][j*3+1];
    	}
	}
}

void ekf_position_thread(void const *argument)
{    
    // Do generic EKF initialization
    ekf_init(&ekf, Nsta, Mobs);

    static int ekf_position_init = 1;
    xyz_t  satxyz[12];
    xyz_t  satvxyz[12];
    double alpha;
	#define OMEGA_E 7.2921151467E-5

    while (1) {
        osSignalWait(0x0005, osWaitForever);
        if (ekf_position_init) {
            // Do local initialization
            init(&ekf);
            ekf_position_init = 0;
        }

        for (int i = 0; i < 4; i++) {
			// alpha is the angular rotation of the earth since the signal left
			 // satellite[i]. [radians]
			alpha = m_rho[i] * OMEGA_E / SPEED_OF_LIGHT;

			 // Compute the change in satellite position during signal propagation.
			 // Exact relations:
			 //   sa = sin( alpha);
			 //   ca = cos( alpha);
			 //   dx = (sat_position[i].x * ca - sat_position[i].y * sa) - x;
			 //   dy = (sat_position[i].y * ca + sat_position[i].x * sa) - y;
			 //   dz = sat_position[i].z - z;
			 // Make the small angle approximation
			 //   cos( alpha) ~= 1 and sin( alpha) ~= alpha.

			 satxyz[i].x = sat_position[i].x + sat_position[i].y * alpha;
			 satxyz[i].y = sat_position[i].y - sat_position[i].x * alpha;
			 satxyz[i].z = sat_position[i].z;

			 satvxyz[i].x = sat_position[i].vx;
			 satvxyz[i].y = sat_position[i].vy;
			 satvxyz[i].z = sat_position[i].vz;

            SV_Pos[i][0] = satxyz[i].x;
            SV_Pos[i][1] = satxyz[i].y;
            SV_Pos[i][2] = satxyz[i].z;
        }

        for (int i = 0; i < 4; i++) {
            SV_Pos[i + 4][0] = satvxyz[i].x;
            SV_Pos[i + 4][1] = satvxyz[i].y;
            SV_Pos[i + 4][2] = satvxyz[i].z;

        }

        for (int i = 0; i <3; i++){
        SV_Rho[i] = m_rho[0]-m_rho[i+1];
        SV_Rho[i + 3] = m_rho_dot[0]-m_rho_dot[i+1];
        }

        model(&ekf, SV_Pos);
        ekf_step(&ekf, SV_Rho);

        ekf_pos.x = ekf.x[0];
        ekf_pos.vx = ekf.x[1];
        ekf_pos.ax = ekf.x[2];
        ekf_pos.y = ekf.x[3];
        ekf_pos.vy = ekf.x[4];
        ekf_pos.ay = ekf.x[5];
        ekf_pos.z = ekf.x[6];
        ekf_pos.vz = ekf.x[7];
        ekf_pos.az = ekf.x[8];
        //ekf_pos.b = ekf.x[9];
        //ekf_pos.df = ekf.x[10];
    }
}
