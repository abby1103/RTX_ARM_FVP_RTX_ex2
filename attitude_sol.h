#ifndef __attitude_sol_H
#define __attitude_sol_H

int attitude_sol( int n, ECEF_pos P_ant0, llh_pos ant0_llh, ECEF_pos P_sat[], double pseudo_range[], double sdcp[], double sdstd, double old_an[], double small_an[], double angle[]);

#endif
