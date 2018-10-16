#ifndef __EKF_POSITION_H
#define __EKF_POSITION_H

void ekf_position_thread(void const *argument);

typedef struct
{
    double x, y, z, b;
    double vx, vy, vz, df;
    double ax, ay, az;
} pvat_t;

extern pvat_t ekf_pos;

#endif // __EKF_POSITION_H
