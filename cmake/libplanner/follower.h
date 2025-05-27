#ifndef FOLLOWER_H
#define FOLLOWER_H

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double pos, vel, prev_ref;
} AxisState;

typedef struct {
    AxisState x, y, z;
} Follower;

static inline void follower_init(Follower* f, double x0, double y0, double z0) {
    *f = (Follower){
        .x = {x0, 0.0, x0},
        .y = {y0, 0.0, y0},
        .z = {z0, 0.0, z0}
    };
}

static inline void follower_update(Follower* f,
                                 double ref_x, double ref_y, double ref_z,
                                 double dt, double k_p, double k_d, double a_max)
{
    if (dt <= 0.0 || a_max <= 0.0) return;

    // Update each axis in a loop instead of using macro
    AxisState* axes[] = {&f->x, &f->y, &f->z};
    double refs[] = {ref_x, ref_y, ref_z};

    for (int i = 0; i < 3; i++) {
        double err_pos = refs[i] - axes[i]->pos;
        double ref_vel = (refs[i] - axes[i]->prev_ref) / dt;
        double a = k_p * err_pos + k_d * (ref_vel - axes[i]->vel);
        a = fmax(fmin(a, a_max), -a_max);
        axes[i]->vel += a * dt;
        axes[i]->pos += axes[i]->vel * dt;
        axes[i]->prev_ref = refs[i];
    }
}

#ifdef __cplusplus
}
#endif

#endif // FOLLOWER_H
