/**
 * @file    kinematics.c
 * @brief   Robot kinematics implementation
 */

#include "kinematics.h"
#include "robot_config.h"
#include <string.h>
#include <math.h>

/* ─────────────────────────────────────────────
 * Differential Drive
 * ───────────────────────────────────────────── */

void DiffDrive_Init(DiffDrive_t *dd, float wheel_radius_m, float wheel_base_m)
{
    memset(dd, 0, sizeof(DiffDrive_t));
    dd->wheel_radius  = wheel_radius_m;
    dd->wheel_base    = wheel_base_m;
    dd->max_speed_mps = 1.0f;
}

void DiffDrive_SetMaxSpeed(DiffDrive_t *dd, float max_mps)
{
    dd->max_speed_mps = max_mps;
}

void DiffDrive_VelToWheels(const DiffDrive_t *dd, float v_mps, float omega_rps,
                            float *left_pct, float *right_pct)
{
    /* Inverse kinematics:
     *   v_left  = v - ω * L/2
     *   v_right = v + ω * L/2      (where L = wheel_base)
     */
    float half_L = dd->wheel_base * 0.5f;
    float v_l = v_mps - omega_rps * half_L;
    float v_r = v_mps + omega_rps * half_L;

    /* Normalise so the faster wheel = 100% */
    float max_v = fabsf(v_l) > fabsf(v_r) ? fabsf(v_l) : fabsf(v_r);
    if (max_v > dd->max_speed_mps) {
        float scale = dd->max_speed_mps / max_v;
        v_l *= scale;
        v_r *= scale;
    }

    /* Convert m/s → percent */
    float scale_pct = 100.0f / dd->max_speed_mps;
    *left_pct  = ROBOT_CLAMP(v_l * scale_pct, -100.0f, 100.0f);
    *right_pct = ROBOT_CLAMP(v_r * scale_pct, -100.0f, 100.0f);
}

void DiffDrive_UpdateOdometry(DiffDrive_t *dd,
                              float v_left_mps, float v_right_mps,
                              float dt)
{
    dd->v     = (v_right_mps + v_left_mps) * 0.5f;
    dd->omega = (v_right_mps - v_left_mps) / dd->wheel_base;

    /* Runge-Kutta 2 (midpoint) integration */
    float mid_theta = dd->theta + dd->omega * dt * 0.5f;
    dd->x     += dd->v * cosf(mid_theta) * dt;
    dd->y     += dd->v * sinf(mid_theta) * dt;
    dd->theta += dd->omega * dt;

    /* Wrap theta to [-π, π] */
    while (dd->theta >  (float)M_PI) dd->theta -= 2.0f * (float)M_PI;
    while (dd->theta < -(float)M_PI) dd->theta += 2.0f * (float)M_PI;
}

void DiffDrive_ResetOdometry(DiffDrive_t *dd, float x, float y, float theta)
{
    dd->x     = x;
    dd->y     = y;
    dd->theta = theta;
    dd->v     = 0.0f;
    dd->omega = 0.0f;
}

/* ─────────────────────────────────────────────
 * Mecanum Drive
 * ───────────────────────────────────────────── */

void Mecanum_Init(Mecanum_t *mc, float wheel_radius_m, float lx, float ly)
{
    memset(mc, 0, sizeof(Mecanum_t));
    mc->wheel_radius  = wheel_radius_m;
    mc->lx            = lx;
    mc->ly            = ly;
    mc->max_speed_mps = 1.0f;
}

void Mecanum_SetMaxSpeed(Mecanum_t *mc, float max_mps)
{
    mc->max_speed_mps = max_mps;
}

void Mecanum_VelToWheels(const Mecanum_t *mc,
                         float vx_mps, float vy_mps, float omega_rps,
                         MecanumSpeeds_t *out)
{
    /* Mecanum inverse kinematics:
     *   ω_fl =  (vx - vy - (lx+ly)*ω) / r
     *   ω_fr =  (vx + vy + (lx+ly)*ω) / r
     *   ω_rl =  (vx + vy - (lx+ly)*ω) / r
     *   ω_rr =  (vx - vy + (lx+ly)*ω) / r
     */
    float sum  = (mc->lx + mc->ly) * omega_rps;
    float inv_r = 1.0f / mc->wheel_radius;

    float w_fl = (vx_mps - vy_mps - sum) * inv_r;
    float w_fr = (vx_mps + vy_mps + sum) * inv_r;
    float w_rl = (vx_mps + vy_mps - sum) * inv_r;
    float w_rr = (vx_mps - vy_mps + sum) * inv_r;

    /* Convert rad/s → m/s, then normalise to percent */
    float fl = w_fl * mc->wheel_radius;
    float fr = w_fr * mc->wheel_radius;
    float rl = w_rl * mc->wheel_radius;
    float rr = w_rr * mc->wheel_radius;

    /* Find max magnitude */
    float max_v = fabsf(fl);
    if (fabsf(fr) > max_v) max_v = fabsf(fr);
    if (fabsf(rl) > max_v) max_v = fabsf(rl);
    if (fabsf(rr) > max_v) max_v = fabsf(rr);

    /* Normalise if over max */
    if (max_v > mc->max_speed_mps) {
        float s = mc->max_speed_mps / max_v;
        fl *= s; fr *= s; rl *= s; rr *= s;
    }

    float scale_pct = 100.0f / mc->max_speed_mps;
    out->fl = ROBOT_CLAMP(fl * scale_pct, -100.0f, 100.0f);
    out->fr = ROBOT_CLAMP(fr * scale_pct, -100.0f, 100.0f);
    out->rl = ROBOT_CLAMP(rl * scale_pct, -100.0f, 100.0f);
    out->rr = ROBOT_CLAMP(rr * scale_pct, -100.0f, 100.0f);
}
