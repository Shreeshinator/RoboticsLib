/**
 * @file    pid.c
 * @brief   Discrete PID controller implementation
 *
 * Discrete form (Tustin / bilinear):
 *   P = Kp * e
 *   I = Ki * dt/2 * (e + e_prev)           [trapezoidal integration]
 *   D = Kd * N / (1 + N*dt) * (e - e_prev) [derivative with filter]
 */

#include "pid.h"
#include "robot_config.h"
#include <string.h>

void PID_Init(PID_t *pid, float kp, float ki, float kd, float dt)
{
    memset(pid, 0, sizeof(PID_t));

    pid->kp           = kp;
    pid->ki           = ki;
    pid->kd           = kd;
    pid->dt           = dt;
    pid->out_min      = ROBOT_PID_OUTPUT_MIN;
    pid->out_max      = ROBOT_PID_OUTPUT_MAX;
    pid->filter_coeff = 0.0f;   /* disabled by default */
    pid->derivative_on_measurement = true;
    pid->first_run    = true;
}

void PID_SetGains(PID_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_SetOutputLimits(PID_t *pid, float min, float max)
{
    pid->out_min = min;
    pid->out_max = max;
}

void PID_SetFilterCoeff(PID_t *pid, float N)
{
    pid->filter_coeff = N;
}

void PID_SetDerivativeOnMeasurement(PID_t *pid, bool enable)
{
    pid->derivative_on_measurement = enable;
}

float PID_Compute(PID_t *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;

    /* ── Proportional ───────────────────────────────────────────────────── */
    float p_term = pid->kp * error;

    /* ── Integral (trapezoidal, with anti-windup clamping) ──────────────── */
    pid->integrator += 0.5f * pid->ki * pid->dt * (error + pid->prev_error);

    /* Dynamic integrator clamp */
    float int_max =  (pid->out_max - p_term);
    float int_min =  (pid->out_min - p_term);
    if (int_max < 0.0f) int_max = 0.0f;
    if (int_min > 0.0f) int_min = 0.0f;
    pid->integrator = ROBOT_CLAMP(pid->integrator, int_min, int_max);

    /* ── Derivative ─────────────────────────────────────────────────────── */
    float d_term = 0.0f;
    if (!pid->first_run) {
        float d_input = pid->derivative_on_measurement
                        ? -(measurement - pid->prev_measurement)
                        :  (error       - pid->prev_error);

        if (pid->filter_coeff > 0.0f) {
            /* First-order Tustin low-pass filter */
            float alpha = pid->filter_coeff * pid->dt
                          / (1.0f + pid->filter_coeff * pid->dt);
            pid->differentiator = (1.0f - alpha) * pid->differentiator
                                  + alpha * pid->kd * d_input / pid->dt;
            d_term = pid->differentiator;
        } else {
            d_term = pid->kd * d_input / pid->dt;
        }
    }

    pid->first_run = false;

    /* ── Total output ───────────────────────────────────────────────────── */
    float output = ROBOT_CLAMP(p_term + pid->integrator + d_term,
                               pid->out_min, pid->out_max);

    /* Save state */
    pid->prev_error       = error;
    pid->prev_measurement = measurement;

    return output;
}

void PID_Reset(PID_t *pid)
{
    pid->integrator       = 0.0f;
    pid->prev_error       = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->differentiator   = 0.0f;
    pid->first_run        = true;
}

void PID_SetIntegrator(PID_t *pid, float value)
{
    pid->integrator = value;
}

float PID_GetIntegrator(const PID_t *pid)
{
    return pid->integrator;
}
