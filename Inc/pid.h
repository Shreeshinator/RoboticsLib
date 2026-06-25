/**
 * @file    pid.h
 * @brief   Generic discrete PID controller with anti-windup and derivative filter
 *
 * Features:
 *   • Anti-windup via integrator clamping
 *   • Optional derivative low-pass filter (N coefficient)
 *   • Output saturation with separate min/max
 *   • Feed-forward term
 *   • Bumpless gain update
 *
 * Usage:
 * @code
 *   PID_t pid;
 *   PID_Init(&pid, 1.5f, 0.2f, 0.05f, 0.01f);   // Kp, Ki, Kd, dt
 *   PID_SetOutputLimits(&pid, -100.0f, 100.0f);
 *
 *   // In control loop (every dt seconds):
 *   float output = PID_Compute(&pid, setpoint, measurement);
 * @endcode
 */

#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* Gains */
    float kp, ki, kd;

    /* Timing */
    float dt;           /* sample period in seconds        */

    /* Output limits */
    float out_min;
    float out_max;

    /* Internal state */
    float integrator;
    float prev_error;
    float prev_measurement;   /* for derivative-on-measurement */
    float differentiator;

    /* Derivative filter coefficient (0 = off, higher = more filtering) */
    float filter_coeff;       /* N in tustin approximation     */

    /* Flags */
    bool  derivative_on_measurement;  /* avoids derivative kick on setpoint change */
    bool  first_run;
} PID_t;

/**
 * @brief  Initialise PID with gains and sample period.
 * @param  pid  Pointer to PID_t.
 * @param  kp   Proportional gain.
 * @param  ki   Integral gain.
 * @param  kd   Derivative gain.
 * @param  dt   Sample period in seconds.
 */
void  PID_Init(PID_t *pid, float kp, float ki, float kd, float dt);

/** @brief  Update gains without resetting state (bumpless). */
void  PID_SetGains(PID_t *pid, float kp, float ki, float kd);

/** @brief  Set output saturation limits. */
void  PID_SetOutputLimits(PID_t *pid, float min, float max);

/** @brief  Set derivative low-pass filter coefficient N (0 = bypass). */
void  PID_SetFilterCoeff(PID_t *pid, float N);

/** @brief  Use derivative-on-measurement to avoid setpoint kick. */
void  PID_SetDerivativeOnMeasurement(PID_t *pid, bool enable);

/**
 * @brief  Compute one PID iteration.
 * @param  pid          Pointer to PID_t.
 * @param  setpoint     Desired value.
 * @param  measurement  Current measured value.
 * @return Control output clamped to [out_min, out_max].
 */
float PID_Compute(PID_t *pid, float setpoint, float measurement);

/** @brief  Reset integrator and state (e.g. after mode change). */
void  PID_Reset(PID_t *pid);

/** @brief  Manually set integrator value (pre-load / anti-windup external). */
void  PID_SetIntegrator(PID_t *pid, float value);

/** @brief  Get current integrator value. */
float PID_GetIntegrator(const PID_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
