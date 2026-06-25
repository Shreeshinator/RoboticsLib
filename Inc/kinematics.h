/**
 * @file    kinematics.h
 * @brief   Robot kinematics — differential drive & mecanum wheel models
 *
 * ── Differential Drive ──────────────────────────────────────────────────
 * Converts between:
 *   • (linear velocity v, angular velocity ω) ←→ (left speed, right speed)
 * and integrates odometry from encoder measurements.
 *
 * ── Mecanum Drive ───────────────────────────────────────────────────────
 * 4-wheel holonomic drive — converts (vx, vy, ω) to individual wheel speeds.
 * Wheel layout (viewed from above):
 *
 *      FL ── FR
 *      |      |
 *      RL ── RR
 *
 * Usage (differential drive):
 * @code
 *   DiffDrive_t dd;
 *   DiffDrive_Init(&dd, 0.033f, 0.160f);   // wheel radius, wheel base
 *
 *   float left_pct, right_pct;
 *   DiffDrive_VelToWheels(&dd, 0.3f, 0.5f, &left_pct, &right_pct);
 *   Motor_SetSpeed(&left_motor,  left_pct);
 *   Motor_SetSpeed(&right_motor, right_pct);
 *
 *   // In control loop:
 *   DiffDrive_UpdateOdometry(&dd, Encoder_GetLinearVelocity(&enc_left, 0.033f),
 *                                  Encoder_GetLinearVelocity(&enc_right, 0.033f),
 *                                  0.01f);
 * @endcode
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ─────────────────────────────────────────────
 * Differential Drive
 * ───────────────────────────────────────────── */

typedef struct {
    float wheel_radius;     /* metres */
    float wheel_base;       /* centre-to-centre track width, metres */
    float max_speed_mps;    /* m/s at 100 % motor output */

    /* Odometry state (world frame) */
    float x;               /* metres */
    float y;               /* metres */
    float theta;           /* radians, CCW positive */
    float v;               /* linear velocity m/s  */
    float omega;           /* angular velocity rad/s */
} DiffDrive_t;

/**
 * @brief  Initialise differential drive.
 * @param  dd              Pointer to DiffDrive_t.
 * @param  wheel_radius_m  Wheel radius in metres.
 * @param  wheel_base_m    Distance between wheel centres in metres.
 */
void DiffDrive_Init(DiffDrive_t *dd, float wheel_radius_m, float wheel_base_m);

/** @brief  Set max linear speed (m/s) for normalisation. Default 1.0 m/s. */
void DiffDrive_SetMaxSpeed(DiffDrive_t *dd, float max_mps);

/**
 * @brief  Convert desired (v, ω) to motor percent outputs.
 * @param  dd         Pointer to DiffDrive_t.
 * @param  v_mps      Desired linear velocity (m/s). +ve = forward.
 * @param  omega_rps  Desired angular velocity (rad/s). +ve = CCW.
 * @param  left_pct   [out] Left motor speed  [-100..100].
 * @param  right_pct  [out] Right motor speed [-100..100].
 */
void DiffDrive_VelToWheels(const DiffDrive_t *dd, float v_mps, float omega_rps,
                            float *left_pct, float *right_pct);

/**
 * @brief  Integrate odometry from measured wheel velocities.
 *         Call this every dt seconds.
 * @param  dd           Pointer to DiffDrive_t.
 * @param  v_left_mps   Measured left wheel linear velocity (m/s).
 * @param  v_right_mps  Measured right wheel linear velocity (m/s).
 * @param  dt           Elapsed time since last call (seconds).
 */
void DiffDrive_UpdateOdometry(DiffDrive_t *dd,
                              float v_left_mps, float v_right_mps,
                              float dt);

/** @brief  Reset odometry to zero (or given pose). */
void DiffDrive_ResetOdometry(DiffDrive_t *dd, float x, float y, float theta);

/* ─────────────────────────────────────────────
 * Mecanum (4-wheel holonomic)
 * ───────────────────────────────────────────── */

typedef struct {
    float wheel_radius;    /* metres */
    float lx;              /* half wheel-base (longitudinal), metres */
    float ly;              /* half wheel-base (lateral), metres      */
    float max_speed_mps;
} Mecanum_t;

typedef struct {
    float fl;   /* Front-Left  [-100..100] */
    float fr;   /* Front-Right [-100..100] */
    float rl;   /* Rear-Left   [-100..100] */
    float rr;   /* Rear-Right  [-100..100] */
} MecanumSpeeds_t;

/**
 * @brief  Initialise mecanum drive.
 * @param  mc             Pointer to Mecanum_t.
 * @param  wheel_radius_m Wheel radius in metres.
 * @param  lx             Half longitudinal wheel-base (front–rear distance / 2).
 * @param  ly             Half lateral wheel-base (left–right distance / 2).
 */
void Mecanum_Init(Mecanum_t *mc, float wheel_radius_m, float lx, float ly);

/** @brief  Set max linear speed for normalisation. Default 1.0 m/s. */
void Mecanum_SetMaxSpeed(Mecanum_t *mc, float max_mps);

/**
 * @brief  Convert (vx, vy, ω) to 4-wheel motor percent speeds.
 * @param  mc         Pointer to Mecanum_t.
 * @param  vx_mps     Desired X (forward) velocity m/s.
 * @param  vy_mps     Desired Y (strafe left) velocity m/s.
 * @param  omega_rps  Desired rotation rad/s (CCW +ve).
 * @param  out        [out] MecanumSpeeds_t with FL/FR/RL/RR percents.
 */
void Mecanum_VelToWheels(const Mecanum_t *mc,
                         float vx_mps, float vy_mps, float omega_rps,
                         MecanumSpeeds_t *out);

#ifdef __cplusplus
}
#endif

#endif /* KINEMATICS_H */
