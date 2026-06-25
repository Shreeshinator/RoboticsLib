/**
 * @file    robot_config.h
 * @brief   User-tunable compile-time configuration
 *
 * Edit this file to match your hardware before building.
 * All timing values assume the HAL tick is 1 ms (default CubeMX config).
 */

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "stm32_hal_include.h"   /* see note below */
#include <stdint.h>
#include <stdbool.h>

/* ─────────────────────────────────────────────
 * STM32 HAL family header shim
 * Uncomment the line that matches your MCU family,
 * or define ROBOT_HAL_INCLUDE in your build system.
 * ───────────────────────────────────────────── */
#ifndef ROBOT_HAL_INCLUDE
  /* Examples — pick one: */
  /* #define ROBOT_HAL_INCLUDE "stm32f4xx_hal.h" */
  /* #define ROBOT_HAL_INCLUDE "stm32g0xx_hal.h" */
  /* #define ROBOT_HAL_INCLUDE "stm32h7xx_hal.h" */
  #define ROBOT_HAL_INCLUDE "main.h"   /* CubeMX default — includes the right HAL */
#endif

/* ─────────────────────────────────────────────
 * PID defaults (override per-instance via PID_SetGains)
 * ───────────────────────────────────────────── */
#define ROBOT_PID_DEFAULT_KP   1.0f
#define ROBOT_PID_DEFAULT_KI   0.0f
#define ROBOT_PID_DEFAULT_KD   0.0f
#define ROBOT_PID_OUTPUT_MAX   100.0f   /* percent or raw PWM units */
#define ROBOT_PID_OUTPUT_MIN  -100.0f

/* ─────────────────────────────────────────────
 * Motor / PWM
 * ───────────────────────────────────────────── */
#define ROBOT_MOTOR_PWM_MAX    1000u    /* matches TIM ARR; adjust for your timer */
#define ROBOT_MOTOR_MAX_COUNT  4u       /* max motors in a MotorGroup */

/* ─────────────────────────────────────────────
 * Servo
 * ───────────────────────────────────────────── */
#define ROBOT_SERVO_PULSE_MIN_US   500u    /* µs — fully CW  */
#define ROBOT_SERVO_PULSE_MAX_US  2500u    /* µs — fully CCW */
#define ROBOT_SERVO_ANGLE_MIN      0.0f    /* degrees */
#define ROBOT_SERVO_ANGLE_MAX    180.0f

/* ─────────────────────────────────────────────
 * Encoder
 * ───────────────────────────────────────────── */
#define ROBOT_ENCODER_MAX_COUNT  4u

/* ─────────────────────────────────────────────
 * Kinematics
 * ───────────────────────────────────────────── */
/* Default differential-drive geometry (metres) — override at run-time */
#define ROBOT_DIFF_WHEEL_RADIUS   0.033f   /* 66 mm wheel */
#define ROBOT_DIFF_WHEEL_BASE     0.160f   /* 160 mm track width */

/* ─────────────────────────────────────────────
 * Utility macros
 * ───────────────────────────────────────────── */
#define ROBOT_CLAMP(x, lo, hi)   ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#define ROBOT_MAP(x, in_lo, in_hi, out_lo, out_hi) \
    ((out_lo) + ((float)((x) - (in_lo)) / (float)((in_hi) - (in_lo))) * (float)((out_hi) - (out_lo)))
#define ROBOT_DEG2RAD(d)   ((d) * 0.017453292f)
#define ROBOT_RAD2DEG(r)   ((r) * 57.295779f)

#endif /* ROBOT_CONFIG_H */
