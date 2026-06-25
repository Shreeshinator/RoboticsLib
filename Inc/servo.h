/**
 * @file    servo.h
 * @brief   RC servo driver (50 Hz PWM, 500–2500 µs pulse)
 *
 * Timer setup (CubeMX):
 *   • Clock source: Internal Clock
 *   • Prescaler   : (Timer_Clock / 1_000_000) - 1   → 1 µs resolution
 *   • Counter Period (ARR): 19999                    → 20 ms (50 Hz)
 *   • Channel mode: PWM Generation CHx
 *
 * Example for 72 MHz timer clock:
 *   Prescaler = 71, ARR = 19999
 *
 * Usage:
 * @code
 *   Servo_t s;
 *   Servo_Init(&s, &htim1, TIM_CHANNEL_1, 1000000u); // 1 MHz timer tick
 *   Servo_SetAngle(&s, 90.0f);   // centre
 * @endcode
 */

#ifndef SERVO_H
#define SERVO_H

#include "stm32_hal_include.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           channel;
    uint32_t           timer_freq_hz;   /* timer tick frequency after prescaler */

    float              min_pulse_us;    /* pulse width for ROBOT_SERVO_ANGLE_MIN */
    float              max_pulse_us;    /* pulse width for ROBOT_SERVO_ANGLE_MAX */
    float              angle_min;
    float              angle_max;
    float              current_angle;
} Servo_t;

/**
 * @brief  Initialise servo and centre it.
 * @param  s               Pointer to Servo_t.
 * @param  htim            HAL timer (PWM mode, 50 Hz already configured).
 * @param  channel         TIM_CHANNEL_x.
 * @param  timer_freq_hz   Frequency of the timer AFTER prescaler, in Hz.
 *                         E.g. 1 000 000 for 1 µs resolution.
 */
void  Servo_Init(Servo_t *s, TIM_HandleTypeDef *htim, uint32_t channel,
                 uint32_t timer_freq_hz);

/**
 * @brief  Override default pulse range (µs).
 *         Useful for servos with non-standard ranges.
 */
void  Servo_SetPulseRange(Servo_t *s, float min_us, float max_us);

/**
 * @brief  Override default angle range (degrees).
 */
void  Servo_SetAngleRange(Servo_t *s, float min_deg, float max_deg);

/**
 * @brief  Command servo to a specific angle in degrees.
 *         Clamped to [angle_min … angle_max].
 */
void  Servo_SetAngle(Servo_t *s, float angle_deg);

/**
 * @brief  Command servo directly via pulse width (µs).
 *         Clamped to [min_pulse_us … max_pulse_us].
 */
void  Servo_SetPulse(Servo_t *s, float pulse_us);

/** @brief  Return the last commanded angle in degrees. */
float Servo_GetAngle(const Servo_t *s);

/** @brief  Disable PWM output (servo will hold last position or go slack). */
void  Servo_Disable(Servo_t *s);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_H */
