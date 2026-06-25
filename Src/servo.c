/**
 * @file    servo.c
 * @brief   RC servo driver implementation
 */

#include "servo.h"
#include "robot_config.h"
#include <string.h>

void Servo_Init(Servo_t *s, TIM_HandleTypeDef *htim, uint32_t channel,
                uint32_t timer_freq_hz)
{
    memset(s, 0, sizeof(Servo_t));

    s->htim          = htim;
    s->channel       = channel;
    s->timer_freq_hz = timer_freq_hz;
    s->min_pulse_us  = (float)ROBOT_SERVO_PULSE_MIN_US;
    s->max_pulse_us  = (float)ROBOT_SERVO_PULSE_MAX_US;
    s->angle_min     = ROBOT_SERVO_ANGLE_MIN;
    s->angle_max     = ROBOT_SERVO_ANGLE_MAX;

    HAL_TIM_PWM_Start(htim, channel);

    /* Centre on startup */
    Servo_SetAngle(s, (s->angle_min + s->angle_max) * 0.5f);
}

void Servo_SetPulseRange(Servo_t *s, float min_us, float max_us)
{
    s->min_pulse_us = min_us;
    s->max_pulse_us = max_us;
}

void Servo_SetAngleRange(Servo_t *s, float min_deg, float max_deg)
{
    s->angle_min = min_deg;
    s->angle_max = max_deg;
}

void Servo_SetAngle(Servo_t *s, float angle_deg)
{
    angle_deg = ROBOT_CLAMP(angle_deg, s->angle_min, s->angle_max);
    s->current_angle = angle_deg;

    /* Map angle → pulse width (µs) */
    float pulse_us = ROBOT_MAP(angle_deg,
                               s->angle_min, s->angle_max,
                               s->min_pulse_us, s->max_pulse_us);

    Servo_SetPulse(s, pulse_us);
}

void Servo_SetPulse(Servo_t *s, float pulse_us)
{
    pulse_us = ROBOT_CLAMP(pulse_us, s->min_pulse_us, s->max_pulse_us);

    /* Convert µs to timer ticks:
     * ticks = pulse_us * timer_freq_hz / 1_000_000 */
    uint32_t ticks = (uint32_t)(pulse_us * (float)s->timer_freq_hz / 1000000.0f);
    __HAL_TIM_SET_COMPARE(s->htim, s->channel, ticks);
}

float Servo_GetAngle(const Servo_t *s)
{
    return s->current_angle;
}

void Servo_Disable(Servo_t *s)
{
    HAL_TIM_PWM_Stop(s->htim, s->channel);
}
