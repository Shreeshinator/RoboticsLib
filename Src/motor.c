/**
 * @file    motor.c
 * @brief   PWM DC motor driver implementation
 */

#include "motor.h"
#include "robot_config.h"
#include <string.h>

/* ── Internal helpers ───────────────────────────────────────────────────── */

static inline void set_dir_pins(Motor_t *m, bool in1, bool in2)
{
    if (m->dir1_port) {
        HAL_GPIO_WritePin(m->dir1_port, m->dir1_pin,
                          in1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    if (m->dir2_port) {
        HAL_GPIO_WritePin(m->dir2_port, m->dir2_pin,
                          in2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

static inline uint32_t pct_to_ccr(float pct)
{
    float abs_pct = pct < 0.0f ? -pct : pct;
    abs_pct = ROBOT_CLAMP(abs_pct, 0.0f, 100.0f);
    return (uint32_t)((abs_pct / 100.0f) * (float)ROBOT_MOTOR_PWM_MAX);
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void Motor_Init(Motor_t *m,
                TIM_HandleTypeDef *htim, uint32_t channel,
                GPIO_TypeDef *dir1_port, uint16_t dir1_pin,
                GPIO_TypeDef *dir2_port, uint16_t dir2_pin)
{
    memset(m, 0, sizeof(Motor_t));

    m->htim       = htim;
    m->channel    = channel;
    m->dir1_port  = dir1_port;
    m->dir1_pin   = dir1_pin;
    m->dir2_port  = dir2_port;
    m->dir2_pin   = dir2_pin;
    m->inverted   = false;
    m->speed_pct  = 0.0f;

    /* Coast on init */
    set_dir_pins(m, false, false);
    __HAL_TIM_SET_COMPARE(m->htim, m->channel, 0);
    HAL_TIM_PWM_Start(m->htim, m->channel);
}

void Motor_SetSpeed(Motor_t *m, float speed_pct)
{
    speed_pct = ROBOT_CLAMP(speed_pct, -100.0f, 100.0f);
    m->speed_pct = speed_pct;

    float effective = m->inverted ? -speed_pct : speed_pct;

    if (effective > 0.0f) {
        /* Forward */
        set_dir_pins(m, true, false);
        __HAL_TIM_SET_COMPARE(m->htim, m->channel, pct_to_ccr(effective));
    } else if (effective < 0.0f) {
        /* Reverse */
        set_dir_pins(m, false, true);
        __HAL_TIM_SET_COMPARE(m->htim, m->channel, pct_to_ccr(-effective));
    } else {
        Motor_Coast(m);
    }
}

void Motor_Brake(Motor_t *m)
{
    set_dir_pins(m, true, true);
    __HAL_TIM_SET_COMPARE(m->htim, m->channel, ROBOT_MOTOR_PWM_MAX);
    m->speed_pct = 0.0f;
}

void Motor_Coast(Motor_t *m)
{
    set_dir_pins(m, false, false);
    __HAL_TIM_SET_COMPARE(m->htim, m->channel, 0);
    m->speed_pct = 0.0f;
}

void Motor_SetInverted(Motor_t *m, bool inverted)
{
    m->inverted = inverted;
}

float Motor_GetSpeed(const Motor_t *m)
{
    return m->speed_pct;
}
