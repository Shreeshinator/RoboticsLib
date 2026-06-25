/**
 * @file    motor.h
 * @brief   PWM DC motor driver (H-bridge / DRV88xx / L298N compatible)
 *
 * Each Motor maps to:
 *   • One TIM channel for PWM speed control
 *   • Two GPIO pins for direction (IN1 / IN2)  — set to GPIO_PIN_RESET when unused
 *
 * Speed is expressed as a signed percentage [-100 … +100].
 * Positive = forward, Negative = reverse, 0 = coast.
 *
 * Usage:
 * @code
 *   Motor_t m;
 *   Motor_Init(&m, &htim2, TIM_CHANNEL_1,
 *               DIR_GPIO_Port, DIR1_Pin,
 *               DIR_GPIO_Port, DIR2_Pin);
 *   Motor_SetSpeed(&m, 75.0f);   // 75 % forward
 * @endcode
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "stm32_hal_include.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Types ──────────────────────────────────────────────────────────────── */

typedef enum {
    MOTOR_COAST = 0,
    MOTOR_FORWARD,
    MOTOR_REVERSE,
    MOTOR_BRAKE
} Motor_Dir_t;

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           channel;

    GPIO_TypeDef      *dir1_port;
    uint16_t           dir1_pin;
    GPIO_TypeDef      *dir2_port;
    uint16_t           dir2_pin;

    float              speed_pct;   /* last commanded speed [-100..100] */
    bool               inverted;    /* swap forward/reverse logic        */
} Motor_t;

/* ── API ────────────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise motor and start PWM output at 0 %.
 * @param  m         Pointer to uninitialised Motor_t struct.
 * @param  htim      HAL timer handle with PWM already configured.
 * @param  channel   TIM_CHANNEL_x for this motor.
 * @param  dir1_port GPIO port for IN1 (NULL to skip direction GPIO).
 * @param  dir1_pin  GPIO pin  for IN1.
 * @param  dir2_port GPIO port for IN2 (NULL to skip direction GPIO).
 * @param  dir2_pin  GPIO pin  for IN2.
 */
void Motor_Init(Motor_t *m,
                TIM_HandleTypeDef *htim, uint32_t channel,
                GPIO_TypeDef *dir1_port, uint16_t dir1_pin,
                GPIO_TypeDef *dir2_port, uint16_t dir2_pin);

/**
 * @brief  Set motor speed as a signed percentage [-100.0 … +100.0].
 *         Values outside the range are clamped.
 */
void Motor_SetSpeed(Motor_t *m, float speed_pct);

/**
 * @brief  Actively brake the motor (both IN pins HIGH).
 */
void Motor_Brake(Motor_t *m);

/**
 * @brief  Coast — remove PWM, let motor freewheel.
 */
void Motor_Coast(Motor_t *m);

/**
 * @brief  Invert the forward/reverse polarity for wiring corrections.
 */
void Motor_SetInverted(Motor_t *m, bool inverted);

/**
 * @brief  Return the last commanded speed percent.
 */
float Motor_GetSpeed(const Motor_t *m);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
