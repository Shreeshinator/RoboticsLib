/**
 * @file    robotics.h
 * @brief   STM32 Robotics Library — Master Include
 * @author  stm32-robotics contributors
 * @version 1.0.0
 *
 * Drop-in robotics primitives for STM32 HAL projects.
 * Include only this file in your application.
 *
 * Modules:
 *   - Motor    : PWM-driven DC motors with direction control
 *   - Encoder  : Quadrature encoder using TIM in encoder mode
 *   - PID      : Generic discrete PID controller
 *   - Servo    : RC-style PWM servo (50 Hz, 1–2 ms pulse)
 *   - Kinematics : Differential-drive & mecanum wheel geometry
 */

#ifndef ROBOTICS_H
#define ROBOTICS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "robot_config.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "servo.h"
#include "kinematics.h"

#ifdef __cplusplus
}
#endif

#endif /* ROBOTICS_H */
