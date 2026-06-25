/**
 * @file    servo_arm_example.c
 * @brief   3-DOF servo arm example
 *
 * Hardware:
 *   TIM1 CH1 — Base servo      (50 Hz, 1 µs tick: PSC=71, ARR=19999 @ 72 MHz)
 *   TIM1 CH2 — Shoulder servo
 *   TIM1 CH3 — Gripper servo
 *
 * All three channels share TIM1 configured for 50 Hz PWM.
 * Timer post-prescaler frequency = 1 000 000 Hz (1 µs resolution).
 */

#include "robotics.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;

#define TIMER_FREQ_HZ  1000000u   /* 1 µs resolution */

static Servo_t base, shoulder, gripper;

void Arm_Init(void)
{
    Servo_Init(&base,     &htim1, TIM_CHANNEL_1, TIMER_FREQ_HZ);
    Servo_Init(&shoulder, &htim1, TIM_CHANNEL_2, TIMER_FREQ_HZ);
    Servo_Init(&gripper,  &htim1, TIM_CHANNEL_3, TIMER_FREQ_HZ);

    /* Gripper has a restricted range */
    Servo_SetAngleRange(&gripper, 0.0f, 60.0f);

    /* Home position */
    Servo_SetAngle(&base,     90.0f);
    Servo_SetAngle(&shoulder, 45.0f);
    Servo_SetAngle(&gripper,   0.0f);
}

void Arm_MoveTo(float base_deg, float shoulder_deg, float gripper_deg)
{
    Servo_SetAngle(&base,     base_deg);
    Servo_SetAngle(&shoulder, shoulder_deg);
    Servo_SetAngle(&gripper,  gripper_deg);
}

void Arm_Open(void)  { Servo_SetAngle(&gripper,  0.0f); }
void Arm_Close(void) { Servo_SetAngle(&gripper, 60.0f); }
