/**
 * @file    encoder.h
 * @brief   Quadrature encoder driver using STM32 TIM encoder mode
 *
 * Configure your timer in CubeMX:
 *   • Mode  → Combined Channels → Encoder Mode
 *   • Prescaler → 0 (count every edge)
 *   • Counter Period → 0xFFFF (16-bit) or 0xFFFFFFFF (32-bit)
 *
 * Encoder_Update() must be called at a fixed interval (e.g. 10 ms RTOS tick
 * or HAL_TIM_PeriodElapsedCallback) to accumulate counts and compute velocity.
 *
 * Usage:
 * @code
 *   Encoder_t enc;
 *   Encoder_Init(&enc, &htim3, 1024, 0.01f);   // 1024 CPR, 10 ms period
 *
 *   // In 10 ms ISR / task:
 *   Encoder_Update(&enc);
 *
 *   float rpm = Encoder_GetRPM(&enc);
 *   int32_t ticks = Encoder_GetTotalTicks(&enc);
 * @endcode
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "stm32_hal_include.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    TIM_HandleTypeDef *htim;

    uint32_t  counts_per_rev;       /* encoder CPR × gear ratio   */
    float     sample_period_s;      /* update interval in seconds  */

    int32_t   total_ticks;          /* accumulated signed count    */
    int32_t   delta_ticks;          /* ticks since last update     */
    float     rpm;                  /* revolutions per minute      */
    float     rad_per_sec;          /* angular velocity            */

    uint32_t  last_cnt;             /* raw TIM CNT at last update  */
    bool      is_32bit;             /* true for 32-bit timers      */
} Encoder_t;

/**
 * @brief  Initialise encoder.
 * @param  enc             Pointer to Encoder_t.
 * @param  htim            HAL timer in encoder mode.
 * @param  counts_per_rev  Total counts per output shaft revolution.
 * @param  sample_period_s Time between Encoder_Update calls (seconds).
 */
void    Encoder_Init(Encoder_t *enc, TIM_HandleTypeDef *htim,
                     uint32_t counts_per_rev, float sample_period_s);

/**
 * @brief  Must be called at a fixed period equal to sample_period_s.
 *         Reads TIM CNT, handles 16/32-bit overflow, updates velocity.
 */
void    Encoder_Update(Encoder_t *enc);

/** @brief  Reset accumulated tick count to zero. */
void    Encoder_ResetTicks(Encoder_t *enc);

/** @brief  Get total accumulated signed ticks since last reset. */
int32_t Encoder_GetTotalTicks(const Encoder_t *enc);

/** @brief  Get ticks counted in the last sample interval. */
int32_t Encoder_GetDeltaTicks(const Encoder_t *enc);

/** @brief  Get angular velocity in RPM (+ = forward). */
float   Encoder_GetRPM(const Encoder_t *enc);

/** @brief  Get angular velocity in rad/s. */
float   Encoder_GetRadPerSec(const Encoder_t *enc);

/** @brief  Get linear velocity in m/s given wheel radius (metres). */
float   Encoder_GetLinearVelocity(const Encoder_t *enc, float wheel_radius_m);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
