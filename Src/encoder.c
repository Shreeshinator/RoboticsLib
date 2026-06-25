/**
 * @file    encoder.c
 * @brief   Quadrature encoder implementation
 */

#include "encoder.h"
#include "robot_config.h"
#include <string.h>
#include <math.h>

/* 2π constant */
#define TWO_PI  6.283185307f

void Encoder_Init(Encoder_t *enc, TIM_HandleTypeDef *htim,
                  uint32_t counts_per_rev, float sample_period_s)
{
    memset(enc, 0, sizeof(Encoder_t));

    enc->htim            = htim;
    enc->counts_per_rev  = counts_per_rev;
    enc->sample_period_s = sample_period_s;

    /* Detect 32-bit timer by checking ARR width */
    enc->is_32bit = (htim->Instance->ARR > 0xFFFFu);

    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
    enc->last_cnt = __HAL_TIM_GET_COUNTER(htim);
}

void Encoder_Update(Encoder_t *enc)
{
    uint32_t now = __HAL_TIM_GET_COUNTER(enc->htim);
    int32_t  delta;

    if (enc->is_32bit) {
        /* Signed arithmetic handles overflow naturally for 32-bit */
        delta = (int32_t)(now - enc->last_cnt);
    } else {
        /* 16-bit overflow handling */
        int32_t raw = (int32_t)now - (int32_t)enc->last_cnt;
        if (raw > 32767)  raw -= 65536;
        if (raw < -32768) raw += 65536;
        delta = raw;
    }

    enc->last_cnt    = now;
    enc->delta_ticks = delta;
    enc->total_ticks += delta;

    /* Velocity: ticks/sample → RPM and rad/s */
    float tps = (float)delta / enc->sample_period_s;   /* ticks per second */
    enc->rpm        = (tps / (float)enc->counts_per_rev) * 60.0f;
    enc->rad_per_sec = (tps / (float)enc->counts_per_rev) * TWO_PI;
}

void Encoder_ResetTicks(Encoder_t *enc)
{
    enc->total_ticks = 0;
    enc->delta_ticks = 0;
}

int32_t Encoder_GetTotalTicks(const Encoder_t *enc)
{
    return enc->total_ticks;
}

int32_t Encoder_GetDeltaTicks(const Encoder_t *enc)
{
    return enc->delta_ticks;
}

float Encoder_GetRPM(const Encoder_t *enc)
{
    return enc->rpm;
}

float Encoder_GetRadPerSec(const Encoder_t *enc)
{
    return enc->rad_per_sec;
}

float Encoder_GetLinearVelocity(const Encoder_t *enc, float wheel_radius_m)
{
    return enc->rad_per_sec * wheel_radius_m;
}
