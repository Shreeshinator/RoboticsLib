/**
 * @file    stm32_hal_include.h
 * @brief   Portable HAL header resolver
 *
 * Pulls in the correct STM32 HAL header based on the
 * ROBOT_HAL_INCLUDE macro defined in robot_config.h.
 * Never include this file directly — use robotics.h.
 */

#ifndef STM32_HAL_INCLUDE_H
#define STM32_HAL_INCLUDE_H

#include "robot_config.h"
#include ROBOT_HAL_INCLUDE

#endif /* STM32_HAL_INCLUDE_H */
