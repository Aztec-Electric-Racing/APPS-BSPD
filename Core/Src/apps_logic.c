//
// Created by kami on 10/11/2026.
//

#include "apps_logic.h"

uint16_t GetSensorPermille(uint16_t sensor, uint32_t lower, uint16_t range) // turns a sensor reading into pedal position from 0-1k.
{
    if ((range == 0U) || (sensor <= lower)) return 0U;
    uint32_t permille = ((uint32_t)(sensor - lower) * 1000U) / range;
    return (uint16_t)((permille > 1000U) ? 1000U : permille);
}
