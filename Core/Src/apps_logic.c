//
// Created by kami on 10/11/2026.
//

#include "apps_logic.h"
#define ADC_MAX_COUNTS          4095
#define DAC_MAX_COUNTS          4095

uint16_t GetSensorPermille(uint16_t sensor, uint32_t lower, uint16_t range) // turns a sensor reading into pedal position from 0-1k.
{
    if ((range == 0U) || (sensor <= lower)) return 0U;
    uint32_t permille = ((uint32_t)(sensor - lower) * 1000U) / range;
    return (uint16_t)((permille > 1000U) ? 1000U : permille);
}

uint32_t ClampDac(int32_t value)
{
    if (value < 0) return 0;
    if (value > DAC_MAX_COUNTS) return DAC_MAX_COUNTS;
    return (uint32_t)value;
}