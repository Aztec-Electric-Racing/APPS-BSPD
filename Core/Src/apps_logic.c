//
// Created by kami on 10/11/2026.
//

#include "apps_logic.h"
#define ADC_MAX_COUNTS          4095
#define DAC_MAX_COUNTS          4095


/* Normalize each redundant sensor against its own calibrated endpoints. Use
 * the higher reading so a low-reading channel cannot mask a high throttle. */
uint16_t GetThrottlePermille(uint16_t sensor1, uint16_t sensor2,
                             uint32_t lower1, uint16_t range1,
                             uint32_t lower2, uint16_t range2)
{
    uint32_t p1 = GetSensorPermille(sensor1, lower1, range1);
    uint32_t p2 = GetSensorPermille(sensor2, lower2, range2);
    uint32_t higher = (p1 > p2) ? p1 : p2;
    return (uint16_t)((higher > 1000U) ? 1000U : higher);
}

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

/* Both sensors must sit inside their own calibrated [lower, upper] limits (inclusive). */
uint8_t AreSensorsValid(uint16_t s1, uint16_t s2,
                        uint32_t lower1, uint32_t upper1,
                        uint32_t lower2, uint32_t upper2)
{
    return (uint8_t)((lower1 <= s1) && (s1 <= upper1) &&
                     (lower2 <= s2) && (s2 <= upper2));
}

/* Erased flash reads as 0xFFFFFFFF, so this also catches a board that has never been calibrated.
 * Each sensor needs lower < upper and upper within the ADC range. */
uint8_t CalibrationIsValid(uint32_t lower1, uint32_t upper1,
                           uint32_t lower2, uint32_t upper2)
{
    return (uint8_t)((lower1 < upper1) && (upper1 <= ADC_MAX_COUNTS) &&
                     (lower2 < upper2) && (upper2 <= ADC_MAX_COUNTS));
}

void MismatchTimerReset(MismatchTimer_t *timer)
{
    timer->timing = 0U;
    timer->startTick = 0U;
}

uint8_t UpdateMismatchFault(MismatchTimer_t *timer,
                            uint16_t permille1, uint16_t permille2,
                            uint32_t nowMs)
{
    uint16_t mismatch = (permille1 > permille2) ?
                        (uint16_t)(permille1 - permille2) : (uint16_t)(permille2 - permille1);

    if (mismatch > APPS_MISMATCH_PERMILLE)
    {
        if (!timer->timing)
        {
            timer->timing = 1U;
            timer->startTick = nowMs;
        }
        /* Unsigned subtraction stays correct when the millisecond tick wraps around. */
        return (uint8_t)((nowMs - timer->startTick) > APPS_MISMATCH_TIME_MS);
    }

    timer->timing = 0U;
    return 0U;
}

uint8_t NextThrottleBrakeLatch(uint8_t currentlyLatched,
                               uint8_t sensorsValid,
                               uint8_t appsFault,
                               uint8_t brakesPressed,
                               uint16_t throttlePermille)
{
    if (!sensorsValid || appsFault || (brakesPressed && (throttlePermille >= APPS_TRIP_PERMILLE)))
    {
        return 1U;
    }
    if (throttlePermille <= APPS_CLEAR_PERMILLE)
    {
        return 0U;
    }
    return currentlyLatched;
}
