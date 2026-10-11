#include "apps_conversion.h"

#define ADC_MAX_COUNTS          4095U
#define APPS_INPUT_DIVIDER_NUM  72U // input voltage-divider ratio
#define APPS_INPUT_DIVIDER_DEN  100U

uint16_t AdcToSensorCounts(uint16_t adcCounts)
{
    uint32_t sensorCounts =
        ((uint32_t)adcCounts * APPS_INPUT_DIVIDER_DEN +
         (APPS_INPUT_DIVIDER_NUM / 2U)) /
        APPS_INPUT_DIVIDER_NUM;

    if (sensorCounts > ADC_MAX_COUNTS)
        sensorCounts = ADC_MAX_COUNTS;

    return (uint16_t)sensorCounts;
}