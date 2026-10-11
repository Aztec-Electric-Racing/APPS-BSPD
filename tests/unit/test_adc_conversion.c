//
// Created by kami on 10/11/2026.
//
#include "unity.h"
#include "apps_conversion.h"

void setUp(void) {}
void tearDown(void) {}

void test_adc_zero(void)
{
    TEST_ASSERT_EQUAL_UINT16(0, AdcToSensorCounts(0));
}

void test_adc_scaling(void)
{
    TEST_ASSERT_EQUAL_UINT16(1000, AdcToSensorCounts(720));
}

void test_adc_midrange(void)
{
    TEST_ASSERT_EQUAL_UINT16(2844, AdcToSensorCounts(2048));
}

void test_adc_clamping(void)
{
    TEST_ASSERT_EQUAL_UINT16(4095, AdcToSensorCounts(4095));
}

void test_adc_near_saturation(void) {
    TEST_ASSERT_EQUAL_UINT16(4094, AdcToSensorCounts(2948));
    TEST_ASSERT_EQUAL_UINT16(4095, AdcToSensorCounts(2949));
}

void test_adc_above_saturation(void)
{
    TEST_ASSERT_EQUAL_UINT16(4095, AdcToSensorCounts(3000));
    TEST_ASSERT_EQUAL_UINT16(4095, AdcToSensorCounts(4095));
}

void test_adc_rounding(void) {
    TEST_ASSERT_EQUAL_UINT16(3, AdcToSensorCounts(2));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_adc_zero);
    RUN_TEST(test_adc_scaling);
    RUN_TEST(test_adc_midrange);
    RUN_TEST(test_adc_clamping);
    RUN_TEST(test_adc_near_saturation);
    RUN_TEST(test_adc_above_saturation);
    RUN_TEST(test_adc_rounding);

    return UNITY_END();
}