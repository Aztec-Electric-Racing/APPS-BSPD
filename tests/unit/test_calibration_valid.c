//
// Created by kami on 11/18/2026.
//

#include "unity.h"
#include "apps_logic.h"

/*
 * Default calibration limits from LoadDefaultCalibration() in main.c
 * (sensor-side counts, not measured pedal values).
 * Defined here so the tests don't depend on firmware macros.
 *
 *   Sensor 1: 1914 to 2749
 *   Sensor 2: 1079 to 1914
 *
 * Argument order: lower1, upper1, lower2, upper2
 */
#define S1_LOWER 1914U
#define S1_UPPER 2749U
#define S2_LOWER 1079U
#define S2_UPPER 1914U

#define VALID   1
#define INVALID 0

void setUp(void) {}
void tearDown(void) {}

/*
 *********************************************
 *
 *      GOOD CALIBRATION
 *
 *********************************************
 */

void test_calibration_default_values_are_valid(void)
{
    TEST_ASSERT_EQUAL_UINT8(VALID, CalibrationIsValid(S1_LOWER, S1_UPPER, S2_LOWER, S2_UPPER));
}

void test_calibration_lower_limits_at_zero_are_valid(void)
{
    TEST_ASSERT_EQUAL_UINT8(VALID, CalibrationIsValid(0, S1_UPPER, 0, S2_UPPER));
}

void test_calibration_narrowest_valid_span_is_valid(void)
{
    TEST_ASSERT_EQUAL_UINT8(VALID, CalibrationIsValid(4094, 4095, S2_LOWER, S2_UPPER));
}

/*
 *********************************************
 *
 *      NEVER-CALIBRATED BOARD (ERASED FLASH)
 *
 *********************************************
 */

void test_calibration_erased_flash_is_invalid(void)
{
    TEST_ASSERT_EQUAL_UINT8(INVALID, CalibrationIsValid(0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU));
}

/*
 *********************************************
 *
 *      LOWER NOT BELOW UPPER
 *
 *********************************************
 */

void test_calibration_s1_lower_equals_upper_is_invalid(void)
{
    TEST_ASSERT_EQUAL_UINT8(INVALID, CalibrationIsValid(2000, 2000, S2_LOWER, S2_UPPER));
}

void test_calibration_s1_lower_above_upper_is_invalid(void)
{
    TEST_ASSERT_EQUAL_UINT8(INVALID, CalibrationIsValid(S1_UPPER, S1_LOWER, S2_LOWER, S2_UPPER));
}

void test_calibration_s2_lower_equals_upper_is_invalid(void)
{
    TEST_ASSERT_EQUAL_UINT8(INVALID, CalibrationIsValid(S1_LOWER, S1_UPPER, 1500, 1500));
}

void test_calibration_s2_lower_above_upper_is_invalid(void)
{
    TEST_ASSERT_EQUAL_UINT8(INVALID, CalibrationIsValid(S1_LOWER, S1_UPPER, S2_UPPER, S2_LOWER));
}

/*
 *********************************************
 *
 *      UPPER LIMIT OUTSIDE THE ADC RANGE
 *
 *********************************************
 */

void test_calibration_s1_upper_exactly_4095_is_valid(void)
{
    TEST_ASSERT_EQUAL_UINT8(VALID, CalibrationIsValid(S1_LOWER, 4095, S2_LOWER, S2_UPPER));
}

void test_calibration_s1_upper_4096_is_invalid(void)
{
    TEST_ASSERT_EQUAL_UINT8(INVALID, CalibrationIsValid(S1_LOWER, 4096, S2_LOWER, S2_UPPER));
}

void test_calibration_s2_upper_exactly_4095_is_valid(void)
{
    TEST_ASSERT_EQUAL_UINT8(VALID, CalibrationIsValid(S1_LOWER, S1_UPPER, S2_LOWER, 4095));
}

void test_calibration_s2_upper_4096_is_invalid(void)
{
    TEST_ASSERT_EQUAL_UINT8(INVALID, CalibrationIsValid(S1_LOWER, S1_UPPER, S2_LOWER, 4096));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_calibration_default_values_are_valid);
    RUN_TEST(test_calibration_lower_limits_at_zero_are_valid);
    RUN_TEST(test_calibration_narrowest_valid_span_is_valid);
    RUN_TEST(test_calibration_erased_flash_is_invalid);
    RUN_TEST(test_calibration_s1_lower_equals_upper_is_invalid);
    RUN_TEST(test_calibration_s1_lower_above_upper_is_invalid);
    RUN_TEST(test_calibration_s2_lower_equals_upper_is_invalid);
    RUN_TEST(test_calibration_s2_lower_above_upper_is_invalid);
    RUN_TEST(test_calibration_s1_upper_exactly_4095_is_valid);
    RUN_TEST(test_calibration_s1_upper_4096_is_invalid);
    RUN_TEST(test_calibration_s2_upper_exactly_4095_is_valid);
    RUN_TEST(test_calibration_s2_upper_4096_is_invalid);

    return UNITY_END();
}
