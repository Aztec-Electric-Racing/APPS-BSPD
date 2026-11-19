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
 *      NORMAL READINGS
 *
 *********************************************
 */

void test_sensors_valid_both_mid_range(void)
{
    TEST_ASSERT_EQUAL_UINT8(VALID, AreSensorsValid(2331, 1496, S1_LOWER, S1_UPPER, S2_LOWER, S2_UPPER));
}

/*
 *********************************************
 *
 *      SENSOR 1 BOUNDARIES
 *
 *********************************************
 */

void test_sensors_valid_s1_exactly_at_lower_is_valid(void)
{
    TEST_ASSERT_EQUAL_UINT8(VALID, AreSensorsValid(1914, 1496, S1_LOWER, S1_UPPER, S2_LOWER, S2_UPPER));
}

void test_sensors_valid_s1_one_below_lower_is_invalid(void)
{
    TEST_ASSERT_EQUAL_UINT8(INVALID, AreSensorsValid(1913, 1496, S1_LOWER, S1_UPPER, S2_LOWER, S2_UPPER));
}

void test_sensors_valid_s1_exactly_at_upper_is_valid(void)
{
    TEST_ASSERT_EQUAL_UINT8(VALID, AreSensorsValid(2749, 1496, S1_LOWER, S1_UPPER, S2_LOWER, S2_UPPER));
}

void test_sensors_valid_s1_one_above_upper_is_invalid(void)
{
    TEST_ASSERT_EQUAL_UINT8(INVALID, AreSensorsValid(2750, 1496, S1_LOWER, S1_UPPER, S2_LOWER, S2_UPPER));
}

/*
 *********************************************
 *
 *      SENSOR 2 BOUNDARIES
 *
 *********************************************
 */

void test_sensors_valid_s2_exactly_at_lower_is_valid(void)
{
    TEST_ASSERT_EQUAL_UINT8(VALID, AreSensorsValid(2331, 1079, S1_LOWER, S1_UPPER, S2_LOWER, S2_UPPER));
}

void test_sensors_valid_s2_one_below_lower_is_invalid(void)
{
    TEST_ASSERT_EQUAL_UINT8(INVALID, AreSensorsValid(2331, 1078, S1_LOWER, S1_UPPER, S2_LOWER, S2_UPPER));
}

void test_sensors_valid_s2_exactly_at_upper_is_valid(void)
{
    TEST_ASSERT_EQUAL_UINT8(VALID, AreSensorsValid(2331, 1914, S1_LOWER, S1_UPPER, S2_LOWER, S2_UPPER));
}

void test_sensors_valid_s2_one_above_upper_is_invalid(void)
{
    TEST_ASSERT_EQUAL_UINT8(INVALID, AreSensorsValid(2331, 1915, S1_LOWER, S1_UPPER, S2_LOWER, S2_UPPER));
}

/*
 *********************************************
 *
 *      FAULT-LIKE READINGS
 *
 *********************************************
 */

void test_sensors_valid_s1_unplugged_reads_zero_is_invalid(void)
{
    TEST_ASSERT_EQUAL_UINT8(INVALID, AreSensorsValid(0, 1496, S1_LOWER, S1_UPPER, S2_LOWER, S2_UPPER));
}

void test_sensors_valid_s2_shorted_high_is_invalid(void)
{
    TEST_ASSERT_EQUAL_UINT8(INVALID, AreSensorsValid(2331, 4095, S1_LOWER, S1_UPPER, S2_LOWER, S2_UPPER));
}

void test_sensors_valid_both_out_of_range_is_invalid(void)
{
    TEST_ASSERT_EQUAL_UINT8(INVALID, AreSensorsValid(0, 4095, S1_LOWER, S1_UPPER, S2_LOWER, S2_UPPER));
}

/*
 *********************************************
 *
 *      CORRUPT CALIBRATION (LOWER ABOVE UPPER)
 *
 *   With swapped limits no reading can satisfy
 *   both checks, so everything is invalid.
 *
 *********************************************
 */

void test_sensors_valid_s1_limits_inverted_is_invalid(void)
{
    TEST_ASSERT_EQUAL_UINT8(INVALID, AreSensorsValid(2331, 1496, S1_UPPER, S1_LOWER, S2_LOWER, S2_UPPER));
}

void test_sensors_valid_s2_limits_inverted_is_invalid(void)
{
    TEST_ASSERT_EQUAL_UINT8(INVALID, AreSensorsValid(2331, 1496, S1_LOWER, S1_UPPER, S2_UPPER, S2_LOWER));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_sensors_valid_both_mid_range);
    RUN_TEST(test_sensors_valid_s1_exactly_at_lower_is_valid);
    RUN_TEST(test_sensors_valid_s1_one_below_lower_is_invalid);
    RUN_TEST(test_sensors_valid_s1_exactly_at_upper_is_valid);
    RUN_TEST(test_sensors_valid_s1_one_above_upper_is_invalid);
    RUN_TEST(test_sensors_valid_s2_exactly_at_lower_is_valid);
    RUN_TEST(test_sensors_valid_s2_one_below_lower_is_invalid);
    RUN_TEST(test_sensors_valid_s2_exactly_at_upper_is_valid);
    RUN_TEST(test_sensors_valid_s2_one_above_upper_is_invalid);
    RUN_TEST(test_sensors_valid_s1_unplugged_reads_zero_is_invalid);
    RUN_TEST(test_sensors_valid_s2_shorted_high_is_invalid);
    RUN_TEST(test_sensors_valid_both_out_of_range_is_invalid);
    RUN_TEST(test_sensors_valid_s1_limits_inverted_is_invalid);
    RUN_TEST(test_sensors_valid_s2_limits_inverted_is_invalid);

    return UNITY_END();
}
