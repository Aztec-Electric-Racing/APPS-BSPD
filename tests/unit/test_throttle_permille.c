//
// Created by kami on 11/18/2026.
//

#include "unity.h"
#include "apps_logic.h"

/*
 * Default calibration from LoadDefaultCalibration() in main.c
 * (sensor-side counts, not measured pedal values).
 * Defined here so the tests don't depend on firmware macros.
 */
#define S1_LOWER 1914U
#define S1_RANGE 835U
#define S2_LOWER 1079U
#define S2_RANGE 835U

void setUp(void) {}
void tearDown(void) {}

/*
 *********************************************
 *
 *      BOTH SENSORS AGREE
 *
 *********************************************
 */

void test_throttle_both_released_returns_zero(void)
{
    TEST_ASSERT_EQUAL_UINT16(0, GetThrottlePermille(1914, 1079, S1_LOWER, S1_RANGE, S2_LOWER, S2_RANGE));
}

void test_throttle_both_mid_travel(void)
{
    TEST_ASSERT_EQUAL_UINT16(499, GetThrottlePermille(2331, 1496, S1_LOWER, S1_RANGE, S2_LOWER, S2_RANGE));
}

void test_throttle_both_fully_pressed_returns_1000(void)
{
    TEST_ASSERT_EQUAL_UINT16(1000, GetThrottlePermille(2749, 1914, S1_LOWER, S1_RANGE, S2_LOWER, S2_RANGE));
}

/*
 *********************************************
 *
 *      HIGHER SENSOR WINS
 *
 *********************************************
 */

void test_throttle_sensor1_higher_wins(void)
{
    TEST_ASSERT_EQUAL_UINT16(748, GetThrottlePermille(2539, 1496, S1_LOWER, S1_RANGE, S2_LOWER, S2_RANGE));
}

void test_throttle_sensor2_higher_wins(void)
{
    TEST_ASSERT_EQUAL_UINT16(850, GetThrottlePermille(2331, 1789, S1_LOWER, S1_RANGE, S2_LOWER, S2_RANGE));
}

/*
 *********************************************
 *
 *      A LOW SENSOR MUST NOT MASK A HIGH ONE
 *
 *********************************************
 */

void test_throttle_sensor1_reads_zero_sensor2_still_counts(void)
{
    TEST_ASSERT_EQUAL_UINT16(850, GetThrottlePermille(0, 1789, S1_LOWER, S1_RANGE, S2_LOWER, S2_RANGE));
}

void test_throttle_sensor2_reads_zero_sensor1_still_counts(void)
{
    TEST_ASSERT_EQUAL_UINT16(748, GetThrottlePermille(2539, 0, S1_LOWER, S1_RANGE, S2_LOWER, S2_RANGE));
}

/*
 *********************************************
 *
 *      CLAMPING
 *
 *********************************************
 */

void test_throttle_sensor1_above_range_clamps_to_1000(void)
{
    TEST_ASSERT_EQUAL_UINT16(1000, GetThrottlePermille(3500, 1079, S1_LOWER, S1_RANGE, S2_LOWER, S2_RANGE));
}

void test_throttle_sensor2_above_range_clamps_to_1000(void)
{
    TEST_ASSERT_EQUAL_UINT16(1000, GetThrottlePermille(1914, 3000, S1_LOWER, S1_RANGE, S2_LOWER, S2_RANGE));
}

/*
 *********************************************
 *
 *      ZERO RANGE ON ONE OR BOTH SENSORS
 *
 *********************************************
 */

void test_throttle_sensor1_range_zero_sensor2_drives_result(void)
{
    TEST_ASSERT_EQUAL_UINT16(499, GetThrottlePermille(2500, 1496, S1_LOWER, 0, S2_LOWER, S2_RANGE));
}

void test_throttle_sensor2_range_zero_sensor1_drives_result(void)
{
    TEST_ASSERT_EQUAL_UINT16(499, GetThrottlePermille(2331, 1900, S1_LOWER, S1_RANGE, S2_LOWER, 0));
}

void test_throttle_both_ranges_zero_returns_zero(void)
{
    TEST_ASSERT_EQUAL_UINT16(0, GetThrottlePermille(2500, 1900, S1_LOWER, 0, S2_LOWER, 0));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_throttle_both_released_returns_zero);
    RUN_TEST(test_throttle_both_mid_travel);
    RUN_TEST(test_throttle_both_fully_pressed_returns_1000);
    RUN_TEST(test_throttle_sensor1_higher_wins);
    RUN_TEST(test_throttle_sensor2_higher_wins);
    RUN_TEST(test_throttle_sensor1_reads_zero_sensor2_still_counts);
    RUN_TEST(test_throttle_sensor2_reads_zero_sensor1_still_counts);
    RUN_TEST(test_throttle_sensor1_above_range_clamps_to_1000);
    RUN_TEST(test_throttle_sensor2_above_range_clamps_to_1000);
    RUN_TEST(test_throttle_sensor1_range_zero_sensor2_drives_result);
    RUN_TEST(test_throttle_sensor2_range_zero_sensor1_drives_result);
    RUN_TEST(test_throttle_both_ranges_zero_returns_zero);

    return UNITY_END();
}
