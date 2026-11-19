#include "unity.h"
#include "apps_logic.h"

void setUp(void) {}
void tearDown(void) {}


/*
*********************************************
 *
 *      EDGES AND GUARDS
 *
 *********************************************
 */

void test_permille_range_zero_returns_zero(void)
{
    TEST_ASSERT_EQUAL_UINT16(0, GetSensorPermille(1500, 1000, 0));
}

void test_permille_below_bottom(void)
{
    TEST_ASSERT_EQUAL_UINT16(0, GetSensorPermille(500, 1000, 1000));
}

void test_permille_exact_bottom(void)
{
    TEST_ASSERT_EQUAL_UINT16(0, GetSensorPermille(1000, 1000, 1000));
}

void test_permille_nearly_above_bottom(void)
{
    TEST_ASSERT_EQUAL_UINT16(1, GetSensorPermille(1001, 1000, 1000));
}

/*
 *********************************************
 *
 *      NORMAL TRAVEL & TOP
 *
 *********************************************
 */

void test_permille_middle(void)
{
    TEST_ASSERT_EQUAL_UINT16(500, GetSensorPermille(1500, 1000, 1000));
}

void test_permille_top_exact(void)
{
    TEST_ASSERT_EQUAL_UINT16(1000, GetSensorPermille(2000, 1000, 1000));
}

void test_permille_above_top(void) //clamp
{
    TEST_ASSERT_EQUAL_UINT16(1000, GetSensorPermille(2500, 1000, 1000));
}

/*
 *********************************************
 *
 *      TRUNCATION AND AWKWARD RANGES
 *
 *********************************************
 */

void test_permille_truncates_not_rounds(void)
{
    TEST_ASSERT_EQUAL_UINT16(119, GetSensorPermille(1100, 1000, 836));
}

void test_permille_clean_fifty_awkward_range(void)
{
    TEST_ASSERT_EQUAL_UINT16(500, GetSensorPermille(1418, 1000, 836));
}

void test_permille_top_awkward_range(void)
{
    TEST_ASSERT_EQUAL_UINT16(1000, GetSensorPermille(1836, 1000, 836));
}

void test_permille_small_round_step(void)
{
    TEST_ASSERT_EQUAL_UINT16(0, GetSensorPermille(1003, 1000, 4000));
}

void test_permille_first_step_one(void)
{
    TEST_ASSERT_EQUAL_UINT16(1, GetSensorPermille(1004, 1000, 4000));
}

/*
 *********************************************
 *
 *      DEFAULT CALIBRATION VALUES
 *
 *   From LoadDefaultCalibration() in main.c (sensor-side counts):
 *     Sensor 1: lower 1914, upper 2749, range 835
 *     Sensor 2: lower 1079, upper 1914, range 835
 *   These are the firmware's defaults, not measured pedal values.
 *
 *********************************************
 */

void test_permille_sensor1_released(void)
{
    TEST_ASSERT_EQUAL_UINT16(0, GetSensorPermille(1914, 1914, 835));
}

void test_permille_sensor1_mid_travel(void)
{
    TEST_ASSERT_EQUAL_UINT16(499, GetSensorPermille(2331, 1914, 835));
}

void test_permille_sensor1_fully_pressed(void)
{
    TEST_ASSERT_EQUAL_UINT16(1000, GetSensorPermille(2749, 1914, 835));
}

void test_permille_sensor2_released(void)
{
    TEST_ASSERT_EQUAL_UINT16(0, GetSensorPermille(1079, 1079, 835));
}

void test_permille_sensor2_mid_travel(void)
{
    TEST_ASSERT_EQUAL_UINT16(499, GetSensorPermille(1496, 1079, 835));
}

void test_permille_sensor2_fully_pressed(void)
{
    TEST_ASSERT_EQUAL_UINT16(1000, GetSensorPermille(1914, 1079, 835));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_permille_range_zero_returns_zero);
    RUN_TEST(test_permille_below_bottom);
    RUN_TEST(test_permille_exact_bottom);
    RUN_TEST(test_permille_nearly_above_bottom);
    RUN_TEST(test_permille_middle);
    RUN_TEST(test_permille_top_exact);
    RUN_TEST(test_permille_above_top);
    RUN_TEST(test_permille_truncates_not_rounds);
    RUN_TEST(test_permille_clean_fifty_awkward_range);
    RUN_TEST(test_permille_top_awkward_range);
    RUN_TEST(test_permille_small_round_step);
    RUN_TEST(test_permille_first_step_one);
    RUN_TEST(test_permille_sensor1_released);
    RUN_TEST(test_permille_sensor1_mid_travel);
    RUN_TEST(test_permille_sensor1_fully_pressed);
    RUN_TEST(test_permille_sensor2_released);
    RUN_TEST(test_permille_sensor2_mid_travel);
    RUN_TEST(test_permille_sensor2_fully_pressed);

    return UNITY_END();
}
