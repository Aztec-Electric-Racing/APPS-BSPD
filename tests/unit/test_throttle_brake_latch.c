//
// Created by kami on 11/18/2026.
//

#include "unity.h"
#include "apps_logic.h"

/*
 * NextThrottleBrakeLatch(currentlyLatched, sensorsValid, appsFault,
 *                        brakesPressed, throttlePermille)
 *
 * Returns the NEW latch state (1 = latched / output inhibited).
 *
 *   Latches (1)   when sensors are invalid, OR an APPS fault is active,
 *                 OR the brake is pressed with throttle at 25.0% (250) or more.
 *   Clears  (0)   only when none of the above apply AND throttle is at 5.0% (50) or less.
 *   Otherwise     keeps its current state (hysteresis between 5% and 25%).
 *
 * Argument order is easy to mix up, so the tests use these names:
 */

#define UNLATCHED 0
#define LATCHED   1

#define SENSORS_OK  1
#define SENSORS_BAD 0

#define NO_FAULT 0
#define FAULT    1

#define BRAKE_OFF 0
#define BRAKE_ON  1

void setUp(void) {}
void tearDown(void) {}

/*
 *********************************************
 *
 *      CLEARING THE LATCH
 *
 *********************************************
 */

void test_latch_clears_at_zero_throttle(void)
{
    TEST_ASSERT_EQUAL_UINT8(UNLATCHED, NextThrottleBrakeLatch(LATCHED, SENSORS_OK, NO_FAULT, BRAKE_OFF, 0));
}

void test_latch_clears_exactly_at_clear_threshold(void)
{
    TEST_ASSERT_EQUAL_UINT8(UNLATCHED, NextThrottleBrakeLatch(LATCHED, SENSORS_OK, NO_FAULT, BRAKE_OFF, 50));
}

void test_latch_stays_one_above_clear_threshold(void)
{
    TEST_ASSERT_EQUAL_UINT8(LATCHED, NextThrottleBrakeLatch(LATCHED, SENSORS_OK, NO_FAULT, BRAKE_OFF, 51));
}

void test_latch_stays_latched_in_hysteresis_band(void)
{
    TEST_ASSERT_EQUAL_UINT8(LATCHED, NextThrottleBrakeLatch(LATCHED, SENSORS_OK, NO_FAULT, BRAKE_OFF, 100));
    TEST_ASSERT_EQUAL_UINT8(LATCHED, NextThrottleBrakeLatch(LATCHED, SENSORS_OK, NO_FAULT, BRAKE_OFF, 249));
}

void test_latch_stays_latched_at_high_throttle_after_brake_released(void)
{
    TEST_ASSERT_EQUAL_UINT8(LATCHED, NextThrottleBrakeLatch(LATCHED, SENSORS_OK, NO_FAULT, BRAKE_OFF, 1000));
}

void test_latch_clears_at_low_throttle_even_with_brake_pressed(void)
{
    TEST_ASSERT_EQUAL_UINT8(UNLATCHED, NextThrottleBrakeLatch(LATCHED, SENSORS_OK, NO_FAULT, BRAKE_ON, 50));
}

/*
 *********************************************
 *
 *      BRAKE + THROTTLE TRIP
 *
 *********************************************
 */

void test_latch_brake_with_throttle_below_trip_does_not_latch(void)
{
    TEST_ASSERT_EQUAL_UINT8(UNLATCHED, NextThrottleBrakeLatch(UNLATCHED, SENSORS_OK, NO_FAULT, BRAKE_ON, 249));
}

void test_latch_brake_with_throttle_exactly_at_trip_latches(void)
{
    TEST_ASSERT_EQUAL_UINT8(LATCHED, NextThrottleBrakeLatch(UNLATCHED, SENSORS_OK, NO_FAULT, BRAKE_ON, 250));
}

void test_latch_brake_with_full_throttle_latches(void)
{
    TEST_ASSERT_EQUAL_UINT8(LATCHED, NextThrottleBrakeLatch(UNLATCHED, SENSORS_OK, NO_FAULT, BRAKE_ON, 1000));
}

void test_latch_full_throttle_without_brake_does_not_latch(void)
{
    TEST_ASSERT_EQUAL_UINT8(UNLATCHED, NextThrottleBrakeLatch(UNLATCHED, SENSORS_OK, NO_FAULT, BRAKE_OFF, 1000));
}

void test_latch_unlatched_stays_unlatched_in_hysteresis_band(void)
{
    TEST_ASSERT_EQUAL_UINT8(UNLATCHED, NextThrottleBrakeLatch(UNLATCHED, SENSORS_OK, NO_FAULT, BRAKE_OFF, 100));
}

/*
 *********************************************
 *
 *      INVALID SENSORS
 *
 *********************************************
 */

void test_latch_invalid_sensors_latch(void)
{
    TEST_ASSERT_EQUAL_UINT8(LATCHED, NextThrottleBrakeLatch(UNLATCHED, SENSORS_BAD, NO_FAULT, BRAKE_OFF, 0));
}

void test_latch_invalid_sensors_do_not_clear_latch(void)
{
    TEST_ASSERT_EQUAL_UINT8(LATCHED, NextThrottleBrakeLatch(LATCHED, SENSORS_BAD, NO_FAULT, BRAKE_OFF, 0));
}

/*
 *********************************************
 *
 *      APPS FAULT (SENSOR MISMATCH)
 *
 *********************************************
 */

void test_latch_apps_fault_latches(void)
{
    TEST_ASSERT_EQUAL_UINT8(LATCHED, NextThrottleBrakeLatch(UNLATCHED, SENSORS_OK, FAULT, BRAKE_OFF, 0));
}

void test_latch_apps_fault_does_not_clear_latch(void)
{
    TEST_ASSERT_EQUAL_UINT8(LATCHED, NextThrottleBrakeLatch(LATCHED, SENSORS_OK, FAULT, BRAKE_OFF, 0));
}

/*
 *********************************************
 *
 *      SEQUENCES (STATE CARRIED FROM ONE CALL TO THE NEXT)
 *
 *********************************************
 */

void test_latch_sequence_trip_then_release_brake_then_lift_throttle(void)
{
    uint8_t latch = UNLATCHED;

    latch = NextThrottleBrakeLatch(latch, SENSORS_OK, NO_FAULT, BRAKE_ON, 400);   /* trips          */
    TEST_ASSERT_EQUAL_UINT8(LATCHED, latch);

    latch = NextThrottleBrakeLatch(latch, SENSORS_OK, NO_FAULT, BRAKE_OFF, 400);  /* brake released */
    TEST_ASSERT_EQUAL_UINT8(LATCHED, latch);                                      /* still latched  */

    latch = NextThrottleBrakeLatch(latch, SENSORS_OK, NO_FAULT, BRAKE_OFF, 100);  /* throttle eased */
    TEST_ASSERT_EQUAL_UINT8(LATCHED, latch);                                      /* above 5%       */

    latch = NextThrottleBrakeLatch(latch, SENSORS_OK, NO_FAULT, BRAKE_OFF, 50);   /* at 5%          */
    TEST_ASSERT_EQUAL_UINT8(UNLATCHED, latch);                                    /* cleared        */
}

void test_latch_sequence_power_up_latched_until_pedal_released(void)
{
    /* The firmware starts with the latch ON (throttleBrakeLatched = 1). */
    uint8_t latch = LATCHED;

    latch = NextThrottleBrakeLatch(latch, SENSORS_OK, NO_FAULT, BRAKE_OFF, 300);
    TEST_ASSERT_EQUAL_UINT8(LATCHED, latch);

    latch = NextThrottleBrakeLatch(latch, SENSORS_OK, NO_FAULT, BRAKE_OFF, 0);
    TEST_ASSERT_EQUAL_UINT8(UNLATCHED, latch);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_latch_clears_at_zero_throttle);
    RUN_TEST(test_latch_clears_exactly_at_clear_threshold);
    RUN_TEST(test_latch_stays_one_above_clear_threshold);
    RUN_TEST(test_latch_stays_latched_in_hysteresis_band);
    RUN_TEST(test_latch_stays_latched_at_high_throttle_after_brake_released);
    RUN_TEST(test_latch_clears_at_low_throttle_even_with_brake_pressed);
    RUN_TEST(test_latch_brake_with_throttle_below_trip_does_not_latch);
    RUN_TEST(test_latch_brake_with_throttle_exactly_at_trip_latches);
    RUN_TEST(test_latch_brake_with_full_throttle_latches);
    RUN_TEST(test_latch_full_throttle_without_brake_does_not_latch);
    RUN_TEST(test_latch_unlatched_stays_unlatched_in_hysteresis_band);
    RUN_TEST(test_latch_invalid_sensors_latch);
    RUN_TEST(test_latch_invalid_sensors_do_not_clear_latch);
    RUN_TEST(test_latch_apps_fault_latches);
    RUN_TEST(test_latch_apps_fault_does_not_clear_latch);
    RUN_TEST(test_latch_sequence_trip_then_release_brake_then_lift_throttle);
    RUN_TEST(test_latch_sequence_power_up_latched_until_pedal_released);

    return UNITY_END();
}
