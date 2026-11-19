//
// Created by kami on 11/18/2026.
//

#include "unity.h"
#include "apps_logic.h"

/*
 * UpdateMismatchFault(timer, permille1, permille2, nowMs)
 *
 * Fault rule: the two sensors must disagree by MORE than 100 permille (10%)
 * for MORE than 100 ms. Exactly 100 permille or exactly 100 ms is still OK.
 *
 * Time is passed in by the test, so no real clock is needed.
 */

#define FAULT 1
#define OK    0

static MismatchTimer_t timer;

void setUp(void)
{
    MismatchTimerReset(&timer);
}

void tearDown(void) {}

/*
 *********************************************
 *
 *      SENSORS AGREE
 *
 *********************************************
 */

void test_mismatch_identical_readings_no_fault(void)
{
    TEST_ASSERT_EQUAL_UINT8(OK, UpdateMismatchFault(&timer, 500, 500, 0));
    TEST_ASSERT_EQUAL_UINT8(OK, UpdateMismatchFault(&timer, 500, 500, 1000));
}

void test_mismatch_small_difference_no_fault(void)
{
    TEST_ASSERT_EQUAL_UINT8(OK, UpdateMismatchFault(&timer, 500, 550, 0));
    TEST_ASSERT_EQUAL_UINT8(OK, UpdateMismatchFault(&timer, 500, 550, 5000));
}

/*
 *********************************************
 *
 *      DIFFERENCE THRESHOLD (100 PERMILLE)
 *
 *********************************************
 */

void test_mismatch_exactly_at_limit_never_faults(void)
{
    TEST_ASSERT_EQUAL_UINT8(OK, UpdateMismatchFault(&timer, 500, 600, 0));
    TEST_ASSERT_EQUAL_UINT8(OK, UpdateMismatchFault(&timer, 500, 600, 10000));
}

void test_mismatch_one_over_limit_faults_after_timeout(void)
{
    TEST_ASSERT_EQUAL_UINT8(OK,    UpdateMismatchFault(&timer, 500, 601, 0));
    TEST_ASSERT_EQUAL_UINT8(FAULT, UpdateMismatchFault(&timer, 500, 601, 101));
}

void test_mismatch_sensor1_higher_also_faults(void)
{
    TEST_ASSERT_EQUAL_UINT8(OK,    UpdateMismatchFault(&timer, 601, 500, 0));
    TEST_ASSERT_EQUAL_UINT8(FAULT, UpdateMismatchFault(&timer, 601, 500, 101));
}

/*
 *********************************************
 *
 *      TIME THRESHOLD (100 MS)
 *
 *********************************************
 */

void test_mismatch_first_sample_does_not_fault(void)
{
    TEST_ASSERT_EQUAL_UINT8(OK, UpdateMismatchFault(&timer, 100, 800, 5000));
}

void test_mismatch_99ms_no_fault(void)
{
    UpdateMismatchFault(&timer, 100, 800, 0);
    TEST_ASSERT_EQUAL_UINT8(OK, UpdateMismatchFault(&timer, 100, 800, 99));
}

void test_mismatch_exactly_100ms_no_fault(void)
{
    UpdateMismatchFault(&timer, 100, 800, 0);
    TEST_ASSERT_EQUAL_UINT8(OK, UpdateMismatchFault(&timer, 100, 800, 100));
}

void test_mismatch_101ms_faults(void)
{
    UpdateMismatchFault(&timer, 100, 800, 0);
    TEST_ASSERT_EQUAL_UINT8(FAULT, UpdateMismatchFault(&timer, 100, 800, 101));
}

void test_mismatch_fault_stays_while_mismatch_continues(void)
{
    UpdateMismatchFault(&timer, 100, 800, 0);
    UpdateMismatchFault(&timer, 100, 800, 150);
    TEST_ASSERT_EQUAL_UINT8(FAULT, UpdateMismatchFault(&timer, 100, 800, 5000));
}

/*
 *********************************************
 *
 *      RECOVERY AND RESTART
 *
 *********************************************
 */

void test_mismatch_clears_when_sensors_agree_again(void)
{
    UpdateMismatchFault(&timer, 100, 800, 0);
    TEST_ASSERT_EQUAL_UINT8(FAULT, UpdateMismatchFault(&timer, 100, 800, 150));
    TEST_ASSERT_EQUAL_UINT8(OK,    UpdateMismatchFault(&timer, 500, 500, 160));
}

void test_mismatch_timer_restarts_after_short_glitch(void)
{
    /* Mismatch at t=0, agrees at t=50, mismatches again at t=60.
     * Only 60 ms have passed since the NEW mismatch began, not 120. */
    UpdateMismatchFault(&timer, 100, 800, 0);
    UpdateMismatchFault(&timer, 500, 500, 50);
    UpdateMismatchFault(&timer, 100, 800, 60);
    TEST_ASSERT_EQUAL_UINT8(OK, UpdateMismatchFault(&timer, 100, 800, 120));
}

void test_mismatch_timer_restarts_after_fault_clears(void)
{
    UpdateMismatchFault(&timer, 100, 800, 0);
    UpdateMismatchFault(&timer, 100, 800, 150);
    UpdateMismatchFault(&timer, 500, 500, 200);
    UpdateMismatchFault(&timer, 100, 800, 300);
    TEST_ASSERT_EQUAL_UINT8(OK,    UpdateMismatchFault(&timer, 100, 800, 400));
    TEST_ASSERT_EQUAL_UINT8(FAULT, UpdateMismatchFault(&timer, 100, 800, 401));
}

void test_mismatch_reset_forgets_mismatch_in_progress(void)
{
    /* Sensors go invalid at t=50 (main loop resets the timer). */
    UpdateMismatchFault(&timer, 100, 800, 0);
    MismatchTimerReset(&timer);
    UpdateMismatchFault(&timer, 100, 800, 60);
    TEST_ASSERT_EQUAL_UINT8(OK, UpdateMismatchFault(&timer, 100, 800, 120));
}

/*
 *********************************************
 *
 *      MILLISECOND TICK WRAPS AROUND
 *
 *********************************************
 */

void test_mismatch_works_across_tick_wraparound(void)
{
    /* Starts 16 ms before the 32-bit counter wraps to 0. */
    UpdateMismatchFault(&timer, 100, 800, 0xFFFFFFF0U);
    TEST_ASSERT_EQUAL_UINT8(OK,    UpdateMismatchFault(&timer, 100, 800, 0x00000040U)); /* 80 ms elapsed  */
    TEST_ASSERT_EQUAL_UINT8(FAULT, UpdateMismatchFault(&timer, 100, 800, 0x00000070U)); /* 128 ms elapsed */
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_mismatch_identical_readings_no_fault);
    RUN_TEST(test_mismatch_small_difference_no_fault);
    RUN_TEST(test_mismatch_exactly_at_limit_never_faults);
    RUN_TEST(test_mismatch_one_over_limit_faults_after_timeout);
    RUN_TEST(test_mismatch_sensor1_higher_also_faults);
    RUN_TEST(test_mismatch_first_sample_does_not_fault);
    RUN_TEST(test_mismatch_99ms_no_fault);
    RUN_TEST(test_mismatch_exactly_100ms_no_fault);
    RUN_TEST(test_mismatch_101ms_faults);
    RUN_TEST(test_mismatch_fault_stays_while_mismatch_continues);
    RUN_TEST(test_mismatch_clears_when_sensors_agree_again);
    RUN_TEST(test_mismatch_timer_restarts_after_short_glitch);
    RUN_TEST(test_mismatch_timer_restarts_after_fault_clears);
    RUN_TEST(test_mismatch_reset_forgets_mismatch_in_progress);
    RUN_TEST(test_mismatch_works_across_tick_wraparound);

    return UNITY_END();
}
