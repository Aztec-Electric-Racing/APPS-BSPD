//
// Created by kami on 11/18/2026.
//
#include "unity.h"
#include "apps_logic.h"

void setUp(void) {}
void tearDown(void) {}

void test_clampdac_normal(void) {
    TEST_ASSERT_EQUAL_UINT32(2048, ClampDac(2048));
}

void test_clampdac_zero(void) {
    TEST_ASSERT_EQUAL_UINT32(0, ClampDac(0));
}

void test_clampdac_one(void) {
    TEST_ASSERT_EQUAL_UINT32(1, ClampDac(1));
}

void test_clamp_dac_negative_one_returns_zero(void)
{
    TEST_ASSERT_EQUAL_UINT32(0, ClampDac(-1));
}

void test_large_neg_returns_zero(void) {
    TEST_ASSERT_EQUAL_UINT32(0, ClampDac(-100000));
}

void test_extreme_neg(void) {
    TEST_ASSERT_EQUAL_UINT32(0, ClampDac(-2147483648));
}

void test_one_below_max(void) {
    TEST_ASSERT_EQUAL_UINT32(4094, ClampDac(4094));
}

void test_max(void) {
    TEST_ASSERT_EQUAL_UINT32(4095, ClampDac(4095));
}

void test_just_above_max(void) {
    TEST_ASSERT_EQUAL_UINT32(4095, ClampDac(4096));
}

void test_large_positive(void) {
    TEST_ASSERT_EQUAL_UINT32(4095, ClampDac(1000000));
}

void test_extreme_positive(void) {
    TEST_ASSERT_EQUAL_UINT32(4095, ClampDac(2147483647));
}


int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_clampdac_normal);
    RUN_TEST(test_clampdac_zero);
    RUN_TEST(test_clampdac_one);
    RUN_TEST(test_clamp_dac_negative_one_returns_zero);
    RUN_TEST(test_large_neg_returns_zero);
    RUN_TEST(test_extreme_neg);
    RUN_TEST(test_one_below_max);
    RUN_TEST(test_max);
    RUN_TEST(test_just_above_max);
    RUN_TEST(test_large_positive);
    RUN_TEST(test_extreme_positive);




    return UNITY_END();
}
