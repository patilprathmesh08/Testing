/**
 ******************************************************************************
 * @file    test_motor_telemetry_math.c
 * @brief   Unit tests for motor_telemetry_math.c (see main.c extraction).
 *
 * These run on the HOST (plain gcc), not the ARM target - that's the whole
 * point: this is pure math with no hardware/HAL dependency, so it can be
 * verified automatically on every CI run without a physical STM32 board.
 *
 * NOTE on constants: U_RPM, SPEED_UNIT, and CURRENT_CONV_FACTOR below are
 * simple round test values chosen to make the expected results easy to
 * verify by hand. They are NOT necessarily this board's real values from
 * parameters_conversion.h - what's under test is the conversion FORMULA,
 * which is identical regardless of the specific constants used at runtime.
 ******************************************************************************
 */

#include "unity.h"
#include "motor_telemetry_math.h"

void setUp(void)
{
}

void tearDown(void)
{
}

/* ---------- speed_unit_to_rpm ---------- */

void test_speed_unit_to_rpm_positive_value(void)
{
    /* 50 units * 100 (U_RPM) / 10 (SPEED_UNIT) = 500 RPM */
    TEST_ASSERT_EQUAL_INT32(500, speed_unit_to_rpm(50, 100, 10));
}

void test_speed_unit_to_rpm_negative_value(void)
{
    /* Motor spinning the other direction should give a negative RPM */
    TEST_ASSERT_EQUAL_INT32(-200, speed_unit_to_rpm(-20, 100, 10));
}

void test_speed_unit_to_rpm_zero_is_zero(void)
{
    TEST_ASSERT_EQUAL_INT32(0, speed_unit_to_rpm(0, 100, 10));
}

/* ---------- s16_angle_to_degrees ---------- */

void test_s16_angle_to_degrees_zero(void)
{
    TEST_ASSERT_EQUAL_INT32(0, s16_angle_to_degrees(0));
}

void test_s16_angle_to_degrees_quarter_turn(void)
{
    /* 16384 s16 = 90 electrical degrees */
    TEST_ASSERT_EQUAL_INT32(90, s16_angle_to_degrees(16384));
}

void test_s16_angle_to_degrees_negative_quarter_turn(void)
{
    /* -16384 s16 = -90 electrical degrees (this is g_el_offset_s16!) */
    TEST_ASSERT_EQUAL_INT32(-90, s16_angle_to_degrees(-16384));
}

void test_s16_angle_to_degrees_half_turn(void)
{
    /* angle_s16 is int16_t (range -32768..32767), so +32768 is not a valid
     * input - it wraps. -32768 is the valid representation of a half turn
     * in this signed scheme, and correctly gives -180, not +180. */
    TEST_ASSERT_EQUAL_INT32(-180, s16_angle_to_degrees(-32768));
}

void test_s16_angle_to_degrees_near_positive_max_truncates(void)
{
    /* 32767 is the largest valid int16_t value, just short of +180 degrees.
     * Integer division truncates toward zero, so this lands on 179, not
     * 180. This documents the existing truncation behavior rather than
     * changing it. */
    TEST_ASSERT_EQUAL_INT32(179, s16_angle_to_degrees(32767));
}

/* ---------- counts_to_milliamps ---------- */

void test_counts_to_milliamps_positive(void)
{
    /* 1000 counts / 500.0 conv factor * 1000 = 2000 mA */
    TEST_ASSERT_EQUAL_INT32(2000, counts_to_milliamps(1000, 500.0f));
}

void test_counts_to_milliamps_negative(void)
{
    TEST_ASSERT_EQUAL_INT32(-2000, counts_to_milliamps(-1000, 500.0f));
}

void test_counts_to_milliamps_zero(void)
{
    TEST_ASSERT_EQUAL_INT32(0, counts_to_milliamps(0, 500.0f));
}

/* ---------- current_magnitude_milliamps ---------- */

void test_current_magnitude_milliamps_3_4_5_triangle(void)
{
    /* Classic 3-4-5 right triangle scaled up: sqrt(300^2 + 400^2) = 500 */
    TEST_ASSERT_EQUAL_INT32(1000, current_magnitude_milliamps(300, 400, 500.0f));
}

void test_current_magnitude_milliamps_zero_current(void)
{
    TEST_ASSERT_EQUAL_INT32(0, current_magnitude_milliamps(0, 0, 500.0f));
}

void test_current_magnitude_milliamps_only_q_axis(void)
{
    /* Pure torque current, no d-axis (the "commutation is correct" case) */
    TEST_ASSERT_EQUAL_INT32(1000, current_magnitude_milliamps(500, 0, 500.0f));
}

/* ---------- are_offsets_within_expected_range ---------- */

void test_offsets_all_within_range_returns_true(void)
{
    TEST_ASSERT_TRUE(are_offsets_within_expected_range(32000, 32500, 33000, 28000, 38000));
}

void test_offsets_phase_a_below_range_returns_false(void)
{
    TEST_ASSERT_FALSE(are_offsets_within_expected_range(27000, 32500, 33000, 28000, 38000));
}

void test_offsets_phase_b_above_range_returns_false(void)
{
    TEST_ASSERT_FALSE(are_offsets_within_expected_range(32000, 39000, 33000, 28000, 38000));
}

void test_offsets_phase_c_above_range_returns_false(void)
{
    TEST_ASSERT_FALSE(are_offsets_within_expected_range(32000, 32500, 40000, 28000, 38000));
}

void test_offsets_exactly_at_boundaries_returns_true(void)
{
    /* Boundaries are inclusive */
    TEST_ASSERT_TRUE(are_offsets_within_expected_range(28000, 32500, 38000, 28000, 38000));
}

/* ---------- Test runner ----------
 * Plain Unity (unlike Ceedling) does not auto-generate this - it has to be
 * written by hand, calling every test function via RUN_TEST(). */
int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_speed_unit_to_rpm_positive_value);
    RUN_TEST(test_speed_unit_to_rpm_negative_value);
    RUN_TEST(test_speed_unit_to_rpm_zero_is_zero);

    RUN_TEST(test_s16_angle_to_degrees_zero);
    RUN_TEST(test_s16_angle_to_degrees_quarter_turn);
    RUN_TEST(test_s16_angle_to_degrees_negative_quarter_turn);
    RUN_TEST(test_s16_angle_to_degrees_half_turn);
    RUN_TEST(test_s16_angle_to_degrees_near_positive_max_truncates);

    RUN_TEST(test_counts_to_milliamps_positive);
    RUN_TEST(test_counts_to_milliamps_negative);
    RUN_TEST(test_counts_to_milliamps_zero);

    RUN_TEST(test_current_magnitude_milliamps_3_4_5_triangle);
    RUN_TEST(test_current_magnitude_milliamps_zero_current);
    RUN_TEST(test_current_magnitude_milliamps_only_q_axis);

    RUN_TEST(test_offsets_all_within_range_returns_true);
    RUN_TEST(test_offsets_phase_a_below_range_returns_false);
    RUN_TEST(test_offsets_phase_b_above_range_returns_false);
    RUN_TEST(test_offsets_phase_c_above_range_returns_false);
    RUN_TEST(test_offsets_exactly_at_boundaries_returns_true);

    return UNITY_END();
}
