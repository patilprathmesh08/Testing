/**
 ******************************************************************************
 * @file    motor_telemetry_math.c
 *
 * IMPORTANT - STANDALONE DUPLICATE, NOT CALLED FROM main.c
 * ---------------------------------------------------------------------------
 * main.c was NOT modified. The functions below are a SEPARATE COPY of math
 * that also exists inline inside main.c's telemetry loop. They are tested
 * here on the host, but main.c does not call them - main.c still runs its
 * own inline version on the real ARM target.
 *
 * CONSEQUENCE: if the inline math in main.c is ever changed (a units fix,
 * a tuned constant, a bug fix), these functions - and their tests - will
 * NOT reflect that change. Passing tests here would then be verifying
 * outdated logic, not what actually runs on the motor. If that happens,
 * these functions must be updated by hand to match main.c again.
 *
 * See motor_telemetry_math.h for what each function is a copy of.
 ******************************************************************************
 */

#include "motor_telemetry_math.h"
#include <math.h>

int32_t speed_unit_to_rpm(int16_t unit_speed, int32_t rpm_unit_num, int32_t speed_unit_den)
{
    return ((int32_t)unit_speed * rpm_unit_num) / speed_unit_den;
}

int32_t s16_angle_to_degrees(int16_t angle_s16)
{
    return ((int32_t)angle_s16 * 360) / 65536;
}

int32_t counts_to_milliamps(int16_t counts, float current_conv_factor)
{
    return (int32_t)(((float)counts / current_conv_factor) * 1000.0f);
}

int32_t current_magnitude_milliamps(int16_t iq_counts, int16_t id_counts, float current_conv_factor)
{
    return (int32_t)((sqrtf((float)iq_counts * iq_counts + (float)id_counts * id_counts)
                       / current_conv_factor) * 1000.0f);
}

bool are_offsets_within_expected_range(uint16_t phase_a_offset,
                                       uint16_t phase_b_offset,
                                       uint16_t phase_c_offset,
                                       uint16_t min_expected,
                                       uint16_t max_expected)
{
    if (phase_a_offset < min_expected || phase_a_offset > max_expected ||
        phase_b_offset < min_expected || phase_b_offset > max_expected ||
        phase_c_offset < min_expected || phase_c_offset > max_expected)
    {
        return false;
    }
    return true;
}
