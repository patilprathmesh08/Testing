/**
 ******************************************************************************
 * @file    motor_telemetry_math.h
 * @brief   Pure numeric conversions matching the FOC telemetry/diagnostics
 *          math in main.c. Compiled/tested on a host build (no STM32 HAL /
 *          MCSDK headers required, no hardware access).
 *
 * IMPORTANT: main.c is NOT modified and does NOT call these functions.
 * This is a standalone duplicate of the same formulas, kept in sync by
 * hand. If the inline math in main.c changes, update these functions (and
 * their tests) to match, or they will silently verify stale logic.
 *
 * All constants that would normally come from parameters_conversion.h
 * (U_RPM, SPEED_UNIT, CURRENT_CONV_FACTOR) are passed in explicitly by the
 * caller rather than included here, precisely so this module has zero
 * MCSDK/HAL dependency and stays host-compilable.
 ******************************************************************************
 */

#ifndef MOTOR_TELEMETRY_MATH_H
#define MOTOR_TELEMETRY_MATH_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Converts an MCSDK mechanical speed reading (SPEED_UNIT scale) to RPM.
 * @param unit_speed   Raw value from MC_GetMecSpeedAverageMotor1().
 * @param rpm_unit_num Value of U_RPM (from parameters_conversion.h).
 * @param speed_unit_den Value of SPEED_UNIT (from parameters_conversion.h).
 * @return Speed in RPM.
 */
int32_t speed_unit_to_rpm(int16_t unit_speed, int32_t rpm_unit_num, int32_t speed_unit_den);

/**
 * @brief Converts an s16 electrical angle (65536 s16 = 360 degrees) to degrees.
 * @param angle_s16 Electrical angle in MCSDK s16 format.
 * @return Angle in electrical degrees.
 */
int32_t s16_angle_to_degrees(int16_t angle_s16);

/**
 * @brief Converts a raw ADC current reading (counts) to milliamps.
 * @param counts              Raw current reading (Iq or Id, counts).
 * @param current_conv_factor Board's CURRENT_CONV_FACTOR (counts per amp).
 * @return Current in milliamps.
 */
int32_t counts_to_milliamps(int16_t counts, float current_conv_factor);

/**
 * @brief Computes current magnitude |I| = sqrt(Iq^2 + Id^2), in milliamps.
 * @param iq_counts           Q-axis current, counts.
 * @param id_counts           D-axis current, counts.
 * @param current_conv_factor Board's CURRENT_CONV_FACTOR (counts per amp).
 * @return Current magnitude in milliamps.
 */
int32_t current_magnitude_milliamps(int16_t iq_counts, int16_t id_counts, float current_conv_factor);

/**
 * @brief Checks DRV8316 CSA offset calibration values against an expected
 *        midscale window (sanity check run once at boot).
 * @param phase_a_offset Calibrated offset, phase A.
 * @param phase_b_offset Calibrated offset, phase B.
 * @param phase_c_offset Calibrated offset, phase C.
 * @param min_expected   Lower bound of the expected window (inclusive).
 * @param max_expected   Upper bound of the expected window (inclusive).
 * @return true if all three phases fall within [min_expected, max_expected].
 */
bool are_offsets_within_expected_range(uint16_t phase_a_offset,
                                       uint16_t phase_b_offset,
                                       uint16_t phase_c_offset,
                                       uint16_t min_expected,
                                       uint16_t max_expected);

#endif /* MOTOR_TELEMETRY_MATH_H */
