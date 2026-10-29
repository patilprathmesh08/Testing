/**
  ******************************************************************************
  * @file    motor_homing.h
  * @brief   Index search, hard-stop homing and position helpers for the CNR
  *          BLDC drive.
  *
  * @details These are APPLICATION-level routines built on top of MCSDK's public
  *          mc_api.h. No MCSDK source is modified, so the module survives a MC
  *          Workbench or CubeMX regeneration untouched.
  *
  *          They exist because MCSDK provides no equivalents:
  *            - no stall detection API
  *            - no "has the shaft reached the target" API; the trajectory status
  *              is decided purely from elapsed time and never reads the encoder
  *            - no feasibility check on a position command
  *
  *          Call order after MC_StartMotor1() has reached RUN:
  *            1. IndexSearch()   - establishes commutation offset and the
  *                                 encoder zero at the Z index
  *            2. one of the homing routines, if a mechanical reference is needed
  *            3. MC_ProgramPositionCommandMotor1(), checked with
  *               WaitPositionReached()
  ******************************************************************************
  */

#ifndef MOTOR_HOMING_H
#define MOTOR_HOMING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Shared state
 * ------------------------------------------------------------------------- */

/** Position reported by MC_GetCurrentPosition1() at the homed reference, just
 *  clear of the hard stop. Application targets are expressed relative to this. */
extern volatile float g_home_offset;

/* ---------------------------------------------------------------------------
 * Index search - run this first
 * ------------------------------------------------------------------------- */

/**
 * @brief  Open-loop Z-index search. Measures the commutation offset and applies
 *         it to g_el_offset_s16 for this power cycle.
 * @retval Number of index passes detected; 0 means failure.
 * @note   The offset is referenced to TIM4's arbitrary power-up count, so it is
 *         valid only for this boot and must be re-measured after every reset.
 */
uint8_t IndexSearch(void);

/* ---------------------------------------------------------------------------
 * Hard-stop homing - pick one
 * ------------------------------------------------------------------------- */

/**
 * @brief  Minimal homing. One long trajectory, stall detected from the position
 *         no longer changing. Uses only MC_GetCurrentPosition1().
 * @retval 1 if the stop was found and g_home_offset recorded, 0 otherwise.
 * @note   No speed ceiling, so it suits a fast approach.
 */
float HomeToStop(void);


/**
 * @brief  Has the shaft actually reached the commanded position?
 * @param  tolerance_counts  allowed error in encoder counts. 1 count = 0.35 deg
 *                           on this encoder (1024 counts/rev).
 * @retval 1 if the trajectory has finished AND the shaft is within tolerance.
 * @note   MC_GetControlPositionStatusMotor1() alone is not enough: it reports
 *         the trajectory generator's elapsed time, not the shaft.
 */
uint8_t PositionReached(float tolerance_counts);

/**
 * @brief  Waits for the shaft to reach the commanded position.
 * @param  tolerance_counts  as above.
 * @param  timeout_ms        give up after this long.
 * @retval 1 if reached, 0 on timeout or fault.
 * @note   Bounded and fault-aware, unlike a bare
 *         "while (status != TC_READY_FOR_COMMAND) {}".
 */
uint8_t WaitPositionReached(float tolerance_counts, uint32_t timeout_ms);
;

/**
 * @brief  Prints an MCSDK fault word as readable names.
 * @param  tag     short prefix identifying the caller in the log.
 * @param  faults  bitmask from MC_GetCurrentFaultsMotor1() or
 *                 MC_GetOccurredFaultsMotor1().
 */
void McPrintFaults(const char *tag, uint16_t faults);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_HOMING_H */
