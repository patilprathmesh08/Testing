#ifndef CNR_ELEC_ANGLE_H
#define CNR_ELEC_ANGLE_H

#include <stdint.h>
#include <stdbool.h>

/* Raw TIM4 count and encoder electrical angle captured at the Z index. */

#define CNR_ALIGNMENT_CURRENT_A          0.0f
#define CNR_ALIGNMENT_TIMEOUT            10000U 
#define CNR_START_TIMEOUT                5000U
/**
 * Finds the electrical offset by spinning the motor open loop until the Z
 * index passes. Returns true on success and writes the offset (s16).
 */
bool cnr_commutation_alignemnt(void);

/** Applies an offset from cnr_find_electrical_offset() to the encoder. */
void cnr_apply_electrical_offset(int16_t offset);

/** Finds the offset, applies it, and starts the motor in closed loop. */
bool cnr_start_motor_offset_find(float current_a);

#endif /* ELEC_ANGLE_H */