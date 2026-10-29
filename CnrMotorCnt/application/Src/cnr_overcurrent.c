#include "main.h"
#include "mc_config.h"
#include "mc_config_common.h"         
#include "parameters_conversion.h"     
#include "pwm_curr_fdbk.h"             
#include "r3_1_g4xx_pwm_curr_fdbk.h"   
#include "cnr_overcurrent.h"

static int32_t limit_s16 = 0;        

// /**
//  * @brief  Phase-current reading with overcurrent check (MCSDK callback).
//  *
//  * @details Registered with PWMC_RegisterGetPhaseCurrentsCallBack(), so MCSDK
//  *          calls it every PWM cycle (ADC interrupt) instead of reading the
//  *          currents directly. It:
//  *          1. reads Ia, Ib with the normal R3_1_GetPhaseCurrents()
//  *             (FOC receives exactly the same values as before),
//  *          2. calculates Ic = -Ia - Ib,
//  *          3. if any |I| > limit_s16, calls PWMC_OCP_Handler():
//  *             PWM off at once + overcurrent flag. The safety task then raises
//  *             MC_OVER_CURR (0x0040) and the state goes FAULT_NOW -> FAULT_OVER.
//  *
//  * @param[in]  pwm_handle      MCSDK PWM/current-feedback handle.
//  * @param[out] phase_currents  Phase currents Ia (a), Ib (b) in s16 units.

//  */
// static void cnr_read_phase_currents_checked(PWMC_Handle_t *pwm_handle, ab_t *phase_currents)
// {
//   R3_1_GetPhaseCurrents(pwm_handle, phase_currents);        

//   int32_t current_a = phase_currents->a;
//   int32_t current_b = phase_currents->b;
//   int32_t current_c = -current_a - current_b;                

//   if ((current_a > limit_s16) || (current_a < -limit_s16) ||(current_b > limit_s16) || (current_b < -limit_s16) || (current_c > limit_s16) || (current_c < -limit_s16))
//   {
//     PWMC_OCP_Handler(pwm_handle);                    
//   }
// }
// /**
//  * @brief  Enables the software overcurrent fault.
//  *
//  * @details Converts the limit from amps to MCSDK units
//  *          (limit_a x CURRENT_CONV_FACTOR, about 2979 per A) and installs
//  *          read_phase_currents_checked() as MCSDK's current-reading callback.
//  *          The callback survives MCSDK's offset calibration (it is saved and
//  *          restored by R3_1), so one call is enough.
//  *
//  * @param[in] limit_a  Trip level [A], e.g. OVERCURRENT_FAULT_A (0.8 A).
//  *                     Keep above the command limits and >= 0.1 A
//  *                     (DRV8316 current-sense offset is +/-50 mA).
//  */
// void cnr_overcurrent_enable(float limit_a)
// {
//   limit_s16 = (int32_t)(limit_a * (float)CURRENT_CONV_FACTOR);  
//   PWMC_RegisterGetPhaseCurrentsCallBack(&cnr_read_phase_currents_checked, pwmcHandle[M1]);
// }

// /**
//  * @brief  Phase-current reading with inverted sign (MCSDK callback).
//  *
//  * @details DRV8316 CSA output rises with phase current, while R3_1 computes
//  *          current as (offset - ADC). Inverting Ia and Ib makes the current
//  *          loop feedback negative (correct). INT16_MIN is clamped to avoid
//  *          overflow when negated.
//  */
// static void cnr_read_phase_currents_inverted(PWMC_Handle_t *pwm_handle, ab_t *phase_currents)
// {
//   R3_1_GetPhaseCurrents(pwm_handle, phase_currents);

//   phase_currents->a = (phase_currents->a == INT16_MIN) ? INT16_MAX : (int16_t)(-phase_currents->a);
//   phase_currents->b = (phase_currents->b == INT16_MIN) ? INT16_MAX : (int16_t)(-phase_currents->b);
// }

// /**
//  * @brief  Installs the sign-fix callback. Call once before alignment.
//  */
// void cnr_current_sign_fix_enable(void)
// {
//   // PWMC_RegisterGetPhaseCurrentsCallBack(&cnr_read_phase_currents_inverted, pwmcHandle[M1]);
// }


/**
 * @brief  Phase-current reading: sign fix + overcurrent check (MCSDK callback).
 *
 * @details MCSDK keeps only ONE current-reading callback (pFctGetPhaseCurrents),
 *          so the sign fix and the overcurrent check must live in one function.
 *          1. reads Ia, Ib with R3_1_GetPhaseCurrents()
 *          2. inverts Ia, Ib (DRV8316 CSA polarity is opposite to R3_1's offset - ADC)
 *          3. Ic = -Ia - Ib; if any |I| > limit_s16 -> PWMC_OCP_Handler()
 *          The check uses |I|, so it gives the same result before or after the inversion.
 */
static void cnr_read_phase_currents_fixed_checked(PWMC_Handle_t *pwm_handle, ab_t *phase_currents)
{
  R3_1_GetPhaseCurrents(pwm_handle, phase_currents);

  /* sign fix; INT16_MIN cannot be negated in int16, so clamp to INT16_MAX */
  phase_currents->a = (phase_currents->a == INT16_MIN) ? INT16_MAX : (int16_t)(-phase_currents->a);
  phase_currents->b = (phase_currents->b == INT16_MIN) ? INT16_MAX : (int16_t)(-phase_currents->b);

  /* overcurrent check only when a limit has been set */
  if (limit_s16 > 0)
  {
    int32_t current_a = phase_currents->a;
    int32_t current_b = phase_currents->b;
    int32_t current_c = -current_a - current_b;

    if ((current_a > limit_s16) || (current_a < -limit_s16) ||
        (current_b > limit_s16) || (current_b < -limit_s16) ||
        (current_c > limit_s16) || (current_c < -limit_s16))
    {
      PWMC_OCP_Handler(pwm_handle);      /* PWM off + overcurrent flag */
    }
  }
}

void cnr_overcurrent_enable(float limit_a)
{
  limit_s16 = (int32_t)(limit_a * (float)CURRENT_CONV_FACTOR);
  PWMC_RegisterGetPhaseCurrentsCallBack(&cnr_read_phase_currents_fixed_checked, pwmcHandle[M1]);
}

void cnr_current_sign_fix_enable(void)
{
  /* same combined callback; overcurrent check stays off until cnr_overcurrent_enable() sets a limit */
  PWMC_RegisterGetPhaseCurrentsCallBack(&cnr_read_phase_currents_fixed_checked, pwmcHandle[M1]);
}