#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>

#include "mc_api.h"
#include "mc_config.h"
#include "mc_interface.h"
#include "mc_type.h"

#include "cnr_motor_homing.h"
#include"cnr_elec_angle.h"

#define MOTOR_RUN_TIMEOUT_MS 600

void McPrintFaults(const char *tag, uint16_t faults)
{
  if (0U == faults)
  {
    printf("%s no faults\r\n", tag);
    return;
  }

  printf("%s faults 0x%04X:", tag, (unsigned)faults);

  if (0U != (faults & MC_DURATION))   { printf(" FOC_DURATION");  }
  if (0U != (faults & MC_OVER_VOLT))  { printf(" OVER_VOLT");     }
  if (0U != (faults & MC_UNDER_VOLT)) { printf(" UNDER_VOLT");    }
  if (0U != (faults & MC_OVER_TEMP))  { printf(" OVER_TEMP");     }
  if (0U != (faults & MC_START_UP))   { printf(" START_UP");      }
  if (0U != (faults & MC_SPEED_FDBK)) { printf(" SPEED_FDBK");    }
  if (0U != (faults & MC_OVER_CURR))  { printf(" OVER_CURRENT");  }
  if (0U != (faults & MC_SW_ERROR))   { printf(" SW_ERROR");      }
  if (0U != (faults & MC_DP_FAULT))   { printf(" DRIVER_PROT");   }

  printf("\r\n");
}

/**
 * @brief  Homes the axis by driving into the hard stop and detecting the stall.
 *
 * @details 1. Waits for MCSDK alignment (TC_ALIGNMENT_COMPLETED) and
 *             position controller ready (timeout CNR_ALIGNMENT_TIMEOUT).
 *          2. Programs a long move (travel_radians = -10000 rad in
 *             move_duration = 8 s) and starts the motor.
 *          3. Every 5 ticks checks the position:
 *             - moved more than motion_epsilon_rad -> still moving,
 *             - no motion for stall_hold_ms after moving at least
 *               minimum_travel_rad -> stop found (success),
 *             - stall before minimum_travel_rad -> fail (friction),
 *             - never moved within breakaway_limit_ms -> fail,
 *             - trajectory finished without stall -> fail,
 *             - MCSDK fault -> stop motor, fail.
 *          4. Parks: commands the present position (0.25 s) so the motor
 *             holds without pushing into the stop.
 *
 * @retval HAL_OK    (0) Stop found.
 * @retval HAL_ERROR (1) Homing failed (see log).
 *
 * @note   Timing in FreeRTOS ticks (0.5 ms each on this build).
 */
uint8_t cnr_home_to_stop(void)
{
  /* ---- Bench-tunable ---- */
  const float    travel_radians      = -10000.0f; /* must exceed the FULL mechanical
                                                  * travel, or the trajectory just
                                                  * finishes without finding the
                                                  * stop. NEGATIVE drives toward it. */
  const float    motion_epsilon_rad  = 0.005f;   /* ~0.3 deg; below this is encoder* noise, not movement */
  const uint32_t stall_hold_ms       = 300U;     /* motionless this long = at the stop */
  const uint32_t breakaway_limit_ms  = 4000U;    /* must start moving within this */
  const float    minimum_travel_rad  = 3.14f;    /* half a rev; less than this is friction, not the stop */


  const float move_duration  = 8 ;
  uint8_t stop_found = 1U;
  uint8_t has_moved  = 0U;
  float   start_position;
  float   last_position;

  TickType_t start_tick = osKernelSysTick();

  while (TC_ALIGNMENT_COMPLETED != MC_GetAlignmentStatusMotor1()) 
  {
    if ((osKernelSysTick() - start_tick) >= pdMS_TO_TICKS(CNR_ALIGNMENT_TIMEOUT))
      {
          printf("[HOME] Timeout waiting for motor alignment\r\n");
          return HAL_ERROR;
      }else{
        /* TODO: */
      }

    osDelay(pdMS_TO_TICKS(5)); 
  }


    
  start_tick =  osKernelSysTick();

  while (TC_READY_FOR_COMMAND != MC_GetControlPositionStatusMotor1())
  {
    if (( osKernelSysTick() - start_tick)  >= pdMS_TO_TICKS(CNR_ALIGNMENT_TIMEOUT))
    {
      printf("[HOME] FAIL: controller never became ready (status %d)\r\n",(int)MC_GetControlPositionStatusMotor1());
      return HAL_ERROR;
    }else{
      /* TODO: */
    }

    osDelay(pdMS_TO_TICKS(5));
  }


  start_position = MC_GetCurrentPosition1();
  last_position  = start_position;

  MC_ProgramPositionCommandMotor1(start_position + travel_radians, move_duration);

  MC_StartMotor1();

  uint32_t start_time = osKernelSysTick();

  while (RUN != MC_GetSTMStateMotor1())
  {
    if ((osKernelSysTick() - start_time) > CNR_START_TIMEOUT)
    {
      printf("[HOME] did not reach RUN: state %d, faults 0x%04X\r\n",MC_GetSTMStateMotor1(),MC_GetCurrentFaultsMotor1());
      return HAL_ERROR;
    }
    osDelay(pdMS_TO_TICKS(1));
  }


  if (TC_MOVEMENT_ON_GOING != MC_GetControlPositionStatusMotor1())
  {
    printf("[HOME] FAIL: move command rejected (status %d)\r\n", (int)MC_GetControlPositionStatusMotor1());
    return HAL_ERROR;
  }else{
      /* TODO: */
  }

  {
    uint32_t move_start_tick  =  osKernelSysTick();
    uint32_t last_motion_tick = move_start_tick;
    uint32_t last_log_tick    = move_start_tick;

    while (1)
    {
      float now_position = MC_GetCurrentPosition1();
      float step         = now_position - last_position;
      float travelled    = now_position - start_position;

      if (step < 0.0f)      
      { 
        step = -step; 
      }
      if (travelled < 0.0f) 
      { 
        travelled = -travelled; 
      }

      /* ---- controller fault ---- */
      if (MC_GetSTMStateMotor1() == FAULT_OVER || MC_GetSTMStateMotor1() == FAULT_NOW)
      {
        printf("[HOME] ABORTED by fault after %ld rad\r\n", (long)travelled);
        McPrintFaults("[HOME]  occurred", MC_GetOccurredFaultsMotor1());
        MC_StopMotor1();
        break;
      }

      if (step > motion_epsilon_rad)
      {
        last_position    = now_position;
        last_motion_tick =  osKernelSysTick();

        if (0U == has_moved)
        {
          has_moved = 1U;
          printf("[HOME] moving\r\n");
        }
      }
      else if (0U != has_moved)
      {
        if (( osKernelSysTick() - last_motion_tick) > stall_hold_ms)
        {
          if (travelled >= minimum_travel_rad)
          {
            stop_found = HAL_OK;;

          }
          else
          {
            printf("[HOME] FAIL: stalled after only %0.2f deg, minimum %0.2f deg.\r\n",(travelled * 57.2958f),(minimum_travel_rad * 57.2958f));
          }
          break;         
        }
      }
      else if (( osKernelSysTick() - move_start_tick) > breakaway_limit_ms)
      {
        printf("[HOME] FAIL: never started moving in %lu ms.\r\n",breakaway_limit_ms);
        break;
      }

      if (TC_READY_FOR_COMMAND == MC_GetControlPositionStatusMotor1())
      {
        printf("[HOME] FAIL: travelled the full %0.2f rad and never stalled.\r\n",travel_radians);
        printf("[HOME]       Wrong direction, or the stop is further away.\r\n");
        break;
      }

      if (( osKernelSysTick() - last_log_tick) >= 400U)
      {
        last_log_tick =  osKernelSysTick();
      }
     osDelay(pdMS_TO_TICKS(5));
    }
  }

  while(TC_READY_FOR_COMMAND != MC_GetControlPositionStatusMotor1()){}

  {
    float    resting_position = MC_GetCurrentPosition1();
    uint32_t park_tick        =  osKernelSysTick();

    while (TC_READY_FOR_COMMAND != MC_GetControlPositionStatusMotor1())
    {
      if (( osKernelSysTick() - park_tick) > 3000U) { break; }
      osDelay(pdMS_TO_TICKS(5));
    }

    MC_ProgramPositionCommandMotor1(resting_position, 0.25f);

    park_tick =  osKernelSysTick();
    while (TC_READY_FOR_COMMAND != MC_GetControlPositionStatusMotor1())
    {
      if (( osKernelSysTick() - park_tick) > 3000U) { break; }
     osDelay(pdMS_TO_TICKS(5));
    }
  }

  // printf("[HOME] HOME Completed : %d\r\n",stop_found);

   return stop_found;
}