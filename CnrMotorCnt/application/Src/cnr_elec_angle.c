/**
  ******************************************************************************
  * @file    elec_angle.c
  * @brief   Electrical offset search - same method as SimpleFOC's
  *          ZeroElecMaxonSearch().
  *
  * @details HOW IT WORKS
  *          The Z index is a fixed mark on the rotor. The magnet's electrical
  *          angle at that mark is a constant (INDEX_ELECTRICAL_ANGLE_DEG).
  *            1. spin the field slowly (open loop) until the rotor passes
  *               the index
  *            2. at the index, read the TIM4 count and convert it to an angle
  *            3. offset = index angle - encoder angle at the index
  *
  *          NO MCSDK FILE IS CHANGED
  *          - the field angle is forced by switching the controller to
  *            MCSDK's virtual angle sensor (the SDK does this itself at
  *            mc_tasks_foc.c:301)
  *          - the index is detected by reading the PB6 pin directly
  *          - the offset is written into the encoder once
  *
  *          HARDWARE
  *          PB6 (Z index) must be set to Pull-DOWN, IT Rising in CubeMX.
  ******************************************************************************
  */

#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>

#include "mc_api.h"
#include "mc_config.h"                 /* pSTC, FOCVars, pPIDIq, pPIDId, PosCtrlM1 */
#include "mc_config_common.h"          /* pwmcHandle, ENCODER_M1, VirtualSpeedSensorM1, EncAlignCtrlM1 */
#include "mc_interface.h"
#include "mc_type.h"
#include "pwm_curr_fdbk.h"
#include "speed_torq_ctrl.h"
#include "virtual_speed_sensor.h"
#include "encoder_speed_pos_fdbk.h"
#include "trajectory_ctrl.h"
#include "enc_align_ctrl.h"
#include "parameters_conversion.h"
#include "pmsm_motor_parameters.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_tim.h"
#include "cnr_elec_angle.h"


/**
 * @brief  Runs MCSDK's encoder alignment (alignment + Z index search).
 *
 * @details 1. Buffers a torque ramp at CNR_ALIGNMENT_CURRENT_A and calls
 *             MC_StartMotor1(): MCSDK aligns the rotor, then searches the
 *             Z index (up to one revolution).
 *          2. Waits for RUN (CNR_START_TIMEOUT), then for
 *             TC_ALIGNMENT_COMPLETED (CNR_ALIGNMENT_TIMEOUT).
 *          3. Stops the motor and waits for IDLE. The alignment is kept
 *             for later starts (until a fault).
 *
 * @retval HAL_OK  Alignment completed.
 * @retval HAL_ERROR   Failed: RUN, alignment or IDLE not reached (see log).
 */
bool cnr_commutation_alignemnt(void){

  uint8_t motor_fault = HAL_ERROR;

  MC_ProgramTorqueRampMotor1((int16_t)(CNR_ALIGNMENT_CURRENT_A * CURRENT_CONV_FACTOR), 1000U);
  
  MC_StartMotor1();

  uint32_t  start_time = osKernelSysTick();

  while (RUN != MC_GetSTMStateMotor1())
  {
    if ((osKernelSysTick() - start_time) > CNR_START_TIMEOUT)
    {
      printf("[COMMUTATION] did not reach RUN: state %d, faults 0x%04X\r\n",MC_GetSTMStateMotor1(),MC_GetCurrentFaultsMotor1());
       motor_fault = HAL_ERROR;
      break;
    }
    osDelay(pdMS_TO_TICKS(1));
  }


  

  start_time = osKernelSysTick();

  while(MC_GetAlignmentStatusMotor1() != TC_ALIGNMENT_COMPLETED){

    if ((osKernelSysTick() - start_time) > CNR_ALIGNMENT_TIMEOUT)
    {
      printf("[COMMUTATION] did not reach TC_ALIGNMENT_COMPLETED state %d, faults 0x%d\r\n",MC_GetAlignmentStatusMotor1(),MC_GetCurrentFaultsMotor1());
      motor_fault = HAL_ERROR;
      break;
    }
    osDelay(pdMS_TO_TICKS(1));
  }

  if(MC_GetAlignmentStatusMotor1() == TC_ALIGNMENT_COMPLETED){
    printf("Alignment completed\n\n");
    motor_fault = HAL_OK;
  }else{
    /*TODO*/
  }

  MC_StopMotor1();

  start_time = osKernelSysTick();

  while (IDLE != MC_GetSTMStateMotor1())
  {
    if ((osKernelSysTick() - start_time) > CNR_START_TIMEOUT)
    {
      printf("MOTOR STATE : %d\n",MC_GetSTMStateMotor1());
      motor_fault = HAL_ERROR;
      break;
    }
    osDelay(pdMS_TO_TICKS(1));
  }
  return motor_fault;
}