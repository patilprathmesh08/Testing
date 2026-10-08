 #include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include "mc_api.h"
#include "mcp_config.h"
#include "mc_tasks.h"

#include <cnr_elec_angle.h>
#include <cnr_position_control.h>
#include"cnr_command_handler.h"
#include"cnr_mcsdk_position_control_test.h"


/**
 * @brief  Prints one CSV log line: position, q current, speed.
 *
 * @details Format: "<position_rad>,<iq_A>,<speed_rpm>\r\n"
 *          - position: MC_GetCurrentPosition1()        [rad, from index]
 *          - iq:       MC_GetIqdMotor1_F().q           [A]
 *          - speed:    MC_GetAverageMecSpeedMotor1_F() [rpm, 6 rpm steps]
 */
void cnr_capture_log(void){
   
  float current_pos = MC_GetCurrentPosition1();  
  float speed_rpm = MC_GetAverageMecSpeedMotor1_F();
  qd_f_t iqd     = MC_GetIqdMotor1_F();
    
  printf("%.2f,%.2f,%.2f\r\n",current_pos,iqd.q,speed_rpm);
}

/**
 * @brief  Logs continuously for a fixed time (used for follow-mode moves).
 *
 * @details Calls cnr_capture_log() every CNR_FOLLOW_LOG_INTERVAL_MS ticks
 *          until FOLLOW_TIMEOUT_MS ticks have passed. Follow mode never
 *          reports "ready", so a fixed time is used instead.
 */

void cnr_follow_capture_log(void){
  uint32_t start_tick = osKernelSysTick();
  while (1)
  {
      cnr_capture_log();

      if ((osKernelSysTick() - start_tick) > (FOLLOW_TIMEOUT_MS))   
      {
          break;
      }
      osDelay(CNR_FOLLOW_LOG_INTERVAL_MS);
  }
}

/**
 * @brief  Executes a relative position move and logs until it ends.
 *
 * @details target = current position + position_rad.
 *          - duration > 0: move mode (MCSDK trajectory). Logs every tick
 *            until status is TC_READY_FOR_COMMAND.
 *          - duration = 0: follow mode. Logs for a fixed time
 *            (cnr_follow_capture_log()).
 *          Prints one final log line 10 ticks after the end.
 *
 * @param[in] position_rad  Relative move [rad], + or -.
 * @param[in] duration      Move time [s]; 0 = follow mode.
 *
 * @retval HAL_OK    Move done.
 * @retval HAL_ERROR   Motor not in RUN, or command rejected by MCSDK.
 */
bool cnr_position_command_handler(float position_rad,float duration){

    if(RUN != MC_GetSTMStateMotor1()){
        printf("[COMMAND] : Motor is in not run mode %d\n",MC_GetSTMStateMotor1());
         return HAL_ERROR;
    }

    float target = MC_GetCurrentPosition1()  +  position_rad;
    
    MC_ProgramPositionCommandMotor1(target,duration);

    PosCtrlStatus_t status = MC_GetControlPositionStatusMotor1();

    if((TC_MOVEMENT_ON_GOING != status) && (TC_FOLLOWING_ON_GOING != status)){
        printf("[COMMAND] FAIL: move command rejected (status %d)\r\n",(int)MC_GetControlPositionStatusMotor1());
        return HAL_ERROR;
    }

    if(TC_MOVEMENT_ON_GOING == status){
        while(TC_READY_FOR_COMMAND != MC_GetControlPositionStatusMotor1()){
            cnr_capture_log();
            osDelay(pdMS_TO_TICKS(1));
        }
    }else{
        cnr_follow_capture_log();
    }

     osDelay(10);

     cnr_capture_log();

    return HAL_OK;
}

/**
 * @brief  Starts the motor (ME1) and waits for RUN state.
 *
 * @details Calls MC_StartMotor1() and waits until the state machine reaches
 *          RUN, or CNR_START_TIMEOUT ticks pass. If already in RUN, does
 *          nothing.
 *
 * @retval HAL_OK     Motor is in RUN.
 * @retval HAL_ERROR  Timeout: RUN not reached (e.g. fault present,
 *                           not acknowledged).
 */

bool cnr_motor_start(void)
{
    if (RUN == MC_GetSTMStateMotor1())
    {
        printf("[ME1] already running\r\n");
        return HAL_OK;
    }

    MC_StartMotor1();

    uint32_t  start_time = osKernelSysTick();

    while (RUN != MC_GetSTMStateMotor1())
    {
        if ((osKernelSysTick() - start_time) > CNR_START_TIMEOUT)
        {
            printf("[COMMAND] did not reach RUN: state %d, faults 0x%04X\r\n",MC_GetSTMStateMotor1(),MC_GetCurrentFaultsMotor1());
            return HAL_ERROR;
        }
        osDelay(pdMS_TO_TICKS(1));
    }

    printf("[COMMAND] : motor start successful\r\n");
    return HAL_OK;
}

/**
 * @brief  Stops the motor (ME0) and waits for IDLE state.
 *
 * @details Calls MC_StopMotor1() and waits until the state machine reaches
 *          IDLE, or CNR_START_TIMEOUT ticks pass. If already IDLE, does
 *          nothing. A normal stop keeps MCSDK's encoder alignment.
 *
 * @retval HAL_OK    Motor is IDLE.
 * @retval HAL_ERROR  Timeout: IDLE not reached.
 */
bool cnr_motor_stop(void)
{

     if (IDLE == MC_GetSTMStateMotor1())
    {
        printf("[ME1] already IDLE\r\n");
        return HAL_OK;
    }

    MC_StopMotor1();

    uint32_t start_time = osKernelSysTick();

    while (IDLE != MC_GetSTMStateMotor1())
    {
        if ((osKernelSysTick() - start_time) > CNR_START_TIMEOUT)
        {
            printf("[ME0] did not reach IDLE: state %d\r\n", (int)MC_GetSTMStateMotor1());
            return HAL_ERROR;
        }
        osDelay(pdMS_TO_TICKS(1));
    }

    printf("[COMMAND] : motor stop successful\r\n");
    return HAL_OK;
}
/**
 * @brief  Runs one parsed command (dispatcher).
 *
 * @details - CNR_MOTOR_DISABLE -> cnr_motor_stop()
 *          - CNR_MOTOR_ENABLE  -> cnr_motor_start()
 *          - CNR_MOTOR_MOVE    -> cnr_position_command_handler(rad, seconds)
 *
 * @param[in] cmd  Command from cnr_parse_motion_command().
 *
 * @retval HAL_OK     Command executed.
 * @retval HAL_ERROR   Start/stop failed, or unknown command type.
 */

bool cnr_execute_command(motion_command_t *cmd)
{
    switch (cmd->command_type)
    {
        case CNR_MOTOR_DISABLE:
             if(cnr_motor_stop()){
                return HAL_ERROR;
             }
             break;
        case CNR_MOTOR_ENABLE:
            if(cnr_motor_start()){
               return HAL_ERROR;
            }
            break;
        case CNR_MOTOR_MOVE:
            if(cnr_position_command_handler(cmd->move_radians,cmd->duration_seconds)){
              return HAL_ERROR;
            }
            break;

        default:
            printf("[CMD] unknown command type %d\r\n", cmd->command_type);
             return HAL_ERROR;
    }

    return HAL_OK;
}