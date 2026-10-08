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
#include"cnr_mcsdk_position_control_test.h"


/**
 * @brief  Prints a test header and the CSV column names.
 *
 * @param[in] buff      Test name (e.g. "1A").
 * @param[in] postion   Commanded move [rad].
 * @param[in] duration  Move time [s] (0 = follow mode).
 */
void cnr_header_log(char buff[], float postion , float duration){

  printf("%s -----------------> %0.2f %0.2f\n",buff,postion,duration);
  printf("Current Position,q Current,Current Velocity\r\n");

}

/**
 * @brief  Position-control proof-of-concept test sequence with logging.
 *
 * @details Runs 8 relative moves, logging position, iq and speed:
 *          - 1A/1B: +/-100 rad, follow mode (duration 0)
 *          - 2A/2B: +/-1000 rad, follow mode
 *          - 3A/3B: +/-100 rad, trajectory (2 s / 2 s)
 *          - 4A/4B: +/-1000 rad, trajectory (2 s / 4 s)
 *          Waits CNR_FOLLOW_SETTLE_TIME_MS ticks between moves.
 *          Enabled with CNR_POSITION_CONTROL_POC_ENABLE.
 *
 * @pre    Motor in RUN, aligned, position controller ready.
 */
void cnr_mcsdk_position_control_poc_test(void){
    
  cnr_header_log("1A",POS_CTRL_100RAD_TARGET,FOLLOW_MODE_DURATION_S);
  MC_ProgramPositionCommandMotor1(MC_GetCurrentPosition1() + POS_CTRL_100RAD_TARGET,FOLLOW_MODE_DURATION_S);
  cnr_follow_capture_log();
  osDelay((CNR_FOLLOW_SETTLE_TIME_MS));

  cnr_header_log("1B",-POS_CTRL_100RAD_TARGET,FOLLOW_MODE_DURATION_S);
  MC_ProgramPositionCommandMotor1(MC_GetCurrentPosition1() + (-TRAJ_CTRL_100RAD_TARGET),FOLLOW_MODE_DURATION_S);
  cnr_follow_capture_log();
  osDelay((CNR_FOLLOW_SETTLE_TIME_MS));

  cnr_header_log("2A",POS_CTRL_1000RAD_TARGET,FOLLOW_MODE_DURATION_S);
  MC_ProgramPositionCommandMotor1(MC_GetCurrentPosition1() + POS_CTRL_1000RAD_TARGET,FOLLOW_MODE_DURATION_S);
  cnr_follow_capture_log();
  osDelay((CNR_FOLLOW_SETTLE_TIME_MS));

  cnr_header_log("2B",-POS_CTRL_1000RAD_TARGET,FOLLOW_MODE_DURATION_S);
  MC_ProgramPositionCommandMotor1(MC_GetCurrentPosition1() + (-POS_CTRL_1000RAD_TARGET),FOLLOW_MODE_DURATION_S);
  cnr_follow_capture_log();
  osDelay((CNR_FOLLOW_SETTLE_TIME_MS));

  cnr_header_log("3A",TRAJ_CTRL_100RAD_TARGET,TRAJ_CTRL_100RAD_OUT_TIME);
  MC_ProgramPositionCommandMotor1(MC_GetCurrentPosition1() + TRAJ_CTRL_100RAD_TARGET,TRAJ_CTRL_100RAD_OUT_TIME);
  while(TC_READY_FOR_COMMAND != MC_GetControlPositionStatusMotor1()){
    cnr_capture_log();
    osDelay((CNR_FOLLOW_LOG_INTERVAL_MS));
  }
  osDelay((CNR_FOLLOW_SETTLE_TIME_MS));
  cnr_capture_log();

  cnr_header_log("3B",-TRAJ_CTRL_100RAD_TARGET,TRAJ_CTRL_100RAD_BACK_TIME);
  MC_ProgramPositionCommandMotor1(MC_GetCurrentPosition1() + (-TRAJ_CTRL_100RAD_TARGET),TRAJ_CTRL_100RAD_BACK_TIME);

  while(TC_READY_FOR_COMMAND != MC_GetControlPositionStatusMotor1()){
    cnr_capture_log();
    osDelay((CNR_FOLLOW_LOG_INTERVAL_MS));
  }
  osDelay((CNR_FOLLOW_SETTLE_TIME_MS));
  cnr_capture_log();


  cnr_header_log("4A",TRAJ_CTRL_1000RAD_TARGET,TRAJ_CTRL_1000RAD_OUT_TIME);
  MC_ProgramPositionCommandMotor1(MC_GetCurrentPosition1() + TRAJ_CTRL_1000RAD_TARGET,TRAJ_CTRL_1000RAD_OUT_TIME);
  while(TC_READY_FOR_COMMAND != MC_GetControlPositionStatusMotor1()){
    cnr_capture_log();
    osDelay((CNR_FOLLOW_LOG_INTERVAL_MS));
  }
  osDelay((CNR_FOLLOW_SETTLE_TIME_MS));
  cnr_capture_log();


  cnr_header_log("4B",-TRAJ_CTRL_1000RAD_TARGET,TRAJ_CTRL_1000RAD_BACK_TIME);
  MC_ProgramPositionCommandMotor1(MC_GetCurrentPosition1()+ (-TRAJ_CTRL_1000RAD_TARGET),TRAJ_CTRL_1000RAD_BACK_TIME);
  while(TC_READY_FOR_COMMAND != MC_GetControlPositionStatusMotor1()){
    cnr_capture_log();
    osDelay((CNR_FOLLOW_LOG_INTERVAL_MS));
  }
  osDelay((CNR_FOLLOW_SETTLE_TIME_MS));
  cnr_capture_log();


  printf("-----------------------------------------------------------------------------------------------------------------------------\n");
}