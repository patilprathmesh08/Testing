#ifndef CNR_POSITION_CONTROL_H
#define CNR_POSITION_CONTROL_H

#include"cnr_command_handler.h"

bool cnr_position_command_handler(float position_rad,float duration);
void cnr_capture_log(void);
bool cnr_motor_start(void);
bool cnr_motor_stop(void);
bool cnr_execute_command(motion_command_t *out_cmd);

#endif