#ifndef CNR_COMMAND_HANDLER_H
#define CNR_COMMAND_HANDLER_H


#define COMMAND_LINE_MAX_CHARS  64U
#define UART3_RX_TERMINATOR1     '\n'
#define UART3_RX_TERMINATOR2     '\r'

typedef struct
{
    float move_radians;        
    float duration_seconds;
    uint8_t command_type;   
} motion_command_t;

typedef enum
{
    CNR_MOTOR_DISABLE = 0,
    CNR_MOTOR_ENABLE,             
    CNR_MOTOR_MOVE      
} command_type_t;

uint8_t cnr_parse_motion_command(const char *rec_data, motion_command_t *out_cmd);


#endif