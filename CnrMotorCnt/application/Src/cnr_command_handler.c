#include "main.h"
#include "cnr_command_handler.h"
#include<stdio.h>
#include "cmsis_os.h"
#include <stdlib.h>  
#include "cnr_command_handler.h"
#include<string.h>


volatile uint8_t    uart3_line_buffer[COMMAND_LINE_MAX_CHARS];
volatile uint8_t  uart3_received_byte;
extern UART_HandleTypeDef huart3;
volatile uint8_t uart3_line_length;
volatile uint8_t uart3_line_complete;
volatile uint8_t uart3_received_byte;
extern osSemaphoreId uart_line_semHandle;

/**
 * @brief  Reads one floating-point number from a text buffer.
 *
 * @details Uses strtof() to convert the text at *cursor (accepts +, -,
 *          decimals, e.g. "0.4", "-1.25", ".5"). On success, *cursor is
 *          moved to the first character after the number, so the caller
 *          can continue parsing (e.g. at ',' or end of line).
 *
 * @param[in,out] cursor  Address of the read pointer. In: where the number
 *                        starts. Out: first character after the number
 *                        (unchanged on failure).
 * @param[out]    value   Converted number (written only on success).
 *
 * @retval 1U  A number was found and converted.
 * @retval 0U  No number at *cursor (e.g. "abc").
 */

static uint8_t read_float(const char **cursor, float *value)
{
    char *end_pointer;
    float result = strtof(*cursor, &end_pointer);

    if ((end_pointer == *cursor))   
    {
        return 0U;
    }
    *value  = result;
    *cursor = end_pointer;
    return 1U;
}

/**
 * @brief  Parses one UART command line into a motion command.
 *
 * @details Supported formats (line terminator already removed):
 *          - "ME0"               -> CNR_MOTOR_DISABLE (stop motor)
 *          - "ME1"               -> CNR_MOTOR_ENABLE  (start motor)
 *          - "M<rad>"            -> CNR_MOTOR_MOVE, duration_seconds = 0
 *                                   e.g. "M0.4", "M-1.5"
 *          - "M<rad>,<seconds>"  -> CNR_MOTOR_MOVE with duration
 *                                   e.g. "M45,2"
 *          A duration <= 0 is stored as 0 (follow mode in MCSDK).
 *          out_cmd is cleared first, so unused fields are 0.
 *
 * @param[in]  rec_data  NUL-terminated command line.
 * @param[out] out_cmd   Parsed command (valid only when 1U is returned).
 *
 * @retval 1U  Line parsed, out_cmd filled.
 * @retval 0U  Invalid line (NULL pointer, first char not 'M', bad number,
 *             'ME' followed by other than '0'/'1').
 */
uint8_t cnr_parse_motion_command(const char *rec_data, motion_command_t *out_cmd)
{
    const char *cursor;
    float       value;

    if ((NULL == rec_data) || (NULL == out_cmd) || ('M' != rec_data[0]))
    {
        return 0U;
    }

    memset(out_cmd, 0, sizeof(*out_cmd));

    if ('E' == rec_data[1])
    {
        cursor = &rec_data[2];

        if (('0' == *cursor))
        {
            out_cmd->command_type   = CNR_MOTOR_DISABLE;
            return 1U;
        }else if(('1' == *cursor)){
             out_cmd->command_type   = CNR_MOTOR_ENABLE;
            return 1U;
        }
        return 0U;                              
    }

 
    cursor = &rec_data[1];

    if ((0U == read_float(&cursor, &value)))
    {
        return 0U;                                  
    }
    out_cmd->command_type         = CNR_MOTOR_MOVE;
    out_cmd->move_radians = value;
    out_cmd->duration_seconds = 0.0f;                    

    if (',' == *cursor)                                  
    {
        cursor++;                                        
        if (0U == read_float(&cursor, &value))
        {
            return 0U;                                  
        }
        if(value <= 0){
          out_cmd->duration_seconds = 0.0f;
        }else{
          out_cmd->duration_seconds = value;
        }    
    }

    return 1U;
}
/**
 * @brief  UART receive-complete interrupt callback (HAL): builds a command line.
 *
 * @details Called by HAL for every byte received on UART3 (1-byte interrupt
 *          reception). Bytes are stored in uart3_line_buffer until a
 *          terminator ('\n' or '\r') arrives; then the line is NUL-terminated,
 *          uart3_line_complete is set and uart_line_semHandle is released to
 *          wake the command handler task.
 *          - While a previous line is not processed (uart3_line_complete = 1),
 *            new bytes are dropped.
 *          - Empty lines (terminator only) are ignored.
 *          - A line longer than COMMAND_LINE_MAX_CHARS - 1 is discarded.
 *          Reception is re-armed at the end with HAL_UART_Receive_IT().
 *
 * @param[in] huart  UART handle that completed reception (not checked:
 *                   assumes only UART3 uses interrupt reception).
 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    char received_char = (char)uart3_received_byte;

    if (0U != uart3_line_complete)
    {
       
    }
    else if ((received_char == UART3_RX_TERMINATOR1) || (received_char == UART3_RX_TERMINATOR2))
  {
      if (uart3_line_length > 0U)
      {
          uart3_line_buffer[uart3_line_length] = '\0';
          uart3_line_complete = 1U;   
          uart3_line_length = 0;
          osSemaphoreRelease(uart_line_semHandle);                 
      }
      uart3_line_length = 0U;
    }
    else if (uart3_line_length < (COMMAND_LINE_MAX_CHARS - 1U))
    {
        uart3_line_buffer[uart3_line_length] = received_char;
        uart3_line_length++;
    }
    else
    {
        uart3_line_length = 0U;                        
    }

    HAL_UART_Receive_IT(&huart3, (uint8_t *)&uart3_received_byte, 1U); 
}