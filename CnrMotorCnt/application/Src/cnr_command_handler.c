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
extern osSemaphoreId uart_line_semaHandle;
static uint8_t uart3_dma_rx_buffer[COMMAND_LINE_MAX_CHARS];   
/**
 * @brief  Starts / restarts UART3 reception with DMA. The callback fires when the
 *         line goes idle after a burst (end of a command) or the buffer is full.
 */
void cnr_uart3_rx_start(void)
{
    if (HAL_OK ==  huart3.hdmarx)
    {
        return;
    }

    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart3_dma_rx_buffer, sizeof(uart3_dma_rx_buffer));
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);        /* no half-transfer event */
}
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
 * @brief  HAL Rx event callback (interrupt context): one UART3 command was received by DMA.
 * @details Called when the line goes idle after a burst or the DMA buffer is full.
 *          Copies the bytes up to the first '\n' / '\r' into the command line buffer,
 *          terminates it, wakes the command handler task and restarts DMA reception.
 *          A command that arrives while the previous one is still unread is dropped.
 *
 * @param[in] uart_handle    UART that produced the event (only USART3 is handled).
 * @param[in] received_size  Number of bytes written by DMA into uart3_dma_rx_buffer.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart_handle, uint16_t received_size)
{
    if (uart_handle->Instance != USART3)
    {
        return;
    }

    if ((0U == uart3_line_complete) && (received_size > 0U))
    {
        uint16_t char_count = 0U;

        while ((char_count < received_size) &&(char_count < (COMMAND_LINE_MAX_CHARS - 1U)) && (uart3_dma_rx_buffer[char_count] != UART3_RX_TERMINATOR1) && (uart3_dma_rx_buffer[char_count] != UART3_RX_TERMINATOR2))
        {
            uart3_line_buffer[char_count] = uart3_dma_rx_buffer[char_count];
            char_count++;
        }

        if (char_count > 0U)                                  
        {
            uart3_line_buffer[char_count] = '\0';
            uart3_line_complete = 1U;
            (void)osSemaphoreRelease(uart_line_semaHandle);  
        }
    }

    cnr_uart3_rx_start();                     
}
/**
 * @brief  HAL UART error callback: clears UART3 errors and restarts reception.
 * @details Without this, one overrun / framing error stops UART3 reception until reset.
 */
volatile uint32_t g_uart3_err_count = 0U;    /* number of UART3 errors */
volatile uint32_t g_uart3_last_err  = 0U;    /* HAL_UART_ERROR_* bits of the last error */

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) 
    { 
        return; 
    }

    g_uart3_last_err = huart->ErrorCode;        /* 0x08 ORE, 0x04 FE, 0x02 NE, 0x01 PE */
    g_uart3_err_count++;
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
    uart3_line_length = 0U;
}
