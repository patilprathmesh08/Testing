/**
 * @file    main.h (PC test stub)
 * @brief   Replaces the firmware main.h for host unit tests.
 *          Provides only the HAL types and functions that
 *          application/Src/cnr_command_handler.c uses.
 *          The test file supplies the fake function bodies.
 */
#ifndef MAIN_H_STUB
#define MAIN_H_STUB

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* fake UART handle (contents not used by the module) */
typedef struct
{
  int dummy;
} UART_HandleTypeDef;

typedef enum
{
  HAL_OK    = 0,
  HAL_ERROR = 1
} HAL_StatusTypeDef;

/* re-arms 1-byte interrupt reception (faked in the test) */
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size);

#endif /* MAIN_H_STUB */
