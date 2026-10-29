/**
 * @file    cmsis_os.h (PC test stub)
 * @brief   Replaces CMSIS-RTOS v1 for host unit tests.
 *          Provides only what application/Src/cnr_command_handler.c uses.
 *          The test file supplies the fake function bodies.
 */
#ifndef CMSIS_OS_H_STUB
#define CMSIS_OS_H_STUB

typedef void *osSemaphoreId;

typedef enum
{
  osOK = 0
} osStatus;

/* wakes the command handler task (faked in the test: counts calls) */
osStatus osSemaphoreRelease(osSemaphoreId semaphore_id);

#endif /* CMSIS_OS_H_STUB */
