/**
 * @file    test_command_handler.c
 * @brief   Unit tests for application/Src/cnr_command_handler.c
 *
 *   cnr_parse_motion_command():
 *     - ME0 / ME1 and invalid enable values
 *     - M<rad>: positive, negative, zero, signs, decimals, exponent,
 *       float min/max limits, overflow (inf), NaN, hex
 *     - M<rad>,<seconds>: positive, zero, negative, tiny, max, missing
 *     - malformed lines, NULL pointers, output clearing
 *
 *   HAL_UART_RxCpltCallback():
 *     - '\n' and '\r' terminators, "\r\n", empty lines
 *     - line length limits (max fits, max+1 overflow)
 *     - bytes while previous line not processed, back-to-back lines
 *
 * Tests marked SPEC GAP describe the safe/intended behaviour and FAIL with
 * the current parser (they show real holes to fix).
 */
#include <string.h>
#include <float.h>
#include "unity.h"
#include "main.h"
#include "cmsis_os.h"
#include "cnr_command_handler.h"

/* ====================== fakes for module dependencies ====================== */
UART_HandleTypeDef huart3;
osSemaphoreId      uart_line_semHandle;

static int semaphore_release_count = 0;   /* lines signalled to the handler task */
static int receive_rearm_count     = 0;   /* HAL_UART_Receive_IT() calls         */

osStatus osSemaphoreRelease(osSemaphoreId semaphore_id)
{
  (void)semaphore_id;
  semaphore_release_count++;
  return osOK;
}

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
  (void)huart;
  (void)data;
  (void)size;
  receive_rearm_count++;
  return HAL_OK;
}

/* module globals (defined in cnr_command_handler.c) */
extern volatile uint8_t uart3_line_buffer[COMMAND_LINE_MAX_CHARS];
extern volatile uint8_t uart3_received_byte;
extern volatile uint8_t uart3_line_length;
extern volatile uint8_t uart3_line_complete;

/* ============================== helpers ============================== */

/* sends one byte through the UART interrupt callback */
static void feed_byte(char byte)
{
  uart3_received_byte = (uint8_t)byte;
  HAL_UART_RxCpltCallback(&huart3);
}

/* sends a whole string, byte by byte */
static void feed_text(const char *text)
{
  while ('\0' != *text)
  {
    feed_byte(*text++);
  }
}

/* sends the same byte 'count' times */
static void feed_repeated(char byte, int count)
{
  for (int i = 0; i < count; i++)
  {
    feed_byte(byte);
  }
}

/* what the handler task does after taking a line */
static void task_takes_line(void)
{
  uart3_line_complete = 0U;
}

/* parses and expects success; returns the parsed command */
static motion_command_t parse_ok(const char *text)
{
  motion_command_t cmd;
  TEST_ASSERT_EQUAL_UINT8_MESSAGE(1U, cnr_parse_motion_command(text, &cmd), text);
  return cmd;
}

/* parses and expects rejection */
static void parse_rejected(const char *text)
{
  motion_command_t cmd;
  TEST_ASSERT_EQUAL_UINT8_MESSAGE(0U, cnr_parse_motion_command(text, &cmd), text);
}

/* runs before every test: clean module state and fake counters */
void setUp(void)
{
  memset((void *)uart3_line_buffer, 0, COMMAND_LINE_MAX_CHARS);
  uart3_line_length       = 0U;
  uart3_line_complete     = 0U;
  semaphore_release_count = 0;
  receive_rearm_count     = 0;
}

void tearDown(void)
{
}

/* ===================================================================== */
/*  1. Motor enable / disable                                            */
/* ===================================================================== */

void test_enable_ME0_is_disable(void)
{
  TEST_ASSERT_EQUAL_UINT8(CNR_MOTOR_DISABLE, parse_ok("ME0").command_type);
}

void test_enable_ME1_is_enable(void)
{
  TEST_ASSERT_EQUAL_UINT8(CNR_MOTOR_ENABLE, parse_ok("ME1").command_type);
}

void test_enable_clears_move_fields(void)
{
  motion_command_t cmd = parse_ok("ME1");
  TEST_ASSERT_EQUAL_FLOAT(0.0f, cmd.move_radians);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, cmd.duration_seconds);
}

void test_enable_rejects_other_values(void)
{
  parse_rejected("ME");      /* missing value */
  parse_rejected("ME2");     /* out of range  */
  parse_rejected("ME9");
  parse_rejected("MEx");     /* not a digit   */
  parse_rejected("ME-1");    /* negative      */
}

/* SPEC GAP: extra characters after ME0/ME1 must be rejected */
void test_enable_rejects_trailing_characters(void)
{
  parse_rejected("ME01");
  parse_rejected("ME10");
  parse_rejected("ME1x");
  parse_rejected("ME1 2");
}

/* ===================================================================== */
/*  2. Move: sign and value range                                        */
/* ===================================================================== */

void test_move_positive_integer(void)
{
  motion_command_t cmd = parse_ok("M5");
  TEST_ASSERT_EQUAL_UINT8(CNR_MOTOR_MOVE, cmd.command_type);
  TEST_ASSERT_EQUAL_FLOAT(5.0f, cmd.move_radians);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, cmd.duration_seconds);     /* no duration given */
}

void test_move_negative_integer(void)
{
  TEST_ASSERT_EQUAL_FLOAT(-5.0f, parse_ok("M-5").move_radians);
}

void test_move_explicit_plus_sign(void)
{
  TEST_ASSERT_EQUAL_FLOAT(3.0f, parse_ok("M+3").move_radians);
}

void test_move_zero_values(void)
{
  TEST_ASSERT_EQUAL_FLOAT(0.0f, parse_ok("M0").move_radians);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, parse_ok("M0.0").move_radians);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, parse_ok("M-0").move_radians);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, parse_ok("M+0").move_radians);
}

void test_move_decimals(void)
{
  TEST_ASSERT_EQUAL_FLOAT(0.4f,     parse_ok("M0.4").move_radians);
  TEST_ASSERT_EQUAL_FLOAT(-1.25f,   parse_ok("M-1.25").move_radians);
  TEST_ASSERT_EQUAL_FLOAT(0.5f,     parse_ok("M.5").move_radians);      /* no leading 0 */
  TEST_ASSERT_EQUAL_FLOAT(-0.5f,    parse_ok("M-.5").move_radians);
  TEST_ASSERT_EQUAL_FLOAT(5.0f,     parse_ok("M5.").move_radians);      /* trailing dot */
  TEST_ASSERT_EQUAL_FLOAT(3.14159f, parse_ok("M3.14159").move_radians);
}

void test_move_small_values(void)
{
  TEST_ASSERT_EQUAL_FLOAT(0.001f,  parse_ok("M0.001").move_radians);
  TEST_ASSERT_EQUAL_FLOAT(-0.001f, parse_ok("M-0.001").move_radians);
}

void test_move_large_values(void)
{
  TEST_ASSERT_EQUAL_FLOAT(1000.0f,   parse_ok("M1000").move_radians);
  TEST_ASSERT_EQUAL_FLOAT(-10000.0f, parse_ok("M-10000").move_radians);
}

void test_move_exponent_notation(void)
{
  TEST_ASSERT_EQUAL_FLOAT(100.0f, parse_ok("M1e2").move_radians);
  TEST_ASSERT_EQUAL_FLOAT(0.01f,  parse_ok("M1E-2").move_radians);
}

void test_move_float_max_and_min(void)
{
  TEST_ASSERT_EQUAL_FLOAT(FLT_MAX,  parse_ok("M3.4028235e38").move_radians);
  TEST_ASSERT_EQUAL_FLOAT(-FLT_MAX, parse_ok("M-3.4028235e38").move_radians);
}

void test_move_tiny_value_becomes_about_zero(void)
{
  TEST_ASSERT_EQUAL_FLOAT(0.0f, parse_ok("M1e-50").move_radians);   /* below float range */
}

/* SPEC GAP: values beyond float range (become inf) must be rejected */
void test_move_rejects_overflow(void)
{
  parse_rejected("M1e39");
  parse_rejected("M-1e39");
}

/* SPEC GAP: inf / nan text must be rejected */
void test_move_rejects_inf_nan(void)
{
  parse_rejected("Minf");
  parse_rejected("M-inf");
  parse_rejected("Mnan");
  parse_rejected("Minfinity");
}

/* SPEC GAP: hexadecimal numbers (strtof accepts "0x10" = 16) must be rejected */
void test_move_rejects_hex(void)
{
  parse_rejected("M0x10");
}

/* ===================================================================== */
/*  3. Move with duration                                                */
/* ===================================================================== */

void test_duration_positive(void)
{
  motion_command_t cmd = parse_ok("M45,2");
  TEST_ASSERT_EQUAL_FLOAT(45.0f, cmd.move_radians);
  TEST_ASSERT_EQUAL_FLOAT(2.0f,  cmd.duration_seconds);
}

void test_duration_with_negative_move(void)
{
  motion_command_t cmd = parse_ok("M-45,2.5");
  TEST_ASSERT_EQUAL_FLOAT(-45.0f, cmd.move_radians);
  TEST_ASSERT_EQUAL_FLOAT(2.5f,   cmd.duration_seconds);
}

void test_duration_zero_is_follow_mode(void)
{
  TEST_ASSERT_EQUAL_FLOAT(0.0f, parse_ok("M1,0").duration_seconds);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, parse_ok("M1,0.0").duration_seconds);
}

void test_duration_negative_becomes_zero(void)
{
  TEST_ASSERT_EQUAL_FLOAT(0.0f, parse_ok("M1,-2").duration_seconds);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, parse_ok("M1,-0.001").duration_seconds);
}

void test_duration_small_and_large(void)
{
  TEST_ASSERT_EQUAL_FLOAT(0.001f,  parse_ok("M1,0.001").duration_seconds);
  TEST_ASSERT_EQUAL_FLOAT(3600.0f, parse_ok("M1,3600").duration_seconds);
  TEST_ASSERT_EQUAL_FLOAT(FLT_MAX, parse_ok("M1,3.4028235e38").duration_seconds);
}

void test_duration_missing_or_invalid(void)
{
  parse_rejected("M1,");       /* comma without value */
  parse_rejected("M1,abc");
  parse_rejected("M1,,2");
  parse_rejected("M,2");       /* no move value       */
}

/* SPEC GAP: extra fields / garbage after duration must be rejected */
void test_duration_rejects_trailing_characters(void)
{
  parse_rejected("M1,2,3");
  parse_rejected("M1,2x");
}

/* SPEC GAP: inf duration must be rejected */
void test_duration_rejects_inf(void)
{
  parse_rejected("M1,inf");
  parse_rejected("M1,1e39");
}

/* ===================================================================== */
/*  4. Malformed lines and pointers                                      */
/* ===================================================================== */

void test_rejects_null_pointers(void)
{
  motion_command_t cmd;
  TEST_ASSERT_EQUAL_UINT8(0U, cnr_parse_motion_command(NULL, &cmd));
  TEST_ASSERT_EQUAL_UINT8(0U, cnr_parse_motion_command("M1", NULL));
}

void test_rejects_wrong_or_missing_header(void)
{
  parse_rejected("");          /* empty         */
  parse_rejected("X1");        /* wrong letter  */
  parse_rejected("m1");        /* lowercase     */
  parse_rejected(" M1");       /* leading space */
  parse_rejected("1");         /* no header     */
}

void test_rejects_missing_number(void)
{
  parse_rejected("M");
  parse_rejected("Mabc");
  parse_rejected("M-");
  parse_rejected("M.");
  parse_rejected("M+");
}

/* SPEC GAP: trailing garbage after a move value must be rejected */
void test_move_rejects_trailing_characters(void)
{
  parse_rejected("M0.4x");
  parse_rejected("M1 2");
  parse_rejected("M1.2.3");
}

/* ===================================================================== */
/*  5. UART line assembly (interrupt callback)                           */
/* ===================================================================== */

void test_uart_newline_completes_line(void)
{
  feed_text("ME1\n");
  TEST_ASSERT_EQUAL_STRING("ME1", (const char *)uart3_line_buffer);
  TEST_ASSERT_EQUAL_UINT8(1U, uart3_line_complete);
  TEST_ASSERT_EQUAL_INT(1, semaphore_release_count);
  TEST_ASSERT_EQUAL_INT(4, receive_rearm_count);      /* re-armed after every byte */
}

void test_uart_carriage_return_completes_line(void)
{
  feed_text("M0.4\r");
  TEST_ASSERT_EQUAL_STRING("M0.4", (const char *)uart3_line_buffer);
  TEST_ASSERT_EQUAL_INT(1, semaphore_release_count);
}

void test_uart_crlf_gives_one_line(void)
{
  feed_text("ME1\r\n");                                /* '\n' arrives while line pending */
  TEST_ASSERT_EQUAL_INT(1, semaphore_release_count);
  task_takes_line();
  TEST_ASSERT_EQUAL_UINT8(0U, uart3_line_length);      /* no leftover */
}

void test_uart_empty_lines_ignored(void)
{
  feed_text("\n");
  feed_text("\r");
  feed_text("\r\n");
  TEST_ASSERT_EQUAL_UINT8(0U, uart3_line_complete);
  TEST_ASSERT_EQUAL_INT(0, semaphore_release_count);
}

void test_uart_two_lines_after_processing(void)
{
  feed_text("ME1\n");
  task_takes_line();
  feed_text("M-2,1\n");
  TEST_ASSERT_EQUAL_STRING("M-2,1", (const char *)uart3_line_buffer);
  TEST_ASSERT_EQUAL_INT(2, semaphore_release_count);
}

void test_uart_bytes_dropped_while_line_pending(void)
{
  feed_text("ME1\n");
  feed_text("ME0\n");                                  /* task has not taken first line */
  TEST_ASSERT_EQUAL_STRING("ME1", (const char *)uart3_line_buffer);
  TEST_ASSERT_EQUAL_INT(1, semaphore_release_count);
}

void test_uart_max_length_line_fits(void)
{
  char expected[COMMAND_LINE_MAX_CHARS];
  memset(expected, '1', COMMAND_LINE_MAX_CHARS - 1U);
  expected[COMMAND_LINE_MAX_CHARS - 1U] = '\0';       /* 63 chars + NUL */

  feed_repeated('1', (int)COMMAND_LINE_MAX_CHARS - 1);
  feed_byte('\n');

  TEST_ASSERT_EQUAL_STRING(expected, (const char *)uart3_line_buffer);
  TEST_ASSERT_EQUAL_INT(1, semaphore_release_count);
}

void test_uart_overlong_line_is_not_signalled(void)
{
  feed_repeated('1', (int)COMMAND_LINE_MAX_CHARS);     /* one char too many */
  feed_byte('\n');
  TEST_ASSERT_EQUAL_INT(0, semaphore_release_count);
}

/* SPEC GAP: the tail of an over-long line must not become a new command.
   Today: after overflow the next bytes start a new line. */
void test_uart_overlong_line_tail_is_discarded(void)
{
  feed_repeated('1', (int)COMMAND_LINE_MAX_CHARS);     /* overflow */
  feed_text("ME1\n");                                  /* tail     */
  TEST_ASSERT_EQUAL_INT(0, semaphore_release_count);
}

/* ============================== runner ============================== */
int main(void)
{
  UNITY_BEGIN();

  /* 1. enable / disable */
  RUN_TEST(test_enable_ME0_is_disable);
  RUN_TEST(test_enable_ME1_is_enable);
  RUN_TEST(test_enable_clears_move_fields);
  RUN_TEST(test_enable_rejects_other_values);
  RUN_TEST(test_enable_rejects_trailing_characters);         /* SPEC GAP */

  /* 2. move value */
  RUN_TEST(test_move_positive_integer);
  RUN_TEST(test_move_negative_integer);
  RUN_TEST(test_move_explicit_plus_sign);
  RUN_TEST(test_move_zero_values);
  RUN_TEST(test_move_decimals);
  RUN_TEST(test_move_small_values);
  RUN_TEST(test_move_large_values);
  RUN_TEST(test_move_exponent_notation);
  RUN_TEST(test_move_float_max_and_min);
  RUN_TEST(test_move_tiny_value_becomes_about_zero);
  RUN_TEST(test_move_rejects_overflow);                      /* SPEC GAP */
  RUN_TEST(test_move_rejects_inf_nan);                       /* SPEC GAP */
  RUN_TEST(test_move_rejects_hex);                           /* SPEC GAP */

  /* 3. duration */
  RUN_TEST(test_duration_positive);
  RUN_TEST(test_duration_with_negative_move);
  RUN_TEST(test_duration_zero_is_follow_mode);
  RUN_TEST(test_duration_negative_becomes_zero);
  RUN_TEST(test_duration_small_and_large);
  RUN_TEST(test_duration_missing_or_invalid);
  RUN_TEST(test_duration_rejects_trailing_characters);       /* SPEC GAP */
  RUN_TEST(test_duration_rejects_inf);                       /* SPEC GAP */

  /* 4. malformed */
  RUN_TEST(test_rejects_null_pointers);
  RUN_TEST(test_rejects_wrong_or_missing_header);
  RUN_TEST(test_rejects_missing_number);
  RUN_TEST(test_move_rejects_trailing_characters);           /* SPEC GAP */

  /* 5. UART callback */
  RUN_TEST(test_uart_newline_completes_line);
  RUN_TEST(test_uart_carriage_return_completes_line);
  RUN_TEST(test_uart_crlf_gives_one_line);
  RUN_TEST(test_uart_empty_lines_ignored);
  RUN_TEST(test_uart_two_lines_after_processing);
  RUN_TEST(test_uart_bytes_dropped_while_line_pending);
  RUN_TEST(test_uart_max_length_line_fits);
  RUN_TEST(test_uart_overlong_line_is_not_signalled);
  RUN_TEST(test_uart_overlong_line_tail_is_discarded);       /* SPEC GAP */

  return UNITY_END();
}
