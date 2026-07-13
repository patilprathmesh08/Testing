/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : CNR Motor Driver - FOC with Encoder (RPM Control)
  *
  * KEY FIX: DRV8316 nSLEEP must stay HIGH always. The MCSDK ES_GPIO mode
  * toggles the enable GPIO on every SwitchOn/SwitchOff call, which puts the
  * DRV8316 to sleep and kills the current sense amplifiers. We override the
  * __weak R3_1_SwitchOnPWM/SwitchOffPWM/TurnOnLowSides to never touch nSLEEP.
  *
  * Hardware: STM32G431CB + DRV8316 (3-PWM mode) + ECX SPEED 10M (36V)
  * Encoder:  TIM4 (PA11/PA12), Index on PB6
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "mc_config.h"
#include "parameters_conversion.h"  
#include "pid_regulator.h"
#include "mc_config_common.h"
#include "mc_api.h"
#include "mc_interface.h"
#include "stm32g4xx_ll_usart.h"
#include "r3_1_g4xx_pwm_curr_fdbk.h"
#include "encoder_speed_pos_fdbk.h"
#include "enc_align_ctrl.h"
#include <math.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define TARGET_RPM          300
#define TARGET_RPM_UNIT     ((int16_t)(TARGET_RPM * SPEED_UNIT / U_RPM))
#define RAMP_DURATION_MS    5000
#define MOTOR_START_DELAY_MS  1000
#define MAX_FAULT_RETRIES     5
#define RUN_PRINT_INTERVAL_MS 1000
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
CORDIC_HandleTypeDef hcordic;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
/* -----------------------------------------------------------------------------
 * COMMUTATION OFFSET CORRECTION
 * -----------------------------------------------------------------------------
 * Added to the encoder-derived electrical angle (in mc_tasks_foc.c), applied
 * ONLY while the state machine is in RUN so it never disturbs MCSDK alignment.
 *
 * Why: on this hardware MCSDK's Encoder Alignment Controller settles the
 * electrical zero ~90 electrical degrees from the true rotor d-axis. Without
 * this correction a commanded q-axis (torque) current is mis-projected onto
 * the d-axis: the rotor holds and heats but produces no torque. A -90 degree
 * offset re-aligns the frame so torque current sits on the q-axis.
 *
 * Units: s16 electrical angle, where 65536 s16 = 360 electrical degrees.
 *        -16384 s16 = -90.0 electrical degrees.
 * -------------------------------------------------------------------------- */
volatile int16_t g_el_offset_s16 = -16384;

/* Electrical-angle debug/override hooks consumed by mc_tasks_foc.c.
 * Kept for the commutation path; not exercised in this torque-mode build
 * (g_force_angle_en stays 0, so the forced-angle override is inactive). */
volatile uint8_t g_force_angle_en = 0;   /* 1 = force hElAngle to g_force_angle */
volatile int16_t g_force_angle    = 0;   /* forced electrical angle (s16) when enabled */
volatile int16_t g_el_raw_dbg     = 0;   /* raw pre-offset electrical angle, for debug */

/* Encoder Z-index (PB6) capture. g_index_enc and g_index_seen are written by
 * EXTI9_5_IRQHandler at the first index crossing. Retained as an absolute
 * position reference for the encoder-based position control to follow. */
volatile int16_t g_index_enc  = 0;   /* raw TIM4 count captured at the index pulse */
volatile int16_t g_index_el   = 0;   /* electrical angle at the index pulse (reserved) */
volatile uint8_t g_index_seen = 0;   /* set once the first index pulse is captured */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CORDIC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN 0 */
/* ---------- printf redirect ---------- */
#ifdef __GNUC__
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* ==========================================================================
 * CRITICAL FIX: Override __weak R3_1 PWM functions to prevent nSLEEP toggle.
 *
 * The DRV8316's nSLEEP pin (PC6) is used as ES_GPIO enable in MCSDK.
 * The default R3_1 driver toggles this GPIO on every SwitchOn/SwitchOff,
 * which puts the DRV8316 to sleep - killing the CSA during offset calibration
 * and between state transitions. This causes wrong offsets and no motor movement.
 *
 * These overrides are identical to the MCSDK originals but with ALL GPIO
 * toggle lines removed. nSLEEP is set HIGH once at boot and stays there.
 * ==========================================================================
 */

/* Mask for TIM1 channels CH1/CH1N/CH2/CH2N/CH3/CH3N */
#define TIMxCCER_MASK_CH123_LOCAL  ((uint16_t)(LL_TIM_CHANNEL_CH1|LL_TIM_CHANNEL_CH1N|\
                                               LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH2N|\
                                               LL_TIM_CHANNEL_CH3|LL_TIM_CHANNEL_CH3N))

void R3_1_TurnOnLowSides(PWMC_Handle_t *pHdl, uint32_t ticks)
{
  PWMC_R3_1_Handle_t *pHandle = (PWMC_R3_1_Handle_t *)pHdl;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.TurnOnLowSidesAction = true;

  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_SetCompareCH1(TIMx, ticks);
  LL_TIM_OC_SetCompareCH2(TIMx, ticks);
  LL_TIM_OC_SetCompareCH3(TIMx, ticks);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);

  LL_TIM_EnableAllOutputs(TIMx);
  /* GPIO toggle REMOVED - nSLEEP stays HIGH */
}

void R3_1_SwitchOnPWM(PWMC_Handle_t *pHdl)
{
  PWMC_R3_1_Handle_t *pHandle = (PWMC_R3_1_Handle_t *)pHdl;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.TurnOnLowSidesAction = false;

  /* Set all duty to 50% */
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH4);
  LL_TIM_OC_SetCompareCH1(TIMx, ((uint32_t)pHandle->Half_PWMPeriod / (uint32_t)2));
  LL_TIM_OC_SetCompareCH2(TIMx, ((uint32_t)pHandle->Half_PWMPeriod / (uint32_t)2));
  LL_TIM_OC_SetCompareCH3(TIMx, ((uint32_t)pHandle->Half_PWMPeriod / (uint32_t)2));
  LL_TIM_OC_SetCompareCH4(TIMx, ((uint32_t)pHandle->Half_PWMPeriod - (uint32_t)5));
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH4);

  TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
  LL_TIM_EnableAllOutputs(TIMx);
  /* GPIO toggle REMOVED - nSLEEP stays HIGH */

  pHandle->_Super.PWMState = true;
}

void R3_1_SwitchOffPWM(PWMC_Handle_t *pHdl)
{
  PWMC_R3_1_Handle_t *pHandle = (PWMC_R3_1_Handle_t *)pHdl;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.PWMState = false;
  pHandle->_Super.TurnOnLowSidesAction = false;

  LL_TIM_DisableAllOutputs(TIMx);
  /* GPIO toggle REMOVED - nSLEEP stays HIGH, DRV8316 CSA remains active */
}

/* ---------- Diagnostic helpers ---------- */
static void PrintFaults(uint16_t f)
{
  printf("[FAULT] code=0x%04X\r\n", (unsigned)f);
  if (f & 0x0001U) printf("  -> MC_DURATION\r\n");
  if (f & 0x0002U) printf("  -> MC_OVER_VOLT\r\n");
  if (f & 0x0004U) printf("  -> MC_UNDER_VOLT\r\n");
  if (f & 0x0008U) printf("  -> MC_OVER_TEMP\r\n");
  if (f & 0x0010U) printf("  -> MC_START_UP\r\n");
  if (f & 0x0020U) printf("  -> MC_SPEED_FDBK\r\n");
  if (f & 0x0040U) printf("  -> MC_OVER_CURR\r\n");
  if (f & 0x0080U) printf("  -> MC_SW_ERROR\r\n");
  if (f & 0x0400U) printf("  -> MC_DP_FAULT\r\n");
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_CORDIC_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */

  /* === DRV8316 Wake-up ===
   * Set nSLEEP HIGH ONCE and leave it. The overridden R3_1 functions
   * above will never toggle this pin, keeping the DRV8316 awake at all
   * times so the CSA works during offset calibration.
   */
  HAL_GPIO_WritePin(M1_PWM_EN_UVW_GPIO_Port, M1_PWM_EN_UVW_Pin, GPIO_PIN_SET);
  HAL_Delay(10);  /* DRV8316 wake-up time */

  printf("\r\n========================================\r\n");
  printf("  CNR BLDC FOC - Torque Mode (Encoder Commutation)\r\n");
  printf("========================================\r\n");
  printf("PWM=%uHz  PPR=%d  Target=%dRPM (unit=%d)\r\n",
         (unsigned)PWM_FREQUENCY, M1_ENCODER_PPR, TARGET_RPM, (int)TARGET_RPM_UNIT);
  printf("Vbus(virtual)=%uV  Rs=%.1f  Ls=%.4fmH  PP=%d\r\n",
         (unsigned)NOMINAL_BUS_VOLTAGE_V, RS, LS*1000.0, POLE_PAIR_NUM);
  printf("FIX: nSLEEP override active (never toggled)\r\n");
  printf("========================================\r\n\r\n");

  /* Initialize MCSDK - now calibration will read correct CSA offsets
   * because DRV8316 stays awake (nSLEEP HIGH) throughout. */
  MX_MotorControl_Init();
  MX_NVIC_Init();

  /* Read calibrated offsets */
  {
    PolarizationOffsets_t off;
    R3_1_GetOffsetCalib(pwmcHandle[M1], &off);
    printf("[INIT] Calibrated offsets: A=%u B=%u C=%u\r\n",
           (unsigned)off.phaseAOffset, (unsigned)off.phaseBOffset,
           (unsigned)off.phaseCOffset);
    /* Expect values near 32768 (VCC/2 of DRV8316 CSA). If not, warn. */
    if (off.phaseAOffset < 28000U || off.phaseAOffset > 38000U ||
        off.phaseBOffset < 28000U || off.phaseBOffset > 38000U ||
        off.phaseCOffset < 28000U || off.phaseCOffset > 38000U)
    {
      printf("[WARN] Offsets look wrong! Expected ~32768. Check CSA wiring.\r\n");
    }
    else
    {
      printf("[INIT] Offsets OK (near midscale)\r\n");
    }
  }

  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  printf("[INIT] Encoder count: %d\r\n", (int)(int16_t)LL_TIM_GetCounter(TIM4));

  /* === MCSDK-NATIVE ENCODER ALIGNMENT ===
   * We no longer do a custom open-loop alignment or manual offset. Instead we
   * let MCSDK's built-in Encoder Alignment Controller (EAC) run: on the first
   * MC_StartMotor1(), the state machine sees the encoder is not aligned and
   * enters the ALIGNMENT state, energizing the motor at M1_ALIGNMENT_ANGLE_DEG
   * (90 deg) with FINAL_I_ALIGNMENT_A current for M1_ALIGNMENT_DURATION, then
   * establishes the correct electrical zero automatically. This is the proper
   * MCSDK d/q alignment - no manual ALIGN_ELEC_OFFSET_S16 guessing.
   *
   * NOTE: the earlier reason we bypassed EAC (nSLEEP/CSA calibration issues)
   * is fixed (nSLEEP override keeps DRV8316 awake), so EAC can run correctly.
   */
  printf("[INIT] Using MCSDK native EAC for encoder alignment.\r\n");

  printf("[INIT] Ready. Motor starts in %d ms\r\n\r\n", MOTOR_START_DELAY_MS);

  printf("Motor is running \n");

  /* USER CODE END 2 */

  /* =====================================================================
   *  MCSDK-NATIVE FOC - TORQUE MODE  (working commutation baseline)
   * ---------------------------------------------------------------------
   *  PURPOSE
   *    Run the motor under ST-MCSDK Field Oriented Control in TORQUE mode
   *    and stream clearly-labelled telemetry. This is the validated base
   *    for the next step (encoder-based POSITION control).
   *
   *  COMMUTATION FIX (the key result of this build)
   *    MCSDK's Encoder Alignment Controller (EAC) leaves the electrical
   *    zero ~90 electrical degrees away from the true rotor d-axis on this
   *    hardware. Uncorrected, a commanded q-axis (torque) current lands on
   *    the d-axis instead: the rotor grips and heats but makes no torque.
   *    We correct it at run time with a fixed -90 deg electrical offset,
   *    g_el_offset_s16 = -16384  ( -16384 s16 = -90.0 electrical degrees ),
   *    applied only while the state machine is in RUN (see mc_tasks_foc.c).
   *    With this correction the commanded current sits on the q-axis: the
   *    motor spins freely, Id ~ 0, and current/heat stay low.
   *
   *  CONTROL MODE
   *    TORQUE mode: we command a fixed q-axis current (Iq reference) via
   *    MC_ProgramTorqueRampMotor1(). We do NOT close a speed loop here -
   *    speed regulation is deliberately out of scope for this build.
   *
   *  SENSOR NOTES (this board's MCSDK configuration)
   *    Iq, Id  - REAL, measured from the DRV8316 shunt + ADC.
   *    Vq, Vd  - REAL, the FOC-applied voltages (controller output).
   *    Speed   - REAL, derived from the TIM4 quadrature encoder.
   *    Vbus    - VIRTUAL (no bus-voltage divider fitted): a fixed nominal
   *              value, NOT a measurement - so it is not printed.
   *    Temp    - VIRTUAL (no NTC fitted): a fixed value, NOT a measurement
   *              - so it is not printed. Judge heating from |Imag| instead.
   *
   *  SEQUENCE
   *    phase 0 : wait MOTOR_START_DELAY_MS, then MC_StartMotor1()
   *    phase 1 : MCSDK runs ALIGNMENT -> RUN automatically; on RUN we issue
   *              the torque command
   *    phase 2 : stream telemetry every TELEM_PERIOD_MS
   *    phase 9 : halted (start timed out) - safe idle
   *    A motor fault at any time is reported once and the motor is left
   *    stopped.
   * ===================================================================== */

  /* ---- Test configuration (edit these) ---- */
  const int16_t  TORQUE_REF_COUNTS = 150;   /* q-axis current reference.
                                             * counts -> amps: A = counts / CURRENT_CONV_FACTOR
                                             * 150 counts ~= 50 mA. Raise for more torque. */
  const uint16_t TORQUE_RAMP_MS    = 800;   /* ramp time to the torque target */
  const uint32_t TELEM_PERIOD_MS   = 250;   /* telemetry print period */

  /* ---- Run-loop state ---- */
  uint8_t  run_phase     = 0;   /* 0=wait/start 1=wait-RUN 2=telemetry 9=halted */
  uint32_t phase_t0      = 0;   /* timestamp for phase timeouts */
  uint32_t log_t0        = 0;   /* timestamp for telemetry cadence */
  uint8_t  fault_printed = 0;   /* so a fault is reported only once */

  while (1)
  {
    /* USER CODE BEGIN 3 */
    MCI_State_t state = MCI_GetSTMState(pMCI[M1]);

    if (state == FAULT_NOW || state == FAULT_OVER)
    {
      if (!fault_printed)
      {
        PrintFaults(MC_GetOccurredFaultsMotor1());
        printf("[RUN] Motor in FAULT. Stopped. Fix cause / power-cycle.\r\n");
        fault_printed = 1;
      }
      HAL_Delay(200);
      continue;
    }

    switch (run_phase)
    {
      case 0:  /* Wait the start-up delay, then command the motor to start.
                * MCSDK will automatically run ALIGNMENT and advance to RUN. */
        if (HAL_GetTick() > MOTOR_START_DELAY_MS)
        {
          bool ok = MC_StartMotor1();
          printf("[FOC ] MC_StartMotor1 -> %s  (MCSDK will ALIGN, then enter RUN)\r\n",
                 ok ? "OK" : "FAIL");
          run_phase = 1;
          phase_t0  = HAL_GetTick();
        }
        break;

      case 1:  /* Once MCSDK reaches RUN, command the q-axis torque current.
                * The -90 deg commutation offset (g_el_offset_s16) is already
                * active in RUN, so this current produces real torque. */
        if (state == RUN)
        {
          printf("[FOC ] RUN reached. Commutation offset = %d s16 (-90.0 deg).\r\n",
                 (int)g_el_offset_s16);
          printf("[FOC ] TORQUE mode: Iq reference = %d counts (~%ld mA).\r\n",
                 (int)TORQUE_REF_COUNTS,
                 (long)((int32_t)(((float)TORQUE_REF_COUNTS / CURRENT_CONV_FACTOR) * 1000.0f)));
          MC_ProgramTorqueRampMotor1(TORQUE_REF_COUNTS, TORQUE_RAMP_MS);
          run_phase = 2;
          log_t0    = HAL_GetTick();
        }
        else if (HAL_GetTick() - phase_t0 > 6000U)
        {
          printf("[FOC ] Timed out waiting for RUN (state = %d). Halting.\r\n", (int)state);
          run_phase = 9;
        }
        break;

      case 2:  /* Steady state: stream telemetry at a fixed cadence.
                *
                * Column guide (printed once as a header below):
                *   rpm    - mechanical speed from the encoder (RPM)
                *   Iq_mA  - q-axis (torque-producing) current, milliamps
                *   Id_mA  - d-axis (flux) current, milliamps; ~0 when
                *            commutation is correct
                *   |I|_mA - current magnitude sqrt(Iq^2+Id^2), the heating
                *            proxy (there is no real temperature sensor)
                *   Vq,Vd  - FOC-applied voltages on q and d axes (s16)
                *   enc    - raw TIM4 encoder count (0..CPR-1), wraps each rev
                *   eldeg  - electrical angle used for commutation, degrees */
        if (HAL_GetTick() - log_t0 >= TELEM_PERIOD_MS)
        {
          log_t0 = HAL_GetTick();

          /* --- Gather the live quantities --- */
          qd_t    iqd      = MC_GetIqdMotor1();              /* measured Iq,Id (counts) */
          qd_t    vqd      = MC_GetVqdMotor1();              /* applied  Vq,Vd (s16)    */
          int16_t spd_unit = MC_GetMecSpeedAverageMotor1();  /* speed in SPEED_UNIT      */
          int32_t spd_rpm  = ((int32_t)spd_unit * U_RPM) / SPEED_UNIT;
          int16_t enc_cnt  = (int16_t)LL_TIM_GetCounter(TIM4);

          extern FOCVars_t FOCVars[NBR_OF_MOTORS];
          int16_t el_s16   = FOCVars[M1].hElAngle;           /* electrical angle (s16)   */
          int32_t el_deg   = ((int32_t)el_s16 * 360) / 65536;/* -> electrical degrees    */

          /* Currents in milliamps: mA = counts / CURRENT_CONV_FACTOR * 1000 */
          int32_t iq_ma   = (int32_t)(((float)iqd.q / CURRENT_CONV_FACTOR) * 1000.0f);
          int32_t id_ma   = (int32_t)(((float)iqd.d / CURRENT_CONV_FACTOR) * 1000.0f);
          int32_t imag_ma = (int32_t)((sqrtf((float)iqd.q * iqd.q +
                                             (float)iqd.d * iqd.d)
                                       / CURRENT_CONV_FACTOR) * 1000.0f);

          /* Print a column header once, then aligned data rows. */
          static uint8_t hdr_done = 0;
          if (!hdr_done)
          {
            printf("[TELEM] %6s %7s %7s %7s %7s %7s %6s %6s\r\n",
                   "rpm", "Iq_mA", "Id_mA", "|I|_mA", "Vq", "Vd", "enc", "eldeg");
            hdr_done = 1;
          }
          printf("[TELEM] %6ld %7ld %7ld %7ld %7d %6d %6d %6ld\r\n",
                 (long)spd_rpm,
                 (long)iq_ma,
                 (long)id_ma,
                 (long)imag_ma,
                 (int)vqd.q,
                 (int)vqd.d,
                 (int)enc_cnt,
                 (long)el_deg);
        }
        break;

      case 9:  /* Halted (start timed out). Idle safely; motor is stopped. */
      default:
        break;
    }

    HAL_Delay(5);
    /* USER CODE END 3 */
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

static void MX_NVIC_Init(void)
{
  HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 4, 1);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  HAL_NVIC_SetPriority(TIM4_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

static void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) Error_Handler();

  /* PA1=CH2(SOA), PA2=CH3(SOB), PA3=CH4(SOC) - DRV8316 CSA outputs */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK) Error_Handler();

  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK) Error_Handler();

  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK) Error_Handler();
}

static void MX_CORDIC_Init(void)
{
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK) Error_Handler();
}

static void MX_TIM1_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = ((TIM_CLOCK_DIVIDER) - 1);
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = ((PWM_PERIOD_CYCLES) / 2);
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim1.Init.RepetitionCounter = (REP_COUNTER);
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) Error_Handler();

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = ((PWM_PERIOD_CYCLES) / 4);
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();

  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = (((PWM_PERIOD_CYCLES) / 2) - (HTMIN));
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 3;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) Error_Handler();

  HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM4_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = M1_PULSE_NBR;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  /* Encoder direction: with the phase swap active, the correct polarity is
   * FALLING (the RISING flip caused runaway - angle tracked backward). */
  /* Encoder input-capture polarity. Both channels use RISING so that TIM4
   * counts in the direction that agrees with the commutation electrical
   * angle (with the CH1<->CH3 PWM phase order on this board). A mismatch here
   * causes a positive-feedback lock: the rotor holds at a detent under torque
   * instead of turning. */
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = M1_ENC_IC_FILTER;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = M1_ENC_IC_FILTER;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) Error_Handler();
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* nSLEEP starts LOW, will be set HIGH in main before MCSDK init */
  HAL_GPIO_WritePin(M1_PWM_EN_UVW_GPIO_Port, M1_PWM_EN_UVW_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = M1_PWM_EN_UVW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(M1_PWM_EN_UVW_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = M1_ENCODER_Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(M1_ENCODER_Z_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  printf("[ASSERT] %s:%lu\r\n", file, (unsigned long)line);
}
#endif