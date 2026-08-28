/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---- Drive geometry -------------------------------------------------------
 * PULSES_PER_REV must match the CL86Y DIP switches.
 * GEAR_RATIO = wheel teeth / worm starts.
 *   80 teeth, single-start worm -> 80.0
 *   80 teeth, double-start worm -> 40.0
 */
#define PULSES_PER_REV   1600.0f
#define GEAR_RATIO       80.0f
#define STEPS_PER_DEG    (PULSES_PER_REV * GEAR_RATIO / 360.0f)

/* ---- Soft travel limits ---- */
#define AOA_MIN_DEG      -20.0f
#define AOA_MAX_DEG       20.0f

/* ---- Motion profile -------------------------------------------------------
 * Trapezoidal: start slow, accelerate to cruise, decelerate before arrival.
 * A NEMA 34 rotor going from rest to full rate in one step puts a shock
 * through the worm teeth, and the closed-loop driver has to work hard to
 * catch up. Ramping removes both problems and lets the cruise rate be much
 * higher than a fixed rate could safely be.
 *
 * At 5 kHz, 1600 pulses/rev and 80:1, the airfoil moves 14 deg/s, so a full
 * -20 to +20 sweep takes under three seconds.
 */
#define START_RATE_HZ    500u
#define MAX_RATE_HZ      5000u
#define RAMP_STEPS       800u       /* steps spent accelerating */

/* ---- Status stream period ---- */
#define STATUS_MS        100u

/* ---- DIR pin levels. Swap if positive angle drives the wrong way. ---- */
#define DIR_POSITIVE     GPIO_PIN_SET
#define DIR_NEGATIVE     GPIO_PIN_RESET

/* ---- MF (enable). High = opto dark = motor holding. ---- */
#define MOTOR_HOLD       GPIO_PIN_SET
#define MOTOR_RELEASE    GPIO_PIN_RESET

/* ---- Serial line buffer ---- */
#define RX_LINE_MAX      32u

/* ---- Mux scanning ---------------------------------------------------------
 * Two CD74HC4067s share the four select lines, so one address selects a
 * channel on both at once. Each scan step reads both SIG outputs, so 16
 * steps collect 32 channels of which 24 are used:
 *
 *   mux A (PA0) channels 0..11  -> upper surface taps  1..12
 *   mux B (PA1) channels 0..11  -> lower surface taps 13..24
 *   channels 12..15 on both     -> spare
 */
#define MUX_CHANNELS     16u
#define TAPS_PER_MUX     12u
#define MUX_SETTLE_US    10u

/* ---- Position storage -----------------------------------------------------
 * The worm is self-locking, so the airfoil cannot move while the motor is
 * off. That means the angle at power-down is still the angle at power-up,
 * and storing the step count is enough to survive a power cycle without any
 * homing hardware.
 *
 * Sector 7 of the F411RE is 128 KB at 0x08060000, well clear of the program.
 *
 * Records are appended rather than overwriting one location. Flash bits can
 * go 1 -> 0 without an erase, so a fresh record can be written into the next
 * blank slot; only when the sector fills does it need erasing. Erasing on
 * every save would wear the sector out after about ten thousand moves, which
 * is roughly 250 sweeps - reachable within a year of class use. Appending
 * pushes that out by four orders of magnitude.
 */
#define STORE_BASE       0x08060000UL
#define STORE_SECTOR     FLASH_SECTOR_7
#define STORE_SIZE       (128u * 1024u)
#define STORE_MAGIC      0xA5F00FA5UL
#define STORE_SLOTS      (STORE_SIZE / 8u)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Motor state, shared with the TIM1 interrupt */
static volatile int32_t  pos_steps   = 0;
static volatile uint32_t steps_left  = 0;
static volatile uint32_t steps_total = 0;
static volatile uint32_t steps_done  = 0;
static volatile int8_t   step_dir    = 0;
static volatile uint8_t  save_due    = 0;   /* set by ISR, acted on in main */

/* Serial receive, shared with the UART interrupt */
static uint8_t          rx_byte;
static char             rx_line[RX_LINE_MAX];
static volatile uint8_t rx_len = 0;
static volatile uint8_t line_ready = 0;

/* Latest raw ADC counts.
 *   [0..15]  mux A channels 0..15
 *   [16..31] mux B channels 0..15
 */
static uint16_t raw[MUX_CHANNELS * 2];

/* Set by the 'R' command: stream all 32 channels instead of the 24 taps */
static uint8_t raw_mode = 0;

/* Last value written to flash, so unchanged positions are not rewritten */
static int32_t saved_pos = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
static void     motor_init(void);
static void     motor_apply_rate(uint32_t hz);
static uint32_t rate_for_step(uint32_t done, uint32_t total);
static int      motor_move_to(float deg);
static void     motor_stop(void);
static int      motor_busy(void);
static float    motor_angle(void);
static void     delay_us(uint32_t us);
static void     mux_select(uint8_t ch);
static void     scan_all(void);
static int32_t  store_load(void);
static void     store_save(int32_t value);
static void     uart_send(const char *s);
static void     send_status(void);
static void     handle_line(char *line);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ---- Position storage ----------------------------------------------------*/

/* Walk the sector and return the last valid record, or 0 if there is none. */
static int32_t store_load(void)
{
  const uint32_t *p = (const uint32_t *)STORE_BASE;
  int32_t found = 0;

  for (uint32_t i = 0; i < STORE_SLOTS; i++)
  {
    if (p[i * 2] != STORE_MAGIC) break;      /* first blank slot ends it */
    found = (int32_t)p[i * 2 + 1];
  }
  return found;
}

/* Append a record. Erases and restarts when the sector is full. */
static void store_save(int32_t value)
{
  const uint32_t *p = (const uint32_t *)STORE_BASE;

  uint32_t slot = 0;
  while (slot < STORE_SLOTS && p[slot * 2] == STORE_MAGIC) slot++;

  HAL_FLASH_Unlock();

  if (slot >= STORE_SLOTS)
  {
    FLASH_EraseInitTypeDef er = {0};
    uint32_t err = 0;
    er.TypeErase    = FLASH_TYPEERASE_SECTORS;
    er.Sector       = STORE_SECTOR;
    er.NbSectors    = 1;
    er.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_FLASHEx_Erase(&er, &err);
    slot = 0;
  }

  uint32_t addr = STORE_BASE + slot * 8u;

  /* Value first, then magic. If power is lost between the two writes the
   * record has no magic and is simply ignored on the next boot, leaving the
   * previous record as the newest valid one.
   */
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 4u, (uint32_t)value);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, STORE_MAGIC);

  HAL_FLASH_Lock();
  saved_pos = value;
}

/* ---- Motor ---------------------------------------------------------------*/

static void motor_init(void)
{
  __HAL_TIM_SET_PRESCALER(&htim1, 83);          /* 84 MHz -> 1 MHz tick */
  motor_apply_rate(START_RATE_HZ);
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);

  HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, MOTOR_HOLD);
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, DIR_POSITIVE);
}

/* With auto-reload preload enabled these take effect at the next update
 * event, so the pulse currently in flight is never truncated.
 */
static void motor_apply_rate(uint32_t hz)
{
  uint32_t arr = (1000000u / hz) - 1u;
  __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (arr + 1u) / 2u);
}

/* Trapezoidal profile. Ramp length is halved for short moves so the
 * acceleration and deceleration phases never overlap.
 */
static uint32_t rate_for_step(uint32_t done, uint32_t total)
{
  uint32_t ramp = RAMP_STEPS;
  if (ramp > total / 2u) ramp = total / 2u;
  if (ramp == 0u) return START_RATE_HZ;     /* move too short to ramp */

  uint32_t remaining = total - done;
  uint32_t phase;

  if (done < ramp)            phase = done;         /* accelerating */
  else if (remaining <= ramp) phase = remaining;    /* decelerating */
  else                        return MAX_RATE_HZ;   /* cruising     */

  return START_RATE_HZ + (MAX_RATE_HZ - START_RATE_HZ) * phase / ramp;
}

/* Absolute move, non-blocking. Returns 0 on success, -1 if already moving.
 * Target is computed fresh from the angle every time, never accumulated, so
 * rounding error stays under one step however long the rig runs.
 */
static int motor_move_to(float deg)
{
  if (motor_busy()) return -1;

  if (deg < AOA_MIN_DEG) deg = AOA_MIN_DEG;
  if (deg > AOA_MAX_DEG) deg = AOA_MAX_DEG;

  int32_t target = (int32_t)lroundf(deg * STEPS_PER_DEG);
  int32_t delta  = target - pos_steps;
  if (delta == 0) return 0;

  step_dir = (delta > 0) ? 1 : -1;
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin,
                    (delta > 0) ? DIR_POSITIVE : DIR_NEGATIVE);

  for (volatile int i = 0; i < 500; i++) { __NOP(); }  /* DIR setup time */

  steps_total = (uint32_t)((delta > 0) ? delta : -delta);
  steps_done  = 0;
  steps_left  = steps_total;

  motor_apply_rate(START_RATE_HZ);
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  return 0;
}

static void motor_stop(void)
{
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  steps_left = 0;
  save_due = 1;
}

static int motor_busy(void)
{
  return (steps_left > 0);
}

static float motor_angle(void)
{
  return (float)pos_steps / STEPS_PER_DEG;
}

/* One step per timer update. The rate for the next step is computed here,
 * which is what produces the ramp.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM1) return;
  if (steps_left == 0) return;

  pos_steps += step_dir;
  steps_done++;
  steps_left--;

  if (steps_left == 0)
  {
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    save_due = 1;                 /* flash write happens in the main loop */
  }
  else
  {
    motor_apply_rate(rate_for_step(steps_done, steps_total));
  }
}

/* ---- Mux scanning --------------------------------------------------------*/

/* Short busy-wait. HAL_Delay only resolves to 1 ms, which would stretch a
 * full scan from a few hundred microseconds to 16 ms.
 */
static void delay_us(uint32_t us)
{
  uint32_t cycles = us * (SystemCoreClock / 1000000u) / 4u;
  while (cycles--) { __NOP(); }
}

/* Write a 4-bit channel address. S0 is the least significant bit. Get this
 * backwards and every tap ends up at the wrong chord position, which is very
 * hard to spot once 24 tubes are connected.
 */
static void mux_select(uint8_t ch)
{
  HAL_GPIO_WritePin(MUX_S0_GPIO_Port, MUX_S0_Pin,
                    (ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MUX_S1_GPIO_Port, MUX_S1_Pin,
                    (ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MUX_S2_GPIO_Port, MUX_S2_Pin,
                    (ch & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MUX_S3_GPIO_Port, MUX_S3_Pin,
                    (ch & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* One full pass: 16 addresses, both ADC inputs read at each. */
static void scan_all(void)
{
  for (uint8_t ch = 0; ch < MUX_CHANNELS; ch++)
  {
    mux_select(ch);
    delay_us(MUX_SETTLE_US);

    HAL_ADC_Start(&hadc1);

    /* Rank 1 = PA0 = mux A */
    if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK)
      raw[ch] = (uint16_t)HAL_ADC_GetValue(&hadc1);

    /* Rank 2 = PA1 = mux B */
    if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK)
      raw[MUX_CHANNELS + ch] = (uint16_t)HAL_ADC_GetValue(&hadc1);

    HAL_ADC_Stop(&hadc1);
  }
}

/* ---- Serial --------------------------------------------------------------*/

static void uart_send(const char *s)
{
  HAL_UART_Transmit(&huart2, (const uint8_t *)s, strlen(s), HAL_MAX_DELAY);
}

/* Byte-at-a-time receive. CR or LF ends a line. The ISR only fills the
 * buffer and raises a flag; parsing happens in the main loop.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2) return;

  char c = (char)rx_byte;

  if (c == '\n' || c == '\r')
  {
    if (rx_len > 0 && !line_ready)
    {
      rx_line[rx_len] = '\0';
      line_ready = 1;
    }
    rx_len = 0;
  }
  else if (rx_len < RX_LINE_MAX - 1)
  {
    rx_line[rx_len++] = c;
  }
  else
  {
    rx_len = 0;                          /* overlong line: discard */
  }

  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);   /* re-arm */
}

/* Status line.
 *
 *   normal:  S,<angle>,<moving>,<t1>..<t24>      the 24 tap channels
 *   raw:     S,<angle>,<moving>,<c0>..<c31>      every channel, both muxes
 *
 * Values are raw ADC counts, 0 to 4095. Converting to pressure happens on
 * the PC so the calibration can change without reflashing.
 */
static void send_status(void)
{
  char buf[320];
  int n = snprintf(buf, sizeof(buf), "S,%.3f,%d",
                   motor_angle(), motor_busy() ? 1 : 0);

  if (raw_mode)
  {
    for (uint8_t i = 0; i < MUX_CHANNELS * 2; i++)
      n += snprintf(buf + n, sizeof(buf) - n, ",%u", raw[i]);
  }
  else
  {
    for (uint8_t i = 0; i < TAPS_PER_MUX; i++)          /* upper surface */
      n += snprintf(buf + n, sizeof(buf) - n, ",%u", raw[i]);
    for (uint8_t i = 0; i < TAPS_PER_MUX; i++)          /* lower surface */
      n += snprintf(buf + n, sizeof(buf) - n, ",%u", raw[MUX_CHANNELS + i]);
  }

  snprintf(buf + n, sizeof(buf) - n, "\r\n");
  uart_send(buf);
}

/* ---- Command set ----------------------------------------------------------
 *  A<angle>   absolute move, e.g. A12.5 or A-7.25
 *  X          stop immediately
 *  Z          set current position as zero, and store it
 *  R          toggle raw channel streaming (32 values instead of 24)
 *  ?          identify
 */
static void handle_line(char *line)
{
  char cmd = line[0];

  if (cmd == 'A' || cmd == 'a')
  {
    char *end;
    float deg = strtof(line + 1, &end);
    if (end == line + 1)
    {
      uart_send("ERR,PARSE\r\n");
      return;
    }
    if (motor_move_to(deg) == 0) uart_send("OK\r\n");
    else                         uart_send("ERR,BUSY\r\n");
  }
  else if (cmd == 'X' || cmd == 'x')
  {
    motor_stop();
    uart_send("OK\r\n");
  }
  else if (cmd == 'Z' || cmd == 'z')
  {
    if (motor_busy())
    {
      uart_send("ERR,BUSY\r\n");
    }
    else
    {
      pos_steps = 0;
      store_save(0);
      uart_send("OK\r\n");
    }
  }
  else if (cmd == 'R' || cmd == 'r')
  {
    raw_mode = !raw_mode;
    uart_send(raw_mode ? "OK,RAW\r\n" : "OK,TAPS\r\n");
  }
  else if (cmd == '?')
  {
    uart_send("AFD,1.2\r\n");
  }
  else
  {
    uart_send("ERR,CMD\r\n");
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  motor_init();

  /* Pick up where the last session left off. The self-locking worm means the
   * airfoil cannot have moved while the power was off.
   */
  pos_steps = store_load();
  saved_pos = pos_steps;

  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  uart_send("AFD,1.2\r\n");

  uint32_t t_status = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    scan_all();          /* a few hundred microseconds, runs continuously */

    /* Flash writes stall the CPU while they complete, which would delay the
     * step interrupt. Doing them here rather than in the ISR means they only
     * ever happen with the motor stationary.
     */
    if (save_due)
    {
      save_due = 0;
      if (pos_steps != saved_pos) store_save(pos_steps);
    }

    if (line_ready)
    {
      handle_line(rx_line);
      line_ready = 0;
    }

    if (HAL_GetTick() - t_status >= STATUS_MS)
    {
      t_status = HAL_GetTick();
      send_status();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data
  * Alignment and number of conversions)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in
  * the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in
  * the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 99;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR_Pin|ENA_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MUX_S0_Pin|MUX_S1_Pin|MUX_S2_Pin|MUX_S3_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_Pin ENA_Pin */
  GPIO_InitStruct.Pin = DIR_Pin|ENA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MUX_S0_Pin MUX_S1_Pin MUX_S2_Pin MUX_S3_Pin */
  GPIO_InitStruct.Pin = MUX_S0_Pin|MUX_S1_Pin|MUX_S2_Pin|MUX_S3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
