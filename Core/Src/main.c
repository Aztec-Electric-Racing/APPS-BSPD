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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ee.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
 uint32_t sens1Lower;
 uint32_t sens2Lower;
 uint32_t sens1Upper;
 uint32_t sens2Upper;

} Storage_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_MAX_COUNTS          4095

/* The APPS input divider passes approximately 72% of the sensor voltage.
 * Calibration values are kept in sensor-side counts; normalize ADC readings
 * before checking ranges and computing the APPS differential. */
#define APPS_INPUT_DIVIDER_NUM  51U
#define APPS_INPUT_DIVIDER_DEN  100U

/* Allowed APPS1/APPS2 disagreement as a fraction of pedal travel (FSAE: 10%) */
#define SENS_ACCEPTABLE_DIFF    0.10f
#define APPS_MISMATCH_TIME_MS   100U

/* How often the console heartbeat line is printed when enabled */
#define HEARTBEAT_PERIOD_MS     1000

/* BRAKEREF is divided before reaching the ADC. Switch at the sensor-side
 * midpoint, then convert through the divider to 12-bit ADC counts. */
#define BRAKE_DIVIDER_NUM          72U
#define BRAKE_DIVIDER_DEN          100U

#define BRAKE_RELEASED_MV          490U
#define BRAKE_ENGAGED_MV           1240U
#define BRAKE_THRESHOLD_SENSOR_MV  ((BRAKE_RELEASED_MV + BRAKE_ENGAGED_MV) / 2U)

#define BRAKE_THRESHOLD_ADC_MV \
    ((BRAKE_THRESHOLD_SENSOR_MV * BRAKE_DIVIDER_NUM + \
      (BRAKE_DIVIDER_DEN / 2U)) / BRAKE_DIVIDER_DEN)

#define BRAKE_ENGAGED_COUNTS \
    ((BRAKE_THRESHOLD_ADC_MV * ADC_MAX_COUNTS + 1650U) / 3300U)
/* PA7 (APPS/BC_Fail) is the throttle/brake inhibit output: HIGH = OK/enabled, LOW = inhibit.
 * The board has a 10k pull-down (R3), so a Hi-Z pin also inhibits. PB1/PB5 are fan/pump PWM. */
#define INHIBIT_GPIO_PORT       GPIOA
#define INHIBIT_GPIO_PIN        GPIO_PIN_7
#define APPS_TRIP_PERMILLE      250U  /* Trip only above 25.0% travel. */
#define APPS_CLEAR_PERMILLE     50U   /* Clear only below 5.0% travel. */

/* Converts ADC counts to millivolts at the MCU pin (before any external divider) */
#define COUNTS_TO_MV(c)         ((uint32_t)(c) * 3300U / 4095U)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
volatile uint16_t adc_buf[3];   // [0] = PC4/APPS1, [1] = PC5/APPS2, [2] = PA3/brake
volatile uint16_t apps1 = 0;
volatile uint16_t apps2 = 0;
volatile uint16_t brakeAdc = 0;
uint16_t checkedApps1 = 0;
uint16_t checkedApps2 = 0;

//Emulated EEPROM
Storage_t ee;

//Derived from ee, recalculate with UpdateSensorRanges() whenever ee changes
uint16_t sens1Range = 0;
uint16_t sens2Range = 0;

uint8_t appsFault = 1;
uint8_t brakesPressed = 0;
uint8_t throttleBrakeLatched = 1;
uint16_t throttlePermille = 0;
uint16_t apps1Permille = 0;
uint16_t apps2Permille = 0;
uint16_t brakePermille = 0;
static uint8_t appsMismatchTiming = 0U;
static uint32_t appsMismatchStartTick = 0U;

uint8_t heartbeatEnabled = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
static void LoadDefaultCalibration(void);
static uint8_t CalibrationIsValid(void);
static void UpdateSensorRanges(void);
static void InhibitForceLow(void);
static uint16_t AdcToSensorCounts(uint16_t adcCounts);
static uint16_t GetThrottlePermille(uint16_t sensor1, uint16_t sensor2);
static uint16_t GetSensorPermille(uint16_t sensor, uint32_t lower, uint16_t range);
static void UpdateAppsAndShutdown(void);
static void UpdateThrottleBrakeLatch(uint8_t sensorsValid);
static void HandleConsoleCommand(const char* cmd);
static void SendStatusLine(const char* prefix);
static void ConsoleSendString(const char* str);
static void HandleConsoleInput(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void LoadDefaultCalibration(void)
{
  ee.sens1Lower = (uint32_t)((1.543 / 3.3) * ADC_MAX_COUNTS);
  ee.sens2Lower = (uint32_t)((0.87 / 3.3) * ADC_MAX_COUNTS);
  ee.sens1Upper = (uint32_t)((2.216 / 3.3) * ADC_MAX_COUNTS);
  ee.sens2Upper = (uint32_t)((1.543 / 3.3) * ADC_MAX_COUNTS);
}

/* Erased flash reads as 0xFFFFFFFF, so this also catches a board that has never been calibrated */
static uint8_t CalibrationIsValid(void)
{
  return (ee.sens1Lower < ee.sens1Upper) && (ee.sens1Upper <= ADC_MAX_COUNTS) &&
         (ee.sens2Lower < ee.sens2Upper) && (ee.sens2Upper <= ADC_MAX_COUNTS);
}

static void UpdateSensorRanges(void)
{
  sens1Range = (uint16_t)abs((int32_t)ee.sens1Upper - (int32_t)ee.sens1Lower);
  sens2Range = (uint16_t)abs((int32_t)ee.sens2Upper - (int32_t)ee.sens2Lower);
}

/* Drive the inhibit line low with a single register write (safe from any context). */
static void InhibitForceLow(void)
{
  INHIBIT_GPIO_PORT->BSRR = ((uint32_t)INHIBIT_GPIO_PIN) << 16U;
}

static uint16_t AdcToSensorCounts(uint16_t adcCounts)
{
  uint32_t sensorCounts = ((uint32_t)adcCounts * APPS_INPUT_DIVIDER_DEN +
                           (APPS_INPUT_DIVIDER_NUM / 2U)) /
                          APPS_INPUT_DIVIDER_NUM;
  if (sensorCounts > ADC_MAX_COUNTS) sensorCounts = ADC_MAX_COUNTS;
  return (uint16_t)sensorCounts;
}

/* Normalize each redundant sensor against its own calibrated endpoints. Use
 * the higher reading so a low-reading channel cannot mask a high throttle. */
static uint16_t GetThrottlePermille(uint16_t sensor1, uint16_t sensor2)
{
  uint32_t p1 = GetSensorPermille(sensor1, ee.sens1Lower, sens1Range);
  uint32_t p2 = GetSensorPermille(sensor2, ee.sens2Lower, sens2Range);
  uint32_t higher = (p1 > p2) ? p1 : p2;
  return (uint16_t)((higher > 1000U) ? 1000U : higher);
}

static uint16_t GetSensorPermille(uint16_t sensor, uint32_t lower, uint16_t range)
{
  if ((range == 0U) || (sensor <= lower)) return 0U;
  uint32_t permille = ((uint32_t)(sensor - lower) * 1000U) / range;
  return (uint16_t)((permille > 1000U) ? 1000U : permille);
}

/* Read the latest samples, check both APPS channels, and update the shutdown state. */
static void UpdateAppsAndShutdown(void)
{
  /* Keep one ADC snapshot so every calculation in this pass uses the same sample. */
  checkedApps1 = apps1;
  checkedApps2 = apps2;
  uint16_t sensor1 = AdcToSensorCounts(checkedApps1);
  uint16_t sensor2 = AdcToSensorCounts(checkedApps2);
  uint8_t sensorsValid =
      (ee.sens1Lower <= sensor1) && (sensor1 <= ee.sens1Upper) &&
      (ee.sens2Lower <= sensor2) && (sensor2 <= ee.sens2Upper);

  if (!sensorsValid)
  {
    apps1Permille = 0U;
    apps2Permille = 0U;
    throttlePermille = 0U;
    appsMismatchTiming = 0U;
    appsFault = 1U;
  }
  else
  {
    apps1Permille = GetSensorPermille(sensor1, ee.sens1Lower, sens1Range);
    apps2Permille = GetSensorPermille(sensor2, ee.sens2Lower, sens2Range);
    throttlePermille = GetThrottlePermille(sensor1, sensor2);

    /* A mismatch is a difference greater than 10% pedal travel for >100 ms. */
    uint16_t difference = (apps1Permille > apps2Permille) ?
                          (apps1Permille - apps2Permille) :
                          (apps2Permille - apps1Permille);
    if (difference > (uint16_t)(SENS_ACCEPTABLE_DIFF * 1000.0f))
    {
      if (!appsMismatchTiming)
      {
        appsMismatchTiming = 1U;
        appsMismatchStartTick = HAL_GetTick();
      }
      appsFault = (HAL_GetTick() - appsMismatchStartTick) > APPS_MISMATCH_TIME_MS;
    }
    else
    {
      appsMismatchTiming = 0U;
      appsFault = 0U;
    }
  }

  UpdateThrottleBrakeLatch(sensorsValid);
}

static void UpdateThrottleBrakeLatch(uint8_t sensorsValid)
{
  uint16_t brake = brakeAdc;
  brakesPressed = (brake >= BRAKE_ENGAGED_COUNTS);
  brakePermille = (uint16_t)(((uint32_t)brake * 1000U) / ADC_MAX_COUNTS);

  /* Any APPS failure shuts down immediately. Brake plus >25% also latches. */
  if (!sensorsValid || appsFault ||
      (brakesPressed && (throttlePermille > APPS_TRIP_PERMILLE)))
  {
    throttleBrakeLatched = 1U;
  }
  /* Brake state does not matter when clearing: APPS must be below 5%. */
  else if (sensorsValid && !appsFault && (throttlePermille < APPS_CLEAR_PERMILLE))
  {
    throttleBrakeLatched = 0U;
  }

  /* PA7: low means the shutdown request is active. */
  HAL_GPIO_WritePin(INHIBIT_GPIO_PORT, INHIBIT_GPIO_PIN,
                    (sensorsValid && !throttleBrakeLatched) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void SendStatusLine(const char* prefix)
{
  char msg[768];
  uint16_t adc1 = checkedApps1;
  uint16_t adc2 = checkedApps2;
  uint16_t sensor1 = AdcToSensorCounts(adc1);
  uint16_t sensor2 = AdcToSensorCounts(adc2);
  int32_t measuredDiffCounts = (int32_t)sensor1 - (int32_t)sensor2;
  int32_t measuredDiffMv = (measuredDiffCounts * 3300L) / ADC_MAX_COUNTS;

  snprintf(msg, sizeof(msg),
           "%s up %lus | APPS: %s | Brake: %s (%u.%u%%) | APPS1: %u.%u%% | APPS2: %u.%u%% | Throttle: %u.%u%% | Inhibit: %s\r\n"
           "ADC pins: PC4/APPS1=%u counts (%lumV), PC5/APPS2=%u counts (%lumV), PA3/Brake=%u counts\r\n"
           "APPS sensor-side (divider compensated): APPS1=%lumV [LOW %lumV, HIGH %lumV]; APPS2=%lumV [LOW %lumV, HIGH %lumV]\r\n"
           "APPS_DIFF measured (APPS1-APPS2): %+ldmV\r\n\r\n",
           prefix, HAL_GetTick() / 1000, appsFault ? "FAULT" : "OK",
           brakesPressed ? "PRESSED" : "released", brakePermille / 10U, brakePermille % 10U,
           apps1Permille / 10U, apps1Permille % 10U, apps2Permille / 10U, apps2Permille % 10U,
           throttlePermille / 10U, throttlePermille % 10U,
           (HAL_GPIO_ReadPin(INHIBIT_GPIO_PORT, INHIBIT_GPIO_PIN) == GPIO_PIN_RESET) ? "LOW" : "HIGH",
           adc1, COUNTS_TO_MV(adc1), adc2, COUNTS_TO_MV(adc2), brakeAdc,
           COUNTS_TO_MV(sensor1), COUNTS_TO_MV(ee.sens1Lower), COUNTS_TO_MV(ee.sens1Upper),
           COUNTS_TO_MV(sensor2), COUNTS_TO_MV(ee.sens2Lower), COUNTS_TO_MV(ee.sens2Upper),
           (long)measuredDiffMv);
  ConsoleSendString(msg);
}

static void ConsoleSendString(const char* str)
{
  if (UsbPortOpen)
  {
    (void)CDC_SendString(str);
  }
}

static void HandleConsoleInput(void)
{
  if (DataReceivedFlag)
  {
    HandleConsoleCommand((const char*)UserRxBuffer);
    DataReceivedFlag = 0;
  }
}

static void HandleConsoleCommand(const char* cmd)
{
  char msg[128];

  if (strcmp(cmd, "HELP") == 0)
  {
    ConsoleSendString("Commands:\r\n"
                   "  PING            Reply PONG\r\n"
                   "  READ            Raw ADC counts for APPS1/APPS2\r\n"
                   "  STATUS          APPS, brake, throttle-latch, and limits\r\n"
                   "  GETSAVED        Show saved calibration\r\n"
                   "  SAVE LOWER      Save current pedal position as lower limit\r\n"
                   "  SAVE UPPER      Save current pedal position as upper limit\r\n"
                   "  HEARTBEAT ON    Print STATUS every second (default off)\r\n"
                   "  HEARTBEAT OFF   Stop the periodic STATUS line\r\n");
  }
  else if (strcmp(cmd, "STATUS") == 0)
  {
    SendStatusLine("STATUS");
  }
  else if (strcmp(cmd, "HEARTBEAT ON") == 0 || strcmp(cmd, "HEARTBEAT OFF") == 0)
  {
    heartbeatEnabled = (strcmp(cmd, "HEARTBEAT ON") == 0);
    ConsoleSendString(heartbeatEnabled ? "Heartbeat on.\r\n" : "Heartbeat off.\r\n");
  }
  else if (strcmp(cmd, "PING") == 0)
  {
    ConsoleSendString("PONG\r\n");
  }
  else if (strcmp(cmd, "READ") == 0)
  {
    // Read current ADC value and display it
    snprintf(msg, sizeof(msg), "APPS1=%u, APPS2=%u, Brake=%u\r\n", apps1, apps2, brakeAdc);
    ConsoleSendString(msg);
  }
  else if (strcmp(cmd, "SAVE UPPER") == 0 || strcmp(cmd, "SAVE LOWER") == 0)
  {
    // Store sensor-side counts after compensating for the APPS input divider.
    // Note: erasing the 128K sector stalls the CPU for ~1-2s, so the inhibit is forced low first.
    uint8_t upper = (strcmp(cmd, "SAVE UPPER") == 0);
    Storage_t previous = ee;
    if (upper)
    {
      ee.sens1Upper = AdcToSensorCounts(apps1);
      ee.sens2Upper = AdcToSensorCounts(apps2);
    }
    else
    {
      ee.sens1Lower = AdcToSensorCounts(apps1);
      ee.sens2Lower = AdcToSensorCounts(apps2);
    }

    if (!CalibrationIsValid())
    {
      ee = previous;
      ConsoleSendString("ERROR: lower must be below upper, calibration not saved.\r\n");
    }
    else if (InhibitForceLow(), !ee_write())
    {
      ee = previous; /* keep RAM calibration identical to what is in flash */
      ConsoleSendString("ERROR: flash write failed.\r\n");
    }
    else
    {
      UpdateSensorRanges();
      snprintf(msg, sizeof(msg), "SAVED %s1=%lu, %s2=%lu\r\n",
               upper ? "Upper" : "Lower", upper ? ee.sens1Upper : ee.sens1Lower,
               upper ? "Upper" : "Lower", upper ? ee.sens2Upper : ee.sens2Lower);
      ConsoleSendString(msg);
    }
  }
  else if (strcmp(cmd, "GETSAVED") == 0)
  {
    snprintf(msg, sizeof(msg), "Saved Upper1=%lu, Saved Upper2=%lu, Saved Lower1=%lu, Saved Lower2=%lu\r\n",
             ee.sens1Upper, ee.sens2Upper, ee.sens1Lower, ee.sens2Lower);
    ConsoleSendString(msg);
  }
  else
  {
    ConsoleSendString("Unknown command. Type HELP.\r\n");
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
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  // Inhibit (PA7) is already low from MX_GPIO_Init: stay shut down until calibration is loaded
  // Pump (PB3/TIM2_CH2) and fan (PB6/TIM4_CH1) PWM, 20 kHz / 50%
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 3);
  HAL_Delay(1000);

  // Load calibration from flash. Only fall back to (and write) the defaults when
  // flash is blank/invalid, so a USB calibration isn't overwritten on every boot.
  if (!ee_init(&ee, sizeof(Storage_t)))
  {
    Error_Handler();
  }
  ee_read();
  if (!CalibrationIsValid())
  {
    LoadDefaultCalibration();
    InhibitForceLow();
    ee_write();
  }
  UpdateSensorRanges();

  /*IMPORTANT: Should probably move off of Emulated EEPROM to save memory life. I was reasearching
   * and it seemed as if there was a way to write to flash as long as it was empty. Just have a bunch of
   * indexed entries in the sector and only erase it and rewrite once it is full. Use the highest indexed value.
   * Currently the EEPROM has to erase every time we write I believe even though we only use a small amount of the sector most likely */
  uint32_t lastHeartbeat = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    UpdateAppsAndShutdown();

    // Greet USB CDC terminals when they open, even if the firmware booted first.
    if (UsbPortJustOpened)
    {
      UsbPortJustOpened = 0U;
      (void)CDC_SendString("\r\n=== AER APPS-BSPD firmware, built " __DATE__ " " __TIME__ " ===\r\n"
                           "USB serial OK. Type HELP for commands.\r\n");
    }

    if (heartbeatEnabled && (HAL_GetTick() - lastHeartbeat >= HEARTBEAT_PERIOD_MS))
    {
      lastHeartbeat = HAL_GetTick();
      SendStatusLine(UsbPortOpen ? "[USB OK]" : "[CONSOLE]");
    }

    HandleConsoleInput();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 750;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1499;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 750;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        apps1 = adc_buf[0];   // PC4
        apps2 = adc_buf[1];   // PC5
        brakeAdc = adc_buf[2]; // PA3
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  InhibitForceLow();
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
