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
#define DAC_MAX_COUNTS          4095

/* The APPS input divider passes approximately 72% of the sensor voltage.
 * Calibration values are kept in sensor-side counts; normalize ADC readings
 * before checking ranges and computing the APPS differential. */
#define APPS_INPUT_DIVIDER_NUM  72U
#define APPS_INPUT_DIVIDER_DEN  100U

/* Allowed APPS1/APPS2 disagreement as a fraction of pedal travel (FSAE: 10%) */
#define SENS_ACCEPTABLE_DIFF    0.10f
#define APPS_MISMATCH_TIME_MS   100U

/* How often the console heartbeat line is printed when enabled */
#define HEARTBEAT_PERIOD_MS     1000

/* Adjust this threshold to the bench brake sensor's released/pressed voltages. */
#define BRAKE_ENGAGED_COUNTS    1024U
/* PB1 is the active-low throttle/brake inhibit output (low = inhibit). */
#define INHIBIT_GPIO_PORT       GPIOB
#define INHIBIT_GPIO_PIN        GPIO_PIN_1
#define APPS_TRIP_PERMILLE      250U  /* 25.0% calibrated travel */
#define APPS_CLEAR_PERMILLE     50U   /* 5.0% calibrated travel */
#define UART_COMMAND_BUFFER_SIZE 128U

/* Converts ADC/DAC counts to millivolts at the MCU pin (before any external divider) */
#define COUNTS_TO_MV(c)         ((uint32_t)(c) * 3300U / 4095U)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint16_t adc_buf[3];   // [0] = PC4/APPS1, [1] = PC5/APPS2, [2] = PA1/brake
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

//Last values written to the DACs, for STATUS/heartbeat output
uint32_t dac1Out = 0;
uint32_t dac2Out = 0;
uint8_t appsFault = 1;
uint8_t brakesPressed = 0;
uint8_t throttleBrakeLatched = 1;
uint16_t throttlePermille = 0;
uint16_t apps1Permille = 0;
uint16_t apps2Permille = 0;
uint16_t brakePermille = 0;
static uint8_t appsMismatchTiming = 0U;
static uint32_t appsMismatchStartTick = 0U;
static uint8_t uartRxByte;
static char uartCommandBuffer[UART_COMMAND_BUFFER_SIZE];
static uint16_t uartCommandLength = 0U;
static volatile uint8_t uartCommandReady = 0U;

uint8_t heartbeatEnabled = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void LoadDefaultCalibration(void);
static uint8_t CalibrationIsValid(void);
static void UpdateSensorRanges(void);
static uint32_t ClampDac(int32_t value);
static uint16_t AdcToSensorCounts(uint16_t adcCounts);
static uint16_t GetThrottlePermille(uint16_t sensor1, uint16_t sensor2);
static uint16_t GetSensorPermille(uint16_t sensor, uint32_t lower, uint16_t range);
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

static uint32_t ClampDac(int32_t value)
{
  if (value < 0) return 0;
  if (value > DAC_MAX_COUNTS) return DAC_MAX_COUNTS;
  return (uint32_t)value;
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

static void UpdateThrottleBrakeLatch(uint8_t sensorsValid)
{
  uint16_t brake = brakeAdc;
  brakesPressed = (brake >= BRAKE_ENGAGED_COUNTS);
  brakePermille = (uint16_t)(((uint32_t)brake * 1000U) / ADC_MAX_COUNTS);

  if (!sensorsValid || appsFault || (brakesPressed && (throttlePermille >= APPS_TRIP_PERMILLE)))
  {
    throttleBrakeLatched = 1U;
  }
  else if (sensorsValid && (throttlePermille <= APPS_CLEAR_PERMILLE))
  {
    throttleBrakeLatched = 0U;
  }

  /* A bad APPS reading also forces the bench inhibit low. */
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
  int32_t calibratedOffset = abs((int32_t)ee.sens1Lower - (int32_t)ee.sens2Lower);
  int32_t tolerance = (int32_t)(sens1Range * SENS_ACCEPTABLE_DIFF);
  uint32_t plusRef = ClampDac(calibratedOffset + tolerance);
  uint32_t minusRef = ClampDac(calibratedOffset - tolerance);

  snprintf(msg, sizeof(msg),
           "%s up %lus | APPS: %s | Brake: %s (%u.%u%%) | APPS1: %u.%u%% | APPS2: %u.%u%% | Throttle: %u.%u%% | Inhibit: %s\r\n"
           "ADC pins: PC4/APPS1=%u counts (%lumV), PC5/APPS2=%u counts (%lumV), PA1/Brake=%u counts\r\n"
           "APPS sensor-side (divider compensated): APPS1=%lumV [LOW %lumV, HIGH %lumV]; APPS2=%lumV [LOW %lumV, HIGH %lumV]\r\n"
           "APPS_DIFF measured (APPS1-APPS2): %+ldmV | Calibrated +/-10%% refs: PA4 +10%%=%lumV, PA5 -10%%=%lumV\r\n"
           "DAC commands (calculated, not pin feedback): PA4/DAC1=%lu counts (%lumV); PA5/DAC2=%lu counts (%lumV)\r\n\r\n",
           prefix, HAL_GetTick() / 1000, appsFault ? "FAULT" : "OK",
           brakesPressed ? "PRESSED" : "released", brakePermille / 10U, brakePermille % 10U,
           apps1Permille / 10U, apps1Permille % 10U, apps2Permille / 10U, apps2Permille % 10U,
           throttlePermille / 10U, throttlePermille % 10U,
           (HAL_GPIO_ReadPin(INHIBIT_GPIO_PORT, INHIBIT_GPIO_PIN) == GPIO_PIN_RESET) ? "LOW" : "HIGH",
           adc1, COUNTS_TO_MV(adc1), adc2, COUNTS_TO_MV(adc2), brakeAdc,
           COUNTS_TO_MV(sensor1), COUNTS_TO_MV(ee.sens1Lower), COUNTS_TO_MV(ee.sens1Upper),
           COUNTS_TO_MV(sensor2), COUNTS_TO_MV(ee.sens2Lower), COUNTS_TO_MV(ee.sens2Upper),
           (long)measuredDiffMv, COUNTS_TO_MV(plusRef), COUNTS_TO_MV(minusRef),
           dac1Out, COUNTS_TO_MV(dac1Out), dac2Out, COUNTS_TO_MV(dac2Out));
  ConsoleSendString(msg);
}

static void ConsoleSendString(const char* str)
{
  size_t length = strlen(str);
  if (length > 0U)
  {
    (void)HAL_UART_Transmit(&huart2, (uint8_t*)str, (uint16_t)length, 100U);
  }
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
  if (uartCommandReady)
  {
    HandleConsoleCommand(uartCommandBuffer);
    __disable_irq();
    uartCommandLength = 0U;
    uartCommandReady = 0U;
    __enable_irq();
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
                   "  STATUS          APPS, brake, throttle-latch, limits, and DAC status\r\n"
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
    // Note: erasing the 128K sector stalls the CPU for ~1-2s, the DACs hold their last value meanwhile.
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
    else if (!ee_write())
    {
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
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  /* USER CODE BEGIN 2 */
  // Start in the "shut down" state until calibration is loaded
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 3);
  (void)HAL_UART_Receive_IT(&huart2, &uartRxByte, 1U);
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
    // Take one snapshot so both checks and the USB READ use the same sample
    uint16_t rawS1 = apps1;
    uint16_t rawS2 = apps2;
    checkedApps1 = rawS1;
    checkedApps2 = rawS2;
    uint16_t s1 = AdcToSensorCounts(rawS1);
    uint16_t s2 = AdcToSensorCounts(rawS2);

    //Math section
    /*Make sure that the sensors going into the mcu are correct values, shutdown otherwise
     *
     * Also for the DAC math I need to account for the voltage divider on the adc and reverse it for the DAC output. It shouldn't be
     * that high so reversing it should be fine*/
    uint8_t sensorsValid = ((ee.sens1Lower <= s1) && (s1 <= ee.sens1Upper) &&
                            (ee.sens2Lower <= s2) && (s2 <= ee.sens2Upper));
    if (sensorsValid)
    {
      apps1Permille = GetSensorPermille(s1, ee.sens1Lower, sens1Range);
      apps2Permille = GetSensorPermille(s2, ee.sens2Lower, sens2Range);
      throttlePermille = GetThrottlePermille(s1, s2);
      uint16_t mismatch = (apps1Permille > apps2Permille) ?
                          (apps1Permille - apps2Permille) : (apps2Permille - apps1Permille);
      if (mismatch > (uint16_t)(SENS_ACCEPTABLE_DIFF * 1000.0f))
      {
        if (!appsMismatchTiming)
        {
          appsMismatchTiming = 1U;
          appsMismatchStartTick = HAL_GetTick();
        }
        appsFault = ((HAL_GetTick() - appsMismatchStartTick) > APPS_MISMATCH_TIME_MS);
      }
      else
      {
        appsMismatchTiming = 0U;
        appsFault = 0U;
      }
      int32_t offset = abs((int32_t)ee.sens1Lower - (int32_t)ee.sens2Lower);
      int32_t tolerance = (int32_t)(sens1Range * SENS_ACCEPTABLE_DIFF);
      dac1Out = ClampDac(offset + tolerance);
      dac2Out = ClampDac(offset - tolerance);
    }
    else
    {
      throttlePermille = 0U;
      apps1Permille = 0U;
      apps2Permille = 0U;
      appsMismatchTiming = 0U;
      // Out of range: drive both outputs to 0 V (DAC counts 0-4095 map to 0-3.3 V)
      dac1Out = 0;
      dac2Out = 0;
      appsFault = 1;
    }
    UpdateThrottleBrakeLatch(sensorsValid);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac1Out); //PA4
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac2Out); //PA5

    // USB output is command-driven by default; no automatic banner or heartbeat.
    if (heartbeatEnabled && (HAL_GetTick() - lastHeartbeat >= HEARTBEAT_PERIOD_MS))
    {
      lastHeartbeat = HAL_GetTick();
      SendStatusLine("[CONSOLE]");
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 initialization for the Nucleo ST-LINK virtual COM port.
  */
static void MX_USART2_UART_Init(void)
{
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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 3;
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
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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

  /* Set the low-true inhibit before configuring PB1 as an output. */
  HAL_GPIO_WritePin(INHIBIT_GPIO_PORT, INHIBIT_GPIO_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = INHIBIT_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(INHIBIT_GPIO_PORT, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
        brakeAdc = adc_buf[2]; // PA1
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
  if (huart->Instance == USART2)
  {
    uint8_t c = uartRxByte;
    if (!uartCommandReady)
    {
      if (c == '\n')
      {
        if (uartCommandLength > 0U)
        {
          uartCommandBuffer[uartCommandLength] = '\0';
          uartCommandReady = 1U;
        }
      }
      else if (c != '\r')
      {
        if (uartCommandLength < UART_COMMAND_BUFFER_SIZE - 1U)
        {
          uartCommandBuffer[uartCommandLength++] = (char)c;
        }
        else
        {
          uartCommandLength = 0U; /* Drop an overlong line. */
        }
      }
    }
    (void)HAL_UART_Receive_IT(&huart2, &uartRxByte, 1U);
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
