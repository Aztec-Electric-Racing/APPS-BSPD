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

/* Allowed APPS1/APPS2 disagreement as a fraction of pedal travel (FSAE: 10%) */
#define SENS_ACCEPTABLE_DIFF    0.10f

/* How often the "USB OK" heartbeat line is printed while a terminal is open */
#define HEARTBEAT_PERIOD_MS     1000

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

/* USER CODE BEGIN PV */
volatile uint16_t adc_buf[2];   // [0] = PC4, [1] = PC5 (based on rank order)
volatile uint16_t apps1 = 0;
volatile uint16_t apps2 = 0;

//Emulated EEPROM
Storage_t ee;

//Derived from ee, recalculate with UpdateSensorRanges() whenever ee changes
uint16_t sens1Range = 0;
uint16_t sens2Range = 0;

//Last values written to the DACs, for STATUS/heartbeat output
uint32_t dac1Out = 0;
uint32_t dac2Out = 0;
uint8_t appsFault = 1;

uint8_t heartbeatEnabled = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
/* USER CODE BEGIN PFP */
static void LoadDefaultCalibration(void);
static uint8_t CalibrationIsValid(void);
static void UpdateSensorRanges(void);
static uint32_t ClampDac(int32_t value);
static void HandleUsbCommand(const char* cmd);
static void SendStatusLine(const char* prefix);
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

static void SendStatusLine(const char* prefix)
{
  char msg[160];
  uint16_t s1 = apps1;
  uint16_t s2 = apps2;
  snprintf(msg, sizeof(msg),
           "%s up %lus | APPS1=%u (%lumV) APPS2=%u (%lumV) | %s | DAC1=%lu (%lumV) DAC2=%lu (%lumV)\r\n",
           prefix, HAL_GetTick() / 1000,
           s1, COUNTS_TO_MV(s1), s2, COUNTS_TO_MV(s2),
           appsFault ? "FAULT" : "OK",
           dac1Out, COUNTS_TO_MV(dac1Out), dac2Out, COUNTS_TO_MV(dac2Out));
  CDC_SendString(msg);
}

static void SendBanner(void)
{
  CDC_SendString("\r\n=== AER APPS-BSPD firmware, built " __DATE__ " " __TIME__ " ===\r\n"
                 "USB serial OK. Type HELP for commands.\r\n");
}

static void HandleUsbCommand(const char* cmd)
{
  char msg[128];

  if (strcmp(cmd, "HELP") == 0)
  {
    CDC_SendString("Commands:\r\n"
                   "  PING            Reply PONG\r\n"
                   "  READ            Raw ADC counts for APPS1/APPS2\r\n"
                   "  STATUS          ADC + DAC values in counts and mV, OK/FAULT\r\n"
                   "  GETSAVED        Show saved calibration\r\n"
                   "  SAVE LOWER      Save current pedal position as lower limit\r\n"
                   "  SAVE UPPER      Save current pedal position as upper limit\r\n"
                   "  HEARTBEAT ON    Print STATUS every second (default)\r\n"
                   "  HEARTBEAT OFF   Stop the periodic STATUS line\r\n");
  }
  else if (strcmp(cmd, "STATUS") == 0)
  {
    SendStatusLine("STATUS");
  }
  else if (strcmp(cmd, "HEARTBEAT ON") == 0 || strcmp(cmd, "HEARTBEAT OFF") == 0)
  {
    heartbeatEnabled = (strcmp(cmd, "HEARTBEAT ON") == 0);
    CDC_SendString(heartbeatEnabled ? "Heartbeat on.\r\n" : "Heartbeat off.\r\n");
  }
  else if (strcmp(cmd, "PING") == 0)
  {
    CDC_SendString("PONG\r\n");
  }
  else if (strcmp(cmd, "READ") == 0)
  {
    // Read current ADC value and display it
    snprintf(msg, sizeof(msg), "APPS1=%u, APPS2=%u\r\n", apps1, apps2);
    CDC_SendString(msg);
  }
  else if (strcmp(cmd, "SAVE UPPER") == 0 || strcmp(cmd, "SAVE LOWER") == 0)
  {
    // Store raw ADC counts, the same units the main loop compares against.
    // Note: erasing the 128K sector stalls the CPU for ~1-2s, the DACs hold their last value meanwhile.
    uint8_t upper = (strcmp(cmd, "SAVE UPPER") == 0);
    Storage_t previous = ee;
    if (upper)
    {
      ee.sens1Upper = apps1;
      ee.sens2Upper = apps2;
    }
    else
    {
      ee.sens1Lower = apps1;
      ee.sens2Lower = apps2;
    }

    if (!CalibrationIsValid())
    {
      ee = previous;
      CDC_SendString("ERROR: lower must be below upper, calibration not saved.\r\n");
    }
    else if (!ee_write())
    {
      CDC_SendString("ERROR: flash write failed.\r\n");
    }
    else
    {
      UpdateSensorRanges();
      snprintf(msg, sizeof(msg), "SAVED %s1=%lu, %s2=%lu\r\n",
               upper ? "Upper" : "Lower", upper ? ee.sens1Upper : ee.sens1Lower,
               upper ? "Upper" : "Lower", upper ? ee.sens2Upper : ee.sens2Lower);
      CDC_SendString(msg);
    }
  }
  else if (strcmp(cmd, "GETSAVED") == 0)
  {
    snprintf(msg, sizeof(msg), "Saved Upper1=%lu, Saved Upper2=%lu, Saved Lower1=%lu, Saved Lower2=%lu\r\n",
             ee.sens1Upper, ee.sens2Upper, ee.sens1Lower, ee.sens2Lower);
    CDC_SendString(msg);
  }
  else
  {
    CDC_SendString("Unknown command. Type HELP.\r\n");
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
  MX_DAC_Init();
  /* USER CODE BEGIN 2 */
  // Start in the "shut down" state until calibration is loaded
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 2);
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
    uint16_t s1 = apps1;
    uint16_t s2 = apps2;

    //Math section
    /*Make sure that the sensors going into the mcu are correct values, shutdown otherwise
     *
     * Also for the DAC math I need to account for the voltage divider on the adc and reverse it for the DAC output. It shouldn't be
     * that high so reversing it should be fine*/
    if ((ee.sens1Lower <= s1) && (s1 <= ee.sens1Upper) && (ee.sens2Lower <= s2) && (s2 <= ee.sens2Upper))
    {
      int32_t offset = abs((int32_t)ee.sens1Lower - (int32_t)ee.sens2Lower);
      int32_t tolerance = (int32_t)(sens1Range * SENS_ACCEPTABLE_DIFF);
      dac1Out = ClampDac(offset + tolerance);
      dac2Out = ClampDac(offset - tolerance);
      appsFault = 0;
    }
    else
    {
      // Out of range: drive both outputs to 0 V (DAC counts 0-4095 map to 0-3.3 V)
      dac1Out = 0;
      dac2Out = 0;
      appsFault = 1;
    }
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac1Out);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac2Out);

    // Greet the terminal when it opens the port, then print a heartbeat so it's obvious USB is alive.
    // Only while the port is open: with no terminal reading, transmits would never complete.
    if (UsbPortJustOpened)
    {
      UsbPortJustOpened = 0;
      SendBanner();
    }
    if (UsbPortOpen && heartbeatEnabled && (HAL_GetTick() - lastHeartbeat >= HEARTBEAT_PERIOD_MS))
    {
      lastHeartbeat = HAL_GetTick();
      SendStatusLine("[USB OK]");
    }

    if (DataReceivedFlag)
    {
      HandleUsbCommand((const char*)UserRxBuffer);
      DataReceivedFlag = 0;
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
  hadc1.Init.NbrOfConversion = 2;
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
