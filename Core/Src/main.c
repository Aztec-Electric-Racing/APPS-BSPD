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
#include "usbd_cdc_if.h"
#include "ee.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;

/* USER CODE BEGIN PV */
extern uint8_t UserRxBuffer[512];
extern uint32_t UserRxLength;
extern volatile uint8_t DataReceivedFlag;

volatile uint16_t adc_buf[2];   // [0] = PC4, [1] = PC5 (based on rank order)
volatile uint16_t apps1 = 0;
volatile uint16_t apps2 = 0;

//Values for USB Commands
volatile uint16_t apps1_saved = 0;
volatile uint16_t apps2_saved = 0;
volatile uint8_t  saved_valid = 0;



//Emulated EEPROM
Storage_t ee;
volatile uint8_t EEWriteLatch = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void cli_send(const char *s)
{
    CDC_Transmit_HS((uint8_t *)s, strlen(s));
}

static void cli_trim(char *s)
{
    size_t len = strlen(s);

    while (len > 0 && (s[len - 1] == '\r' || s[len - 1] == '\n' || s[len - 1] == ' '))
    {
        s[len - 1] = '\0';
        len--;
    }

    while (*s == ' ')
    {
        memmove(s, s + 1, strlen(s));
    }
}

static void cli_uppercase(char *s)
{
    while (*s)
    {
        *s = toupper((unsigned char)*s);
        s++;
    }
}

static uint16_t clamp_u12(int32_t value)
{
    if (value < 0) return 0;
    if (value > 4095) return 4095;
    return (uint16_t)value;
}

static void cli_process(char *cmd)
{
    cli_trim(cmd);
    cli_uppercase(cmd);

    if (strlen(cmd) == 0)
    {
        return;
    }

    if (strcmp(cmd, "HELP") == 0 || strcmp(cmd, "?") == 0)
    {
        cli_send(
            "\r\nCommands:\r\n"
            "  HELP              Show this menu\r\n"
            "  PING              Test USB connection\r\n"
            "  READ              Read current APPS ADC values\r\n"
            "  GETSAVED          Show saved calibration values\r\n"
            "  SAVE LOWER        Save current ADCs as lower calibration\r\n"
            "  SAVE UPPER        Save current ADCs as upper calibration\r\n"
            "  SET LOWER a b     Manually set lower values\r\n"
            "  SET UPPER a b     Manually set upper values\r\n"
            "  STATUS            Show current safety/calibration state\r\n"
            "  RESETCAL          Reset calibration to safe defaults\r\n"
            "\r\n"
        );
    }
    else if (strcmp(cmd, "PING") == 0)
    {
        cli_send("PONG\r\n");
    }
    else if (strcmp(cmd, "READ") == 0)
    {
        char msg[96];
        snprintf(msg, sizeof(msg), "APPS1=%u, APPS2=%u\r\n", apps1, apps2);
        cli_send(msg);
    }
    else if (strcmp(cmd, "SAVE LOWER") == 0)
    {
        ee.sens1Lower = apps1;
        ee.sens2Lower = apps2;
        ee_write();

        char msg[128];
        snprintf(msg, sizeof(msg),
                 "Saved lower: APPS1=%lu, APPS2=%lu\r\n",
                 ee.sens1Lower, ee.sens2Lower);
        cli_send(msg);
    }
    else if (strcmp(cmd, "SAVE UPPER") == 0)
    {
        ee.sens1Upper = apps1;
        ee.sens2Upper = apps2;
        ee_write();

        char msg[128];
        snprintf(msg, sizeof(msg),
                 "Saved upper: APPS1=%lu, APPS2=%lu\r\n",
                 ee.sens1Upper, ee.sens2Upper);
        cli_send(msg);
    }
    else if (strncmp(cmd, "SET LOWER ", 10) == 0)
    {
        int s1, s2;

        if (sscanf(cmd + 10, "%d %d", &s1, &s2) == 2)
        {
            ee.sens1Lower = clamp_u12(s1);
            ee.sens2Lower = clamp_u12(s2);
            ee_write();

            cli_send("Manual lower calibration saved.\r\n");
        }
        else
        {
            cli_send("Usage: SET LOWER <apps1> <apps2>\r\n");
        }
    }
    else if (strncmp(cmd, "SET UPPER ", 10) == 0)
    {
        int s1, s2;

        if (sscanf(cmd + 10, "%d %d", &s1, &s2) == 2)
        {
            ee.sens1Upper = clamp_u12(s1);
            ee.sens2Upper = clamp_u12(s2);
            ee_write();

            cli_send("Manual upper calibration saved.\r\n");
        }
        else
        {
            cli_send("Usage: SET UPPER <apps1> <apps2>\r\n");
        }
    }
    else if (strcmp(cmd, "GETSAVED") == 0)
    {
        char msg[160];
        snprintf(msg, sizeof(msg),
                 "Lower1=%lu, Lower2=%lu, Upper1=%lu, Upper2=%lu\r\n",
                 ee.sens1Lower, ee.sens2Lower,
                 ee.sens1Upper, ee.sens2Upper);
        cli_send(msg);
    }
    else if (strcmp(cmd, "STATUS") == 0)
    {
        uint8_t valid =
            (ee.sens1Lower <= apps1) &&
            (apps1 <= ee.sens1Upper) &&
            (ee.sens2Lower <= apps2) &&
            (apps2 <= ee.sens2Upper);

        char msg[192];
        snprintf(msg, sizeof(msg),
                 "APPS1=%u, APPS2=%u, State=%s\r\n"
                 "Limits: S1[%lu,%lu], S2[%lu,%lu]\r\n",
                 apps1, apps2,
                 valid ? "VALID" : "FAULT",
                 ee.sens1Lower, ee.sens1Upper,
                 ee.sens2Lower, ee.sens2Upper);
        cli_send(msg);
    }
    else if (strcmp(cmd, "RESETCAL") == 0)
    {
        ee.sens1Lower = (uint32_t)((1.543f / 3.3f) * 4095.0f);
        ee.sens2Lower = (uint32_t)((0.870f / 3.3f) * 4095.0f);
        ee.sens1Upper = (uint32_t)((2.216f / 3.3f) * 4095.0f);
        ee.sens2Upper = (uint32_t)((1.543f / 3.3f) * 4095.0f);
        ee_write();

        cli_send("Calibration reset to default values.\r\n");
    }
    else
    {
        cli_send("Unknown command. Type HELP.\r\n");
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
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 2);
  HAL_Delay(1000);

  ee_init(&ee, sizeof(Storage_t));
  ee_read();


  uint16_t sens1Range = ee.sens1Upper - ee.sens1Lower;
  uint16_t sens2Range = ee.sens2Upper - ee.sens2Lower;

  /*Just setting specified values for testing I think?
  	   Am going to keep this but for safe values at least for now I
  	   will likely change it later */
  //EEWriteLatch = 1;
  if(EEWriteLatch == 1) {
  			//Adjusted values
	 		 ee.sens1Lower = (uint32_t)((1.543f / 3.3f) * 4095.0f);
	         ee.sens2Lower = (uint32_t)((0.870f / 3.3f) * 4095.0f);
	         ee.sens1Upper = (uint32_t)((2.216f / 3.3f) * 4095.0f);
	         ee.sens2Upper = (uint32_t)((1.543f / 3.3f) * 4095.0f);

  			EEWriteLatch = 0;
  		}

  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 apps1 = adc_buf[0];
	 apps2 = adc_buf[1];
	  /* USER CODE END WHILE */


	 	 //Math section
	 	 /*Make sure that the sensors going into the mcu are correct values, shutdown otherwise
	 	  *
	 	  * Also for the DAC math I need to account for the voltage divider on the adc and reverse it for the DAC output. It shouldn't be
	 	  * that high so reversing it should be fine*/
	  if((ee.sens1Lower <= apps1) && (apps1 <= ee.sens1Upper) && (ee.sens2Lower <= apps2) && (apps2 <= ee.sens2Upper)) {
		  /**/
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, abs(ee.sens1Lower - ee.sens2Lower) + sens1Range * 0.1 ); // 0–4095
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, abs(ee.sens1Lower - ee.sens2Lower) - sens1Range * 0.1 );
	  }
	  else {
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0); // 0–4095 idk what this number means bruh JT why didnt you document properlu
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0); //0-4095 is the range of stored values for 12 bit adc I think we are using 12 rn
	  }

	  /*IMPORTANT: Should probably move off of Emulated EEPROM to save memory life. I was reasearching
	   * and it seemed as if there was a way to write to flash as long as it was empty. Just have a bunch of
	   * indexed entries in the sector and only erase it and rewrite once it is full. Use the highest indexed value.
	   * Currently the EEPROM has to erase every time we write I believe even though we only use a small amount of the sector most likely */




	  	 if (DataReceivedFlag){
          DataReceivedFlag = 0;

          if (UserRxLength >= sizeof(UserRxBuffer))
          {
              UserRxLength = sizeof(UserRxBuffer) - 1;
          }

          UserRxBuffer[UserRxLength] = '\0';

          cli_process((char *)UserRxBuffer);
        }

	      /* USER CODE BEGIN 3 */
	    

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
