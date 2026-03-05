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
#include "ee.h"

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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	 uint16_t sens1Range = abs(ee.sens1Upper - ee.sens1Lower);
	 uint16_t sens2Range = abs(ee.sens2Upper - ee.sens2Lower);
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

  EE_Init(&ee, sizeof(Storage_t));
  EE_Read();


  /*Just setting specified values for testing I think?
  	   Am going to keep this but for safe values at least for now I
  	   will likely change it later */
  EEWriteLatch = 1;
  if(EEWriteLatch == 1) {
  			//Adjusted values
  			ee.sens1Lower = (int)((1.543/3.3) * 4095);
  			ee.sens2Lower = (int)((0.87/3.3) * 4095);
  			ee.sens1Upper = (int)((2.216/3.3) * 4095);
  			ee.sens2Upper = (int)((1.543/3.3) * 4095);
  			EE_Write();

  			EEWriteLatch = 0;
  		}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 apps1 = adc_buf[0];
	  /* USER CODE END WHILE */


	 	 //Math section
	 	 /*Make sure that the sensors going into the mcu are correct values, shutdown otherwise
	 	  *
	 	  * Also for the DAC math I need to account for the voltage divider on the adc and reverse it for the DAC output. It shouldn't be
	 	  * that high so reversing it should be fine*/
	  if((ee.sens1Lower <= apps1) && (apps1 <= ee.sens1Upper) && (ee.sens2Lower <= apps2) && (apps2 <= ee.sens2Upper)) {
		  /**/
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, abs(ee.sens1Lower - ee.sens2Lower) + sens1Range * sensAcceptibleDiff ); // 0–4095
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, abs(ee.sens1Lower - ee.sens2Lower) - sens1Range * sensAcceptibleDiff );
	  }
	  else {
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0); // 0–4095 idk what this number means bruh JT why didnt you document properlu
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0); //0-4095 is the range of stored values for 12 bit adc I think we are using 12 rn
	  }

	  /*IMPORTANT: Should probably move off of Emulated EEPROM to save memory life. I was reasearching
	   * and it seemed as if there was a way to write to flash as long as it was empty. Just have a bunch of
	   * indexed entries in the sector and only erase it and rewrite once it is full. Use the highest indexed value.
	   * Currently the EEPROM has to erase every time we write I believe even though we only use a small amount of the sector most likely */




	  	 if (DataReceivedFlag)
	  	 	      {
	  	 	          DataReceivedFlag = 0;

	  	 	         // Ensure null-termination for string compares
	  				 if (UserRxLength >= sizeof(UserRxBuffer))
	  					 UserRxLength = sizeof(UserRxBuffer) - 1;
	  				 UserRxBuffer[UserRxLength] = '\0';

	  				 if (strcmp(cmd, "PING") == 0)
	  				    {
	  				        CDC_SendString("PONG\r\n");
	  				    }
	  				    else if (strcmp(cmd, "READ") == 0)
	  				    {
	  				        // Read current ADC value and display it

	  				        char msg[64];
	  				        snprintf(msg, sizeof(msg), "APPS1=%u, APPS2=%u\r\n", apps1, apps2);
	  				        CDC_SendString(msg);
	  				    }
	  				    else if (strcmp(cmd, "SAVE UPPER") == 0)
	  				    {
	  				        // Read ADC, save it, and display current + saved
	  				    	//Convert adc value to volts
	  				    	ee.sens1Upper = (int)((apps1/4095)*3.3); //Should use internal reference value instead of 3.3
							ee.sens2Upper = (int)((apps2/4095)*3.3);
							EE_Write();

							//Message to transmit over USB
	  				        char msg[96];
	  				        snprintf(msg, sizeof(msg), "SAVED Uppper1=%u, Upper2=%u\r\n", ee.sens1Upper, ee.sens2Upper);
	  				        CDC_SendString(msg);
	  				    }
	  				    else if (strcmp(cmd, "SAVE LOWER") == 0)
						{
							// Read ADC, save it, and display current + saved
							//Convert adc value to volts
							ee.sens1Lower = (int)((apps1/4095)*3.3); //Should use internal reference value instead of 3.3
							ee.sens2Lower = (int)((apps2/4095)*3.3);
							EE_Write();

							//Message to transmit over USB
							char msg[96];
							snprintf(msg, sizeof(msg), "SAVED Lower1=%u, Lower2=%u\r\n", ee.sens1Lower, ee.sens2Lower);
							CDC_SendString(msg);
						}
	  				    else if (strcmp(cmd, "GETSAVED") == 0)
	  				    {

	  				            char msg[64];
	  				            snprintf(msg, sizeof(msg), "Saved Upper1=%u, Saved Upper2=%u, Saved Lower1=%u, Saved Lower2=%u \r\n", ee.sens1Upper, ee.sens2Upper, ee.sens1Lower, ee.sens2Lower);
	  				            CDC_SendString(msg);

	  				    }
	  				    else
	  				    {
	  				        CDC_SendString("Unknown command.\r\n");
	  				    }

	      /* USER CODE BEGIN 3 */
	    }

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
