/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include <stdlib.h>


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "ssd1306_conf.h"
#include "ssd1306_fonts.h"
#include "ssd1306_tests.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint8_t reportId;                                 // Report ID = 0x01 (1)
  uint8_t modifier;
  uint8_t reverse;
  uint8_t keycode[6];                              // Value = 0 to 255
} keyboardReport_t;


typedef struct
{
  uint8_t reportId;                                 // Report ID = 0x02 (2)
  uint8_t leftClick:1;
  uint8_t rightClick:1;
  uint8_t midClick:1;
  uint8_t pad:5;
  int8_t pointerX;                         // Usage 0x00010030: X, Value = -127 to 127
  int8_t pointerY;                         // Usage 0x00010031: Y, Value = -127 to 127
  int8_t wheel;      	                   // Usage 0x00010038: Wheel, Value = -127 to 127
} mouseReport_t;


typedef struct
{
	uint8_t reportId; // = 0x03
	uint16_t buttons;
	int8_t leftX;
	int8_t leftY;
	int8_t rightX;
	int8_t rightY;
}gamepadReport_t;


GPIO_TypeDef* port[16] = {GPIOB, GPIOA, GPIOA, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOA, GPIOB, GPIOA, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB};
uint16_t pin[16] = {GPIO_PIN_0, GPIO_PIN_7, GPIO_PIN_6, GPIO_PIN_1, GPIO_PIN_12, GPIO_PIN_10, GPIO_PIN_13, GPIO_PIN_2, GPIO_PIN_9, GPIO_PIN_15, GPIO_PIN_4, GPIO_PIN_14, GPIO_PIN_5, GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_6};
uint8_t keycode[16] = {0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM 15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;



/* USER CODE BEGIN PV */
keyboardReport_t keyboardReport;
mouseReport_t mouseReport;
gamepadReport_t gamepadReport;

uint8_t lcdBuffer1[100];
uint8_t lcdBuffer2[100];
uint8_t lcdBuffer3[100];

uint16_t adcValue0, adcValue1, adcValue2, adcValue3;

volatile uint8_t encoderValue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
extern USBD_HandleTypeDef hUsbDeviceFS;
void USBD_CUSTOM_HID_SendReport();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


int getValue(int channel)
{
	int value = 0;
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = channel;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_ADC_Start(&hadc1);
	HAL_Delay(1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	value = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return value;
}

void keyboardProcess(void)
{
	for(int i = 0; i < NUM; i++)
	{
	    if(HAL_GPIO_ReadPin(port[i], pin[i]) == GPIO_PIN_RESET)
	    {
		  keyboardReport.keycode[i] = keycode[i];
	    }
	    else
		  keyboardReport.keycode[i] = 0;
	}
	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, &keyboardReport, sizeof(keyboardReport));
}

void mouseProcess(void)
{
	mouseReport.pointerX = (float)(adcValue3 - 2047)/4095*230;
	mouseReport.pointerY = (float)(2047 - adcValue2)/4095*230;
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET)
		mouseReport.leftClick = 1;
	else
		mouseReport.leftClick = 0;
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET)
		mouseReport.rightClick = 1;
	else
		mouseReport.rightClick = 0;
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET)
		mouseReport.midClick = 1;
	else
		mouseReport.midClick = 0;
	mouseReport.wheel = encoderValue - 127;
	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, &mouseReport, sizeof(mouseReport));
	//HAL_Delay(8);
}
void gamepadProcess(void)
{
	  gamepadReport.leftX = (float)(adcValue3 - 2047)/4095*230;
	  gamepadReport.leftY = (float)(2047 - adcValue2)/4095*230;
	  gamepadReport.rightX = (float)(adcValue1 - 2047)/4095*230;
	  gamepadReport.rightY = (float)(2047 - adcValue0)/4095*230;
	  gamepadReport.buttons = 0;
	  for(int i = 0; i < NUM; i++)
	  {
		  if(HAL_GPIO_ReadPin(port[i], pin[i]) == GPIO_PIN_RESET)
			  gamepadReport.buttons |= (1 << i);
	  }
	  USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, &gamepadReport, sizeof(gamepadReport));
	  //HAL_Delay(8);
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
  MX_ADC1_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  ssd1306_Init();

  mouseReport.reportId = 0x03;
  keyboardReport.reportId = 0x02;
  gamepadReport.reportId = 0x01;
  keyboardReport.modifier = 0;
  keyboardReport.reverse = 0;
  mouseReport.pad = 0;
  ssd1306_Fill(Black);
  ssd1306_SetCursor(1, 15);
  ssd1306_WriteString("Test 2", Font_6x8, White);
  ssd1306_UpdateScreen();
  HAL_Delay(3000);
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  short modeState = 0;
  while (1)
  {
    /* USER CODE END WHILE */
	  adcValue0 = 2047;//getValue(ADC_CHANNEL_0);
	  adcValue1 = 2047;//getValue(ADC_CHANNEL_1);
	  adcValue2 = getValue(ADC_CHANNEL_2);
	  adcValue3 = getValue(ADC_CHANNEL_3);
	  encoderValue = __HAL_TIM_GET_COUNTER(&htim2);

	  sprintf((char*)lcdBuffer2, "%d   %d", adcValue0, adcValue1);
	  sprintf((char*)lcdBuffer3, "%d ", encoderValue);

	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0)
	  {
		  while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0);
		  modeState++;
		  if(modeState > 2)
			  modeState = 0;
	  }

	  switch(modeState)
	  {
	  	  case 0: //keyboard mode
	  	  {
	  		  sprintf((char*)lcdBuffer1, "Gamepad mode ON");
	  		  gamepadProcess();
	  		  break;
	  	  }
	  	  case 1:  //mouse mode
	  	  {
	  		  sprintf((char*)lcdBuffer1, "Mouse mode ON");
	  		  mouseProcess();
	  		  break;
	  	  }
	  	  case 2:  //gamepad
	  	  {
	  		  sprintf((char*)lcdBuffer1, "Keyboard mode ON");
	  		  keyboardProcess();
	  		  break;
	  	  }
	  }
	  ssd1306_Fill(Black);
	  ssd1306_SetCursor(1, 10);
	  ssd1306_WriteString((char*)lcdBuffer1, Font_6x8, White);
	  ssd1306_SetCursor(1, 30);
	  ssd1306_WriteString((char*)lcdBuffer2, Font_6x8, White);
	  ssd1306_SetCursor(1, 50);
	  ssd1306_WriteString((char*)lcdBuffer3, Font_6x8, White);
	  ssd1306_UpdateScreen();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 256;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA4 PA6 PA7 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB12 PB13 PB14 PB15
                           PB5 PB6 PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
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

#ifdef  USE_FULL_ASSERT
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
