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
#include "cmsis_os.h"
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
  uint8_t  reportId;                                 // Report ID = 0x01 (1)
                                                     // Collection: CA:Keyboard
  uint8_t  KB_KeyboardKeyboardLeftControl : 1;       // Usage 0x000700E0: Keyboard Left Control, Value = 0 to 1
  uint8_t  KB_KeyboardKeyboardLeftShift : 1;         // Usage 0x000700E1: Keyboard Left Shift, Value = 0 to 1
  uint8_t  KB_KeyboardKeyboardLeftAlt : 1;           // Usage 0x000700E2: Keyboard Left Alt, Value = 0 to 1
  uint8_t  KB_KeyboardKeyboardLeftGui : 1;           // Usage 0x000700E3: Keyboard Left GUI, Value = 0 to 1
  uint8_t  KB_KeyboardKeyboardRightControl : 1;      // Usage 0x000700E4: Keyboard Right Control, Value = 0 to 1
  uint8_t  KB_KeyboardKeyboardRightShift : 1;        // Usage 0x000700E5: Keyboard Right Shift, Value = 0 to 1
  uint8_t  KB_KeyboardKeyboardRightAlt : 1;          // Usage 0x000700E6: Keyboard Right Alt, Value = 0 to 1
  uint8_t  KB_KeyboardKeyboardRightGui : 1;          // Usage 0x000700E7: Keyboard Right GUI, Value = 0 to 1
  uint8_t  pad_2;                                    // Pad
  uint8_t  Keyboard[6];                              // Value = 0 to 255
} keyboardReport_t;

typedef struct
{
  uint8_t  reportId;                                 // Report ID = 0x02 (2)
                                                     // Collection: CA:Mouse CP:Pointer
  uint8_t  BTN_MousePointerButton1 : 1;              // Usage 0x00090001: Button 1 Primary/trigger, Value = 0 to 1
  uint8_t  BTN_MousePointerButton2 : 1;              // Usage 0x00090002: Button 2 Secondary, Value = 0 to 1
  uint8_t  BTN_MousePointerButton3 : 1;              // Usage 0x00090003: Button 3 Tertiary, Value = 0 to 1
  uint8_t  : 5;                                      // Pad
  int8_t   GD_MousePointerX;                         // Usage 0x00010030: X, Value = -127 to 127
  int8_t   GD_MousePointerY;                         // Usage 0x00010031: Y, Value = -127 to 127
  int8_t   GD_MousePointerWheel;                     // Usage 0x00010038: Wheel, Value = -127 to 127
} mouseReport_t;

typedef struct
{
	uint8_t reportId;
	uint16_t buttons;
	int8_t left_x;
	int8_t left_y;
	int8_t right_x;
	int8_t right_y;
}gamepadReport_t;
GPIO_TypeDef* port[16] = {GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOA, GPIOA, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOA, GPIOB, GPIOA};
uint16_t pin[16] = {GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_1, GPIO_PIN_0, GPIO_PIN_7, GPIO_PIN_6, GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_10, GPIO_PIN_2, GPIO_PIN_14, GPIO_PIN_4, GPIO_PIN_15, GPIO_PIN_9};
uint8_t keycode[16] = {0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;

osThreadId collectingTaskHandle;
osThreadId mouseTaskHandle;
osThreadId keyboardTaskHandle;
osThreadId displayTaskHandle;
/* USER CODE BEGIN PV */


keyboardReport_t keyboard_report;
mouseReport_t mouse_report;
gamepadReport_t gamepad_report;

uint8_t lcd_buf_1[100];
uint8_t lcd_buf_2[100];

uint16_t adc_val_0, adc_val_1, adc_val_2, adc_val_3;

volatile uint8_t tim_count;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
void StartCollectingTask(void const * argument);
void StrartMouseTask(void const * argument);
void StartKeyboardTask(void const * argument);
void StartDisplayTask(void const * argument);

/* USER CODE BEGIN PFP */
extern USBD_HandleTypeDef hUsbDeviceFS;
void USBD_CUSTOM_HID_SendReport();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int getValueButShorter(int channel)
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
	//HAL_Delay(10);
	HAL_ADC_PollForConversion(&hadc1, 10);
	value = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return value;
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
  keyboard_report.reportId = 0x01;
  mouse_report.reportId = 0x02;
  gamepad_report.reportId = 0x03;
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of collectingTask */
  osThreadDef(collectingTask, StartCollectingTask, osPriorityNormal, 0, 128);
  collectingTaskHandle = osThreadCreate(osThread(collectingTask), NULL);

  /* definition and creation of mouseTask */
  osThreadDef(mouseTask, StrartMouseTask, osPriorityNormal, 0, 128);
  mouseTaskHandle = osThreadCreate(osThread(mouseTask), NULL);

  /* definition and creation of keyboardTask */
  osThreadDef(keyboardTask, StartKeyboardTask, osPriorityNormal, 0, 128);
  keyboardTaskHandle = osThreadCreate(osThread(keyboardTask), NULL);

  /* definition and creation of displayTask */
  osThreadDef(displayTask, StartDisplayTask, osPriorityNormal, 0, 128);
  displayTaskHandle = osThreadCreate(osThread(displayTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  htim2.Init.Period = 255;
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

  /*Configure GPIO pins : PA4 PA7 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB12 PB13 PB14 PB15
                           PB5 PB6 PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartCollectingTask */
/**
  * @brief  Function implementing the collectingTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartCollectingTask */
void StartCollectingTask(void const * argument)
{
  /* init code for USB_DEVICE */
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  while(1)
  {
	  adc_val_0 = getValueButShorter(ADC_CHANNEL_0);
	  adc_val_1 = 2047;//getValueButShorter(ADC_CHANNEL_1);
	  adc_val_2 = getValueButShorter(ADC_CHANNEL_2);
	  adc_val_3 = getValueButShorter(ADC_CHANNEL_3);
	  //Xu ly va scale gia tri
	  mouse_report.GD_MousePointerX = (adc_val_0 - 2047)/120;
	  mouse_report.GD_MousePointerY = (adc_val_0 - 2047)/120;
	  if(adc_val_0 > 4000)
	  {
		  sprintf((char*)lcd_buf_2, "presased A and B");
		  keyboard_report.Keyboard[0] = keycode[0];
		  keyboard_report.Keyboard[1] = keycode[1];
  //		  for(int i = 0; i < 16; i++)
  //		  {
  //			  keyboard_report.Keyboard[i] = keycode[i];
  //		  }
	  }
	  else
	  {
		  sprintf((char*)lcd_buf_2, "pressed none");
		  keyboard_report.Keyboard[0] = 0x00;
		  keyboard_report.Keyboard[1] = 0x00;
	  }
	  sprintf((char*)lcd_buf_1, "%d   %d", adc_val_0, adc_val_1);
	  osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartKeyboardTask */
/**
* @brief Function implementing the keyboardTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKeyboardTask */
void StartKeyboardTask(void const * argument)
{
  /* USER CODE BEGIN StartKeyboardTask */
  /* Infinite loop */
  while(1)
  {
	  USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS,&keyboard_report, sizeof(keyboard_report));
	  osDelay(5);

  }
  /* USER CODE END StartKeyboardTask */
}
/* USER CODE BEGIN Header_StrartMouseTask */
/**
* @brief Function implementing the mouseTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StrartMouseTask */
void StrartMouseTask(void const * argument)
{
  /* USER CODE BEGIN StrartMouseTask */
  /* Infinite loop */
  while(1)
  {
	  USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS,&mouse_report, sizeof(mouse_report));
	  osDelay(5);
  }
  /* USER CODE END StrartMouseTask */
}



/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the displayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void const * argument)
{
  /* USER CODE BEGIN StartDisplayTask */
  /* Infinite loop */
  while(1)
  {
  	  ssd1306_Fill(Black);
  	  ssd1306_SetCursor(1, 15);
  	  ssd1306_WriteString((char*)lcd_buf_1, Font_6x8, White);
  	  ssd1306_SetCursor(1, 40);
  	  ssd1306_WriteString((char*)lcd_buf_2, Font_6x8, White);
  	  ssd1306_SetCursor(1, 55);
  	  ssd1306_WriteString("testss", Font_6x8, White);
	  ssd1306_UpdateScreen();
	  osDelay(1);
  }
  /* USER CODE END StartDisplayTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
