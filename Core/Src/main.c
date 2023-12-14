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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h> // only for debugging purposes, remove if space is limited
#include "../CAN/lib/can1.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Define which ADCs are taking which measurements */
#define ADC_N 5 // number of adc pins active
#define ADC_curr_N 0
#define ADC_therm1_N 1
#define ADC_therm2_N 2
#define ADC_voltBat_N 3
#define ADC_voltVehicle_N 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
int period = 100; /* period given in ms */
int timeout_time = 50; /* time given in ms */
int SEND_ON_CAN = 0;

typedef enum {
	no_error,
	cell_over_voltage,
	cell_under_voltage,
	cell_over_temperature,
	cell_under_temperature,
	over_current,
	under_current,
	ADC_timeout
} error_t;

error_t error = no_error;
volatile uint16_t ADC_read_value_raw[5]; // Range: [0,4095]
volatile float ADC_reference_voltage = 3.3;

volatile int ADC_interrupt_flag;  // set by ADC callback
volatile int timeout;
volatile double current_I, vBat_V, vVehicle_V, tTherm1_C, tTherm2_C; // measurement variables
volatile uint8_t current_CAN;
HAL_StatusTypeDef ADC_status;
/* ======= CAN =========== */
CAN_TxHeaderTypeDef pTxHeader;
CAN_RxHeaderTypeDef pRxHeader;
uint8_t TxData[8]; // transmitted data
uint8_t RxData[8]; // received data
uint32_t TxMailbox; // mailbox to store transmitted message

struct can1_ivt_improved_status_t ivt_improved_status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
/* Timer 3 has a 500kHz period */
void current_measure(void); // Process ADC values for current measurement
void voltage_measure(void);
double ADC_to_Temperature(double ADC_value);

// CAN functions
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void CAN_send_status(void);
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
  MX_ADC_Init();
  MX_CAN_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan);
  HAL_CAN_WakeUp(&hcan);

  // set isolator pin to high permanently
  HAL_GPIO_WritePin(isolator_enable_GPIO_Port, isolator_enable_Pin, GPIO_PIN_SET);
  // Calibrate the ADC
  ADC_status = HAL_ADCEx_Calibration_Start(&hadc);
  if (ADC_status == HAL_ERROR)
	  Error_Handler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {


		/* Only for checking on the development board */
		/* REPLACE WITH ACTUAL LEDS for PCB */
		HAL_GPIO_TogglePin(MCU_STATUS_LED_3_GREEN_GPIO_Port, MCU_STATUS_LED_3_GREEN_Pin);

		/* Initialise flags on loop */
		ADC_interrupt_flag = 0;
		timeout = 1;

		// Take ADC values using the DMA
		HAL_ADC_Start_DMA(&hadc, (uint32_t *)ADC_read_value_raw, (uint16_t)ADC_N);
		for (int i = 0; i < ((timeout_time / 10) - 1); i++) {
			if (ADC_interrupt_flag == 1) {

				timeout = 0;

				//Check current reading, store in current_I
				current_measure();
				//Check voltage readings, store in vBat_V and vVehicle_V
				voltage_measure();
				// Take thermistor readings
				tTherm1_C = ADC_to_Temperature(ADC_therm1_N);
				tTherm2_C = ADC_to_Temperature(ADC_therm2_N);

				break;
			}
			HAL_Delay(50);
		}
		CAN_send_status(); // bypass the timer
		//Send data and status on CAN
		if (SEND_ON_CAN == 1)
		{
			HAL_TIM_Base_Stop_IT(&htim3); // stop timer once it is complete
			CAN_send_status(); //pack data and send on CAN
			SEND_ON_CAN = 0;
			HAL_TIM_Base_Start_IT(&htim3); // start timer again
		}

		//If we haven't received values from ADC trough DMA -> timeout error
		if (timeout) {
			error = ADC_timeout;
		}

		//Check if we have an error, if yes go to error handler
		if (error) {
			Error_Handler();
		}
		HAL_Delay(period);
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 1;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 49999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Ch1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_LD2_GPIO_Port, LED_LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, isolator_enable_Pin|MCU_STATUS_LED_1_RED_Pin|MCU_STATUS_LED_2_YLW_Pin|MCU_STATUS_LED_3_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_LD2_Pin */
  GPIO_InitStruct.Pin = LED_LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : isolator_enable_Pin MCU_STATUS_LED_1_RED_Pin MCU_STATUS_LED_2_YLW_Pin MCU_STATUS_LED_3_GREEN_Pin */
  GPIO_InitStruct.Pin = isolator_enable_Pin|MCU_STATUS_LED_1_RED_Pin|MCU_STATUS_LED_2_YLW_Pin|MCU_STATUS_LED_3_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	ADC_interrupt_flag = 1;
}

void current_measure(void) {
	double currMeas_V, sensMeas_V;
	// Calculate the voltage on the ADC input
	currMeas_V = (ADC_reference_voltage * ADC_read_value_raw[ADC_curr_N]) / 4095.0; // Range: [0, 3.3V]
	// Calculate the sensor output
	sensMeas_V = currMeas_V * (27 + 10) / 27.0;
	// Calculate the current through the sensor: Vsens = 2,5 + 10e-3*Iactual
	// Based on measurements: Vsens = 2,482 + 10e-3*Iactual
	// Need a lot of characterisation to make this at all accurate
	current_I = 100*sensMeas_V - 248.2 + 8.78; //Amps (8.78 is an offset to keep it at roughly 0A when it should be 0)
	// TODO: Should add some sort of averaging here, need to improve accuracy somehow of current measurement
	current_CAN = (uint8_t) current_I;
}

void voltage_measure(void)
{
	double ADCBat_V, ADCVehicle_V;
	ADCBat_V = (ADC_reference_voltage * ADC_read_value_raw[ADC_voltBat_N]) / 4095.0; // Range: [0, 3.3V]
	ADCVehicle_V = (ADC_reference_voltage * ADC_read_value_raw[ADC_voltVehicle_N]) / 4095.0; // Range: [0, 3.3V]

	/* Reverse the voltage division to get actual voltage value */
	/* division: (2Meg + 2Meg + 2Meg + 1Meg + 300k + (120k || 120k || 120k)) / (120k || 120k || 120k) */
	vBat_V = (734/4)*ADCBat_V;
	vVehicle_V = (734/4)*ADCVehicle_V;
}

/*
 The code defines a function "ADC_to_Temperature" that calculates the temperature in degrees Celsius
 based on an ADC value, using the Steinhart-Hart equation and specific parameters such as the
 Beta value, reference temperature, and nominal resistance of a thermistor. The function takes
 the ADC value as input, performs the calculation, and returns the corresponding temperature.
 */
double ADC_to_Temperature(double ADC_value) {	//TODO: calibrate
	double beta = 3500;
	double temp = 25 + 273.15;
	double R0 = 10000 * exp(-beta / temp);
	double R = 10000;
	return beta / (log(R * ADC_value ) - log(-R0 * (ADC_value - 4095))) - 273.15;
}

void CAN_send_status(void)
{
	ivt_improved_status.ivt_current = can1_ivt_improved_status_ivt_current_encode(current_I);
	ivt_improved_status.ivt_voltage_battery = can1_ivt_improved_status_ivt_voltage_battery_encode(vBat_V);
	ivt_improved_status.ivt_voltage_vehicle = can1_ivt_improved_status_ivt_voltage_vehicle_encode(vVehicle_V);
	ivt_improved_status.temp_h_vplus = can1_ivt_improved_status_temp_h_vplus_encode(tTherm1_C);
	ivt_improved_status.temp_h_vminus = can1_ivt_improved_status_temp_h_vminus_encode(tTherm2_C);

	// TODO : fix mapping of voltage signals so that they can be signed
	pTxHeader.DLC = CAN1_IVT_IMPROVED_STATUS_LENGTH;
	pTxHeader.IDE = CAN_ID_STD;
	pTxHeader.StdId = CAN1_IVT_IMPROVED_STATUS_FRAME_ID;
	pTxHeader.RTR = CAN_RTR_DATA;
	pTxHeader.TransmitGlobalTime = DISABLE;

	can1_ivt_improved_status_pack(TxData, &ivt_improved_status, CAN1_IVT_IMPROVED_STATUS_LENGTH);
	if (HAL_CAN_AddTxMessage(&hcan, &pTxHeader, TxData, &TxMailbox) != HAL_OK) {
		Error_Handler();

	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and send on CAN if it's tim3.
  if (htim == &htim3 )
  {
    SEND_ON_CAN = 1;

  }
}

/* CAN Functions - should be adapted for IVT
void can_lv_bms_data_a_store(void) {
	lv_bms_data_a.current = current_I;
}

void can_lv_bms_data_a_send_status(void){
	pTxHeader.DLC = CAN1_LV_BMS_DATA_A_LENGTH;
	pTxHeader.IDE = CAN_ID_STD;
	pTxHeader.StdId = CAN1_LV_BMS_DATA_A_FRAME_ID;
	pTxHeader.RTR = CAN_RTR_DATA;
	pTxHeader.TransmitGlobalTime = DISABLE;

	//Need to pack data in different messages due to lack of space per message. See can.c file.
	for (uint8_t multiplexor = 0; multiplexor <= 3; multiplexor++) {
	        lv_bms_data_a.data_multiplexor = multiplexor; // Update the multiplexor
	        can1_lv_bms_data_a_pack(TxData, &lv_bms_data_a, CAN1_LV_BMS_DATA_A_LENGTH); // Call the packing function
	        //Send the message
	        if (HAL_CAN_AddTxMessage(&hcan, &pTxHeader, TxData, &TxMailbox) != HAL_OK) {
	        		Error_Handler();
	        }
	}
}

void can_lv_bms_status_a_store(void) {
	lv_bms_status_a.current = current_I;
	lv_bms_status_a.lv_bms_error = error;
}

void can_lv_bms_status_a_send_status(void){
	pTxHeader.DLC = CAN1_LV_BMS_STATUS_A_LENGTH;
	pTxHeader.IDE = CAN_ID_STD;
	pTxHeader.StdId = CAN1_LV_BMS_STATUS_A_FRAME_ID;
	pTxHeader.RTR = CAN_RTR_DATA;
	pTxHeader.TransmitGlobalTime = DISABLE;

	can1_lv_bms_status_a_pack(TxData, &lv_bms_status_a, CAN1_LV_BMS_STATUS_A_LENGTH);

	if (HAL_CAN_AddTxMessage(&hcan, &pTxHeader, TxData, &TxMailbox) != HAL_OK) {
		Error_Handler();

	}
} */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	//__disable_irq();
	// Send error information on CAN
	while (1) {
		HAL_GPIO_WritePin(MCU_STATUS_LED_2_YLW_GPIO_Port,
				MCU_STATUS_LED_2_YLW_Pin, 1);
		if (ADC_status == HAL_ERROR)
			printf("ADC Error");
		HAL_Delay(100);
	}
	//NVIC_SystemReset();
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
