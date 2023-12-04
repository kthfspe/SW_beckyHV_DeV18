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
#include "../CAN/lib/can1.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */
int period = 100; /* period given in ms */
int timeout_time = 50; /* time given in ms */

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
volatile uint16_t ADC_read_value_raw[5]; // Range: [0,4096]
float ADC_reference_voltage = 3.3;

volatile int ADC_interrupt_flag;  // set by ADC callback
volatile int timeout;
double current_I[5], currMeas_V[5], senseMeas_V[5]; // measurement variables

/* ======= CAN =========== */
CAN_TxHeaderTypeDef pTxHeader;
uint8_t TxData[8]; // transmitted data
uint32_t TxMailbox; // mailbox to store received message

struct can1_ivt_data_t can1_ivt_data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
void current_measure(void); // Process ADC values for current measurement

// can functions taken from LVBMS - should be adjusted to match IVT message
void can_lv_bms_data_a_send_status(void);
void can_lv_bms_data_a_store(void);
void can_lv_bms_status_a_send_status(void);
void can_lv_bms_status_a_store(void);
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
  /* USER CODE BEGIN 2 */
	HAL_CAN_Start(&hcan);
	HAL_CAN_WakeUp(&hcan);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		HAL_GPIO_TogglePin(GPIOA, LED_LD2_Pin);

		/* Initialise flags on loop */
		ADC_interrupt_flag = 0;
		timeout = 1;

		/* Take ADC values using the DMA */
		HAL_ADC_Start_DMA(&hadc, (uint16_t *)ADC_read_value_raw, (uint16_t)5);
		for (int i = 0; i < ((timeout_time / 10) - 1); i++) {
			if (ADC_interrupt_flag == 1) {

				timeout = 0;

				//Check current reading
				current_measure();
				break;
			}
			HAL_Delay(50);
		}

		//Send data and status on CAN
	//	can_lv_bms_data_a_store();
	//	can_lv_bms_data_a_send_status();
	//	can_lv_bms_status_a_store();
	//	can_lv_bms_status_a_send_status();

		//If we haven't received values from ADC trough DMA -> timeout error
		if (timeout) {
			error = ADC_timeout;
		}

		//Check if we have an error, if yes go to error handler
		if (error) {
			Error_Handler();

			HAL_Delay(period);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_LD2_Pin */
  GPIO_InitStruct.Pin = LED_LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void current_measure(void) {
	for (int i=0;i < 5; i++)
	{
		// Calculate the voltage on the ADC input
		currMeas_V[i] = (ADC_reference_voltage * ADC_read_value_raw[i]) / 4096.0; // Range: [0, 3.3V]
		// Calculate the sensor output
		senseMeas_V[i] = currMeas_V[i] * (27 + 10) / 27.0;
		// Calculate the current through the sensor
		current_I[i] = 800 * (senseMeas_V[i] - 9 / 4.0) / 9.0;
	}
}


/* CAN Functions - should be adapted for IVT */
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
	//__disable_irq();
	// Send error information on CAN
	//can_lv_bms_data_a_store();
//	can_lv_bms_data_a_send_status();
//	can_lv_bms_status_a_store();
//	can_lv_bms_status_a_send_status();
	HAL_Delay(100);
	NVIC_SystemReset();
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
