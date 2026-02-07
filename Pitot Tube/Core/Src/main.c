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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEVICE_ADDRESS 0x6C
#define DATA_ADDRESS 0x2E //Reads corrected IC Temp, corrected Pressure signal
						  //and synchronized status bits of sensor
						  //status bits: bit 3 is dsp_s_up, bit 4 is dsp_t_up
						  //pressure, temperature indicator
#define COMMAND_REGISTER 0x22
#define AIR_DENSITY 1.2250
#define CAN_ID 0x260 //Check this
#define MIN_PRESSURE -0.29
#define MAX_PRESSURE 0.29
#define MIN_PRESSURE_COUNTS -26215
#define MAX_PRESSURE_COUNTS 26214

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

/* USER CODE BEGIN PV */
float pressure_diff;
float wind_vel = 0;


//CAN Variables
CAN_TxHeaderTypeDef Tx_Header;
uint32_t Tx_Mailbox;
uint8_t data_buff[4] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
float process_pressure_data(int raw_pressure);
float read_sensor_data(void);
void CAN_header_init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* Read the data collected by SM7391 differential pressure
 * sensor and return it as a float in pascals
 */
float read_sensor_data(void){
	uint8_t buff[6] = {0}; //three 16-bit "words"
	//int raw_temperature_reading = 0; //should this be int? //Don't think I actually need this
	int16_t raw_pressure_reading = 0; //should this be int?
	uint8_t dsp_s_up = 0;
	//uint8_t dsp_t_up = 0;

	float pressure_reading;

	//Device 7-bit address has to be shifted left 1 bit
	if(HAL_I2C_Mem_Read(&hi2c1, DEVICE_ADDRESS << 1, DATA_ADDRESS, 1, buff, 6, HAL_MAX_DELAY) != HAL_OK){
		Error_Handler();
	}

	//Temperature and Pressure Registers have invalid data after power-up
	//After power-up, wait until status dsp_s_up and dsp_t_up bits have been set at least once

	//The dsp status bits are bits 3 (pressure) and 4 (temperature) of buff[4]
	dsp_s_up = buff[4] & (1 << 3);
	//dsp_t_up = buff[4] & (1 << 4);

	//Temp readings are in buff[0] and buff[1]
	//buff[1] is the hi-byte
	//pressure readings are in buff[2] and buff[3]
	//buff[3] is the hi-byte

	if(dsp_s_up){
		raw_pressure_reading = (buff[3] << 8) | buff[2];
		pressure_reading = process_pressure_data(raw_pressure_reading);
	}

	//what should happen if pressure data isn't valid?
	else{
		pressure_reading = 0;
	}

	return pressure_reading;

}


/* Process the raw pressure reading
 * and return it in pascals
 */

float process_pressure_data(int raw_pressure){
	float pressure_reading;


	//this is in psi,
	//equation from datasheet
	pressure_reading = MIN_PRESSURE + (((float)((raw_pressure - MIN_PRESSURE_COUNTS) /
									 (MAX_PRESSURE_COUNTS - MIN_PRESSURE_COUNTS))) * (MAX_PRESSURE - MIN_PRESSURE));

	//1 psi = 6894.75729 pascal
	pressure_reading = pressure_reading * 6894.75729;




	return pressure_reading; //in pascal
}

void CAN_header_init(void){
	Tx_Header.StdId = CAN_ID;
	Tx_Header.DLC = 4; //Four bytes for the wind velocity reading (32-bit)
	Tx_Header.IDE = CAN_ID_STD;
	Tx_Header.RTR = CAN_RTR_DATA;
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
  MX_CAN1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  CAN_header_init();

  //Send a reset to the IC
  uint8_t reset[2] = {0x69, 0xB1}; //0xB169: Reset
  if(HAL_I2C_Mem_Write(&hi2c1, DEVICE_ADDRESS << 1, COMMAND_REGISTER, 1, reset, 2, HAL_MAX_DELAY) != HAL_OK){
	  Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  pressure_diff = read_sensor_data(); //in pascals
	  wind_vel = sqrt( (2.0 / ((float) AIR_DENSITY)) * (pressure_diff) ); //in m/s

	  uint32_t wind_vel_mm = (uint32_t) (wind_vel * 1000); //stored in mm/s now with 32-bits

	  //store 32-bit data into 4 bytes for sending over CAN
	  data_buff[0] = (wind_vel_mm >> 24) & 255;
	  data_buff[1] = (wind_vel_mm >> 16) & 255;
	  data_buff[2] = (wind_vel_mm >> 8) & 255;
	  data_buff[3] = (wind_vel_mm) & 255;

	  while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0); //Wait until there's an open TxMailbox

	  if(HAL_CAN_AddTxMessage(&hcan1, &Tx_Header, data_buff, &Tx_Mailbox) != HAL_OK){
		  Error_Handler();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
