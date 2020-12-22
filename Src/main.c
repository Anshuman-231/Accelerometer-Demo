/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WHO_AM_I 0x0F
#define CTRL1_XL 0x10
#define CTRL3_C 0x12
#define CTRL9_XL 0x18
#define CTRL10_C 0x19
#define FIFO_CTRL1 0x06
#define FIFO_CTRL2 0x07
#define FIFO_CTRL3 0x08
#define FIFO_CTRL4 0x09
#define FIFO_CTRL5 0x0A
#define INT1_CTRL 0x0D
#define TAP_CFG 0x58

#define ACCEL_X_L_ADDR 0x28
#define ACCEL_X_H_ADDR 0x29
#define ACCEL_Y_L_ADDR 0x2A
#define ACCEL_Y_H_ADDR 0x2B
#define ACCEL_Z_L_ADDR 0x2C
#define ACCEL_Z_H_ADDR 0x2D
#define TIMESTAMP0_REG 0x40
#define TIMESTAMP2_REG 0x42

#define FIFO_STATUS1 0x3A
#define FIFO_STATUS2 0x3B
#define FIFO_STATUS3 0x3C
#define FIFO_STATUS4 0x3D
#define FIFO_DATA_OUT_L 0x3E
#define FIFO_DATA_OUT_H 0x3F
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t dataBuff[6], command, data;
uint16_t msb;
uint32_t temp;
double ax, ay, az, ts, accel;
float Xaccel, Yaccel, Zaccel, time[30], XLx[30], XLy[30], XLz[30];

uint8_t fifo_l, fifo_h, fifo_empty;
uint8_t f1, f2, fifo_overrun=0;
uint16_t fd, i=1, j=0, k=0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t ReadByte(uint8_t addr){
	// Read from register specified by address 'addr'
		command = 0x80 | addr;
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, &command, 1, 50);
		HAL_SPI_Receive(&hspi1, &data, 1, 50);
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

		return data;
}

void WriteByte(uint8_t addr, uint8_t data){
	// Write 8-bit 'data' to register specified by address 'addr'
	command = addr;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &command, 1, 50);
	HAL_SPI_Transmit(&hspi1, &data, 1, 50);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

void GetAccelData(void){
	// Poll accelerometer for data
	command = 0x80 | ACCEL_X_L_ADDR;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &command, 1, 50);
	HAL_SPI_Receive(&hspi1, &dataBuff[0], 6, 50);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	temp = dataBuff[1];
	ax = (temp<<8) | dataBuff[0];
	temp = dataBuff[3];
	ay = (temp<<8) | dataBuff[2];
	temp = dataBuff[5];
	az = (temp<<8) | dataBuff[4];

	msb = ax;
	msb = msb>>15;
	if(msb)
		ax = -(65536 - ax);
	msb = ay;
	msb = msb>>15;
	if(msb)
		ay = -(65536 - ay);
	msb = az;
	msb = msb>>15;
	if(msb)
		az = -(65536 - az);

  Xaccel = (ax * 0.061)/1000.0;
  Yaccel = (ay * 0.061)/1000.0;
  Zaccel = (az * 0.061)/1000.0;
}

void GetTimestamp(void){
	command = 0x80 | TIMESTAMP0_REG;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &command, 1, 50);
	HAL_SPI_Receive(&hspi1, &dataBuff[0], 3, 50);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	temp = dataBuff[2];
	temp = (temp<<8) | dataBuff[1];
	temp = (temp<<8) | dataBuff[0];
  ts = (temp * 6.4)/1000.0;
}
float fifoAccel(void){
	temp = fifo_h;
	accel = (temp<<8) | fifo_l;
	msb = accel;
	msb = msb>>15;
	if(msb)
		accel = -(65536 - accel);
	accel = (accel * 0.061)/1000.0;
	return accel;
}
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  	WriteByte(CTRL9_XL, 0x38);		//Start accelorometer
  	WriteByte(CTRL1_XL, 0x20);		//Accelerometer: 26Hz, +/-2g
  	WriteByte(CTRL10_C, 0x00);		//Disable Gyro output
  	WriteByte(TAP_CFG, 0x80);			//Enable timestamp count
  	WriteByte(CTRL3_C, 0x44);			//Block data update while reading FIFO_DATA

  	f1 = ReadByte(FIFO_STATUS1);
  	f2 = ReadByte(FIFO_STATUS2);
  	fd = (f2<<8) | f1;
  	fd = fd & 0x0FFF;
  	fifo_overrun = f2<<1;
  	fifo_overrun = fifo_overrun>>7;
  	while(fd>0 || fifo_overrun){
  		fifo_l = ReadByte(FIFO_DATA_OUT_L);
  		fifo_h = ReadByte(FIFO_DATA_OUT_H);
  		f1 = ReadByte(FIFO_STATUS1);
  		f2 = ReadByte(FIFO_STATUS2);
  		fd = (f2<<8) | f1;
  		fd = fd & 0x0FFF;
  		fifo_overrun = f2<<1;
  		fifo_overrun = fifo_overrun>>7;
  	}

  	WriteByte(TIMESTAMP2_REG, 0xAA);	//Reset timestamp to 0


  	//Disable pedometer step counter and timestamp as 4th FIFO data set & set FIFO Threshold
  	WriteByte(FIFO_CTRL1, 0xF0);
  	WriteByte(FIFO_CTRL2, 0x08);

  	//WriteByte(FIFO_CTRL2, 0x80);	//Enable pedometer step counter and timestamp as 4th FIFO data set
  	WriteByte(FIFO_CTRL3, 0x01);	//Accelerometer FIFO no decimation
  	//WriteByte(FIFO_CTRL3, 0x00);
  	//WriteByte(FIFO_CTRL4, 0x08);	//Fourth FIFO data set (timestamp & pedometer) no decimation
  	WriteByte(FIFO_CTRL5, 0x16);	//FIFO ODR 26Hz. Continuous mode
  	//WriteByte(FIFO_CTRL5, 0x11);	//FIFO ODR 26Hz. FIFO Mode
  	//WriteByte(FIFO_CTRL5, 0x13);	//FIFO ODR 26Hz. Continuous mode until trigger is deasserted, then FIFO mode.
  	//WriteByte(FIFO_CTRL5, 0x14);	//FIFO ODR 26Hz. Bypass mode until trigger is deasserted, then Continuous mode
  	//WriteByte(INT1_CTRL, 0x20);		//FIFO full flag interrupt enable on INT1
  	//WriteByte(INT1_CTRL, 0x30);		//FIFO full flag OR overflow interrupt enable on INT1

  	WriteByte(INT1_CTRL, 0x28);		//FIFO full/overrun/threshold interrupt enable on INT1
  /* USER CODE END 2 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
