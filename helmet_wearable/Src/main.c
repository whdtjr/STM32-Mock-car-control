/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "MPU6050.h"
#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
void MPU6050_Write(uint8_t Address, uint8_t data);
void MPU6050_Write_bits(uint8_t Address, uint8_t bitStart, uint8_t length, uint8_t data);
uint8_t MPU6050_Read(uint8_t Address);
void init_MPU6050(void);
void read_MPU6050_data(void);
int _write(int32_t file, uint8_t *ptr, int32_t len);
void calc_degree();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t Ac_X0, Ac_Y0, Ac_Z0, Gy_X0, Gy_Y0, Gy_Z0;
float Ac_X1, Ac_Y1, Ac_Z1, Gy_X1, Gy_Y1, Gy_Z1;
float Ac_X2, Ac_Y2, Ac_Z2;
float Deg_X, Deg_Y, Deg_Z, Deg_XC, Deg_YC, Deg_ZC;
uint8_t MPU6050 = 0;

volatile unsigned int time3_1msCnt = 0;
volatile int time3_100msFlag = 0;
int time3_100msCnt = 0;
volatile int time3_1secFlag = 0;

extern cb_data_t cb_data;
char strBuff[MAX_ESP_COMMAND_LEN];

int Deg_XC_Cnt = 0;
char GyroBuff[100] = {0,};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	int ret = 0;
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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  init_MPU6050();

  if(HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) //timer 인터럽드 시작ㄴ
	  Error_Handler();

  ret |= drv_esp_init();
  if(ret != 0)
  {
	  printf("Esp response error\r\n");
	  Error_Handler();
  }

  AiotClient_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		calc_degree();
//		printf("X : %5.2f, Y : %5.2f\r\n", Deg_XC, Deg_YC);
	if(time3_100msFlag){
		read_MPU6050_data();
		calc_degree();
		time3_100msFlag = 0;
		time3_100msCnt++;

		if(time3_100msCnt % 10){
			time3_1secFlag = 1;
			time3_100msCnt = 0;
		}
	}
	if(time3_1secFlag){
		if(esp_get_status() != 0)
		{
			printf("server connecting ...\r\n");
			esp_client_conn();
		}

		if(Deg_XC > 30.0){
			Deg_XC_Cnt++;
		}

		time3_1secFlag = 0;
	}
	if(Deg_XC_Cnt > 3){
		sprintf(GyroBuff,"[CHI_SQL]SETDB@status@ON\n\0");
		esp_send_data(GyroBuff);
		HAL_Delay(100);
		//sprintf(GyroBuff, "[CHI_STM2]DETECT@ON\n\0");
		esp_send_data(GyroBuff);
		Deg_XC_Cnt = 0;
	}

	if(strstr((char *)cb_data.buf,"+IPD") && cb_data.buf[cb_data.length-1] == '\n')
	{
		//?��?��?���??  \r\n+IPD,15:[KSH_LIN]HELLO\n
		strcpy(strBuff,strchr((char *)cb_data.buf,'['));
		memset(cb_data.buf,0x0,sizeof(cb_data.buf));
		cb_data.length = 0;
//		esp_event(strBuff);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 38400;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

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

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void MPU6050_Write(uint8_t Address, uint8_t data){
  HAL_I2C_Mem_Write(&hi2c1, MPU6050, Address, 1, (uint8_t *)&data, 1, 10);
}

void MPU6050_Write_bits(uint8_t Address, uint8_t bitStart, uint8_t length, uint8_t data){
  uint8_t tmp = 0;
  HAL_I2C_Mem_Read(&hi2c1, MPU6050, Address, 1, (uint8_t *)&tmp, 1, 10);
  uint8_t mask = 0;
  switch(length){
    case 1: mask = 0x01; break;
    case 2: mask = 0x03; break;
    case 3: mask = 0x07; break;
    case 4: mask = 0x0F; break;
    case 5: mask = 0x1F; break;
    case 6: mask = 0x3F; break;
    case 7: mask = 0x7F; break;
    case 8: mask = 0xFF; break;
  }
  tmp &= ~(mask << bitStart);
  tmp |= (data << bitStart);
  HAL_I2C_Mem_Write(&hi2c1, MPU6050, Address, 1, (uint8_t *)&tmp, 1, 10);
}

uint8_t MPU6050_Read(uint8_t Address){
  uint8_t data;
  HAL_I2C_Mem_Read(&hi2c1, MPU6050, Address, 1, (uint8_t *)&data, 1, 10);
  return data;
}

void init_MPU6050(void){
  while(HAL_I2C_IsDeviceReady(&hi2c1, MPU6050, 10, 1000)!=HAL_OK) {
    MPU6050++;
  }
  printf("MPU6050 I2C Address is 0x%02X(7bit value)\r\n", MPU6050>>1);

  uint8_t temp = MPU6050_Read(MPU6050_RA_WHO_AM_I);
  printf("Who am I = 0x%02X\r\n", temp);
  printf("MPU6050 Initialize..... \r\n");
  printf("--------------------------------------------------------\r\n");

  HAL_Delay(100);
  /* Power Management 1, SLEEP Diasble*/
  MPU6050_Write_bits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, MPU6050_PWR1_SLEEP_LENGTH, DISABLE);
  HAL_Delay(10);
  /* Power Management 1, Internal 8MHz oscillator */
  MPU6050_Write_bits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_INTERNAL);
  /* Gyroscope Configuration, ± 250 °/s, 131 LSB/°/s */
  MPU6050_Write_bits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS_250);
  /* Accelerometer Configuration, ± 2g, 16384 LSB/g */
  MPU6050_Write_bits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, MPU6050_ACCEL_FS_2);
}

void read_MPU6050_data(void){
  Ac_X0 = (MPU6050_Read(MPU6050_RA_ACCEL_XOUT_H)<<8) | MPU6050_Read(MPU6050_RA_ACCEL_XOUT_L);
  Ac_Y0 = (MPU6050_Read(MPU6050_RA_ACCEL_YOUT_H)<<8) | MPU6050_Read(MPU6050_RA_ACCEL_YOUT_L);
  Ac_Z0 = (MPU6050_Read(MPU6050_RA_ACCEL_ZOUT_H)<<8) | MPU6050_Read(MPU6050_RA_ACCEL_ZOUT_L);
  Gy_X0 = (MPU6050_Read(MPU6050_RA_GYRO_XOUT_H)<<8) | MPU6050_Read(MPU6050_RA_GYRO_XOUT_L);
  Gy_Y0 = (MPU6050_Read(MPU6050_RA_GYRO_YOUT_H)<<8) | MPU6050_Read(MPU6050_RA_GYRO_YOUT_L);
  Gy_Z0 = (MPU6050_Read(MPU6050_RA_GYRO_ZOUT_H)<<8) | MPU6050_Read(MPU6050_RA_GYRO_ZOUT_L);
}

int _write(int32_t file, uint8_t *ptr, int32_t len){
  HAL_UART_Transmit(&huart2, ptr, len, 10);
  return len;
}

void calc_degree(){
	static uint16_t cnt=1;
	Ac_X1 = (float)Ac_X0 / 16384.0;
	Ac_Y1 = (float)Ac_Y0 / 16384.0;
	Ac_Z1 = (float)Ac_Z0 / 16384.0;
	Gy_X1 = (float)Gy_X0 / 131.0;
	Gy_Y1 = (float)Gy_Y0 / 131.0;
	Gy_Z1 = (float)Gy_Z0 / 131.0;

	Ac_X2 = Ac_X2 * ((float)cnt-1) / (float)cnt + Ac_X1 / (float)cnt;
	Ac_Y2 = Ac_Y2 * ((float)cnt-1) / (float)cnt + Ac_Y1 / (float)cnt;
	Ac_Z2 = Ac_Z2 * ((float)cnt-1) / (float)cnt + Ac_Z1 / (float)cnt;

	Deg_X = atan(Ac_Y1 / sqrt(pow(Ac_X1, 2) + pow(Ac_Z1, 2))) * 180.0 / M_PI;
	Deg_Y = atan(Ac_X1 / sqrt(pow(Ac_Y1, 2) + pow(Ac_Z1, 2))) * 180.0 / M_PI;
	Deg_Z = atan(sqrt(pow(Ac_X1, 2) + pow(Ac_Y1, 2)) / Ac_Z1) * 180.0 / M_PI;

	/* Complementary filter */
	Deg_XC = 0.98 * (Deg_XC + Gy_X1 * 0.005) + 0.02 * Deg_X;
	Deg_YC = 0.98 * (Deg_YC + Gy_Y1 * 0.005) + 0.02 * Deg_Y;
	Deg_ZC = 0.98 * Deg_ZC + 0.02 * Deg_Z;
	if(++cnt>20) cnt=20;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM3){
		time3_1msCnt++;
		if(!(time3_1msCnt%100)){
			time3_100msFlag = 1;
		}
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
