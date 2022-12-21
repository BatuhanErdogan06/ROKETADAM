/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/*
 *
 *
 *
 *
 *
 *
 * CRATED BY : BATUHAN ERDOĞAN :)
 *
 *
 *
 * 14.12.2022 06.04
 *
 *
 *
 *
 *
 *
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "BME280.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
signed long temperature_raw;
signed long pressure_raw;
signed long humadity_raw;
float temperature;
float pressure;
float altitude;
float humanity;


unsigned short dig_T1;
unsigned short dig_P1;
unsigned char dig_H1;
unsigned char dig_H3;
signed char dig_H6 ;
signed short dig_H5;
signed short dig_H4;
signed short dig_H2;
signed short dig_T2;
signed short dig_T3;
signed short dig_P2;
signed short dig_P3;
signed short dig_P4;
signed short dig_P5;
signed short dig_P6;
signed short dig_P7;
signed short dig_P8;
signed short dig_P9;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t I2CRegisterRead(uint8_t dataAddr){
	uint8_t rxBuff;
	HAL_I2C_Mem_Read(&hi2c1, bme280deviceadd, dataAddr, 1, &rxBuff, 1, 1000);
	return rxBuff;
}
void I2CRegisterWrite(uint8_t dataAddr, uint8_t data){
	HAL_I2C_Mem_Write(&hi2c1, bme280deviceadd, dataAddr, 1, &data, 1, 1000);
}


void BME280Init(uint8_t ctrl_meas, uint8_t config){

	I2CRegisterWrite(CTRL_MEAS, ctrl_meas);
	I2CRegisterWrite(CONFIG, config);
	//calibrationOfBME280();

}
//calibration
void calibrationOfBME280(void){
	uint8_t theCalibData[32] = {0};
	HAL_I2C_Mem_Read(
			&hi2c1,
			bme280deviceadd,
			CALIB_DATA_P_ADDR,
			1,
			theCalibData,
			CALIB_DATA_P_SIZE,
			1000
			);
	    dig_T1 = ((theCalibData[1]  << 8) | (theCalibData[0]));
		dig_T2 = ((theCalibData[3]  << 8) | (theCalibData[2]));
		dig_T3 = ((theCalibData[5]  << 8) | (theCalibData[4]));
		dig_P1 = ((theCalibData[7]  << 8) | (theCalibData[6]));
		dig_P2 = ((theCalibData[9]  << 8) | (theCalibData[8]));
		dig_P3 = ((theCalibData[11] << 8) | (theCalibData[10]));
		dig_P4 = ((theCalibData[13] << 8) | (theCalibData[12]));
		dig_P5 = ((theCalibData[15] << 8) | (theCalibData[14]));
		dig_P6 = ((theCalibData[17] << 8) | (theCalibData[16]));
		dig_P7 = ((theCalibData[19] << 8) | (theCalibData[18]));
		dig_P8 = ((theCalibData[21] << 8) | (theCalibData[20]));
		dig_P9 = ((theCalibData[23] << 8) | (theCalibData[22]));
		dig_H1 = ( theCalibData[23]);
		dig_H2 = ((theCalibData[24] << 8) | (theCalibData[25]));
		dig_H3 = ( theCalibData[26]);
		dig_H4 = ((theCalibData[27] << 8) | (theCalibData[28]));
		dig_H5 = ((theCalibData[29] << 8) | (theCalibData[30]));
		dig_H6= ( theCalibData[31]);



}

void BME280Calculation(void){
	uint8_t status;
	uint8_t rawData[9];



	do{
		status=I2CRegisterRead(STATUS);
	} while(
			((status & 0x08) == STATUS_CONV)||
			((status & 0x01) == STATUS_COPY)
			);

	HAL_I2C_Mem_Read(
			&hi2c1,
			bme280deviceadd,
			START_RAW_DATA_POINT,
			1,
			rawData,
			RAW_DATA_LENGTH,
			1000
			);

		temperature_raw =  ((rawData[3] << 12) | (rawData[4] << 4) | (rawData[5] >> 4));
		pressure_raw    =  ((rawData[0] << 12) | (rawData[1] << 4) | (rawData[2] >> 4));
		humadity_raw    =  ((rawData[6] << 12) | (rawData[7] << 4) | (rawData[8] >> 4));



		    double var1, var2;
			var1=(((double)temperature_raw)/16384.0-((double)dig_T1)/1024.0)*((double)dig_T2);
			var2=((((double)temperature_raw)/131072.0-((double)dig_T1)/8192.0)*(((double)temperature_raw)/131072.0-((double)dig_T1)/8192.0))*((double)dig_T3);
			double t_fine = (int32_t)(var1+var2);
			volatile float T = (var1+var2)/5120.0;

			var1=((double)t_fine/2.0)-64000.0;
			var2=var1*var1*((double)dig_P6)/32768.0;
			var2=var2+var1*((double)dig_P5)*2.0;
			var2=(var2/4.0)+(((double)dig_P4)*65536.0);
			var1=(((double)dig_P3)*var1*var1/524288.0+((double)dig_P2)*var1)/524288.0;
			var1=(1.0+var1/32768.0)*((double)dig_P1);
			volatile double p = 1048576.0-(double)pressure_raw;
			p=(p-(var2/4096.0))*6250.0/var1;
			var1=((double)dig_P9)*p*p/2147483648.0;
			var2=p*((double)dig_P8)/32768.0;
			p=p+(var1+var2+((double)dig_P7))/16.0;

			temperature = T;
			pressure    = p;
			altitude=44330.0f*(1-powf(pressure/101325.0f,1.0f/5.255f));



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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  uint8_t f4 = OVERSAMPLING_T_5BIT|
		  	   OVERSAMPLING_P_5BIT|
			   NORMAL_MODE;
  uint8_t f5 = T_SB_500|
  		  	   FILTER_22|
  			   0x00;


  BME280Init(f4, f5);
  calibrationOfBME280();//çalışmazsa initin içine at bu fonkiyonu.

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  BME280Calculation();
	  HAL_Delay(200);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
