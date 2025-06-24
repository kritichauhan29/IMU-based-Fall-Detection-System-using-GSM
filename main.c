/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include"stdio.h"
#include"math.h"
#include"string.h"
#include"icm20948.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
float thetaHat_acc_rad;
float phiHat_acc_rad;
int16_t P ;
int16_t Q;
char tx_buffer[100];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char response[300];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
int _write(int fd,char*ptr,int len)
	  	{
	  		HAL_UART_Transmit(&huart1,(uint8_t*)ptr,len,HAL_MAX_DELAY);
	  		return len;
	  	}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void cs_high();
static void cs_low();
void userbank_select(uint8_t ub);
void icm20948_read(userbank ub,uint8_t reg,uint8_t *data);
void icm20948_write(userbank ub,uint8_t reg,uint8_t data);
void icm20948_init();
void icm20948_accel_data(icm_20948_data * data);
void icm20948_gyro_data(icm_20948_data * data);
void fall_detection();
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	icm_20948_data accel_data;
	icm_20948_data gyro_data;
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  icm20948_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  icm20948_accel_data(&accel_data);
	  icm20948_gyro_data(&gyro_data);
	  fall_detection();

	 //		 	  HAL_Delay(1000);
//	 		 		if(Q>60||Q<-60||P>60||P<-60)
//	 		 			{
//	 		 			fall_detection();
//	 		 			}
//	 		 		else{
//	 		 			break;
//	 		 		}

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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void cs_high()
{
	HAL_GPIO_WritePin(ICM20948_SPI_CS_PIN_PORT, ICM20948_SPI_CS_PIN_NUMBER, 1);
}

//Function to RESET the GPIO_OUT pin connect to CS pin of icm20948
static void cs_low()
{
	HAL_GPIO_WritePin(ICM20948_SPI_CS_PIN_PORT, ICM20948_SPI_CS_PIN_NUMBER, 0);
}

//Function to select the user bank to access specify icm20948 register
void userbank_select(uint8_t ub)
{
	cs_low();
	uint8_t reg = 0x7f;//USER_BANK_SELECT_REGISTER ADDRESS
	uint8_t user_bank = ub;//SELECTING USER BAK 0
	HAL_SPI_Transmit(ICM20948_SPI, &reg, 1, 100);
	HAL_SPI_Transmit(ICM20948_SPI, &user_bank, 1, 100);
	cs_high();
}

//Function to read specific icm20948 register values and print using USART on serial monitor
void icm20948_read(userbank ub,uint8_t reg, uint8_t * data)
{
	userbank_select(ub);
	cs_low();
	reg = READ |reg;
	uint8_t reg_val;
	HAL_SPI_Transmit(ICM20948_SPI, &reg, 1, 100);
	HAL_SPI_Receive(ICM20948_SPI, &reg_val, 1, 100);
	*data = reg_val;
	cs_high();
	//printf("Address:0x%x Register value: 0x%x \n\r",reg,reg_val);
}


void icm20948_write(userbank ub,uint8_t reg,uint8_t reg_val)
{
	userbank_select(ub);
	cs_low();
	reg = WRITE |reg;
	HAL_SPI_Transmit(ICM20948_SPI, &reg, 1, 100);
	HAL_SPI_Transmit(ICM20948_SPI, &reg_val, 1, 100);
	cs_high();
//	printf("Address:0x%x Register value: 0x%x \n\r\r",reg,reg_val);
	HAL_Delay(100);
}
void icm20948_init()
{
	//uint8_t temp_data;
	icm20948_write(ub_0,B0_PWR_MGMT_1,0xc1);
	icm20948_write(ub_0,B0_PWR_MGMT_1,0x01);
	icm20948_write(ub_0,B0_LP_CONFIG,0x30);
	icm20948_write(ub_0,B0_PWR_MGMT_2,0x00);
	icm20948_write(ub_2,B2_ODR_ALIGN_EN,0x01);

	icm20948_write(ub_2,B2_GYRO_SMPLRT_DIV,0x00);

	icm20948_write(ub_2,B2_GYRO_CONFIG_1,_250dps|0x01);
	icm20948_write(ub_2,B2_GYRO_CONFIG_2,0x00);
	icm20948_write(ub_2,B2_ACCEL_SMPLRT_DIV_1,0x00);
	icm20948_write(ub_2,B2_ACCEL_SMPLRT_DIV_2,0x00);

	icm20948_write(ub_2,B2_ACCEL_CONFIG_1,_2g|0x01);

	//icm20948_read(ub_0,B0_USER_CTRL,&temp_data);
	//temp_data |=0x10;
	icm20948_write(ub_0,B0_USER_CTRL,0xd0);
	//icm20948_read(ub_0,B0_USER_CTRL,&temp_data);
	//userbank_select(ub_0);

}

void icm20948_accel_data(icm_20948_data * data)  //function to read data from the accelerometer

{
	uint8_t data_accel_rx[6];//define an array to store high and low bytes from accelerometer
	icm20948_read(ub_0,B0_ACCEL_XOUT_H,&data_accel_rx[0]); //Read high byte of accelerometer from X axis
	icm20948_read(ub_0,B0_ACCEL_YOUT_H,&data_accel_rx[2]); //Read high byte of accelerometer from Y axis
	icm20948_read(ub_0,B0_ACCEL_ZOUT_H,&data_accel_rx[4]); //Read high byte of accelerometer from Z axis
	icm20948_read(ub_0,B0_ACCEL_XOUT_L,&data_accel_rx[1]); //Read Low byte of accelerometer from X axis
	icm20948_read(ub_0,B0_ACCEL_YOUT_L,&data_accel_rx[3]); //Read Low byte of accelerometer from Y axis
	icm20948_read(ub_0,B0_ACCEL_ZOUT_L,&data_accel_rx[5]); //Read Low byte of accelerometer from Z axis
	data->x_accel = (int16_t)((data_accel_rx[0]<<8)|(data_accel_rx[1]));//combine the 2 byte data acquired from accelerometer at X axis
	data->y_accel = (int16_t)((data_accel_rx[2]<<8)|(data_accel_rx[3]));//combine the 2 byte data acquired from accelerometer at Y axis
	data->z_accel = (int16_t)((data_accel_rx[4]<<8)|(data_accel_rx[5]));//combine the 2 byte data acquired from accelerometer at Z axis

	//convert the RAW DATA value to m/s^2 in terms of g
	data->x_accel = (data->x_accel/65536)*4.0*9.8;
	data->y_accel = (data->y_accel/65536)*4.0*9.8;
	data->z_accel = (data->z_accel/65536)*4.0*9.8;

	//print acceleration in 3-axis
	//printf("Accelerometer : %.2f  %.2f 	%.2f \n\r",data->x_accel,data->y_accel,data->z_accel);

	//estimate angles using accelerometer measurements
	 phiHat_acc_rad = atanf(data->y_accel / data->z_accel)*(180/3.14); //Roll angle
	 thetaHat_acc_rad = asinf(data->x_accel)*(180/3.14); //Pitch angle
//	 if(Pa==90)
//	 {
//		 thetaHat_acc_rad=90;
//	 }
	//print the values of roll and pitch angles via USART3
//	printf("Linear Accelerations : %.2f  %.2f	%.2f\n\r",data->x_accel,data->y_accel,data->z_accel);

}
void fall_detection(){

	char arr[][60]={"AT\r\n","AT+CPIN?\r\n","AT+CMGF=1\r\n","AT+CSMP=17,167,0,0\r\n","AT+CMGS=\"+918171612523\"\r\n","HELP !SOS! Wheelchair Fallen\x1A"};

	if(Q>60||Q<-60||P>60||P<-60)
	{
		for(int i=0; i<6;i++)
		   {
			  HAL_UART_Transmit(&huart1,(uint8_t*)arr[i],strlen(arr[i]),HAL_MAX_DELAY);
			  HAL_UART_Receive(&huart1,(uint8_t*)response,30,500);
			  HAL_Delay(500);
			  HAL_UART_Transmit(&huart2,(uint8_t*)response,strlen(response),HAL_MAX_DELAY);
//			  HAL_Delay(3000);
		   }
	}
}
void icm20948_gyro_data(icm_20948_data * data)

{
//	char arr[][60]={"AT\r\n","AT+CPIN?\r\n","AT+CSQ\r\n","AT+CMGF=1\r\n","AT+CSMP=17,167,0,0\r\n","AT+CMGS=\"+918171612523\"\r\n","GSM_IMU Test1\x1A"};

	uint8_t data_gyro_rx[6];
	icm20948_read(ub_0,B0_GYRO_XOUT_H,&data_gyro_rx[0]);
	icm20948_read(ub_0,B0_GYRO_YOUT_H,&data_gyro_rx[2]);
	icm20948_read(ub_0,B0_GYRO_ZOUT_H,&data_gyro_rx[4]);
	icm20948_read(ub_0,B0_GYRO_XOUT_L,&data_gyro_rx[1]);
	icm20948_read(ub_0,B0_GYRO_YOUT_L,&data_gyro_rx[3]);
	icm20948_read(ub_0,B0_GYRO_ZOUT_L,&data_gyro_rx[5]);
	data->x_gyro = (int16_t)((data_gyro_rx[0]<<8)|(data_gyro_rx[1]));
	data->y_gyro = (int16_t)((data_gyro_rx[2]<<8)|(data_gyro_rx[3]));
	data->z_gyro = (int16_t)((data_gyro_rx[4]<<8)|(data_gyro_rx[5]));
	data->x_gyro = ((data->x_gyro)/131)*0.01744444f;
	data->y_gyro = ((data->y_gyro)/131)*0.01744444f;
	data->z_gyro = ((data->z_gyro)/131)*0.01744444f;

	//transform body rates to euler rates
	float phiHat_rad = 0.0f;
	float thetaHat_rad = 0.0f;
//	float a=0.0f,b=0.0f;
	float phiDot_rps = data->x_gyro + tanf(thetaHat_rad) * (sinf(phiHat_rad) * data->y_gyro + cosf(phiHat_rad) * data->z_gyro);
	float thetaDot_rps = cosf(phiHat_rad) * data->y_gyro - sinf(phiHat_rad) * data->z_gyro;

	//combine accelerometer estimates with integral of gyro readings

	phiHat_rad = COMP_FILT_ALPHA * phiHat_acc_rad + (1.0f - COMP_FILT_ALPHA) * (phiHat_rad + (SAMPLE_TIME_MS_USB / 1000.0f) * phiDot_rps);

	thetaHat_rad = COMP_FILT_ALPHA * thetaHat_acc_rad + (1.0f - COMP_FILT_ALPHA) * (thetaHat_rad + (SAMPLE_TIME_MS_USB / 1000.0f) * thetaDot_rps);
//	a=a+phiHat_rad;
//	b=b+thetaHat_rad;

	 P = phiHat_rad;
	 Q = thetaHat_rad;

//	printf("Angular Velocities+ : %.2f  %.2f	%.2f \r\n",data->x_gyro,data->y_gyro,data->z_gyro);
	if(Q>86){
		Q=90;
	}

//}
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
