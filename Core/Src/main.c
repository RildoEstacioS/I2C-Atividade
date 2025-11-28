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
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PCF8591_ADDRESS (0x48 << 1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

#define RX_BUFFER_SIZE 32

uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_byte;
uint8_t rx_index = 0;

// FLAGS para comandos
volatile uint8_t comando_pendente = 0; // 0 = nenhum, 1 = AIN0, 2 = AIN1, 3 = AIN3, 4 = DAC
volatile int valor_dac = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

uint8_t PCF8591_ReadAnalog(uint8_t channel);

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
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */


  HAL_UART_Receive_IT(&huart3, &rx_byte, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  char buffer[50];
	  if (comando_pendente == 1) {
	      uint8_t v = PCF8591_ReadAnalog(0);
	      sprintf(buffer, "AIN0: %d\r\n", v);
	      HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 10);
	      comando_pendente = 0;
	  } else if (comando_pendente == 2) {
	      uint8_t v = PCF8591_ReadAnalog(1);
	      sprintf(buffer, "AIN1: %d\r\n", v);
	      HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 10);
	      comando_pendente = 0;
	  } else if (comando_pendente == 3) {
	      uint8_t v = PCF8591_ReadAnalog(3);
	      sprintf(buffer, "AIN3: %d\r\n", v);
	      HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 10);
	      comando_pendente = 0;
	  } else if (comando_pendente == 4) {
	      PCF8591_WriteDAC((uint8_t)valor_dac);
	      sprintf(buffer, "Valor do DAC: %d\r\n", valor_dac);
	      HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 10);
	      comando_pendente = 0;
	  }


//	  HAL_UART_Receive_IT(&huart3, &rx_byte, 1);

//	  char buffer[32];

//	  uint8_t valor_ain0 = PCF8591_ReadAnalog(0);
//	  sprintf(buffer, "AIN0 %d\r\n", valor_ain0);
//	  HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 10);
//	  HAL_Delay(200);
//
//	  uint8_t valor_ain1 = PCF8591_ReadAnalog(1);
//	  sprintf(buffer, "AIN1 %d\r\n", valor_ain1);
//	  HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 10);
//	  HAL_Delay(200);
//
//	  uint8_t valor_ain3 = PCF8591_ReadAnalog(3);
//	  sprintf(buffer, "AIN3 %d\r\n", valor_ain3);
//	  HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 10);
//	  HAL_Delay(200);

//
//	  char buffer[32];
//	  uint8_t valor_ain0 = PCF8591_ReadAnalog(3);
//	  sprintf(buffer, "AIN0 %d\r\n", valor_ain0);
//	  HAL_UART_Transmit(& huart3, (uint8_t*)buffer, strlen(buffer), 10);
//	  HAL_Delay(200);
//
//	  uint8_t valor_ain0 = PCF8591_ReadAnalog(0);
//	  sprintf()
//	  HAL_UART_Transmit(&huart3, &valor_ain0, 1, 10);
//	  HAL_Delay(200);
//
//	  uint8_t valor_ain1 = PCF8591_ReadAnalog(1);
//	  HAL_UART_Transmit(&huart3, &valor_ain1, 1, 10);
//	  HAL_Delay(200);
//
//	  uint8_t valor_ain3 = PCF8591_ReadAnalog(3);
//	  HAL_UART_Transmit(&huart3, &valor_ain3, 1, 10);
//	  HAL_Delay(200);

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10707DBC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//void processa_comando(char* comando) {
//  char buffer[50];
//  sprintf(buffer, "Recebido: %s\r\n", comando);
//  HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 10);
//}

void processa_comando(char* comando) {
    if(strcmp(comando, "Read_AIN0") == 0) {
        comando_pendente = 1;
    } else if(strcmp(comando, "Read_AIN1") == 0) {
        comando_pendente = 2;
    } else if(strcmp(comando, "Read_AIN3") == 0) {
        comando_pendente = 3;
    } else if(strncmp(comando, "Set_DAC_", 8) == 0) {
        valor_dac = atoi(&comando[8]);
        comando_pendente = 4;
    } else {
        char buffer[50];
        sprintf(buffer, "Comando inválido: %s\r\n", comando);
        HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 10);
    }
}

uint8_t PCF8591_ReadAnalog(uint8_t channel){
    uint8_t config_byte = 0x40 | (channel & 0x03);
    uint8_t analog_data[2];

    HAL_I2C_Master_Transmit(&hi2c1, PCF8591_ADDRESS, &config_byte, 1, 1000);
    HAL_Delay(10);

    HAL_I2C_Master_Receive(&hi2c1, PCF8591_ADDRESS, analog_data, 2, 1000);
    HAL_Delay(10);
    return analog_data[1];
}

void PCF8591_WriteDAC(uint8_t value) {
    uint8_t config_byte = 0x40; // Modo DAC
    uint8_t tx_data[2] = {config_byte, value};
    HAL_I2C_Master_Transmit(&hi2c1, PCF8591_ADDRESS, tx_data, 2, 1000);
    HAL_Delay(10);
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (rx_byte != '\n' && rx_byte != '\r' && rx_index < RX_BUFFER_SIZE - 1) {
		rx_buffer[rx_index++] = rx_byte;
	} else if (rx_index > 0) {
		rx_buffer[rx_index] = '\0';
		processa_comando((char*)rx_buffer);
		rx_index = 0;
	}
	HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
}


//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//  if (huart->Instance == USART3) {
//    // Só pra testar se chega qualquer coisa
//    HAL_UART_Transmit(&huart3, &rx_byte, 1, 10);
//    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
//  }
//}

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
