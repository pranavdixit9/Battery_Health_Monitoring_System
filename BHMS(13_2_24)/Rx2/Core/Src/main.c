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
#include<stdio.h>
#include "math.h"
#include<string.h>
#include<stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ADC_RESOLUTION 4095.0 // 12-bit ADC
#define VOLTAGE_HISTORY_SIZE 10

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t rpm;
float capacity = 2200;
float Ro = 219; // Internal resistance of the cell at the time of manufacturing
float SOC = 0;
float SOH = 0;
float Q = 0; // Depth of Discharge (DOD)
float voltage;
float current;
float deltaQ;
float VREF = 11.1;
float I_MAX = 66.0;
float DELTA_T = 100.0;
uint8_t adcValueCurrent = 0;
uint8_t adcValueVoltage = 1;
float voltageHistory[VOLTAGE_HISTORY_SIZE];
uint8_t voltageHistoryIndex = 0;
uint8_t counter=0;

//char receivedRPM;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void SOC_Calc();
void SOH_Calc();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[5];

char UARTsend[21];
char receivedRPM[3];




void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET); // blue
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
	adcValueCurrent= RxData[1];
	adcValueVoltage= RxData[0];

	counter++;
}

void UART_ReceiveString(uint8_t* buffer, uint16_t maxLength) {
    uint16_t index = 0;
    uint8_t receivedChar = 0; // Initialize receivedChar to zero

    //memset(buffer, 0, maxLength);
    // Loop until carriage return character '\r' is received or maximum length reached
    do {
        HAL_UART_Receive(&huart1, (uint8_t *)&receivedChar, 1, HAL_MAX_DELAY); // Polling receive
        if (receivedChar == '\r' || receivedChar == '\n') {
                    continue;
                }
        buffer[index++] = receivedChar;

        if (index >= maxLength) {
            break;
        }
    } while (receivedChar != '\r' && index < maxLength);

    // Null-terminate the string
//    buffer[index] = '\0';
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//uint8_t receivedRPM[4];
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
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
//  HAL_Delay(100);
//  HAL_Delay(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_UART_Receive(&huart1, &rpm, sizeof(rpm), 10);//HAL_MAX_DELAY);

	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); //

	  SOC_Calc();
	  SOH_Calc();
	  counter++;
	  RxData[3]= SOC;
	  RxData[4]= SOH;
	  memset(UARTsend, 0, sizeof(UARTsend));
//	  		//UART=CURRENT,TEMPERATURE,SOC,SOH,RPM
	  snprintf(UARTsend, sizeof(UARTsend), "%03d,%03d,%03d,%03d,%03d,#", RxData[1], RxData[2], RxData[3], RxData[4], rpm);


			  // Add the delimiter at the end (modify "#" as needed)
	  strcat(UARTsend, "#");
	  HAL_UART_Transmit(&huart2, (uint8_t *)UARTsend, sizeof(UARTsend), HAL_MAX_DELAY);

	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

	  HAL_Delay(500);


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hcan1.Init.Prescaler = 18;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
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
  CAN_FilterTypeDef canFilterConfig;
  canFilterConfig.FilterActivation=CAN_FILTER_ENABLE;
  canFilterConfig.SlaveStartFilterBank=14;
  canFilterConfig.FilterBank=2;
  canFilterConfig.FilterFIFOAssignment=CAN_RX_FIFO0;
  canFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
  canFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
  canFilterConfig.FilterMaskIdLow=0x0000;
  canFilterConfig.FilterMaskIdHigh=0xFF00;
  canFilterConfig.FilterIdLow=0x0000;
  canFilterConfig.FilterIdHigh=0x1500;
  HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig);
  /* USER CODE END CAN1_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void SOC_Calc() {

    // Calculate SOC
	float flt_adcValueCurrent=adcValueCurrent/1000;
    voltage = (adcValueVoltage / ADC_RESOLUTION) * VREF; // VREF is the reference voltage of the ADC
    current = (flt_adcValueCurrent / ADC_RESOLUTION) * I_MAX; // I_MAX is the maximum current value
    deltaQ = current * DELTA_T; // DELTA_T is the time interval between each calculation
    Q += deltaQ;
    SOC = (1 - Q / capacity) * 100;

}

void SOH_Calc() {

	float currentVoltage = (adcValueVoltage / ADC_RESOLUTION) * VREF;

	    // Update voltage history array
	    voltageHistory[voltageHistoryIndex] = currentVoltage;
	    voltageHistoryIndex = (voltageHistoryIndex + 1) % VOLTAGE_HISTORY_SIZE;

	    // Calculate average voltage over the history period
	    float sumVoltage = 0;
	    for (uint8_t i = 0; i < VOLTAGE_HISTORY_SIZE; ++i) {
	        sumVoltage += voltageHistory[i];
	    }
	    float averageVoltage = sumVoltage / VOLTAGE_HISTORY_SIZE;

	    // Calculate the percentage difference between current voltage and average voltage
	    float voltageDifferencePercentage = ((currentVoltage - averageVoltage) / averageVoltage) * 100.0;

	    // Adjust the following threshold based on your specific battery characteristics
	    float degradationThreshold = 11.1;/* Specify the threshold for significant degradation */;

	    // Calculate SOH within a valid range (0 to 100)
	    if (voltageDifferencePercentage > 0) {
	        SOH = 100.0 - fmin(voltageDifferencePercentage, degradationThreshold);
	    } else {
	        SOH = 100.0;
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
