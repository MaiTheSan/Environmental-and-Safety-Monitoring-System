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
#include "a7680c.h"
//#include "pms7003m.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DHT11_PORT GPIOB
#define DHT11_PIN GPIO_PIN_6

uint8_t pms_buffer[32];
char display_line[32];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart2;
A7680C_Handle sim4g;

//extern UART_HandleTypeDef huart1;
//PMS7003M_Handle pms;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
//uint16_t SUM, RH, TEMP;
//
//float Temperature = 0;
//float Humidity = 0;
//uint8_t Presence = 0;

/*********************************** PMS7003 FUNCTIONS ********************************************/

uint16_t extract_uint16(uint8_t *data, uint8_t high_idx) {
    return (data[high_idx] << 8) | data[high_idx + 1];
}

/**
 * Calculate and verify PMS7003 checksum
 * @param data    Buffer containing PMS7003 data (32 bytes)
 * @return        1 if checksum is valid, 0 otherwise
 */
uint8_t verify_pms7003_checksum(uint8_t *data) {
    uint16_t checksum = 0;
    // Sum bytes 0 to 29
    for (int i = 0; i < 30; i++) {
        checksum += data[i];
    }

    // Extract checksum from bytes 30-31
    uint16_t received_checksum = extract_uint16(data, 30);

    return (checksum == received_checksum) ? 1 : 0;
}

/**
 * Read data from PMS7003 sensor and display on OLED
 * @return 1 if successful read, 0 if error
 */
uint8_t read_pms7003(void) {
    HAL_StatusTypeDef status;

    // Receive data from UART with timeout
    status = HAL_UART_Receive(&huart1, pms_buffer, 32, 1000);

    if (status != HAL_OK) {
        // Handle error based on status
        if (status == HAL_TIMEOUT) {
            ssd1306_SetCursor(0, 48);
            ssd1306_WriteString("PMS: No response", Font_7x10, White);
            ssd1306_UpdateScreen();
        } else {
            ssd1306_SetCursor(0, 48);
            ssd1306_WriteString("PMS: UART Error", Font_7x10, White);
            ssd1306_UpdateScreen();
        }
        return 0;
    }

    // Check for correct header (0x42, 0x4D)
    if (pms_buffer[0] != 0x42 || pms_buffer[1] != 0x4D) {
        ssd1306_SetCursor(0, 48);
        ssd1306_WriteString("PMS: Invalid header", Font_7x10, White);
        ssd1306_UpdateScreen();
        return 0;
    }

    // Verify checksum
    if (!verify_pms7003_checksum(pms_buffer)) {
        ssd1306_SetCursor(0, 48);
        ssd1306_WriteString("PMS: Checksum error", Font_7x10, White);
        ssd1306_UpdateScreen();
        return 0;
    }

    // Extract PM values (standard particle values at indices 10, 12, 14)
    uint16_t pm1_0 = extract_uint16(pms_buffer, 10);
    uint16_t pm2_5 = extract_uint16(pms_buffer, 12);
    uint16_t pm10 = extract_uint16(pms_buffer, 14);

    // Print to OLED
    ssd1306_SetCursor(0, 48);
    snprintf(display_line, sizeof(display_line), "PM1.0   PM2.5   PM10");
    ssd1306_WriteString(display_line, Font_7x10, White);

    ssd1306_SetCursor(0, 58); // Next line
    snprintf(display_line, sizeof(display_line), "%2d-%2d-%2d", pm1_0, pm2_5, pm10);
    ssd1306_WriteString(display_line, Font_7x10, White);

    return 1;
}


/*********************************** DHT11 PRE FUNCTIONS ********************************************/

void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}
void DHT_SetPinOutput(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

void DHT_SetPinInput(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

/*********************************** DHT11 FUNCTIONS ********************************************/

void DHT11_Start (void)
{

    DHT_SetPinOutput();
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
    HAL_Delay(18);
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
    delay_us(30);
    DHT_SetPinInput();
}
void DHT11_Init(void) {
    // Set the DHT11 pin as output and pull high (idle state)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);  // Set pin high
}

uint8_t DHT_Read(uint8_t* temperature, uint8_t* humidity) {
    uint8_t data[5] = {0};
    uint8_t i, j;

    // Start Signal
    DHT11_Start();

    if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) return 1;
    delay_us(80);
    if (!HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) return 2;
    delay_us(80);

    // Read 40 bits
    for (j = 0; j < 5; j++) {
        for (i = 0; i < 8; i++) {
            while (!HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));
            delay_us(40);
            if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))
                data[j] |= (1 << (7 - i));
            while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));
        }
    }

    if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) return 3;

    *humidity = data[0];
    *temperature = data[2];
    return 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
//	float tempf = 0, humf = 0;
	uint8_t temp = 0, hum = 0;
	uint8_t status;
	char buffer[32];
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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);  // for us Delay


  A7680C_Init(&sim4g, &huart2);
//  PMS7003M_Init(&pms, &huart1);



  //set rgb to white
  DHT11_Init();

  ssd1306_Init();
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("Initialize", Font_7x10, White);
  ssd1306_UpdateScreen();
  HAL_Delay(1000);

  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("Module      Status", Font_7x10, White);
  ssd1306_SetCursor(0, 18);
  ssd1306_WriteString("DHT11          .", Font_7x10, White);
  ssd1306_SetCursor(0, 28);
  ssd1306_WriteString("A7680c         .", Font_7x10, White);
  ssd1306_SetCursor(0, 38);
  ssd1306_WriteString("SIM card       .", Font_7x10, White);
  ssd1306_SetCursor(0, 48);
  ssd1306_WriteString("PMS7003        .", Font_7x10, White);
  ssd1306_UpdateScreen();
  HAL_Delay(500);


  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("Module      Status", Font_7x10, White);
  ssd1306_SetCursor(0, 18);
  ssd1306_WriteString("DHT11          ..", Font_7x10, White);
  ssd1306_SetCursor(0, 28);
  ssd1306_WriteString("A7680c         ..", Font_7x10, White);
  ssd1306_SetCursor(0, 38);
  ssd1306_WriteString("SIM card       ..", Font_7x10, White);
  ssd1306_SetCursor(0, 48);
  ssd1306_WriteString("PMS7003        ..", Font_7x10, White);
  ssd1306_UpdateScreen();
  HAL_Delay(500);

  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("Module      Status", Font_7x10, White);
  ssd1306_SetCursor(0, 18);
  ssd1306_WriteString("DHT11          ...", Font_7x10, White);
  ssd1306_SetCursor(0, 28);
  ssd1306_WriteString("A7680c         ...", Font_7x10, White);
  ssd1306_SetCursor(0, 38);
  ssd1306_WriteString("SIM card       ...", Font_7x10, White);
  ssd1306_SetCursor(0, 48);
  ssd1306_WriteString("PMS7003        ...", Font_7x10, White);

  ssd1306_UpdateScreen();
  HAL_Delay(500);

  if (DHT_Read(&temp, &hum) == 0){
	  ssd1306_SetCursor(0, 18);
	  ssd1306_WriteString("DHT11           OK", Font_7x10, White);
	  ssd1306_UpdateScreen();
	  //set nó màu xanh
	  HAL_Delay(500);
  }
  if (A7680C_SendCommand(&sim4g, "AT", "OK", 1000)) {
	  ssd1306_SetCursor(0, 28);
	  ssd1306_WriteString("A7680c          OK", Font_7x10, White);
	  ssd1306_UpdateScreen();
	  //set nó màu xanh
	  HAL_Delay(500);
  }
  if (A7680C_CheckSIMReady(&sim4g)) {
      // SIM is ready, proceed
	  ssd1306_SetCursor(0, 38);
	  ssd1306_WriteString("SIM card        OK", Font_7x10, White);
	  ssd1306_UpdateScreen();
	  //set nó màu xanh
	  HAL_Delay(500);
  }
  if (read_pms7003()) {
	  ssd1306_SetCursor(0, 48);
	  ssd1306_WriteString("PMS7003         OK", Font_7x10, White);
	  ssd1306_UpdateScreen();
	  //set nó màu xanh
	  HAL_Delay(500);
  }
  if (A7680C_SendSMS(&sim4g, "0848177013", "Hello from STM32!")) {
      // success
  }

  HAL_Delay(500);

//  else if(DHT_Read(&temp, &hum) == 1){
//  {
//	  ssd1306_SetCursor(0, 18);
//	  ssd1306_WriteString("DHT11        ERROR", Font_7x10, White);
//	  ssd1306_UpdateScreen();
//	  HAL_Delay(1000);
//  }



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


      ssd1306_Fill(Black);
	  read_pms7003();
      status = DHT_Read(&temp, &hum);

      if (status == 0) {
          sprintf(buffer, "Temp: %d C", temp);
          ssd1306_SetCursor(0, 18);
          ssd1306_WriteString(buffer, Font_7x10, White);

          sprintf(buffer, "Humi: %d %%", hum);
          ssd1306_SetCursor(0, 36);
          ssd1306_WriteString(buffer, Font_7x10, White);
      }
      else {
          ssd1306_SetCursor(0, 18);
          ssd1306_WriteString("DHT11 Error!", Font_7x10, White);
      }
      ssd1306_UpdateScreen();
      HAL_Delay(100);




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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
