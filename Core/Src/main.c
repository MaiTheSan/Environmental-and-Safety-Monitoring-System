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
#include "ws2812b.h"  // Add this new include
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    STATE_NORMAL = 0,
    STATE_INTERRUPT,
	STATE_HOUSEFIRE
} SystemState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_PIN_4

#define PMS_FRAME_LENGTH 32  // PMS7003 truyền mỗi lần 32 byte
#define PMS_HEADER_HIGH   0x42
#define PMS_HEADER_LOW    0x4D

// WS2812B definitions
#define WS2812B_PORT GPIOA
#define WS2812B_PIN GPIO_PIN_6
#define WS2812B_NUM_LEDS 12

#define ADC_TIMEOUT      10
#define ADC_FIRE_THRESH  4000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

volatile SystemState currentState = STATE_NORMAL;


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t temp = 0, hum = 0;
char buffer[32];

uint8_t pms_rx_byte;
uint8_t pms_rx_buffer[PMS_FRAME_LENGTH];
uint8_t pms_index = 0;
uint32_t adc_val;
volatile bool pms_ready_flag = false;

A7680C_Handle sim4g;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
    uint16_t timeout;

    // Start Signal
    DHT11_Start();

    // Wait for DHT11 response (should go LOW)
    timeout = 0;
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) {
        delay_us(1);
        timeout++;
        if (timeout > 100) return 1; // Timeout waiting for response
    }

    // Wait for DHT11 to pull up (80us LOW expected)
    timeout = 0;
    while (!HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) {
        delay_us(1);
        timeout++;
        if (timeout > 100) return 2; // Timeout waiting for pull up
    }

    // Wait for DHT11 to pull down (80us HIGH expected)
    timeout = 0;
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) {
        delay_us(1);
        timeout++;
        if (timeout > 100) return 3; // Timeout waiting for pull down
    }

    // Read 40 bits with better timeout handling
    for (j = 0; j < 5; j++) {
        for (i = 0; i < 8; i++) {
            // Wait for rising edge (signal start)
            timeout = 0;
            while (!HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) {
                delay_us(1);
                timeout++;
                if (timeout > 60) return 4; // Timeout during bit start
            }

            // Wait ~40us and check pin state
            delay_us(40);

            if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))
                data[j] |= (1 << (7 - i));

            // Wait for falling edge (end of bit)
            timeout = 0;
            while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) {
                delay_us(1);
                timeout++;
                if (timeout > 70) return 5; // Timeout during bit end
            }
        }
    }

    // Verify checksum
    if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) return 6;

    *humidity = data[0];
    *temperature = data[2];
    return 0;
}

/*********************************** PMS7003 FUNCTIONS ********************************************/

uint16_t extract_uint16(uint8_t *data, uint8_t index) {
    return ((uint16_t)data[index] << 8) | data[index + 1];
}

void parse_and_display_pms_data(uint8_t *data) {
    uint16_t pm1_0 = extract_uint16(data, 4);
    uint16_t pm2_5 = extract_uint16(data, 6);
    uint16_t pm10  = extract_uint16(data, 8);

    // Đánh dấu đã nhận được frame hợp lệ đầu tiên
    pms_ready_flag = true;

    // Chỉ cập nhật 3 dòng dữ liệu, tránh làm nhấp nháy toàn màn hình
    char line1[22], line2[22], line3[22];
    snprintf(line1, sizeof(line1), "PM1.0 : %4u ug/m3", pm1_0);
    snprintf(line2, sizeof(line2), "PM2.5 : %4u ug/m3", pm2_5);
    snprintf(line3, sizeof(line3), "PM10  : %4u ug/m3", pm10);

    // Xóa vùng 3 dòng (nếu cần) bằng cách viết khoảng trắng
    ssd1306_FillRectangle(0, 0, 127, 33, Black);


    // Viết lại dữ liệu mới
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString(line1, Font_7x10, White);
    ssd1306_SetCursor(0, 12);
    ssd1306_WriteString(line2, Font_7x10, White);
    ssd1306_SetCursor(0, 24);
    ssd1306_WriteString(line3, Font_7x10, White);

    ssd1306_UpdateScreen();
}
/**
 * @brief  Kiểm tra header + checksum của frame PMS7003, đồng thời align lại buffer nếu cần.
 * @param  buf: con trỏ đến buffer kích thước PMS_FRAME_LENGTH
 * @param  len: độ dài frame (32)
 * @retval true nếu frame hợp lệ, false nếu phải tìm frame tiếp theo
 */
bool check_and_align_pms_frame(uint8_t *buf, uint8_t len) {
    // 1) Kiểm tra header
    if (buf[0] != PMS_HEADER_HIGH || buf[1] != PMS_HEADER_LOW) {
        // Tìm vị trí header mới trong buffer
        for (int i = 1; i < len - 1; i++) {
            if (buf[i] == PMS_HEADER_HIGH && buf[i+1] == PMS_HEADER_LOW) {
                uint8_t remain = len - i;
                memmove(buf, buf + i, remain);
                pms_index = remain;
                return false;
            }
        }
        // không tìm được header, reset index
        pms_index = 0;
        return false;
    }

    // 2) Tính checksum
    uint16_t calculated = 0;
    for (int i = 0; i < len - 2; i++) {
        calculated += buf[i];
    }
    uint16_t received = extract_uint16(buf, len - 2);
    if (calculated != received) {
        // checksum sai -> tìm header mới
        for (int i = 1; i < len - 1; i++) {
            if (buf[i] == PMS_HEADER_HIGH && buf[i+1] == PMS_HEADER_LOW) {
                uint8_t remain = len - i;
                memmove(buf, buf + i, remain);
                pms_index = remain;
                return false;
            }
        }
        pms_index = 0;
        return false;
    }

    // Frame hợp lệ
    return true;
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        pms_rx_buffer[pms_index++] = pms_rx_byte;

        if (pms_index >= PMS_FRAME_LENGTH) {
            if (check_and_align_pms_frame(pms_rx_buffer, PMS_FRAME_LENGTH)) {
                parse_and_display_pms_data(pms_rx_buffer);
                pms_index = 0;
            }
            // nếu không hợp lệ, pms_index đã được điều chỉnh bên trong hàm
        }

        HAL_UART_Receive_IT(&huart1, &pms_rx_byte, 1);
    }
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  WS2812B_Init(WS2812B_PORT, WS2812B_PIN);

  // Initial test pattern - light up all LEDs in rainbow pattern
  WS2812B_Rainbow(WS2812B_PORT, WS2812B_PIN, WS2812B_NUM_LEDS);
  HAL_Delay(1000);

  // Set all LEDs to red initially
  WS2812B_SetAllLEDs(WS2812B_PORT, WS2812B_PIN, RED, WS2812B_NUM_LEDS);

  // Set tất cả là màu đỏ
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


  HAL_TIM_Base_Start(&htim1); // Bắt đầu Timer cho delay_us
  DHT11_Init();
  A7680C_Init(&sim4g, &huart2);

  if (DHT_Read(&temp, &hum) == 0){
	  ssd1306_SetCursor(0, 18);
	  ssd1306_WriteString("DHT11           OK", Font_7x10, White);
	  ssd1306_UpdateScreen();
      // Update LED status - first 3 LEDs green
      RGB_Color ledStatus[WS2812B_NUM_LEDS] = {0};
      for (int i = 0; i < 3; i++) ledStatus[i] = GREEN;
      for (int i = 3; i < WS2812B_NUM_LEDS; i++) ledStatus[i] = RED;
      WS2812B_SetLED(WS2812B_PORT, WS2812B_PIN, ledStatus, WS2812B_NUM_LEDS);
	  HAL_Delay(500);
  }


  if (A7680C_SendCommand(&sim4g, "AT", "OK", 1000)) {
	  ssd1306_SetCursor(0, 28);
	  ssd1306_WriteString("A7680c          OK", Font_7x10, White);
	  ssd1306_UpdateScreen();
      // Update LED status - next 3 LEDs green
      RGB_Color ledStatus[WS2812B_NUM_LEDS] = {0};
      for (int i = 0; i < 6; i++) ledStatus[i] = GREEN;
      for (int i = 6; i < WS2812B_NUM_LEDS; i++) ledStatus[i] = RED;
      WS2812B_SetLED(WS2812B_PORT, WS2812B_PIN, ledStatus, WS2812B_NUM_LEDS);
	  HAL_Delay(500);
  }
  if (A7680C_CheckSIMReady(&sim4g)) {
      // SIM is ready, proceed
	  ssd1306_SetCursor(0, 38);
	  ssd1306_WriteString("SIM card        OK", Font_7x10, White);
	  ssd1306_UpdateScreen();
      // Update LED status - next 3 LEDs green
      RGB_Color ledStatus[WS2812B_NUM_LEDS] = {0};
      for (int i = 0; i < 9; i++) ledStatus[i] = GREEN;
      for (int i = 9; i < WS2812B_NUM_LEDS; i++) ledStatus[i] = RED;
      WS2812B_SetLED(WS2812B_PORT, WS2812B_PIN, ledStatus, WS2812B_NUM_LEDS);
	  HAL_Delay(500);
  }
  if (A7680C_SendSMS(&sim4g, "0848177013", "SMS Check OK")) {
      // success
  }


  if (HAL_UART_Receive(&huart1, &pms_rx_byte, 1, 10) == HAL_OK) {
      // Có dữ liệu nhận được -> Hiển thị thông báo OK
      ssd1306_SetCursor(0, 48);
      ssd1306_WriteString("PMS7003         OK", Font_7x10, White);
      ssd1306_UpdateScreen();
	  //set 3 đèn cuối màu xanh
      WS2812B_SetAllLEDs(WS2812B_PORT, WS2812B_PIN, GREEN, WS2812B_NUM_LEDS);
      HAL_UART_Receive_IT(&huart1, &pms_rx_byte, 1);
      HAL_Delay(500);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      // ADC-based alarm check
      HAL_ADC_Start(&hadc1);
      if (HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT) == HAL_OK) {
          adc_val = HAL_ADC_GetValue(&hadc1);
          if (adc_val > ADC_FIRE_THRESH) {
              currentState = STATE_HOUSEFIRE;
          }
      }
      HAL_ADC_Stop(&hadc1);

      switch (currentState) {
          case STATE_NORMAL:
              // Normal periodic sensor update
	    ssd1306_FillRectangle(0, 34, 127, 63, Black);

	    // Try up to 3 times to get a valid reading
	    uint8_t status = 255;
	    uint8_t retry_count = 0;

	    while (retry_count < 3 && status != 0) {
	        status = DHT_Read(&temp, &hum);
	        if (status != 0) {
	            HAL_Delay(500); // Wait before retry
	        }
	        retry_count++;
	    }

	    if (status == 0) {
	        sprintf(buffer, "Temp: %d C", temp);
	        ssd1306_SetCursor(0, 36);
	        ssd1306_WriteString(buffer, Font_7x10, White);
	        sprintf(buffer, "Humi: %d %%", hum);
	        ssd1306_SetCursor(0, 48);
	        ssd1306_WriteString(buffer, Font_7x10, White);
	        // Hiển thị giá trị ADC
	        sprintf(buffer, "ADC : %lu", adc_val);
	        ssd1306_SetCursor(0, 58);
	        ssd1306_WriteString(buffer, Font_6x8, White);
	    }
	    else {
	        ssd1306_SetCursor(0, 36);
	        sprintf(buffer, "DHT11 Error: %d", status);
	        ssd1306_WriteString(buffer, Font_7x10, White);
	    }
	    ssd1306_UpdateScreen();
        RGB_Color statusColor;
        if (temp < 25) statusColor = GREEN;       // Cool
        else if (temp < 30) statusColor = YELLOW; // Moderate
        else statusColor = RED;                   // Hot
        WS2812B_SetAllLEDs(WS2812B_PORT, WS2812B_PIN, statusColor, WS2812B_NUM_LEDS);
	    HAL_Delay(1000);
        break;



    case STATE_INTERRUPT:
        // Disable UART1 receive interrupt
        HAL_UART_AbortReceive_IT(&huart1);

        // Flash LEDs to signal interrupt
        for (int i = 0; i < 10; i++) {
            RGB_Color ledPattern[WS2812B_NUM_LEDS];
            for (int i = 0; i < WS2812B_NUM_LEDS; i++) {
                ledPattern[i] = (i < WS2812B_NUM_LEDS/2) ? BLUE : RED;
            }
            WS2812B_SetLED(WS2812B_PORT, WS2812B_PIN, ledPattern, WS2812B_NUM_LEDS);
            ssd1306_Fill(Black);
            ssd1306_UpdateScreen();
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // Bật còi
            HAL_Delay(500);


            // Invert colors
            for (int i = 0; i < WS2812B_NUM_LEDS; i++) {
                ledPattern[i] = (i < WS2812B_NUM_LEDS/2) ? RED : BLUE;
            }
            WS2812B_SetLED(WS2812B_PORT, WS2812B_PIN, ledPattern, WS2812B_NUM_LEDS);
            ssd1306_Fill(White);
            ssd1306_UpdateScreen();
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // Tắt còi
            HAL_Delay(500);


        }
        A7680C_SendSMS(&sim4g, "0848177013", "Warning!!!! Earthquake!");
        // Re-enable UART1 interrupt before returning
        HAL_UART_Receive_IT(&huart1, &pms_rx_byte, 1);

        // Return to normal state
        currentState = STATE_NORMAL;
        break;

    case STATE_HOUSEFIRE:
        // Disable UART1 receive interrupt
        HAL_UART_AbortReceive_IT(&huart1);

        // Flash LEDs to signal interrupt
        for (int i = 0; i < 10; i++) {
            RGB_Color ledPattern[WS2812B_NUM_LEDS];
            for (int i = 0; i < WS2812B_NUM_LEDS; i++) {
                ledPattern[i] = (i < WS2812B_NUM_LEDS/2) ? BLUE : RED;
            }
            WS2812B_SetLED(WS2812B_PORT, WS2812B_PIN, ledPattern, WS2812B_NUM_LEDS);
            ssd1306_Fill(Black);
            ssd1306_UpdateScreen();
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // Bật còi
            HAL_Delay(500);

            // Invert colors
            for (int i = 0; i < WS2812B_NUM_LEDS; i++) {
                ledPattern[i] = (i < WS2812B_NUM_LEDS/2) ? RED : BLUE;
            }
            WS2812B_SetLED(WS2812B_PORT, WS2812B_PIN, ledPattern, WS2812B_NUM_LEDS);
            ssd1306_Fill(White);
            ssd1306_UpdateScreen();
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // Tắt còi
            HAL_Delay(500);
        }
        A7680C_SendSMS(&sim4g, "0848177013", "Warning!!!! House fire!");
        // Re-enable UART1 interrupt before returning
        HAL_UART_Receive_IT(&huart1, &pms_rx_byte, 1);

        // Return to normal state
        currentState = STATE_NORMAL;
        break;

    default:
        currentState = STATE_NORMAL;
        break;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0) {
        // Signal interrupt state
        currentState = STATE_INTERRUPT;
        //STATE_INTERRUPT
        //STATE_NORMAL
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
