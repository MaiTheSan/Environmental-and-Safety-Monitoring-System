#include "ws2812b.h"

// Predefined colors
const RGB_Color RED = {255, 0, 0};
const RGB_Color GREEN = {0, 255, 0};
const RGB_Color BLUE = {0, 0, 255};
const RGB_Color WHITE = {255, 255, 255};
const RGB_Color BLACK = {0, 0, 0};
const RGB_Color YELLOW = {255, 255, 0};
const RGB_Color CYAN = {0, 255, 255};
const RGB_Color MAGENTA = {255, 0, 255};
const RGB_Color ORANGE = {255, 165, 0};
const RGB_Color PURPLE = {128, 0, 128};

// CRITICAL: Updated timing for the WS2812B protocol
// Using assembly to guarantee precise timing for STM32 running at 84MHz
// T0H: 0.4us, T0L: 0.85us
// T1H: 0.8us, T1L: 0.45us
// RES: >50us

__STATIC_INLINE void DWT_Delay_us(volatile uint32_t microseconds)
{
  uint32_t clk_cycle_start = DWT->CYCCNT;
  microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);
  while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}

// Send a single bit with precise timing using assembly
static void WS2812B_SendBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t bit)
{
    // Use direct register access for maximum speed
    uint32_t pin_set_mask = GPIO_Pin;
    uint32_t pin_reset_mask = GPIO_Pin << 16U;

    if (bit) {
        // T1H: ~0.8us
        GPIOx->BSRR = pin_set_mask;
        __asm volatile (
            "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"
        );

        // T1L: ~0.45us
        GPIOx->BSRR = pin_reset_mask;
        __asm volatile (
            "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"
        );
    } else {
        // T0H: ~0.4us
        GPIOx->BSRR = pin_set_mask;
        __asm volatile (
            "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"
        );

        // T0L: ~0.85us
        GPIOx->BSRR = pin_reset_mask;
        __asm volatile (
            "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
        );
    }
}

static void WS2812B_SendByte(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t data)
{
    // MSB first
    for (int i = 7; i >= 0; i--) {
        WS2812B_SendBit(GPIOx, GPIO_Pin, (data >> i) & 1);
    }
}

void WS2812B_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    // Configure DWT for microsecond delay
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // Initialize the pin as output
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  // Use HIGH speed
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);

    // Initial state is LOW
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
    HAL_Delay(1); // Short delay
}

void WS2812B_SetLED(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, RGB_Color* colors, uint16_t numLEDs)
{
    uint32_t irq_state;

    // Save and disable interrupts to prevent timing disruption
    irq_state = __get_PRIMASK();
    __disable_irq();

    for (uint16_t i = 0; i < numLEDs; i++) {
        // WS2812B expects GRB order
        WS2812B_SendByte(GPIOx, GPIO_Pin, colors[i].green);
        WS2812B_SendByte(GPIOx, GPIO_Pin, colors[i].red);
        WS2812B_SendByte(GPIOx, GPIO_Pin, colors[i].blue);
    }

    // Reset pulse (>50us)
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
    DWT_Delay_us(80);  // Increased for better reliability

    // Restore interrupts
    __set_PRIMASK(irq_state);
}

void WS2812B_SetAllLEDs(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, RGB_Color color, uint16_t numLEDs)
{
    uint32_t irq_state;

    // Save and disable interrupts
    irq_state = __get_PRIMASK();
    __disable_irq();

    for (uint16_t i = 0; i < numLEDs; i++) {
        // WS2812B expects GRB order
        WS2812B_SendByte(GPIOx, GPIO_Pin, color.green);
        WS2812B_SendByte(GPIOx, GPIO_Pin, color.red);
        WS2812B_SendByte(GPIOx, GPIO_Pin, color.blue);
    }

    // Reset pulse (>50us)
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
    DWT_Delay_us(80);  // Increased for better reliability

    // Restore interrupts
    __set_PRIMASK(irq_state);
}

void WS2812B_Clear(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint16_t numLEDs)
{
    WS2812B_SetAllLEDs(GPIOx, GPIO_Pin, BLACK, numLEDs);
}

// Rainbow effect
void WS2812B_Rainbow(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint16_t numLEDs)
{
    RGB_Color colors[numLEDs];

    for (uint16_t i = 0; i < numLEDs; i++) {
        uint8_t position = 255 * i / numLEDs;

        // Convert position to RGB color using a modified HSV to RGB algorithm
        uint8_t segment = position / 85;  // Divide into 3 segments for better color distribution
        uint8_t pos_in_segment = position % 85;  // Position within the segment
        uint8_t value = pos_in_segment * 3;  // Scale to 0-255

        switch (segment) {
            case 0:  // Red to Green
                colors[i].red = 255 - value;
                colors[i].green = value;
                colors[i].blue = 0;
                break;
            case 1:  // Green to Blue
                colors[i].red = 0;
                colors[i].green = 255 - value;
                colors[i].blue = value;
                break;
            case 2:  // Blue to Red
                colors[i].red = value;
                colors[i].green = 0;
                colors[i].blue = 255 - value;
                break;
        }
    }

    WS2812B_SetLED(GPIOx, GPIO_Pin, colors, numLEDs);
}

// Air quality indicator (PM2.5)
void WS2812B_AirQualityIndicator(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint16_t pm25, uint16_t numLEDs)
{
    RGB_Color color;

    // PM2.5 level categories (based on common air quality standards)
    if (pm25 <= 12) {
        // Good - Green
        color = GREEN;
    } else if (pm25 <= 35) {
        // Moderate - Yellow
        color = YELLOW;
    } else if (pm25 <= 55) {
        // Unhealthy for Sensitive Groups - Orange
        color = ORANGE;
    } else if (pm25 <= 150) {
        // Unhealthy - Red
        color = RED;
    } else if (pm25 <= 250) {
        // Very Unhealthy - Purple
        color = PURPLE;
    } else {
        // Hazardous - Magenta
        color = MAGENTA;
    }

    // Light up LEDs based on value
    uint16_t ledsToLight = (numLEDs * pm25) / 300; // Max expected value ~300
    if (ledsToLight > numLEDs) ledsToLight = numLEDs;
    if (ledsToLight == 0 && pm25 > 0) ledsToLight = 1; // At least one LED for any non-zero reading

    RGB_Color colors[numLEDs];

    for (uint16_t i = 0; i < numLEDs; i++) {
        if (i < ledsToLight) {
            colors[i] = color;
        } else {
            colors[i] = BLACK;
        }
    }

    WS2812B_SetLED(GPIOx, GPIO_Pin, colors, numLEDs);
}

// Add a simple test function to verify color operation
void WS2812B_TestColors(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint16_t numLEDs)
{
    // Test each primary color individually
    WS2812B_SetAllLEDs(GPIOx, GPIO_Pin, RED, numLEDs);
    HAL_Delay(1000);

    WS2812B_SetAllLEDs(GPIOx, GPIO_Pin, GREEN, numLEDs);
    HAL_Delay(1000);

    WS2812B_SetAllLEDs(GPIOx, GPIO_Pin, BLUE, numLEDs);
    HAL_Delay(1000);

    // Test combined colors
    WS2812B_SetAllLEDs(GPIOx, GPIO_Pin, YELLOW, numLEDs);
    HAL_Delay(1000);

    WS2812B_SetAllLEDs(GPIOx, GPIO_Pin, CYAN, numLEDs);
    HAL_Delay(1000);

    WS2812B_SetAllLEDs(GPIOx, GPIO_Pin, MAGENTA, numLEDs);
    HAL_Delay(1000);

    WS2812B_SetAllLEDs(GPIOx, GPIO_Pin, WHITE, numLEDs);
    HAL_Delay(1000);

    WS2812B_Clear(GPIOx, GPIO_Pin, numLEDs);
}

