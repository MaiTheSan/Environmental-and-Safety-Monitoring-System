#ifndef WS2812B_H
#define WS2812B_H

#include "main.h"
#include <stdbool.h>

// RGB Color structure
typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} RGB_Color;

// Function prototypes
void WS2812B_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void WS2812B_SetLED(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, RGB_Color* colors, uint16_t numLEDs);
void WS2812B_SetAllLEDs(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, RGB_Color color, uint16_t numLEDs);
void WS2812B_Clear(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint16_t numLEDs);
void WS2812B_Rainbow(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint16_t numLEDs);
void WS2812B_AirQualityIndicator(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint16_t pm25, uint16_t numLEDs);
void WS2812B_TestColors(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint16_t numLEDs);

// Predefined colors
extern const RGB_Color RED;
extern const RGB_Color GREEN;
extern const RGB_Color BLUE;
extern const RGB_Color WHITE;
extern const RGB_Color BLACK;
extern const RGB_Color YELLOW;
extern const RGB_Color CYAN;
extern const RGB_Color MAGENTA;
extern const RGB_Color ORANGE;
extern const RGB_Color PURPLE;

#endif // WS2812B_H
