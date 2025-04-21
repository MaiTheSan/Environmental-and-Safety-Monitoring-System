#ifndef __PMS7003M_H__
#define __PMS7003M_H__

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    UART_HandleTypeDef *huart;
    uint16_t pm1_0;
    uint16_t pm2_5;
    uint16_t pm10;
    uint8_t raw_data[32];
} PMS7003M_Handle;

void PMS7003M_Init(PMS7003M_Handle *sensor, UART_HandleTypeDef *huart);
bool PMS7003M_Read(PMS7003M_Handle *sensor);

#endif
