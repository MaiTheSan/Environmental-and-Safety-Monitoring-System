#ifndef __A7680C_H__
#define __A7680C_H__

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define A7680C_RX_BUFFER_SIZE 512

typedef struct {
    UART_HandleTypeDef *huart;
    char rx_buffer[A7680C_RX_BUFFER_SIZE];
} A7680C_Handle;

void A7680C_Init(A7680C_Handle *modem, UART_HandleTypeDef *huart);
bool A7680C_SendCommand(A7680C_Handle *modem, const char *cmd, const char *expected, uint32_t timeout);
const char* A7680C_GetLastResponse(A7680C_Handle *modem);
bool A7680C_CheckSIMReady(A7680C_Handle *modem);
bool A7680C_SendSMS(A7680C_Handle *modem, const char *phoneNumber, const char *message);



#endif
