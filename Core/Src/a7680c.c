#include "a7680c.h"
#include <string.h>
#include <stdio.h>

void A7680C_Init(A7680C_Handle *modem, UART_HandleTypeDef *huart) {
    modem->huart = huart;
    memset(modem->rx_buffer, 0, A7680C_RX_BUFFER_SIZE);
}

bool A7680C_SendCommand(A7680C_Handle *modem, const char *cmd, const char *expected, uint32_t timeout) {
    char command_buffer[128];
    snprintf(command_buffer, sizeof(command_buffer), "%s\r\n", cmd);
    HAL_UART_Transmit(modem->huart, (uint8_t*)command_buffer, strlen(command_buffer), timeout);

    memset(modem->rx_buffer, 0, A7680C_RX_BUFFER_SIZE);
    HAL_UART_Receive(modem->huart, (uint8_t*)modem->rx_buffer, A7680C_RX_BUFFER_SIZE - 1, timeout);

    return (strstr(modem->rx_buffer, expected) != NULL);
}

const char* A7680C_GetLastResponse(A7680C_Handle *modem) {
    return modem->rx_buffer;
}
bool A7680C_CheckSIMReady(A7680C_Handle *modem) {
    if (A7680C_SendCommand(modem, "AT+CPIN?", "+CPIN: READY", 1000)) {
        return true; // SIM is ready
    }
    return false; // SIM not ready or error
}
bool A7680C_SendSMS(A7680C_Handle *modem, const char *phoneNumber, const char *message) {
    char cmd[64];
    uint8_t ctrlZ = 0x1A;

    // 1. Set text mode
    if (!A7680C_SendCommand(modem, "AT+CMGF=1", "OK", 1000)) {
        return false;
    }

    // 2. Prepare phone number command
    snprintf(cmd, sizeof(cmd), "AT+CMGS=\"%s\"", phoneNumber);
    HAL_UART_Transmit(modem->huart, (uint8_t*)cmd, strlen(cmd), 1000);
    HAL_UART_Transmit(modem->huart, (uint8_t*)"\r\n", 2, 1000);
    HAL_Delay(500); // wait for prompt '>'

    // 3. Send message text
    HAL_UART_Transmit(modem->huart, (uint8_t*)message, strlen(message), 1000);
    HAL_Delay(100);

    // 4. End with Ctrl+Z
    HAL_UART_Transmit(modem->huart, &ctrlZ, 1, 1000);
    HAL_Delay(5000); // Wait for SMS to send

    // 5. Optional: read response
    memset(modem->rx_buffer, 0, A7680C_RX_BUFFER_SIZE);
    HAL_UART_Receive(modem->huart, (uint8_t*)modem->rx_buffer, A7680C_RX_BUFFER_SIZE - 1, 3000);

    return (strstr(modem->rx_buffer, "+CMGS") != NULL || strstr(modem->rx_buffer, "OK") != NULL);
}
