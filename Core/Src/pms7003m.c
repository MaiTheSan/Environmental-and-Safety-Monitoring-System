#include "pms7003m.h"
#include <string.h>

#define PMS_FRAME_LENGTH 32
#define PMS_HEADER_HIGH  0x42
#define PMS_HEADER_LOW   0x4D

void PMS7003M_Init(PMS7003M_Handle *sensor, UART_HandleTypeDef *huart) {
    sensor->huart = huart;
    sensor->pm1_0 = 0;
    sensor->pm2_5 = 0;
    sensor->pm10 = 0;
    memset(sensor->raw_data, 0, sizeof(sensor->raw_data));
}

static uint16_t pms_calculate_checksum(uint8_t *data, int len) {
    uint16_t sum = 0;
    for (int i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

bool PMS7003M_Read(PMS7003M_Handle *sensor) {
    // Đọc 32 byte (1 frame chuẩn)
    if (HAL_UART_Receive(sensor->huart, sensor->raw_data, PMS_FRAME_LENGTH, 1000) != HAL_OK) {
        return false;
    }

    // Kiểm tra header
    if (sensor->raw_data[0] != PMS_HEADER_HIGH || sensor->raw_data[1] != PMS_HEADER_LOW) {
        return false;
    }

    // Tính checksum
    uint16_t received_checksum = (sensor->raw_data[30] << 8) | sensor->raw_data[31];
    uint16_t calculated_checksum = pms_calculate_checksum(sensor->raw_data, 30);
    if (received_checksum != calculated_checksum) {
        return false;
    }

    // Trích xuất dữ liệu
    sensor->pm1_0 = (sensor->raw_data[10] << 8) | sensor->raw_data[11];
    sensor->pm2_5 = (sensor->raw_data[12] << 8) | sensor->raw_data[13];
    sensor->pm10  = (sensor->raw_data[14] << 8) | sensor->raw_data[15];

    return true;
}
