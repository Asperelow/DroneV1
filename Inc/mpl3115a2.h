

#ifndef MPL3115A2_H
#define MPL3115A2_H

#include "stm32f4xx_hal.h"  // Include the STM32 HAL header


// MPL3115A2 I2C address
#define MPL3115A2_I2C_ADDR 0x60

// Register addresses
#define MPL3115A2_WHO_AM_I      0x0C
#define MPL3115A2_CTRL_REG1     0x26
#define MPL3115A2_DATA_CONFIG   0x13
#define MPL3115A2_PRESSURE_MSB  0x01
#define MPL3115A2_PRESSURE_CSB  0x02
#define MPL3115A2_PRESSURE_LSB  0x03
#define MPL3115A2_TEMP_MSB      0x04
#define MPL3115A2_TEMP_LSB      0x05

// ... add other necessary register definitions ...
typedef struct {
    float pressure; // Pressure data
    float temperature; // Temperature data
} MPL3115A2_DataTypeDef;

void MPL3115A2_Init(I2C_HandleTypeDef *hi2c);
MPL3115A2_DataTypeDef MPL3115A2_ReadData(I2C_HandleTypeDef *hi2c);

#endif /* MPL3115A2_H */
