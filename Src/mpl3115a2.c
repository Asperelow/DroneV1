#include <mpl3115a2.h>



static void MPL3115A2_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t regAddr, uint8_t data) {
    HAL_I2C_Mem_Write(hi2c, MPL3115A2_I2C_ADDR << 1, regAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

static uint8_t MPL3115A2_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t regAddr) {
    uint8_t data;
    HAL_I2C_Mem_Read(hi2c, MPL3115A2_I2C_ADDR << 1, regAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    return data;
}

void MPL3115A2_Init(I2C_HandleTypeDef *hi2c) {
    // Set the sensor to Standby mode
    MPL3115A2_WriteReg(hi2c, MPL3115A2_CTRL_REG1, 0x00);
    MPL3115A2_WriteReg(hi2c, MPL3115A2_CTRL_REG1, 0x01);

    // Configure the sensor for polling mode (not using INT1 or INT2)
    MPL3115A2_WriteReg(hi2c, MPL3115A2_CTRL_REG1, 0xB8); 	// Example: ALT=1, OST=1, SBYB=1 (active mode)
    MPL3115A2_WriteReg(hi2c, MPL3115A2_DATA_CONFIG, 0x07);
    MPL3115A2_WriteReg(hi2c, MPL3115A2_CTRL_REG1, 0xB9);


    // Further configuration as needed (e.g., setting up over-sampling, etc.)
}

MPL3115A2_DataTypeDef MPL3115A2_ReadData(I2C_HandleTypeDef *hi2c) {
    MPL3115A2_DataTypeDef data;

    // Read pressure data
    uint8_t pressureData[3];
    HAL_I2C_Mem_Read(hi2c, MPL3115A2_I2C_ADDR << 1, MPL3115A2_PRESSURE_MSB, I2C_MEMADD_SIZE_8BIT, pressureData, 3, HAL_MAX_DELAY);
    data.pressure = (pressureData[0] << 16 | pressureData[1] << 8 | pressureData[2]) / 64.0; // Conversion to pressure

    // Read temperature data
    uint8_t tempData[2];
    HAL_I2C_Mem_Read(hi2c, MPL3115A2_I2C_ADDR << 1, MPL3115A2_TEMP_MSB, I2C_MEMADD_SIZE_8BIT, tempData, 2, HAL_MAX_DELAY);
    data.temperature = (tempData[0] << 8 | tempData[1]) / 256.0; // Conversion to temperature

    return data;
}

